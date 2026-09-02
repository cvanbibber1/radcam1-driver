"""Selecting one camera from several, and keeping the rest switched off.

A payload with more than one camera on the same CSI/I2C fabric has a hard
constraint: **exactly one may be enabled at a time**. Two sensors holding the
same I2C address answer together and neither can be addressed; two driving the
same CSI lanes contend. So this is not a convenience layer that picks a
preferred camera, it is an interlock that guarantees the others are off.

That shapes the API. There is no "enable camera N" - only `select(n)`, which
disables every camera and then enables one. A caller cannot express the unsafe
state, and a partial failure leaves everything off rather than two cameras on:
if enabling the requested one fails, the disables have already happened.

Each camera's enable line is a GPIO named in configuration, because the mapping
is a board fact and differs per build. On the current board the two connector
enables are RP1 lines 35 (CAM0) and 48 (CAM1); a downstream board with more
cameras will use a different set, and nothing here assumes otherwise.

Sixteen is the ceiling, which is a protocol limit rather than an electrical
one: the camera id travels as a single byte field and the ground addresses
cameras by index in canned commands.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field

log = logging.getLogger(__name__)

__all__ = ["CameraSelector", "CameraEntry", "MAX_CAMERAS", "NO_CAMERA"]

#: Protocol ceiling: the camera id is one byte and the ground indexes by it.
MAX_CAMERAS = 16

#: Reported when nothing is selected, and accepted by select() to mean
#: "disable everything" - which is a legitimate low-power state, not an error.
NO_CAMERA = 0xFF

try:
    import gpiod
    from gpiod.line import Direction, Value
except ImportError:                                    # pragma: no cover
    gpiod = None


@dataclass
class CameraEntry:
    """One camera's identity and how to switch it on."""

    index: int
    gpio: int
    name: str = ""
    active_high: bool = True
    i2c_bus: int = -1
    present: bool = False
    #: True when the camera's enable line is not ours to switch because it is
    #: permanently powered - held by the kernel's camera regulator, or tied
    #: high on the board. Such a camera is always on, so the interlock treats
    #: it as the selected one by default and never claims its GPIO. Only one
    #: always-on camera may be configured, for the obvious reason.
    always_on: bool = False
    #: False when the enable line is held by something else - the kernel's
    #: camera regulator, most often. Such a camera is reported but cannot be
    #: switched, which is worth saying out loud rather than failing silently.
    controllable: bool = True
    blocked_by: str = ""

    def describe(self) -> str:
        return (f"cam{self.index}"
                + (f" ({self.name})" if self.name else "")
                + f" enable GPIO{self.gpio}"
                + (" active-low" if not self.active_high else "")
                + (f" i2c-{self.i2c_bus}" if self.i2c_bus >= 0 else "")
                + (" always-on" if self.always_on else ""))


class CameraSelector:
    """Holds every camera's enable line and guarantees at most one is on."""

    def __init__(self, entries: list[CameraEntry] | None = None,
                 chip: str = "/dev/gpiochip0",
                 consumer: str = "radcam-camsel",
                 default_camera: int = NO_CAMERA):
        self.chip = chip
        self.default_camera = default_camera
        self.consumer = consumer
        self.entries: dict[int, CameraEntry] = {}
        for entry in (entries or []):
            if 0 <= entry.index < MAX_CAMERAS:
                self.entries[entry.index] = entry
            else:
                log.warning("ignoring camera index %d: outside 0..%d",
                            entry.index, MAX_CAMERAS - 1)
        self._requests: dict[int, object] = {}
        self._active = NO_CAMERA
        self.selections = 0
        self.failures = 0

    # -- lifecycle -------------------------------------------------------

    def open(self) -> bool:
        """Claim every enable line, all starting disabled.

        Starting from all-off rather than from whatever the pins happened to be
        holding means the interlock is true from the first instant, including
        after a reset that left two lines high.
        """
        if not self.entries:
            log.info("no cameras configured; selection disabled")
            return False
        if gpiod is None:
            log.error("libgpiod not available; camera selection disabled")
            return False

        # Claim one line at a time rather than all at once. On this board the
        # two connector enables are already held by the kernel as cam0_reg and
        # cam1_reg regulators, from dtparam=camN_reg_gpio, and a single
        # all-or-nothing request would fail on those and leave every other
        # camera unmanaged too. Partial availability is the realistic case, so
        # it is the supported one - and a camera we cannot switch is named in
        # the log rather than disappearing quietly.
        self._requests = {}
        claimed, blocked = [], []
        for entry in sorted(self.entries.values(), key=lambda e: e.index):
            if entry.always_on:
                # Nothing to claim: this camera's supply is not ours. Asking
                # for the line would fail against the kernel regulator holding
                # it, and succeeding would be worse - we would then be able to
                # power down the only camera in the payload.
                entry.controllable = False
                entry.blocked_by = "always on (not switchable by design)"
                continue
            off = Value.INACTIVE if entry.active_high else Value.ACTIVE
            try:
                self._requests[entry.index] = gpiod.request_lines(
                    self.chip, consumer=f"{self.consumer}-{entry.index}",
                    config={entry.gpio: gpiod.LineSettings(
                        direction=Direction.OUTPUT, output_value=off)})
                entry.controllable = True
                claimed.append(entry)
            except Exception as exc:                   # noqa: BLE001
                entry.controllable = False
                entry.blocked_by = str(exc)
                blocked.append(entry)

        for entry in claimed:
            log.info("  camera %d controllable: %s", entry.index,
                     entry.describe())
        for entry in blocked:
            log.warning("  camera %d NOT controllable: %s - %s",
                        entry.index, entry.describe(), entry.blocked_by)
        if blocked:
            log.warning("%d camera enable line(s) are held elsewhere; on this "
                        "board the connector enables belong to the kernel's "
                        "cam0_reg/cam1_reg regulators. Downstream cameras "
                        "should use free GPIOs.", len(blocked))

        self._active = NO_CAMERA
        always = self._always_on_index()
        if always != NO_CAMERA:
            # An always-on camera is on right now whatever we do, so reporting
            # anything else as active would be a lie to the ground.
            self._active = always
            log.info("camera %d is always on and is the active camera",
                     always)
        elif self.default_camera != NO_CAMERA:
            if not self.select(int(self.default_camera)):
                log.error("default camera %d could not be selected",
                          self.default_camera)

        if not claimed and always == NO_CAMERA:
            log.error("no camera enable lines could be claimed; "
                      "selection unavailable")
            return False
        log.info("camera selector ready: %d of %d camera(s) switchable, "
                 "active camera %s", len(claimed), len(self.entries),
                 "none" if self._active == NO_CAMERA else self._active)
        return True

    def _always_on_index(self) -> int:
        for index in sorted(self.entries):
            if self.entries[index].always_on:
                return index
        return NO_CAMERA

    def close(self) -> None:
        if not self._requests:
            return
        try:
            self.disable_all()
        except Exception as exc:                       # noqa: BLE001
            log.warning("disabling cameras during close failed: %s", exc)
        for index, request in list(self._requests.items()):
            try:
                request.release()
            except Exception as exc:                   # noqa: BLE001
                log.warning("releasing camera %d failed: %s", index, exc)
        self._requests.clear()

    @property
    def is_open(self) -> bool:
        return bool(self._requests)

    def controllable(self) -> list:
        """Indices this selector can actually switch."""
        return sorted(self._requests)

    # -- switching -------------------------------------------------------

    def _drive(self, entry: CameraEntry, on: bool) -> None:
        request = self._requests.get(entry.index)
        if request is None:
            raise RuntimeError(f"camera {entry.index} is not controllable")
        level = Value.ACTIVE if (on == entry.active_high) else Value.INACTIVE
        request.set_value(entry.gpio, level)

    def disable_all(self) -> None:
        if not self._requests:
            self._active = self._always_on_index()
            return
        for entry in self.entries.values():
            if entry.always_on:
                continue
            try:
                self._drive(entry, False)
            except Exception as exc:                   # noqa: BLE001
                log.error("disabling cam%d failed: %s", entry.index, exc)
        self._active = self._always_on_index()

    def select(self, index: int, settle_s: float = 0.05) -> bool:  # noqa: C901
        """Disable every camera, then enable exactly one.

        `index` of NO_CAMERA disables everything, which is a valid request.
        Disables always happen first, so a failure part-way through leaves the
        fabric quiet rather than contended.
        """
        always = self._always_on_index()
        if index != NO_CAMERA and index not in self.entries:
            log.warning("no camera configured at index %d", index)
            self.failures += 1
            return False
        if index != NO_CAMERA and index not in self._requests and \
                index != always:
            log.warning("camera %d is not switchable: %s", index,
                        self.entries[index].blocked_by or "no line claimed")
            self.failures += 1
            return False
        if always != NO_CAMERA and index != always:
            # Refuse rather than half-do it. Turning the others on while an
            # always-on camera cannot be turned off is exactly the contention
            # this class exists to prevent.
            log.error("cannot select camera %s: camera %d is always on and "
                      "cannot be disabled", index, always)
            self.failures += 1
            return False

        self.disable_all()
        if index == always and index != NO_CAMERA:
            self._active = index
            self.selections += 1
            log.info("camera %d selected (always on)", index)
            return True
        if not self._requests:
            return False
        if index == NO_CAMERA:
            log.info("all cameras disabled")
            self.selections += 1
            return True

        entry = self.entries[index]
        try:
            self._drive(entry, True)
        except Exception as exc:                       # noqa: BLE001
            log.error("enabling cam%d failed: %s", index, exc)
            self.failures += 1
            return False

        # A sensor needs its supply and internal oscillator to settle before it
        # will acknowledge its I2C address.
        if settle_s > 0:
            time.sleep(settle_s)
        self._active = index
        self.selections += 1
        log.info("camera %d selected (%s); all others disabled",
                 index, entry.describe())
        return True

    # -- reporting -------------------------------------------------------

    @property
    def active(self) -> int:
        return self._active

    def active_entry(self) -> CameraEntry | None:
        return self.entries.get(self._active)

    def summary(self) -> dict:
        return {
            "camera_count": len(self.entries) or len(self._requests),
            "camera_active": self._active,
            "camera_selections": self.selections,
            "camera_select_failures": self.failures,
        }

    def pack_table(self) -> bytes:
        """The camera table for the ground: 6 bytes per entry, little-endian.

        index u8, gpio u8, flags u8 (bit0 active-high, bit1 currently enabled,
        bit2 always-on and therefore not switchable),
        i2c_bus i8, name length u8, then that many name bytes.
        """
        import struct
        out = bytearray(struct.pack("<BB", len(self.entries),
                                    self._active & 0xFF))
        for index in sorted(self.entries):
            entry = self.entries[index]
            flags = (0x01 if entry.active_high else 0) | \
                    (0x02 if index == self._active else 0) | \
                    (0x04 if entry.always_on else 0)
            name = entry.name.encode("ascii", "replace")[:16]
            out += struct.pack("<BBBbB", entry.index, entry.gpio & 0xFF,
                               flags, entry.i2c_bus, len(name)) + name
        return bytes(out)


def entries_from_config(raw: list | None) -> list[CameraEntry]:
    """Build entries from the `cameras` list in the daemon configuration."""
    out = []
    for item in (raw or []):
        try:
            out.append(CameraEntry(
                index=int(item["index"]),
                gpio=int(item["gpio"]),
                name=str(item.get("name", "")),
                active_high=bool(item.get("active_high", True)),
                i2c_bus=int(item.get("i2c_bus", -1)),
                always_on=bool(item.get("always_on", False))))
        except Exception as exc:                       # noqa: BLE001
            log.error("bad camera entry %r: %s", item, exc)
    return out
