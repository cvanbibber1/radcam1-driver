"""Half-duplex RS-422 transport for the DICE bus.

The ADM2582E's driver-enable pin is the whole difficulty here. The bus may
carry up to five chained transceivers plus the master, so asserting DE for one
byte longer than necessary corrupts somebody else's traffic, and releasing it
one byte early truncates our own. Both failures are intermittent and miserable
to debug from orbit, so the timing is derived rather than tuned:

    char_time = 10 / baud          (8N1 -> 8 data + start + stop)

The obvious implementation - `Serial.flush()`, which calls `tcdrain()` - is
unusable here, and measurement is the only way that shows it. On this PL011
`tcdrain()` takes **8 to 13 milliseconds regardless of how much was written**:
draining an 8-byte ACK whose wire time is 87 microseconds took 12 ms, holding
DE for 140 times longer than we were actually transmitting. On a bus shared
with up to five other experiments that is not a latency problem, it is a
jamming problem.

What works instead is `TIOCSERGETLSR` / `TIOCSER_TEMT`, the transmitter-empty
bit, which reports the moment the shift register clears. Polled that way, DE is
released within **8 to 37 microseconds** of the exact wire time:

    packet      wire time    DE released after
    8 bytes        87 us        104-123 us
    120 bytes    1302 us           1311 us
    1288 bytes  13976 us          13984 us

To avoid spinning through the whole of a 14 ms HRT packet, the transmitter
sleeps for the computed wire time less a small margin, then polls TEMT for the
remainder. If the ioctl is unavailable the code falls back to `tcdrain()` plus
a `guard_chars` margin, which is correct but coarse - and says so in the log,
because the difference matters to everyone else on the bus.

DE is released in a `finally`, so an exception mid-transmission cannot leave
this experiment jamming the bus for every other payload.
"""

from __future__ import annotations

import fcntl
import logging
import struct
import time

log = logging.getLogger(__name__)

try:
    import serial
except ImportError:                                    # pragma: no cover
    serial = None

try:
    import gpiod
    from gpiod.line import Direction, Value
except ImportError:                                    # pragma: no cover
    gpiod = None

__all__ = ["Rs422Link", "DeLine", "NullDeLine"]

DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 921600
DEFAULT_DE_GPIO = 4                # confirmed for this board
DEFAULT_CHIP = "/dev/gpiochip0"    # pinctrl-rp1

#: Linux ioctls used to release DE at the right instant.
TIOCOUTQ = 0x5411
TIOCSERGETLSR = 0x5459
TIOCSER_TEMT = 0x01

#: How long before the computed end of transmission to stop sleeping and start
#: polling TEMT. Covers scheduler jitter without spinning for long.
_POLL_MARGIN_S = 500e-6


def _busy_wait(seconds: float) -> None:
    """Spin for sub-millisecond intervals that sleep() cannot resolve."""
    if seconds <= 0:
        return
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        pass


class DeLine:
    """The driver-enable GPIO, held as an output for the process lifetime."""

    def __init__(self, gpio: int = DEFAULT_DE_GPIO, chip: str = DEFAULT_CHIP,
                 active_high: bool = True, consumer: str = "radcam-rs422-de"):
        self.gpio = gpio
        self.chip = chip
        self.active_high = active_high
        self.consumer = consumer
        self._request = None

    def open(self) -> None:
        if gpiod is None:
            raise RuntimeError("libgpiod Python bindings are not installed")
        inactive = Value.INACTIVE if self.active_high else Value.ACTIVE
        self._request = gpiod.request_lines(
            self.chip, consumer=self.consumer,
            config={self.gpio: gpiod.LineSettings(
                direction=Direction.OUTPUT, output_value=inactive)})
        log.info("DE on GPIO%d (%s, active %s)", self.gpio, self.chip,
                 "high" if self.active_high else "low")

    def set(self, enabled: bool) -> None:
        if self._request is None:
            return
        on = Value.ACTIVE if self.active_high else Value.INACTIVE
        off = Value.INACTIVE if self.active_high else Value.ACTIVE
        self._request.set_value(self.gpio, on if enabled else off)

    def close(self) -> None:
        if self._request is not None:
            try:
                self.set(False)
                self._request.release()
            except Exception as exc:                   # noqa: BLE001
                log.warning("releasing DE line failed: %s", exc)
            self._request = None

    @property
    def is_open(self) -> bool:
        return self._request is not None


class NullDeLine(DeLine):
    """Stand-in for boards where DE is not software-controlled.

    Only safe when this experiment is the sole transmitter on the bus, or the
    transceiver does its own direction control.
    """

    def __init__(self):
        super().__init__(gpio=-1)

    def open(self) -> None:
        log.warning("DE is not software-controlled; this is only safe if "
                    "nothing else transmits on the bus")

    def set(self, enabled: bool) -> None:
        return

    def close(self) -> None:
        return

    @property
    def is_open(self) -> bool:
        return True


class Rs422Link:
    """A half-duplex RS-422 port with driver-enable control.

    `send()` is the only path that touches DE, and it is serialised by the
    caller (the experiment holds a TX lock), so DE can never be asserted by two
    transmissions at once.
    """

    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD,
                 de: DeLine | None = None, guard_chars: float = 2.0,
                 setup_us: float = 10.0, discard_echo: bool = True):
        self.port = port
        self.baud = baud
        self.de = de if de is not None else DeLine()
        self.guard_chars = guard_chars
        self.setup_s = setup_us / 1e6
        self.discard_echo = discard_echo
        self._serial = None
        self._have_lsr = False
        self.tx_packets = 0
        self.tx_bytes = 0
        self.tx_errors = 0
        #: Counts transmissions where TEMT did not assert in time and the
        #: guard expired instead. Non-zero means DE timing is being estimated.
        self.tx_drain_timeouts = 0

    # -- lifecycle -------------------------------------------------------

    @property
    def char_time_s(self) -> float:
        """One 8N1 character time, in seconds."""
        return 10.0 / float(self.baud)

    @property
    def guard_s(self) -> float:
        return self.guard_chars * self.char_time_s

    def open(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self._serial = serial.Serial(self.port, self.baud, timeout=0,
                                     write_timeout=2.0)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self._have_lsr = self._probe_lsr()
        self.de.open()
        self.de.set(False)
        log.info("RS-422 up on %s at %d baud (char %.2f us), DE release by %s",
                 self.port, self.baud, self.char_time_s * 1e6,
                 "TEMT" if self._have_lsr
                 else "tcdrain + %.0f us guard" % (self.guard_s * 1e6))
        if not self._have_lsr:
            log.warning("TIOCSERGETLSR unavailable on %s: DE will be held "
                        "longer than the packet needs, which risks colliding "
                        "with other devices on the bus", self.port)

    def _probe_lsr(self) -> bool:
        """Is the transmitter-empty bit readable on this port?"""
        try:
            fcntl.ioctl(self._serial.fileno(), TIOCSERGETLSR,
                        struct.pack("I", 0))
            return True
        except OSError as exc:
            log.debug("TIOCSERGETLSR probe failed: %s", exc)
            return False

    def _transmitter_empty(self) -> bool:
        raw = fcntl.ioctl(self._serial.fileno(), TIOCSERGETLSR,
                          struct.pack("I", 0))
        return bool(struct.unpack("I", raw)[0] & TIOCSER_TEMT)

    def _await_transmission(self, nbytes: int, started: float) -> None:
        """Return as close as possible to the last stop bit leaving the pin.

        `started` is the instant just before `write()` was called, and it has
        to be, because `write()` does not return promptly for a full packet:
        the kernel write buffer is smaller than a 1288-byte HRT payload, so the
        call blocks for most of the transmission it is queuing. Timing the wait
        from after the write instead of from before it double-counts that,
        which measured as 24.5 ms of DE assertion on a 14.0 ms packet.
        """
        wire_s = nbytes * self.char_time_s

        if not self._have_lsr:
            self._serial.flush()               # tcdrain: correct but coarse
            _busy_wait(self.guard_s)
            return

        # Sleep through whatever is left of the transmission rather than
        # spinning; the payload is power-minimised and a 1288-byte packet is
        # 14 ms. Most of it has usually already elapsed inside write().
        remaining = (started + wire_s - _POLL_MARGIN_S) - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)

        # Then poll for the shift register to clear. Bounded, so a driver that
        # never asserts TEMT cannot wedge the transmit path.
        deadline = started + wire_s + 0.010
        while time.perf_counter() < deadline:
            try:
                if self._transmitter_empty():
                    return
            except OSError:
                # The ioctl worked at open and has stopped working; fall back
                # rather than spin to the deadline on every packet from now on.
                self._have_lsr = False
                self._serial.flush()
                _busy_wait(self.guard_s)
                return
        self.tx_drain_timeouts += 1
        log.warning("transmitter-empty not seen within deadline for %d bytes",
                    nbytes)

    def close(self) -> None:
        try:
            self.de.close()
        finally:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:                      # noqa: BLE001
                    pass
                self._serial = None

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    # -- traffic ---------------------------------------------------------

    def read(self, size: int = 8192) -> bytes:
        """Non-blocking drain of whatever has arrived."""
        if not self.is_open:
            return b""
        try:
            waiting = self._serial.in_waiting
            if not waiting:
                return b""
            return self._serial.read(min(waiting, size))
        except Exception as exc:                       # noqa: BLE001
            log.error("RS-422 read failed: %s", exc)
            return b""

    def read_wait(self, timeout_s: float = 0.005, size: int = 8192) -> bytes:
        """Block up to `timeout_s` for the first byte, then drain the rest.

        The service loop has to answer an LRT request promptly, which argues
        for polling fast; the payload is power-minimised, which argues against
        spinning. Blocking on the first byte gets both: the thread sleeps in
        the kernel until the UART has something, and once woken it takes
        everything available in one further read.
        """
        if not self.is_open:
            return b""
        try:
            previous = self._serial.timeout
            if previous != timeout_s:
                self._serial.timeout = timeout_s
            first = self._serial.read(1)
            if not first:
                return b""
            waiting = self._serial.in_waiting
            if waiting:
                return first + self._serial.read(min(waiting, size - 1))
            return first
        except Exception as exc:                       # noqa: BLE001
            log.error("RS-422 read failed: %s", exc)
            return b""

    def send(self, data: bytes) -> bool:
        """Transmit one packet with DE asserted for exactly its duration."""
        if not self.is_open or not data:
            return False

        try:
            self.de.set(True)
            # Let the isolated driver's outputs settle before the start bit.
            _busy_wait(self.setup_s)
            started = time.perf_counter()
            self._serial.write(data)
            self._await_transmission(len(data), started)
        except Exception as exc:                       # noqa: BLE001
            self.tx_errors += 1
            log.error("RS-422 write failed: %s", exc)
            return False
        finally:
            # Never leave the bus driven, whatever went wrong above.
            self.de.set(False)

        self.tx_packets += 1
        self.tx_bytes += len(data)

        if self.discard_echo:
            # If /RE is not tied to DE the transceiver hears our own
            # transmission. DICE is the master and does not talk while we
            # answer, so anything sitting in the input buffer now is echo.
            try:
                self._serial.reset_input_buffer()
            except Exception:                          # noqa: BLE001
                pass
        return True

    def stats(self) -> dict[str, int]:
        return {"tx_packets": self.tx_packets, "tx_bytes": self.tx_bytes,
                "tx_errors": self.tx_errors,
                "tx_drain_timeouts": self.tx_drain_timeouts}
