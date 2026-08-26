"""Telemetry downlink: RS422 flight link, mirrored to a ground debug UART.

Frame format is a checksummed ASCII line, deliberately readable so a ground
operator can watch the debug port with nothing more than a terminal:

    $RADCAM,<seq>,<utc>,<key>=<value>,...*<crc16>\\r\\n

The CRC is CRC-16/CCITT-FALSE over everything between the '$' and the '*'. A
frame that fails CRC is discarded by the receiver rather than acted on.

Fields whose corruption would actually matter - dose, calibration, state - are
sent triplicated as `key=v|v|v`. A single bit flip anywhere in transit is then
corrected by majority vote at the receiving end instead of merely detected.
That is cheap here: telemetry is small and the link is not bandwidth bound.

Hardware note: the flight link is uart0 on GPIO14/15 (/dev/ttyAMA0), feeding an
ADM2582E isolated RS422 transceiver. The mirror port is configurable; see
DEVELOPMENT_STATE.md for why GPIO23/24 cannot be a hardware UART on a Pi 5.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Iterable

import serial

log = logging.getLogger(__name__)

FLIGHT_PORT = "/dev/ttyAMA0"        # uart0, GPIO14 TXD / GPIO15 RXD -> RS422
DEBUG_PORT = "/dev/ttyAMA10"        # Pi 5 dedicated debug UART header
#: 921600 everywhere. This was 115200 for the legacy ASCII beacon while
#: protocol.md's whole link budget assumed 921600 - an 8x discrepancy that made
#: every transfer estimate wrong. The mission has since confirmed 921600 and the
#: rate is proven on the hardware, so the two agree at last.
DEFAULT_BAUD = 921600

FRAME_START = "$"
CRC_SEP = "*"
TMR_SEP = "|"


def crc16_ccitt(data: bytes, seed: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE."""
    crc = seed
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 \
                else (crc << 1) & 0xFFFF
    return crc


def _fmt(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    if isinstance(value, bool):
        return "1" if value else "0"
    if value is None:
        return ""
    return str(value)


def encode_frame(fields: dict[str, Any], seq: int,
                 tmr_keys: Iterable[str] = ()) -> bytes:
    """Build one telemetry frame. Keys in `tmr_keys` are sent triplicated."""
    tmr = set(tmr_keys)
    utc = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"

    parts = [f"RADCAM,{seq},{utc}"]
    for key, value in fields.items():
        text = _fmt(value)
        if key in tmr:
            text = TMR_SEP.join((text, text, text))
        parts.append(f"{key}={text}")

    body = ",".join(parts)
    crc = crc16_ccitt(body.encode("ascii", "replace"))
    return f"{FRAME_START}{body}{CRC_SEP}{crc:04X}\r\n".encode("ascii", "replace")


def _vote(text: str) -> str:
    """Majority-vote a triplicated field value."""
    if TMR_SEP not in text:
        return text
    copies = text.split(TMR_SEP)
    best = max(set(copies), key=copies.count)
    if copies.count(best) < 2:
        # No majority: all three differ, so nothing is trustworthy.
        raise ValueError(f"no majority in TMR field {text!r}")
    return best


def decode_frame(line: str | bytes) -> dict[str, Any]:
    """Parse and validate a frame. Raises ValueError if it is not intact."""
    if isinstance(line, bytes):
        line = line.decode("ascii", "replace")
    line = line.strip()

    if not line.startswith(FRAME_START) or CRC_SEP not in line:
        raise ValueError("malformed frame")

    body, _, crc_text = line[1:].rpartition(CRC_SEP)
    try:
        want = int(crc_text, 16)
    except ValueError as exc:
        raise ValueError(f"bad CRC field {crc_text!r}") from exc

    got = crc16_ccitt(body.encode("ascii", "replace"))
    if got != want:
        raise ValueError(f"CRC mismatch: computed {got:04X}, frame says {want:04X}")

    tokens = body.split(",")
    if len(tokens) < 3 or tokens[0] != "RADCAM":
        raise ValueError("not a RADCAM frame")

    out: dict[str, Any] = {"seq": int(tokens[1]), "utc": tokens[2]}
    for token in tokens[3:]:
        if "=" not in token:
            continue
        key, _, value = token.partition("=")
        out[key] = _vote(value)
    return out


@dataclass
class PortConfig:
    device: str
    baud: int = DEFAULT_BAUD
    name: str = ""


class TelemetryLink:
    """One serial port. Failures are contained so a dead port cannot
    take the other one down with it."""

    def __init__(self, cfg: PortConfig):
        self.cfg = cfg
        self.name = cfg.name or cfg.device
        self._port: serial.Serial | None = None
        self.failures = 0

    def open(self) -> bool:
        try:
            self._port = serial.Serial(
                self.cfg.device, self.cfg.baud, timeout=0, write_timeout=1.0
            )
            log.info("telemetry port %s open at %d baud",
                     self.name, self.cfg.baud)
            return True
        except (OSError, serial.SerialException) as exc:
            log.error("cannot open telemetry port %s: %s", self.name, exc)
            self._port = None
            return False

    def close(self) -> None:
        if self._port is not None:
            try:
                self._port.close()
            except Exception:
                pass
            self._port = None

    @property
    def is_open(self) -> bool:
        return self._port is not None and self._port.is_open

    def write(self, data: bytes) -> bool:
        """Write a frame. Returns False on failure and tries to reopen."""
        if not self.is_open and not self.open():
            return False
        try:
            self._port.write(data)          # type: ignore[union-attr]
            self._port.flush()              # type: ignore[union-attr]
            return True
        except (OSError, serial.SerialException) as exc:
            self.failures += 1
            log.error("telemetry write failed on %s (%d failures): %s",
                      self.name, self.failures, exc)
            self.close()
            return False

    def read_bytes(self, size: int = 4096) -> bytes:
        """Non-blocking read of whatever has arrived. Never raises."""
        if not self.is_open:
            return b""
        try:
            waiting = self._port.in_waiting          # type: ignore[union-attr]
            if not waiting:
                return b""
            return self._port.read(min(waiting, size))  # type: ignore[union-attr]
        except (OSError, serial.SerialException) as exc:
            log.error("telemetry read failed on %s: %s", self.name, exc)
            self.close()
            return b""

    def read_line(self) -> str | None:
        if not self.is_open:
            return None
        try:
            raw = self._port.readline()     # type: ignore[union-attr]
        except (OSError, serial.SerialException) as exc:
            log.error("telemetry read failed on %s: %s", self.name, exc)
            self.close()
            return None
        return raw.decode("ascii", "replace") if raw else None


class NullTelemetryLink(TelemetryLink):
    """A flight link that goes nowhere.

    Used when the STP/DICE protocol owns the flight port. Two things must not
    happen on that bus: a second `serial.Serial` opening the same device, and
    an unsolicited ASCII beacon landing in the middle of another experiment's
    reply. Substituting this for the flight link makes both structurally
    impossible rather than merely discouraged - there is no port to write to.
    """

    def __init__(self, reason: str = "flight port owned by the STP link"):
        super().__init__(PortConfig("/dev/null", name="flight-disabled"))
        self.reason = reason

    def open(self) -> bool:
        return False

    def close(self) -> None:
        return

    @property
    def is_open(self) -> bool:
        return False

    def write(self, data: bytes) -> bool:
        return False

    def read_bytes(self, size: int = 4096) -> bytes:
        return b""

    def read_line(self) -> str | None:
        return None


class Telemetry:
    """The flight link plus an optional ground-debug mirror.

    Every frame goes to both ports. The mirror is best effort: if the ground
    port is absent or fails, the flight link carries on regardless, which is
    the behaviour a zero-intervention system needs.

    Passing `mirror=None` disables mirroring entirely. That is the default,
    because on a Pi 5 the GPIO23/24 pins this project wants for EXTUART have no
    hardware UART function, so the port has to be chosen deliberately rather
    than guessed at - picking one by default risks writing telemetry into the
    serial console.
    """

    def __init__(self, flight: PortConfig | None = None,
                 mirror: PortConfig | None = None,
                 tmr_keys: Iterable[str] = ()):
        self.flight = TelemetryLink(
            flight or PortConfig(FLIGHT_PORT, name="flight-rs422"))
        self.mirror = TelemetryLink(mirror) if mirror is not None else None
        self.tmr_keys = set(tmr_keys)
        self.seq = 0

    def open(self) -> "Telemetry":
        self.flight.open()
        if self.mirror is not None:
            self.mirror.open()
        return self

    def close(self) -> None:
        self.flight.close()
        if self.mirror is not None:
            self.mirror.close()

    def __enter__(self) -> "Telemetry":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    def send(self, fields: dict[str, Any],
             flight: bool = True) -> dict[str, bool]:
        """Send one ASCII housekeeping frame.

        `flight=False` keeps the human-readable beacon off the flight link.
        That matters when the flight link also carries the binary command
        protocol: the two framings cannot share a stream, because the binary
        reader treats everything before a 0x00 delimiter as one frame and
        would splice the ASCII text onto the front of the next binary frame.
        The ground-debug mirror is where the readable form belongs anyway.
        """
        frame = encode_frame(fields, self.seq, self.tmr_keys)
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        return {
            "flight": self.flight.write(frame) if flight else False,
            "mirror": (self.mirror.write(frame)
                       if self.mirror is not None else False),
        }
