"""LTC2485 24-bit delta-sigma ADC - the radiation dosimeter front end.

Wiring on this board: SDA = GPIO2, SCL = GPIO3, so bus i2c-1. The part answers
at 0x24, which per the datasheet address table means CA0 and CA1 are both left
floating.

Protocol notes that matter:

* The LTC2485 has no register map. A read returns 4 bytes holding the last
  completed conversion; an optional single command byte configures the *next*
  conversion.
* While a conversion is in flight the part NAKs its address. A read that fails
  with EREMOTEIO is therefore "not ready", not an error - poll until it ACKs.
  A conversion takes roughly 150 ms in 1x / 50-60 Hz rejection mode.

The 32-bit to signed-24-bit decode follows Analog Devices' own Linduino
reference driver (LTSketchbook/libraries/LTC2485/LTC2485.cpp).
"""

from __future__ import annotations

import fcntl
import logging
import os
import time
from contextlib import contextmanager

from smbus2 import SMBus, i2c_msg

log = logging.getLogger(__name__)

DEFAULT_BUS = 1
DEFAULT_ADDR = 0x24

#: Cross-process lock directory. The LTC2485 has no register map and no way to
#: interleave transactions safely: a conversion belongs to whoever started it.
#: Two readers (radcamd plus a radcamctl invocation) will otherwise consume each
#: other's conversions and both get nonsense. A file lock serialises them.
LOCK_DIR = "/run/radcam"


@contextmanager
def bus_lock(bus: int, timeout: float = 5.0):
    """Serialise access to one I2C bus across processes."""
    try:
        os.makedirs(LOCK_DIR, exist_ok=True)
        path = os.path.join(LOCK_DIR, f"i2c-{bus}.lock")
        fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
    except OSError as exc:
        # Losing the lock file must not stop a flight system reading its
        # dosimeter; degrade to unlocked rather than failing.
        log.warning("i2c lock unavailable (%s); proceeding unlocked", exc)
        yield False
        return

    deadline = time.monotonic() + timeout
    held = False
    try:
        while True:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                held = True
                break
            except OSError:
                if time.monotonic() >= deadline:
                    log.warning("i2c-%d lock timeout; proceeding unlocked", bus)
                    break
                time.sleep(0.02)
        yield held
    finally:
        if held:
            try:
                fcntl.flock(fd, fcntl.LOCK_UN)
            except OSError:
                pass
        os.close(fd)

# Command byte fields (datasheet Table 1).
SPEED_1X = 0x00
SPEED_2X = 0x01
REJECT_50_60 = 0x00     # rejects both mains frequencies - slowest, quietest
REJECT_50 = 0x02
REJECT_60 = 0x04
INTERNAL_TEMP = 0x08

# 1x speed with 50/60 Hz rejection: the lowest noise setting, which is what a
# slowly-integrating dose measurement wants.
DEFAULT_COMMAND = SPEED_1X | REJECT_50_60

# Full scale is +/- VREF/2 spread over a signed 24-bit code.
FULL_SCALE_CODE = 1 << 23

POSITIVE_OVERFLOW = 0xC0
NEGATIVE_OVERFLOW = 0x3F


class LTC2485Error(Exception):
    pass


class ConversionPending(LTC2485Error):
    """The ADC NAKed because a conversion is still running."""


class LTC2485:
    """Driver for a single LTC2485 on an I2C bus.

    `vref` is the reference voltage fitted on the board. It only sets the
    nominal scale factor; the dosimeter's two-point calibration absorbs any
    error in it, so an approximate value here is not a problem.
    """

    def __init__(self, bus: int = DEFAULT_BUS, address: int = DEFAULT_ADDR,
                 vref: float = 5.0, command: int = DEFAULT_COMMAND):
        self.bus_num = bus
        self.address = address
        self.vref = vref
        self.command = command
        self._bus: SMBus | None = None

    def open(self) -> "LTC2485":
        if self._bus is None:
            self._bus = SMBus(self.bus_num)
        return self

    def close(self) -> None:
        if self._bus is not None:
            self._bus.close()
            self._bus = None

    def __enter__(self) -> "LTC2485":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    # -- raw access ------------------------------------------------------

    def _read_raw(self) -> int:
        """Read the 4-byte conversion word. Raises ConversionPending on NAK."""
        if self._bus is None:
            raise LTC2485Error("bus not open")
        msg = i2c_msg.read(self.address, 4)
        try:
            self._bus.i2c_rdwr(msg)
        except OSError as exc:
            # EREMOTEIO (121) is the documented "still converting" response.
            raise ConversionPending(str(exc)) from exc
        data = bytes(bytearray(list(msg)))
        if len(data) != 4:
            raise LTC2485Error(f"short read: {len(data)} bytes")
        return int.from_bytes(data, "big")

    @staticmethod
    def decode(raw: int) -> int:
        """Convert the 32-bit word into a signed 24-bit code.

        Mirrors LTC2485_read() in Analog Devices' Linduino library: detect the
        overflow patterns, strip the sign/EOC flag, shift left one to restore
        two's complement, then narrow from 32 to 24 bits.
        """
        msb = (raw >> 24) & 0xFF
        if msb == POSITIVE_OVERFLOW:
            return FULL_SCALE_CODE - 1
        if msb == NEGATIVE_OVERFLOW:
            return -FULL_SCALE_CODE

        value = raw & 0x7FFFFFFF        # remove sign/EOC bit
        value = (value << 1) & 0xFFFFFFFF

        # Interpret as signed 32-bit, then arithmetic-shift down to 24 bits.
        if value >= 0x80000000:
            value -= 0x100000000
        return value // 256

    def read_code(self, timeout: float = 1.0,
                  poll_interval: float = 0.01) -> int:
        """Poll until a conversion is available and return its signed code."""
        deadline = time.monotonic() + timeout
        last: Exception | None = None
        # Hold the lock across the whole poll: a conversion belongs to the
        # reader that waited for it.
        with bus_lock(self.bus_num, timeout=timeout):
            while time.monotonic() < deadline:
                try:
                    return self.decode(self._read_raw())
                except ConversionPending as exc:
                    last = exc
                    time.sleep(poll_interval)
        raise LTC2485Error(f"no conversion within {timeout}s ({last})")

    def code_to_volts(self, code: int) -> float:
        """Nominal volts for a signed code. Full scale is +/- VREF/2."""
        return code * (self.vref / 2.0) / FULL_SCALE_CODE

    def read_volts(self, **kw) -> float:
        return self.code_to_volts(self.read_code(**kw))

    def read_code_tmr(self, samples: int = 3, **kw) -> int:
        """Take several conversions and return their median.

        Guards a single reading against a transient upset on the bus or in the
        converter, which matters more than usual in orbit.
        """
        values = sorted(self.read_code(**kw) for _ in range(samples))
        return values[len(values) // 2]


def probe(bus: int = DEFAULT_BUS, address: int = DEFAULT_ADDR) -> bool:
    """True if an LTC2485 answers at this address."""
    try:
        with LTC2485(bus, address) as adc:
            adc.read_code(timeout=1.0)
        return True
    except (OSError, LTC2485Error):
        return False
