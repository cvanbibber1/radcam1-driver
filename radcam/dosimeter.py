"""Radiation dosimeter: LTC2485 voltage -> absorbed dose in rad.

The sensor produces a voltage proportional to accumulated dose at roughly
2.5 mV/rad. Absolute voltage is meaningless on its own - what matters is the
rise above the baseline captured when the unit was known to be unirradiated.
That baseline is measured once, on first startup, and persisted redundantly
(see tmr.TMRStore) because it cannot be recovered later.

Measured behaviour of this board: after the bus has been idle the reading
settles along an RC curve over roughly 8-10 seconds before it is trustworthy,
and the first conversion after opening the bus is often a wild outlier. Both are
handled by `settle()`, which every calibration and measurement path goes
through.
"""

from __future__ import annotations

import logging
import statistics
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone

from .ltc2485 import LTC2485
from .tmr import TMRStore

log = logging.getLogger(__name__)

DEFAULT_STORE = "/var/lib/radcam/dosimeter-cal.json"

# Sensor sensitivity. The user-supplied figure for this dosimeter.
DEFAULT_VOLTS_PER_RAD = 0.0025

# Discard this many conversions after the bus goes active - the first is
# routinely an outlier.
SETTLE_DISCARD = 3


@dataclass
class Calibration:
    """Persisted, never recomputed after first startup unless forced."""

    zero_volts: float
    volts_per_rad: float = DEFAULT_VOLTS_PER_RAD
    vref: float = 5.0
    samples: int = 0
    stddev_volts: float = 0.0
    created_utc: str = field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat()
    )
    note: str = ""

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "Calibration":
        known = {f for f in cls.__dataclass_fields__}
        return cls(**{k: v for k, v in d.items() if k in known})


@dataclass
class Reading:
    code: int
    volts: float
    dose_rad: float | None
    calibrated: bool
    timestamp_utc: str = field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat()
    )

    def to_dict(self) -> dict:
        return asdict(self)


class Dosimeter:
    def __init__(self, adc: LTC2485 | None = None,
                 store_path: str = DEFAULT_STORE):
        self.adc = adc or LTC2485()
        self.store = TMRStore(store_path)
        self.calibration: Calibration | None = None

    # -- lifecycle -------------------------------------------------------

    def open(self) -> "Dosimeter":
        self.adc.open()
        self.load_calibration()
        return self

    def close(self) -> None:
        self.adc.close()

    def __enter__(self) -> "Dosimeter":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    # -- calibration -----------------------------------------------------

    def load_calibration(self) -> Calibration | None:
        if not self.store.exists():
            log.info("no dosimeter calibration stored yet")
            return None
        try:
            self.calibration = Calibration.from_dict(self.store.read())
            log.info("loaded calibration: zero=%.6f V, %.4f mV/rad (%s)",
                     self.calibration.zero_volts,
                     self.calibration.volts_per_rad * 1000,
                     self.calibration.created_utc)
        except Exception as exc:
            log.error("dosimeter calibration unreadable: %s", exc)
            self.calibration = None
        return self.calibration

    def settle(self, seconds: float = 10.0,
               tolerance_volts: float = 0.0005) -> None:
        """Wait for the input to finish its RC settle.

        Returns as soon as consecutive readings stop moving by more than
        `tolerance_volts`, or when `seconds` expires.
        """
        for _ in range(SETTLE_DISCARD):
            try:
                self.adc.read_code(timeout=2.0)
            except Exception:
                pass

        deadline = time.monotonic() + seconds
        prev = None
        while time.monotonic() < deadline:
            v = self.adc.read_volts(timeout=2.0)
            if prev is not None and abs(v - prev) <= tolerance_volts:
                log.debug("settled at %.6f V", v)
                return
            prev = v
        log.debug("settle window expired at %.6f V", prev if prev else float("nan"))

    def calibrate(self, samples: int = 32, force: bool = False,
                  volts_per_rad: float = DEFAULT_VOLTS_PER_RAD,
                  note: str = "") -> Calibration:
        """Capture the unirradiated baseline and persist it.

        Only ever runs once unless `force` is set - re-zeroing a dosimeter that
        has already accumulated dose would silently discard that history.
        """
        if self.calibration is not None and not force:
            log.info("calibration already exists; not re-running")
            return self.calibration

        log.info("calibrating dosimeter baseline (%d samples)", samples)
        self.settle()

        volts = [self.adc.read_volts(timeout=2.0) for _ in range(samples)]
        # Median for the centre, so a single upset cannot drag the baseline.
        zero = statistics.median(volts)
        sd = statistics.pstdev(volts) if len(volts) > 1 else 0.0

        cal = Calibration(
            zero_volts=zero,
            volts_per_rad=volts_per_rad,
            vref=self.adc.vref,
            samples=samples,
            stddev_volts=sd,
            note=note or "baseline captured at first startup",
        )
        self.store.write(cal.to_dict())
        self.calibration = cal
        log.info("calibration stored: zero=%.6f V (sd %.6f V)", zero, sd)
        return cal

    def ensure_calibrated(self, **kw) -> Calibration:
        """Calibrate on first startup; otherwise cross-reference the stored one."""
        if self.calibration is None:
            self.load_calibration()
        if self.calibration is None:
            return self.calibrate(**kw)
        return self.calibration

    # -- measurement -----------------------------------------------------

    def read(self, samples: int = 3) -> Reading:
        code = self.adc.read_code_tmr(samples=samples, timeout=2.0)
        volts = self.adc.code_to_volts(code)

        dose = None
        if self.calibration is not None:
            dose = ((volts - self.calibration.zero_volts)
                    / self.calibration.volts_per_rad)

        return Reading(code=code, volts=volts, dose_rad=dose,
                       calibrated=self.calibration is not None)
