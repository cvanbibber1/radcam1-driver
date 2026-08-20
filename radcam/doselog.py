"""Long-term dose history.

The dosimeter drifts over weeks to months, so the mission wants a coarse time
series it can download later, not a high-rate stream. One record per minute is
ample: a year of continuous operation is ~526k records, about 12 MB.

Each record is fixed-width and carries its own CRC-32, so a bit flip in storage
corrupts exactly one minute of history rather than desynchronising the file.
Records are appended and never rewritten, which is also the friendliest pattern
for flash endurance.

Timestamps are deliberately "rough". The Pi has no guaranteed RTC battery, so
wall-clock time can jump backwards after a power cycle. Every record therefore
carries **both** the wall clock and the monotonic uptime; the monotonic value
survives clock resets and lets the ground reconstruct ordering and elapsed time
even when the wall clock is wrong.
"""

from __future__ import annotations

import logging
import os
import struct
import time
import zlib
from dataclasses import dataclass
from pathlib import Path

log = logging.getLogger(__name__)

DEFAULT_PATH = "/var/lib/radcam/dose-log.bin"

# unix_ts (d) | uptime_s (d) | dose_rad (f) | volts (f) | crc32 (I)
RECORD_FMT = "<ddffI"
RECORD_LEN = struct.calcsize(RECORD_FMT)     # 28 bytes

#: Stop the log growing without bound; at 1/minute this is several years.
DEFAULT_MAX_RECORDS = 2_000_000


@dataclass
class DoseRecord:
    unix_ts: float
    uptime_s: float
    dose_rad: float
    volts: float

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.unix_ts, self.dose_rad, self.volts)


def _pack(rec: DoseRecord) -> bytes:
    body = struct.pack("<ddff", rec.unix_ts, rec.uptime_s,
                       rec.dose_rad, rec.volts)
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def _unpack(blob: bytes) -> DoseRecord | None:
    """Return the record, or None if its CRC fails."""
    if len(blob) != RECORD_LEN:
        return None
    body, crc = blob[:-4], struct.unpack("<I", blob[-4:])[0]
    if (zlib.crc32(body) & 0xFFFFFFFF) != crc:
        return None
    unix_ts, uptime_s, dose, volts = struct.unpack("<ddff", body)
    return DoseRecord(unix_ts, uptime_s, dose, volts)


class DoseLog:
    def __init__(self, path: str = DEFAULT_PATH,
                 max_records: int = DEFAULT_MAX_RECORDS):
        self.path = Path(path)
        self.max_records = max_records
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.corrupt_records = 0

    def append(self, dose_rad: float, volts: float) -> DoseRecord:
        rec = DoseRecord(time.time(), time.monotonic(), dose_rad, volts)
        try:
            with open(self.path, "ab") as fh:
                fh.write(_pack(rec))
                fh.flush()
                os.fsync(fh.fileno())
        except OSError as exc:
            log.error("cannot append to dose log: %s", exc)
        self._trim()
        return rec

    def count(self) -> int:
        try:
            return self.path.stat().st_size // RECORD_LEN
        except OSError:
            return 0

    def _trim(self) -> None:
        """Drop the oldest records once the cap is exceeded."""
        n = self.count()
        if n <= self.max_records:
            return
        keep = self.max_records // 2          # halve, so trimming is rare
        try:
            with open(self.path, "rb") as fh:
                fh.seek((n - keep) * RECORD_LEN)
                tail = fh.read()
            tmp = self.path.with_suffix(".tmp")
            tmp.write_bytes(tail)
            os.replace(tmp, self.path)
            log.info("dose log trimmed to %d records", keep)
        except OSError as exc:
            log.error("cannot trim dose log: %s", exc)

    def read(self, start_ts: float | None = None,
             end_ts: float | None = None,
             limit: int | None = None) -> list[DoseRecord]:
        """Read records, optionally bounded by wall-clock time.

        Records failing CRC are skipped and counted rather than raising: one
        corrupt minute must not make the rest of the history unreadable.
        """
        out: list[DoseRecord] = []
        try:
            with open(self.path, "rb") as fh:
                while True:
                    blob = fh.read(RECORD_LEN)
                    if len(blob) < RECORD_LEN:
                        break
                    rec = _unpack(blob)
                    if rec is None:
                        self.corrupt_records += 1
                        continue
                    if start_ts is not None and rec.unix_ts < start_ts:
                        continue
                    if end_ts is not None and rec.unix_ts > end_ts:
                        continue
                    out.append(rec)
                    if limit and len(out) >= limit:
                        break
        except FileNotFoundError:
            return []
        except OSError as exc:
            log.error("cannot read dose log: %s", exc)
        return out

    def summary(self) -> dict:
        recs = self.read()
        if not recs:
            return {"records": 0, "corrupt": self.corrupt_records}
        doses = [r.dose_rad for r in recs]
        return {
            "records": len(recs),
            "corrupt": self.corrupt_records,
            "first_unix": recs[0].unix_ts,
            "last_unix": recs[-1].unix_ts,
            "span_hours": (recs[-1].uptime_s - recs[0].uptime_s) / 3600.0,
            "dose_min": min(doses),
            "dose_max": max(doses),
            "dose_last": doses[-1],
        }
