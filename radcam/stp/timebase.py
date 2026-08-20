"""DICE time <-> UTC, on the GPS epoch.

The mission has confirmed Coarse Time counts seconds from the **GPS epoch,
1980-01-06 00:00:00 UTC**. GPS time does not observe leap seconds, so it runs
ahead of UTC by a count that changes when the IERS inserts one; as of the 2017
insertion that offset is 18 s and it has not moved since.

The offset is a parameter, not a constant, because a leap second inserted
during the mission would otherwise put every timestamp a second out with no way
to correct it from the ground. It is also the reason raw ticks are what get
stored: `radcam/stp/lrt.py` reports `coarse_time` and `fine_time` exactly as
received, so if the offset turns out to be wrong every past record can still be
re-derived. Only the presentation layer applies the conversion.

Fine Time is ~15.3 microseconds per count. Two bytes at that scale wrap after
about one second, which is consistent with it being the sub-second part of the
coarse count rather than an independent clock.
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

__all__ = ["GPS_EPOCH", "GPS_UTC_OFFSET_S", "FINE_TICK_S",
           "dice_to_unix", "unix_to_dice", "dice_to_datetime", "split_unix"]

#: 1980-01-06 00:00:00 UTC, as a Unix timestamp.
GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
GPS_EPOCH_UNIX = GPS_EPOCH.timestamp()          # 315964800.0

#: Seconds GPS runs ahead of UTC. 18 since 2017-01-01; update if IERS inserts.
GPS_UTC_OFFSET_S = 18

#: Fine Time resolution, seconds per count, as stated by the ICD.
FINE_TICK_S = 15.3e-6

#: Coarse Time is 4 bytes, so it wraps here - in the year 2116, comfortably
#: past any mission, but the masks below keep the arithmetic honest anyway.
COARSE_MODULO = 1 << 32
FINE_MODULO = 1 << 16


def dice_to_unix(coarse: int, fine: int = 0,
                 leap_offset_s: int = GPS_UTC_OFFSET_S) -> float:
    """Convert a DICE timestamp to a Unix (UTC) timestamp."""
    return (GPS_EPOCH_UNIX + (coarse % COARSE_MODULO)
            + (fine % FINE_MODULO) * FINE_TICK_S - leap_offset_s)


def unix_to_dice(unix_ts: float,
                 leap_offset_s: int = GPS_UTC_OFFSET_S) -> tuple[int, int]:
    """Convert a Unix timestamp to (coarse, fine) DICE ticks."""
    gps = unix_ts - GPS_EPOCH_UNIX + leap_offset_s
    if gps < 0:
        return 0, 0
    coarse = int(gps)
    fine = int(round((gps - coarse) / FINE_TICK_S))
    if fine >= FINE_MODULO:
        # Rounding can push the sub-second part past what 16 bits hold.
        coarse += 1
        fine = 0
    return coarse % COARSE_MODULO, fine


def dice_to_datetime(coarse: int, fine: int = 0,
                     leap_offset_s: int = GPS_UTC_OFFSET_S) -> datetime:
    """Convert a DICE timestamp to an aware UTC datetime."""
    return datetime.fromtimestamp(dice_to_unix(coarse, fine, leap_offset_s),
                                  tz=timezone.utc)


def split_unix(unix_ts: float,
               leap_offset_s: int = GPS_UTC_OFFSET_S) -> tuple[int, int]:
    """Alias of `unix_to_dice`, named for use at the transmit side."""
    return unix_to_dice(unix_ts, leap_offset_s)
