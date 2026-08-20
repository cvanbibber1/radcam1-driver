"""Triple modular redundancy for state held in RAM.

`radcam/tmr.py` already protects what lives on disk. This module protects what
lives in memory between reboots, which in low Earth orbit is not a theoretical
concern: an SEU in the byte holding `hrt_enabled` either jams the bus or goes
silent, and neither state announces itself.

The approach mirrors `tmr.py` deliberately - three copies, byte-wise majority
vote on read, damaged copies rewritten in place - so there is one mental model
for redundancy across the codebase rather than two.

What this can and cannot do:

* One corrupted copy is corrected *and repaired*, silently, on the next read.
* Two copies corrupted in the same byte position cannot be voted on. That is
  detected, counted, and raised, so the caller falls back to a safe default
  rather than acting on a value nobody can vouch for.
* A cell nobody reads is never repaired, and damage accumulates until a second
  hit makes it unrecoverable. `Scrubber` exists for exactly that reason: it
  walks every registered cell on a timer so single-bit damage is corrected long
  before a second event can land on top of it.
"""

from __future__ import annotations

import logging
import struct
import threading
import time

log = logging.getLogger(__name__)

__all__ = ["TMRCell", "TMRInt", "TMRBool", "Scrubber", "TMRUnrecoverable"]


class TMRUnrecoverable(Exception):
    """No majority existed, so no value could be trusted."""


def _vote(copies: list[bytearray]) -> tuple[bytes, bool]:
    """Byte-wise majority vote. Returns (value, repair_needed)."""
    width = len(copies[0])
    out = bytearray(width)
    disagreed = False

    for i in range(width):
        a, b, c = copies[0][i], copies[1][i], copies[2][i]
        if a == b == c:
            out[i] = a
            continue
        disagreed = True
        if a == b or a == c:
            out[i] = a
        elif b == c:
            out[i] = b
        else:
            raise TMRUnrecoverable(
                f"three-way disagreement at byte {i}: "
                f"{a:02x}/{b:02x}/{c:02x}")
    return bytes(out), disagreed


class TMRCell:
    """A fixed-width byte value stored three times.

    Thread-safe: the experiment's command worker and its main loop both touch
    this state, and a torn read during a vote would defeat the point.
    """

    def __init__(self, value: bytes, name: str = "cell"):
        self.name = name
        self.width = len(value)
        self._copies = [bytearray(value) for _ in range(3)]
        self._lock = threading.Lock()
        self.corrections = 0
        self.failures = 0

    def get(self) -> bytes:
        with self._lock:
            return self._get_locked()

    def _get_locked(self) -> bytes:
        try:
            value, damaged = _vote(self._copies)
        except TMRUnrecoverable:
            self.failures += 1
            raise
        if damaged:
            # Repair immediately: a corrected read that leaves the damage in
            # place is a latent double fault waiting for its second hit.
            for copy in self._copies:
                copy[:] = value
            self.corrections += 1
            log.warning("TMR cell %r corrected a bit flip (total %d)",
                        self.name, self.corrections)
        return value

    def set(self, value: bytes) -> None:
        if len(value) != self.width:
            raise ValueError(f"{self.name}: expected {self.width} bytes, "
                             f"got {len(value)}")
        with self._lock:
            for copy in self._copies:
                copy[:] = value

    def scrub(self) -> bool:
        """Force a vote-and-repair pass. Returns True if damage was found."""
        with self._lock:
            before = self.corrections
            try:
                self._get_locked()
            except TMRUnrecoverable:
                log.error("TMR cell %r is unrecoverable", self.name)
                return True
            return self.corrections != before

    def get_or(self, default: bytes) -> bytes:
        """Read, falling back to `default` if no majority exists."""
        try:
            return self.get()
        except TMRUnrecoverable:
            log.error("TMR cell %r unrecoverable; using safe default", self.name)
            return default


class TMRInt(TMRCell):
    """An unsigned integer under TMR. Width is fixed at construction."""

    _FMT = {1: ">B", 2: ">H", 4: ">I", 8: ">Q"}

    def __init__(self, value: int = 0, width: int = 4, name: str = "int"):
        if width not in self._FMT:
            raise ValueError(f"unsupported width {width}")
        self.fmt = self._FMT[width]
        super().__init__(struct.pack(self.fmt, value), name)

    def value(self, default: int = 0) -> int:
        return struct.unpack(self.fmt, self.get_or(
            struct.pack(self.fmt, default)))[0]

    def store(self, value: int) -> None:
        mask = (1 << (self.width * 8)) - 1
        self.set(struct.pack(self.fmt, value & mask))

    def add(self, delta: int = 1) -> int:
        """Read-modify-write under one lock, so counters cannot lose updates."""
        with self._lock:
            mask = (1 << (self.width * 8)) - 1
            try:
                current = struct.unpack(self.fmt, self._get_locked())[0]
            except TMRUnrecoverable:
                current = 0
            new = (current + delta) & mask
            packed = struct.pack(self.fmt, new)
            for copy in self._copies:
                copy[:] = packed
            return new


class TMRBool(TMRCell):
    """A flag under TMR, stored as a full byte per copy.

    Stored as 0x00 / 0xFF rather than 0x00 / 0x01 so that a single bit flip
    can never turn one valid state into the other - it takes four flips in the
    same byte to move between them, and any single flip is outvoted.
    """

    TRUE = 0xFF
    FALSE = 0x00

    def __init__(self, value: bool = False, name: str = "flag"):
        super().__init__(bytes([self.TRUE if value else self.FALSE]), name)

    def value(self, default: bool = False) -> bool:
        raw = self.get_or(bytes([self.TRUE if default else self.FALSE]))[0]
        # Anything that is not cleanly FALSE after voting counts by popcount,
        # so a mangled byte still resolves to the nearer valid state.
        return bin(raw).count("1") >= 4

    def store(self, value: bool) -> None:
        self.set(bytes([self.TRUE if value else self.FALSE]))


class Scrubber:
    """Walks registered cells on a timer, repairing single-bit damage.

    Runs as a daemon thread: it must never keep the process alive, and its
    failure must never be able to stop the payload. Every pass is wrapped.
    """

    def __init__(self, interval_s: float = 30.0):
        self.interval_s = interval_s
        self._cells: list[TMRCell] = []
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.passes = 0
        self.repairs = 0

    def register(self, *cells: TMRCell) -> None:
        self._cells.extend(cells)

    def scrub_once(self) -> int:
        repaired = 0
        for cell in self._cells:
            try:
                if cell.scrub():
                    repaired += 1
            except Exception as exc:                   # noqa: BLE001
                log.error("scrubbing %r failed: %s", cell.name, exc)
        self.passes += 1
        self.repairs += repaired
        return repaired

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="tmr-scrub",
                                        daemon=True)
        self._thread.start()
        log.info("TMR scrubber started over %d cells every %.0f s",
                 len(self._cells), self.interval_s)

    def _run(self) -> None:
        while not self._stop.wait(self.interval_s):
            try:
                n = self.scrub_once()
                if n:
                    log.warning("scrubber repaired %d cell(s)", n)
            except Exception as exc:                   # noqa: BLE001
                log.error("scrubber pass failed: %s", exc)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
