"""Triple modular redundancy for data that must survive bit flips.

Anything the system cannot recompute after a reset - the dosimeter calibration
above all - is stored three times, each copy carrying its own CRC32. Reads
verify every copy, majority-vote byte by byte when they disagree, and rewrite
whichever copies were damaged.

A single flipped bit in one copy is therefore corrected and repaired silently.
Two copies damaged in the *same* byte position is not recoverable by voting, but
is still detected via CRC so the caller can fall back rather than trust
corrupted data.
"""

from __future__ import annotations

import json
import logging
import os
import zlib
from pathlib import Path
from typing import Any

log = logging.getLogger(__name__)

MAGIC = b"RADCAM-TMR1\n"
DEFAULT_COPIES = 3


class TMRError(Exception):
    """Raised when no trustworthy copy of the data could be recovered."""


def _encode(payload: bytes) -> bytes:
    """Frame a payload as MAGIC + CRC32 (8 hex chars) + newline + payload."""
    return MAGIC + b"%08x\n" % (zlib.crc32(payload) & 0xFFFFFFFF) + payload


def _decode(blob: bytes) -> bytes | None:
    """Return the payload if the frame is intact, else None."""
    if not blob.startswith(MAGIC):
        return None
    rest = blob[len(MAGIC):]
    nl = rest.find(b"\n")
    if nl != 8:      # exactly 8 hex digits of CRC32 precede the newline
        return None
    try:
        want = int(rest[:nl], 16)
    except ValueError:
        return None
    payload = rest[nl + 1:]
    if (zlib.crc32(payload) & 0xFFFFFFFF) != want:
        return None
    return payload


def _majority_vote(blobs: list[bytes]) -> bytes | None:
    """Byte-wise majority vote across copies of equal length."""
    lengths = [len(b) for b in blobs]
    # Length itself is voted on: a copy of the wrong length is discarded.
    best_len = max(set(lengths), key=lengths.count)
    candidates = [b for b in blobs if len(b) == best_len]
    if len(candidates) < 2:
        return None

    out = bytearray(best_len)
    for i in range(best_len):
        counts: dict[int, int] = {}
        for b in candidates:
            counts[b[i]] = counts.get(b[i], 0) + 1
        byte, n = max(counts.items(), key=lambda kv: kv[1])
        if n < 2:
            return None  # no majority at this position
        out[i] = byte
    return bytes(out)


def _atomic_write(path: Path, data: bytes) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "wb") as fh:
        fh.write(data)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)
    # Durability of the rename itself.
    dir_fd = os.open(path.parent, os.O_DIRECTORY)
    try:
        os.fsync(dir_fd)
    finally:
        os.close(dir_fd)


class TMRStore:
    """A JSON document stored redundantly across `copies` files."""

    def __init__(self, path: str | os.PathLike, copies: int = DEFAULT_COPIES):
        self.base = Path(path)
        self.copies = copies
        self.base.parent.mkdir(parents=True, exist_ok=True)

    def _paths(self) -> list[Path]:
        return [
            self.base.with_name(f"{self.base.name}.{i}")
            for i in range(self.copies)
        ]

    def exists(self) -> bool:
        return any(p.exists() for p in self._paths())

    def write(self, obj: Any) -> None:
        payload = json.dumps(obj, sort_keys=True, indent=2).encode()
        frame = _encode(payload)
        for p in self._paths():
            _atomic_write(p, frame)

    def read(self) -> Any:
        """Read the document, repairing any damaged copies in place."""
        raw: dict[Path, bytes] = {}
        for p in self._paths():
            try:
                raw[p] = p.read_bytes()
            except OSError as exc:
                log.warning("TMR copy %s unreadable: %s", p, exc)

        if not raw:
            raise TMRError(f"no copies of {self.base} exist")

        good = {p: payload for p, b in raw.items()
                if (payload := _decode(b)) is not None}

        if good:
            payloads = list(good.values())
            winner = max(set(payloads), key=payloads.count)
            if len(set(payloads)) > 1:
                log.warning("TMR copies of %s disagree; using majority",
                            self.base)
        else:
            # Every CRC failed - try to reconstruct by voting on raw frames.
            log.error("all TMR copies of %s failed CRC; attempting vote",
                      self.base)
            voted = _majority_vote(list(raw.values()))
            winner = _decode(voted) if voted else None
            if winner is None:
                raise TMRError(f"{self.base}: no recoverable copy")
            log.warning("TMR reconstructed %s by majority vote", self.base)

        # Repair anything that does not match the winning payload.
        frame = _encode(winner)
        for p in self._paths():
            if raw.get(p) != frame:
                log.warning("TMR repairing copy %s", p)
                try:
                    _atomic_write(p, frame)
                except OSError as exc:
                    log.error("TMR could not repair %s: %s", p, exc)

        return json.loads(winner)


def vote3(a: Any, b: Any, c: Any) -> Any:
    """Majority vote over three in-memory values.

    Used for readings that are sampled three times before being acted on or
    transmitted. Returns the value at least two of them agree on, otherwise the
    median, which for numeric samples is the safest single choice.
    """
    if a == b or a == c:
        return a
    if b == c:
        return b
    try:
        return sorted((a, b, c))[1]
    except TypeError:
        return a
