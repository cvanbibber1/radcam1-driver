"""Forward error correction for chunked transfers: XOR parity groups.

Every chunk already carries a CRC-32, which *detects* corruption. Detection on
its own means a retransmission, and a retransmission means another round trip
with DICE - which on the LRT path costs a whole poll interval, and on the HRT
path needs the master to reopen a tap it may not reopen for a long time. So the
transfers here also carry enough redundancy to *correct* a limited amount of
damage without asking for anything back.

The scheme is XOR parity over groups of `group_size` data chunks: after each
group, one parity chunk is emitted whose bytes are the XOR of every data chunk
in that group. If exactly one chunk of a group fails to arrive, or arrives with
a bad CRC-32, the receiver reconstructs it by XOR-ing the survivors with the
parity chunk. Two losses in the same group are not recoverable and fall back to
an explicit RESEND.

Why this rather than Reed-Solomon: RS would correct more, but it needs a field
implementation and lookup tables on a flight machine where the rule is stdlib
only, and its benefit is concentrated in the multi-error case that RESEND
already handles correctly. XOR parity is a few lines, has no failure mode of
its own, and covers the case that actually dominates on a UART link - a single
chunk lost to a burst of noise. The cost is one chunk of bandwidth per group:
6.25% at the default group size of 16.

Group size is a tunable, not a constant: a noisy link wants smaller groups
(more overhead, more correction), a clean one wants larger or none at all.
`group_size = 0` disables parity entirely.
"""

from __future__ import annotations

import zlib
from dataclasses import dataclass

__all__ = [
    "DEFAULT_GROUP_SIZE", "parity_of", "group_of", "indices_in_group",
    "group_count", "recover_missing", "FecStats",
]

#: One parity chunk per 16 data chunks: 6.25% overhead, corrects any single
#: chunk lost per group. Chosen as the point where overhead is small enough to
#: leave on permanently.
DEFAULT_GROUP_SIZE = 16


def parity_of(chunks: list[bytes], size: int) -> bytes:
    """XOR every chunk together, zero-padded to `size`.

    Zero padding is what makes a short final chunk safe: XOR with zero is the
    identity, so a group whose last chunk is short still reconstructs exactly,
    provided the receiver truncates to the length it was told.
    """
    if size <= 0:
        return b""
    accumulator = 0
    for chunk in chunks:
        if len(chunk) > size:
            raise ValueError(f"chunk of {len(chunk)} exceeds parity size {size}")
        accumulator ^= int.from_bytes(chunk.ljust(size, b"\x00"), "big")
    return accumulator.to_bytes(size, "big")


def group_of(chunk_index: int, group_size: int) -> int:
    """Which parity group a data chunk belongs to."""
    if group_size <= 0:
        return 0
    return chunk_index // group_size


def indices_in_group(group: int, group_size: int, chunk_total: int) -> list[int]:
    """The data chunk indices a given parity chunk covers."""
    if group_size <= 0:
        return []
    start = group * group_size
    return list(range(start, min(start + group_size, chunk_total)))


def group_count(chunk_total: int, group_size: int) -> int:
    """How many parity chunks a transfer of `chunk_total` data chunks emits."""
    if group_size <= 0 or chunk_total <= 0:
        return 0
    return (chunk_total + group_size - 1) // group_size


def recover_missing(present: dict[int, bytes], parity: bytes,
                    indices: list[int], size: int) -> tuple[int, bytes] | None:
    """Reconstruct the one chunk of `indices` absent from `present`.

    Returns `(index, data)`, or None when nothing is missing (nothing to do) or
    more than one is (beyond what a single parity chunk can express). The
    returned chunk is full-width; the caller truncates it to the length the
    transfer advertised, because parity cannot carry a length.
    """
    missing = [i for i in indices if i not in present]
    if len(missing) != 1:
        return None

    survivors = [present[i] for i in indices if i in present]
    recovered = parity_of(survivors + [parity], size)
    return missing[0], recovered


@dataclass
class FecStats:
    """What the parity actually bought, for the ground to see."""

    groups: int = 0
    recovered: int = 0
    unrecoverable: int = 0

    def as_dict(self) -> dict[str, int]:
        return {"fec_groups": self.groups, "fec_recovered": self.recovered,
                "fec_unrecoverable": self.unrecoverable}


def verify_and_repair(chunks: dict[int, bytes], parities: dict[int, bytes],
                      chunk_total: int, group_size: int, size: int,
                      stats: FecStats | None = None) -> list[int]:
    """Fill gaps in `chunks` from `parities`, in place.

    Returns the indices still missing afterwards - the ones that need an
    explicit RESEND. This is the receive-side entry point, used by the ground
    tooling and the tests; the payload never runs it.
    """
    stats = stats or FecStats()
    for group in sorted(parities):
        indices = indices_in_group(group, group_size, chunk_total)
        if not indices:
            continue
        stats.groups += 1
        result = recover_missing(chunks, parities[group], indices, size)
        if result is None:
            continue
        index, data = result
        chunks[index] = data
        stats.recovered += 1

    still_missing = [i for i in range(chunk_total) if i not in chunks]
    stats.unrecoverable += len(still_missing)
    return still_missing
