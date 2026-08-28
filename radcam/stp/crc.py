"""CRC-16 for the STP/DICE RS-422 link.

The supplied ICD excerpts state only that the CRC is 16 bits wide and, for the
HRT classes, that it covers everything after the last sync byte and before the
CRC itself. Polynomial, initial value, reflection, final XOR and the stored
byte order are *not* defined by those tables. They therefore live in a
`Crc16Params` the mission sets, rather than being baked into the packet code:
if the authoritative ICD contradicts us, it is a config change and not a
rewrite.

The mission has confirmed **CRC-16/CCITT-FALSE** (poly 0x1021, init 0xFFFF, no
reflection, no final XOR), which is the default here.

**Verified against the flight computer's own implementation.** DICE computes
packet checksums with a precomputed CRC-16/CCITT table over `data[4:length]`,
where the caller sets `length` to the CRC field's offset. That source is
transcribed verbatim in `tests/test_stp_crc_flight.py` and asserted to agree
with this module over every packet class on the link, on random content and on
the degenerate all-zero and all-ones cases where a wrong initial value would
show. Their table is generated from polynomial 0x1021, MSB-first, and matches
one generated here entry for entry.

That test is not this module checked against itself in another form - it is
this module checked against the other end of the link, which is the only
comparison that decides whether packets are accepted.

`solve()` exists for the case where that turns out to be wrong. Given captured
known-good packets it searches the standard parameter space - including the
coverage range and the stored byte order - and reports every variant that
reproduces the stored value. That is far cheaper than guessing against a
flight computer.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

__all__ = [
    "Crc16Params", "CCITT_FALSE", "XMODEM", "ARC", "MODBUS", "CATALOG",
    "crc16", "solve", "CrcCandidate",
]


def _reflect(value: int, width: int) -> int:
    out = 0
    for _ in range(width):
        out = (out << 1) | (value & 1)
        value >>= 1
    return out


_REFLECT8 = [_reflect(i, 8) for i in range(256)]

#: Byte-table cache, keyed by polynomial. The table depends only on the
#: polynomial: reflection is handled by reflecting the input byte and the
#: final register, which keeps one table serving every variant of a poly.
_TABLES: dict[int, list[int]] = {}


def _table(poly: int) -> list[int]:
    table = _TABLES.get(poly)
    if table is not None:
        return table
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFFFF if crc & 0x8000 \
                else (crc << 1) & 0xFFFF
        table.append(crc)
    _TABLES[poly] = table
    return table


@dataclass(frozen=True)
class Crc16Params:
    """A CRC-16 in the Rocksoft model.

    `big_endian_store` describes how the 16-bit result is written into the
    packet, which is a separate question from the algorithm and is likewise
    undefined by the supplied tables.
    """

    name: str = "CRC-16/CCITT-FALSE"
    poly: int = 0x1021
    init: int = 0xFFFF
    reflect_in: bool = False
    reflect_out: bool = False
    xor_out: int = 0x0000
    big_endian_store: bool = True

    def compute(self, data: bytes) -> int:
        table = _table(self.poly)
        crc = self.init
        if self.reflect_in:
            for byte in data:
                crc = ((crc << 8) & 0xFFFF) ^ table[((crc >> 8) ^ _REFLECT8[byte]) & 0xFF]
        else:
            for byte in data:
                crc = ((crc << 8) & 0xFFFF) ^ table[((crc >> 8) ^ byte) & 0xFF]
        if self.reflect_out:
            crc = _reflect(crc, 16)
        return (crc ^ self.xor_out) & 0xFFFF

    def pack(self, value: int) -> bytes:
        return (value & 0xFFFF).to_bytes(2, "big" if self.big_endian_store else "little")

    def unpack(self, raw: bytes) -> int:
        return int.from_bytes(raw, "big" if self.big_endian_store else "little")


CCITT_FALSE = Crc16Params()
XMODEM = Crc16Params("CRC-16/XMODEM", 0x1021, 0x0000, False, False, 0x0000)
ARC = Crc16Params("CRC-16/ARC", 0x8005, 0x0000, True, True, 0x0000)
MODBUS = Crc16Params("CRC-16/MODBUS", 0x8005, 0xFFFF, True, True, 0x0000)
_KERMIT = Crc16Params("CRC-16/KERMIT", 0x1021, 0x0000, True, True, 0x0000)
_IBM3740 = CCITT_FALSE
_GENIBUS = Crc16Params("CRC-16/GENIBUS", 0x1021, 0xFFFF, False, False, 0xFFFF)
_MCRF4XX = Crc16Params("CRC-16/MCRF4XX", 0x1021, 0xFFFF, True, True, 0x0000)
_X25 = Crc16Params("CRC-16/X-25", 0x1021, 0xFFFF, True, True, 0xFFFF)
_USB = Crc16Params("CRC-16/USB", 0x8005, 0xFFFF, True, True, 0xFFFF)
_CDMA2000 = Crc16Params("CRC-16/CDMA2000", 0xC867, 0xFFFF, False, False, 0x0000)
_DDS110 = Crc16Params("CRC-16/DDS-110", 0x8005, 0x800D, False, False, 0x0000)
_EN13757 = Crc16Params("CRC-16/EN-13757", 0x3D65, 0x0000, False, False, 0xFFFF)
_T10DIF = Crc16Params("CRC-16/T10-DIF", 0x8BB7, 0x0000, False, False, 0x0000)
_DECTR = Crc16Params("CRC-16/DECT-R", 0x0589, 0x0000, False, False, 0x0001)
_MAXIM = Crc16Params("CRC-16/MAXIM", 0x8005, 0x0000, True, True, 0xFFFF)

#: Every variant `solve()` will try. Ordered with the likeliest first so a
#: report reads sensibly when several candidates survive.
CATALOG: tuple[Crc16Params, ...] = (
    CCITT_FALSE, XMODEM, _KERMIT, ARC, MODBUS, _X25, _MCRF4XX, _GENIBUS,
    _USB, _MAXIM, _CDMA2000, _DDS110, _EN13757, _T10DIF, _DECTR,
)


def crc16(data: bytes, params: Crc16Params = CCITT_FALSE) -> int:
    """Compute a CRC-16 over `data` under `params`."""
    return params.compute(data)


@dataclass(frozen=True)
class CrcCandidate:
    """A parameter set that reproduced every supplied sample."""

    params: Crc16Params
    start: int
    #: Offset of the CRC field relative to the end of the packet, so it reads
    #: the same for packets of different lengths.
    crc_offset_from_end: int

    def describe(self) -> str:
        order = "big" if self.params.big_endian_store else "little"
        return (f"{self.params.name}: poly=0x{self.params.poly:04X} "
                f"init=0x{self.params.init:04X} "
                f"refin={self.params.reflect_in} refout={self.params.reflect_out} "
                f"xorout=0x{self.params.xor_out:04X} store={order}-endian, "
                f"covers packet[{self.start}:len-{self.crc_offset_from_end}]")


def solve(packets: Sequence[bytes],
          crc_offset_from_end: int = 2,
          starts: Iterable[int] = (0, 4, 6),
          catalog: Sequence[Crc16Params] = CATALOG) -> list[CrcCandidate]:
    """Recover the CRC parameters from captured known-good packets.

    Every combination of catalogue entry, coverage start offset and stored
    byte order is tried against every packet; only those matching *all* of
    them are returned. Supply at least three packets - a single sample admits
    coincidental matches at roughly one in 65536 per candidate, and the
    catalogue is large enough that one sample regularly leaves several.
    """
    if not packets:
        raise ValueError("need at least one packet to solve against")

    survivors: list[CrcCandidate] = []
    for base in catalog:
        for big in (True, False):
            params = Crc16Params(base.name, base.poly, base.init,
                                 base.reflect_in, base.reflect_out,
                                 base.xor_out, big)
            for start in starts:
                if all(_matches(p, params, start, crc_offset_from_end)
                       for p in packets):
                    survivors.append(CrcCandidate(params, start,
                                                  crc_offset_from_end))
    return survivors


def _matches(packet: bytes, params: Crc16Params, start: int,
             crc_offset_from_end: int) -> bool:
    end = len(packet) - crc_offset_from_end
    if end <= start or end + 2 > len(packet):
        return False
    stored = params.unpack(packet[end:end + 2])
    return params.compute(packet[start:end]) == stored
