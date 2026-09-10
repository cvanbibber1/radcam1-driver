"""Packet reassembly from the RS-422 byte stream.

This is the layer that has to survive a shared bus. Unlike the old
point-to-point link, the wire carries traffic for other experiments, plus H&S
and File Transfer classes whose lengths this implementation does not know. The
receiver therefore never assumes it is aligned to a packet boundary: it hunts
for the sync pattern, decides the length from the type byte, and validates.

Three rules make it robust rather than merely correct:

* **Unknown types are resynchronised past, not consumed.** If byte 10 is not a
  type we handle, its length is unknown, so skipping "the rest of the packet"
  would be a guess that could swallow the start of a packet meant for us. The
  reader advances past this sync occurrence and hunts for the next one.

* **A CRC failure also resynchronises rather than consuming.** A bad CRC means
  something is wrong, and that something may be the length assumption itself -
  a corrupted type byte on an unrelated packet looks exactly like a short
  request. Consuming 120 bytes on that basis can destroy a good packet that
  followed. Rescanning costs only CPU.

* **The buffer is bounded.** A stuck or babbling transmitter must not be able
  to exhaust memory; past the cap the oldest bytes are dropped and counted.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

from .packets import (
    DEFAULT_WIRE, PacketError, SHORT_PACKET_TYPE_OFFSET, Wire,
    decode_command, decode_short_request, rx_length_for_type,
    COMMAND_PACKET_SIZE, SHORT_REQUEST_SIZE, PacketType,
    IGNORED_RX_TYPES,
)

log = logging.getLogger(__name__)

__all__ = ["PacketReader", "RxStats"]

#: Bytes that must be present past a sync before the type byte can be read.
_MIN_HEADER = SHORT_PACKET_TYPE_OFFSET + 2      # sync + coarse + fine + type + id


@dataclass
class RxStats:
    """Counters the ground can use to judge link health. All monotonic."""

    good: int = 0
    bad_crc: int = 0
    bad_format: int = 0
    not_for_us: int = 0
    unknown_type: int = 0
    resyncs: int = 0
    dropped_bytes: int = 0
    #: Correctly framed packets addressed to us that need no action - health
    #: and status, and the flight computer's 16-byte packet. Counted apart
    #: from unknown_type so that a genuinely unrecognised packet still shows.
    ignored: int = 0

    def as_dict(self) -> dict[str, int]:
        return {
            "good": self.good, "bad_crc": self.bad_crc,
            "bad_format": self.bad_format, "not_for_us": self.not_for_us,
            "unknown_type": self.unknown_type, "resyncs": self.resyncs,
            "dropped_bytes": self.dropped_bytes,
            "ignored": self.ignored,
        }


class PacketReader:
    """Turns an arbitrarily chunked byte stream into decoded packets.

    Only packets addressed to `wire.target_id` are returned. Packets for other
    experiments are counted and dropped here rather than higher up, so no
    application code can act on another target's command by accident.
    """

    def __init__(self, wire: Wire = DEFAULT_WIRE, max_buffer: int = 16384):
        self.wire = wire
        self.max_buffer = max_buffer
        self.stats = RxStats()
        self._buf = bytearray()

    # -- public ----------------------------------------------------------

    def feed(self, data: bytes) -> list:
        """Absorb bytes; return every complete packet addressed to us."""
        if data:
            self._buf.extend(data)
            self._trim()
        return self._drain()

    def reset(self) -> None:
        """Discard partial state - used after transmitting, to drop any echo."""
        self._buf.clear()

    @property
    def pending_bytes(self) -> int:
        return len(self._buf)

    # -- internals -------------------------------------------------------

    def _trim(self) -> None:
        if len(self._buf) <= self.max_buffer:
            return
        excess = len(self._buf) - self.max_buffer
        del self._buf[:excess]
        self.stats.dropped_bytes += excess
        log.warning("RX buffer overflow, dropped %d bytes", excess)

    def _drain(self) -> list:
        sync = self.wire.sync_bytes
        out = []

        while True:
            start = self._buf.find(sync)
            if start < 0:
                # No sync in the buffer at all. Keep only the last 3 bytes, in
                # case a sync pattern straddles this read and the next.
                if len(self._buf) > len(sync) - 1:
                    drop = len(self._buf) - (len(sync) - 1)
                    del self._buf[:drop]
                    self.stats.dropped_bytes += drop
                return out

            if start:
                # Bytes before the sync belong to traffic we could not parse.
                del self._buf[:start]
                self.stats.dropped_bytes += start

            if len(self._buf) < _MIN_HEADER:
                return out               # need the type byte to proceed

            packet_type = self._buf[SHORT_PACKET_TYPE_OFFSET]
            length = rx_length_for_type(packet_type)

            if length is None:
                # A type we have no length for - a corrupted type byte, or a
                # packet class nobody has told us about. We do not know how
                # long it is, so step past this sync and hunt again. Every one
                # of these costs a resync, which is why any type seen
                # regularly on the wire belongs in RX_LENGTHS even when we
                # have nothing to do with it.
                self.stats.unknown_type += 1
                self._skip_one_sync()
                continue

            if len(self._buf) < length:
                return out               # wait for the rest of the packet

            if packet_type in IGNORED_RX_TYPES:
                # Addressed to us, correctly framed, and requiring no action.
                # Consume it whole so the next packet starts at a sync we
                # trust - the CRC is still checked first, because a corrupted
                # type byte that lands on one of these values must not be
                # allowed to eat the bytes of a real packet behind it.
                candidate = bytes(self._buf[:length])
                if not self.wire.check_crc(candidate, length - 2):
                    self.stats.bad_crc += 1
                    self._skip_one_sync()
                    continue
                del self._buf[:length]
                self.stats.ignored += 1
                continue

            candidate = bytes(self._buf[:length])
            packet = self._decode(candidate, packet_type)

            if packet is None:
                self._skip_one_sync()
                continue

            del self._buf[:length]

            if packet.target_id != self.wire.target_id:
                self.stats.not_for_us += 1
                continue

            self.stats.good += 1
            out.append(packet)

    def _skip_one_sync(self) -> None:
        """Advance past the current sync so the next find() makes progress."""
        del self._buf[:len(self.wire.sync_bytes)]
        self.stats.dropped_bytes += len(self.wire.sync_bytes)
        self.stats.resyncs += 1

    def _decode(self, candidate: bytes, packet_type: int):
        try:
            if packet_type == PacketType.COMMAND:
                return decode_command(candidate, self.wire)
            return decode_short_request(candidate, self.wire)
        except PacketError as exc:
            text = str(exc)
            if "CRC" in text:
                self.stats.bad_crc += 1
            else:
                self.stats.bad_format += 1
            log.debug("dropping %d-byte candidate: %s", len(candidate), exc)
            return None
