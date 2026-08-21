"""The 1280-byte HRT payload - bulk media downlink.

HRT is the only channel wide enough to carry an image, and DICE gates it: we
may transmit only between an `HRT Go` (0x87) and a `Stop` (0x85/0x86). So a
transfer is not a burst the payload controls but a tap the master opens and
closes, possibly mid-file, possibly for hours. The manager below is built
around that: all state is per-chunk and restartable, and no part of it assumes
a transfer will run to completion in one window.

Payload layout (big-endian), 1280 bytes:

     offset  size  field
          0     2  sub_type      IDLE / MEDIA_INFO / MEDIA_DATA / MEDIA_END
          2     2  flags
          4     4  media_id
          8     4  chunk_index
         12     4  chunk_total
         16     2  data_len      0 .. 1256
         18     2  reserved
         20     4  data_crc32    over data[0:data_len]
         24  1256  data
    1280 total

**Why a per-chunk CRC-32 under the envelope's CRC-16.** A CRC-16 over 1282
bytes is a weak check for a file transfer: it misses a meaningful fraction of
multi-bit error patterns, and a single bad chunk silently corrupts an image
that costs minutes of downlink to send. The per-chunk CRC-32 lets the ground
identify *which* chunks are bad and re-request exactly those through the
command channel, rather than discovering at the end that the whole file's
CRC-32 does not match and starting over.

**The two stops.** Both halt HRT immediately; they differ only in what happens
to a packet already going out. **0x85 Stop** lets it finish, so the ground gets
a whole valid final packet. **0x86 Stop with loss** cuts it short - the
receiver sees a truncated frame, fails its CRC and discards it, which is what
"with loss" names. A file chunk truncated that way is rewound by exactly one so
it goes again; a video frame is dropped instead, because by the time the tap
reopens it is stale.
"""

from __future__ import annotations

import logging
import struct
import threading
import zlib
from dataclasses import dataclass, field

from . import fec

log = logging.getLogger(__name__)

__all__ = [
    "HRT_PAYLOAD_LEN", "HRT_HEADER_LEN", "HRT_CHUNK_DATA", "SubType",
    "build_hrt_payload", "decode_hrt_payload", "Transfer", "TransferManager",
    "FLAG_LAST_CHUNK", "FLAG_RETRANSMIT", "FLAG_PARITY", "FLAG_KEYFRAME",
]

HRT_PAYLOAD_LEN = 1280
HRT_HEADER_LEN = 24
HRT_CHUNK_DATA = HRT_PAYLOAD_LEN - HRT_HEADER_LEN        # 1256

_HDR_FMT = ">HHIIIHHI"


class SubType:
    IDLE = 0x0000
    MEDIA_INFO = 0x0001
    MEDIA_DATA = 0x0002
    MEDIA_END = 0x0003
    #: XOR parity over a group of data chunks; `chunk_index` names the group.
    #: See `radcam/stp/fec.py` - lets the ground rebuild any single chunk lost
    #: per group without asking for a retransmission.
    MEDIA_PARITY = 0x0004
    #: One chunk of a live video frame. `media_id` carries the frame number and
    #: `chunk_total` the chunks in that frame, so the reassembler needs no
    #: prior announcement - unlike a file, a stream has no MEDIA_INFO because
    #: it has no known length and no beginning the receiver is guaranteed to
    #: have seen. Frames are self-describing so a ground station can join at
    #: any point and start decoding from the next keyframe.
    STREAM_DATA = 0x0005


#: Set on the last data chunk of a file, so a reassembler that missed the
#: MEDIA_END packet still knows where the file stops.
FLAG_LAST_CHUNK = 0x0001
#: Set on a chunk sent again in response to a RESEND or a stop-with-loss.
FLAG_RETRANSMIT = 0x0002
#: Set on a MEDIA_PARITY payload, so a reassembler cannot mistake parity for
#: file data even if it ignores sub_type.
FLAG_PARITY = 0x0004
#: Set on every chunk of a stream frame that carries SPS/PPS and an IDR - the
#: frames a late-joining receiver can start decoding from.
FLAG_KEYFRAME = 0x0008


def build_hrt_payload(sub_type: int, media_id: int = 0, chunk_index: int = 0,
                      chunk_total: int = 0, data: bytes = b"",
                      flags: int = 0) -> bytes:
    """Assemble one 1280-byte HRT payload, zero-padded to full length."""
    if len(data) > HRT_CHUNK_DATA:
        raise ValueError(f"HRT chunk data {len(data)} exceeds {HRT_CHUNK_DATA}")

    header = struct.pack(_HDR_FMT, sub_type & 0xFFFF, flags & 0xFFFF,
                         media_id & 0xFFFFFFFF, chunk_index & 0xFFFFFFFF,
                         chunk_total & 0xFFFFFFFF, len(data), 0,
                         zlib.crc32(data) & 0xFFFFFFFF)
    payload = header + data
    return payload + b"\x00" * (HRT_PAYLOAD_LEN - len(payload))


def decode_hrt_payload(payload: bytes) -> dict:
    """Decode an HRT payload. `data_crc_ok` is reported, never enforced."""
    if len(payload) != HRT_PAYLOAD_LEN:
        raise ValueError(f"HRT payload must be {HRT_PAYLOAD_LEN} bytes, "
                         f"got {len(payload)}")

    (sub_type, flags, media_id, chunk_index, chunk_total,
     data_len, _reserved, data_crc) = struct.unpack_from(_HDR_FMT, payload, 0)

    if data_len > HRT_CHUNK_DATA:
        raise ValueError(f"data_len {data_len} exceeds {HRT_CHUNK_DATA}")

    data = payload[HRT_HEADER_LEN:HRT_HEADER_LEN + data_len]
    return {
        "sub_type": sub_type, "flags": flags, "media_id": media_id,
        "chunk_index": chunk_index, "chunk_total": chunk_total,
        "data_len": data_len, "data_crc32": data_crc, "data": data,
        "data_crc_ok": (zlib.crc32(data) & 0xFFFFFFFF) == data_crc,
        "last_chunk": bool(flags & FLAG_LAST_CHUNK),
        "retransmit": bool(flags & FLAG_RETRANSMIT),
        "parity": bool(flags & FLAG_PARITY) or sub_type == SubType.MEDIA_PARITY,
        "keyframe": bool(flags & FLAG_KEYFRAME),
    }


def encode_media_info(media_id: int, size: int, chunk_total: int,
                      file_crc32: int, kind: int = 0, width: int = 0,
                      height: int = 0, created_unix: float = 0.0) -> bytes:
    """The MEDIA_INFO body: everything needed to allocate and verify."""
    return struct.pack(">IQIIBHHd", media_id & 0xFFFFFFFF, size & (2**64 - 1),
                       chunk_total & 0xFFFFFFFF, file_crc32 & 0xFFFFFFFF,
                       kind & 0xFF, width & 0xFFFF, height & 0xFFFF,
                       float(created_unix))


def decode_media_info(data: bytes) -> dict:
    (media_id, size, chunk_total, file_crc32, kind, width, height,
     created) = struct.unpack(">IQIIBHHd", data[:33])
    return {"media_id": media_id, "size": size, "chunk_total": chunk_total,
            "file_crc32": file_crc32, "kind": kind, "width": width,
            "height": height, "created_unix": created}


@dataclass
class Transfer:
    """One media file being pushed down the HRT channel."""

    media_id: int
    data: bytes
    kind: int = 0
    width: int = 0
    height: int = 0
    created_unix: float = 0.0

    #: 0 disables parity. Defaults to one parity chunk per 16 data chunks.
    group_size: int = fec.DEFAULT_GROUP_SIZE

    chunk_next: int = 0
    info_sent: bool = False
    end_sent: bool = False
    parity_sent: set = field(default_factory=set)
    parity_chunks_sent: int = 0
    #: Chunks the ground has explicitly re-requested, sent ahead of new ones.
    resend: list = field(default_factory=list)
    file_crc32: int = 0
    chunks_sent: int = 0

    def __post_init__(self):
        self.file_crc32 = zlib.crc32(self.data) & 0xFFFFFFFF

    @property
    def chunk_total(self) -> int:
        if not self.data:
            return 0
        return (len(self.data) + HRT_CHUNK_DATA - 1) // HRT_CHUNK_DATA

    @property
    def complete(self) -> bool:
        return self.end_sent

    def chunk(self, index: int) -> bytes:
        start = index * HRT_CHUNK_DATA
        return self.data[start:start + HRT_CHUNK_DATA]

    def parity(self, group: int) -> bytes:
        indices = fec.indices_in_group(group, self.group_size, self.chunk_total)
        return fec.parity_of([self.chunk(i) for i in indices], HRT_CHUNK_DATA)

    @property
    def parity_total(self) -> int:
        return fec.group_count(self.chunk_total, self.group_size)

    def parity_group_due(self) -> int | None:
        """The parity chunk owed before the next data chunk goes out.

        Emitted as each group closes rather than all at the end, so a transfer
        that DICE stops part-way still leaves every completed group repairable.
        """
        if self.group_size <= 0:
            return None
        for group in range(self.chunk_next // self.group_size):
            if group not in self.parity_sent:
                return group
        if self.chunk_next >= self.chunk_total:
            for group in range(self.parity_total):
                if group not in self.parity_sent:
                    return group
        return None


class TransferManager:
    """Queues media for HRT downlink and emits one payload at a time.

    Thread-safe, because commands arrive on a worker thread while the main loop
    is pumping packets out.
    """

    def __init__(self, max_queue: int = 16, loss_rewind_chunks: int = 1,
                 retain: int = 2, reload=None,
                 group_size: int = fec.DEFAULT_GROUP_SIZE):
        """`reload(media_id) -> bytes | None` lets a resend outlive retention.

        The ground only learns which chunks were corrupted *after* the file has
        finished arriving, so a resend almost always targets a transfer this
        manager has already retired. Two mechanisms cover that: the last
        `retain` completed transfers are kept whole, and beyond them `reload`
        fetches the bytes again from the media store. Without both, the resend
        path works only in the one case where it is least needed.
        """
        self.max_queue = max_queue
        self.loss_rewind_chunks = loss_rewind_chunks
        self.retain = retain
        self.reload = reload
        self.group_size = group_size
        self._queue: list[Transfer] = []
        self._recent: dict[int, Transfer] = {}
        self._recent_order: list[int] = []
        self._lock = threading.Lock()
        self.completed = 0
        self.aborted = 0
        self.chunks_sent = 0
        self.parity_sent = 0
        self.loss_events = 0
        self.revived = 0

    # -- queue management ------------------------------------------------

    def enqueue(self, media_id: int, data: bytes, kind: int = 0,
                width: int = 0, height: int = 0,
                created_unix: float = 0.0) -> bool:
        """Queue a file. Returns False if the queue is full or it is a dup."""
        with self._lock:
            if len(self._queue) >= self.max_queue:
                log.warning("HRT queue full (%d); refusing media %d",
                            self.max_queue, media_id)
                return False
            if any(t.media_id == media_id for t in self._queue):
                log.info("media %d already queued for HRT", media_id)
                return False
            self._queue.append(Transfer(media_id=media_id, data=data,
                                        kind=kind, width=width, height=height,
                                        created_unix=created_unix,
                                        group_size=self.group_size))
            log.info("queued media %d for HRT: %d bytes, %d chunks",
                     media_id, len(data), self._queue[-1].chunk_total)
            return True

    def request_resend(self, media_id: int, chunks: list[int]) -> int:
        """Mark chunks for re-transmission. Returns how many were accepted.

        Looks in the active queue, then in retained completed transfers, then
        asks `reload` for the bytes. A revived transfer re-enters the queue
        with `info_sent` already true, so the ground gets the chunks it asked
        for and not a fresh MEDIA_INFO it did not.
        """
        with self._lock:
            transfer = self._find_locked(media_id)
            if transfer is None:
                return 0

            valid = [c for c in chunks if 0 <= c < transfer.chunk_total]
            if not valid:
                return 0

            transfer.resend.extend(valid)
            if transfer.end_sent:
                # Re-open so MEDIA_END is emitted again after the resends,
                # keeping the ground's "file finished" signal truthful.
                transfer.end_sent = False
            if transfer not in self._queue:
                self._queue.insert(0, transfer)
                self.revived += 1
                log.info("revived media %d for resend of %d chunk(s)",
                         media_id, len(valid))
            return len(valid)

    def _find_locked(self, media_id: int) -> "Transfer | None":
        for transfer in self._queue:
            if transfer.media_id == media_id:
                return transfer
        transfer = self._recent.get(media_id)
        if transfer is not None:
            return transfer
        if self.reload is None:
            return None
        try:
            data = self.reload(media_id)
        except Exception as exc:                       # noqa: BLE001
            log.error("reload of media %d failed: %s", media_id, exc)
            return None
        if not data:
            return None
        transfer = Transfer(media_id=media_id, data=data,
                            group_size=self.group_size)
        transfer.info_sent = True        # the ground already has MEDIA_INFO
        self._retain_locked(transfer)
        return transfer

    def _retain_locked(self, transfer: "Transfer") -> None:
        self._recent[transfer.media_id] = transfer
        if transfer.media_id in self._recent_order:
            self._recent_order.remove(transfer.media_id)
        self._recent_order.append(transfer.media_id)
        while len(self._recent_order) > self.retain:
            self._recent.pop(self._recent_order.pop(0), None)

    def abort(self, media_id: int) -> bool:
        with self._lock:
            for i, transfer in enumerate(self._queue):
                if transfer.media_id == media_id:
                    del self._queue[i]
                    self._recent.pop(media_id, None)
                    if media_id in self._recent_order:
                        self._recent_order.remove(media_id)
                    self.aborted += 1
                    return True
        return False

    def clear(self) -> int:
        with self._lock:
            n = len(self._queue)
            self._queue.clear()
            self.aborted += n
            return n

    # -- flow control ----------------------------------------------------

    def rewind_one(self) -> None:
        """Send the last data chunk again, because it was truncated in flight.

        Called only when a transmission was actually cut short, so unlike a
        guess about what the master might have lost, this knows exactly which
        chunk did not arrive.
        """
        with self._lock:
            if not self._queue:
                return
            transfer = self._queue[0]
            if transfer.chunk_next > 0:
                transfer.chunk_next -= 1
                log.info("media %d: chunk %d truncated, will resend",
                         transfer.media_id, transfer.chunk_next)

    # -- emission --------------------------------------------------------

    def next_payload(self) -> bytes | None:
        """Return the next 1280-byte payload, or None if nothing is pending."""
        with self._lock:
            while self._queue:
                transfer = self._queue[0]

                if not transfer.info_sent:
                    transfer.info_sent = True
                    return build_hrt_payload(
                        SubType.MEDIA_INFO, transfer.media_id, 0,
                        transfer.chunk_total,
                        encode_media_info(transfer.media_id, len(transfer.data),
                                          transfer.chunk_total,
                                          transfer.file_crc32, transfer.kind,
                                          transfer.width, transfer.height,
                                          transfer.created_unix))

                if transfer.resend:
                    index = transfer.resend.pop(0)
                    self.chunks_sent += 1
                    transfer.chunks_sent += 1
                    return self._data_payload(transfer, index,
                                              FLAG_RETRANSMIT)

                group = transfer.parity_group_due()
                if group is not None:
                    transfer.parity_sent.add(group)
                    transfer.parity_chunks_sent += 1
                    self.parity_sent += 1
                    return build_hrt_payload(
                        SubType.MEDIA_PARITY, transfer.media_id, group,
                        transfer.chunk_total, transfer.parity(group),
                        FLAG_PARITY)

                if transfer.chunk_next < transfer.chunk_total:
                    index = transfer.chunk_next
                    transfer.chunk_next += 1
                    self.chunks_sent += 1
                    transfer.chunks_sent += 1
                    return self._data_payload(transfer, index)

                if not transfer.end_sent:
                    transfer.end_sent = True
                    return build_hrt_payload(
                        SubType.MEDIA_END, transfer.media_id,
                        transfer.chunk_total, transfer.chunk_total,
                        struct.pack(">III", transfer.media_id,
                                    transfer.file_crc32, transfer.chunks_sent))

                # Finished, and no resend arrived before we got back here.
                self._queue.pop(0)
                self._retain_locked(transfer)
                self.completed += 1
                log.info("HRT transfer of media %d complete (%d chunks)",
                         transfer.media_id, transfer.chunks_sent)
            return None

    def _data_payload(self, transfer: Transfer, index: int,
                      extra_flags: int = 0) -> bytes:
        flags = extra_flags
        if index == transfer.chunk_total - 1:
            flags |= FLAG_LAST_CHUNK
        return build_hrt_payload(SubType.MEDIA_DATA, transfer.media_id, index,
                                 transfer.chunk_total, transfer.chunk(index),
                                 flags)

    # -- introspection ---------------------------------------------------

    def status(self) -> dict:
        with self._lock:
            if not self._queue:
                return {"xfer_state": 0, "xfer_media_id": 0,
                        "xfer_chunk_next": 0, "xfer_chunk_total": 0,
                        "xfer_bytes_total": 0, "xfer_file_crc32": 0,
                        "xfer_queue_depth": 0}
            transfer = self._queue[0]
            return {
                "xfer_state": 1,
                "xfer_media_id": transfer.media_id,
                "xfer_chunk_next": transfer.chunk_next,
                "xfer_chunk_total": transfer.chunk_total,
                "xfer_bytes_total": len(transfer.data),
                "xfer_file_crc32": transfer.file_crc32,
                "xfer_queue_depth": len(self._queue),
            }

    @property
    def pending(self) -> int:
        with self._lock:
            return len(self._queue)
