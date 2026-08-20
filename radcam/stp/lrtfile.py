"""File transfer over LRT - the downlink path that is never blocked.

HRT is the fast way to move an image: 1256 bytes per packet, sent as fast as
the link allows. But HRT only flows between an `HRT Go` and a `Stop`, and that
decision belongs to DICE. If the master never opens the tap - because HRT is
allocated to another experiment, or the pass schedule does not permit it - a
payload with only an HRT path has no way to return an image at all.

LRT is the channel that is always polled. So a file can also be pulled through
the LRT file block, one chunk per poll. It is much slower, and deliberately so:

    HRT   1256 B per packet, back-to-back      ~89 kB/s measured
    LRT    512 B per poll, at the master's rate  512 B x poll rate

At 10 LRT polls per second that is about 5 kB/s, so a 3 MB image takes ten
minutes rather than thirty-five seconds. That is a bad way to move a large
image and a perfectly good way to move a thumbnail, a calibration record, a
dose log, or an image that would otherwise never arrive.

**Every chunk is protected twice.** Each carries its own CRC-32 in the LRT file
header, and every `group_size` chunks are followed by an XOR parity chunk, so a
single chunk lost or corrupted per group is reconstructed by the ground without
a retransmission - which matters far more here than on HRT, because a
retransmission costs an entire poll interval. See `radcam/stp/fec.py`.

There is no per-chunk acknowledgement: the payload streams, marks the transfer
complete with the whole-file CRC-32, and the ground asks for whatever did not
survive with `LRT_FILE_RESEND`.
"""

from __future__ import annotations

import logging
import threading
import zlib
from dataclasses import dataclass, field

from . import fec
from . import lrt as L

log = logging.getLogger(__name__)

__all__ = ["LrtFileTransfer", "LrtFileManager", "PARITY_REQUEST_BIT"]

#: In a resend request, this bit marks the index as a parity group number
#: rather than a data chunk index. Parity chunks can be lost too, and a group
#: whose parity is missing has lost its ability to repair itself.
PARITY_REQUEST_BIT = 0x80000000


@dataclass
class LrtFileTransfer:
    """One file being streamed through the LRT file block."""

    media_id: int
    data: bytes
    chunk_size: int = L.FILE_DATA_MAX
    group_size: int = fec.DEFAULT_GROUP_SIZE

    chunk_next: int = 0
    #: Parity groups already emitted, so the schedule is resumable.
    parity_sent: set = field(default_factory=set)
    resend: list = field(default_factory=list)
    complete: bool = False
    file_crc32: int = 0
    chunks_sent: int = 0
    parity_chunks_sent: int = 0

    def __post_init__(self):
        self.chunk_size = max(1, min(self.chunk_size, L.FILE_DATA_MAX))
        self.file_crc32 = zlib.crc32(self.data) & 0xFFFFFFFF

    @property
    def chunk_total(self) -> int:
        if not self.data:
            return 0
        return (len(self.data) + self.chunk_size - 1) // self.chunk_size

    def chunk(self, index: int) -> bytes:
        start = index * self.chunk_size
        return self.data[start:start + self.chunk_size]

    def parity(self, group: int) -> bytes:
        indices = fec.indices_in_group(group, self.group_size, self.chunk_total)
        return fec.parity_of([self.chunk(i) for i in indices], self.chunk_size)

    @property
    def parity_total(self) -> int:
        return fec.group_count(self.chunk_total, self.group_size)

    def parity_group_due(self) -> int | None:
        """The parity chunk owed before the next data chunk goes out.

        Emitted as soon as its group is complete rather than all at the end, so
        that a transfer interrupted part-way still leaves the ground able to
        repair the groups it did receive.
        """
        if self.group_size <= 0:
            return None
        finished = self.chunk_next // self.group_size
        for group in range(finished):
            if group not in self.parity_sent:
                return group
        # The final, partial group once every data chunk has gone.
        if self.chunk_next >= self.chunk_total:
            last = fec.group_count(self.chunk_total, self.group_size)
            for group in range(last):
                if group not in self.parity_sent:
                    return group
        return None


class LrtFileManager:
    """Serves one LRT file transfer at a time.

    One at a time is deliberate: the LRT file block holds a single chunk, so
    interleaving two transfers would halve each one's rate and complicate
    reassembly for no gain. A new start replaces the current transfer.
    """

    def __init__(self, resolve=None, default_group_size: int = fec.DEFAULT_GROUP_SIZE):
        #: `resolve(media_id) -> bytes | None`. Lets a transfer be started for
        #: anything addressable, not only stored media - an oversized command
        #: response is served the same way.
        self.resolve = resolve
        self.default_group_size = default_group_size
        self._transfer: LrtFileTransfer | None = None
        self._lock = threading.Lock()
        self.started = 0
        self.completed = 0
        self.chunks_sent = 0
        self.parity_sent = 0
        self.resends = 0

    # -- control ---------------------------------------------------------

    def start(self, media_id: int, data: bytes | None = None,
              chunk_size: int = L.FILE_DATA_MAX,
              group_size: int | None = None) -> bool:
        if data is None:
            if self.resolve is None:
                return False
            try:
                data = self.resolve(media_id)
            except Exception as exc:                   # noqa: BLE001
                log.error("resolving media %d failed: %s", media_id, exc)
                return False
        if data is None:
            return False

        group = self.default_group_size if group_size is None else group_size
        with self._lock:
            self._transfer = LrtFileTransfer(
                media_id=media_id, data=data, chunk_size=chunk_size,
                group_size=max(0, group))
            self.started += 1
            transfer = self._transfer
        log.info("LRT file transfer of media %d started: %d bytes, %d chunks "
                 "of %d, parity group %d", media_id, len(data),
                 transfer.chunk_total, transfer.chunk_size, transfer.group_size)
        return True

    def stop(self) -> bool:
        with self._lock:
            had = self._transfer is not None
            self._transfer = None
        return had

    def request_resend(self, media_id: int, indices: list[int]) -> int:
        """Queue chunks to go again. Parity groups are flagged by the high bit."""
        with self._lock:
            transfer = self._transfer
            if transfer is None or transfer.media_id != media_id:
                return 0

            accepted = 0
            for raw in indices:
                if raw & PARITY_REQUEST_BIT:
                    group = raw & ~PARITY_REQUEST_BIT
                    if 0 <= group < transfer.parity_total:
                        transfer.resend.append(raw)
                        accepted += 1
                elif 0 <= raw < transfer.chunk_total:
                    transfer.resend.append(raw)
                    accepted += 1

            if accepted:
                # Re-open a finished transfer, or the resend would never be
                # emitted - and the ground only knows what is missing once the
                # transfer has finished.
                transfer.complete = False
                self.resends += accepted
            return accepted

    # -- emission --------------------------------------------------------

    def next_block(self) -> dict:
        """The `file_*` fields for the next LRT payload.

        Called once per LRT reply. Returns an idle block when there is nothing
        to send, so the caller can merge it unconditionally.
        """
        with self._lock:
            transfer = self._transfer
            if transfer is None:
                return {"file_state": L.FILE_IDLE}

            base = {
                "file_media_id": transfer.media_id,
                "file_chunk_total": transfer.chunk_total,
                "file_size": len(transfer.data),
                "file_crc32": transfer.file_crc32,
                "file_fec_group": transfer.group_size,
            }

            if transfer.resend:
                raw = transfer.resend.pop(0)
                block = self._emit(transfer, raw, retransmit=True)
                block.update(base)
                return block

            group = transfer.parity_group_due()
            if group is not None:
                transfer.parity_sent.add(group)
                transfer.parity_chunks_sent += 1
                self.parity_sent += 1
                block = self._emit(transfer, group | PARITY_REQUEST_BIT)
                block.update(base)
                return block

            if transfer.chunk_next < transfer.chunk_total:
                index = transfer.chunk_next
                transfer.chunk_next += 1
                transfer.chunks_sent += 1
                self.chunks_sent += 1
                block = self._emit(transfer, index)
                block.update(base)
                return block

            if not transfer.complete:
                transfer.complete = True
                self.completed += 1
                log.info("LRT file transfer of media %d complete: %d data + "
                         "%d parity chunks", transfer.media_id,
                         transfer.chunks_sent, transfer.parity_chunks_sent)

            # Stay COMPLETE rather than going idle: the ground needs to see the
            # whole-file CRC-32 to know what to verify against, and it may not
            # have been polling at the instant the last chunk went out.
            block = {"file_state": L.FILE_COMPLETE, "file_data": b"",
                     "file_chunk_index": transfer.chunk_total}
            block.update(base)
            return block

    def _emit(self, transfer: LrtFileTransfer, raw: int,
              retransmit: bool = False) -> dict:
        flags = L.FILE_FLAG_RETRANSMIT if retransmit else 0

        if raw & PARITY_REQUEST_BIT:
            group = raw & ~PARITY_REQUEST_BIT
            return {"file_state": L.FILE_ACTIVE,
                    "file_flags": flags | L.FILE_FLAG_PARITY,
                    "file_chunk_index": group,
                    "file_data": transfer.parity(group)}

        if raw == transfer.chunk_total - 1:
            flags |= L.FILE_FLAG_LAST_DATA
        return {"file_state": L.FILE_ACTIVE, "file_flags": flags,
                "file_chunk_index": raw, "file_data": transfer.chunk(raw)}

    # -- introspection ---------------------------------------------------

    def status(self) -> dict:
        with self._lock:
            transfer = self._transfer
            if transfer is None:
                return {"lrt_file_active": False, "lrt_file_media_id": 0,
                        "lrt_file_progress": 0, "lrt_file_total": 0}
            return {"lrt_file_active": not transfer.complete,
                    "lrt_file_media_id": transfer.media_id,
                    "lrt_file_progress": transfer.chunk_next,
                    "lrt_file_total": transfer.chunk_total}

    @property
    def active(self) -> bool:
        with self._lock:
            return self._transfer is not None
