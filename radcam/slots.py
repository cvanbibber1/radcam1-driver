"""Numbered storage slots for images and video.

The media store underneath assigns an id to every capture, counting upwards
forever. That is fine for a system somebody is watching, and wrong for one
commanded by a fixed set of hex strings from the ground: the id of the next
capture depends on how many captures came before it, so a canned "download the
image I just took" command cannot exist.

Slots fix the address. There are `count` of them, numbered from zero, and each
holds at most one image or one video. Capture into slot 3, download slot 3,
delete slot 3 - the same three commands every time, whatever happened
previously. Deleting is what frees a slot for the next experiment, so storage
is managed explicitly by the ground rather than filling silently.

The index is stored triple-redundantly through `radcam.tmr`, because the slot
table is the one piece of state that makes the stored bytes findable: lose it
and there are files on disk that nothing can name. The payload bytes themselves
are not triplicated - they are far too large, and a corrupted image is
recoverable by taking another one.

A slot's CRC-32 is computed when it is written and checked on read, so the
ground learns that a file rotted in storage rather than discovering it after
spending minutes of downlink on it.
"""

from __future__ import annotations

import logging
import os
import time
import zlib
from dataclasses import asdict, dataclass
from pathlib import Path

from .tmr import TMRStore

log = logging.getLogger(__name__)

__all__ = ["Slot", "SlotStore", "SLOT_EMPTY", "SLOT_IMAGE", "SLOT_VIDEO",
           "SLOT_RECORDING", "SLOT_ERROR", "DEFAULT_SLOTS", "SLOT_MEDIA_BASE"]

SLOT_EMPTY = 0
SLOT_IMAGE = 1
SLOT_VIDEO = 2
SLOT_RECORDING = 3        # a recording is in progress into this slot
SLOT_ERROR = 4            # written, but the stored bytes fail their CRC

DEFAULT_SLOTS = 16
DEFAULT_DIR = "/var/lib/radcam/slots"

#: HRT transfers of a slot use this id space, so a slot download can never be
#: confused with a legacy media id on the wire. The low byte is the slot.
SLOT_MEDIA_BASE = 0x51000000


@dataclass
class Slot:
    index: int
    kind: int = SLOT_EMPTY
    size: int = 0
    width: int = 0
    height: int = 0
    created_unix: float = 0.0
    crc32: int = 0
    duration_s: float = 0.0

    @property
    def occupied(self) -> bool:
        return self.kind != SLOT_EMPTY

    @property
    def media_id(self) -> int:
        return SLOT_MEDIA_BASE | (self.index & 0xFF)


class SlotStore:
    """A fixed table of storage slots, with a redundantly stored index."""

    def __init__(self, directory: str | os.PathLike = DEFAULT_DIR,
                 count: int = DEFAULT_SLOTS):
        self.dir = Path(directory)
        self.count = count
        self.dir.mkdir(parents=True, exist_ok=True)
        self._index = TMRStore(self.dir / "index.json")
        self._slots: dict[int, Slot] = {}
        self._recording: int | None = None
        self._record_started = 0.0
        self._record_limit = 0.0
        self._load()

    # -- persistence -----------------------------------------------------

    def _load(self) -> None:
        self._slots = {i: Slot(index=i) for i in range(self.count)}
        if not self._index.exists():
            return
        try:
            raw = self._index.read()
        except Exception as exc:                       # noqa: BLE001
            log.error("slot index unreadable, starting empty: %s", exc)
            return
        for entry in (raw or {}).get("slots", []):
            index = int(entry.get("index", -1))
            if 0 <= index < self.count:
                self._slots[index] = Slot(**entry)
        # A recording that was in progress when power went is not a recording
        # any more; nothing was finalised, so the slot is free.
        for slot in self._slots.values():
            if slot.kind == SLOT_RECORDING:
                slot.kind = SLOT_EMPTY
                slot.size = 0

    def _save(self) -> None:
        try:
            self._index.write({"slots": [asdict(s) for s in
                                         self._slots.values() if s.occupied]})
        except Exception as exc:                       # noqa: BLE001
            log.error("cannot write slot index: %s", exc)

    def _path(self, index: int) -> Path:
        return self.dir / f"slot-{index:02d}.bin"

    # -- queries ---------------------------------------------------------

    def valid(self, index: int) -> bool:
        return 0 <= index < self.count

    def get(self, index: int) -> Slot | None:
        return self._slots.get(index)

    def list(self) -> list[Slot]:
        return [self._slots[i] for i in range(self.count)]

    def occupied(self) -> list[Slot]:
        return [s for s in self.list() if s.occupied]

    def free_slots(self) -> list[int]:
        return [s.index for s in self.list() if not s.occupied]

    def first_free(self) -> int | None:
        free = self.free_slots()
        return free[0] if free else None

    @property
    def recording_slot(self) -> int | None:
        return self._recording

    def bytes_used(self) -> int:
        return sum(s.size for s in self.occupied())

    # -- writing ---------------------------------------------------------

    def store(self, index: int, data: bytes, kind: int, width: int = 0,
              height: int = 0, duration_s: float = 0.0) -> Slot | None:
        """Write a slot, overwriting whatever was there.

        Returns None if the index is out of range. Overwriting an occupied
        slot is allowed and deliberate: the ground asked for this slot, and
        refusing would mean a capture command that sometimes silently does
        nothing.
        """
        if not self.valid(index):
            return None
        try:
            tmp = self._path(index).with_suffix(".tmp")
            with open(tmp, "wb") as handle:
                handle.write(data)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(tmp, self._path(index))
        except Exception as exc:                       # noqa: BLE001
            log.error("cannot write slot %d: %s", index, exc)
            return None

        slot = Slot(index=index, kind=kind, size=len(data), width=width,
                    height=height, created_unix=time.time(),
                    crc32=zlib.crc32(data) & 0xFFFFFFFF,
                    duration_s=duration_s)
        self._slots[index] = slot
        self._save()
        log.info("slot %d written: %s, %d bytes",
                 index, "image" if kind == SLOT_IMAGE else "video", len(data))
        return slot

    def mark_recording(self, index: int, limit_s: float = 0.0) -> bool:
        if not self.valid(index):
            return False
        self._slots[index] = Slot(index=index, kind=SLOT_RECORDING,
                                  created_unix=time.time())
        self._recording = index
        self._record_started = time.monotonic()
        self._record_limit = limit_s
        self._save()
        return True

    def clear_recording(self) -> int | None:
        index, self._recording = self._recording, None
        self._record_limit = 0.0
        return index

    def recording_expired(self) -> bool:
        """Has a time-limited recording run past its limit?"""
        return (self._recording is not None and self._record_limit > 0
                and time.monotonic() - self._record_started >= self._record_limit)

    @property
    def recording_elapsed_s(self) -> float:
        if self._recording is None:
            return 0.0
        return time.monotonic() - self._record_started

    # -- reading and deleting --------------------------------------------

    def read(self, index: int) -> bytes | None:
        """Read a slot's bytes, verifying the CRC recorded when it was written."""
        slot = self._slots.get(index)
        if slot is None or not slot.occupied or slot.kind == SLOT_RECORDING:
            return None
        try:
            data = self._path(index).read_bytes()
        except Exception as exc:                       # noqa: BLE001
            log.error("cannot read slot %d: %s", index, exc)
            return None

        if slot.crc32 and (zlib.crc32(data) & 0xFFFFFFFF) != slot.crc32:
            # Say so rather than sending minutes of corrupted downlink and
            # letting the ground find out at the end.
            log.error("slot %d failed its stored CRC-32", index)
            slot.kind = SLOT_ERROR
            self._save()
            return None
        return data

    def read_by_media_id(self, media_id: int) -> bytes | None:
        """Resolve an HRT transfer id back to a slot."""
        if (media_id & 0xFFFFFF00) != SLOT_MEDIA_BASE:
            return None
        return self.read(media_id & 0xFF)

    def delete(self, index: int) -> bool:
        if not self.valid(index):
            return False
        slot = self._slots[index]
        if not slot.occupied:
            return False
        try:
            self._path(index).unlink(missing_ok=True)
        except Exception as exc:                       # noqa: BLE001
            log.error("cannot delete slot %d: %s", index, exc)
            return False
        if self._recording == index:
            self.clear_recording()
        self._slots[index] = Slot(index=index)
        self._save()
        log.info("slot %d cleared", index)
        return True

    def delete_all(self) -> int:
        return sum(1 for slot in list(self.occupied())
                   if self.delete(slot.index))

    # -- reporting -------------------------------------------------------

    def summary(self) -> dict:
        occupied = self.occupied()
        return {
            "slot_count": self.count,
            "slots_used": len(occupied),
            "slots_free": self.count - len(occupied),
            "slot_bytes_used": self.bytes_used(),
            "slot_recording": -1 if self._recording is None else self._recording,
        }

    def pack_table(self) -> bytes:
        """The whole table, for SLOT_LIST. 22 bytes per slot, little-endian."""
        import struct
        out = bytearray(struct.pack("<BB", self.count, len(self.occupied())))
        for slot in self.list():
            out += struct.pack("<BBIHHId", slot.index, slot.kind, slot.size,
                               slot.width, slot.height, slot.crc32,
                               slot.created_unix)
        return bytes(out)

    def pack_one(self, index: int) -> bytes:
        import struct
        slot = self._slots.get(index) or Slot(index=index)
        return struct.pack("<BBIHHIdf", slot.index, slot.kind, slot.size,
                           slot.width, slot.height, slot.crc32,
                           slot.created_unix, slot.duration_s)
