"""Camera-module EEPROM: per-camera identity and calibration.

Cameras and Pis are interchangeable, so anything that describes a *specific
camera* — its colour calibration, its serial, its lens — has to travel with the
camera, not the host. The AR1335 module carries a 24C64 (8 KB, 16-bit
addressing) at 0x50 on the same I2C bus as the sensor, which is exactly the
right place for it.

Layout: three independent copies of the record, well separated, each with its
own CRC32. Calibration cannot be recomputed in flight and a corrupted matrix
would silently ruin every image, so it gets the same triple-redundancy
treatment as the dosimeter baseline — see tmr.py. A copy that fails CRC is
rebuilt from a good one on the next write.

    0x0000  copy 0
    0x0800  copy 1
    0x1000  copy 2
    0x1800  spare

Each copy:
    magic   8 bytes  "RADCAM\\x01\\x00"
    length  2 bytes  payload length, little-endian
    crc32   4 bytes  over the payload
    payload N bytes  JSON
"""

from __future__ import annotations

import json
import logging
import struct
import time
import zlib
from typing import Any

from smbus2 import SMBus, i2c_msg

log = logging.getLogger(__name__)

EEPROM_ADDR = 0x50
EEPROM_SIZE = 8192
PAGE_SIZE = 32              # 24C64 page; writes must not cross a page boundary
WRITE_CYCLE_S = 0.006       # datasheet max write time is 5 ms

MAGIC = b"RADCAM\x01\x00"
HEADER_FMT = "<8sHI"
HEADER_LEN = struct.calcsize(HEADER_FMT)

COPY_OFFSETS = (0x0000, 0x0800, 0x1000)
MAX_PAYLOAD = 0x0800 - HEADER_LEN


class EEPROMError(Exception):
    pass


class CameraEEPROM:
    """Raw access to the module's 24C64."""

    def __init__(self, bus: int = 4, address: int = EEPROM_ADDR):
        self.bus_num = bus
        self.address = address

    # -- raw byte access -------------------------------------------------

    def read(self, offset: int, length: int) -> bytes:
        if offset + length > EEPROM_SIZE:
            raise EEPROMError("read past end of device")
        out = bytearray()
        with SMBus(self.bus_num) as bus:
            # Sequential reads are fine; chunk them so no single transfer is
            # unreasonably long.
            while length:
                n = min(length, 128)
                w = i2c_msg.write(self.address,
                                  [(offset >> 8) & 0xFF, offset & 0xFF])
                r = i2c_msg.read(self.address, n)
                bus.i2c_rdwr(w, r)
                out += bytes(bytearray(list(r)))
                offset += n
                length -= n
        return bytes(out)

    def write(self, offset: int, data: bytes) -> None:
        """Write, respecting page boundaries and the write-cycle delay."""
        if offset + len(data) > EEPROM_SIZE:
            raise EEPROMError("write past end of device")
        with SMBus(self.bus_num) as bus:
            i = 0
            while i < len(data):
                # A page write wraps within the page rather than carrying, so
                # never let a chunk straddle a boundary.
                space = PAGE_SIZE - ((offset + i) % PAGE_SIZE)
                chunk = data[i:i + space]
                payload = [((offset + i) >> 8) & 0xFF, (offset + i) & 0xFF]
                payload += list(chunk)
                bus.i2c_rdwr(i2c_msg.write(self.address, payload))
                time.sleep(WRITE_CYCLE_S)
                i += len(chunk)

    def present(self) -> bool:
        try:
            self.read(0, 1)
            return True
        except OSError:
            return False

    # -- framed records --------------------------------------------------

    @staticmethod
    def _frame(payload: bytes) -> bytes:
        return (struct.pack(HEADER_FMT, MAGIC, len(payload),
                            zlib.crc32(payload) & 0xFFFFFFFF) + payload)

    @staticmethod
    def _unframe(blob: bytes) -> bytes | None:
        if len(blob) < HEADER_LEN:
            return None
        magic, length, crc = struct.unpack(HEADER_FMT, blob[:HEADER_LEN])
        if magic != MAGIC or length == 0 or length > MAX_PAYLOAD:
            return None
        payload = blob[HEADER_LEN:HEADER_LEN + length]
        if len(payload) != length:
            return None
        if (zlib.crc32(payload) & 0xFFFFFFFF) != crc:
            return None
        return payload

    def load(self) -> dict[str, Any] | None:
        """Read the calibration record, repairing damaged copies."""
        payloads: dict[int, bytes] = {}
        for off in COPY_OFFSETS:
            try:
                blob = self.read(off, HEADER_LEN + 512)
            except OSError as exc:
                log.warning("EEPROM copy at 0x%04X unreadable: %s", off, exc)
                continue
            # The stored length may exceed the speculative read; re-read exactly.
            if blob[:8] == MAGIC:
                length = struct.unpack("<H", blob[8:10])[0]
                if 0 < length <= MAX_PAYLOAD:
                    try:
                        blob = self.read(off, HEADER_LEN + length)
                    except OSError:
                        continue
            p = self._unframe(blob)
            if p is not None:
                payloads[off] = p

        if not payloads:
            return None

        values = list(payloads.values())
        winner = max(set(values), key=values.count)
        if len(set(values)) > 1:
            log.warning("EEPROM copies disagree; using the majority")

        # Repair anything that did not match.
        for off in COPY_OFFSETS:
            if payloads.get(off) != winner:
                log.warning("repairing EEPROM copy at 0x%04X", off)
                try:
                    self.write(off, self._frame(winner))
                except OSError as exc:
                    log.error("cannot repair copy at 0x%04X: %s", off, exc)

        try:
            return json.loads(winner)
        except ValueError as exc:
            log.error("EEPROM payload is not valid JSON: %s", exc)
            return None

    def store(self, record: dict[str, Any]) -> None:
        payload = json.dumps(record, sort_keys=True,
                             separators=(",", ":")).encode()
        if len(payload) > MAX_PAYLOAD:
            raise EEPROMError(
                f"record is {len(payload)} bytes, limit is {MAX_PAYLOAD}")
        frame = self._frame(payload)
        for off in COPY_OFFSETS:
            self.write(off, frame)

        # Read back and verify every copy: a silent EEPROM write failure would
        # be indistinguishable from a good one until the camera flew.
        check = self.load()
        if check != record:
            raise EEPROMError("verification after write failed")
        log.info("stored %d-byte calibration record in %d copies",
                 len(payload), len(COPY_OFFSETS))

    def _payloads(self) -> dict[int, bytes]:
        """Whichever copies currently pass their CRC, keyed by offset."""
        out: dict[int, bytes] = {}
        for off in COPY_OFFSETS:
            try:
                blob = self.read(off, HEADER_LEN + 512)
                if blob[:8] == MAGIC:
                    length = struct.unpack("<H", blob[8:10])[0]
                    if 0 < length <= MAX_PAYLOAD:
                        blob = self.read(off, HEADER_LEN + length)
                p = self._unframe(blob)
            except OSError:
                p = None
            if p is not None:
                out[off] = p
        return out

    def copy_status(self) -> list[bool]:
        """Which of the three copies verify, in offset order.

        The question to ask first after a radiation event, and cheap enough to
        ask often: it reads three headers rather than parsing anything, so a
        record whose JSON is mangled still reports honestly.
        """
        good = self._payloads()
        return [off in good for off in COPY_OFFSETS]

    def repair(self) -> bool:
        """Rewrite every copy from a surviving one. True if anything changed.

        Deliberately separate from `load()`, which repairs as a side effect.
        Ground needs to be able to *ask* for the repair and get a yes or no,
        rather than infer it from a log line it cannot see.
        """
        payloads = self._payloads()
        if not payloads:
            return False
        values = list(payloads.values())
        winner = max(set(values), key=values.count)
        healed = False
        for off in COPY_OFFSETS:
            if payloads.get(off) != winner:
                self.write(off, self._frame(winner))
                healed = True
        return healed

    def update(self, section: str, value: Any, **top) -> dict[str, Any]:
        """Merge one calibration section into the stored record.

        Every calibration - colour, shading, distortion, exposure - lives in
        the same record, and each is measured by a different tool on a
        different day. A tool that called store() with only its own results
        would silently wipe the others, so adding a section is a read, merge
        and write rather than a write.
        """
        record = self.load() or {"schema": 1, "sensor": "ar1335"}
        record[section] = value
        for k, v in top.items():
            if v is not None:
                record[k] = v
        self.store(record)
        return record

    def erase(self) -> None:
        blank = b"\xff" * (HEADER_LEN + 8)
        for off in COPY_OFFSETS:
            self.write(off, blank)
