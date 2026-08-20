"""Wire framing for the ground↔payload protocol - see protocol.md §2.

COBS (Consistent Overhead Byte Stuffing) removes every zero byte from the
frame body, which lets a single `0x00` act as an unambiguous delimiter. That
matters more than it sounds on a noisy link: after any corruption the receiver
resynchronises simply by scanning to the next zero byte, with no risk of a
payload byte masquerading as a delimiter. The cost is bounded and tiny - one
byte per 254, worst case.

Frame layout, before COBS:

     offset  size  field
          0     1  version (0x01)
          1     1  type
          2     2  seq          little-endian
          4     2  payload_len  little-endian
          6     N  payload
        6+N     4  crc32        IEEE 802.3, over bytes [0 .. 6+N-1]
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass

PROTOCOL_VERSION = 0x01
HEADER_FMT = "<BBHH"
HEADER_LEN = struct.calcsize(HEADER_FMT)     # 6
CRC_LEN = 4
MAX_PAYLOAD = 1024
DELIMITER = b"\x00"


class FramingError(Exception):
    """A frame could not be decoded. Always recoverable: drop and resync."""


# ------------------------------------------------------------------ COBS

def cobs_encode(data: bytes) -> bytes:
    """Encode so the result contains no zero bytes."""
    out = bytearray()
    block = bytearray()

    for byte in data:
        if byte == 0:
            out.append(len(block) + 1)
            out.extend(block)
            block.clear()
        else:
            block.append(byte)
            if len(block) == 254:
                out.append(0xFF)
                out.extend(block)
                block.clear()

    out.append(len(block) + 1)
    out.extend(block)
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Inverse of cobs_encode. Raises FramingError on a malformed stream."""
    out = bytearray()
    i = 0
    n = len(data)

    while i < n:
        code = data[i]
        if code == 0:
            raise FramingError("zero byte inside COBS data")
        i += 1
        end = i + code - 1
        if end > n:
            raise FramingError("COBS block overruns the frame")
        out.extend(data[i:end])
        i = end
        # A full 255 block is a continuation, so it emits no implicit zero.
        if code != 0xFF and i < n:
            out.append(0)

    return bytes(out)


# ----------------------------------------------------------------- frames

@dataclass
class Frame:
    type: int
    seq: int
    payload: bytes = b""

    def __post_init__(self):
        if len(self.payload) > MAX_PAYLOAD:
            raise FramingError(
                f"payload {len(self.payload)} exceeds {MAX_PAYLOAD}")


def encode_frame(frame: Frame) -> bytes:
    """Build a complete on-wire frame, delimiter included."""
    body = struct.pack(HEADER_FMT, PROTOCOL_VERSION, frame.type,
                       frame.seq & 0xFFFF, len(frame.payload))
    body += frame.payload
    body += struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)
    return cobs_encode(body) + DELIMITER


def decode_frame(encoded: bytes) -> Frame:
    """Decode one COBS-encoded frame (delimiter optional)."""
    if encoded.endswith(DELIMITER):
        encoded = encoded[:-1]
    if not encoded:
        raise FramingError("empty frame")

    body = cobs_decode(encoded)

    if len(body) < HEADER_LEN + CRC_LEN:
        raise FramingError(f"frame too short: {len(body)} bytes")

    given = struct.unpack("<I", body[-CRC_LEN:])[0]
    computed = zlib.crc32(body[:-CRC_LEN]) & 0xFFFFFFFF
    if given != computed:
        raise FramingError(
            f"CRC mismatch: computed {computed:08X}, frame says {given:08X}")

    version, msg_type, seq, payload_len = struct.unpack(
        HEADER_FMT, body[:HEADER_LEN])

    if version != PROTOCOL_VERSION:
        raise FramingError(f"unsupported protocol version {version}")

    payload = body[HEADER_LEN:-CRC_LEN]
    if len(payload) != payload_len:
        raise FramingError(
            f"payload_len says {payload_len}, got {len(payload)}")

    return Frame(type=msg_type, seq=seq, payload=payload)


class FrameReader:
    """Reassembles frames from an arbitrarily chunked byte stream.

    Feed it whatever arrives from the serial port; it yields whole frames.
    Garbage between delimiters is discarded and counted rather than raised,
    because a link that occasionally corrupts a frame must not be able to stop
    the receiver.
    """

    def __init__(self, max_buffer: int = 8192):
        self._buf = bytearray()
        self.max_buffer = max_buffer
        self.bad_frames = 0
        self.dropped_bytes = 0

    def feed(self, data: bytes) -> list[Frame]:
        frames: list[Frame] = []
        self._buf.extend(data)

        while True:
            idx = self._buf.find(DELIMITER)
            if idx < 0:
                break

            chunk = bytes(self._buf[:idx])
            del self._buf[:idx + 1]

            if not chunk:
                continue        # delimiter run, or leading delimiter

            try:
                frames.append(decode_frame(chunk))
            except FramingError:
                self.bad_frames += 1

        # Never let an un-delimited stream grow without bound.
        if len(self._buf) > self.max_buffer:
            self.dropped_bytes += len(self._buf)
            self._buf.clear()

        return frames
