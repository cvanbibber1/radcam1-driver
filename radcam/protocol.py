"""Message types, configuration and the command dispatcher - see protocol.md.

The dispatcher is deliberately total: every command either produces a response
or a NACK, and no command is allowed to raise out of `handle()`. A payload that
stops answering the ground because of an unhandled exception is worse than one
that answers "I failed", so the top level catches everything.
"""

from __future__ import annotations

import logging
import struct
import time
import zlib
from dataclasses import asdict, dataclass, field
from enum import IntEnum

from .framing import MAX_PAYLOAD, Frame

log = logging.getLogger(__name__)


class Msg(IntEnum):
    PING = 0x01
    PONG = 0x81
    SET_CONFIG = 0x10
    CONFIG_ACK = 0x90
    GET_CONFIG = 0x11
    CONFIG_REPORT = 0x91
    GET_TELEMETRY = 0x20
    TELEMETRY = 0xA0
    GET_MEDIA_LIST = 0x21
    MEDIA_LIST = 0xA1
    GET_DOSE_LOG = 0x22
    DOSE_LOG = 0xA2
    CAPTURE_IMAGE = 0x30
    CAPTURE_ACK = 0xB0
    START_RECORD = 0x31
    RECORD_ACK = 0xB1
    STOP_RECORD = 0x32
    RECORD_DONE = 0xB2
    REQUEST_MEDIA = 0x40
    MEDIA_INFO = 0xC0
    MEDIA_DATA = 0xC1
    MEDIA_END = 0xC2
    RESEND = 0x41
    DELETE_MEDIA = 0x42
    DELETE_ACK = 0xC3
    SET_LED = 0x50
    LED_ACK = 0xD0
    # Full-resolution capture cropped to a region and rescaled on the payload,
    # so only the interesting part of the frame crosses the link.
    CAPTURE_REGION = 0x33
    # Direct EEPROM access. Calibration is the one thing aboard that cannot be
    # recomputed in flight, so it must be readable and repairable from ground.
    EEPROM_READ = 0x60
    EEPROM_DATA = 0xE0
    EEPROM_WRITE = 0x61
    EEPROM_WRITE_ACK = 0xE1
    EEPROM_STATUS = 0x62
    EEPROM_STATUS_REPORT = 0xE2
    EEPROM_REPAIR = 0x63
    EEPROM_REPAIR_ACK = 0xE3
    NACK = 0xEE


class Err(IntEnum):
    BAD_CRC = 1
    BAD_TYPE = 2
    BAD_PARAM = 3
    BUSY = 4
    NO_MEDIA = 5
    CAMERA_FAULT = 6
    STORAGE_FULL = 7
    BITRATE_EXCEEDS_LINK = 8
    NOT_CALIBRATED = 9
    EEPROM_FAULT = 10
    REGION_INVALID = 11
    WRITE_PROTECTED = 12


class Cfg(IntEnum):
    FLASH_PERCENT = 0x01
    FLASH_DURATION_MS = 0x02
    IMAGE_RESOLUTION = 0x03
    VIDEO_RESOLUTION = 0x04
    VIDEO_DURATION_S = 0x05
    IMAGE_COMPRESSION = 0x06
    VIDEO_COMPRESSION = 0x07
    TELEMETRY_INTERVAL_S = 0x08
    CHUNK_SIZE = 0x09
    # Crop, in units of 1/10000 of the frame, so a u32 carries a fraction.
    CROP_X = 0x0A
    CROP_Y = 0x0B
    CROP_W = 0x0C
    CROP_H = 0x0D
    # Apply the camera's stored distortion model to stills before storing them.
    UNDISTORT = 0x0E


#: Sentinel: record until STOP_RECORD. Chosen by the mission; the side effect
#: is that an exactly-67-second clip cannot be requested (use 66 or 68).
VIDEO_DURATION_INFINITE = 67

RESOLUTIONS = {0: (4096, 3072), 1: (1920, 1080)}

#: name -> nominal encoded bitrate in bits/s, for the link-budget check.
VIDEO_BITRATES = {0: 4_000_000, 1: 1_500_000, 2: 600_000, 3: 400_000}
VIDEO_NAMES = {0: "H264_HIGH", 1: "H264_MED", 2: "H264_LOW", 3: "H264_STREAM"}
IMAGE_NAMES = {0: "PNG_LOSSLESS", 1: "JPEG_HIGH", 2: "JPEG_MED",
               3: "JPEG_LOW", 4: "JPEG_TINY"}

#: Usable link throughput after framing, CRC and ACK overhead (protocol.md §1).
LINK_BYTES_PER_S = 88_000
LINK_BITS_PER_S = LINK_BYTES_PER_S * 8

#: Hard ceiling on illumination, matching radcam.led.MAX_DUTY.
FLASH_PERCENT_MAX = 10


@dataclass
class Config:
    flash_percent: int = 0
    flash_duration_ms: int = 50
    image_resolution: int = 0
    video_resolution: int = 1
    video_duration_s: int = 10
    image_compression: int = 2
    video_compression: int = 2
    telemetry_interval_s: int = 5
    chunk_size: int = 1024
    # Region of the frame actually captured, as fractions. The default is the
    # whole frame. Cropping cuts file size roughly in proportion to area, which
    # on an 88 kB/s link is the difference between a 22 s and a 6 s downlink.
    crop_x: float = 0.0
    crop_y: float = 0.0
    crop_w: float = 1.0
    crop_h: float = 1.0
    #: Correct lens distortion on capture, if the camera carries a model.
    undistort: int = 0

    def to_dict(self) -> dict:
        return asdict(self)


def _clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(int(value), hi))


def apply_config_value(cfg: Config, key: int, value: int) -> None:
    """Apply one key, clamping per protocol.md §4. Unknown keys are ignored."""
    if key == Cfg.FLASH_PERCENT:
        # Mission rule: above the cap becomes the cap; anything else invalid
        # becomes zero. Erring toward "off" is the safe direction for a very
        # bright array.
        cfg.flash_percent = FLASH_PERCENT_MAX if value > FLASH_PERCENT_MAX \
            else (value if 0 <= value <= FLASH_PERCENT_MAX else 0)
    elif key == Cfg.FLASH_DURATION_MS:
        cfg.flash_duration_ms = _clamp(value, 0, 1000)
    elif key == Cfg.IMAGE_RESOLUTION:
        cfg.image_resolution = value if value in RESOLUTIONS else 0
    elif key == Cfg.VIDEO_RESOLUTION:
        cfg.video_resolution = value if value in RESOLUTIONS else 1
    elif key == Cfg.VIDEO_DURATION_S:
        cfg.video_duration_s = (VIDEO_DURATION_INFINITE
                                if value == VIDEO_DURATION_INFINITE
                                else _clamp(value, 1, 3600))
    elif key == Cfg.IMAGE_COMPRESSION:
        cfg.image_compression = value if value in IMAGE_NAMES else 2
    elif key == Cfg.VIDEO_COMPRESSION:
        cfg.video_compression = value if value in VIDEO_NAMES else 2
    elif key == Cfg.TELEMETRY_INTERVAL_S:
        cfg.telemetry_interval_s = _clamp(value, 1, 3600)
    elif key == Cfg.CHUNK_SIZE:
        cfg.chunk_size = _clamp(value, 256, 1024)
    elif key == Cfg.UNDISTORT:
        # Any non-zero value enables it; the correction either happens or it
        # does not, and a "partial" undistort is not a meaningful request.
        cfg.undistort = 1 if value else 0
    elif key in (Cfg.CROP_X, Cfg.CROP_Y, Cfg.CROP_W, Cfg.CROP_H):
        frac = _clamp(value, 0, 10000) / 10000.0
        if key == Cfg.CROP_X:
            cfg.crop_x = frac
        elif key == Cfg.CROP_Y:
            cfg.crop_y = frac
        elif key == Cfg.CROP_W:
            cfg.crop_w = max(frac, 0.01)
        else:
            cfg.crop_h = max(frac, 0.01)
        # Keep the window inside the frame whichever order the keys arrive in.
        cfg.crop_w = min(cfg.crop_w, 1.0 - cfg.crop_x)
        cfg.crop_h = min(cfg.crop_h, 1.0 - cfg.crop_y)


def check_link_budget(cfg: Config) -> bool:
    """False if continuous recording could never be transmitted in real time.

    Finite clips at any bitrate are allowed - they simply take longer than the
    recording did. Infinite recording at a bitrate above the link is a trap:
    the backlog grows without bound, so it is refused up front.
    """
    if cfg.video_duration_s != VIDEO_DURATION_INFINITE:
        return True
    return VIDEO_BITRATES.get(cfg.video_compression, 0) < LINK_BITS_PER_S


def estimate_transfer_s(size_bytes: int) -> float:
    return size_bytes / LINK_BYTES_PER_S


@dataclass
class MediaRecord:
    media_id: int
    kind: str                  # "image" | "video"
    path: str
    size: int
    width: int
    height: int
    created_unix: float
    compression: str
    duration_s: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)


class Dispatcher:
    """Turns request frames into response frames.

    `camera`, `dosimeter`, `led` and `store` are injected so this is testable
    without any hardware, which is the only reason the protocol could be
    verified at all while the sensor is unresponsive.
    """

    #: Bytes per EEPROM_DATA frame. Kept small so a corrupted reply costs
    #: little to re-request, and so a dump interleaves with telemetry rather
    #: than blocking the link for seconds at a time.
    EEPROM_CHUNK = 128

    def __init__(self, config: Config | None = None, camera=None,
                 dosimeter=None, led=None, store=None, version: str = "1.0",
                 eeprom=None, eeprom_writable: bool = False):
        self.config = config or Config()
        self.camera = camera
        self.dosimeter = dosimeter
        self.led = led
        self.store = store
        self.eeprom = eeprom
        # Writing is opt-in. A stray or corrupted EEPROM_WRITE would destroy
        # the one piece of state that cannot be regenerated in flight, so the
        # command exists but the door stays shut until the mission opens it.
        self.eeprom_writable = eeprom_writable
        self.version = version
        self.started = time.monotonic()
        self.recording: int | None = None
        self._transfers: dict[int, bytes] = {}

    # -- helpers ---------------------------------------------------------

    def _nack(self, seq: int, code: Err) -> Frame:
        return Frame(Msg.NACK, seq, struct.pack("<HB", seq, int(code)))

    def _config_payload(self) -> bytes:
        """Every key/value pair, so the ground never has to infer the result."""
        items = [
            (Cfg.FLASH_PERCENT, self.config.flash_percent),
            (Cfg.FLASH_DURATION_MS, self.config.flash_duration_ms),
            (Cfg.IMAGE_RESOLUTION, self.config.image_resolution),
            (Cfg.VIDEO_RESOLUTION, self.config.video_resolution),
            (Cfg.VIDEO_DURATION_S, self.config.video_duration_s),
            (Cfg.IMAGE_COMPRESSION, self.config.image_compression),
            (Cfg.VIDEO_COMPRESSION, self.config.video_compression),
            (Cfg.TELEMETRY_INTERVAL_S, self.config.telemetry_interval_s),
            (Cfg.CHUNK_SIZE, self.config.chunk_size),
            (Cfg.CROP_X, int(round(self.config.crop_x * 10000))),
            (Cfg.CROP_Y, int(round(self.config.crop_y * 10000))),
            (Cfg.CROP_W, int(round(self.config.crop_w * 10000))),
            (Cfg.CROP_H, int(round(self.config.crop_h * 10000))),
            (Cfg.UNDISTORT, self.config.undistort),
        ]
        return b"".join(struct.pack("<BI", int(k), v) for k, v in items)

    # -- dispatch --------------------------------------------------------

    def handle(self, frame: Frame) -> list[Frame]:
        """Handle one request. Never raises."""
        try:
            return self._handle(frame)
        except Exception as exc:                      # noqa: BLE001
            log.exception("dispatcher failed on type 0x%02X: %s",
                          frame.type, exc)
            return [self._nack(frame.seq, Err.BAD_PARAM)]

    def _handle(self, frame: Frame) -> list[Frame]:
        t = frame.type

        if t == Msg.PING:
            uptime = int(time.monotonic() - self.started)
            return [Frame(Msg.PONG, frame.seq,
                          struct.pack("<I", uptime) + self.version.encode())]

        if t == Msg.SET_CONFIG:
            if len(frame.payload) % 5:
                return [self._nack(frame.seq, Err.BAD_PARAM)]
            for off in range(0, len(frame.payload), 5):
                key, value = struct.unpack("<BI", frame.payload[off:off + 5])
                apply_config_value(self.config, key, value)
            if not check_link_budget(self.config):
                return [self._nack(frame.seq, Err.BITRATE_EXCEEDS_LINK)]
            return [Frame(Msg.CONFIG_ACK, frame.seq, self._config_payload())]

        if t == Msg.GET_CONFIG:
            return [Frame(Msg.CONFIG_REPORT, frame.seq, self._config_payload())]

        if t == Msg.GET_TELEMETRY:
            return [Frame(Msg.TELEMETRY, frame.seq, self._telemetry_payload())]

        if t == Msg.GET_MEDIA_LIST:
            return [Frame(Msg.MEDIA_LIST, frame.seq, self._media_list_payload())]

        if t == Msg.SET_LED:
            if len(frame.payload) < 1:
                return [self._nack(frame.seq, Err.BAD_PARAM)]
            requested = frame.payload[0]
            applied = FLASH_PERCENT_MAX if requested > FLASH_PERCENT_MAX \
                else requested
            if self.led is not None:
                applied = int(round(self.led.set_percent(applied) * 100))
            return [Frame(Msg.LED_ACK, frame.seq, bytes([applied]))]

        if t == Msg.CAPTURE_IMAGE:
            if self.camera is None or not self.camera.available():
                return [self._nack(frame.seq, Err.CAMERA_FAULT)]
            rec = self.camera.capture_image(self.config)
            return [Frame(Msg.CAPTURE_ACK, frame.seq,
                          struct.pack("<IQ", rec.media_id, rec.size))]

        if t == Msg.START_RECORD:
            if self.camera is None or not self.camera.available():
                return [self._nack(frame.seq, Err.CAMERA_FAULT)]
            if self.recording is not None:
                return [self._nack(frame.seq, Err.BUSY)]
            if not check_link_budget(self.config):
                return [self._nack(frame.seq, Err.BITRATE_EXCEEDS_LINK)]
            self.recording = self.camera.start_record(self.config)
            return [Frame(Msg.RECORD_ACK, frame.seq,
                          struct.pack("<I", self.recording))]

        if t == Msg.STOP_RECORD:
            if self.recording is None:
                return [self._nack(frame.seq, Err.BAD_PARAM)]
            rec = self.camera.stop_record()
            self.recording = None
            return [Frame(Msg.RECORD_DONE, frame.seq,
                          struct.pack("<IQf", rec.media_id, rec.size,
                                      rec.duration_s))]

        if t == Msg.CAPTURE_REGION:
            return self._capture_region(frame)

        if t in (Msg.EEPROM_READ, Msg.EEPROM_WRITE, Msg.EEPROM_STATUS,
                 Msg.EEPROM_REPAIR):
            return self._eeprom(frame)

        if t == Msg.REQUEST_MEDIA:
            return self._request_media(frame)

        if t == Msg.RESEND:
            return self._resend(frame)

        if t == Msg.DELETE_MEDIA:
            if self.store is None or len(frame.payload) < 4:
                return [self._nack(frame.seq, Err.BAD_PARAM)]
            media_id = struct.unpack("<I", frame.payload[:4])[0]
            if not self.store.delete(media_id):
                return [self._nack(frame.seq, Err.NO_MEDIA)]
            self._transfers.pop(media_id, None)
            return [Frame(Msg.DELETE_ACK, frame.seq,
                          struct.pack("<I", media_id))]

        if t == Msg.GET_DOSE_LOG:
            return [Frame(Msg.DOSE_LOG, frame.seq, self._dose_log_payload(frame))]

        return [self._nack(frame.seq, Err.BAD_TYPE)]

    # -- payload builders ------------------------------------------------

    def _telemetry_payload(self) -> bytes:
        dose = 0.0
        volts = 0.0
        calibrated = 0
        if self.dosimeter is not None:
            try:
                r = self.dosimeter.read()
                volts = r.volts
                calibrated = 1 if r.calibrated else 0
                dose = r.dose_rad if r.dose_rad is not None else 0.0
            except Exception as exc:                  # noqa: BLE001
                log.error("telemetry dosimeter read failed: %s", exc)

        brightness = self.led.brightness if self.led is not None else 0.0
        return struct.pack("<ffBfI", dose, volts, calibrated, brightness,
                           int(time.monotonic() - self.started))

    def _media_list_payload(self) -> bytes:
        if self.store is None:
            return struct.pack("<H", 0)
        records = self.store.list()
        out = struct.pack("<H", len(records))
        for r in records:
            out += struct.pack("<IBQHHd", r.media_id,
                               0 if r.kind == "image" else 1,
                               r.size, r.width, r.height, r.created_unix)
        return out

    def _dose_log_payload(self, frame: Frame) -> bytes:
        start = end = None
        if len(frame.payload) >= 16:
            start, end = struct.unpack("<dd", frame.payload[:16])
        records = []
        if self.store is not None and hasattr(self.store, "dose_records"):
            records = self.store.dose_records(start, end)
        out = struct.pack("<H", len(records))
        for ts, dose, volts in records:
            out += struct.pack("<dff", ts, dose, volts)
        return out

    # -- media transfer --------------------------------------------------

    def _capture_region(self, frame: Frame) -> list[Frame]:
        """Capture at full sensor resolution, keep one region, rescale it.

        Payload: x, y, w, h in sensor pixels, then out_w, out_h (all u16).

        The point is bandwidth, not framing. Downlink is 88 kB/s, so a full
        12 MP still is minutes of airtime; sending a 1920x1080 window onto the
        part that matters costs a fraction of that while keeping full sensor
        resolution *within* the window. Cropping before rescaling, rather than
        capturing at low resolution, is what preserves the detail - a 1080p
        capture of the whole frame throws the detail away in the sensor.
        """
        if self.camera is None or not self.camera.available():
            return [self._nack(frame.seq, Err.CAMERA_FAULT)]
        if len(frame.payload) < 12:
            return [self._nack(frame.seq, Err.BAD_PARAM)]
        x, y, w, h, ow, oh = struct.unpack("<6H", frame.payload[:12])
        full_w, full_h = RESOLUTIONS[0]
        if w == 0 or h == 0 or x + w > full_w or y + h > full_h:
            return [self._nack(frame.seq, Err.REGION_INVALID)]
        if ow == 0 or oh == 0 or ow > full_w or oh > full_h:
            return [self._nack(frame.seq, Err.REGION_INVALID)]
        rec = self.camera.capture_region(self.config, (x, y, w, h), (ow, oh))
        return [Frame(Msg.CAPTURE_ACK, frame.seq,
                      struct.pack("<IQ", rec.media_id, rec.size))]

    def _eeprom(self, frame: Frame) -> list[Frame]:
        """Read, verify, repair or rewrite the camera's calibration EEPROM.

        Calibration is measured on the ground and cannot be reproduced in
        flight, so the ground needs to be able to see exactly what the part is
        carrying and put it back if a bit flips. All four operations work on
        raw bytes rather than the parsed record: if the JSON is what got
        corrupted, a parsing interface could not report it, let alone fix it.
        """
        if self.eeprom is None:
            return [self._nack(frame.seq, Err.EEPROM_FAULT)]
        t = frame.type
        try:
            if t == Msg.EEPROM_STATUS:
                # Which of the three copies still verify. This is the question
                # to ask first after a radiation event, and it is cheap.
                ok = self.eeprom.copy_status()
                return [Frame(Msg.EEPROM_STATUS_REPORT, frame.seq,
                              struct.pack("<BB", len(ok),
                                          sum(1 for v in ok if v))
                              + bytes(1 if v else 0 for v in ok))]

            if t == Msg.EEPROM_REPAIR:
                # Rewrite every copy from whichever one still passes CRC.
                healed = self.eeprom.repair()
                return [Frame(Msg.EEPROM_REPAIR_ACK, frame.seq,
                              struct.pack("<B", 1 if healed else 0))]

            if t == Msg.EEPROM_READ:
                if len(frame.payload) < 4:
                    return [self._nack(frame.seq, Err.BAD_PARAM)]
                off, length = struct.unpack("<HH", frame.payload[:4])
                length = min(length or self.EEPROM_CHUNK, self.EEPROM_CHUNK)
                data = self.eeprom.read(off, length)
                return [Frame(Msg.EEPROM_DATA, frame.seq,
                              struct.pack("<HH", off, len(data)) + bytes(data))]

            if t == Msg.EEPROM_WRITE:
                if not self.eeprom_writable:
                    return [self._nack(frame.seq, Err.WRITE_PROTECTED)]
                if len(frame.payload) < 4:
                    return [self._nack(frame.seq, Err.BAD_PARAM)]
                off, length = struct.unpack("<HH", frame.payload[:4])
                data = frame.payload[4:4 + length]
                if len(data) != length or length > self.EEPROM_CHUNK:
                    return [self._nack(frame.seq, Err.BAD_PARAM)]
                self.eeprom.write(off, data)
                # Read back and report what is actually there, so the ground
                # confirms the write rather than trusting an ACK.
                back = bytes(self.eeprom.read(off, length))
                return [Frame(Msg.EEPROM_WRITE_ACK, frame.seq,
                              struct.pack("<HHB", off, length,
                                          1 if back == data else 0))]
        except Exception as exc:                      # noqa: BLE001
            log.warning("eeprom op 0x%02X failed: %s", t, exc)
            return [self._nack(frame.seq, Err.EEPROM_FAULT)]
        return [self._nack(frame.seq, Err.BAD_TYPE)]

    def _request_media(self, frame: Frame) -> list[Frame]:
        if self.store is None or len(frame.payload) < 4:
            return [self._nack(frame.seq, Err.BAD_PARAM)]

        media_id = struct.unpack("<I", frame.payload[:4])[0]
        data = self.store.read(media_id)
        if data is None:
            return [self._nack(frame.seq, Err.NO_MEDIA)]

        self._transfers[media_id] = data
        chunk = self.config.chunk_size
        # Chunk payloads carry a 10-byte header, so keep frames inside the cap.
        usable = min(chunk, MAX_PAYLOAD - 10)
        n_chunks = (len(data) + usable - 1) // usable

        out = [Frame(Msg.MEDIA_INFO, frame.seq,
                     struct.pack("<IQIf", media_id, len(data), n_chunks,
                                 estimate_transfer_s(len(data))))]
        out.extend(self._chunks(media_id, data, usable, range(n_chunks)))
        out.append(Frame(Msg.MEDIA_END, frame.seq,
                         struct.pack("<II", media_id,
                                     zlib.crc32(data) & 0xFFFFFFFF)))
        return out

    def _chunks(self, media_id: int, data: bytes, usable: int,
                indices) -> list[Frame]:
        frames = []
        for i in indices:
            start = i * usable
            piece = data[start:start + usable]
            if not piece:
                continue
            frames.append(Frame(
                Msg.MEDIA_DATA, i & 0xFFFF,
                struct.pack("<IIH", media_id, i, len(piece)) + piece))
        return frames

    def _resend(self, frame: Frame) -> list[Frame]:
        if len(frame.payload) < 4:
            return [self._nack(frame.seq, Err.BAD_PARAM)]
        media_id = struct.unpack("<I", frame.payload[:4])[0]
        data = self._transfers.get(media_id)
        if data is None:
            data = self.store.read(media_id) if self.store else None
            if data is None:
                return [self._nack(frame.seq, Err.NO_MEDIA)]
            self._transfers[media_id] = data

        rest = frame.payload[4:]
        wanted = [struct.unpack("<I", rest[i:i + 4])[0]
                  for i in range(0, len(rest) - 3, 4)]
        usable = min(self.config.chunk_size, MAX_PAYLOAD - 10)
        return self._chunks(media_id, data, usable, wanted)
