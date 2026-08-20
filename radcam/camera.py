"""Capture, compression and local media storage.

The camera source is abstracted so the whole pipeline - encoding, storage,
metadata, the protocol's media transfer - can be built and measured before the
AR1335 responds on I2C. `SyntheticSource` generates frames with realistic
spatial-frequency content; `LibcameraSource` drives the real sensor through
rpicam-apps and takes over unchanged the moment the hardware works.

Compression profiles come straight from protocol.md §4. Two things about them
are worth knowing:

* **The Pi 5 has no hardware H.264 encoder.** Unlike the Pi 4, video is encoded
  in software by libx264, which costs CPU and therefore power - a real
  consideration for a power-minimised payload. Encode time is measured by
  tools/bench-compression.py rather than assumed.
* PNG at full resolution is enormous on a slow link. It exists for a lossless
  reference frame, not for routine downlink.
"""

from __future__ import annotations

import io
import json
import logging
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

from .protocol import (IMAGE_NAMES, RESOLUTIONS, VIDEO_BITRATES, VIDEO_NAMES,
                       Config, MediaRecord)

log = logging.getLogger(__name__)

MEDIA_DIR = "/var/lib/radcam/media"

#: JPEG quality per image compression profile (index -> quality, None = PNG).
JPEG_QUALITY = {0: None, 1: 92, 2: 80, 3: 60, 4: 40}


# --------------------------------------------------------------- sources

class CameraSource:
    """A source of raw RGB frames."""

    def available(self) -> bool:
        raise NotImplementedError

    def grab(self, width: int, height: int):
        """Return a PIL Image at the requested size."""
        raise NotImplementedError


class SyntheticSource(CameraSource):
    """Test-pattern frames, for building and benchmarking without a sensor.

    Deliberately not flat: a uniform image compresses unrealistically well and
    would make the link budget look far better than reality. This mixes
    gradients, hard edges and fine noise so the compressed sizes land in a
    plausible range for a natural scene.
    """

    def available(self) -> bool:
        return True

    def grab(self, width: int, height: int, crop=None):
        """Render the test scene, optionally windowed to `crop`.

        `crop` is (x, y, w, h) in fractions of the frame, matching
        LibcameraSource: the window is what gets imaged, and the result comes
        back at the requested width and height.

        The scene is defined over world coordinates rather than pixel indices,
        so a crop genuinely selects a different part of it. That is what lets
        the region-capture path be tested without a sensor - with the crop
        ignored, every window returns the same picture and the test proves
        nothing.
        """
        import numpy as np
        from PIL import Image

        x0, y0, cw, ch = (0.0, 0.0, 1.0, 1.0) if not crop else crop
        u = x0 + (np.arange(width) + 0.5) / width * cw
        v = y0 + (np.arange(height) + 0.5) / height * ch
        U, V = np.meshgrid(u.astype(np.float32), v.astype(np.float32))

        # Coarse gradient, so position in the world is visible in the pixels.
        r = U * 255.0
        g = V * 255.0
        b = np.full_like(U, 128.0)

        # Hard edges at fixed world positions, which is where a codec actually
        # spends its bits.
        bar = U * 12.0
        in_bar = ((bar % 1.0) < 0.30) & (V > 0.25) & (V < 0.75)
        light = (bar.astype(np.int32) % 2) == 1
        for ch_arr in (r, g, b):
            ch_arr[in_bar & light] = 255.0
            ch_arr[in_bar & ~light] = 0.0

        # Per-pixel noise. Deliberately not flat: a uniform image compresses
        # unrealistically well and would make the link budget look far better
        # than reality.
        rng = np.random.default_rng(1234)
        n = rng.integers(-24, 25, size=(height, width, 1)).astype(np.float32)
        out = np.clip(np.stack([r, g, b], axis=-1) + n, 0, 255)
        return Image.fromarray(out.astype(np.uint8), "RGB")


class LibcameraSource(CameraSource):
    """The real AR1335, driven through rpicam-still."""

    def __init__(self, timeout_s: float = 15.0):
        self.timeout_s = timeout_s
        self._checked: bool | None = None

    def available(self) -> bool:
        if self._checked is None:
            try:
                out = subprocess.run(["rpicam-hello", "--list-cameras"],
                                     capture_output=True, text=True,
                                     timeout=10)
                self._checked = "ar1335" in (out.stdout + out.stderr).lower()
            except Exception:
                self._checked = False
            if not self._checked:
                log.warning("no AR1335 visible to libcamera")
        return self._checked

    def grab(self, width: int, height: int, crop=None):
        """Capture a frame, optionally from a sub-region of the sensor.

        `crop` is (x, y, w, h) as fractions. Passing it to --roi *and* asking
        for a correspondingly smaller output gives a true 1:1 crop rather than
        a digital zoom: rpicam upscales a ROI back to the requested size, so
        without shrinking the output the file stays just as large.
        """
        from PIL import Image

        tmp = Path("/tmp/radcam-grab.png")
        cmd = ["rpicam-still", "-n", "--width", str(width), "--height",
               str(height), "-t", "1000", "-e", "png", "-o", str(tmp)]
        if crop and (crop[2] < 1.0 or crop[3] < 1.0):
            cmd += ["--roi", ",".join(f"{v:.4f}" for v in crop)]
        subprocess.run(cmd, check=True, capture_output=True,
                       timeout=self.timeout_s)
        img = Image.open(tmp).convert("RGB")
        img.load()
        tmp.unlink(missing_ok=True)
        return img


# ------------------------------------------------------------- encoding

def encode_image(img, profile: int) -> tuple[bytes, str]:
    """Compress a frame per the image profile. Returns (bytes, extension)."""
    buf = io.BytesIO()
    quality = JPEG_QUALITY.get(profile, 80)

    if quality is None:
        img.save(buf, format="PNG", optimize=True)
        return buf.getvalue(), "png"

    img.save(buf, format="JPEG", quality=quality, optimize=True,
             subsampling=1 if quality >= 90 else 2)
    return buf.getvalue(), "jpg"


def encode_video(frames_dir: Path, out_path: Path, profile: int, fps: int,
                 width: int, height: int) -> None:
    """Encode a directory of PNG frames to H.264 at the profile's bitrate.

    Software encoding (libx264): the Pi 5 has no hardware H.264 encoder.
    """
    bitrate = VIDEO_BITRATES.get(profile, 600_000)
    subprocess.run(
        ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
         "-framerate", str(fps), "-i", str(frames_dir / "f%05d.png"),
         "-c:v", "libx264", "-preset", "veryfast",
         "-b:v", str(bitrate), "-maxrate", str(bitrate),
         "-bufsize", str(bitrate * 2),
         "-pix_fmt", "yuv420p", "-s", f"{width}x{height}",
         str(out_path)],
        check=True, capture_output=True)


# ---------------------------------------------------------------- store

class MediaStore:
    """Local media storage plus the metadata the protocol reports."""

    def __init__(self, directory: str = MEDIA_DIR):
        self.dir = Path(directory)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.index = self.dir / "index.json"
        self._records: dict[int, MediaRecord] = {}
        self._next_id = 1
        self._load()

    def _load(self) -> None:
        try:
            data = json.loads(self.index.read_text())
            for d in data:
                rec = MediaRecord(**d)
                self._records[rec.media_id] = rec
            if self._records:
                self._next_id = max(self._records) + 1
        except (OSError, ValueError):
            pass

    def _save(self) -> None:
        try:
            self.index.write_text(json.dumps(
                [r.to_dict() for r in self._records.values()], indent=1))
        except OSError as exc:
            log.error("cannot write media index: %s", exc)

    def add(self, data: bytes, kind: str, width: int, height: int,
            compression: str, ext: str, duration_s: float = 0.0) -> MediaRecord:
        media_id = self._next_id
        self._next_id += 1
        path = self.dir / f"{media_id:06d}.{ext}"
        path.write_bytes(data)

        rec = MediaRecord(media_id=media_id, kind=kind, path=str(path),
                          size=len(data), width=width, height=height,
                          created_unix=time.time(), compression=compression,
                          duration_s=duration_s)
        self._records[media_id] = rec
        self._save()
        return rec

    def read(self, media_id: int) -> bytes | None:
        rec = self._records.get(media_id)
        if rec is None:
            return None
        try:
            return Path(rec.path).read_bytes()
        except OSError:
            return None

    def list(self) -> list[MediaRecord]:
        return list(self._records.values())

    def delete(self, media_id: int) -> bool:
        rec = self._records.pop(media_id, None)
        if rec is None:
            return False
        Path(rec.path).unlink(missing_ok=True)
        self._save()
        return True

    def free_bytes(self) -> int:
        return shutil.disk_usage(self.dir).free


# --------------------------------------------------------------- camera

@dataclass
class CaptureResult:
    record: MediaRecord
    encode_s: float


class Camera:
    """Capture pipeline: grab, flash, encode, store.

    The LEDs are lit only inside the `flash()` block, which guarantees they go
    out even if the capture raises.
    """

    def __init__(self, source: CameraSource | None = None,
                 store: MediaStore | None = None, led=None,
                 distortion_model: dict | None = None):
        self.source = source or LibcameraSource()
        self.store = store or MediaStore()
        self.led = led
        # The camera's stored distortion model, if it carries one. Injected
        # rather than read here: the daemon already has the EEPROM open, and
        # the capture path should not do I2C.
        self.distortion_model = distortion_model
        self._recording: MediaRecord | None = None
        self._record_started = 0.0

    def _maybe_undistort(self, img, cfg):
        """Correct lens distortion, if asked and if the camera carries a model.

        Silently doing nothing when there is no model is deliberate: an
        uncalibrated camera must still return pictures. The alternative -
        failing the capture - turns a missing calibration into a missing image.
        """
        if not getattr(cfg, "undistort", 0) or not self.distortion_model:
            return img
        try:
            import numpy as np
            from PIL import Image
            from . import distortion as D

            m = self.distortion_model
            t0 = time.monotonic()
            out, _ = D.undistort_image(
                np.asarray(img.convert("RGB")), tuple(m["centre"]), m["k"],
                model=m.get("model", D.MODEL_POLY), r_valid=m.get("r_valid"))
            log.info("undistorted in %.2fs", time.monotonic() - t0)
            return Image.fromarray(out, "RGB")
        except Exception as exc:                       # noqa: BLE001
            log.warning("undistort failed, storing the raw frame: %s", exc)
            return img

    def available(self) -> bool:
        return self.source.available()

    def capture_image(self, cfg: Config) -> MediaRecord:
        width, height = RESOLUTIONS.get(cfg.image_resolution, (4096, 3072))

        crop = (cfg.crop_x, cfg.crop_y, cfg.crop_w, cfg.crop_h)
        if crop[2] < 1.0 or crop[3] < 1.0:
            # Scale the output to match the cropped area so the capture is 1:1
            # and the file shrinks with the area, rather than being upscaled.
            width = max(int(width * crop[2]) & ~1, 2)
            height = max(int(height * crop[3]) & ~1, 2)

        if self.led is not None and cfg.flash_percent > 0:
            with self.led.flash(cfg.flash_percent):
                img = self.source.grab(width, height, crop)
        else:
            img = self.source.grab(width, height, crop)

        img = self._maybe_undistort(img, cfg)

        t0 = time.monotonic()
        data, ext = encode_image(img, cfg.image_compression)
        encode_s = time.monotonic() - t0

        rec = self.store.add(data, "image", width, height,
                             IMAGE_NAMES.get(cfg.image_compression, "?"), ext)
        log.info("captured image %d: %dx%d %s, %d B, encoded in %.2fs",
                 rec.media_id, width, height, rec.compression, rec.size,
                 encode_s)
        return rec

    def capture_region(self, cfg: Config, region, out_size) -> MediaRecord:
        """Capture at full sensor resolution, keep `region`, rescale to `out_size`.

        `region` is (x, y, w, h) in full-resolution sensor pixels; `out_size` is
        (w, h) for the stored file.

        This is not the same as configuring a smaller capture. The sensor is
        read at its full 4096x3072 and the window is taken from *that*, so the
        detail inside the window is real sensor detail. Asking the camera for a
        1080p frame of the whole scene would bin it away before it ever reached
        here. The saving is in what crosses the 88 kB/s link, not in what the
        sensor does.
        """
        full_w, full_h = RESOLUTIONS[0]
        x, y, w, h = (int(v) for v in region)
        ow, oh = (int(v) for v in out_size)
        # The source takes a crop as fractions of the frame.
        crop = (x / full_w, y / full_h, w / full_w, h / full_h)

        if self.led is not None and cfg.flash_percent > 0:
            with self.led.flash(cfg.flash_percent):
                img = self.source.grab(full_w, full_h, crop)
        else:
            img = self.source.grab(full_w, full_h, crop)

        # Correct before cropping to the output size: the model is defined
        # over the whole frame, so applying it to an already-cropped window
        # would use the wrong radii and bend the picture the wrong way.
        img = self._maybe_undistort(img, cfg)
        if hasattr(img, "size") and tuple(img.size) != (ow, oh):
            img = img.resize((ow, oh))

        t0 = time.monotonic()
        data, ext = encode_image(img, cfg.image_compression)
        encode_s = time.monotonic() - t0

        rec = self.store.add(data, "image", ow, oh,
                             IMAGE_NAMES.get(cfg.image_compression, "?"), ext)
        log.info("captured region %d: %dx%d at (%d,%d) from %dx%d -> %dx%d, "
                 "%d B, encoded in %.2fs", rec.media_id, w, h, x, y,
                 full_w, full_h, ow, oh, rec.size, encode_s)
        return rec

    def start_record(self, cfg: Config) -> int:
        width, height = RESOLUTIONS.get(cfg.video_resolution, (1920, 1080))
        rec = self.store.add(b"", "video", width, height,
                             VIDEO_NAMES.get(cfg.video_compression, "?"), "mp4")
        self._recording = rec
        self._record_started = time.monotonic()
        return rec.media_id

    def stop_record(self) -> MediaRecord:
        rec = self._recording
        self._recording = None
        if rec is not None:
            rec.duration_s = time.monotonic() - self._record_started
            self.store._save()
        return rec
