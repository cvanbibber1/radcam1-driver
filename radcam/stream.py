"""Live video for the HRT channel.

A livestream is not a file transfer with the end left off. The difference that
drives every decision here is what to do when production outruns the link:

* A **file transfer** must not lose a byte, so it queues and the queue drains
  in its own time. Latency does not matter; completeness does.
* A **livestream** must not fall behind, so it discards. Completeness does not
  matter; latency does. A stream that buffers whatever it cannot send is a
  stream whose delay grows without bound - after ten minutes of a closed HRT
  tap you are watching ten-minute-old video, which is worse than useless on a
  spacecraft.

So frames go into a bounded ring and the **oldest is dropped** when it fills.
A rising drop count under load is correct operation, not a fault.

Dropping requires whole frames, which is why this module parses the encoder's
Annex-B output into access units rather than treating it as an opaque byte
stream. Discarding an arbitrary slice of bytes would corrupt everything up to
the next keyframe; discarding a whole frame costs exactly that frame.

SPS and PPS are repeated ahead of every keyframe, so a ground station that
joins mid-stream - or rejoins after DICE closed the tap - can start decoding at
the next keyframe instead of waiting for a parameter set that already went past.

## Why two processes

`rpicam-vid --codec h264` does not work on this board, and the reason is worth
recording because it is not obvious: the Pi 5 dropped the hardware H.264
encoder the Pi 4 had, there is no `/dev/video11`, and this build of rpicam-apps
was compiled without libav support. `rpicam-vid` answers
"Unable to find an appropriate H.264 codec" and exits.

So the camera produces raw YUV420 and **ffmpeg/libx264 encodes it in software**
- the same encoder `tools/bench-compression.py` measured. Two processes in a
pipe, both owned and torn down together.

x264 is told `sliced-threads=0` and `threads=1`. Sliced threading splits each
picture into one slice per core, which at 640x480 buys nothing and cost real
confusion during bring-up: it produced four VCL NALs per frame and made a naive
access-unit parser report 55 fps when the true rate was 14. The parser below
handles multi-slice pictures correctly regardless, but a single slice per frame
is simpler to reason about and lower latency.

Measured on this board at 640x480, 15 fps, 600 kbit/s requested:

    13.8 fps sustained, 592 kbit/s, one keyframe per second, 1 slice per frame

## Region of interest

The sensor is 4208x3120 and the link carries perhaps 600 kbit/s. Streaming the
whole frame at that rate wastes almost all of it on detail that is destroyed by
the encoder anyway. Instead the stream takes a **crop of `crop_w` x `crop_h`
centred on a chosen sensor pixel**, and scales it to the output size. Setting
the crop equal to the output gives 1:1 sensor pixels - the region of interest at
full native resolution, with nothing spent on the rest of the field.
"""

from __future__ import annotations

import logging
import shlex
import shutil
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, replace

log = logging.getLogger(__name__)

__all__ = ["StreamConfig", "VideoStream", "SENSOR_WIDTH", "SENSOR_HEIGHT",
           "EncodedFrame"]

#: AR1335 native array. The ROI is expressed as a fraction of this.
SENSOR_WIDTH = 4208
SENSOR_HEIGHT = 3120

#: Bounds on what the ground may ask for. Wide enough to be useful, narrow
#: enough that a corrupted command cannot ask the encoder for something absurd.
MIN_DIMENSION = 64
MAX_DIMENSION = 1920
MAX_FPS = 30
MIN_BITRATE = 50_000
MAX_BITRATE = 8_000_000

_VCL_TYPES = (1, 5)          # coded slice, and coded slice of an IDR picture
_PARAM_TYPES = (7, 8)        # SPS, PPS


def _clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


@dataclass(frozen=True)
class StreamConfig:
    """Everything the ground can set about the stream.

    Defaults are the mission's target: 640x480 at 15 fps and 600 kbit/s, with
    the crop equal to the output so those are native sensor pixels.
    """

    width: int = 640
    height: int = 480
    fps: int = 15
    bitrate: int = 600_000
    centre_x: int = SENSOR_WIDTH // 2
    centre_y: int = SENSOR_HEIGHT // 2
    crop_w: int = 640
    crop_h: int = 480

    def sanitised(self) -> "StreamConfig":
        """Clamp into range, and keep the crop box inside the sensor.

        Clamping rather than rejecting is deliberate: a command that asks for a
        box hanging off the edge of the sensor has a sensible nearest answer,
        and refusing it would leave the ground guessing. The effective values
        go out in telemetry, so what was applied is never in doubt.
        """
        width = _clamp(self.width, MIN_DIMENSION, MAX_DIMENSION) & ~1
        height = _clamp(self.height, MIN_DIMENSION, MAX_DIMENSION) & ~1
        crop_w = _clamp(self.crop_w or width, MIN_DIMENSION, SENSOR_WIDTH)
        crop_h = _clamp(self.crop_h or height, MIN_DIMENSION, SENSOR_HEIGHT)

        # The centre must leave room for half the box on each side.
        half_w, half_h = crop_w // 2, crop_h // 2
        centre_x = _clamp(self.centre_x, half_w, SENSOR_WIDTH - half_w)
        centre_y = _clamp(self.centre_y, half_h, SENSOR_HEIGHT - half_h)

        return replace(self, width=width, height=height,
                       fps=_clamp(self.fps, 1, MAX_FPS),
                       bitrate=_clamp(self.bitrate, MIN_BITRATE, MAX_BITRATE),
                       crop_w=crop_w, crop_h=crop_h,
                       centre_x=centre_x, centre_y=centre_y)

    @property
    def roi(self) -> tuple[float, float, float, float]:
        """The crop as normalised fractions, which is what rpicam-vid wants."""
        x = (self.centre_x - self.crop_w / 2) / SENSOR_WIDTH
        y = (self.centre_y - self.crop_h / 2) / SENSOR_HEIGHT
        return (max(0.0, x), max(0.0, y),
                min(1.0, self.crop_w / SENSOR_WIDTH),
                min(1.0, self.crop_h / SENSOR_HEIGHT))

    @property
    def native_pixels(self) -> bool:
        """True when the crop is not being scaled - maximum detail."""
        return self.crop_w == self.width and self.crop_h == self.height

    def describe(self) -> str:
        return (f"{self.width}x{self.height}@{self.fps}fps "
                f"{self.bitrate // 1000}kbit/s, crop {self.crop_w}x{self.crop_h} "
                f"at ({self.centre_x},{self.centre_y})"
                f"{' native' if self.native_pixels else ' scaled'}")


@dataclass
class EncodedFrame:
    index: int
    data: bytes
    keyframe: bool
    produced: float


class VideoStream:
    """Runs the encoder and hands out whole frames, newest-first priority.

    The encoder is a child process; if it dies the stream reports a fault and
    keeps the rest of the payload running. Nothing here is allowed to raise
    into the command path.
    """

    def __init__(self, config: StreamConfig | None = None,
                 queue_frames: int = 8, camera: str = "rpicam-vid",
                 encoder: str = "ffmpeg"):
        self.config = (config or StreamConfig()).sanitised()
        #: Bounded on purpose. Eight frames is about half a second at 15 fps -
        #: enough to ride out a brief stall, short enough that what arrives is
        #: still recognisably live.
        self.queue_frames = queue_frames
        self.camera = camera
        self.encoder = encoder

        self._camera_proc: subprocess.Popen | None = None
        self._proc: subprocess.Popen | None = None
        self._reader: threading.Thread | None = None
        self._frames: deque[EncodedFrame] = deque(maxlen=queue_frames)
        self._lock = threading.Lock()
        self._running = threading.Event()

        self.frames_encoded = 0
        self.frames_dropped = 0
        self.frames_taken = 0
        self.bytes_encoded = 0
        self.fault: str | None = None
        self.started_at = 0.0

    # -- lifecycle -------------------------------------------------------

    @property
    def available(self) -> bool:
        return (shutil.which(self.camera) is not None
                and shutil.which(self.encoder) is not None)

    def missing(self) -> list[str]:
        return [tool for tool in (self.camera, self.encoder)
                if shutil.which(tool) is None]

    @property
    def running(self) -> bool:
        return self._running.is_set() and self._proc is not None \
            and self._proc.poll() is None

    def _camera_command(self) -> list[str]:
        """Camera to raw YUV420 on stdout, cropped to the region of interest."""
        cfg = self.config
        x, y, w, h = cfg.roi
        return [
            self.camera, "-n", "-t", "0",
            "--codec", "yuv420",
            "--width", str(cfg.width), "--height", str(cfg.height),
            "--framerate", str(cfg.fps),
            "--roi", f"{x:.5f},{y:.5f},{w:.5f},{h:.5f}",
            "-o", "-",
        ]

    def _encoder_command(self) -> list[str]:
        """Raw YUV420 in, Annex-B H.264 out, tuned for latency not ratio."""
        cfg = self.config
        # keyint bounds how long a ground station has to wait to join, and how
        # long a dropped frame can corrupt the picture for: one second.
        params = (f"sliced-threads=0:threads=1:scenecut=0"
                  f":keyint={cfg.fps}:min-keyint={cfg.fps}")
        return [
            self.encoder, "-hide_banner", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", "yuv420p",
            "-s", f"{cfg.width}x{cfg.height}", "-r", str(cfg.fps), "-i", "-",
            "-c:v", "libx264", "-preset", "veryfast", "-tune", "zerolatency",
            "-x264-params", params,
            "-b:v", str(cfg.bitrate), "-maxrate", str(cfg.bitrate),
            "-bufsize", str(cfg.bitrate * 2),
            "-bf", "0",              # no B-frames: they only add latency here
            "-f", "h264", "-",
        ]

    def describe_pipeline(self) -> str:
        return (" ".join(shlex.quote(a) for a in self._camera_command())
                + " | " + " ".join(shlex.quote(a) for a in self._encoder_command()))

    def start(self, config: StreamConfig | None = None) -> bool:
        """Start, or restart with new settings. Returns False on failure."""
        if config is not None:
            self.config = config.sanitised()
        self.stop()

        missing = self.missing()
        if missing:
            self.fault = f"not installed: {', '.join(missing)}"
            log.error("cannot start stream: %s", self.fault)
            return False

        try:
            self._camera_proc = subprocess.Popen(
                self._camera_command(), stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL, bufsize=0)
            self._proc = subprocess.Popen(
                self._encoder_command(), stdin=self._camera_proc.stdout,
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
            # The encoder owns the read end now; if this process kept it open,
            # the camera would never see a broken pipe when the encoder dies.
            self._camera_proc.stdout.close()
        except Exception as exc:                       # noqa: BLE001
            self.fault = str(exc)
            log.error("cannot start stream: %s", exc)
            self._terminate()
            return False

        self.fault = None
        self.started_at = time.monotonic()
        self._running.set()
        self._reader = threading.Thread(target=self._read, name="stream-read",
                                        daemon=True)
        self._reader.start()
        log.info("stream started: %s", self.config.describe())
        return True

    def _terminate(self) -> None:
        """Tear down both halves of the pipeline, whatever state they are in."""
        for attr in ("_proc", "_camera_proc"):
            proc = getattr(self, attr)
            if proc is None:
                continue
            try:
                proc.terminate()
                proc.wait(timeout=2.0)
            except Exception:                          # noqa: BLE001
                try:
                    proc.kill()
                    proc.wait(timeout=1.0)
                except Exception:                      # noqa: BLE001
                    pass
            setattr(self, attr, None)

    def stop(self) -> bool:
        was_running = self.running
        self._running.clear()
        self._terminate()

        if self._reader is not None:
            self._reader.join(timeout=2.0)
            self._reader = None

        with self._lock:
            self._frames.clear()
        if was_running:
            log.info("stream stopped")
        return was_running

    # -- encoder output --------------------------------------------------

    @staticmethod
    def _starts_picture(unit: bytes) -> bool:
        """Does this VCL NAL begin a new picture?

        The slice header opens with `first_mb_in_slice` as an Exp-Golomb
        value, and only the first slice of a picture has it at zero. ue(0) is
        encoded as a single set bit, so a new picture is exactly the case where
        the top bit of the first payload byte is set.

        Checking this rather than assuming one slice per frame is what makes
        the parser correct if x264 is ever configured with sliced threading -
        where four slices per picture would otherwise look like four frames.
        """
        return len(unit) >= 2 and bool(unit[1] & 0x80)

    def _read(self) -> None:
        """Split the Annex-B bytestream into access units.

        A new access unit begins at the first slice of a new picture; parameter
        sets that precede it belong to the frame that follows, not the one
        before, which is what lets a receiver join mid-stream at a keyframe.
        """
        buffer = bytearray()
        pending: list[bytes] = []
        seen_slice = False

        try:
            stream = self._proc.stdout
            while self._running.is_set():
                block = stream.read(65536)
                if not block:
                    break
                buffer.extend(block)

                # Keep the last few bytes: a start code may straddle reads.
                units, buffer = self._split(buffer)
                for unit in units:
                    nal_type = unit[0] & 0x1F if unit else 0
                    if nal_type in _VCL_TYPES:
                        if seen_slice and self._starts_picture(unit):
                            self._emit(pending)
                            pending = []
                        seen_slice = True
                    elif nal_type in _PARAM_TYPES and seen_slice:
                        # Parameter sets introduce the *next* frame.
                        self._emit(pending)
                        pending = []
                        seen_slice = False
                    pending.append(unit)
        except Exception as exc:                       # noqa: BLE001
            if self._running.is_set():
                self.fault = str(exc)
                log.error("stream reader failed: %s", exc)
        finally:
            if pending:
                self._emit(pending)
            if self._running.is_set():
                self.fault = self.fault or "encoder exited"
                log.warning("stream encoder ended unexpectedly")

    @staticmethod
    def _split(buffer: bytearray) -> tuple[list[bytes], bytearray]:
        """Pull complete NAL units out, keeping any partial tail."""
        units: list[bytes] = []
        starts: list[tuple[int, int]] = []
        i, n = 0, len(buffer)
        # A 3-byte start code needs bytes i..i+2, so scan while i <= n-3. The
        # 4-byte form needs one more, guarded separately. Getting these bounds
        # too tight is harmless but real: a start code sitting exactly at the
        # end of a read went undetected, deferring a whole NAL to the next one.
        while i < n - 2:
            if buffer[i] == 0 and buffer[i + 1] == 0:
                if buffer[i + 2] == 1:
                    starts.append((i, 3))
                    i += 3
                    continue
                if buffer[i + 2] == 0 and i + 3 < n and buffer[i + 3] == 1:
                    starts.append((i, 4))
                    i += 4
                    continue
            i += 1

        if not starts:
            return units, buffer

        for index in range(len(starts) - 1):
            offset, length = starts[index]
            end = starts[index + 1][0]
            unit = bytes(buffer[offset + length:end])
            if unit:
                units.append(unit)

        # The final start code may head an incomplete unit; keep it for later.
        return units, bytearray(buffer[starts[-1][0]:])

    def _emit(self, units: list[bytes]) -> None:
        if not units:
            return
        keyframe = any((u[0] & 0x1F) in (5, 7) for u in units if u)
        data = b"".join(b"\x00\x00\x00\x01" + u for u in units)

        with self._lock:
            if len(self._frames) == self._frames.maxlen:
                # The ring is full, so the oldest frame is about to fall out.
                # That is the whole design: a livestream discards rather than
                # delays. Count it so the ground can see the link is saturated.
                self.frames_dropped += 1
            self._frames.append(EncodedFrame(self.frames_encoded, data,
                                             keyframe, time.monotonic()))
        self.frames_encoded += 1
        self.bytes_encoded += len(data)

    # -- consumption -----------------------------------------------------

    def take(self) -> EncodedFrame | None:
        """Take the oldest queued frame, or None if nothing is ready."""
        with self._lock:
            if not self._frames:
                return None
            self.frames_taken += 1
            return self._frames.popleft()

    def flush(self, keep_keyframe: bool = False) -> int:
        """Discard queued frames - used when HRT closes.

        Holding frames while the tap is shut is exactly the latency trap this
        module exists to avoid, so by default everything goes.
        """
        with self._lock:
            if keep_keyframe:
                newest = next((f for f in reversed(self._frames)
                               if f.keyframe), None)
                dropped = len(self._frames) - (1 if newest else 0)
                self._frames.clear()
                if newest:
                    self._frames.append(newest)
            else:
                dropped = len(self._frames)
                self._frames.clear()
        self.frames_dropped += dropped
        return dropped

    @property
    def queue_depth(self) -> int:
        with self._lock:
            return len(self._frames)

    def encoder_late(self) -> bool:
        """Is the encoder producing fewer frames than it was asked for?"""
        if not self.running or self.started_at <= 0:
            return False
        elapsed = time.monotonic() - self.started_at
        if elapsed < 2.0:
            return False
        expected = elapsed * self.config.fps
        return self.frames_encoded < expected * 0.8

    def status(self) -> dict:
        return {
            "stream_width": self.config.width,
            "stream_height": self.config.height,
            "stream_fps": self.config.fps,
            "stream_bitrate": self.config.bitrate,
            "stream_centre_x": self.config.centre_x,
            "stream_centre_y": self.config.centre_y,
            "stream_crop_w": self.config.crop_w,
            "stream_crop_h": self.config.crop_h,
            "stream_frames_sent": self.frames_taken,
            "stream_frames_dropped": self.frames_dropped,
            "stream_bytes_sent": self.bytes_encoded,
            "stream_queue_depth": self.queue_depth,
        }
