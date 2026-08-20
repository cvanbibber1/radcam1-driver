#!/usr/bin/env python3
"""End-to-end test of everything downstream of the image sensor.

Capture -> compress -> store -> MEDIA_INFO -> chunked MEDIA_DATA -> RESEND ->
reassembly -> whole-file CRC, driven through the real protocol dispatcher over
a lossy simulated link.

The point is to prove that when the AR1335 finally answers on I2C, nothing
else is in the way. `SyntheticSource` stands in for the sensor; every other
component - encoder, media store, dispatcher, framing, chunking, ARQ - is the
real one, so swapping in `LibcameraSource` is the only change needed.

    python3 tools/test-capture-pipeline.py
"""

import random
import struct
import sys
import tempfile
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from radcam.camera import Camera, MediaStore, SyntheticSource   # noqa: E402
from radcam.framing import Frame, FrameReader, encode_frame     # noqa: E402
from radcam.protocol import (Cfg, Config, Dispatcher, Err,      # noqa: E402
                             IMAGE_NAMES, Msg, RESOLUTIONS)

FAILURES = 0


def check(name, cond, extra=""):
    global FAILURES
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{(' — ' + extra) if extra else ''}")
    if not cond:
        FAILURES += 1


def lossy(frames, loss=0.03, seed=None):
    """Serialise frames, drop some, reassemble - a real lossy channel."""
    rnd = random.Random(seed)
    reader = FrameReader()
    out = []
    for f in frames:
        wire = encode_frame(f)
        if rnd.random() < loss:
            continue
        out.extend(reader.feed(wire))
    return out


def transfer(dispatcher, media_id, loss=0.03, seed=1):
    """Run a full media transfer with loss and one RESEND round."""
    frames = dispatcher.handle(
        Frame(Msg.REQUEST_MEDIA, 100, struct.pack("<I", media_id)))
    if frames[0].type != Msg.MEDIA_INFO:
        return None, None, None

    mid, size, n_chunks, eta = struct.unpack("<IQIf", frames[0].payload)
    end_crc = struct.unpack("<II", frames[-1].payload)[1]

    pieces = {}
    for f in lossy(frames[1:-1], loss=loss, seed=seed):
        _m, idx, ln = struct.unpack("<IIH", f.payload[:10])
        pieces[idx] = f.payload[10:10 + ln]

    missing = sorted(set(range(n_chunks)) - set(pieces))
    rounds = 0
    while missing and rounds < 5:
        rounds += 1
        req = struct.pack("<I", media_id) + b"".join(
            struct.pack("<I", i) for i in missing)
        for f in lossy(dispatcher.handle(Frame(Msg.RESEND, 101, req)),
                       loss=loss / 2, seed=seed + rounds):
            _m, idx, ln = struct.unpack("<IIH", f.payload[:10])
            pieces[idx] = f.payload[10:10 + ln]
        missing = sorted(set(range(n_chunks)) - set(pieces))

    if missing:
        return None, None, None
    data = b"".join(pieces[i] for i in range(n_chunks))
    return data, end_crc, (size, n_chunks, eta, rounds)


def main() -> int:
    print("Capture pipeline end-to-end (synthetic sensor, everything else real)\n")

    with tempfile.TemporaryDirectory() as tmp:
        store = MediaStore(tmp)
        camera = Camera(source=SyntheticSource(), store=store, led=None)
        cfg = Config()
        disp = Dispatcher(config=cfg, camera=camera, store=store)

        check("synthetic source reports available", camera.available())

        # Capture one still per compression profile at 1080p (fast enough to
        # run as a regression test; the 12 MP path is the same code).
        cfg.image_resolution = 1                     # 1920x1080
        captured = []
        print("\nCapture and compress, all profiles:")
        for profile in sorted(IMAGE_NAMES):
            cfg.image_compression = profile
            rec = camera.capture_image(cfg)
            captured.append(rec)
            w, h = RESOLUTIONS[cfg.image_resolution]
            ok = rec.size > 0 and rec.width == w and rec.height == h
            check(f"{IMAGE_NAMES[profile]:<13} captured",
                  ok, f"{rec.size/1000:.0f} KB, id={rec.media_id}")

        # Sizes must be monotonically non-increasing as quality drops.
        jpegs = [r for r in captured if r.compression.startswith("JPEG")]
        sizes = [r.size for r in jpegs]
        check("JPEG size decreases as quality drops",
              all(a >= b for a, b in zip(sizes, sizes[1:])),
              " > ".join(f"{s//1000}KB" for s in sizes))
        png = next(r for r in captured if r.compression == "PNG_LOSSLESS")
        check("PNG_LOSSLESS is the largest", png.size > max(sizes),
              f"{png.size/1000:.0f} KB")

        print("\nProtocol view of stored media:")
        r = disp.handle(Frame(Msg.GET_MEDIA_LIST, 1))[0]
        count = struct.unpack("<H", r.payload[:2])[0]
        check("MEDIA_LIST reports every capture", count == len(captured),
              f"{count} items")

        print("\nFull downlink of a real encoded image over a 3% loss link:")
        target = jpegs[1]                            # JPEG_MED
        original = store.read(target.media_id)
        data, end_crc, info = transfer(disp, target.media_id, loss=0.03)
        if data is None:
            check("transfer completed", False)
        else:
            size, n_chunks, eta, rounds = info
            check("MEDIA_INFO matches stored file",
                  size == len(original), f"{size} B in {n_chunks} chunks")
            check("transfer-time estimate is sane",
                  0 < eta < 60, f"{eta:.1f}s at 88 kB/s")
            check(f"recovered after {rounds} RESEND round(s)", True)
            check("reassembled file is byte-identical", data == original)
            check("whole-file CRC matches",
                  zlib.crc32(data) & 0xFFFFFFFF == end_crc, f"{end_crc:08X}")
            check("recovered bytes are a valid JPEG",
                  data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9")

        print("\nLossless profile survives the same treatment:")
        original = store.read(png.media_id)
        data, end_crc, info = transfer(disp, png.media_id, loss=0.05, seed=7)
        check("PNG reassembled byte-identical", data == original,
              f"{info[0]} B in {info[1]} chunks" if info else "failed")
        check("recovered bytes are a valid PNG",
              bool(data) and data[:8] == b"\x89PNG\r\n\x1a\n")

        print("\nHousekeeping:")
        r = disp.handle(Frame(Msg.DELETE_MEDIA, 2,
                              struct.pack("<I", target.media_id)))[0]
        check("DELETE_MEDIA frees the file", r.type == Msg.DELETE_ACK)
        r = disp.handle(Frame(Msg.REQUEST_MEDIA, 3,
                              struct.pack("<I", target.media_id)))[0]
        check("deleted media is then unknown",
              r.type == Msg.NACK and r.payload[2] == Err.NO_MEDIA)

    print(f"\n{'CAPTURE PIPELINE VERIFIED END TO END' if not FAILURES else f'{FAILURES} FAILURES'}")
    print("Only the sensor itself is missing; swapping SyntheticSource for")
    print("LibcameraSource is the sole change needed once it answers.")
    return 1 if FAILURES else 0


if __name__ == "__main__":
    raise SystemExit(main())
