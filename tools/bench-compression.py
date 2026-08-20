#!/usr/bin/env python3
"""Measure real compression sizes and encode times on this hardware.

protocol.md §1 sized the link budget from estimates. This replaces the encode
times with measurements, which are valid regardless of what the sensor shows,
and gives indicative sizes from a synthetic frame.

The sizes are honest but *indicative*: a synthetic pattern does not compress
identically to a real scene. Encode *time* and the H.264 throughput ratio are
properties of this CPU and are directly usable. Re-run against real frames once
the AR1335 responds.

    python3 tools/bench-compression.py
"""

import json
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from radcam.camera import SyntheticSource, encode_image          # noqa: E402
from radcam.protocol import (IMAGE_NAMES, LINK_BYTES_PER_S,      # noqa: E402
                             VIDEO_BITRATES, VIDEO_NAMES)

RESOLUTIONS = [(4096, 3072, "12.6 MP full"), (1920, 1080, "2.1 MP binned")]


def fmt_size(n: int) -> str:
    return f"{n/1e6:.2f} MB" if n >= 1e6 else f"{n/1e3:.0f} KB"


def fmt_time(s: float) -> str:
    return f"{int(s//60)} min {s%60:4.1f} s" if s >= 60 else f"{s:.1f} s"


def bench_images() -> list[dict]:
    src = SyntheticSource()
    rows = []

    for width, height, label in RESOLUTIONS:
        print(f"\n  generating {width}x{height} test frame...", flush=True)
        t0 = time.monotonic()
        img = src.grab(width, height)
        print(f"    frame generated in {time.monotonic()-t0:.1f}s")

        for profile in sorted(IMAGE_NAMES):
            t0 = time.monotonic()
            data, ext = encode_image(img, profile)
            encode_s = time.monotonic() - t0
            tx_s = len(data) / LINK_BYTES_PER_S

            rows.append({
                "resolution": f"{width}x{height}", "label": label,
                "profile": IMAGE_NAMES[profile], "bytes": len(data),
                "encode_s": encode_s, "transmit_s": tx_s,
            })
            print(f"    {IMAGE_NAMES[profile]:<13} {fmt_size(len(data)):>9}  "
                  f"encode {encode_s:5.2f}s  transmit {fmt_time(tx_s)}")
    return rows


def bench_video() -> list[dict]:
    """Encode a short clip to measure software H.264 throughput.

    The Pi 5 has no hardware H.264 encoder, so this is libx264 on the CPU and
    the encode rate is a real constraint on continuous recording.
    """
    if not shutil.which("ffmpeg"):
        print("  ffmpeg missing, skipping video benchmark")
        return []

    rows = []
    src = SyntheticSource()
    width, height, fps, n_frames = 1920, 1080, 30, 30    # 1 second of 1080p30

    with tempfile.TemporaryDirectory() as tmp:
        tmpdir = Path(tmp)
        print(f"\n  rendering {n_frames} frames of {width}x{height}...",
              flush=True)
        # Frames MUST differ. Encoding identical frames makes inter-frame
        # prediction trivial and the output collapses to almost nothing, which
        # would badly understate the real bitrate. Pan the content instead.
        base = src.grab(width + 64, height)
        for i in range(n_frames):
            off = (i * 2) % 64
            base.crop((off, 0, off + width, height)).save(tmpdir / f"f{i:05d}.png")

        for profile in sorted(VIDEO_NAMES):
            out = tmpdir / f"out{profile}.mp4"
            bitrate = VIDEO_BITRATES[profile]
            t0 = time.monotonic()
            subprocess.run(
                ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
                 "-framerate", str(fps), "-i", str(tmpdir / "f%05d.png"),
                 "-c:v", "libx264", "-preset", "veryfast",
                 "-b:v", str(bitrate), "-maxrate", str(bitrate),
                 "-bufsize", str(bitrate * 2), "-pix_fmt", "yuv420p",
                 str(out)], check=True, capture_output=True)
            encode_s = time.monotonic() - t0
            size = out.stat().st_size
            clip_s = n_frames / fps
            achieved_bps = size * 8 / clip_s

            # Plan from the CONFIGURED bitrate, not this clip's size: a short
            # synthetic clip never fully exercises the rate controller, so its
            # size understates sustained real-world output. The configured
            # bitrate is what the encoder will produce on real scenes.
            transmit_ratio = bitrate / (LINK_BYTES_PER_S * 8)

            rows.append({
                "profile": VIDEO_NAMES[profile], "bitrate": bitrate,
                "bytes": size, "achieved_bps": achieved_bps,
                "clip_s": clip_s, "encode_s": encode_s,
                "encode_ratio": encode_s / clip_s,
                "transmit_ratio": transmit_ratio,
            })
            print(f"    {VIDEO_NAMES[profile]:<13} {bitrate/1000:>5.0f} kbit/s "
                  f"target, {achieved_bps/1000:>5.0f} kbit/s on this clip  "
                  f"encode {encode_s:5.2f}s ({encode_s/clip_s:.2f}x realtime)  "
                  f"transmit {transmit_ratio:.2f}x realtime")
    return rows


def main() -> int:
    print("radcam compression benchmark")
    print(f"link budget: {LINK_BYTES_PER_S} B/s "
          f"({LINK_BYTES_PER_S*8/1000:.0f} kbit/s)")

    print("\n=== STILL IMAGES ===")
    images = bench_images()

    print("\n=== VIDEO (software libx264 - the Pi 5 has no HW H.264) ===")
    video = bench_video()

    out = Path(__file__).resolve().parent.parent / "logs" / "compression-bench.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps({"images": images, "video": video}, indent=2))
    print(f"\nresults written to {out}")

    print("\n=== CONCLUSIONS ===")
    for r in video:
        if r["encode_ratio"] > 1.0:
            print(f"  {r['profile']}: encode is {r['encode_ratio']:.2f}x "
                  f"realtime - CANNOT sustain continuous recording on this CPU")
        elif r["transmit_ratio"] > 1.0:
            print(f"  {r['profile']}: encodes fast enough, but transmit is "
                  f"{r['transmit_ratio']:.2f}x realtime - finite clips only")
        else:
            print(f"  {r['profile']}: encode {r['encode_ratio']:.2f}x, "
                  f"transmit {r['transmit_ratio']:.2f}x - sustainable "
                  f"continuously")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
