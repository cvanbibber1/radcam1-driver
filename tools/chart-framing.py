#!/usr/bin/env python3
"""Live feedback for framing and focusing a colour chart.

Colour calibration is only as good as the capture it is measured from, and a
soft or badly framed chart produces a confidently wrong matrix. This loops,
capturing and scoring, so the chart can be positioned and focused against real
numbers instead of guesswork.

    python3 tools/chart-framing.py            # repeat until Ctrl-C
    python3 tools/chart-framing.py --once

What it reports, and what "good" looks like:

  focus       variance of the Laplacian. Turn the lens until this peaks.
              Under ~100 is soft; a well focused chart reaches several hundred.
  hue spread  how many of 12 hue bins carry >3% of the coloured pixels. The
              chart has red, yellow, green, cyan, blue and magenta patches, so
              a good capture lights up 6 or more.
  uniformity  mean within-tile standard deviation over a 6x3 grid. Patches are
              flat, so this should fall below ~10 once the grid lines up.
  clipping    percentage of blown pixels. Keep under 1%, or the white patch
              saturates and the calibration is meaningless.

Aim for: chart filling most of the frame, square to the lens, evenly lit, no
glare on the patches.
"""

import argparse
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
from PIL import Image


def capture(width, height, timeout_ms):
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "f.jpg"
        subprocess.run(
            ["rpicam-still", "-n", "--width", str(width), "--height",
             str(height), "-t", str(timeout_ms), "-o", str(out)],
            check=True, capture_output=True, timeout=120)
        return Image.open(out).copy()


def score(im):
    a = np.asarray(im.convert("RGB")).astype(np.float32)
    g = np.asarray(im.convert("L")).astype(np.float32)
    H, W = g.shape

    lap = (g[:-2, 1:-1] + g[2:, 1:-1] + g[1:-1, :-2] + g[1:-1, 2:]
           - 4 * g[1:-1, 1:-1])
    focus = float(lap.var())

    hsv = np.asarray(im.convert("HSV")).astype(np.float32)
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    strong = hue[(sat > 80) & (val > 40)]
    if strong.size:
        hist, _ = np.histogram(strong, bins=12, range=(0, 256))
        pct = 100.0 * hist / strong.size
        bins = int((pct > 3).sum())
    else:
        pct = np.zeros(12)
        bins = 0

    # Within-tile uniformity over a 6x3 grid covering the central 80%.
    y0, y1 = int(H * 0.1), int(H * 0.9)
    x0, x1 = int(W * 0.1), int(W * 0.9)
    stds = []
    for r in range(3):
        for c in range(6):
            ty0 = y0 + (y1 - y0) * r // 3
            ty1 = y0 + (y1 - y0) * (r + 1) // 3
            tx0 = x0 + (x1 - x0) * c // 6
            tx1 = x0 + (x1 - x0) * (c + 1) // 6
            t = a[ty0 + 10:ty1 - 10, tx0 + 10:tx1 - 10]
            if t.size:
                stds.append(float(t.std(axis=(0, 1)).mean()))
    uniformity = float(np.mean(stds)) if stds else 99.0

    clip = float((g >= 254).mean() * 100)
    return {"focus": focus, "hue_bins": bins, "hue_pct": pct,
            "uniformity": uniformity, "clip": clip, "mean": float(g.mean())}


def verdict(s):
    problems = []
    if s["focus"] < 100:
        problems.append("SOFT - adjust focus")
    if s["hue_bins"] < 6:
        problems.append(f"only {s['hue_bins']}/12 hue bins - chart not in frame?")
    if s["uniformity"] > 12:
        problems.append("tiles not uniform - align the grid / fill the frame")
    if s["clip"] > 1:
        problems.append(f"{s['clip']:.1f}% clipped - reduce light or exposure")
    if s["mean"] < 40:
        problems.append("very dark - add light")
    return problems


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("-t", "--timeout", type=int, default=2500)
    ap.add_argument("--once", action="store_true")
    args = ap.parse_args()

    best = 0.0
    print("framing/focus aid - Ctrl-C to stop\n")
    print(f"{'focus':>9} {'hue bins':>9} {'uniform':>8} {'clip%':>7} {'mean':>6}   verdict")
    print("-" * 78)
    while True:
        try:
            s = score(capture(args.width, args.height, args.timeout))
        except subprocess.CalledProcessError as exc:
            print("capture failed:", exc)
            return 1
        best = max(best, s["focus"])
        problems = verdict(s)
        mark = "  <-- best so far" if s["focus"] >= best else ""
        print(f"{s['focus']:9.1f} {s['hue_bins']:6d}/12 {s['uniformity']:8.1f} "
              f"{s['clip']:7.2f} {s['mean']:6.1f}   "
              + ("GOOD - ready to calibrate" if not problems
                 else "; ".join(problems)) + mark)
        if args.once:
            return 0 if not problems else 1
        time.sleep(0.3)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nstopped")
