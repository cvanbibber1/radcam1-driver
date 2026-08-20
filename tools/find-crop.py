#!/usr/bin/env python3
"""Find the usable part of the frame, and store it on the camera.

The lens does not illuminate the whole sensor evenly: corners fall off, and
beyond the image circle they go black. Transmitting those pixels wastes link
time that this payload does not have - at 88 kB/s a full 12 MP JPEG costs 22
seconds, and a quarter of that frame may carry nothing.

This measures where the image actually is and proposes a crop. Because lens
alignment varies from module to module, the result is a *per-camera* property
and is stored on the camera's own EEPROM alongside the colour calibration, so
it follows the module between Pis.

    python3 tools/find-crop.py                    # measure and report
    sudo python3 tools/find-crop.py --store       # write it to the EEPROM
    python3 tools/find-crop.py --threshold 0.3

Point the camera at an evenly lit, bright surface - a white wall or a sheet of
paper - so the falloff being measured is the lens's, not the scene's.
"""

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from radcam.eeprom import CameraEEPROM  # noqa: E402


def capture(width, height, timeout_ms):
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "f.jpg"
        subprocess.run(
            ["rpicam-still", "-n", "--width", str(width), "--height",
             str(height), "-t", str(timeout_ms), "-o", str(out)],
            check=True, capture_output=True, timeout=180)
        return Image.open(out).copy()


def check_scene(gray: np.ndarray) -> list[str]:
    """Reject scenes this method cannot measure meaningfully.

    Vignetting is only separable from scene content when the target is bright
    and flat. On an ordinary scene the falloff being measured would be the
    subject, not the lens, so refuse rather than return a confident wrong crop.
    """
    problems = []
    H, W = gray.shape
    centre = gray[H // 3:2 * H // 3, W // 3:2 * W // 3]

    if centre.mean() < 90:
        problems.append(f"centre is dark ({centre.mean():.0f}/255) - "
                        "aim at a brightly lit white surface")
    if (gray >= 254).mean() > 0.02:
        problems.append(f"{(gray>=254).mean()*100:.1f}% clipped - "
                        "reduce the light or exposure")
    # A flat field has low relative spread across the central region.
    rel = float(centre.std() / max(centre.mean(), 1e-6))
    if rel > 0.18:
        problems.append(f"centre is not uniform (rel. spread {rel:.2f}) - "
                        "the scene has detail; use a blank surface")
    return problems


def find_usable(gray: np.ndarray, threshold: float):
    """Largest axis-aligned box whose border rows/cols are mostly lit."""
    H, W = gray.shape
    peak = float(np.percentile(gray, 99))
    if peak <= 0:
        raise SystemExit("frame is black")
    limit = peak * threshold
    lit = gray >= limit

    # Shrink each edge inwards while that edge is mostly dark. Requiring 90%
    # of a row to be lit tolerates noise without accepting a half-shadowed
    # edge. Stop well before collapse: crossing that guard means the scene is
    # not a flat field and the answer would be meaningless anyway.
    min_w, min_h = W // 8, H // 8
    top, bottom, left, right = 0, H - 1, 0, W - 1
    while bottom - top > min_h and lit[top, left:right + 1].mean() < 0.9:
        top += 1
    while bottom - top > min_h and lit[bottom, left:right + 1].mean() < 0.9:
        bottom -= 1
    while right - left > min_w and lit[top:bottom + 1, left].mean() < 0.9:
        left += 1
    while right - left > min_w and lit[top:bottom + 1, right].mean() < 0.9:
        right -= 1

    collapsed = (bottom - top <= min_h) or (right - left <= min_w)
    return left, top, right - left + 1, bottom - top + 1, peak, limit, collapsed


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=4096)
    ap.add_argument("--height", type=int, default=3072)
    ap.add_argument("-t", "--timeout", type=int, default=4000)
    ap.add_argument("--threshold", type=float, default=0.25,
                    help="fraction of peak brightness counted as lit")
    ap.add_argument("--margin", type=float, default=0.01,
                    help="extra safety margin to trim, as a fraction")
    ap.add_argument("--store", action="store_true",
                    help="write the crop to the camera EEPROM")
    ap.add_argument("--bus", type=int, default=4)
    args = ap.parse_args()

    print(f"capturing {args.width}x{args.height}...")
    im = capture(args.width, args.height, args.timeout)
    g = np.asarray(im.convert("L")).astype(np.float32)
    H, W = g.shape

    problems = check_scene(g)
    if problems:
        print("\nSCENE UNSUITABLE for measuring lens falloff:")
        for p in problems:
            print(f"  - {p}")
        print("\nPoint the camera at an evenly lit blank white surface that")
        print("fills the frame, then run this again.")
        return 1

    x, y, w, h, peak, limit, collapsed = find_usable(g, args.threshold)
    if collapsed:
        print("\nMeasurement collapsed - the frame has no clear lit region.")
        print("This method needs a bright, flat, evenly lit target.")
        return 1

    # Trim a little more so the crop never grazes the falloff.
    mx, my = int(W * args.margin), int(H * args.margin)
    x, y = x + mx, y + my
    w, h = max(w - 2 * mx, 2), max(h - 2 * my, 2)

    # Even coordinates keep the Bayer phase intact.
    x, y, w, h = x & ~1, y & ~1, w & ~1, h & ~1

    fx, fy, fw, fh = x / W, y / H, w / W, h / H
    area = 100.0 * (w * h) / (W * H)

    print(f"\n  frame            : {W}x{H}")
    print(f"  peak luma        : {peak:.0f}   lit threshold {limit:.0f} "
          f"({args.threshold:.0%})")
    print(f"  corner luma      : "
          f"TL={g[:H//8,:W//8].mean():.0f} TR={g[:H//8,-W//8:].mean():.0f} "
          f"BL={g[-H//8:,:W//8].mean():.0f} BR={g[-H//8:,-W//8:].mean():.0f}")
    print(f"  centre luma      : {g[H//3:2*H//3, W//3:2*W//3].mean():.0f}")
    print()
    print(f"  usable region    : {w}x{h} at ({x},{y})")
    print(f"  as a fraction    : {fx:.4f},{fy:.4f},{fw:.4f},{fh:.4f}")
    print(f"  keeps            : {area:.1f}% of the frame")

    if area > 98:
        print("\n  The whole frame is usable - no crop needed. If you expected")
        print("  vignetting, check the scene really is evenly lit and bright.")
    else:
        saved = 100 - area
        print(f"\n  Cropping saves roughly {saved:.0f}% of every frame.")
        print(f"  Capture it 1:1 with:")
        print(f"    rpicam-still --roi {fx:.4f},{fy:.4f},{fw:.4f},{fh:.4f} "
              f"--width {w} --height {h} -o out.jpg")

    if args.store:
        ee = CameraEEPROM(bus=args.bus)
        if not ee.present():
            print("\nno camera EEPROM found")
            return 1
        rec = ee.load() or {"schema": 1, "sensor": "ar1335"}
        rec["crop"] = {"x": round(fx, 5), "y": round(fy, 5),
                       "w": round(fw, 5), "h": round(fh, 5),
                       "native": [W, H], "pixels": [x, y, w, h]}
        ee.store(rec)
        print(f"\nstored on the camera EEPROM; it now travels with the module")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
