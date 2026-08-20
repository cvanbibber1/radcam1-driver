#!/usr/bin/env python3
"""Measure the AR1335's raw channel ratios, for white-balance calibration.

libcamera's AWB works from a `ct_curve` in the tuning file: a list of
(colour temperature, R/G, B/G) points giving the *raw sensor* channel ratios
of a neutral white under each illuminant. Those ratios are a property of the
sensor's spectral response and its IR filter, so a curve borrowed from another
sensor is wrong - and if the real operating point falls outside the borrowed
curve, AWB extrapolates and reports nonsense colour temperatures.

This captures a raw frame, unpacks the Bayer mosaic, and reports the true
R/G and B/G of whatever the camera is pointed at, plus what the tuning file
would need to say for that scene to come out neutral.

Point the camera at a **neutral grey or white surface**, filling the metering
box, lit by the illuminant you care about. Then:

    python3 tools/measure-awb.py                    # centre 50% of the frame
    python3 tools/measure-awb.py --box 0.3 0.3 0.4 0.4
    python3 tools/measure-awb.py --ct 5000          # label it as 5000 K

Repeat under two or three different illuminants (daylight, indoor lamp) and
feed the results to tools/fit-awb-curve.py.
"""

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
TUNING = ROOT / "libcamera" / "ar1335.json"

# Bayer site positions for GRBG: G R / B G
SITES = {"G1": (0, 0), "R": (0, 1), "B": (1, 0), "G2": (1, 1)}


def capture_raw(width: int, height: int, timeout_ms: int,
                extra: list[str] | None = None) -> np.ndarray:
    """Capture one raw frame and return the unpacked Bayer plane."""
    with tempfile.TemporaryDirectory() as tmp:
        jpg = Path(tmp) / "c.jpg"
        cmd = ["rpicam-still", "-n", "--width", str(width), "--height",
               str(height), "-t", str(timeout_ms), "-r", "-o", str(jpg)]
        if extra:
            cmd += extra
        subprocess.run(cmd, check=True, capture_output=True, timeout=120)

        dng = jpg.with_suffix(".dng")
        if not dng.is_file():
            raise RuntimeError("no DNG produced")
        # LibRaw unpacks to a plain 16-bit TIFF of the Bayer plane.
        subprocess.run(["unprocessed_raw", "-T", str(dng)],
                       check=True, capture_output=True, cwd=tmp, timeout=120)
        tif = Path(tmp) / (dng.name + ".tiff")
        return np.asarray(Image.open(tif)).astype(np.float32)


def black_level() -> float:
    try:
        d = json.loads(TUNING.read_text())
        for algo in d["algorithms"]:
            if "rpi.black_level" in algo:
                return float(algo["rpi.black_level"]["black_level"])
    except Exception:
        pass
    return 2688.0


def ct_curve() -> list[tuple[float, float, float]]:
    try:
        d = json.loads(TUNING.read_text())
        for algo in d["algorithms"]:
            if "rpi.awb" in algo:
                c = algo["rpi.awb"]["ct_curve"]
                return [(c[i], c[i + 1], c[i + 2]) for i in range(0, len(c), 3)]
    except Exception:
        return []
    return []


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("-t", "--timeout", type=int, default=4000)
    ap.add_argument("--box", nargs=4, type=float, metavar=("X", "Y", "W", "H"),
                    default=[0.25, 0.25, 0.5, 0.5],
                    help="metering box as fractions of the frame")
    ap.add_argument("--ct", type=float, default=None,
                    help="label this measurement with a known colour temperature")
    ap.add_argument("--json", action="store_true", help="emit JSON only")
    args = ap.parse_args()

    raw = capture_raw(args.width, args.height, args.timeout)
    bl = black_level()

    H, W = raw.shape
    x, y, w, h = args.box
    x0, y0 = int(W * x) & ~1, int(H * y) & ~1     # keep the Bayer phase
    x1, y1 = int(W * (x + w)) & ~1, int(H * (y + h)) & ~1
    box = raw[y0:y1, x0:x1]

    means = {}
    for name, (r, c) in SITES.items():
        plane = box[r::2, c::2]
        # Ignore saturated pixels: they bias the ratios toward grey.
        valid = plane[plane < 60000]
        means[name] = float(max(valid.mean() - bl, 1e-6)) if valid.size else 0.0

    g = (means["G1"] + means["G2"]) / 2.0
    r_over_g = means["R"] / g
    b_over_g = means["B"] / g

    sat = float((box >= 60000).mean() * 100)
    result = {
        "box": [x0, y0, x1 - x0, y1 - y0],
        "black_level": bl,
        "raw_means_minus_black": {k: round(v, 1) for k, v in means.items()},
        "green_balance_pct": round(100 * abs(means["G1"] - means["G2"]) / g, 2),
        "r_over_g": round(r_over_g, 4),
        "b_over_g": round(b_over_g, 4),
        "awb_gains_to_neutralise": [round(1 / r_over_g, 4), round(1 / b_over_g, 4)],
        "saturated_pct": round(sat, 2),
    }
    if args.ct:
        result["labelled_ct"] = args.ct

    if args.json:
        print(json.dumps(result, indent=2))
        return 0

    print("AR1335 raw channel measurement")
    print(f"  metering box     : {x1-x0}x{y1-y0} px at ({x0},{y0})")
    print(f"  black level      : {bl:.0f}")
    print(f"  saturated pixels : {sat:.2f}%"
          + ("   <-- TOO BRIGHT, reduce exposure" if sat > 1 else ""))
    print()
    for k in ("R", "G1", "G2", "B"):
        print(f"    {k:2} = {means[k]:9.1f}")
    print(f"  green balance    : {result['green_balance_pct']:.2f}% "
          "(should be well under 1%)")
    print()
    print(f"  R/G = {r_over_g:.4f}     B/G = {b_over_g:.4f}")
    print(f"  gains to neutralise this scene: "
          f"R={1/r_over_g:.3f}  B={1/b_over_g:.3f}")

    curve = ct_curve()
    if curve:
        print("\n  tuning ct_curve currently spans:")
        print(f"    R/G {min(c[1] for c in curve):.3f} .. {max(c[1] for c in curve):.3f}")
        print(f"    B/G {min(c[2] for c in curve):.3f} .. {max(c[2] for c in curve):.3f}")
        inside = (min(c[1] for c in curve) <= r_over_g <= max(c[1] for c in curve)
                  and min(c[2] for c in curve) <= b_over_g <= max(c[2] for c in curve))
        print(f"    this measurement is {'INSIDE' if inside else 'OUTSIDE'} the curve"
              + ("" if inside else "  <-- AWB is extrapolating; curve needs refitting"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
