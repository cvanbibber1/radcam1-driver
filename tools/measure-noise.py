#!/usr/bin/env python3
"""Measure the AR1335's noise profile for the denoise algorithm.

libcamera's `rpi.noise` models sensor noise as

    sigma(signal) = reference_constant + reference_slope * sqrt(signal)

the read-noise floor plus the photon shot noise that grows with the square
root of signal. Denoising uses it to tell noise from detail; borrowed values
mean it either smears real texture or leaves grain behind.

Measured by the photon-transfer method: capture two frames of a *static* scene
and subtract them. The scene cancels exactly, leaving only noise, so this works
on whatever the camera is pointed at - no calibration target, no flat field.
Binning the difference by signal level gives sigma against signal directly.

    python3 tools/measure-noise.py
    python3 tools/measure-noise.py --install

Requirements: the scene must not move and the light must not flicker between
the two frames. Anything that changes shows up as noise and inflates the fit.
"""

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
TUNING = ROOT / "libcamera" / "ar1335.json"
INSTALLED = Path("/usr/local/share/libcamera/ipa/rpi/pisp/ar1335.json")

SITES = {"G1": (0, 0), "R": (0, 1), "B": (1, 0), "G2": (1, 1)}   # GRBG


def black_level() -> float:
    try:
        d = json.loads(TUNING.read_text())
        for a in d["algorithms"]:
            if "rpi.black_level" in a:
                return float(a["rpi.black_level"]["black_level"])
    except Exception:
        pass
    return 2688.0


def capture_raw(width, height, timeout_ms, gain=None, shutter=None):
    with tempfile.TemporaryDirectory() as tmp:
        jpg = Path(tmp) / "c.jpg"
        cmd = ["rpicam-still", "-n", "--width", str(width), "--height",
               str(height), "-t", str(timeout_ms), "-r",
               "--awbgains", "1,1", "-o", str(jpg)]
        # Pin exposure so the two frames are photometrically identical; AE
        # drifting between them would masquerade as noise.
        if gain is not None:
            cmd += ["--gain", str(gain)]
        if shutter is not None:
            cmd += ["--shutter", str(shutter)]
        subprocess.run(cmd, check=True, capture_output=True, timeout=180)
        dng = jpg.with_suffix(".dng")
        subprocess.run(["unprocessed_raw", "-T", str(dng)],
                       check=True, capture_output=True, cwd=tmp, timeout=180)
        return np.asarray(Image.open(Path(tmp) / (dng.name + ".tiff"))
                          ).astype(np.float32)


def profile(a: np.ndarray, b: np.ndarray, bl: float, bins: int = 24):
    """sigma vs signal, from the difference of two frames of a static scene."""
    # Work on one green plane: same statistics, quarter the data, and no
    # cross-channel mixing.
    ga = a[0::2, 0::2]
    gb = b[0::2, 0::2]

    signal = (ga + gb) / 2.0 - bl
    # Var(a-b) = 2*Var(noise) for independent frames, so divide by sqrt(2).
    diff = (ga - gb) / np.sqrt(2.0)

    lo, hi = 0.0, float(np.percentile(signal, 99.5))
    edges = np.linspace(lo, hi, bins + 1)
    xs, ys, ns = [], [], []
    for i in range(bins):
        m = (signal >= edges[i]) & (signal < edges[i + 1])
        if m.sum() < 500:
            continue
        xs.append(float(signal[m].mean()))
        # Sigma-clipped standard deviation. A percentile of |diff| looks
        # robust but picks an actual sample value, so with only a few LSB of
        # noise it quantises to multiples of 1 LSB (64 in these 16-bit units)
        # and the fit is driven by the quantisation rather than the sensor.
        # A standard deviation averages over all the pixels in the bin and is
        # continuous; clipping keeps residual motion from dominating it.
        d = diff[m]
        for _ in range(3):
            sd = float(d.std())
            if sd <= 0:
                break
            keep = np.abs(d - d.mean()) < 3.0 * sd
            if keep.sum() < 100:
                break
            d = d[keep]
        ys.append(float(d.std()))
        ns.append(int(m.sum()))
    return np.array(xs), np.array(ys), np.array(ns)


def fit(xs, ys):
    """Least squares for sigma = c + s*sqrt(signal)."""
    A = np.stack([np.ones_like(xs), np.sqrt(np.maximum(xs, 0))], axis=1)
    (c, s), *_ = np.linalg.lstsq(A, ys, rcond=None)
    pred = A @ np.array([c, s])
    resid = float(np.sqrt(np.mean((ys - pred) ** 2)))
    return float(c), float(s), resid


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("-t", "--timeout", type=int, default=3000)
    ap.add_argument("--gain", type=float, default=1.0,
                    help="analogue gain to pin for the measurement")
    ap.add_argument("--shutter", type=int, default=None,
                    help="shutter in us; omit to let AE pick once")
    ap.add_argument("--install", action="store_true")
    args = ap.parse_args()

    bl = black_level()
    print(f"capturing two frames at gain {args.gain} (scene must be static)...")
    a = capture_raw(args.width, args.height, args.timeout, args.gain,
                    args.shutter)
    b = capture_raw(args.width, args.height, args.timeout, args.gain,
                    args.shutter)

    ga, gb = a[0::2, 0::2], b[0::2, 0::2]
    drift = abs(float(ga.mean() - gb.mean()))
    print(f"  frame means: {ga.mean():.0f} and {gb.mean():.0f} "
          f"(drift {drift:.0f})")
    if drift > 0.02 * max(ga.mean() - bl, 1):
        print("\n  FRAMES DIFFER TOO MUCH - the scene or the light changed.")
        print("  The difference method needs two identical frames; anything")
        print("  that moved is being counted as noise. Steady the scene and")
        print("  pin --shutter, then try again.")
        return 1

    xs, ys, ns = profile(a, b, bl)
    if len(xs) < 5:
        print("\n  not enough signal range in this scene to fit a curve")
        return 1

    c, s, resid = fit(xs, ys)
    print(f"\n  {'signal':>10} {'sigma':>8} {'pixels':>9}")
    for x, y, n in zip(xs, ys, ns):
        print(f"  {x:10.0f} {y:8.1f} {n:9d}")

    print(f"\n  fit: sigma = {c:.2f} + {s:.4f} * sqrt(signal)")
    print(f"  rms residual: {resid:.2f}")

    # libcamera's reference values are in 16-bit units at unity gain, which is
    # what we captured at.
    print(f"\n  reference_constant = {c:.1f}")
    print(f"  reference_slope    = {s:.4f}")

    if args.install:
        doc = json.loads(TUNING.read_text())
        for algo in doc["algorithms"]:
            if "rpi.noise" in algo:
                old = dict(algo["rpi.noise"])
                algo["rpi.noise"]["reference_constant"] = round(c, 1)
                algo["rpi.noise"]["reference_slope"] = round(s, 4)
                print(f"\n  was: {old}")
                print(f"  now: {algo['rpi.noise']}")
        shutil.copy(TUNING, TUNING.with_suffix(".json.bak"))
        TUNING.write_text(json.dumps(doc, indent=4) + "\n")
        tmp = Path("/tmp/ar1335-noise.json")
        tmp.write_text(json.dumps(doc, indent=4) + "\n")
        subprocess.run(["sudo", "cp", str(tmp), str(INSTALLED)], check=True)
        # Keep the source tree in step, or the next ninja install reverts this.
        src = ROOT / "libcamera-src/src/ipa/rpi/pisp/data/ar1335.json"
        if src.is_file():
            shutil.copy(TUNING, src)
        print(f"\n  installed to {INSTALLED} (and synced to the source tree)")
    else:
        print("\n  re-run with --install to write it into the tuning file")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
