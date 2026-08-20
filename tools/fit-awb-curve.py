#!/usr/bin/env python3
"""Fit the AR1335's AWB colour-temperature curve into the tuning file.

libcamera's Bayesian AWB searches along a `ct_curve`: (CT, R/G, B/G) triples
giving the raw sensor ratios of a neutral white under each illuminant. Those
ratios depend on the sensor's spectral response and its IR filter, so the curve
inherited from the IMX519 does not fit the AR1335 - measurement showed the real
operating point sits well outside it, leaving AWB to extrapolate and report
implausible colour temperatures.

Two ways to use this:

  Anchor (one measurement, minimum viable):
      python3 tools/fit-awb-curve.py --anchor 2700 1.1617 0.3270

    Keeps the borrowed curve's *shape* - how the ratios change with colour
    temperature, which is driven by the illuminants themselves and transfers
    between sensors - and rescales it so it passes through your measured point.
    Much better than extrapolating, but only exact at the anchor.

  Fit (two or more measurements, proper):
      python3 tools/fit-awb-curve.py --points awb-points.json

    where the file is a list of {"ct":…, "r_over_g":…, "b_over_g":…}. Fits
    log-linear models r(CT) and b(CT) through the measurements and rebuilds the
    curve from them. Three points spanning warm to cool is plenty.

Always writes libcamera/ar1335.json and, with --install, the copy libcamera
actually loads.
"""

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
TUNING = ROOT / "libcamera" / "ar1335.json"
INSTALLED = Path("/usr/local/share/libcamera/ipa/rpi/pisp/ar1335.json")

# Colour temperatures to emit. Spanning candlelight to shade keeps AWB inside
# the curve for any plausible illuminant.
OUT_CTS = [2200, 2700, 3200, 4000, 5000, 6500, 8000]


def load_curve(doc) -> list[tuple[float, float, float]]:
    for algo in doc["algorithms"]:
        if "rpi.awb" in algo:
            c = algo["rpi.awb"]["ct_curve"]
            return [(c[i], c[i + 1], c[i + 2]) for i in range(0, len(c), 3)]
    raise SystemExit("no rpi.awb in tuning file")


def set_curve(doc, pts) -> None:
    flat = []
    for ct, r, b in pts:
        flat += [round(float(ct), 1), round(float(r), 5), round(float(b), 5)]
    for algo in doc["algorithms"]:
        if "rpi.awb" in algo:
            algo["rpi.awb"]["ct_curve"] = flat
            return
    raise SystemExit("no rpi.awb in tuning file")


def interp(curve, ct, idx):
    xs = [c[0] for c in curve]
    ys = [c[idx] for c in curve]
    return float(np.interp(ct, xs, ys))


def anchor(curve, ct, r_meas, b_meas):
    """Rescale the borrowed curve so it passes through one measured point."""
    sr = r_meas / interp(curve, ct, 1)
    sb = b_meas / interp(curve, ct, 2)
    print(f"  anchor at {ct:.0f} K: measured r={r_meas:.4f} b={b_meas:.4f}")
    print(f"  borrowed curve there : r={interp(curve,ct,1):.4f} b={interp(curve,ct,2):.4f}")
    print(f"  scale factors        : r x{sr:.4f}   b x{sb:.4f}")
    return [(c[0], c[1] * sr, c[2] * sb) for c in curve], (sr, sb)


def fit(points):
    """Fit log-linear r(CT), b(CT) through >=2 measurements."""
    cts = np.array([p["ct"] for p in points], dtype=float)
    rs = np.array([p["r_over_g"] for p in points], dtype=float)
    bs = np.array([p["b_over_g"] for p in points], dtype=float)

    # Both ratios vary smoothly and monotonically with log(CT).
    x = np.log(cts)
    pr = np.polyfit(x, np.log(rs), 1)
    pb = np.polyfit(x, np.log(bs), 1)
    print(f"  fitted {len(points)} points")
    for p in points:
        print(f"    {p['ct']:6.0f} K  r={p['r_over_g']:.4f}  b={p['b_over_g']:.4f}")

    out = []
    for ct in OUT_CTS:
        lx = np.log(ct)
        out.append((ct, float(np.exp(np.polyval(pr, lx))),
                    float(np.exp(np.polyval(pb, lx)))))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--anchor", nargs=3, type=float,
                   metavar=("CT", "R_OVER_G", "B_OVER_G"))
    g.add_argument("--points", type=Path)
    ap.add_argument("--install", action="store_true",
                    help="also write the installed tuning file (needs root)")
    args = ap.parse_args()

    doc = json.loads(TUNING.read_text())
    curve = load_curve(doc)

    print("current ct_curve:")
    for ct, r, b in curve:
        print(f"    {ct:6.0f} K  r={r:.4f}  b={b:.4f}")
    print()

    if args.anchor:
        ct, r, b = args.anchor
        new, _ = anchor(curve, ct, r, b)
    else:
        pts = json.loads(args.points.read_text())
        if len(pts) < 2:
            raise SystemExit("need at least two points to fit; use --anchor for one")
        new = fit(pts)

    print("\nnew ct_curve:")
    for ct, r, b in new:
        print(f"    {ct:6.0f} K  r={r:.4f}  b={b:.4f}")

    backup = TUNING.with_suffix(".json.bak")
    shutil.copy(TUNING, backup)
    set_curve(doc, new)
    TUNING.write_text(json.dumps(doc, indent=4) + "\n")
    print(f"\nwrote {TUNING}   (backup {backup.name})")

    if args.install:
        tmp = Path("/tmp/ar1335-tuning.json")
        tmp.write_text(json.dumps(doc, indent=4) + "\n")
        subprocess.run(["sudo", "cp", str(tmp), str(INSTALLED)], check=True)
        print(f"installed to {INSTALLED}")
    else:
        print(f"re-run with --install to update {INSTALLED}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
