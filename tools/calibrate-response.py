#!/usr/bin/env python3
"""Measure the sensor's response: exposure linearity, gain linearity, saturation.

Three numbers the rest of the system quietly assumes and nobody has checked:

  * **Exposure linearity.** AGC works by scaling exposure and expecting signal
    to scale with it. If the sensor is not linear in integration time, every
    automatic exposure decision is slightly wrong, and bracketed captures
    cannot be combined.
  * **Gain linearity.** The CamHelper maps a requested analogue gain onto the
    AR1335's banded GLOBAL_GAIN encoding. That mapping is a guess taken from
    the datasheet's description; this measures what the sensor actually does.
  * **Saturation point.** Where the response stops rising. Not 65535: the raw
    is 10-bit data in 16-bit containers, there is a black pedestal underneath,
    and the real ceiling is what decides when a highlight is unrecoverable.

Point the camera at a **static, evenly lit** scene with a good bright area -
the colour chart is fine - and leave it alone while this runs. Nothing may move
or flicker, because a change in the scene is indistinguishable from a change in
the sensor's response.

    python3 tools/calibrate-response.py
    python3 tools/calibrate-response.py --store

Mains flicker is the trap here. Exposures that are not a whole number of mains
cycles pick up a different slice of the flicker each time, which shows up as
scatter that looks like non-linearity. Exposure points are snapped to multiples
of a half-cycle by default; pass --mains 0 to disable if the light is DC.
"""

import argparse
import json
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from radcam.eeprom import CameraEEPROM                          # noqa: E402

TUNING = ROOT / "libcamera" / "ar1335.json"
SITES = {"g1": (0, 0), "r": (0, 1), "b": (1, 0), "g2": (1, 1)}   # GRBG


def black_level() -> float:
    try:
        d = json.loads(TUNING.read_text())
        for a in d["algorithms"]:
            if "rpi.black_level" in a:
                return float(a["rpi.black_level"]["black_level"])
    except Exception:
        pass
    return 2688.0


def capture(shutter_us: int, gain: float, width=1920, height=1080):
    """One raw frame at a pinned exposure and gain."""
    with tempfile.TemporaryDirectory() as tmp:
        jpg = Path(tmp) / "s.jpg"
        subprocess.run(
            ["rpicam-still", "-n", "--width", str(width), "--height",
             str(height), "-t", "1200", "-r", "--awbgains", "1,1",
             "--shutter", str(int(shutter_us)), "--gain", f"{gain:.3f}",
             "--immediate", "-o", str(jpg)],
            check=True, capture_output=True, timeout=180)
        dng = jpg.with_suffix(".dng")
        subprocess.run(["unprocessed_raw", "-T", dng.name], check=True,
                       capture_output=True, cwd=tmp, timeout=180)
        return np.asarray(Image.open(Path(tmp) / (dng.name + ".tiff"))
                          ).astype(np.float64)


def measure(raw, bl):
    """Mean level per channel over the central half of the frame, plus clipping."""
    h, w = raw.shape
    y0, y1 = h // 4, 3 * h // 4
    x0, x1 = w // 4, 3 * w // 4
    c = raw[y0:y1, x0:x1]
    out = {}
    for name, (r, cc) in SITES.items():
        out[name] = float(np.mean(c[r::2, cc::2])) - bl
    out["g"] = (out.pop("g1") + out.pop("g2")) / 2.0
    # Clipping is judged inside the measured region only. A scene with a light
    # fixture in shot has saturated pixels at every exposure, and testing the
    # whole frame would reject every point before the sweep even starts.
    out["clipped"] = float((c >= 65000).mean())
    out["clipped_frame"] = float((raw >= 65000).mean())
    out["p999"] = float(np.percentile(c, 99.9))
    return out


def linearity(x, y):
    """Fit y = a*x through the unsaturated points; return slope and worst error."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    a = float((x * y).sum() / max((x * x).sum(), 1e-9))
    pred = a * x
    with np.errstate(divide="ignore", invalid="ignore"):
        err = np.where(pred > 0, np.abs(y - pred) / pred, 0.0)
    return a, float(np.max(err) * 100), pred


def offset(x, y):
    """Fit y = a*x + b and return b, the signal left at zero exposure.

    This is worth more than it looks. A sensor is linear in integration time,
    so extrapolating the sweep back to zero exposure should give zero signal.
    Whatever is left over is a pedestal the black level did not remove - and
    because it is measured over a whole sweep rather than from one dark frame,
    it is a far better estimate of the true black level than a single capture.
    """
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    A = np.stack([x, np.ones_like(x)], axis=1)
    (a, b), *_ = np.linalg.lstsq(A, y, rcond=None)
    return float(a), float(b)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gain", type=float, default=1.0,
                    help="gain to hold while sweeping exposure")
    ap.add_argument("--shutter", type=int, default=None,
                    help="exposure to hold while sweeping gain; default is "
                         "picked so the scene sits mid-scale")
    ap.add_argument("--mains", type=float, default=120.0,
                    help="mains flicker frequency in Hz (US 120, EU 100); "
                         "0 disables exposure snapping")
    ap.add_argument("--points", type=int, default=9)
    ap.add_argument("--max-clip", type=float, default=1.0,
                    help="percent of the measured region allowed to clip "
                         "before a point is dropped from the fit")
    ap.add_argument("--max-level", type=float, default=0.80,
                    help="drop points above this fraction of full scale")
    ap.add_argument("--bus", type=int, default=4)
    ap.add_argument("--store", action="store_true")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    ee = CameraEEPROM(bus=args.bus)
    if args.show:
        rec = ee.load() or {}
        print(json.dumps(rec.get("response", "no response data stored"),
                         indent=2))
        return 0

    bl = black_level()
    print(f"black level {bl:.0f}; scene must stay still for about a minute\n")

    # --- find an exposure that puts the scene mid-scale -------------------
    probe = 8000
    for _ in range(9):
        m = measure(capture(probe, args.gain), bl)
        level = m["g"] / (65535 - bl)
        if 0.25 <= level <= 0.55 or probe >= 200000 or probe <= 200:
            break
        probe = int(np.clip(probe * (0.40 / max(level, 1e-3)), 200, 200000))
    print(f"anchor exposure {probe} us puts green at "
          f"{100 * m['g'] / (65535 - bl):.0f}% of scale")

    step = 1e6 / (2 * args.mains) if args.mains > 0 else 0.0
    def snap(us):
        if step <= 0:
            return int(max(us, 100))
        # Whole half-cycles of mains, so every point integrates the same slice
        # of the flicker.
        return int(max(round(us / step), 1) * step)

    # --- exposure sweep ---------------------------------------------------
    lo, hi = probe * 0.12, probe * 2.6
    shutters = sorted({snap(v) for v in np.linspace(lo, hi, args.points)})
    print(f"\nEXPOSURE SWEEP at gain {args.gain:.2f}"
          + (f", snapped to {step:.0f} us ({args.mains:.0f} Hz)"
             if step else ""))
    print(f"  {'shutter us':>11} {'R':>9} {'G':>9} {'B':>9} {'clip %':>7}")
    ex, ey = [], {"r": [], "g": [], "b": []}
    rows = []
    for s in shutters:
        m = measure(capture(s, args.gain), bl)
        rows.append((s, m))
        print(f"  {s:11d} {m['r']:9.0f} {m['g']:9.0f} {m['b']:9.0f} "
              f"{100 * m['clipped']:7.2f}")
        if (m["clipped"] < args.max_clip / 100.0
                and m["g"] < args.max_level * (65535 - bl)):
            ex.append(s)
            for c in ("r", "g", "b"):
                ey[c].append(m[c])

    if len(ex) < 4:
        print("\n  too few unsaturated points; lower the light or the gain")
        return 1
    exp_lin, worst_at = {}, {}
    for c in ("r", "g", "b"):
        _, err, pred = linearity(ex, ey[c])
        exp_lin[c] = round(err, 2)
        rel = np.abs(np.array(ey[c]) - pred) / np.maximum(pred, 1e-9)
        worst_at[c] = int(np.array(ex)[int(np.argmax(rel))])
    print(f"\n  exposure linearity (worst deviation from a straight line "
          f"through the origin):")
    print(f"    R {exp_lin['r']:.2f}%   G {exp_lin['g']:.2f}%   "
          f"B {exp_lin['b']:.2f}%   over {len(ex)} points "
          f"({ex[0]}-{ex[-1]} us)")
    print(f"    worst point: R at {worst_at['r']} us, G at {worst_at['g']} us, "
          f"B at {worst_at['b']} us")
    off = {c: offset(ex, ey[c])[1] for c in ("r", "g", "b")}
    print(f"\n  extrapolated back to zero exposure: R {off['r']:+.0f}  "
          f"G {off['g']:+.0f}  B {off['b']:+.0f} DN")
    mean_off = float(np.mean(list(off.values())))
    print(f"    a true black level would give zero here; this implies the "
          f"black level is")
    print(f"    about {bl + mean_off:.0f} rather than {bl:.0f} "
          f"({mean_off:+.0f} DN)")

    if len(ex) >= 5:
        # The shortest exposure is where a sensor most often misbehaves, so
        # quote the fit without it too rather than letting one point set the
        # headline number either way.
        sub = {c: round(linearity(ex[1:], ey[c][1:])[1], 2)
               for c in ("r", "g", "b")}
        print(f"    excluding the shortest exposure: R {sub['r']:.2f}%  "
              f"G {sub['g']:.2f}%  B {sub['b']:.2f}%")

    # --- saturation -------------------------------------------------------
    sat = max(m["p999"] for _, m in rows)
    knee = None
    for s, m in rows:
        if m["clipped"] > 0.01:
            knee = m["p999"]
            break
    print(f"\n  saturation: highest 99.9th percentile seen {sat:.0f} DN "
          f"of 65535")
    print(f"    usable range above black: {sat - bl:.0f} DN "
          f"({(sat - bl) / 64:.0f} in 10-bit terms of 1023)")
    if knee:
        print(f"    clipping sets in at about {knee:.0f} DN")

    # --- gain sweep -------------------------------------------------------
    anchor = args.shutter or snap(probe * 0.35)
    gains = [1.0, 1.4, 2.0, 2.8, 4.0, 5.6, 8.0]
    print(f"\nGAIN SWEEP at shutter {anchor} us")
    print(f"  {'gain':>6} {'G':>9} {'G/gain':>9} {'clip %':>7}")
    gx, gy = [], []
    base = None
    for g in gains:
        m = measure(capture(anchor, g), bl)
        if base is None:
            base = m["g"]
        norm = m["g"] / g
        print(f"  {g:6.2f} {m['g']:9.0f} {norm:9.0f} "
              f"{100 * m['clipped']:7.2f}")
        if (m["clipped"] < args.max_clip / 100.0
                and m["g"] < args.max_level * (65535 - bl)):
            gx.append(g)
            gy.append(m["g"])
    gain_err = None
    if len(gx) >= 3:
        _, gain_err, _ = linearity(gx, gy)
        print(f"\n  gain linearity: worst deviation {gain_err:.2f}% "
              f"over {len(gx)} points ({gx[0]:.1f}x to {gx[-1]:.1f}x)")
        print("  (this is the CamHelper's gain mapping being checked against "
              "the sensor)")
    else:
        print("\n  too few unsaturated gain points to fit")

    record = {
        "black_level": int(bl),
        "saturation_dn": int(round(sat)),
        "usable_range_dn": int(round(sat - bl)),
        "exposure_linearity_pct": exp_lin,
        "exposure_points": len(ex),
        "exposure_range_us": [int(ex[0]), int(ex[-1])],
        "gain_linearity_pct": (round(gain_err, 2) if gain_err is not None
                               else None),
        "gain_range_tested": [gx[0], gx[-1]] if gx else None,
        "zero_exposure_offset_dn": {c: round(off[c], 1) for c in off},
        "implied_black_level": int(round(bl + mean_off)),
        "anchor_shutter_us": int(anchor),
        "mains_hz": args.mains,
        "measured_utc": datetime.now(timezone.utc).isoformat(),
    }
    if not args.store:
        print("\n--store to write it to the camera EEPROM")
        print(json.dumps(record, indent=2))
        return 0
    ee.update("response", record)
    print(f"\nstored on the camera EEPROM (i2c-{args.bus}, 0x50)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
