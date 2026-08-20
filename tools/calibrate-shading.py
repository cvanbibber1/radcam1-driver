#!/usr/bin/env python3
"""Measure lens shading (vignetting and colour shading) and store it.

Point the camera at something *uniform* and fill the frame with it: a sheet of
white paper a few centimetres from the lens, an evenly lit wall, or a lightbox.
Defocusing helps - texture in the target is indistinguishable from shading, and
this tool will refuse a capture that has any.

    python3 tools/calibrate-shading.py                 # measure and report
    python3 tools/calibrate-shading.py --store         # write to the EEPROM
    python3 tools/calibrate-shading.py --store --apply # ...and to the tuning
    python3 tools/calibrate-shading.py --from-capture flat.jpg

What matters is *uniformity*, not colour or brightness: the model is normalised
at the optical centre, so the absolute level cancels. What does not cancel is a
gradient in the illumination itself, which the fit cannot tell from vignetting.
Lighting the target from both sides, or defocusing onto a lightbox, is the way
to avoid baking the room into the camera.

The fit is a radial polynomial per channel about a shared, fitted optical
centre - see radcam/shading.py for why a model rather than a grid.
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

from radcam import shading                                    # noqa: E402
from radcam.eeprom import CameraEEPROM                        # noqa: E402

TUNING = ROOT / "libcamera" / "ar1335.json"

#: Coarse grid the planes are reduced to before fitting. Fine enough to see the
#: shape of the vignette, coarse enough that per-pixel noise averages away.
GRID_W, GRID_H = 48, 36


def black_level() -> float:
    try:
        d = json.loads(TUNING.read_text())
        for a in d["algorithms"]:
            if "rpi.black_level" in a:
                return float(a["rpi.black_level"]["black_level"])
    except Exception:
        pass
    return 2688.0


def capture_raw(width, height, timeout_ms, keep: Path | None = None):
    with tempfile.TemporaryDirectory() as tmp:
        jpg = Path(tmp) / "flat.jpg"
        cmd = ["rpicam-still", "-n", "--width", str(width), "--height",
               str(height), "-t", str(timeout_ms), "-r",
               # Unity AWB and a fixed matrix: the raw is unaffected, but the
               # preview is then a fair picture of what was actually measured.
               "--awbgains", "1,1", "-o", str(jpg)]
        subprocess.run(cmd, check=True, capture_output=True, timeout=180)
        dng = jpg.with_suffix(".dng")
        subprocess.run(["unprocessed_raw", "-T", dng.name], check=True,
                       capture_output=True, cwd=tmp, timeout=180)
        raw = np.asarray(Image.open(Path(tmp) / (dng.name + ".tiff"))
                         ).astype(np.float64)
        if keep:
            keep.parent.mkdir(exist_ok=True)
            Image.open(jpg).save(keep, quality=88)
        return raw


def planes_from_raw(raw: np.ndarray, bl: float):
    """GRBG Bayer -> three coarse, black-subtracted planes."""
    sites = {"g": (raw[0::2, 0::2] + raw[1::2, 1::2]) / 2.0,
             "r": raw[0::2, 1::2],
             "b": raw[1::2, 0::2]}
    out = {}
    for name, plane in sites.items():
        p = plane - bl
        h, w = p.shape
        bh, bw = h // GRID_H, w // GRID_W
        # Block means, which is both the downsample and the noise filter.
        out[name] = p[:bh * GRID_H, :bw * GRID_W].reshape(
            GRID_H, bh, GRID_W, bw).mean(axis=(1, 3))
    return out


def check_flatness(planes, raw, bl) -> list[str]:
    """Reject a capture that cannot support a shading measurement."""
    problems = []
    g = planes["g"]

    if (raw >= 60000).mean() > 0.002:
        problems.append(f"{100 * (raw >= 60000).mean():.1f}% of pixels are "
                        "clipped - reduce exposure")
    if g.max() < 0.15 * (65535 - bl):
        problems.append("target is very dark; raise exposure so the centre "
                        "sits around half scale")

    # Texture test. Shading is smooth by nature, so compare the field with a
    # blurred copy of itself: a uniform target leaves almost nothing behind,
    # while a scene with edges in it leaves a lot. This is what stops someone
    # calibrating shading from a picture of the room.
    k = 5
    pad = np.pad(g, k, mode="edge")
    smooth = np.stack([pad[i:i + g.shape[0], j:j + g.shape[1]]
                       for i in range(2 * k + 1)
                       for j in range(2 * k + 1)]).mean(axis=0)
    detail = float(np.abs(g - smooth).mean() / max(g.mean(), 1e-9))
    if detail > 0.02:
        problems.append(f"target is not uniform ({100 * detail:.1f}% local "
                        "variation) - this looks like a scene, not a flat "
                        "field. Fill the frame with blank paper or a wall.")

    # A strong linear gradient is usually the room lighting rather than the
    # lens, and it is the one error that a radial fit will happily absorb.
    left, right = g[:, :GRID_W // 4].mean(), g[:, -GRID_W // 4:].mean()
    top, bottom = g[:GRID_H // 4].mean(), g[-GRID_H // 4:].mean()
    for tag, a, b in (("left/right", left, right), ("top/bottom", top, bottom)):
        rel = abs(a - b) / max(a + b, 1e-9) * 2
        if rel > 0.25:
            problems.append(f"{tag} brightness differs by {100 * rel:.0f}% - "
                            "the target is lit unevenly, and that would be "
                            "stored as if it were lens shading")
    return problems


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=4096)
    ap.add_argument("--height", type=int, default=3072)
    ap.add_argument("-t", "--timeout", type=int, default=4000)
    ap.add_argument("--bus", type=int, default=4)
    ap.add_argument("--from-capture", type=Path, default=None, metavar="JPG",
                    help="analyse a saved frame (needs its .dng alongside)")
    ap.add_argument("--store", action="store_true",
                    help="write the model to the camera EEPROM")
    ap.add_argument("--apply", action="store_true",
                    help="also expand it into the installed tuning file")
    ap.add_argument("--force", action="store_true",
                    help="measure even if the flat field looks unusable")
    ap.add_argument("--show", action="store_true",
                    help="print the stored shading model and exit")
    args = ap.parse_args()

    ee = CameraEEPROM(bus=args.bus)
    if args.show:
        rec = ee.load() or {}
        print(json.dumps(rec.get("shading", "no shading stored"), indent=2))
        return 0

    bl = black_level()
    if args.from_capture:
        jpg = args.from_capture
        tif = jpg.with_suffix(".dng.tiff")
        if not tif.is_file():
            dng = jpg.with_suffix(".dng")
            if not dng.is_file():
                print(f"need {dng} alongside {jpg}")
                return 1
            subprocess.run(["unprocessed_raw", "-T", dng.name], check=True,
                           capture_output=True, cwd=dng.parent, timeout=180)
        raw = np.asarray(Image.open(tif)).astype(np.float64)
        print(f"analysing {jpg}")
    else:
        print(f"capturing {args.width}x{args.height} flat field...")
        raw = capture_raw(args.width, args.height, args.timeout,
                          ROOT / "captures" / "flatfield.jpg")

    planes = planes_from_raw(raw, bl)
    problems = check_flatness(planes, raw, bl)
    if problems:
        print("\nFLAT FIELD REJECTED:")
        for p in problems:
            print(f"  - {p}")
        if not args.force:
            print("\nPoint the camera at blank white paper or an evenly lit")
            print("wall, filling the frame, and try again. --force overrides.")
            return 1
        print("  (--force given, continuing anyway)")

    centre, coeff, resid = shading.fit(planes)
    print(f"\noptical centre: ({centre[0]:.3f}, {centre[1]:.3f}) "
          f"of the frame (0.5, 0.5 would be perfectly aligned)")
    print("\n  channel   corner/centre   fit residual")
    fall = {}
    for name in ("r", "g", "b"):
        fall[name] = shading.corner_falloff(coeff[name], centre)
        print(f"     {name}        {100 * fall[name]:5.1f}%        "
              f"{100 * resid[name]:.2f}%")

    worst = min(fall.values())
    print(f"\n  the dimmest corner keeps {100 * worst:.0f}% of the centre's "
          f"light")
    # Colour shading is the part that a plain vignette correction misses, and
    # the part that makes a corner look a different colour from the middle.
    ratio_r = fall["r"] / fall["g"]
    ratio_b = fall["b"] / fall["g"]
    print(f"  colour shading at the corner: R/G {ratio_r:.3f}, "
          f"B/G {ratio_b:.3f} (1.000 = no colour shift)")

    model = {
        "centre": [round(centre[0], 5), round(centre[1], 5)],
        "coeff": {k: [round(v, 6) for v in coeff[k]] for k in coeff},
        "corner_response": {k: round(fall[k], 4) for k in fall},
        "fit_residual": {k: round(resid[k], 5) for k in resid},
        "measured_utc": datetime.now(timezone.utc).isoformat(),
    }
    print(f"\nmodel is {len(json.dumps(model))} bytes "
          f"(a 32x32x3 grid would be about 18000)")

    if not args.store:
        print("\n--store to write it to the camera EEPROM")
        print(json.dumps(model, indent=2))
        return 0

    ee.update("shading", model)
    print(f"\nstored on the camera EEPROM (i2c-{args.bus}, 0x50)")

    if args.apply:
        r = subprocess.run(
            ["sudo", sys.executable, "-c",
             "import sys; sys.path.insert(0, %r);"
             " from radcam import calibration;"
             " calibration.load_and_apply(bus=%d)" % (str(ROOT), args.bus)],
            capture_output=True, text=True)
        if r.returncode != 0:
            print("  apply FAILED:", (r.stderr or r.stdout).strip()[-300:])
            return 1
        print("  expanded into the installed tuning; next capture uses it")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
