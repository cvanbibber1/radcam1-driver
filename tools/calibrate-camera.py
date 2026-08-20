#!/usr/bin/env python3
"""Ground calibration command: photograph a chart, solve, store on the camera.

One command per camera. It measures white balance and a colour matrix from a
DGK DKK chart, writes them to the camera module's own EEPROM, and optionally
updates the local tuning file.

Storing on the module's EEPROM rather than the host is the whole point: cameras
and Pis are interchangeable, so calibration has to travel with the camera. Any
Pi that later sees this module reads its calibration back and applies it.

    # check framing first - this refuses to run on a bad capture
    python3 tools/chart-framing.py

    # calibrate
    sudo python3 tools/calibrate-camera.py --camera-id CAM-004 --ct 5000

    # inspect what a camera is carrying
    sudo python3 tools/calibrate-camera.py --show
    sudo python3 tools/calibrate-camera.py --erase

The chart must fill most of the frame, square to the lens, evenly lit, in
focus, with no glare and no clipped patches. Everything downstream depends on
that capture, so the tool checks it and refuses rather than producing a
confidently wrong matrix.
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

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from radcam.ccm import (CHART_COLS, CHART_ROWS, DKK_PATCHES_LAB,  # noqa: E402
                        NEUTRAL_INDICES, chart_targets_linear_srgb,
                        calibrate_from_patches, score_calibration,
                        white_balance_gains)
from radcam.eeprom import CameraEEPROM  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parent))
from detect_chart import annotate, find_patches  # noqa: E402

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


def capture_raw(width, height, timeout_ms, keep: Path | None = None):
    """Capture one frame; return (Bayer plane, processed image).

    The processed JPEG is only used to *locate* the chart - colour is always
    measured from the raw Bayer data, before any white balance or matrix.
    """
    with tempfile.TemporaryDirectory() as tmp:
        jpg = Path(tmp) / "c.jpg"
        subprocess.run(
            ["rpicam-still", "-n", "--width", str(width), "--height",
             str(height), "-t", str(timeout_ms), "-r",
             # Fix the gains so the measurement reflects the sensor, not AWB.
             "--awbgains", "1,1", "-o", str(jpg)],
            check=True, capture_output=True, timeout=180)
        dng = jpg.with_suffix(".dng")
        subprocess.run(["unprocessed_raw", "-T", str(dng)],
                       check=True, capture_output=True, cwd=tmp, timeout=180)
        raw = np.asarray(Image.open(Path(tmp) / (dng.name + ".tiff"))
                         ).astype(np.float32)
        proc = Image.open(jpg).copy()
        if keep:
            proc.save(keep, quality=88)
        return raw, proc


def sample_patches(raw, centres, bl, half=60):
    """Sample the raw Bayer plane at each detected patch centre.

    A small box at the centre keeps the sample inside the patch even when the
    lens bows the chart, which a full-cell average would not.
    """
    rgb, spread = [], []
    for cx, cy in centres:
        x, y = int(cx) & ~1, int(cy) & ~1          # keep the Bayer phase
        box = raw[max(y - half, 0):y + half, max(x - half, 0):x + half]
        if box.size == 0:
            rgb.append([1e-6, 1e-6, 1e-6])
            spread.append(1e9)
            continue
        vals = {n: float(np.median(box[rr::2, cc::2]))
                for n, (rr, cc) in SITES.items()}
        g = (vals["G1"] + vals["G2"]) / 2.0
        rgb.append([vals["R"] - bl, g - bl, vals["B"] - bl])
        spread.append(float(box.std()))
    return np.array(rgb), np.array(spread)


def remove_flare(rgb):
    """Subtract veiling glare, estimated from the black patch.

    A wide lens looking at a bright scene scatters light across the whole
    frame, lifting the blacks. Measured here the L*=0 patch read 26% of white,
    and leaving that in makes every colour look washed out - the CCM then
    over-saturates to compensate. The black patch is by definition zero, so
    whatever it reads is the glare floor.
    """
    # Subtract most, not all, of the black-patch reading. Glare varies across
    # the frame, so removing 100% of the centre estimate drives patches that
    # sit in dimmer corners straight to zero and destroys them. 85% removes
    # the bulk of the cast while leaving dark patches measurable.
    black = rgb[5].copy() * 0.85
    out = rgb - black
    return np.maximum(out, 1e-6), black


def check_capture(raw, rgb, spread) -> list[str]:
    problems = []
    clip = (raw >= 60000).mean() * 100
    if clip > 3.0:
        problems.append(f"{clip:.1f}% of pixels clipped")
    elif clip > 0.2:
        # A little clipping usually means only the white patch blew out. That
        # costs one patch, not the calibration - warn and drop it rather than
        # refusing outright.
        print(f"  note: {clip:.1f}% clipped; the white patch may be affected")
    # The grey wedge must descend. Allow a small tolerance: the darkest steps
    # differ by little and noise alone can invert two adjacent patches without
    # meaning the grid is wrong.
    wedge = rgb[0:6, 1]
    steps = np.diff(wedge)
    tol = 0.02 * max(wedge[0], 1.0)
    if np.any(steps > tol):
        problems.append("grey wedge is not monotonically darker - "
                        "grid misaligned or chart rotated")
    # Chromatic patches must actually differ from neutral.
    chroma = np.abs(rgb[6:12, 0] / np.maximum(rgb[6:12, 1], 1e-6) - 1).mean()
    if chroma < 0.15:
        problems.append("colour patches look neutral - chart not in frame?")
    return problems


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera-id", default=None, help="serial to record")
    ap.add_argument("--ct", type=float, default=5000.0,
                    help="colour temperature of the illuminant (K)")
    ap.add_argument("--width", type=int, default=4096)
    ap.add_argument("--height", type=int, default=3072)
    ap.add_argument("-t", "--timeout", type=int, default=5000)
    ap.add_argument("--bus", type=int, default=4, help="camera I2C bus")
    ap.add_argument("--show", action="store_true", help="print stored record")
    ap.add_argument("--erase", action="store_true")
    ap.add_argument("--apply", action="store_true",
                    help="also write the result into the installed libcamera "
                         "tuning file, so this Pi uses it immediately. "
                         "Without this the calibration only goes to the "
                         "camera's EEPROM, and takes effect next time "
                         "radcam.calibration.load_and_apply() runs.")
    ap.add_argument("--no-store", action="store_true",
                    help="compute and report but do not write the EEPROM")
    ap.add_argument("--force", action="store_true",
                    help="calibrate even if the capture looks bad")
    ap.add_argument("--centres", type=Path, default=None, metavar="JSON",
                    help="use patch centres from a JSON list of [x,y] pairs "
                         "instead of detecting them. The escape hatch for a "
                         "chart the detector cannot find - read the centres "
                         "off the image once and calibrate from them.")
    ap.add_argument("--from-capture", type=Path, default=None, metavar="JPG",
                    help="analyse a saved frame (with its .dng alongside) "
                         "instead of capturing. Useful when the rig is not "
                         "static: capture once when it looks right, then "
                         "calibrate from that frame as often as needed.")
    args = ap.parse_args()

    ee = CameraEEPROM(bus=args.bus)

    if args.show:
        rec = ee.load()
        print(json.dumps(rec, indent=2) if rec else "no calibration stored")
        return 0
    if args.erase:
        ee.erase()
        print("calibration erased")
        return 0

    if args.from_capture:
        jpg = args.from_capture
        dng = jpg.with_suffix(".dng")
        tif = jpg.with_suffix(".dng.tiff")
        if not tif.is_file():
            if not dng.is_file():
                print(f"need {dng} alongside {jpg}")
                return 1
            subprocess.run(["unprocessed_raw", "-T", dng.name], check=True,
                           capture_output=True, cwd=dng.parent, timeout=180)
        raw = np.asarray(Image.open(tif)).astype(np.float32)
        proc = Image.open(jpg).copy()
        print(f"analysing saved capture {jpg} ({proc.size})")
    else:
        preview = ROOT / "captures" / "calib-frame.jpg"
        preview.parent.mkdir(exist_ok=True)
        print(f"capturing {args.width}x{args.height} with AWB fixed at unity...")
        raw, proc = capture_raw(args.width, args.height, args.timeout, preview)
    bl = black_level()

    if args.centres:
        centres = [tuple(map(float, p)) for p in
                   json.loads(args.centres.read_text())]
        if len(centres) != CHART_COLS * CHART_ROWS:
            print(f"{args.centres} has {len(centres)} centres, "
                  f"need {CHART_COLS * CHART_ROWS}")
            return 1
        print(f"using {len(centres)} patch centres from {args.centres}")
    else:
        print("locating the chart...")
        centres = find_patches(proc, debug=True)
    if centres is None:
        print("\nCould not find the chart. Run tools/chart-framing.py and")
        print("tools/focus.py --grid to check it is visible and in focus.")
        return 1
    annotate(proc, centres).save(ROOT / "captures" / "calib-detected.jpg",
                                 quality=88)
    print(f"  annotated preview: captures/calib-detected.jpg")

    rgb_raw, spread = sample_patches(raw, centres, bl)
    rgb, flare = remove_flare(rgb_raw)
    print(f"  veiling glare removed: {flare.round(0)} "
          f"({100*flare[1]/max(rgb_raw[0][1],1):.0f}% of white)")

    print(f"\nblack level {bl:.0f}; sampled {len(rgb)} patches")
    print("  patch   R        G        B      (black-subtracted raw)")
    for i, v in enumerate(rgb):
        tag = " neutral" if i in NEUTRAL_INDICES else ""
        print(f"   {i:2d}  {v[0]:8.1f} {v[1]:8.1f} {v[2]:8.1f}{tag}")

    problems = check_capture(raw, rgb, spread)
    if problems:
        print("\nCAPTURE REJECTED:")
        for p in problems:
            print(f"  - {p}")
        print("\nRun tools/chart-framing.py and fix the framing first.")
        if not args.force:
            return 1
        print("  (--force given, continuing anyway)")

    # Reject patches the capture cannot support: anything at the noise floor
    # after glare removal carries no colour information, and forcing the CCM
    # to fit them would corrupt the patches that *are* good.
    usable = np.array([bool(v[1] > 0.01 * rgb[0][1] and min(v) > 1e-3)
                       for v in rgb])
    print(f"\n  usable patches: {int(usable.sum())}/18"
          + ("" if usable.all() else
             f"  (dropped {list(np.where(~usable)[0])})"))
    # White balance needs only a grey patch or two and is robust to a rough
    # grid; the matrix needs every patch measured accurately. When the capture
    # supports one but not the other, store the half that is trustworthy
    # rather than nothing - a corrected cast is most of the visible win.
    wb_only = usable.sum() < 10
    if wb_only:
        print("  too few usable patches for a trustworthy colour matrix;")
        print("  computing WHITE BALANCE ONLY from the grey wedge")

    targets = chart_targets_linear_srgb()
    # Weight the neutrals up: a visible cast is worse than a small hue error.
    weights = usable.astype(float)
    for i in NEUTRAL_INDICES:
        if usable[i]:
            weights[i] = 3.0
    # The black patch sets the glare estimate, so it cannot also score it.
    weights[5] = 0.0

    # Solve glare, white balance and matrix together from the patches *before*
    # flare removal - the solver fits the pedestal rather than assuming the
    # black patch measures it exactly, which it does not once lens shading is
    # in play.
    sol = calibrate_from_patches(rgb_raw, targets, weights=weights)
    (rg, bg) = sol["gains"]
    r_over_g, b_over_g = sol["ratios"]
    wb = sol["wb_rgb"]

    print(f"\nveiling glare fitted: {sol['offset'].round(0)} "
          f"({100 * sol['offset'][1] / max(rgb_raw[0][1], 1):.0f}% of white; "
          f"black patch reads {rgb_raw[5].round(0)})")
    print("\nwhite balance from neutral patches:")
    print(f"  raw R/G = {r_over_g:.4f}   B/G = {b_over_g:.4f}")
    print(f"  gains   R = {rg:.4f}   B = {bg:.4f}   (at {args.ct:.0f} K)")

    scored = weights > 0
    mean_after = max_after = None
    if wb_only:
        M = None
    else:
        M = sol["ccm"]
        de_before, de_after = sol["delta_e_identity"], sol["delta_e"]
        mean_before = float(de_before[scored].mean())
        mean_after = float(de_after[scored].mean())
        max_after = float(de_after[scored].max())

        print("\ncolour correction matrix:")
        for r in M:
            print("   [" + "  ".join(f"{v:8.4f}" for v in r) + "]")
        print(f"\n  mean dE76  {mean_before:6.2f} -> {mean_after:6.2f}")
        print(f"  max  dE76  {float(de_before[scored].max()):6.2f} -> "
              f"{max_after:6.2f}")
        worst = [i for i in np.argsort(-de_after) if scored[i]][:3]
        print("  worst patches: " + ", ".join(
            f"{i} (dE {de_after[i]:.1f})" for i in worst))

        # Judge the matrix on whether it *helps*, not against a fixed number.
        # A flat threshold rejects a matrix that turns dE 21 into dE 7 - a
        # large, visible win - while a matrix fitted to unmeasurable patches
        # barely moves the error at all, which is the thing actually worth
        # catching.
        if mean_after < 4.0:
            print("  quality: good (mean dE < 4)")
        elif mean_after < 8.0 and mean_after < 0.65 * mean_before:
            print(f"  quality: acceptable - dE {mean_before:.1f} -> "
                  f"{mean_after:.1f}, a {100*(1-mean_after/mean_before):.0f}% "
                  "reduction")
        else:
            # Barely better than no matrix at all means it was fitted to
            # patches the capture could not measure, so it will distort the
            # colours that *were* measured correctly. Keep the white balance,
            # which needs only the grey wedge, and discard the matrix.
            print(f"  quality: UNUSABLE (mean dE {mean_after:.1f} vs "
                  f"{mean_before:.1f} uncorrected) - discarding the matrix")
            print("  the saturated patches carried too little signal to "
                  "constrain it")
            M = None
            wb_only = True

    record = {
        "schema": 1,
        "camera_id": args.camera_id or "unknown",
        "sensor": "ar1335",
        "calibrated_utc": datetime.now(timezone.utc).isoformat(),
        "black_level": int(bl),
        "illuminant_ct": float(args.ct),
        "awb": {"r_over_g": round(r_over_g, 5), "b_over_g": round(b_over_g, 5)},
    }
    if M is not None:
        record["ccm"] = [[round(float(v), 5) for v in row] for row in M]
        record["quality"] = {"mean_de76": round(mean_after, 3),
                             "max_de76": round(max_after, 3)}
        record["flare_offset"] = [round(float(v), 1) for v in sol["offset"]]
    else:
        record["quality"] = {"note": "white balance only; "
                                     "chart capture could not support a matrix"}

    if args.no_store:
        print("\n--no-store: not writing the EEPROM")
        print(json.dumps(record, indent=2))
    else:
        print(f"\nwriting calibration to the camera EEPROM "
              f"(i2c-{args.bus}, 0x50)...")
        ee.store(record)
        print("  stored and verified in 3 redundant copies")
        print("\nThis camera now carries its own calibration. Any Pi that "
              "sees it")
        print("will read it back with radcam.calibration.apply().")

    if args.apply:
        # The installed tuning lives under /usr/local, so this needs root.
        # Re-exec just the apply step rather than making the whole tool sudo.
        print("\napplying to the installed libcamera tuning...")
        rec_json = json.dumps(record)
        r = subprocess.run(
            ["sudo", sys.executable, "-c",
             "import sys, json; sys.path.insert(0, %r);"
             " from radcam import calibration;"
             " calibration.apply(json.loads(sys.argv[1]))" % str(ROOT),
             rec_json], capture_output=True, text=True)
        if r.returncode != 0:
            print("  FAILED:", (r.stderr or r.stdout).strip().splitlines()[-1:])
            return 1
        print("  installed; the next capture uses it")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
