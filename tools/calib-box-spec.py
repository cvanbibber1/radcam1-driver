#!/usr/bin/env python3
"""Dimension the calibration box from the camera's actual field of view.

The appealing design is a horn whose walls *are* the field-of-view pyramid, so
a flat target anywhere along it fills the frame exactly. That works near the
lens and fails badly at range: at 120 degrees diagonal, a 400 mm station needs
a mouth over 1.5 m across. Unprintable, and unnecessary - only the tests that
need full frame coverage (flat field, distortion, FOV) require it, and those
run at the nearest station. Focus needs a sharp feature on axis and tolerates
seeing the walls.

So the body flares along the FOV to one plane and runs parallel past it, and
this script sizes both zones. Rationale and the built spec are in
docs/calibration-box.md.

The FOV is a measured input, not a guess - see "Measuring your FOV first":

    python3 tools/calib-box-spec.py --dfov 120
    python3 tools/calib-box-spec.py --dfov 95 --stations 60,120,240,400

Everything downstream - plate sizes, section lengths, print volumes - follows.
"""

import argparse
import math

# Active area used by the driver's full-resolution mode. The AR1335 is a
# 1/3.2" part with 1.1 um pixels, so this is 4096 x 3072 x 1.1 um.
SENSOR_W_MM = 4096 * 1.1e-3
SENSOR_H_MM = 3072 * 1.1e-3
ASPECT_W, ASPECT_H = 4.0, 3.0


def fov_axes(dfov_deg: float):
    """Split a diagonal FOV into horizontal and vertical, for a 4:3 frame."""
    diag = math.hypot(ASPECT_W, ASPECT_H)          # 5 for 4:3
    t = math.tan(math.radians(dfov_deg) / 2.0)
    h = 2 * math.degrees(math.atan(t * ASPECT_W / diag))
    v = 2 * math.degrees(math.atan(t * ASPECT_H / diag))
    return h, v


def frame_at(distance_mm: float, hfov: float, vfov: float):
    w = 2 * distance_mm * math.tan(math.radians(hfov) / 2)
    h = 2 * distance_mm * math.tan(math.radians(vfov) / 2)
    return w, h


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dfov", type=float, default=120.0,
                    help="measured diagonal field of view, degrees")
    ap.add_argument("--stations", default="60,120,240,400",
                    help="target distances from the lens, mm")
    ap.add_argument("--margin", type=float, default=8.0,
                    help="extra half-angle on the horn walls, degrees")
    ap.add_argument("--print-bed", type=float, default=220.0,
                    help="usable print bed dimension, mm")
    ap.add_argument("--flare-plane", type=float, default=60.0,
                    help="distance at which the horn reaches full section, mm")
    args = ap.parse_args()

    hfov, vfov = fov_axes(args.dfov)
    hw, vw = hfov + 2 * args.margin, vfov + 2 * args.margin
    stations = [float(s) for s in args.stations.split(",")]
    far = max(stations)

    print("CALIBRATION HORN - DIMENSIONS")
    print("=" * 62)
    print(f"\nSensor: {SENSOR_W_MM:.2f} x {SENSOR_H_MM:.2f} mm active "
          f"(4096 x 3072 at 1.1 um)")
    print(f"Field of view: {args.dfov:.1f} deg diagonal")
    print(f"  horizontal {hfov:.1f} deg      vertical {vfov:.1f} deg")
    print(f"Horn walls cut {args.margin:.0f} deg wider per side "
          f"({hw:.1f} x {vw:.1f} deg) so the frame never sees the wall edge.")

    print("\nTARGET PLATE SIZES  (a plate at each station fills the frame)")
    print(f"\n  {'station':>9} {'frame WxH':>18} {'plate WxH (+10mm)':>22}")
    for d in stations:
        w, h = frame_at(d, hfov, vfov)
        print(f"  {d:7.0f}mm {w:8.1f} x {h:6.1f} mm "
              f"{w+10:12.0f} x {h+10:4.0f} mm")

    # A horn that follows the FOV all the way to the far station is enormous:
    # at 120 deg and 400 mm the mouth is over 1.5 m. Only the tests that need
    # the target to fill the frame - flat field, distortion, FOV - require full
    # coverage, and those work at the nearest station. Focus does not: it needs
    # a sharp feature on axis, and the walls may appear around it.
    #
    # So the body flares to full section at one plane, then runs parallel.
    fw, fh = frame_at(args.flare_plane, hw, vw)
    print("\nBODY ENVELOPE  (flare to full section, then parallel)")
    print(f"  flare length (lens to full section) : {args.flare_plane:.0f} mm")
    print(f"  internal section beyond that        : {fw:.0f} x {fh:.0f} mm")
    print(f"  total internal length               : {far + 40:.0f} mm "
          f"(far station + 40 mm behind it)")
    print(f"  throat at the lens                  : 20 x 20 mm (clears the LEDs)")

    naive_w, naive_h = frame_at(far, hw, vw)
    print(f"\n  A horn following the FOV the whole way would need a "
          f"{naive_w:.0f} x {naive_h:.0f} mm")
    print(f"  mouth - unprintable, and unnecessary.")

    n = math.ceil((far + 40) / (args.print_bed * 0.9))
    print(f"\nPRINTING")
    print(f"  bed {args.print_bed:.0f} mm -> {n} sections of "
          f"~{(far+40)/n:.0f} mm, joined by the flanges in the spec")
    print(f"  section footprint : {fw:.0f} x {fh:.0f} mm", end="")
    if fw > args.print_bed:
        print(f"  -> exceeds the bed; print each")
        print(f"                      section as two half-shells split "
              f"lengthwise")
    else:
        print("  (fits)")

    print("\nSANITY CHECKS")
    wf, hf = frame_at(args.flare_plane, hfov, vfov)
    print(f"  full-frame target at {args.flare_plane:.0f} mm is "
          f"{wf:.0f} x {hf:.0f} mm")
    print(f"    - flat field, distortion grid and FOV scale go here")
    print(f"    - A4 paper (210 x 297) covers it: "
          f"{'yes' if wf <= 297 and hf <= 210 else 'NO, use A3'}")
    dkk = 90.0
    print(f"  a {dkk:.0f} mm DKK chart fills the frame width at "
          f"{dkk/(2*math.tan(math.radians(hfov)/2)):.0f} mm;")
    print(f"    at the {args.flare_plane:.0f} mm station it spans "
          f"{100*dkk/wf:.0f}% of the width, which the detector handles")
    for d in stations:
        w, h = frame_at(d, hfov, vfov)
        px_per_mm = 4096 / w
        print(f"  {d:5.0f}mm: {px_per_mm:6.1f} px/mm "
              f"({1000/px_per_mm:5.1f} um per pixel at the target)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
