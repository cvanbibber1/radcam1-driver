#!/usr/bin/env python3
"""Generate a starting libcamera/PiSP tuning file for the AR1335.

A tuning file is normally produced by photographing calibration targets with
the actual module and lens. We cannot do that until the sensor responds, so
this derives a *starting point* from the IMX519 tuning, which is the closest
sensor Raspberry Pi ships: same class of small-pixel, high-resolution Bayer
part.

What is carried over, and why that is defensible:

  agc, awb, contrast, denoise, sharpen, noise, geq, dpc, lux
      Largely sensor-agnostic algorithm tuning. Reasonable defaults for any
      Bayer sensor; refine once real images exist.

  ccm
      Colour correction is sensor-specific, but a plausible matrix from a
      similar sensor gives far better colour than the identity matrix would.
      Must be re-measured against a colour chart.

What is deliberately NOT carried over:

  alsc (lens shading)
      This is a property of the *lens and module*, not the sensor. Applying
      IMX519's shading tables to a different optic would actively introduce
      colour and brightness errors across the frame. The calibration tables
      are therefore flattened to neutral, so images start uncorrected rather
      than wrongly corrected.

Re-run with:  python3 tools/gen-tuning.py
"""

import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC = ROOT / "libcamera-src" / "src" / "ipa" / "rpi" / "pisp" / "data" / "imx519.json"
OUT = ROOT / "libcamera" / "ar1335.json"

# Black level in 16-bit units, MEASURED from raw AR1335 frames rather than
# assumed: the pedestal sits at 42 LSB at 10 bits = 2688.
#
# This was originally guessed at 4096 (64 LSB), copied from the IMX519 tuning.
# That is close to the *median* of a normally exposed frame, not its black
# level, so the ISP subtracted far too much and crushed the weakest channel:
# blue fell from 1846 to 374 and images came out with almost no blue at all.
# Re-measure with tools/measure-black-level.py if the sensor is ever changed.
BLACK_LEVEL = 2688


def neutralise_alsc(alsc: dict) -> dict:
    """Flatten lens-shading tables so nothing is corrected."""
    out = dict(alsc)

    for key in ("calibrations_Cr", "calibrations_Cb"):
        if key in out:
            flattened = []
            for entry in out[key]:
                e = dict(entry)
                if "table" in e:
                    e["table"] = [1.0] * len(e["table"])
                flattened.append(e)
            out[key] = flattened

    if "luminance_lut" in out:
        out["luminance_lut"] = [1.0] * len(out["luminance_lut"])

    # With flat tables there is nothing to solve for, so do not spend the
    # iterations on it.
    out["luminance_strength"] = 0.0
    return out


def main() -> int:
    if not SRC.is_file():
        sys.exit(f"reference tuning not found: {SRC}\n"
                 "Clone the libcamera source first.")

    src = json.loads(SRC.read_text())
    algorithms = []

    for algo in src["algorithms"]:
        name = next(iter(algo))
        body = algo[name]

        if name == "rpi.black_level":
            body = {"black_level": BLACK_LEVEL}
        elif name == "rpi.alsc":
            body = neutralise_alsc(body)

        algorithms.append({name: body})

    out = {
        "version": src.get("version", 2.0),
        "target": src.get("target", "pisp"),
        "algorithms": algorithms,
    }

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(out, indent=4) + "\n")

    print(f"wrote {OUT}")
    print(f"  algorithms: {', '.join(next(iter(a)) for a in algorithms)}")
    print(f"  black_level: {BLACK_LEVEL} (16-bit) = {BLACK_LEVEL >> 6} at 10 bits")
    print("  lens shading: neutralised - recalibrate with the real optic")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
