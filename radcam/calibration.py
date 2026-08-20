"""Apply a camera's own calibration, read from its EEPROM.

Cameras and Pis are interchangeable, so the host must not assume anything about
which module it is looking at. At startup this reads the calibration the camera
is carrying and writes it into the libcamera tuning file, so the ISP uses the
right black level, white balance and colour matrix for *that* module.

If the EEPROM is blank or unreadable the tuning file is left alone: an
uncalibrated camera should produce mediocre colour, not no images.
"""

from __future__ import annotations

import json
import logging
import shutil
from pathlib import Path
from typing import Any

from . import shading as shading_model
from .eeprom import CameraEEPROM

log = logging.getLogger(__name__)

TUNING_PATHS = [
    Path("/usr/local/share/libcamera/ipa/rpi/pisp/ar1335.json"),
    Path("/usr/share/libcamera/ipa/rpi/pisp/ar1335.json"),
]

#: Colour temperatures the ct_curve is emitted at. The measured point anchors
#: the curve; the shape either side comes from the illuminants themselves.
CURVE_CTS = [2200, 2700, 3200, 4000, 5000, 6500, 8000]

#: Shape of the borrowed curve, as ratios relative to its own 4500 K point.
#: Rescaled by the camera's measured anchor, this puts the curve on the right
#: sensor without needing a full multi-illuminant fit from every unit.
_SHAPE = {2200: (1.55, 0.55), 2700: (1.36, 0.66), 3200: (1.22, 0.76),
          4000: (1.08, 0.89), 5000: (1.00, 1.00), 6500: (0.90, 1.12),
          8000: (0.84, 1.20)}


def _tuning_path() -> Path | None:
    for p in TUNING_PATHS:
        if p.is_file():
            return p
    return None


def build_ct_curve(r_over_g: float, b_over_g: float, ct: float) -> list[float]:
    """Curve passing through (ct, r_over_g, b_over_g), flat-shaped elsewhere."""
    # Interpolate the shape at the anchor temperature.
    cts = sorted(_SHAPE)
    import numpy as np
    sr = float(np.interp(ct, cts, [_SHAPE[c][0] for c in cts]))
    sb = float(np.interp(ct, cts, [_SHAPE[c][1] for c in cts]))
    base_r, base_b = r_over_g / sr, b_over_g / sb

    out: list[float] = []
    for c in CURVE_CTS:
        out += [float(c), round(base_r * _SHAPE[c][0], 5),
                round(base_b * _SHAPE[c][1], 5)]
    return out


def apply(record: dict[str, Any], tuning: Path | None = None,
          backup: bool = True) -> bool:
    """Write a calibration record into the tuning file. True if it changed."""
    path = tuning or _tuning_path()
    if path is None:
        log.error("no ar1335 tuning file found")
        return False

    try:
        doc = json.loads(path.read_text())
    except Exception as exc:
        log.error("cannot read tuning file %s: %s", path, exc)
        return False

    changed = False
    for algo in doc["algorithms"]:
        name = next(iter(algo))

        if name == "rpi.black_level" and "black_level" in record:
            if algo[name]["black_level"] != record["black_level"]:
                algo[name]["black_level"] = int(record["black_level"])
                changed = True

        elif name == "rpi.awb" and "awb" in record:
            curve = build_ct_curve(record["awb"]["r_over_g"],
                                   record["awb"]["b_over_g"],
                                   record.get("illuminant_ct", 5000.0))
            if algo[name].get("ct_curve") != curve:
                algo[name]["ct_curve"] = curve
                changed = True

        elif name == "rpi.alsc" and "shading" in record:
            t = shading_model.tables(record["shading"])
            # luminance_strength gates the vignette correction, and the stock
            # tuning ships it at 0.0 - the tables are read but do nothing.
            # Measuring shading and leaving this at zero is a silent no-op.
            want = {
                "luminance_lut": t["luminance_lut"],
                "luminance_strength": 1.0,
                "calibrations_Cr": [{"ct": 5000, "table": t["calibrations_Cr"]}],
                "calibrations_Cb": [{"ct": 5000, "table": t["calibrations_Cb"]}],
            }
            if any(algo[name].get(k) != v for k, v in want.items()):
                algo[name].update(want)
                changed = True

        elif name == "rpi.ccm" and "ccm" in record:
            ct = float(record.get("illuminant_ct", 5000.0))
            entry = {"ct": ct,
                     "ccm": [v for row in record["ccm"] for v in row]}
            # A single measured matrix replaces the inherited set: one right
            # matrix beats several wrong ones.
            if algo[name].get("ccms") != [entry]:
                algo[name]["ccms"] = [entry]
                changed = True

    if not changed:
        log.info("tuning already matches the camera's calibration")
        return False

    if backup:
        shutil.copy(path, path.with_suffix(".json.precal"))
    path.write_text(json.dumps(doc, indent=4) + "\n")
    log.info("applied calibration for camera %s to %s",
             record.get("camera_id", "?"), path)
    return True


def load_and_apply(bus: int = 4) -> dict[str, Any] | None:
    """Read the camera's EEPROM and apply whatever it carries."""
    ee = CameraEEPROM(bus=bus)
    if not ee.present():
        log.info("no camera EEPROM on i2c-%d", bus)
        return None

    record = ee.load()
    if record is None:
        log.warning("camera EEPROM carries no valid calibration; "
                    "using the default tuning")
        return None

    log.info("camera %s calibrated %s (mean dE %.2f)",
             record.get("camera_id", "?"),
             record.get("calibrated_utc", "?"),
             record.get("quality", {}).get("mean_de76", float("nan")))
    apply(record)
    return record


def main(argv: list[str] | None = None) -> int:
    """Entry point for the boot-time unit: read the EEPROM, apply, report.

    Exits 0 even when there is nothing to apply. A camera with no calibration
    must still produce images, so an uncalibrated module is not a boot failure
    - and a unit that fails would hold up everything ordered after it.
    """
    import argparse

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bus", type=int, default=4,
                    help="camera I2C bus (4 = CAM1, 6 = CAM0)")
    ap.add_argument("--any-bus", action="store_true",
                    help="try both camera buses and use whichever answers")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    buses = [args.bus] if not args.any_bus else [args.bus] + [
        b for b in (4, 6) if b != args.bus]

    for bus in buses:
        try:
            record = load_and_apply(bus=bus)
        except Exception as exc:                       # noqa: BLE001
            log.warning("i2c-%d: %s", bus, exc)
            continue
        if record is not None:
            return 0
    log.info("no stored calibration applied; using the default tuning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
