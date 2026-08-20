"""Lens shading: measure it, store it compactly, expand it for libcamera.

A wide lens delivers far less light to the corners of the sensor than to the
centre, and it does so by a different amount in each colour channel. Left
uncorrected that is both a brightness vignette and a colour cast that changes
across the frame - and it quietly corrupts every other calibration, because a
patch measured in a corner is not measured under the same conditions as one in
the middle.

Storage is the awkward part. libcamera wants three 32x32 tables, 1024 doubles
each, and the module's EEPROM has about 2 KB for the entire calibration record.
So this stores a *model* rather than a grid: vignetting is smooth and very
nearly radial, so a few coefficients describe it to well within the measurement
noise, and the grid is regenerated on the host when the calibration is applied.

    response(rho) = 1 + a1*rho^2 + a2*rho^4 + a3*rho^6

with rho the distance from the optical centre, normalised so rho = 1 at the
corner. The optical centre is fitted rather than assumed: a sensor is never
perfectly aligned to its lens, and forcing the model to be centred puts the
error back as a false left-right colour gradient.

Twelve numbers replace 3072.
"""

from __future__ import annotations

import numpy as np

#: Side of the square table libcamera's rpi.alsc expects (1024 entries).
TABLE_SIZE = 32


def _basis(rho2: np.ndarray) -> np.ndarray:
    """Design matrix for the even radial polynomial, without its constant."""
    return np.stack([rho2, rho2 ** 2, rho2 ** 3], axis=-1)


def _rho2(shape, centre) -> np.ndarray:
    """Squared normalised radius for every cell of a (h, w) grid."""
    h, w = shape
    cx, cy = centre
    ys, xs = np.mgrid[0:h, 0:w]
    # Cell centres in 0..1, so the model does not depend on grid resolution.
    u = (xs + 0.5) / w - cx
    v = (ys + 0.5) / h - cy
    # Normalise so rho = 1 at the farthest corner of a centred frame.
    return (u * u + v * v) / 0.5 ** 2 / 2.0


def fit_channel(plane: np.ndarray, centre) -> tuple[np.ndarray, float]:
    """Fit the radial response of one downsampled channel. Returns (coeff, rms).

    The plane is normalised by its value at the optical centre, so coefficients
    describe fall-off relative to the middle of the lens regardless of exposure.
    """
    rho2 = _rho2(plane.shape, centre)
    # Value at the centre, from the model rather than a single noisy cell.
    A = np.concatenate([np.ones(rho2.size)[:, None],
                        _basis(rho2.ravel())], axis=1)
    coef, *_ = np.linalg.lstsq(A, plane.ravel(), rcond=None)
    c0 = coef[0]
    if abs(c0) < 1e-9:
        return np.zeros(3), float("inf")
    coeff = coef[1:] / c0
    pred = 1.0 + _basis(rho2) @ coeff
    rms = float(np.sqrt(np.mean((plane / c0 - pred) ** 2)))
    return coeff, rms


def fit(planes: dict[str, np.ndarray], search: bool = True):
    """Fit all channels, sharing one optical centre.

    The centre is found by minimising the green channel's residual: green has
    the most signal, and using one centre for all three channels is what keeps
    the colour ratios physical - three independent centres would let the model
    invent a colour gradient that is really just a fitting artefact.
    """
    best = (float("inf"), (0.5, 0.5))
    if search:
        for cy in np.linspace(0.35, 0.65, 13):
            for cx in np.linspace(0.35, 0.65, 13):
                _, rms = fit_channel(planes["g"], (cx, cy))
                if rms < best[0]:
                    best = (rms, (float(cx), float(cy)))
        # Refine around the coarse winner.
        cx0, cy0 = best[1]
        for cy in np.linspace(cy0 - 0.03, cy0 + 0.03, 13):
            for cx in np.linspace(cx0 - 0.03, cx0 + 0.03, 13):
                _, rms = fit_channel(planes["g"], (cx, cy))
                if rms < best[0]:
                    best = (rms, (float(cx), float(cy)))
    centre = best[1]

    coeffs, resid = {}, {}
    for name, plane in planes.items():
        c, rms = fit_channel(plane, centre)
        coeffs[name] = [float(v) for v in c]
        resid[name] = rms
    return centre, coeffs, resid


def response(coeff, shape=(TABLE_SIZE, TABLE_SIZE), centre=(0.5, 0.5)):
    """Evaluate a fitted channel response over a grid."""
    rho2 = _rho2(shape, centre)
    return 1.0 + _basis(rho2) @ np.asarray(coeff, dtype=float)


def corner_falloff(coeff, centre=(0.5, 0.5)) -> float:
    """Response at the frame corner as a fraction of the centre."""
    grid = response(coeff, (64, 64), centre)
    return float(min(grid[0, 0], grid[0, -1], grid[-1, 0], grid[-1, -1]))


def tables(record_shading) -> dict[str, list[float]]:
    """Expand a stored model into the three tables rpi.alsc reads.

    The conventions come from libcamera's alsc.cpp, which is worth stating
    because getting them backwards produces a plausible-looking table that
    doubles the vignette instead of removing it:

      luminance_lut    the gain green needs, so 1 / response_G
      calibrations_Cr  the gain red needs *relative to green*, so s_G / s_R
      calibrations_Cb  likewise for blue

    Each is normalised so its minimum is 1 - the algorithm renormalises anyway,
    but writing them that way makes a stored table readable by eye.
    """
    centre = tuple(record_shading.get("centre", (0.5, 0.5)))
    coeff = record_shading["coeff"]
    shape = (TABLE_SIZE, TABLE_SIZE)
    s_r = np.maximum(response(coeff["r"], shape, centre), 1e-6)
    s_g = np.maximum(response(coeff["g"], shape, centre), 1e-6)
    s_b = np.maximum(response(coeff["b"], shape, centre), 1e-6)

    def norm(t):
        t = np.asarray(t, dtype=float)
        return (t / max(t.min(), 1e-9)).ravel()

    return {
        "luminance_lut": [round(float(v), 5) for v in norm(1.0 / s_g)],
        "calibrations_Cr": [round(float(v), 5) for v in norm(s_g / s_r)],
        "calibrations_Cb": [round(float(v), 5) for v in norm(s_g / s_b)],
    }
