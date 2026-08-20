"""Colour calibration maths: chart LAB targets -> white balance and CCM.

The camera's raw RGB is whatever its colour filters happen to pass. Turning
that into accurate colour needs two things measured against a known reference:

  white balance  per-channel gains that make a neutral patch neutral
  CCM            a 3x3 matrix mapping white-balanced camera RGB onto the
                 target colour space, correcting for the filters' overlap

Both come from photographing a chart with known LAB values. This module holds
the DGK DKK chart's patch definitions, the colour-space conversions, and the
least-squares solve.

Convention: the CCM's rows are normalised to sum to 1. That makes the matrix
preserve neutrals, so it corrects hue and saturation without disturbing the
white balance the AWB has already set - which is what libcamera's pipeline
expects, since it applies gains first and the matrix afterwards.
"""

from __future__ import annotations

import numpy as np

#: DGK DKK chart, 6 columns x 3 rows, listed top-left to bottom-right.
#: Row 0 is the neutral wedge; rows 1-2 are chromatic patches.
DKK_PATCHES_LAB = [
    # row 0 - neutrals, white through black
    (100.0, 0.0, 0.0), (73.0, 0.0, 0.0), (62.0, 0.0, 0.0),
    (50.0, 0.0, 0.0), (38.0, 0.0, 0.0), (0.0, 0.0, 0.0),
    # row 1 - primaries and secondaries
    (52.0, 74.0, 54.0), (95.0, -6.0, 95.0), (69.0, -43.0, 50.0),
    (62.0, -44.0, -50.0), (11.0, 10.0, -39.0), (52.0, 81.0, -7.0),
    # row 2 - memory / skin / foliage tones
    (41.0, 51.0, 26.0), (61.0, 29.0, 57.0), (52.0, -24.0, -24.0),
    (52.0, 47.0, -14.0), (69.0, 14.0, 17.0), (64.0, 12.0, 17.0),
]

CHART_COLS, CHART_ROWS = 6, 3

#: Indices of the neutral patches, used for white balance. The extremes are
#: excluded: white often clips and black is dominated by flare and noise, so
#: neither gives a trustworthy ratio.
NEUTRAL_INDICES = (1, 2, 3, 4)

# Reference whites (2 degree observer).
WHITE_D50 = np.array([0.96422, 1.00000, 0.82521])
WHITE_D65 = np.array([0.95047, 1.00000, 1.08883])

# XYZ (D65) -> linear sRGB
XYZ_TO_SRGB = np.array([
    [3.2404542, -1.5371385, -0.4985314],
    [-0.9692660, 1.8760108, 0.0415560],
    [0.0556434, -0.2040259, 1.0572252],
])

# Bradford chromatic adaptation, D50 -> D65.
BRADFORD_D50_TO_D65 = np.array([
    [0.9555766, -0.0230393, 0.0631636],
    [-0.0282895, 1.0099416, 0.0210077],
    [0.0122982, -0.0204830, 1.3299098],
])


def lab_to_xyz(lab, white=WHITE_D50) -> np.ndarray:
    """CIE LAB -> XYZ under the given reference white."""
    L, a, b = np.asarray(lab, dtype=float).T
    fy = (L + 16.0) / 116.0
    fx = fy + a / 500.0
    fz = fy - b / 200.0

    def finv(t):
        t = np.asarray(t, dtype=float)
        # Linear segment below the cube-root knee, as the standard defines.
        return np.where(t > 6.0 / 29.0, t ** 3,
                        3.0 * (6.0 / 29.0) ** 2 * (t - 4.0 / 29.0))

    return np.stack([finv(fx) * white[0],
                     finv(fy) * white[1],
                     finv(fz) * white[2]], axis=-1)


def xyz_to_linear_srgb(xyz, adapt_from_d50: bool = True) -> np.ndarray:
    """XYZ -> linear sRGB, optionally adapting D50 -> D65 first."""
    xyz = np.asarray(xyz, dtype=float)
    if adapt_from_d50:
        xyz = xyz @ BRADFORD_D50_TO_D65.T
    return xyz @ XYZ_TO_SRGB.T


def chart_targets_linear_srgb(patches=None, adapt_from_d50: bool = True):
    """Target linear-sRGB values for every chart patch."""
    lab = np.array(patches if patches is not None else DKK_PATCHES_LAB)
    return xyz_to_linear_srgb(lab_to_xyz(lab), adapt_from_d50)


def white_balance_gains(camera_rgb, neutral_indices=NEUTRAL_INDICES):
    """Gains that make the neutral patches neutral.

    Returns (r_gain, b_gain) relative to green, plus the raw R/G and B/G that
    the tuning file's ct_curve needs.
    """
    c = np.asarray(camera_rgb, dtype=float)
    neutral = c[list(neutral_indices)]
    # Normalise each patch by its own green first, so a bright patch does not
    # dominate a dark one; then average the ratios.
    ratios = neutral / neutral[:, 1:2]
    r_over_g = float(np.mean(ratios[:, 0]))
    b_over_g = float(np.mean(ratios[:, 2]))
    return (1.0 / r_over_g, 1.0 / b_over_g), (r_over_g, b_over_g)


def solve_ccm(camera_rgb, target_rgb, weights=None,
              preserve_white: bool = True) -> np.ndarray:
    """Least-squares 3x3 matrix mapping camera RGB onto target RGB.

    `camera_rgb` must already be white balanced. Rows are renormalised to sum
    to 1 so the matrix preserves neutrals.
    """
    C = np.asarray(camera_rgb, dtype=float)
    T = np.asarray(target_rgb, dtype=float)
    if C.shape != T.shape:
        raise ValueError(f"shape mismatch: {C.shape} vs {T.shape}")

    if weights is None:
        w = np.ones(len(C))
    else:
        w = np.asarray(weights, dtype=float)
    W = np.sqrt(w)[:, None]

    # Solve each output channel independently: M[i] . camera = target[:, i]
    M, *_ = np.linalg.lstsq(C * W, T * W, rcond=None)
    M = M.T

    if preserve_white:
        rows = M.sum(axis=1, keepdims=True)
        rows[rows == 0] = 1.0
        M = M / rows
    return M


def solve_ccm_shading_invariant(camera_rgb, target_rgb, weights=None,
                                iters: int = 80) -> np.ndarray:
    """Least-squares CCM that ignores how bright each patch happens to be.

    `solve_ccm` matches absolute RGB, which quietly assumes every patch was lit
    and imaged with the same gain. Behind an uncalibrated wide lens that is
    false: vignetting darkens the edges of the frame by tens of percent, and a
    chart big enough to measure spans exactly that falloff. The fit then spends
    its freedom describing the vignette as though it were colour, and the
    matrix it produces is wrong everywhere.

    Lens shading is very nearly achromatic, so it slides a patch along its own
    brightness axis and leaves its chromaticity where it was. Give every patch
    its own free scale and fit only the direction:

        minimise  sum_i  w_i * || M . cam_i  -  k_i * target_i ||^2

    k has a closed form once M is fixed, and M is ordinary least squares once k
    is fixed, so alternating the two converges in a handful of passes. Rows are
    held to sum to 1 throughout, which is what keeps neutrals neutral.
    """
    C = np.asarray(camera_rgb, dtype=float)
    T = np.asarray(target_rgb, dtype=float)
    if C.shape != T.shape:
        raise ValueError(f"shape mismatch: {C.shape} vs {T.shape}")
    w = np.ones(len(C)) if weights is None else np.asarray(weights, dtype=float)
    sw = np.sqrt(np.maximum(w, 0.0))

    # Row j is [a, b, 1-a-b], so the constant part is the camera's blue and the
    # free part is how much red and green get mixed in over it.
    A = np.stack([C[:, 0] - C[:, 2], C[:, 1] - C[:, 2]], axis=1) * sw[:, None]

    M = np.eye(3)
    for _ in range(iters):
        P = C @ M.T
        den = (T * T).sum(axis=1)
        k = np.where(den > 1e-12, (P * T).sum(axis=1) / np.maximum(den, 1e-12), 1.0)
        rows = []
        for j in range(3):
            y = (k * T[:, j] - C[:, 2]) * sw
            (a, b), *_ = np.linalg.lstsq(A, y, rcond=None)
            rows.append([a, b, 1.0 - a - b])
        new = np.array(rows)
        if np.abs(new - M).max() < 1e-10:
            M = new
            break
        M = new
    return M


def delta_e_76(lab_a, lab_b) -> np.ndarray:
    """CIE76 colour difference. Crude but the standard first-pass metric."""
    a = np.asarray(lab_a, dtype=float)
    b = np.asarray(lab_b, dtype=float)
    return np.sqrt(((a - b) ** 2).sum(axis=-1))


def linear_srgb_to_lab(rgb, white=WHITE_D50, adapt_to_d50: bool = True):
    """Inverse of the target pipeline, for scoring a calibration."""
    rgb = np.asarray(rgb, dtype=float)
    xyz = rgb @ np.linalg.inv(XYZ_TO_SRGB).T
    if adapt_to_d50:
        xyz = xyz @ np.linalg.inv(BRADFORD_D50_TO_D65).T

    xyz = xyz / white
    def f(t):
        t = np.asarray(t, dtype=float)
        return np.where(t > (6.0 / 29.0) ** 3, np.cbrt(np.maximum(t, 1e-12)),
                        t / (3.0 * (6.0 / 29.0) ** 2) + 4.0 / 29.0)
    fx, fy, fz = f(xyz[..., 0]), f(xyz[..., 1]), f(xyz[..., 2])
    return np.stack([116.0 * fy - 16.0, 500.0 * (fx - fy),
                     200.0 * (fy - fz)], axis=-1)


def score_calibration(camera_rgb, ccm, targets_lab=None):
    """Apply a CCM and report per-patch and mean Delta E against the chart."""
    lab_ref = np.array(targets_lab if targets_lab is not None
                       else DKK_PATCHES_LAB)
    corrected = np.asarray(camera_rgb, dtype=float) @ np.asarray(ccm).T
    lab_got = linear_srgb_to_lab(np.clip(corrected, 0, None))
    # Luminance is set by exposure, not by the matrix, so compare chromaticity
    # by lifting each corrected patch to the reference lightness.
    lab_got[:, 0] = lab_ref[:, 0]
    de = delta_e_76(lab_got, lab_ref)
    return de, lab_got


def calibrate_from_patches(camera_rgb, target_rgb=None, weights=None,
                           black_index: int = 5,
                           neutral_indices=NEUTRAL_INDICES):
    """Solve glare floor, white balance and colour matrix from sampled patches.

    The three are not independent, which is why they are fitted together.
    Veiling glare is *additive*: light scattered inside the lens lands on the
    sensor as a broad pedestal, so it survives every gain and every matrix
    downstream, and it is what makes dark saturated patches read as washed-out
    greys. Get it wrong and the white balance and the matrix both bend trying
    to absorb it.

    The black patch measures the pedestal directly, but only at the point of
    the frame where the black patch happens to sit; lens shading means the
    figure is too large elsewhere, and subtracting it wholesale drives patches
    in the dim corners straight through zero. So use the black patch to set the
    scale of the search only, and pick the actual offset that makes the whole
    chart most self-consistent.

    A plain pattern search does the optimisation - three parameters, a cheap
    objective, and no need for a solver dependency.

    Returns a dict with `offset`, `gains`, `ratios`, `ccm`, `wb_rgb`, `delta_e`
    and `delta_e_identity`.
    """
    C = np.asarray(camera_rgb, dtype=float)
    T = (np.asarray(target_rgb, dtype=float) if target_rgb is not None
         else chart_targets_linear_srgb())
    if weights is None:
        weights = np.ones(len(C))
        # The black patch defines the offset, so scoring the fit against it
        # only measures how hard we subtracted it.
        if 0 <= black_index < len(C):
            weights[black_index] = 0.0
    w = np.asarray(weights, dtype=float)
    base = C[black_index].copy() if 0 <= black_index < len(C) else C.min(axis=0)

    def evaluate(frac, allow_floor: bool = False):
        cc = C - base * np.clip(np.asarray(frac, dtype=float), 0.0, 1.4)
        # One patch pushed to the floor is a dark corner; several means the
        # offset is simply too big. The zero-offset case is exempt: it
        # subtracts nothing, so patches at the floor there are the caller's
        # data, not this function's doing - and it has to stay available as the
        # fallback, or a dark capture leaves nothing to fall back to.
        if not allow_floor and int((cc[:, 1] <= 1e-6).sum()) > 1:
            return None
        cc = np.maximum(cc, 1e-6)
        (rg, bg), ratios = white_balance_gains(cc, neutral_indices)
        wb = cc * np.array([rg, 1.0, bg])
        wb = wb / max(wb[0, 1], 1e-9)
        M = solve_ccm_shading_invariant(wb, T, weights=w)
        de, _ = score_calibration(wb, M)
        return {"score": float(np.average(de, weights=w)), "frac": np.array(frac),
                "gains": (rg, bg), "ratios": ratios, "wb_rgb": wb, "ccm": M,
                "delta_e": de}

    frac = np.array([0.85, 0.85, 0.85])
    best = evaluate(frac) or evaluate([0.0, 0.0, 0.0], allow_floor=True)
    step = 0.30
    while step > 0.004:
        improved = False
        for i in range(3):
            for s in (step, -step):
                trial = best["frac"].copy()
                trial[i] = float(np.clip(trial[i] + s, 0.0, 1.4))
                r = evaluate(trial)
                if r is not None and r["score"] < best["score"]:
                    best, improved = r, True
        if not improved:
            step *= 0.5

    de_id, _ = score_calibration(best["wb_rgb"], np.eye(3))
    best["offset"] = base * best["frac"]
    best["delta_e_identity"] = de_id
    return best
