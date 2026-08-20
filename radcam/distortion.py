"""Radial lens distortion: fit it from straight edges, then undo it.

This lens bows straight lines badly - a doorway or a desk edge near the frame
border curves visibly. Correcting it needs no chart and no special rig, only
the observation that certain things in the world are straight: the plumb-line
method fits the distortion coefficients that make curved image edges straight
again.

    r_undistorted = r * (1 + k1*r^2 + k2*r^4 + k3*r^6)

r is measured from the distortion centre and normalised so r = 1 at the corner
of the frame. The centre is fitted, not assumed: it is where the lens axis
actually meets the sensor, and forcing it to the middle of the image tilts the
whole correction.

The measure being minimised is each edge's *relative* straightness - its
perpendicular residual divided by its own length. Absolute residual would be
minimised by any set of coefficients that shrinks the image toward a point,
which is the classic way this fit goes wrong.

Only numpy is used: this runs on the flight Pi, and a computer-vision stack is
a large dependency to carry for one polynomial.
"""

from __future__ import annotations

import numpy as np


def normalise(points, shape, centre):
    """Pixel coordinates -> coordinates normalised so r = 1 at the corner."""
    h, w = shape
    s = np.hypot(w, h) / 2.0
    cx, cy = centre[0] * w, centre[1] * h
    p = np.asarray(points, dtype=float)
    return np.stack([(p[..., 0] - cx) / s, (p[..., 1] - cy) / s], axis=-1)


def radial_factor(r2, k) -> np.ndarray:
    """1 + k1 r^2 + k2 r^4 + k3 r^6, evaluated from r squared."""
    k = np.asarray(k, dtype=float)
    out = np.ones_like(r2)
    term = np.ones_like(r2)
    for ki in k:
        term = term * r2
        out = out + ki * term
    return out


#: The equidistant fisheye projection. A lens like this maps incidence angle
#: to radius linearly, r = f*theta, while a rectilinear image needs
#: r = f*tan(theta) - so undistorting is r_u = f*tan(r_d/f), with f the only
#: free parameter.
#:
#: This matters more than it looks. Fitting a polynomial in r to a 120-degree
#: fisheye is fitting the wrong function: the polynomial has no term shaped
#: like tan, so the search buys accuracy in mid-field by going wrong at the
#: edges, and lands on k1 < 0 with a huge k2. That solution scores well and is
#: physically nonsense - it folds the corners back on themselves.
MODEL_FISHEYE = "fisheye_equidistant"
MODEL_POLY = "radial_poly"


def radial_factor_fisheye(r, f) -> np.ndarray:
    """f*tan(r/f)/r, the equidistant-to-rectilinear stretch."""
    r = np.asarray(r, dtype=float)
    f = float(np.asarray(f).ravel()[0])
    if f <= 1e-6:
        return np.ones_like(r)
    # Stay inside the asymptote: beyond theta = pi/2 there is no rectilinear
    # image to map to, and tan runs away.
    theta = np.clip(r / f, 0.0, 1.45)
    return np.where(r > 1e-9, f * np.tan(theta) / np.maximum(r, 1e-12), 1.0)


def factor_for(r, params, model: str) -> np.ndarray:
    if model == MODEL_FISHEYE:
        return radial_factor_fisheye(r, params)
    return radial_factor(np.asarray(r) ** 2, params)


def undistort_points(xy, params, model: str = MODEL_POLY) -> np.ndarray:
    """Apply the forward model to normalised points."""
    xy = np.asarray(xy, dtype=float)
    r = np.sqrt((xy ** 2).sum(axis=-1))
    return xy * factor_for(r, params, model)[..., None]


def straightness(chain_xy) -> float:
    """Perpendicular RMS residual of a point chain, relative to its length.

    Dimensionless on purpose. A plain residual can always be reduced by making
    the whole picture smaller, so an absolute measure would reward coefficients
    that collapse the image rather than ones that straighten it.
    """
    p = np.asarray(chain_xy, dtype=float)
    if len(p) < 3:
        return 0.0
    mean = p.mean(axis=0)
    q = p - mean
    # Principal axis by SVD; the second singular value is the spread across it.
    _, sv, vt = np.linalg.svd(q, full_matrices=False)
    length = float(sv[0])
    if length < 1e-9:
        return 0.0
    perp = q @ vt[1]
    return float(np.sqrt(np.mean(perp ** 2)) / (length / np.sqrt(len(p))))


def fit(chains, shape, k_bounds=(-1.5, 3.0), search_centre: bool = True,
        n_terms: int = 2, verbose: bool = False, centre0=(0.5, 0.5),
        centre_range: float = 0.2, trim: float = 0.3, rounds: int = 3,
        model: str = MODEL_POLY):
    """Fit centre and radial coefficients that straighten the given chains.

    `chains` is a list of (N, 2) pixel-coordinate arrays, each one an edge
    believed to be straight in the world. Some of them will not be, however
    hard the caller filters: a rounded object, or two edges linked through a
    corner, looks like a long clean curve. Those are fitted and *then* thrown
    out, over `rounds` passes discarding the worst `trim` fraction each time,
    because a bad chain is far easier to recognise by how badly it fits than by
    how it looks beforehand.

    Returns (centre, k, score, baseline).
    """
    chains = [np.asarray(c, dtype=float) for c in chains if len(c) >= 8]
    if not chains:
        raise ValueError("no usable edge chains")

    def make_weights(cs):
        # sqrt of length, not length: a single 500-point chain would otherwise
        # carry more weight than six good 80-point ones and decide the fit on
        # its own.
        w = np.sqrt(np.array([len(c) for c in cs], dtype=float))
        return w / w.sum()

    active = list(chains)
    weights = make_weights(active)

    def score(centre, k, cs=None, ws=None):
        cs = active if cs is None else cs
        ws = weights if ws is None else ws
        total = 0.0
        for w, c in zip(ws, cs):
            xy = normalise(c, shape, centre)
            total += w * straightness(undistort_points(xy, k, model))
        return float(total)

    # The centre is measured from the illuminated disc and only nudged from
    # there: it trades off against k1 almost exactly, so a wide search finds
    # the same straightness at a physically wrong axis.
    centre = tuple(centre0)
    lo = (centre0[0] - centre_range, centre0[1] - centre_range)
    hi = (centre0[0] + centre_range, centre0[1] + centre_range)
    if model == MODEL_FISHEYE:
        n_terms = 1
        k = np.array([0.85])
        # f = r_disc / theta_max, so f is pinned by geometry far more tightly
        # than the straightness objective pins it. Any real lens sits between
        # about 80 and 180 degrees total field, which puts f in this range for
        # a disc filling most of the frame. Without the bound the search runs
        # off to f = 0.45, implying a 217-degree lens.
        k_bounds = (0.50, 1.30)
    else:
        k = np.zeros(n_terms)
    best = score(centre, k)
    # Baseline on exactly the objective being optimised, so the improvement
    # quoted afterwards is a like-for-like comparison.
    baseline = best

    # Pattern search: a few parameters, a cheap objective, and no solver
    # dependency. Coefficients and centre are refined alternately because they
    # trade off against each other - a centre error looks like extra k1.
    step_k = 0.25
    step_c = 0.04
    for _ in range(60):
        improved = False
        for i in range(n_terms):
            for s in (step_k, -step_k):
                trial = k.copy()
                trial[i] = float(np.clip(trial[i] + s, *k_bounds))
                v = score(centre, trial)
                if v < best - 1e-12:
                    best, k, improved = v, trial, True
        if search_centre:
            for axis in (0, 1):
                for s in (step_c, -step_c):
                    t = list(centre)
                    t[axis] = float(np.clip(t[axis] + s, lo[axis], hi[axis]))
                    v = score(tuple(t), k)
                    if v < best - 1e-12:
                        best, centre, improved = v, tuple(t), True
        if not improved:
            step_k *= 0.5
            step_c *= 0.5
            if step_k < 1e-4 and step_c < 1e-4:
                break
        if verbose:
            print(f"    k={np.round(k, 4)} centre={np.round(centre, 4)} "
                  f"score={best:.5f}")

    # Refit on the chains that actually behaved.
    for _ in range(max(rounds - 1, 0)):
        if len(active) <= 6:
            break
        resid = np.array([straightness(undistort_points(
            normalise(c, shape, centre), k, model)) for c in active])
        keep = np.argsort(resid)[:max(6, int(len(active) * (1.0 - trim)))]
        if len(keep) == len(active):
            break
        active = [active[i] for i in keep]
        weights = make_weights(active)
        step_k, step_c = 0.25, 0.04
        best = score(centre, k)
        for _ in range(60):
            improved = False
            for i in range(n_terms):
                for s in (step_k, -step_k):
                    trial = k.copy()
                    trial[i] = float(np.clip(trial[i] + s, *k_bounds))
                    v = score(centre, trial)
                    if v < best - 1e-12:
                        best, k, improved = v, trial, True
            if search_centre:
                for axis in (0, 1):
                    for s in (step_c, -step_c):
                        tc = list(centre)
                        tc[axis] = float(np.clip(tc[axis] + s, lo[axis],
                                                 hi[axis]))
                        v = score(tuple(tc), k)
                        if v < best - 1e-12:
                            best, centre, improved = v, tuple(tc), True
            if not improved:
                step_k *= 0.5
                step_c *= 0.5
                if step_k < 1e-4 and step_c < 1e-4:
                    break
    zero = np.array([1e6]) if model == MODEL_FISHEYE else np.zeros(n_terms)
    baseline = score(tuple(centre0), zero)
    return centre, k, best, baseline


def build_inverse_lut(k, r_max: float, n: int = 4096,
                      model: str = MODEL_POLY):
    """Table mapping undistorted radius back to distorted radius.

    Rendering a corrected image asks the opposite question from fitting: for
    each output pixel, where in the source did it come from? The forward model
    is not analytically invertible, but it is monotonic over the useful range,
    so tabulate it once and interpolate - far cheaper than solving per pixel.
    """
    r_d = np.linspace(0.0, r_max * 1.6, n)
    r_u = r_d * factor_for(r_d, k, model)
    # Use only the monotonic prefix. Past the first turning point the mapping
    # is not invertible at all - a strong k2, or the fisheye asymptote, folds
    # it back - and interpolating through a fold silently mirrors the corners.
    turn = np.nonzero(np.diff(r_u) <= 0)[0]
    end = int(turn[0]) + 1 if len(turn) else len(r_u)
    return r_u[:end], r_d[:end]


def undistort_image(img: np.ndarray, centre, k, scale: float | None = None,
                    out_shape=None, model: str = MODEL_POLY,
                    r_valid: float | None = None):
    """Remap an image through the fitted model. Bilinear, numpy only."""
    h, w = img.shape[:2]
    oh, ow = out_shape or (h, w)
    s = np.hypot(w, h) / 2.0
    cx, cy = centre[0] * w, centre[1] * h

    if scale is None:
        # Scale from the edge of the *illuminated disc*, not the frame corner.
        # This lens lights only a circle, so the frame corners hold no image at
        # all - and they sit at the largest radius, where the stretch is most
        # extreme. Deriving the zoom from them inflates the whole picture by
        # several times for no gain.
        rv = r_valid if r_valid is not None else 0.85
        scale = float(factor_for(np.array([rv]), k, model)[0])

    ys, xs = np.mgrid[0:oh, 0:ow]
    ocx, ocy = centre[0] * ow, centre[1] * oh
    os_ = np.hypot(ow, oh) / 2.0
    xu = (xs + 0.5 - ocx) / os_ * scale
    yu = (ys + 0.5 - ocy) / os_ * scale
    r_u = np.hypot(xu, yu)

    lut_u, lut_d = build_inverse_lut(k, float(r_u.max()), model=model)
    r_d = np.interp(r_u, lut_u, lut_d)
    with np.errstate(invalid="ignore", divide="ignore"):
        f = np.where(r_u > 1e-9, r_d / r_u, 1.0)

    sx = xu * f * s + cx - 0.5
    sy = yu * f * s + cy - 0.5

    x0 = np.floor(sx).astype(np.int64)
    y0 = np.floor(sy).astype(np.int64)
    fx = (sx - x0)[..., None]
    fy = (sy - y0)[..., None]
    valid = (x0 >= 0) & (x0 < w - 1) & (y0 >= 0) & (y0 < h - 1)
    x0c = np.clip(x0, 0, w - 2)
    y0c = np.clip(y0, 0, h - 2)

    a = img[y0c, x0c].astype(np.float32)
    b = img[y0c, x0c + 1].astype(np.float32)
    c = img[y0c + 1, x0c].astype(np.float32)
    d = img[y0c + 1, x0c + 1].astype(np.float32)
    top = a + (b - a) * fx
    bot = c + (d - c) * fx
    out = top + (bot - top) * fy
    out[~valid] = 0
    return out.astype(img.dtype), float(scale)
