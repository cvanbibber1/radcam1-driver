#!/usr/bin/env python3
"""Fit lens distortion from straight edges in an ordinary photo, and undo it.

No chart is needed. Point the camera at a scene containing things that really
are straight - door frames, ceiling panel joints, a desk edge, the border of
the colour chart - and this finds them, fits the radial model that straightens
them, and stores it on the camera's EEPROM.

    python3 tools/calibrate-distortion.py                        # measure
    python3 tools/calibrate-distortion.py --store                # + EEPROM
    python3 tools/calibrate-distortion.py --from-capture f.jpg
    python3 tools/calibrate-distortion.py --undistort in.jpg out.jpg

The fit is a plumb-line calibration: see radcam/distortion.py for the model and
for why straightness is measured relative to each edge's own length.

Edges are found with a Sobel gradient, thinned to ridges, linked into chains,
and then filtered hard. Most of the work here is *rejecting* edges - a chain
that is genuinely curved in the world, or that is really two edges meeting at a
corner, will drag the fit toward nonsense if it is allowed to vote.
"""

import argparse
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from radcam import distortion                                   # noqa: E402
from radcam.eeprom import CameraEEPROM                          # noqa: E402

#: Work at this width. Distortion is a smooth global effect, so a downsampled
#: copy locates edges to well within the precision the fit needs, and keeps the
#: chain linking fast in pure Python.
WORK_W = 1024


def sobel(g: np.ndarray):
    gx = np.zeros_like(g)
    gy = np.zeros_like(g)
    gx[1:-1, 1:-1] = (g[:-2, 2:] + 2 * g[1:-1, 2:] + g[2:, 2:]
                      - g[:-2, :-2] - 2 * g[1:-1, :-2] - g[2:, :-2])
    gy[1:-1, 1:-1] = (g[2:, :-2] + 2 * g[2:, 1:-1] + g[2:, 2:]
                      - g[:-2, :-2] - 2 * g[:-2, 1:-1] - g[:-2, 2:])
    return gx, gy


def edge_map(g: np.ndarray, keep_frac: float = 0.06):
    """Thinned edge mask: strong gradients that are ridges across the gradient."""
    gx, gy = sobel(g)
    mag = np.hypot(gx, gy)
    thr = float(np.percentile(mag, 100 * (1 - keep_frac)))
    strong = mag >= thr

    # Non-maximum suppression, quantised to four directions. Without it every
    # edge is several pixels wide and the chains fatten into blobs that no
    # longer behave like curves.
    ang = np.rad2deg(np.arctan2(gy, gx)) % 180
    m = np.pad(mag, 1, mode="edge")
    nms = np.zeros_like(strong)
    sel = [((ang < 22.5) | (ang >= 157.5), (0, 1)),
           ((ang >= 22.5) & (ang < 67.5), (1, 1)),
           ((ang >= 67.5) & (ang < 112.5), (1, 0)),
           ((ang >= 112.5) & (ang < 157.5), (1, -1))]
    H, W = mag.shape
    for mask, (dy, dx) in sel:
        a = m[1 + dy:1 + dy + H, 1 + dx:1 + dx + W]
        b = m[1 - dy:1 - dy + H, 1 - dx:1 - dx + W]
        nms |= mask & strong & (mag >= a) & (mag >= b)
    return nms


def chains(mask: np.ndarray, min_len: int = 40):
    """Link edge pixels into connected chains."""
    H, W = mask.shape
    seen = np.zeros_like(mask)
    out = []
    idx = np.argwhere(mask)
    for sy, sx in idx:
        if seen[sy, sx]:
            continue
        stack = [(sy, sx)]
        seen[sy, sx] = True
        pts = []
        while stack:
            y, x = stack.pop()
            pts.append((x, y))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    ny, nx = y + dy, x + dx
                    if (0 <= ny < H and 0 <= nx < W and mask[ny, nx]
                            and not seen[ny, nx]):
                        seen[ny, nx] = True
                        stack.append((ny, nx))
        if len(pts) >= min_len:
            out.append(np.array(pts, dtype=float))
    return out


def image_circle(g: np.ndarray):
    """Centre and radius of the illuminated disc this fisheye lays on the sensor.

    The lens does not fill the frame; it projects a bright circle with dark
    corners. Two things come from finding it, and both matter:

    - Its boundary is the longest, strongest edge in every frame, and it is a
      *circle*. No radial model can straighten it, so allowing it to vote drives
      the fit into its bounds - which is exactly what happened before this
      existed.
    - Its centre is where the optical axis meets the sensor, which is the
      distortion centre. Measuring it directly beats fitting it: the centre and
      k1 trade off against each other, so a free centre gives the search a flat
      direction to wander along.

    Returns (cx, cy) as fractions of the frame, and the radius in pixels.
    """
    h, w = g.shape
    bright = g > 0.25 * float(np.percentile(g, 90))
    if bright.sum() < 0.05 * bright.size:
        return 0.5, 0.5, float(np.hypot(w, h) / 2)
    ys, xs = np.nonzero(bright)
    cx, cy = float(xs.mean()), float(ys.mean())
    # Radius from a radial brightness profile about that centre. An
    # equivalent-area radius would underestimate badly here, because the disc
    # runs off the top and bottom of the frame and the missing area is read as
    # a smaller circle.
    yy, xx = np.mgrid[0:h, 0:w]
    r = np.hypot(xx - cx, yy - cy)
    bins = np.linspace(0, float(r.max()), 80)
    idx = np.clip(np.digitize(r.ravel(), bins) - 1, 0, len(bins) - 2)
    tot = np.bincount(idx, weights=g.ravel(), minlength=len(bins) - 1)
    cnt = np.maximum(np.bincount(idx, minlength=len(bins) - 1), 1)
    prof = tot / cnt
    peak = float(np.percentile(prof, 90))
    inside = np.where(prof > 0.3 * peak)[0]
    rad = float(bins[inside[-1] + 1]) if len(inside) else float(r.max())
    return cx / w, cy / h, rad


def filter_chains(cs, shape, max_bow: float = 0.30, min_aspect: float = 3.0,
                  min_span: float = 50.0, circle=None, max_kink: float = 0.025,
                  stats: dict | None = None):
    """Keep chains that plausibly are straight lines seen through a lens.

    Three things get thrown out, and each of them would corrupt the fit in a
    different way:

    - **Blobs and texture.** A chain has to be long and thin to be an edge at
      all, so anything below `min_aspect` goes.
    - **Corners and junctions.** Linking is greedy, so an L-shaped join arrives
      as one chain, and no distortion can fix it. Rejecting on *bow* alone is
      wrong here, though: a genuinely straight line seen through this lens is
      strongly bowed, and that bow is the entire signal. What separates the two
      is smoothness - a distorted line is a gentle arc that a parabola fits
      almost exactly, while a corner leaves a large residual against one. So
      the test is the residual to a quadratic, not the size of the bend.
    - **Short edges.** They carry almost no curvature signal but are numerous,
      so they would outvote the long edges that actually constrain the model.
    """
    h, w = shape
    if circle is None:
        cx0, cy0, circle_r = w / 2.0, h / 2.0, None
    else:
        cx0, cy0, circle_r = circle[0] * w, circle[1] * h, circle[2]
    kept = []
    rej = stats if stats is not None else {}
    for key in ("vignette", "concentric", "thin", "short", "bow", "kink"):
        rej.setdefault(key, 0)
    for c in cs:
        if circle_r is not None:
            rad = np.hypot(c[:, 0] - cx0, c[:, 1] - cy0)
            # Anything touching the vignette boundary is discarded outright.
            if rad.max() > 0.92 * circle_r:
                rej["vignette"] += 1
                continue
            # ...and so is anything lying at a near-constant radius, which is
            # an arc concentric with the lens rather than a straight edge.
            if np.ptp(rad) < 0.08 * max(rad.mean(), 1e-9):
                rej["concentric"] += 1
                continue
        mean = c.mean(axis=0)
        q = c - mean
        _, sv, vt = np.linalg.svd(q, full_matrices=False)
        if sv[1] < 1e-9:
            continue
        aspect = sv[0] / max(sv[1], 1e-9)
        span = float(np.ptp(q @ vt[0]))
        if aspect < min_aspect:
            rej["thin"] += 1
            continue
        if span < min_span:
            rej["short"] += 1
            continue
        perp = q @ vt[1]
        along = q @ vt[0]
        bow = float(np.abs(perp).max() / max(span, 1e-9))
        if bow > max_bow:
            rej["bow"] += 1
            continue
        # Smoothness: fit a parabola across the chain and see what is left.
        A = np.stack([np.ones_like(along), along, along ** 2], axis=1)
        coef, *_ = np.linalg.lstsq(A, perp, rcond=None)
        kink = float(np.sqrt(np.mean((perp - A @ coef) ** 2)) / max(span, 1e-9))
        if kink > max_kink:
            rej["kink"] += 1
            continue
        kept.append(c)
    return kept


def load_gray(path: Path):
    im = Image.open(path).convert("L")
    full = im.size
    scale = WORK_W / im.width
    small = im.resize((WORK_W, max(1, int(im.height * scale))))
    return np.asarray(small).astype(np.float32), small.size[::-1], full


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--from-capture", type=Path, default=None, metavar="JPG")
    ap.add_argument("--undistort", nargs=2, metavar=("IN", "OUT"), default=None,
                    help="correct an image with the stored model")
    ap.add_argument("--scale", type=float, default=None,
                    help="zoom for the corrected image; default keeps the "
                         "horizontal field of view")
    ap.add_argument("--bus", type=int, default=4)
    ap.add_argument("--store", action="store_true")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--terms", type=int, default=2, choices=(1, 2, 3))
    ap.add_argument("--model", default=distortion.MODEL_FISHEYE,
                    choices=(distortion.MODEL_FISHEYE, distortion.MODEL_POLY),
                    help="fisheye_equidistant (one physical parameter, right "
                         "for this lens) or radial_poly")
    ap.add_argument("--debug-edges", type=Path, default=None,
                    help="write a preview of the chains that were used")
    args = ap.parse_args()

    ee = CameraEEPROM(bus=args.bus)
    if args.show:
        rec = ee.load() or {}
        print(json.dumps(rec.get("distortion", "no distortion stored"),
                         indent=2))
        return 0

    if args.undistort:
        src, dst = Path(args.undistort[0]), Path(args.undistort[1])
        rec = (ee.load() or {}).get("distortion")
        if not rec:
            print("no distortion model on the EEPROM; run without --undistort")
            return 1
        img = np.asarray(Image.open(src).convert("RGB"))
        out, used = distortion.undistort_image(
            img, tuple(rec["centre"]), rec["k"], scale=args.scale,
            model=rec.get("model", distortion.MODEL_POLY),
            r_valid=rec.get("r_valid"))
        Image.fromarray(out).save(dst, quality=92)
        print(f"{src} -> {dst}  (centre {rec['centre']}, k {rec['k']}, "
              f"scale {used:.3f})")
        return 0

    if args.from_capture:
        path = args.from_capture
    else:
        path = ROOT / "captures" / "distortion.jpg"
        path.parent.mkdir(exist_ok=True)
        print("capturing...")
        subprocess.run(["rpicam-still", "-n", "-t", "3000", "--width", "4096",
                        "--height", "3072", "-o", str(path)],
                       check=True, capture_output=True, timeout=180)

    g, shape, full = load_gray(path)
    print(f"{path} ({full[0]}x{full[1]}), working at {shape[1]}x{shape[0]}")

    ccx, ccy, circle_r = image_circle(g)
    print(f"  illuminated disc: centre ({ccx:.3f}, {ccy:.3f}), radius "
          f"{circle_r:.0f} px of {np.hypot(shape[1], shape[0]) / 2:.0f} px "
          f"half-diagonal")
    mask = edge_map(g)
    cs = chains(mask)
    rej = {}
    kept = filter_chains(cs, shape, circle=(ccx, ccy, circle_r), stats=rej)
    print(f"  {int(mask.sum())} edge pixels -> {len(cs)} chains -> "
          f"{len(kept)} usable straight-edge candidates")
    print("  rejected: " + ", ".join(f"{k} {v}" for k, v in rej.items() if v))
    if len(kept) < 6:
        print("\n  too few straight edges to fit. Point the camera at a scene")
        print("  with clear straight lines - door frames, panel joints, a")
        print("  table edge - spread across the frame, not just the middle.")
        return 1

    if args.debug_edges:
        prev = Image.open(path).convert("RGB").resize((shape[1], shape[0]))
        a = np.asarray(prev).copy()
        for i, c in enumerate(kept):
            col = [(255, 80, 80), (80, 255, 80), (120, 160, 255),
                   (255, 220, 60)][i % 4]
            xi = np.clip(c[:, 0].astype(int), 0, shape[1] - 1)
            yi = np.clip(c[:, 1].astype(int), 0, shape[0] - 1)
            a[yi, xi] = col
        Image.fromarray(a).save(args.debug_edges, quality=90)
        print(f"  edge preview: {args.debug_edges}")

    centre, k, score, before = distortion.fit(
        kept, shape, n_terms=args.terms, centre0=(ccx, ccy),
        centre_range=0.04, model=args.model)
    print(f"\n  distortion centre: ({centre[0]:.4f}, {centre[1]:.4f}) "
          f"of the frame")
    if args.model == distortion.MODEL_FISHEYE:
        print(f"  model: {args.model}, f={k[0]:.5f}")
    else:
        print("  coefficients: " + ", ".join(f"k{i+1}={v:+.5f}"
                                             for i, v in enumerate(k)))
    print(f"\n  mean edge straightness  {before:.5f} -> {score:.5f}  "
          f"({100 * (1 - score / max(before, 1e-9)):.0f}% straighter)")

    r_valid = float(circle_r / (np.hypot(shape[1], shape[0]) / 2))
    corner = distortion.factor_for(np.array([r_valid]), k, args.model)[0]
    print(f"  a point at the edge of the illuminated disc moves outward by "
          f"{100 * (corner - 1):.0f}%")
    if args.model == distortion.MODEL_FISHEYE:
        # f = r / theta, so the disc edge is at theta = r_valid / f. This is a
        # direct measurement of the lens field of view - the number
        # docs/calibration-box.md needs before the fixture can be dimensioned.
        import math
        theta = r_valid / float(k[0])
        print(f"  implied field of view: {math.degrees(2 * theta):.0f} deg "
              f"across the illuminated disc")

    model = {
        "model": args.model,
        "centre": [round(float(centre[0]), 5), round(float(centre[1]), 5)],
        "k": [round(float(v), 6) for v in k],
        "straightness": {"before": round(before, 5), "after": round(score, 5)},
        "edges_used": len(kept),
        # Fraction of the half-diagonal that actually carries image. Needed to
        # scale a corrected frame sensibly, and to say where the model stops
        # being constrained by anything.
        "r_valid": round(float(circle_r / (np.hypot(shape[1], shape[0]) / 2)), 4),
        "measured_utc": datetime.now(timezone.utc).isoformat(),
    }
    if not args.store:
        print("\n--store to write it to the camera EEPROM")
        print(json.dumps(model, indent=2))
        return 0

    ee.update("distortion", model)
    print(f"\nstored on the camera EEPROM (i2c-{args.bus}, 0x50)")
    print("correct an image with:")
    print("  python3 tools/calibrate-distortion.py --undistort in.jpg out.jpg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
