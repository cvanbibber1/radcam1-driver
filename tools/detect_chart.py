#!/usr/bin/env python3
"""Locate the 18 DKK chart patches in a frame, tolerating rotation and distortion.

A rigid 6x3 grid over the frame only works if the chart is square to the lens
and fills it exactly. With a wide lens the chart bows outwards, and hand-held
it is never square, so the fixed grid lands between patches and every measured
colour is a blend of neighbours.

This instead finds the patches themselves - large, locally-uniform, similarly
coloured regions - assigns them (column, row) indices from their spacing, and
fits a quadratic surface mapping grid position to image position:

    x = a0 + a1*c + a2*r + a3*c^2 + a4*c*r + a5*r^2      (and likewise y)

The quadratic terms absorb barrel distortion and perspective. Patches that were
too dark or too washed out to detect are then interpolated from the fit, so a
missing black patch does not sink the calibration.

Importing this module gives `find_patches(image) -> 18 centres` in reading
order. Run it directly to see what it found and write an annotated preview.
"""

from __future__ import annotations

import itertools
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

COLS, ROWS = 6, 3
#: Row 0 of the DKK chart is the neutral wedge, white on the left through black
#: on the right. That is chart-specific knowledge, and it is the single most
#: useful constraint available for deciding which candidate grid is real.
NEUTRAL_ROW = 0
BLOCK = 24                 # coarse block size for the uniformity map


def _uniform_regions(a: np.ndarray, block: int = BLOCK,
                     max_std: float = 6.0, min_lum: float = 20.0,
                     min_blocks: int = 60):
    """Connected runs of flat, similarly-coloured blocks."""
    H, W, _ = a.shape
    h, w = H // block, W // block
    blk = a[:h * block, :w * block].reshape(h, block, w, block, 3)
    mean = blk.mean(axis=(1, 3))
    std = blk.std(axis=(1, 3)).mean(axis=2)
    lum = mean.mean(axis=2)

    ok = (std < max_std) & (lum > min_lum)
    seen = np.zeros_like(ok, dtype=bool)
    out = []
    for sy in range(h):
        for sx in range(w):
            if not ok[sy, sx] or seen[sy, sx]:
                continue
            stack = [(sy, sx)]
            seen[sy, sx] = True
            pts = []
            while stack:
                y, x = stack.pop()
                pts.append((y, x))
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ny, nx = y + dy, x + dx
                    if (0 <= ny < h and 0 <= nx < w and ok[ny, nx]
                            and not seen[ny, nx]
                            # Colour continuity stops adjacent patches merging.
                            and np.abs(mean[ny, nx] - mean[y, x]).max() < 14):
                        seen[ny, nx] = True
                        stack.append((ny, nx))
            if len(pts) >= min_blocks:
                ys = np.array([p[0] for p in pts], dtype=float)
                xs = np.array([p[1] for p in pts], dtype=float)
                out.append((xs.mean() * block + block / 2,
                            ys.mean() * block + block / 2, len(pts)))
    return out


def _cluster(values, expected, tol_frac=0.35):
    """Group 1-D values into `expected` clusters by their largest gaps."""
    v = np.sort(np.asarray(values, dtype=float))
    if len(v) <= expected:
        return list(v)
    gaps = np.diff(v)
    idx = np.argsort(gaps)[-(expected - 1):]
    bounds = np.sort(v[np.sort(idx)] + gaps[np.sort(idx)] / 2)
    groups = [[] for _ in range(expected)]
    for x in v:
        g = int(np.searchsorted(bounds, x))
        groups[min(g, expected - 1)].append(x)
    return [float(np.mean(g)) if g else float("nan") for g in groups]


def _fit_quadratic(cols, rows, vals):
    """Least-squares quadratic in (col, row)."""
    c = np.asarray(cols, dtype=float)
    r = np.asarray(rows, dtype=float)
    A = np.stack([np.ones_like(c), c, r, c * c, c * r, r * r], axis=1)
    coef, *_ = np.linalg.lstsq(A, np.asarray(vals, dtype=float), rcond=None)
    return coef


def _eval_quadratic(coef, c, r):
    return (coef[0] + coef[1] * c + coef[2] * r + coef[3] * c * c
            + coef[4] * c * r + coef[5] * r * r)


def _columns_for_row(xs, cols: int = COLS):
    """Assign column indices to the patches found on one row.

    Returns a list of candidate (score, pitch, offset, first_column), where

        column = round(x / pitch - offset) - first_column

    The window `first_column` is searched rather than anchored on the leftmost
    patch: when column 0 is the one patch too dark to detect, anchoring slides
    the whole row sideways by one.
    """
    xs = np.asarray(xs, dtype=float)
    if len(xs) < 3:
        return None
    span = float(xs.max() - xs.min())
    if span <= 0:
        return None
    cands = []
    for step in np.linspace(span / (cols + 1), span, 400):
        u = xs / step
        # Circular mean of the fractional parts is the origin lining the
        # patches up with integers best; a plain min() would be thrown by a
        # single outlier.
        ang = 2 * np.pi * (u - np.floor(u))
        off = np.arctan2(np.sin(ang).mean(), np.cos(ang).mean()) / (2 * np.pi)
        d = u - off
        c = np.round(d)
        fit = np.exp(-(np.abs(d - c) / 0.15) ** 2)
        for c0 in range(int(c.min()), int(c.max()) - cols + 2):
            inside = (c >= c0) & (c < c0 + cols)
            # Only patches inside a real cols-wide window may score; those
            # outside cost something, which stops a squashed pitch winning by
            # offering more integers to land on.
            score = float(fit[inside].sum()) - 0.75 * float((~inside).sum())
            cands.append((score, step, off, c0))
    if not cands:
        return None
    # Keep several options, not just the winner. When a stray background blob
    # sits on the row, "columns 0-5" and "columns 1-6" score almost the same,
    # and the row alone cannot break the tie - the caller does, by insisting
    # the three rows line up into one grid.
    cands.sort(key=lambda t: -t[0])
    out, seen = [], set()
    for sc, st, off, c0 in cands:
        key = (round(st / 8.0), c0)
        if key in seen:
            continue
        seen.add(key)
        out.append((sc, st, off, c0))
        if len(out) >= 10:
            break
    return out


def find_patches(image: Image.Image, debug: bool = False):
    """Return 18 (x, y) patch centres in reading order, or None."""
    a = np.asarray(image.convert("RGB")).astype(np.float32)
    regions = _uniform_regions(a)
    if len(regions) < 10:
        if debug:
            print(f"  only {len(regions)} uniform regions - chart not visible")
        return None

    # Chart patches are all the same physical size, so their areas cluster
    # tightly. Background clutter does not. Filtering on area first removes
    # most false candidates before any geometry is fitted.
    regions.sort(key=lambda t: -t[2])
    areas = np.array([r[2] for r in regions[:COLS * ROWS + 6]], dtype=float)
    med = float(np.median(areas))
    regions = [r for r in regions if 0.35 * med <= r[2] <= 2.5 * med]
    if debug:
        print(f"  {len(regions)} regions within 0.35-2.5x the median patch area")
    if len(regions) < 8:
        return None

    # Find the three rows by scanning candidate row pitches: the chart's rows
    # are evenly spaced, and picking the pitch that captures the most patches
    # rejects background that happens to sit at some other height.
    ys = np.array([r[1] for r in regions])
    best = None
    y_lo, y_hi = ys.min(), ys.max()
    for pitch in np.linspace(60, (y_hi - y_lo), 200):
        for y0 in np.linspace(y_lo, y_hi - 2 * pitch, 120):
            if y0 + 2 * pitch > y_hi + pitch * 0.5:
                continue
            rows_y = np.array([y0, y0 + pitch, y0 + 2 * pitch])
            d = np.abs(ys[:, None] - rows_y[None, :]).min(axis=1)
            hit = d < pitch * 0.3
            score = hit.sum() - 0.002 * d[hit].sum()
            if best is None or score > best[0]:
                best = (score, rows_y, hit)
    if best is None:
        return None
    _, row_centres, hit = best
    if debug:
        print(f"  rows at y = {row_centres.round(0)}, "
              f"{int(hit.sum())} patches on them")

    assigned = []
    for (x, y, n), on in zip(regions, hit):
        if not on:
            continue
        r = int(np.argmin(np.abs(row_centres - y)))
        assigned.append((x, y, r, n))

    xs_all = np.array([t[0] for t in assigned])
    rs_all = np.array([t[2] for t in assigned], dtype=float)
    if len(xs_all) < 8:
        if debug:
            print(f"  only {len(assigned)} patches near the rows")
        return None

    # Columns are not vertical, and their spacing is not constant. A chart
    # held up by hand sits rotated by ten or twenty degrees, which slides each
    # row sideways relative to the one above; it also leans away at the top, so
    # perspective makes the bottom row's columns wider than the top's - 392 px
    # of pitch against 480 px, measured on this rig. Taking the column index
    # straight from x puts whole rows one column out, and every sampled colour
    # then comes from a neighbouring patch.
    #
    # Rather than model the tilt, sidestep it: fit each row on its own. Whatever
    # the chart's orientation, the patches *within one row* are still evenly
    # spaced, so a row needs nothing but an origin and a pitch. Searching one
    # shared geometry across all three rows means searching enough parameters
    # to find well-scoring wrong answers, which is exactly what it did.
    options = {}
    for r in range(ROWS):
        xs_r = np.array([t[0] for t in assigned if t[2] == r])
        got = _columns_for_row(xs_r)
        if got:
            options[r] = got
    if not options:
        if debug:
            print("  no row could be fitted to a column grid")
        return None

    # Pick one option per row jointly. Each row on its own cannot tell
    # "columns 0-5" from "columns 1-6" when a background blob sits beside it,
    # but the three rows together can: they are one rigid chart, so the left
    # edge of each row has to lie on a straight line down the chart, and so
    # does the right edge. Score the combinations on that.
    rows_avail = sorted(options)
    per_row = {}
    if len(rows_avail) == 1:
        r = rows_avail[0]
        per_row[r] = options[r][0][1:]
    else:
        bestc = None
        hbox = max(int(np.median([o[0][1] for o in options.values()]) * 0.10), 6)
        for combo in itertools.product(*(options[r] for r in rows_avail)):
            edges_l, mids, pitches, total = [], [], [], 0.0
            for r, (sc, st, off, c0) in zip(rows_avail, combo):
                total += sc
                edges_l.append((off + c0) * st)
                mids.append((off + c0 + (COLS - 1) / 2.0) * st)
                pitches.append(st)
            rr = np.array(rows_avail, dtype=float)

            # The chart is rigid, so as you go down it the left edge and the
            # centre of each row must travel in a straight line, and the pitch
            # must change smoothly (perspective widens each row by about the
            # same amount). A window that is one column out breaks all three.
            #
            # Note this deliberately does *not* test the right-hand edge.
            # Pitch genuinely differs row to row, so left + 5*pitch is not
            # collinear even for a perfect fit, and penalising it punishes the
            # correct answer.
            def _nonlinearity(v):
                v = np.asarray(v, dtype=float)
                if len(rr) < 3:
                    return 0.0
                A = np.stack([np.ones_like(rr), rr], axis=1)
                coef, *_ = np.linalg.lstsq(A, v, rcond=None)
                return float(np.abs(v - A @ coef).max())

            med = float(np.median(pitches))
            pen = (6.0 * _nonlinearity(edges_l) / max(med, 1.0)
                   + 6.0 * _nonlinearity(mids) / max(med, 1.0)
                   + 2.0 * _nonlinearity(pitches) / max(med, 1.0))
            score = total - pen

            # Geometry alone cannot separate "columns 0-5" from "columns
            # 1-6" when a background blob sits on the row: both windows hold
            # six patches and score alike. The picture can separate them. Row 0
            # of this chart is a grey wedge running white to black, so the real
            # window is the one whose top row actually darkens left to right.
            #
            # Score it on the patches that were genuinely detected, at their
            # own measured positions - a predicted position needs a y, and a
            # tilted row has a different y in every column.
            if NEUTRAL_ROW in rows_avail:
                k = rows_avail.index(NEUTRAL_ROW)
                _, st, off, c0 = combo[k]
                seen = {}
                for (xv, yv, rv, _n) in assigned:
                    if rv != NEUTRAL_ROW:
                        continue
                    cc = int(round(xv / st - off) - c0)
                    if 0 <= cc < COLS and cc not in seen:
                        xi, yi = int(xv), int(yv)
                        if (hbox <= xi < a.shape[1] - hbox
                                and hbox <= yi < a.shape[0] - hbox):
                            seen[cc] = float(a[yi - hbox:yi + hbox,
                                               xi - hbox:xi + hbox].mean())
                order = [seen[c] for c in sorted(seen)]
                if len(order) >= 3:
                    drops = sum(1 for i in range(len(order) - 1)
                                if order[i + 1] < order[i])
                    score += 2.0 * drops - 1.0 * (len(order) - 1 - drops)

            if bestc is None or score > bestc[0]:
                bestc = (score, combo)
        for r, (sc, st, off, c0) in zip(rows_avail, bestc[1]):
            per_row[r] = (st, off, c0)
    if debug:
        for r in sorted(per_row):
            print(f"  row {r}: pitch {per_row[r][0]:.0f} px, "
                  f"{sum(1 for t in assigned if t[2] == r)} patches")

    # A row too sparse to fit on its own borrows the neighbouring pitch and is
    # aligned by its own patches.
    med_step = float(np.median([v[0] for v in per_row.values()]))

    cols, rws, px, py, cell_n = [], [], [], [], []
    for (x, y, r, n) in assigned:
        if r in per_row:
            step_r, off_r, c0_r = per_row[r]
        else:
            step_r, off_r, c0_r = med_step, 0.0, 0
        c = int(round(x / step_r - off_r) - c0_r)
        cols.append(c); rws.append(r); px.append(x); py.append(y)
        cell_n.append(n)

    # One patch per cell: where two land in the same cell keep the bigger,
    # which is the one more likely to be a real patch than a background blob.
    cell = {}
    for c, r, x, y, n in zip(cols, rws, px, py, cell_n):
        if not 0 <= c < COLS:
            continue
        if (c, r) not in cell or n > cell[(c, r)][2]:
            cell[(c, r)] = (x, y, n)

    cols = [c for (c, r) in cell]
    rws = [r for (c, r) in cell]
    px = [v[0] for v in cell.values()]
    py = [v[1] for v in cell.values()]

    if len(set(zip(cols, rws))) < 8:
        if debug:
            print("  too few distinct grid cells to fit")
        return None

    fx = _fit_quadratic(cols, rws, px)
    fy = _fit_quadratic(cols, rws, py)

    resid = np.hypot(
        np.array(px) - _eval_quadratic(fx, np.array(cols), np.array(rws)),
        np.array(py) - _eval_quadratic(fy, np.array(cols), np.array(rws)))
    if debug:
        print(f"  {len(px)} patches located, fit residual "
              f"mean {resid.mean():.0f} px, max {resid.max():.0f} px")

    centres = []
    for r in range(ROWS):
        for c in range(COLS):
            centres.append((float(_eval_quadratic(fx, c, r)),
                            float(_eval_quadratic(fy, c, r))))
    return _refine(a, centres, med_step, debug=debug)


def _refine(a: np.ndarray, centres, pitch: float, debug: bool = False):
    """Nudge each centre onto the flattest spot near it.

    The quadratic is fitted from the patches it could detect, so it is honest
    in the middle of the chart and extrapolating at the corners - which is
    where it drifts, typically by half a patch. A centre that lands on a black
    border averages two colours together and is worse than useless, and the
    outer columns are exactly where the saturated patches live.

    Patches are uniform and borders are not, so search a small window around
    each fitted centre and keep whichever position has the least variation in
    its neighbourhood. The distance penalty stops a centre sliding all the way
    into its neighbour when the fit was already right.
    """
    H, W, _ = a.shape
    half = max(int(pitch * 0.13), 8)
    reach = max(int(pitch * 0.30), 4)
    stepr = max(reach // 7, 1)
    out, moved = [], 0.0
    for (x, y) in centres:
        best = None
        for dy in range(-reach, reach + 1, stepr):
            for dx in range(-reach, reach + 1, stepr):
                cx, cy = int(x + dx), int(y + dy)
                if not (half <= cx < W - half and half <= cy < H - half):
                    continue
                box = a[cy - half:cy + half, cx - half:cx + half]
                score = float(box.std(axis=(0, 1)).mean())
                score += 8.0 * math.hypot(dx, dy) / reach
                if best is None or score < best[0]:
                    best = (score, float(cx), float(cy))
        if best is None:
            out.append((x, y))
            continue
        out.append((best[1], best[2]))
        moved = max(moved, math.hypot(best[1] - x, best[2] - y))
    if debug:
        print(f"  refined onto flat regions, largest move {moved:.0f} px")
    return out


def annotate(image: Image.Image, centres, radius: int = 40) -> Image.Image:
    im = image.convert("RGB").copy()
    d = ImageDraw.Draw(im)
    for i, (x, y) in enumerate(centres):
        d.ellipse([x - radius, y - radius, x + radius, y + radius],
                  outline=(255, 255, 0), width=5)
        d.text((x - radius, y - radius - 30), str(i), fill=(255, 255, 0))
    return im


def main() -> int:
    src = sys.argv[1] if len(sys.argv) > 1 else "captures/chart3.jpg"
    im = Image.open(src)
    print(f"{src}: {im.size}")
    centres = find_patches(im, debug=True)
    if centres is None:
        print("  chart not found")
        return 1

    a = np.asarray(im.convert("RGB")).astype(np.float32)
    print("\n  idx   centre        sampled RGB (processed image)")
    for i, (x, y) in enumerate(centres):
        xi, yi = int(x), int(y)
        patch = a[max(yi - 25, 0):yi + 25, max(xi - 25, 0):xi + 25]
        m = patch.mean(axis=(0, 1)) if patch.size else np.zeros(3)
        print(f"   {i:2d}  ({x:6.0f},{y:6.0f})  "
              f"({m[0]:3.0f},{m[1]:3.0f},{m[2]:3.0f})")

    out = Path(src).with_name("chart-detected.jpg")
    annotate(im, centres).save(out, quality=88)
    print(f"\n  annotated preview: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
