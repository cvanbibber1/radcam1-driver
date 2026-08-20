"""Shading, distortion and colour solvers, checked against known answers.

Every one of these fits a model to data whose true answer is constructed, so a
regression shows up as a number that stops matching rather than as an image
that looks slightly off. That matters because all three failed in exactly that
quiet way during development: a shading table that doubled the vignette, a
distortion fit that scored well while folding the corners, and a colour matrix
fitted to a channel that carried no colour.
"""

import unittest

import numpy as np

from tests.support import ROOT                                   # noqa: F401
from radcam import distortion as D
from radcam import shading as S
from radcam.ccm import (calibrate_from_patches, chart_targets_linear_srgb,
                        solve_ccm_shading_invariant, white_balance_gains)


class TestShading(unittest.TestCase):
    TRUE = {"r": [-0.55, 0.10, -0.02],
            "g": [-0.40, 0.05, -0.01],
            "b": [-0.30, 0.03, -0.01]}
    CENTRE = (0.52, 0.48)

    def planes(self, scale=1000.0):
        return {k: S.response(v, (36, 48), self.CENTRE) * scale
                for k, v in self.TRUE.items()}

    def test_recovers_centre_and_coefficients(self):
        centre, coeff, resid = S.fit(self.planes())
        self.assertAlmostEqual(centre[0], self.CENTRE[0], places=2)
        self.assertAlmostEqual(centre[1], self.CENTRE[1], places=2)
        for ch, want in self.TRUE.items():
            np.testing.assert_allclose(coeff[ch], want, atol=2e-3)
            self.assertLess(resid[ch], 1e-6)

    def test_exposure_cancels(self):
        """Absolute brightness must not change the model."""
        a = S.fit(self.planes(scale=1000.0))[1]
        b = S.fit(self.planes(scale=17.0))[1]
        for ch in a:
            np.testing.assert_allclose(a[ch], b[ch], atol=1e-6)

    def test_tables_correct_rather_than_double_the_vignette(self):
        """The sign convention is the easy thing to get backwards."""
        centre, coeff, _ = S.fit(self.planes())
        t = S.tables({"centre": list(centre), "coeff": coeff})
        for name in ("luminance_lut", "calibrations_Cr", "calibrations_Cb"):
            self.assertEqual(len(t[name]), S.TABLE_SIZE ** 2)
            self.assertAlmostEqual(min(t[name]), 1.0, places=5)
        lum = np.array(t["luminance_lut"]).reshape(S.TABLE_SIZE, S.TABLE_SIZE)
        # Green falls off toward the edge, so the gain must *rise* there.
        self.assertGreater(lum[0, 0], lum[S.TABLE_SIZE // 2, S.TABLE_SIZE // 2])
        # Red falls off faster than green here, so Cr must exceed 1 at the edge.
        cr = np.array(t["calibrations_Cr"]).reshape(S.TABLE_SIZE, S.TABLE_SIZE)
        self.assertGreater(cr[0, 0], 1.0)

    def test_applying_the_model_flattens_the_field(self):
        centre, coeff, _ = S.fit(self.planes())
        t = S.tables({"centre": list(centre), "coeff": coeff})
        n = S.TABLE_SIZE
        obs = S.response(self.TRUE["g"], (n, n), self.CENTRE)
        corrected = obs * np.array(t["luminance_lut"]).reshape(n, n)
        # Not exact: the stored table is rounded to 5 decimal places to keep
        # the tuning file readable, which is worth ~1e-5 of residual ripple.
        self.assertLess(corrected.max() / corrected.min() - 1.0, 1e-4)


class TestDistortion(unittest.TestCase):
    SHAPE = (768, 1024)
    CENTRE = (0.52, 0.48)

    def _chains(self, k_true, model=D.MODEL_POLY, n=14, seed=0):
        """Straight lines pushed through the inverse model, i.e. what a camera
        with this distortion would actually have recorded."""
        h, w = self.SHAPE
        s = np.hypot(w, h) / 2.0
        cx, cy = self.CENTRE[0] * w, self.CENTRE[1] * h
        lut_u, lut_d = D.build_inverse_lut(k_true, 1.2, model=model)
        rng = np.random.default_rng(seed)
        out = []
        for _ in range(n):
            a = rng.uniform(0, np.pi)
            off = rng.uniform(-0.45, 0.45)
            t = np.linspace(-0.55, 0.55, 120)
            xy = np.stack([t * np.cos(a) - off * np.sin(a),
                           t * np.sin(a) + off * np.cos(a)], axis=1)
            xy = xy[np.hypot(xy[:, 0], xy[:, 1]) < 0.75]
            if len(xy) < 30:
                continue
            r_u = np.hypot(xy[:, 0], xy[:, 1])
            r_d = np.interp(r_u, lut_u, lut_d)
            f = np.where(r_u > 1e-9, r_d / np.maximum(r_u, 1e-9), 1.0)
            out.append(np.stack([xy[:, 0] * f * s + cx,
                                 xy[:, 1] * f * s + cy], axis=1))
        return out

    def test_recovers_polynomial_coefficients(self):
        k_true = np.array([0.35, 0.12])
        centre, k, score, base = D.fit(self._chains(k_true), self.SHAPE,
                                       centre0=(0.5, 0.5), centre_range=0.15)
        np.testing.assert_allclose(k, k_true, atol=0.02)
        self.assertAlmostEqual(centre[0], self.CENTRE[0], places=2)
        self.assertLess(score, base / 50)

    def test_survives_curved_and_kinked_liars(self):
        """A rounded object and an L-corner, both longer than the real edges."""
        th = np.linspace(0, 1.1, 500)
        arc = np.stack([300 + 250 * np.cos(th), 400 + 250 * np.sin(th)], axis=1)
        corner = np.concatenate([
            np.stack([np.linspace(100, 400, 250), np.full(250, 600.0)], axis=1),
            np.stack([np.full(250, 400.0), np.linspace(600, 350, 250)], axis=1)])
        k_true = np.array([0.35, 0.12])
        chains = self._chains(k_true) + [arc, corner]
        _, k, _, _ = D.fit(chains, self.SHAPE, centre0=(0.5, 0.5),
                           centre_range=0.15)
        np.testing.assert_allclose(k, k_true, atol=0.03)

    def test_fisheye_model_recovers_f(self):
        f_true = [0.85]
        chains = self._chains(f_true, model=D.MODEL_FISHEYE)
        _, k, score, base = D.fit(chains, self.SHAPE, centre0=(0.5, 0.5),
                                  centre_range=0.15, model=D.MODEL_FISHEYE)
        self.assertAlmostEqual(k[0], f_true[0], places=2)
        self.assertLess(score, base / 10)

    def test_inverse_lut_is_strictly_monotonic(self):
        """A fold would silently mirror the corners when rendering."""
        for k, model in (([0.35, 0.12], D.MODEL_POLY),
                         ([-0.6, 3.0], D.MODEL_POLY),
                         ([0.6], D.MODEL_FISHEYE)):
            lut_u, _ = D.build_inverse_lut(k, 1.0, model=model)
            self.assertTrue(np.all(np.diff(lut_u) > 0), f"{model} {k}")

    def test_undistort_image_restores_a_straight_edge(self):
        k_true = [0.35, 0.12]
        h, w = 240, 320
        img = np.zeros((h, w, 3), np.uint8)
        # Draw a straight vertical line, then distort the picture, then undo it.
        img[:, w // 2 - 1:w // 2 + 1] = 255
        lut_u, lut_d = D.build_inverse_lut(k_true, 1.4)
        ys, xs = np.mgrid[0:h, 0:w]
        s = np.hypot(w, h) / 2.0
        xn, yn = (xs - w / 2) / s, (ys - h / 2) / s
        r = np.hypot(xn, yn)
        f = np.where(r > 1e-9, np.interp(r, lut_d, lut_u) / np.maximum(r, 1e-9), 1.0)
        sx = np.clip((xn * f * s + w / 2).astype(int), 0, w - 1)
        sy = np.clip((yn * f * s + h / 2).astype(int), 0, h - 1)
        bent = img[sy, sx]
        fixed, _ = D.undistort_image(bent, (0.5, 0.5), k_true, scale=1.0)
        rows = [np.flatnonzero(fixed[y, :, 0] > 60) for y in range(40, h - 40, 20)]
        centres = [r.mean() for r in rows if len(r)]
        self.assertGreater(len(centres), 5)
        # A straight line stays at one x. Before correcting it would not.
        self.assertLess(float(np.std(centres)), 2.0)


class TestColour(unittest.TestCase):
    def _synthetic(self, ccm_true, gains, offset):
        """Camera RGB that a sensor with this matrix, gains and glare would give."""
        targets = chart_targets_linear_srgb()
        wb = targets @ np.linalg.inv(ccm_true).T
        cam = wb / np.array(gains)
        return np.maximum(cam, 0) * 1000.0 + np.array(offset)

    def test_white_balance_from_neutrals(self):
        cam = self._synthetic(np.eye(3), [1.6, 1.0, 1.5], [0, 0, 0])
        (rg, bg), (r_over_g, b_over_g) = white_balance_gains(cam)
        self.assertAlmostEqual(rg, 1.6, places=3)
        self.assertAlmostEqual(bg, 1.5, places=3)

    def test_shading_invariant_solver_ignores_per_patch_brightness(self):
        ccm_true = np.array([[1.6, -0.5, -0.1],
                             [-0.3, 1.6, -0.3],
                             [-0.1, -0.5, 1.6]])
        targets = chart_targets_linear_srgb()
        wb = targets @ np.linalg.inv(ccm_true).T
        # Vignette each patch by a different amount, as a real chart would be.
        rng = np.random.default_rng(1)
        wb = wb * rng.uniform(0.55, 1.0, size=(len(wb), 1))
        w = np.ones(len(wb))
        w[5] = 0.0
        M = solve_ccm_shading_invariant(np.maximum(wb, 1e-9), targets, weights=w)
        np.testing.assert_allclose(M, ccm_true, atol=0.02)

    def test_rows_sum_to_one_so_neutrals_stay_neutral(self):
        targets = chart_targets_linear_srgb()
        M = solve_ccm_shading_invariant(np.maximum(targets, 1e-9), targets)
        np.testing.assert_allclose(M.sum(axis=1), [1, 1, 1], atol=1e-9)

    def test_joint_solve_recovers_glare_gains_and_matrix(self):
        ccm_true = np.array([[1.7, -0.6, -0.1],
                             [-0.35, 1.7, -0.35],
                             [-0.15, -0.55, 1.7]])
        gains = [1.6, 1.0, 1.5]
        offset = [140.0, 160.0, 90.0]
        cam = self._synthetic(ccm_true, gains, offset)
        sol = calibrate_from_patches(cam)
        np.testing.assert_allclose(sol["offset"], offset, rtol=0.25, atol=30)
        self.assertAlmostEqual(sol["gains"][0], gains[0], delta=0.08)
        self.assertAlmostEqual(sol["gains"][1], gains[2], delta=0.08)
        self.assertLess(float(np.average(sol["delta_e"],
                                         weights=(np.arange(18) != 5))), 3.0)

    def test_glare_free_capture_does_not_crash_the_solver(self):
        """Regression: a capture with several patches at zero used to raise.

        The glare search rejects any offset that floors more than one patch,
        and its fallback was subject to the same rule - so a synthetic or very
        dark capture, where the black patch really is zero, left the search
        with no starting point at all and it dereferenced None.
        """
        cam = self._synthetic(np.eye(3), [1.0, 1.0, 1.0], [0, 0, 0])
        sol = calibrate_from_patches(cam)
        self.assertIsNotNone(sol["ccm"])
        np.testing.assert_allclose(sol["offset"], [0, 0, 0], atol=1e-6)

    def test_glare_makes_the_uncorrected_error_worse(self):
        """Sanity on the fixture itself: without glare the fit must be better."""
        ccm_true = np.eye(3)
        clean = calibrate_from_patches(
            self._synthetic(ccm_true, [1.0, 1.0, 1.0], [0, 0, 0]))
        dirty = calibrate_from_patches(
            self._synthetic(ccm_true, [1.0, 1.0, 1.0], [400, 400, 400]))
        self.assertLess(float(clean["delta_e_identity"].mean()),
                        float(dirty["delta_e_identity"].mean()))


if __name__ == "__main__":
    unittest.main()
