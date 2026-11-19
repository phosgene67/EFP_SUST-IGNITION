"""Initial orientation accounts for finite-burn gravity; no continuing servo."""
from pathlib import Path
import unittest
import numpy as np
from controller_loader import load_namespace

NS = load_namespace((Path(__file__).resolve().parents[1]/'tests/score_sweeps/score55_rejected_prototypes_controller.py').read_text())


class InitialForceHeadingTests(unittest.TestCase):
    def test_known_transverse_drift_is_compensated_once(self):
        loc, target = np.array([9000., 0., 0.]), np.array([10300., 0., 0.])
        heading = NS['initial_force_heading'](20., loc, np.array([1., .8, 0.]),
            target, np.zeros(3), np.array([-.01, 0., 0.]), .175, 4., 0.)
        self.assertAlmostEqual(np.linalg.norm(heading), 1.)
        self.assertGreater(heading[0], .98)
        self.assertLess(heading[1], 0.)

    def test_zero_cross_drift_keeps_a_shallow_heading(self):
        loc, target = np.array([9000., 0., 0.]), np.array([10300., 0., 0.])
        heading = NS['initial_force_heading'](20., loc, np.array([1., 0., 0.]),
            target, np.zeros(3), np.array([-.01, 0., 0.]), .175, 4., 0.)
        self.assertGreater(heading[0], .999)
        self.assertTrue(np.isfinite(heading).all())

    def test_impossible_burn_screen_returns_direct_heading(self):
        heading = NS['initial_force_heading'](0., np.zeros(3), np.zeros(3),
            np.array([1e7, 0., 0.]), np.zeros(3), np.zeros(3), .025, 0., 0.)
        np.testing.assert_array_equal(heading, [1., 0., 0.])


if __name__ == '__main__':
    unittest.main()
