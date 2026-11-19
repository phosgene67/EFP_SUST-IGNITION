"""A distant fitted crossing can select the cheaper clear side in advance."""
from pathlib import Path
import unittest
import numpy as np
from controller_loader import load_namespace

NS = load_namespace((Path(__file__).resolve().parents[1]/'tests/score_sweeps/score55_rejected_prototypes_controller.py').read_text())


class LookaheadFuelTests(unittest.TestCase):
    def plan(self, enabled, confident):
        p = NS['ShootingNavigator'](launch=False)
        p.lookahead_fuel = enabled
        p.gravity_at = lambda *a: np.zeros(3)
        def scene(*args):
            c, v, r = np.array([[250., 10., 0.]]), np.zeros((1, 3)), np.array([16.])
            rates, accels = np.array([.25]), np.zeros((1, 3))
            p.latest_radar_count = 1
            p.radar_track_confidence = np.array([confident])
            p.radar_report_ages, p.radar_core_radii = np.zeros(1), np.array([11.9])
            p.radar_measured_diameters = np.array([12.])
            p.latest_scene, p.latest_accels, p.radar_forecast_rates = (c, v, r), accels, rates
            return c, v, r, rates, accels, False
        p.observe_scene = scene
        p.command(20., np.array([9000., 0., 0.]), np.array([5., 0., 0.]),
            np.array([10300., 0., 0.]), np.zeros(3), [])
        return p

    def test_fitted_distant_crossing_changes_azimuth_without_braking(self):
        old, new = self.plan(False, True), self.plan(True, True)
        self.assertGreater(old.terminal.inner_goal[1], 0.)
        self.assertLess(new.terminal.inner_goal[1], 0.)
        self.assertAlmostEqual(new.burn_target[0], old.burn_target[0])
        self.assertTrue(new.shooting_active)
        self.assertEqual(new.last_horizon, 20)

    def test_unfitted_tracks_do_not_change_the_decision(self):
        old, new = self.plan(False, False), self.plan(True, False)
        np.testing.assert_array_equal(old.burn_target, new.burn_target)
        self.assertEqual(old.plan_score, new.plan_score)


if __name__ == '__main__':
    unittest.main()
