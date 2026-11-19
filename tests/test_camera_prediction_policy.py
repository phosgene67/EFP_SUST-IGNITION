"""Correlated depth predictions keep geometry and preserve local safeguards."""
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from controller_loader import load_namespace


NS = load_namespace((Path(__file__).resolve().parents[1]/'Controller.py').read_text())


class CameraPredictionPolicyTests(unittest.TestCase):
    def planner(self):
        p = NS['ShootingNavigator'](launch=False)
        def obstacles(*args):
            p.radar_forecast_rates = np.array([.25])
            return np.array([[100., 0., 0.]]), np.array([[0., .3, 0.]]), np.array([10.])
        p.obstacles = obstacles
        return p

    def test_matching_camera_bound_shares_velocity_uncertainty_without_shrinking(self):
        p = self.planner()
        p.matched_camera_prediction = True
        camera = SimpleNamespace(observes=lambda *a: False,
            obstacles=lambda *a: (np.array([[100., 2., 0.]]), np.array([[0., .3, 0.]]), np.array([12.])))
        c, v, r, rates, _, _ = p.observe_scene(10., np.array([9000., 0., 0.]), np.zeros(3),
            np.array([10000., 0., 0.]), np.zeros(3), [], camera)
        np.testing.assert_array_equal(c[-1], [100., 2., 0.])
        self.assertEqual(r[-1], 12.)
        self.assertEqual(rates[-1], .25)

    def test_unmatched_camera_bound_retains_unknown_motion_margin(self):
        p = self.planner()
        p.matched_camera_prediction = True
        camera = SimpleNamespace(observes=lambda *a: False,
            obstacles=lambda *a: (np.array([[100., 2., 0.]]), np.zeros((1, 3)), np.array([12.])))
        *_, rates, accels, observed = p.observe_scene(10., np.array([9000., 0., 0.]), np.zeros(3),
            np.array([10000., 0., 0.]), np.zeros(3), [], camera)
        self.assertEqual(rates[-1], .6)

    def test_radar_transit_policy_still_uses_camera_on_terminal_or_missing_radar(self):
        for distance, radar, expected in ((1000., [1], False), (200., [1], True), (1000., [], True)):
            with self.subTest(distance=distance, radar=radar):
                p = self.planner()
                p.radar_cruise_only = True
                calls = []
                camera = SimpleNamespace(observes=lambda *a: True,
                    obstacles=lambda *a: (calls.append(True) or np.empty((0, 3)), np.empty((0, 3)), np.empty(0)))
                p.observe_scene(10., np.array([9000., 0., 0.]), np.array([5., 0., 0.]),
                    np.array([9000.+distance, 0., 0.]), np.zeros(3), radar, camera)
                self.assertEqual(bool(calls), expected)


if __name__ == '__main__':
    unittest.main()
