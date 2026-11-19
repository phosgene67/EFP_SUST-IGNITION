"""Risk calibration and momentum-preserving local avoidance regression checks."""
from pathlib import Path
import unittest
from unittest import mock
import numpy as np
from controller_loader import load_namespace


NS = load_namespace((Path(__file__).resolve().parents[1]/'Controller.py').read_text())


class MomentumRiskAvoidanceTests(unittest.TestCase):
    def test_nonfatal_energy_model_keeps_geometrically_clear_finite_burn(self):
        # Isolate the fatal-energy gate from radar fit/position noise. The
        # uncertainty and geometric envelope are identical for both models.
        def risk(gap, sigma, speed):
            shape = np.asarray(gap).shape
            return np.full(shape, .02), np.full(shape, 250000.)
        results = []
        for factor in (1., .5):
            p = NS['ShootingNavigator'](launch=False)
            p.expected_risk, p.impact_energy_factor = True, factor
            radar = [np.array([[100., 30., 0.], [100., -8., 4.], [100., 0., 0.]])]
            with mock.patch.dict(NS, gaussian_encounter_risk=risk):
                _, _, gap = p.command(100., np.array([9000., 0., 0.]), np.array([8., 0., 0.]),
                    np.array([10300., 0., 0.]), np.zeros(3), radar)
            results.append((p, gap))
        self.assertFalse(results[0][0].shooting_active)
        self.assertTrue(results[1][0].shooting_active)
        self.assertGreater(results[1][1], .5)
        self.assertAlmostEqual(results[1][0].expected_damage, 2500.)

    def test_gaussian_tail_matches_known_normal_probabilities(self):
        p, energy = NS['gaussian_encounter_risk'](np.array([0., 2., 4., 6.]), 2., 5.)
        np.testing.assert_allclose(p, [.5, .158655254, .022750132, .001349898], atol=1e-9)
        np.testing.assert_allclose(energy, 3388.7*25.)

    def test_small_sidestep_clears_obstacle_without_cancelling_forward_momentum(self):
        outcomes = []
        for enabled in (False, True):
            planner = NS['EconomyNavigator'](launch=False)
            planner.fine_avoidance = enabled
            planner.radar_forecast_rates = np.zeros(1)
            planner.obstacles = lambda *a: (np.array([[70., 0., 0.]]), np.zeros((1, 3)), np.array([5.]))
            velocity = np.array([6., 0., 0.])
            _, mode, gap = planner.command(100., np.array([9000., 0., 0.]), velocity,
                np.array([10300., 0., 0.]), np.zeros(3), [], cruise_speed=6.)
            outcomes.append((mode, gap, planner.previous_velocity.copy()))
        self.assertEqual(outcomes[0][0], 'ESCAPE')
        mode, gap, chosen = outcomes[1]
        self.assertEqual(mode, 'AVOID')
        self.assertGreater(gap, 0.)
        self.assertGreater(chosen[0], 5.8)
        self.assertAlmostEqual(np.linalg.norm(chosen), 6.)
        self.assertLess(np.linalg.norm(chosen-velocity), np.linalg.norm(outcomes[0][2]-velocity))


if __name__ == '__main__':
    unittest.main()
