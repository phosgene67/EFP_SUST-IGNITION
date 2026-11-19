"""Price uncertain failure while keeping the same hard collision geometry."""
from pathlib import Path
from unittest import mock
import unittest
import numpy as np
from controller_loader import load_namespace

NS = load_namespace((Path(__file__).resolve().parents[1]/'tests/score_sweeps/score55_rejected_prototypes_controller.py').read_text())


class SoftRiskGatesTests(unittest.TestCase):
    def plan(self, enabled, probability):
        p = NS['ShootingNavigator'](launch=False)
        p.soft_risk_gates, p.expected_risk = enabled, True
        def risk(gap, sigma, speed):
            return np.full(np.asarray(gap).shape, probability), np.full(np.asarray(gap).shape, 250000.)
        radar = [np.array([[100., 30., 0.], [100., -8., 4.], [100., 0., 0.]])]
        with mock.patch.dict(NS, gaussian_encounter_risk=risk):
            _, _, gap = p.command(100., np.array([9000., 0., 0.]), np.array([8., 0., 0.]),
                np.array([10300., 0., 0.]), np.zeros(3), radar)
        return p, gap

    def test_geometrically_clear_risk_is_priced_instead_of_forcing_local_fallback(self):
        old, _ = self.plan(False, .02)
        new, gap = self.plan(True, .02)
        clean, _ = self.plan(True, 0.)
        self.assertFalse(old.shooting_active)
        self.assertTrue(new.shooting_active)
        self.assertGreater(gap, .5)
        self.assertAlmostEqual(new.expected_damage, 5000.)
        self.assertLess(new.plan_score, clean.plan_score-4.78)

    def test_imminent_geometric_collision_remains_blocked(self):
        p = NS['ShootingNavigator'](launch=False)
        p.soft_risk_gates, p.risk_diagnostics = True, True
        radar = [np.array([[35., 0., 0.], [30., -8., 10.], [20., 0., 0.]])]
        p.command(20., np.array([9000., 0., 0.]), np.array([8., 0., 0.]),
            np.array([10300., 0., 0.]), np.zeros(3), radar)
        self.assertFalse(p.shooting_active)
        self.assertLess(p.last_risk_rejection['clearance_m'], .5)


if __name__ == '__main__':
    unittest.main()
