"""Risk must agree with the existing monotonic-departure geometry rule."""
from pathlib import Path
import unittest
import numpy as np
from controller_loader import load_namespace

NS = load_namespace((Path(__file__).resolve().parents[1]/'tests/score_sweeps/score55_rejected_prototypes_controller.py').read_text())


class DepartureRiskTests(unittest.TestCase):
    def plan(self, enabled, behind):
        p = NS['ShootingNavigator'](launch=False)
        p.departure_risk, p.expected_risk, p.risk_diagnostics = enabled, True, True
        rock = np.array([[-10. if behind else 10., 0., 0.],
                         [5., 8. if behind else -8., 10.], [20., 0., 0.]])
        p.command(20., np.array([9000., 0., 0.]), np.array([8., 0., 0.]),
            np.array([10300., 0., 0.]), np.zeros(3), [rock])
        return p

    def test_receding_enclosure_does_not_cancel_a_safe_forward_burn(self):
        old, new = self.plan(False, True), self.plan(True, True)
        self.assertFalse(old.shooting_active)
        self.assertGreater(old.last_risk_rejection['expected_damage_J'], 40000.)
        self.assertTrue(new.shooting_active)
        self.assertEqual(new.expected_damage, 0.)
        self.assertEqual(new.contact_probability, 0.)
        self.assertGreaterEqual(new.burn_target[0], 8.)

    def test_approaching_enclosure_retains_the_collision_guard(self):
        p = self.plan(True, False)
        self.assertFalse(p.shooting_active)
        self.assertIsNotNone(p.last_risk_rejection)
        self.assertLess(p.last_risk_rejection['clearance_m'], .5)

    def test_small_uncertain_receding_speed_is_not_exempted(self):
        p = NS['ShootingNavigator'](launch=False)
        p.departure_risk, p.expected_risk = True, True
        rock = np.array([[-10., 0., 0.], [5., .1, 10.], [20., 0., 0.]])
        p.command(20., np.array([9000., 0., 0.]), np.array([8., 0., 0.]),
            np.array([10300., 0., 0.]), np.zeros(3), [rock])
        self.assertGreater(p.expected_damage, 0.)


if __name__ == '__main__':
    unittest.main()
