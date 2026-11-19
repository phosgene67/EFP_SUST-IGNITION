"""Optional rejection logs cannot change the commanded trajectory."""
from pathlib import Path
import unittest
import numpy as np
from controller_loader import load_namespace

NS = load_namespace((Path(__file__).resolve().parents[1]/'tests/score_sweeps/score55_rejected_prototypes_controller.py').read_text())


class RiskDiagnosticsTests(unittest.TestCase):
    def test_shorter_forecast_still_rejects_immediate_blockage(self):
        p = NS['ShootingNavigator'](launch=False)
        p.risk_diagnostics, p.risk_horizon_limit = True, 10.
        rock = np.array([[50., 0., 0.], [0., -5., 40.], [10., 0., 0.]])
        p.command(10., np.array([9000., 0., 0.]), np.array([5., 0., 0.]),
            np.array([10300., 0., 0.]), np.zeros(3), [rock])
        self.assertIsNotNone(p.last_risk_rejection)
        self.assertLess(p.last_risk_rejection['clearance_m'], .5)
        self.assertLessEqual(p.last_risk_rejection['closest_obstacle']['time_to_closest_s'], 10.)

    def test_blocked_shots_record_reason_without_changing_control(self):
        results = []
        planners = []
        rock = np.array([[50., 0., 0.], [0., -5., 40.], [10., 0., 0.]])
        for enabled in (False, True):
            p = NS['ShootingNavigator'](launch=False)
            p.risk_diagnostics = enabled
            results.append(p.command(10., np.array([9000., 0., 0.]), np.array([5., 0., 0.]),
                np.array([10300., 0., 0.]), np.zeros(3), [rock]))
            planners.append(p)
        np.testing.assert_array_equal(results[0][0], results[1][0])
        self.assertEqual(results[0][1:], results[1][1:])
        self.assertIsNone(planners[0].last_risk_rejection)
        snapshot = planners[1].last_risk_rejection
        self.assertIsNotNone(snapshot)
        self.assertLess(snapshot['clearance_m'], .5)
        self.assertIn('sigma_m', snapshot['closest_obstacle'])
        self.assertEqual(snapshot['closest_obstacle']['scene_index'], 0)


if __name__ == '__main__':
    unittest.main()
