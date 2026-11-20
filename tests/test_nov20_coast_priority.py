"""Do not retain a lower-scoring burn over an already checked coast."""
from pathlib import Path
import unittest
from unittest import mock
import numpy as np
from controller_loader import load_namespace
import test_sim_adapter

SOURCE = Path(__file__).parent/'score_sweeps'/'nov20_coast_priority_controller.py'
NS = load_namespace(SOURCE.read_text())


class CoastPriorityTests(unittest.TestCase):
    run_adapter = test_sim_adapter.AdapterTests.run_adapter

    def flight(self, enabled, miss=47., radar=None):
        p = NS['ShootingNavigator'](launch=False)
        p.coast_priority = enabled
        p.gravity_at = lambda loc: np.zeros(3)
        p.terminal.side, p.terminal.tangent = np.array([-1., 0., 0.]), np.array([0., 1., 0.])
        p.shooting_active, p.previous_eta = True, 156.25
        p.burn_target = np.array([8., .1, 0.])
        p.axis_done, p.burn_signs = np.array([True, False, True]), np.array([0., 1., 0.])
        p.plan_chosen_at = p.burn_started = 98.
        result = p.command(100., np.array([9000., miss, 0.]), np.array([8., 0., 0.]),
                           np.array([9450., 0., 0.]), np.zeros(3), radar or [])
        return p, result

    def test_higher_scoring_clear_coast_beats_retained_sideways_burn(self):
        original, before = self.flight(False)
        changed, after = self.flight(True)
        self.assertEqual(before[1], 'IMPULSE_BURN')
        self.assertEqual(original.coast_best_suppressed, 1)
        self.assertEqual(after[1], 'IMPULSE_COAST')
        self.assertEqual(changed.coast_best_kept, 1)
        self.assertEqual(changed.last_delta_v, 0.)
        np.testing.assert_array_equal(after[0], np.zeros(3))
        self.assertGreater(before[0][1], 0.)

    def test_predicted_miss_still_requires_a_burn(self):
        p, result = self.flight(True, miss=50.5)
        self.assertEqual(p.coast_best_kept, 0)
        self.assertGreater(np.linalg.norm(result[0]), 0.)

    def test_obstacle_checks_can_still_reject_coasting(self):
        packet = np.array([[120., 0., 0.], [118., -8., 4.], [100., 0., 0.]])
        p, result = self.flight(True, radar=[packet])
        self.assertEqual(p.coast_best_kept, 0)
        self.assertNotEqual(result[1], 'IMPULSE_COAST')

    def test_flag_survives_simulation_clock_reset(self):
        with mock.patch.object(test_sim_adapter, 'SOURCE', SOURCE):
            self.run_adapter([], reset_clock=True, runtime_flags={'EFP_COAST_PRIORITY': '1'})
        self.assertTrue(self.runtime['planner'].coast_priority)


if __name__ == '__main__':
    unittest.main()
