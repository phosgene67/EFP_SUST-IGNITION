"""Early fallback sidesteps preserve momentum without relaxing collision checks."""
from pathlib import Path
import unittest
from unittest import mock
import numpy as np
from controller_loader import load_namespace
import test_sim_adapter

NS = load_namespace((Path(__file__).parent/'score_sweeps'/'nov20_early_fine_controller.py').read_text())


class EarlyFineAvoidanceTests(unittest.TestCase):
    run_adapter = test_sim_adapter.AdapterTests.run_adapter

    def test_opt_in_survives_simulation_clock_reset(self):
        source = Path(__file__).parent/'score_sweeps'/'nov20_early_fine_controller.py'
        with mock.patch.object(test_sim_adapter, 'SOURCE', source):
            self.run_adapter([], reset_clock=True,
                runtime_flags={'EFP_EARLY_FINE_AVOIDANCE': '1'})
        self.assertTrue(self.runtime['planner'].early_fine_avoidance)

    def flight(self, enabled, obstacle_distance):
        planner = NS['EconomyNavigator'](launch=False)
        planner.fine_avoidance, planner.early_fine_avoidance = True, enabled
        planner.radar_forecast_rates = np.zeros(1)
        planner.obstacles = lambda *args: (
            np.array([[obstacle_distance, 0., 0.]]), np.zeros((1, 3)), np.array([5.]))
        velocity = np.array([2., 0., 0.])
        _, mode, gap = planner.command(100., np.array([9000., 0., 0.]), velocity,
            np.array([10300., 0., 0.]), np.zeros(3), [], cruise_speed=6.)
        return planner, mode, gap, np.linalg.norm(planner.previous_velocity-velocity)

    def test_low_speed_sidestep_avoids_large_velocity_change(self):
        original, _, _, original_dv = self.flight(False, 30.)
        changed, mode, gap, changed_dv = self.flight(True, 30.)
        self.assertEqual(mode, 'AVOID')
        self.assertGreater(gap, 0.)
        self.assertGreater(changed.previous_velocity[0], 1.8)
        self.assertLess(changed_dv, .7)
        self.assertLess(changed_dv, original_dv/4.)
        self.assertAlmostEqual(np.linalg.norm(changed.previous_velocity), 2.)

    def test_close_blockage_is_not_declared_safe(self):
        _, mode, gap, _ = self.flight(True, 12.)
        self.assertEqual(mode, 'ESCAPE')
        self.assertLess(gap, 0.)


if __name__ == '__main__':
    unittest.main()
