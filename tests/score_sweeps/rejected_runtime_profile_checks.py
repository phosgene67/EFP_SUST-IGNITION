"""Current experimental profile choices survive native clock resets."""
import unittest
import test_sim_adapter


class ActiveRuntimeProfileTests(unittest.TestCase):
    run_adapter = test_sim_adapter.AdapterTests.run_adapter

    def test_z_alignment_cache_and_sensor_settle_survive_reset(self):
        self.run_adapter([], reset_clock=True, runtime_flags={'EFP_STRONG_AXIS': '1', 'EFP_MAIN_AXIS_Z': '1',
                'EFP_EFFICIENT_MAIN': '1', 'EFP_TRACK_CACHE': '1', 'EFP_SENSOR_SETTLE': '1',
                'EFP_CALIBRATED_LAUNCH': '1', 'EFP_EFFICIENT_X': '1', 'EFP_POWERED_LAUNCH_SLEW': '1',
                'EFP_NEAR_RISK_HORIZON': '1', 'EFP_RISK_DIAGNOSTICS': '1', 'EFP_FORCE_ALIGNED_SLEW': '1',
                'EFP_SCORE_ENTRY_LOCK': '1', 'EFP_LOOKAHEAD_FUEL': '1', 'EFP_DEPARTURE_RISK': '1',
                'EFP_SOFT_RISK_GATES': '1'})
        planner = self.runtime['planner']
        self.assertEqual(planner.main_axis, 2)
        self.assertTrue(planner.efficient_main)
        self.assertTrue(planner.cache_reports)
        self.assertTrue(planner.sensor_settle)
        self.assertTrue(planner.calibrated_launch)
        self.assertTrue(planner.efficient_x)
        self.assertTrue(planner.powered_launch_slew)
        self.assertTrue(planner.force_aligned_slew)
        self.assertTrue(planner.risk_diagnostics)
        self.assertTrue(planner.score_entry_lock)
        self.assertTrue(planner.lookahead_fuel)
        self.assertTrue(planner.departure_risk)
        self.assertTrue(planner.soft_risk_gates)
        self.assertEqual(planner.risk_horizon_limit, 10.)
        self.assertEqual(type(self.runtime['flight_attitude']).__name__, 'MainAxisAttitude')
        self.assertIs(self.runtime['flight_attitude'].planner, planner)

    def test_no_slew_profile_keeps_free_attitude_after_reset(self):
        self.run_adapter([], reset_clock=True, runtime_flags={'EFP_STRONG_AXIS': '0', 'EFP_MAIN_AXIS_Z': '0',
                'EFP_EFFICIENT_MAIN': '1', 'EFP_TRACK_CACHE': '1', 'EFP_SENSOR_SETTLE': '0'})
        self.assertEqual(type(self.runtime['planner']).__name__, 'ShootingNavigator')
        self.assertEqual(type(self.runtime['flight_attitude']).__name__, 'FreeAttitude')


if __name__ == '__main__':
    unittest.main()
