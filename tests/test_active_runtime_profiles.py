"""Selected Momentum configuration survives native simulation resets."""
import unittest
import test_sim_adapter

class ActiveRuntimeProfileTests(unittest.TestCase):
    run_adapter = test_sim_adapter.AdapterTests.run_adapter

    def test_selected_configuration_survives_reset(self):
        self.run_adapter([], reset_clock=True, runtime_flags={
            'EFP_STRONG_AXIS':'1', 'EFP_TRACK_CACHE':'1', 'EFP_RADAR_CRUISE':'1',
            'EFP_EXPECTED_RISK':'1', 'EFP_FINE_AVOIDANCE':'1'})
        p = self.runtime['planner']
        self.assertEqual(type(p).__name__, 'AxisAlignedNavigator')
        self.assertEqual(p.main_axis, 0)
        for name in ('cache_reports','radar_cruise_only','expected_risk','fine_avoidance'):
            self.assertTrue(getattr(p, name))
        self.assertFalse(p.efficient_main)
        self.assertFalse(p.nose_first)
        self.assertIs(self.runtime['flight_attitude'].planner, p)

    def test_environment_can_disable_selected_optional_features(self):
        self.run_adapter([], reset_clock=True, runtime_flags={
            'EFP_STRONG_AXIS':'0', 'EFP_TRACK_CACHE':'0', 'EFP_RADAR_CRUISE':'0',
            'EFP_EXPECTED_RISK':'0', 'EFP_FINE_AVOIDANCE':'0'})
        self.assertEqual(type(self.runtime['planner']).__name__, 'ShootingNavigator')
        self.assertEqual(type(self.runtime['flight_attitude']).__name__, 'FreeAttitude')

if __name__ == '__main__':
    unittest.main()
