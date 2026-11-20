"""Learn uncommanded lateral acceleration without confusing it with thrust."""
from pathlib import Path
import unittest
from unittest import mock
import numpy as np
from controller_loader import load_namespace
import test_sim_adapter

SOURCE = Path(__file__).parent/'score_sweeps'/'nov20_powered_drift_controller.py'
NS = load_namespace(SOURCE.read_text())


class PoweredDriftTests(unittest.TestCase):
    run_adapter = test_sim_adapter.AdapterTests.run_adapter

    def flight(self, enabled=True, transverse=(.002, -.001), quiet=False, rotated=False):
        p = NS['ShootingNavigator'](launch=False)
        p.powered_drift = enabled
        p.gravity_at = lambda loc: p.dynamics.residual.copy()
        p.measurement = (100., np.array([8., 0., 0.]), np.zeros(3), np.eye(3))
        if quiet:
            p.dynamics.quiet_after = 120.
        if rotated:
            angle = .02
            p.body_rotation = np.array([[np.cos(angle), np.sin(angle), 0.],
                [-np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
        velocity, location = np.array([8., 0., 0.]), np.array([9000., 0., 0.])
        for i in range(16):
            # Forward thrust varies independently of the ambient side drift.
            acceleration = np.array([.08+.01*i, *transverse])
            location += .5*velocity+.125*acceleration
            velocity += .5*acceleration
            p.shooting_active, p.selected_axis, p.selected_sign, p.selected_remaining = True, 0, 1., 2.
            p.axis_done = np.array([False, True, True])
            if rotated:
                stamp, oldv, oldg, _ = p.measurement
                p.measurement = (stamp, oldv, oldg, np.eye(3))
            p.command(100.+.5*(i+1), location, velocity, np.array([10300., 0., 0.]), np.zeros(3), [])
        return p

    def test_lateral_estimate_converges_during_variable_forward_burn(self):
        before, after = self.flight(False), self.flight(True)
        np.testing.assert_array_equal(before.dynamics.residual, np.zeros(3))
        np.testing.assert_allclose(after.dynamics.residual[1:], [.002, -.001], atol=.00016)
        self.assertLess(abs(after.dynamics.residual[0]), 1e-12)

    def test_large_cross_axis_acceleration_is_not_learned(self):
        after = self.flight(transverse=(.02, 0.))
        np.testing.assert_array_equal(after.dynamics.residual, np.zeros(3))

    def test_contact_quiet_period_and_turning_prevent_learning(self):
        for options in (dict(quiet=True), dict(rotated=True)):
            np.testing.assert_array_equal(self.flight(**options).dynamics.residual, np.zeros(3))

    def test_flag_survives_simulation_clock_reset(self):
        with mock.patch.object(test_sim_adapter, 'SOURCE', SOURCE):
            self.run_adapter([], reset_clock=True, runtime_flags={'EFP_POWERED_DRIFT': '1'})
        self.assertTrue(self.runtime['planner'].powered_drift)


if __name__ == '__main__':
    unittest.main()
