"""Ship-view recovery is a guarded coast maneuver, not continuous nose following."""
from types import SimpleNamespace
import unittest

import numpy as np
from test_controller import namespace


class ViewRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.attitude = namespace['ViewRecoveryAttitude']()
        self.planner = namespace['ShootingNavigator'](launch=False)
        self.planner.last_phase = 'CRUISE'
        self.planner.latest_radar_count = 1
        self.planner.latest_scene = (np.array([[1000., 0., 0.]]), np.zeros((1, 3)), np.array([5.]))
        self.planner.latest_accels = np.zeros((1, 3))
        self.camera = SimpleNamespace(frames=1, observes=lambda *args: False)
        self.location, self.target = np.array([9000., 0., 0.]), np.array([9000., 0., 300.])

    def configure(self, linear=None, velocity=None):
        self.attitude.configure(100., self.planner, self.location,
            np.zeros(3) if velocity is None else velocity, self.target, np.zeros(3),
            np.zeros(3) if linear is None else linear, self.camera)

    def command(self, omega=None):
        return self.attitude.command(100., np.eye(3), np.zeros(3), np.zeros(3),
            np.array([1., 0., 0.]), np.zeros(3) if omega is None else omega, .02)

    def test_small_camera_turn_only_when_clear_and_unobserved(self):
        self.configure()
        self.assertTrue(self.attitude.eligible)
        result = self.command()
        self.assertLess(result[1], 0.)
        self.assertLess(np.linalg.norm(result), 1e-4)
        self.assertTrue(self.attitude.viewing)
        # A covered bearing with enough view margin needs no body pointing.
        self.camera.observes = lambda *args: True
        self.target = self.location+np.array([300., 0., 0.])
        self.configure()
        np.testing.assert_array_equal(self.command(), np.zeros(3))

    def test_covered_station_keeps_margin_before_the_gimbals_saturate(self):
        self.camera.observes = lambda *args: True
        self.configure()
        self.assertTrue(self.attitude.eligible)
        self.assertLess(self.command()[1], 0.)

    def test_active_thrust_or_blocked_coast_disables_view_turn(self):
        self.configure(linear=np.array([0., 0., .5]))
        self.assertFalse(self.attitude.eligible)
        np.testing.assert_array_equal(self.command(), np.zeros(3))
        self.planner.latest_scene = (np.array([[0., 0., 6.]]), np.zeros((1, 3)), np.array([5.]))
        self.configure()
        self.assertFalse(self.attitude.eligible)

    def test_no_turn_during_an_imminent_station_pass(self):
        self.target = self.location+np.array([0., 0., 100.])
        self.configure(velocity=np.array([0., 0., 6.]))
        self.assertFalse(self.attitude.eligible)

    def test_recovery_settles_rate_after_the_camera_request_ends(self):
        self.attitude.settling = True
        self.attitude.eligible = False
        result = self.command(np.array([0., .005, 0.]))
        self.assertLess(result[1], 0.)
        self.assertLess(np.linalg.norm(result), 1e-4)
        np.testing.assert_array_equal(self.command(), np.zeros(3))
        self.assertFalse(self.attitude.settling)


if __name__ == '__main__':
    unittest.main()
