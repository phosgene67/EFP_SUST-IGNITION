"""Bounded startup coasting matures sensing without repeated cruise holds."""
from pathlib import Path
import unittest
import numpy as np
from controller_loader import load_namespace

NS = load_namespace((Path(__file__).resolve().parents[1]/'tests/score_sweeps/score55_rejected_prototypes_controller.py').read_text())


class SensorSettleTests(unittest.TestCase):
    def planner(self, confident=False):
        p = NS['ShootingNavigator'](launch=False)
        p.sensor_settle = True
        observe = p.observe_scene
        def scene(*args):
            result = observe(*args)
            p.radar_track_confidence[:] = confident
            return result
        p.observe_scene = scene
        return p

    def command(self, p, now, rock=(500., 100., 0.)):
        radar = [np.array([rock, [500., -1., 4.], [now, 0., 0.]])]
        return p.command(now, np.array([9000., 0., 0.]), np.array([1., 0., 0.]),
                         np.array([10300., 0., 0.]), np.zeros(3), radar)

    def test_startup_coast_preserves_speed_and_ends_after_good_observations(self):
        p = self.planner(confident=True)
        for now in (1., 3.):
            command, mode, _ = self.command(p, now)
            self.assertEqual(mode, 'SENSOR_SETTLE')
            np.testing.assert_array_equal(command, np.zeros(3))
            np.testing.assert_array_equal(p.previous_velocity, [1., 0., 0.])
        self.assertNotEqual(self.command(p, 5.)[1], 'SENSOR_SETTLE')
        self.assertTrue(p.sensor_settle_done)
        self.assertEqual(p.sensor_settle_started, 1.)

    def test_low_confidence_does_not_slide_the_timeout(self):
        p = self.planner()
        self.assertEqual(self.command(p, 1.)[1], 'SENSOR_SETTLE')
        self.assertNotEqual(self.command(p, 13.)[1], 'SENSOR_SETTLE')
        self.assertTrue(p.sensor_settle_done)

    def test_immediate_collision_risk_does_not_force_a_coast(self):
        p = self.planner()
        self.assertNotEqual(self.command(p, 1., (2., 0., 0.))[1], 'SENSOR_SETTLE')
        self.assertTrue(p.sensor_settle_done)


if __name__ == '__main__':
    unittest.main()
