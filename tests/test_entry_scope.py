"""Certify entry before ignoring contacts after the official clock has stopped."""
from types import SimpleNamespace
import unittest
from pathlib import Path

import numpy as np
from controller_loader import load_namespace

# The matched trial was rejected for insufficient score gain; test its frozen
# source without restoring the experiment into the active Momentum controller.
namespace = load_namespace((Path(__file__).resolve().parent / 'score_sweeps' /
                            'momentum_terminal_scope_controller.py').read_text())


def camera_sphere(center, radius, observed=True):
    return SimpleNamespace(frames=1, observes=lambda *args: observed,
        obstacles=lambda *args: (np.asarray(center)[None], np.zeros((1, 3)), np.array([radius])),
        last_uncertainty=np.empty(0))


class EntryScopeTests(unittest.TestCase):
    def test_entry_fraction_requires_two_metres_inside_the_safe_sphere(self):
        start = np.array([[-100., 0., 0.], [-100., 0., 0.], [-60., 60., 0.], [-60., 0., 0.]])
        end = np.array([[-50., 0., 0.], [-47., 0., 0.], [60., 60., 0.], [-60., 0., 0.]])
        fraction, enters = namespace['capture_fraction'](start, end)
        np.testing.assert_array_equal(enters, [False, True, False, False])
        point = start[1]+fraction[1]*(end[1]-start[1])
        self.assertAlmostEqual(np.linalg.norm(point), 48.)
        # A contact at 55 m remains on the certified segment.
        self.assertGreater(fraction[1], 45./53.)

    def test_post_capture_obstacle_does_not_cancel_a_successful_coast(self):
        location = np.array([9000., 0., 0.])
        target = location+np.array([60., 0., 0.])
        ratio = 47./60.
        heading = np.array([np.sqrt(1.-ratio**2), ratio, 0.])
        closest = location+60.*np.sqrt(1.-ratio**2)*heading
        obstacle = closest+8.*heading-location
        results = []
        for enabled in (False, True):
            planner = namespace['ShootingNavigator'](launch=False, entry_scope=enabled)
            planner.terminal.side, planner.terminal.tangent = np.array([-1., 0., 0.]), np.array([0., 1., 0.])
            command, mode, _ = planner.command(100., location, 7.*heading, target, np.zeros(3), [],
                                               camera=camera_sphere(obstacle, 4.))
            results.append((mode, planner.coast_clearance, command))
        self.assertLess(results[0][1], 0.)
        self.assertEqual(results[1][0], 'COAST')
        self.assertGreater(results[1][1], 2.)
        np.testing.assert_array_equal(results[1][2], np.zeros(3))

    def test_obstacle_before_entry_is_still_blocked(self):
        planner = namespace['ShootingNavigator'](launch=False, entry_scope=True)
        planner.command(100., np.array([9000., 0., 0.]), np.array([7., 0., 0.]),
            np.array([9200., 0., 0.]), np.zeros(3), [], camera=camera_sphere([200., 0., 0.], 60.))
        self.assertNotEqual(planner.last_phase, 'CAPTURE_PASS')

    def test_scope_does_not_bypass_an_unobserved_station_guard(self):
        planner = namespace['ShootingNavigator'](launch=False, entry_scope=True)
        planner.command(100., np.array([9000., 0., 0.]), np.array([7., 0., 0.]),
            np.array([9200., 0., 0.]), np.zeros(3), [],
            camera=camera_sphere([1000., 0., 0.], 5., observed=False))
        self.assertEqual(planner.last_phase, 'WAIT_SHIP_CAMERA')
        self.assertFalse(namespace['ShootingNavigator']().entry_scope)


if __name__ == '__main__':
    unittest.main()
