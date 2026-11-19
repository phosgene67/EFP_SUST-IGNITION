"""Early scored coast prefixes are offered and retain absolute ignition times."""
from pathlib import Path
import unittest
from unittest import mock
import numpy as np
from controller_loader import load_namespace

NS = load_namespace((Path(__file__).resolve().parents[1]/'tests/score_sweeps/score55_rejected_prototypes_controller.py').read_text())


class EarlyGapWaitTests(unittest.TestCase):
    def test_options_are_available_at_low_positive_launch_speed_only_in_outer_leg(self):
        original = NS['axis_schedule']
        for distance, expected in ((1300., [0., 3., 6., 10.]), (500., [0.])):
            observed = []
            def schedule(*args, **kwargs):
                observed.append(np.unique(args[4]).tolist())
                return original(*args, **kwargs)
            p = NS['ShootingNavigator'](launch=False)
            p.wait_for_gap = True
            with mock.patch.dict(NS, axis_schedule=schedule):
                p.command(20., np.array([9000., 0., 0.]), np.array([.7, 0., 0.]),
                    np.array([9000.+distance, 0., 0.]), np.zeros(3), [])
            self.assertEqual(observed[0], expected)

    def test_retained_wait_never_slides_ignition_forward(self):
        p = NS['ShootingNavigator'](launch=False)
        p.wait_for_gap = True
        location, velocity, target = np.array([9000., 0., 0.]), np.array([2., 0., 0.]), np.array([10300., 0., 0.])
        p.command(100., location, velocity, target, np.zeros(3), [])
        duration, gravity = p.previous_eta-100., p.gravity_at(location)
        needed = (p.terminal.inner_goal-location-velocity*duration-.5*gravity*duration**2)[None]
        dv, _, _, valid = NS['axis_schedule'](needed, np.array([duration]), p.burn_order[None], p.axis_accels, np.array([4.]))
        self.assertTrue(valid[0])
        p.burn_target, p.burn_gravity = velocity+dv[0], gravity
        p.burn_started = p.plan_chosen_at = 100.
        p.axis_done, p.burn_signs = np.abs(dv[0]) < .02, np.sign(dv[0])
        p.not_before, p.shooting_active = 104., True
        for now in (100.5, 101.):
            dt = now-100.
            command, mode, _ = p.command(now, location+dt*velocity+.5*dt**2*gravity,
                velocity+dt*gravity, target, np.zeros(3), [])
            self.assertEqual(mode, 'WAIT_CLEAR_PATH')
            self.assertEqual(p.not_before, 104.)
            np.testing.assert_array_equal(command, np.zeros(3))


if __name__ == '__main__':
    unittest.main()
