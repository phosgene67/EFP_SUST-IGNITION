"""First safe-zone entry and pre-entry hull protection for unpowered passes."""
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from controller_loader import load_namespace

NS = load_namespace((Path(__file__).resolve().parents[1]/'Controller.py').read_text())


class BallisticEntryCoastTests(unittest.TestCase):
    def test_facing_boundary_goal_is_priced_with_finite_burns(self):
        outcomes = []
        for enabled in (False, True):
            p = NS['ShootingNavigator'](launch=False)
            p.near_entry_goal = enabled
            p.command(25., np.array([9000., 0., 0.]), np.array([1.6, 0., 0.]),
                np.array([10300., 0., 0.]), np.zeros(3), [])
            outcomes.append(p)
        self.assertTrue(outcomes[1].last_goal_entry)
        np.testing.assert_allclose(outcomes[1].terminal.inner_goal, [10252., 0., 0.])
        self.assertGreater(outcomes[1].plan_score, outcomes[0].plan_score)
        self.assertLess(outcomes[1].previous_eta, outcomes[0].previous_eta)

    def test_entry_is_first_inward_crossing_with_two_metres_margin(self):
        t = NS['ballistic_entry'](np.array([9000., 0., 0.]), np.array([8., 0., 0.]),
            np.array([9100., 0., 0.]), np.zeros(3), np.zeros(3))
        self.assertAlmostEqual(t, 6.5)

    def test_miss_or_departure_cannot_justify_switching_off_thrust(self):
        for velocity, target in (([-8., 0., 0.], [9100., 0., 0.]), ([8., 0., 0.], [9100., 49., 0.])):
            self.assertFalse(np.isfinite(NS['ballistic_entry'](np.array([9000., 0., 0.]),
                np.array(velocity), np.array(target), np.zeros(3), np.zeros(3))))

    def test_pre_entry_obstacle_still_blocks_coasting(self):
        p = NS['ShootingNavigator'](launch=False)
        p.entry_coast = True
        camera = SimpleNamespace(observes=lambda *a: True,
            obstacles=lambda *a: (np.array([[10., 0., 0.]]), np.zeros((1, 3)), np.array([4.])))
        p.command(100., np.array([9000., 0., 0.]), np.array([7., 0., 0.]),
                  np.array([9060., 0., 0.]), np.zeros(3), [], camera=camera)
        self.assertNotEqual(p.last_mode, 'ENTRY_COAST')

    def test_obstacle_beyond_entry_does_not_cancel_an_acquired_pass(self):
        p = NS['ShootingNavigator'](launch=False)
        p.entry_coast = True
        p.gravity_at = lambda *a: np.zeros(3)
        heading = np.array([np.sqrt(1.-(47./60.)**2), 47./60., 0.])
        closest = 60.*np.sqrt(1.-(47./60.)**2)*heading
        obstacle = closest+8.*heading
        camera = SimpleNamespace(observes=lambda *a: True,
            obstacles=lambda *a: (obstacle[None], np.zeros((1, 3)), np.array([4.])))
        command, mode, gap = p.command(100., np.array([9000., 0., 0.]), 7.*heading,
            np.array([9060., 0., 0.]), np.zeros(3), [], camera=camera)
        self.assertEqual(mode, 'ENTRY_COAST')
        self.assertGreater(gap, 2.)
        np.testing.assert_array_equal(command, np.zeros(3))


if __name__ == '__main__':
    unittest.main()
