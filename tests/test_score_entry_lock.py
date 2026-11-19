"""Stop geometry checks at scoring entry, retaining every pre-entry obstacle."""
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from controller_loader import load_namespace

NS = load_namespace((Path(__file__).resolve().parents[1]/'tests/score_sweeps/score55_rejected_prototypes_controller.py').read_text())


class ScoreEntryLockTests(unittest.TestCase):
    def planner(self):
        p = NS['ShootingNavigator'](launch=False)
        p.score_entry_lock = True
        p.gravity_at = lambda *a: np.zeros(3)
        return p

    def camera(self, x, radius):
        return SimpleNamespace(observes=lambda *a: False,
            obstacles=lambda *a: (np.array([[x, 0., 0.]]), np.zeros((1, 3)), np.array([radius])))

    def test_hull_after_boundary_does_not_cause_braking(self):
        p = self.planner()
        command, mode, gap = p.command(100., np.array([9000., 0., 0.]), np.array([7., 0., 0.]),
            np.array([9060., 0., 0.]), np.zeros(3), [], camera=self.camera(16., 4.))
        self.assertEqual(mode, 'SCORE_ENTRY_COAST')
        self.assertGreater(gap, .25)
        np.testing.assert_array_equal(command, np.zeros(3))
        self.assertIsNone(p.arrival_time)

    def test_obstacle_before_boundary_still_blocks_the_pass(self):
        p = self.planner()
        _, mode, _ = p.command(100., np.array([9000., 0., 0.]), np.array([7., 0., 0.]),
            np.array([9060., 0., 0.]), np.zeros(3), [], camera=self.camera(4., 3.))
        self.assertNotEqual(mode, 'SCORE_ENTRY_COAST')

    def test_subsecond_entry_does_not_extend_check_to_one_second(self):
        p = self.planner()
        command, mode, _ = p.command(100., np.array([9000., 0., 0.]), np.array([8., 0., 0.]),
            np.array([9050.1, 0., 0.]), np.zeros(3), [], camera=self.camera(5., 1.))
        self.assertEqual(mode, 'SCORE_ENTRY_COAST')
        np.testing.assert_array_equal(command, np.zeros(3))

    def test_missed_sphere_or_departure_cannot_lock_coast(self):
        for target, velocity in (([9060., 51., 0.], [7., 0., 0.]),
                                 ([9060., 0., 0.], [-7., 0., 0.])):
            p = self.planner()
            _, mode, _ = p.command(100., np.array([9000., 0., 0.]), np.array(velocity),
                np.array(target), np.zeros(3), [])
            self.assertNotEqual(mode, 'SCORE_ENTRY_COAST')


if __name__ == '__main__':
    unittest.main()
