"""Task 2: radial-tangent basis and unchanged fallback for degenerate headings."""
import unittest
from pathlib import Path
import numpy as np
from controller_loader import load_namespace, load_archived_namespace

namespace = load_archived_namespace()


class GroundLateralTests(unittest.TestCase):
    def plans(self, enabled, up):
        p = namespace['DoglegNavigator'](launch=False, ground_lateral=enabled)
        p.lane_aligned = True
        p.control_time = 15.
        p.launch.up = up
        loc = np.array([9000., 0., 0.])
        forward = np.array([.7, -.2, .5]); forward /= np.linalg.norm(forward)
        target = loc+1300.*forward
        plan, scores = p.candidates(loc, forward, target, np.zeros(3))
        return plan, scores, forward, target

    def test_side_is_tangent_to_radial_plane_and_perpendicular_to_route(self):
        up = np.array([1., 2., 3.]); up /= np.linalg.norm(up)
        plan, _, forward, target = self.plans(True, up)
        side = (plan['goal'][0]-target+20.*forward)/40.
        expected = np.cross(forward, up); expected /= np.linalg.norm(expected)
        np.testing.assert_allclose(side, expected, atol=1e-12)
        self.assertAlmostEqual(side@up, 0., places=12)
        self.assertAlmostEqual(side@forward, 0., places=12)
        tangent = (plan['goal'][22]-target+20.*forward)/40.
        np.testing.assert_allclose(tangent, np.cross(forward, side), atol=1e-12)

    def test_missing_or_parallel_up_preserves_all_original_candidates(self):
        forward = np.array([.7, -.2, .5]); forward /= np.linalg.norm(forward)
        for up in (None, forward, -forward, np.zeros(3), np.full(3, np.nan)):
            original, scores, _, _ = self.plans(False, up)
            changed, new_scores, _, _ = self.plans(True, up)
            for key in original:
                np.testing.assert_array_equal(original[key], changed[key])
            np.testing.assert_array_equal(scores, new_scores)

    def test_launch_up_is_outward_and_ascent_height_is_radial(self):
        launch = namespace['GuardedLaunch']()
        loc = np.array([-650., 95., -8250.])
        target = loc+np.array([-100., 15., -1300.])
        goal = launch.goal(loc, target)
        np.testing.assert_array_equal(launch.origin, loc)
        self.assertGreater(launch.up@launch.origin, 0.)
        self.assertAlmostEqual(np.linalg.norm(launch.up), 1.)
        self.assertGreater((goal-loc)@launch.up, 0.)
        launch.goal(loc+3.*launch.up, target)
        self.assertAlmostEqual(launch.height, 3.)
        self.assertGreater(np.linalg.norm(loc+3.*launch.up), np.linalg.norm(loc))

    def test_flag_defaults_off_and_only_rotates_candidate_directions(self):
        self.assertFalse(namespace['DoglegNavigator']().ground_lateral)
        up = np.array([1., 2., 3.]); up /= np.linalg.norm(up)
        old, _, _, _ = self.plans(False, up)
        new, _, _, _ = self.plans(True, up)
        self.assertEqual(len(old['eta']), len(new['eta']))
        np.testing.assert_array_equal(old['speed'], new['speed'])
        self.assertFalse(np.allclose(old['goal'], new['goal']))

    def test_disabled_flag_matches_the_frozen_task1_candidate_set(self):
        source = Path(__file__).parent/'score_sweeps'/'dogleg_diagnostic_controller.py'
        previous = load_namespace(source.read_text())['DoglegNavigator'](launch=False)
        current = namespace['DoglegNavigator'](launch=False)
        for planner in (previous, current):
            planner.lane_aligned = True
            planner.control_time = 15.
            planner.launch.up = np.array([.1, .2, .97])
        loc, vel, target = np.array([9000., 0., 0.]), np.array([1., .1, 0.]), np.array([10300., 20., 40.])
        for wide in (False, True):
            old, scores = previous.candidates(loc, vel, target, np.zeros(3), wide)
            new, new_scores = current.candidates(loc, vel, target, np.zeros(3), wide)
            for key in old:
                np.testing.assert_array_equal(old[key], new[key])
            np.testing.assert_array_equal(scores, new_scores)
