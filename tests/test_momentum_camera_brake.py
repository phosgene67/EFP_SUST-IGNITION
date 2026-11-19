"""The original Momentum approach must not stop solely for a missing camera."""
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
from controller_loader import load_namespace


ROOT = Path(__file__).resolve().parents[1]
ACTIVE = load_namespace((ROOT / 'Controller.py').read_text())
ORIGINAL = load_namespace((ROOT / 'temp_controller' / 'score_momentum_controller.py').read_text())


class MomentumCameraBrakeTests(unittest.TestCase):
    def test_missing_camera_no_longer_requests_a_station_stop_at_200m(self):
        empty = np.empty((0, 3))
        args = (100., np.array([200., 0., 0.]), np.array([-6., 0., 0.]),
                np.zeros(3), np.zeros(3), empty, np.empty(0))
        old = ORIGINAL['CaptureApproach']()
        self.assertEqual(old.goal(*args)[1], 'WAIT_SHIP_CAMERA')
        for distance in (200., 125., 65., 50.1):
            with self.subTest(distance=distance):
                approach = ACTIVE['CaptureApproach']()
                goal, phase, cap = approach.goal(args[0], np.array([distance, 0., 0.]), *args[2:])
                self.assertEqual(phase, 'CAPTURE_PASS')
                self.assertEqual(cap, 10.)
                delta = goal-np.array([distance, 0., 0.])
                closest = np.array([distance, 0., 0.])-delta*(distance*delta[0])/(delta@delta)
                self.assertAlmostEqual(np.linalg.norm(closest), 47.)

    def test_distant_and_camera_observed_guidance_match_original(self):
        empty = np.empty((0, 3))
        for distance, ready in ((250., False), (500., False), (200., True), (65., True)):
            with self.subTest(distance=distance, ready=ready):
                results = []
                for ns in (ORIGINAL, ACTIVE):
                    approach = ns['CaptureApproach']()
                    approach.camera_ready = ready
                    results.append(approach.goal(100., np.array([distance, 0., 0.]),
                        np.array([-6., 0., 0.]), np.zeros(3), np.zeros(3), empty, np.empty(0)))
                np.testing.assert_array_equal(results[0][0], results[1][0])
                self.assertEqual(results[0][1:], results[1][1:])

    def test_known_blocking_obstacle_is_still_respected(self):
        approach = ACTIVE['CaptureApproach']()
        _, phase, _ = approach.goal(100., np.array([200., 0., 0.]), np.array([-6., 0., 0.]),
            np.zeros(3), np.zeros(3), np.array([[-200., 0., 0.]]), np.array([100.]))
        self.assertEqual(phase, 'OBSERVE_CAPTURE')

    def test_shooting_handoff_can_coast_into_sphere_without_station_camera(self):
        camera = SimpleNamespace(observes=lambda *args: False,
            obstacles=lambda *args: (np.empty((0, 3)), np.empty((0, 3)), np.empty(0)),
            last_uncertainty=np.empty(0))
        outcomes = []
        for ns in (ORIGINAL, ACTIVE):
            planner = ns['ShootingNavigator'](launch=False)
            location, velocity = np.array([9000., 0., 0.]), np.array([6., 0., 0.])
            target = np.array([9200., 47., 0.])
            command, mode, _ = planner.command(200., location, velocity, target, np.zeros(3), [], camera=camera)
            outcomes.append((planner, command, mode))
        self.assertEqual(outcomes[0][2], 'WAIT_SHIP_CAMERA')
        planner, command, mode = outcomes[1]
        self.assertEqual(planner.last_phase, 'CAPTURE_PASS')
        self.assertEqual(mode, 'COAST')
        np.testing.assert_array_equal(command, np.zeros(3))
        when, miss = ACTIVE['ballistic_pass'](np.array([9000., 0., 0.]), np.array([6., 0., 0.]),
            np.array([9200., 47., 0.]), np.zeros(3), planner.gravity_at(np.array([9000., 0., 0.])))
        self.assertTrue(np.isfinite(when))
        self.assertLess(miss, 50.)


if __name__ == '__main__':
    unittest.main()
