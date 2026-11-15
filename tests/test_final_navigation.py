import unittest
import numpy as np
from test_controller import namespace, Planner


class FinalNavigationTests(unittest.TestCase):
    def test_arrival_latches_goal_and_does_not_retreat_from_new_hull_margin(self):
        planner = Planner()
        location = np.array([49.4, 0., 0.])
        planner.command(0., location, np.zeros(3), np.zeros(3), np.zeros(3), [])
        self.assertIsNotNone(planner.arrival_time)
        offset = planner.hold_offset.copy()
        camera = namespace['CameraPerception']()
        camera.occupied['top'] = (1., [(location.copy(), 100.)])
        planner.command(1., location, np.zeros(3), np.zeros(3), np.zeros(3), [], camera=camera)
        np.testing.assert_allclose(planner.hold_offset, offset)
        np.testing.assert_allclose(planner.previous_velocity, np.zeros(3))

    def test_arrival_not_claimed_outside_zone_and_recovers_drift_after_entry(self):
        planner = Planner()
        planner.command(0., np.array([50.1, 0., 0.]), np.zeros(3), np.zeros(3), np.zeros(3), [])
        self.assertIsNone(planner.arrival_time)
        planner.command(1., np.array([49.4, 0., 0.]), np.zeros(3), np.zeros(3), np.zeros(3), [])
        planner.command(2., np.array([52., 0., 0.]), np.zeros(3), np.zeros(3), np.zeros(3), [])
        self.assertLess(planner.previous_velocity[0], 0.)

    def test_global_route_bypasses_wall_and_all_segments_are_clear(self):
        corridor = namespace['CorridorPlanner']()
        centers = np.array([[200., y, z] for y in (-40., 0., 40.) for z in (-40., 0., 40.)])
        radii = np.full(len(centers), 24.)
        points = corridor.plan(0., np.zeros(3), np.array([500., 0., 0.]), centers, radii)
        self.assertGreater(len(points), 1)
        previous = np.zeros(3)
        for point in points:
            self.assertGreater(namespace['segment_clearance'](previous, point, centers, radii), 0.)
            previous = point

    def test_terminal_does_not_drive_into_known_station_surface(self):
        terminal = namespace['TerminalApproach']()
        loc, target = np.zeros(3), np.array([100., 0., 0.])
        centers, radii = np.array([[62., 0., 0.]]), np.array([20.])
        goal, phase, cap = terminal.goal(0., loc, np.zeros(3), target, np.zeros(3), centers, radii)
        self.assertEqual(phase, 'FINAL_CREEP')
        self.assertAlmostEqual(np.linalg.norm(goal-target), 49., places=5)
        self.assertGreater(namespace['segment_clearance'](loc, goal, centers, radii), 2.)
        self.assertLessEqual(cap, 2.)
        self.assertGreater(np.linalg.norm(goal[1:]), 10.)  # Alternate arrival side.

    def test_ship_braking_guard_limits_inward_command_before_arrival(self):
        planner = Planner()
        accel, mode, _ = planner.command(0., np.zeros(3), np.array([6., 0., 0.]),
                                         np.array([150., 0., 0.]), np.zeros(3), [])
        self.assertEqual(mode, 'BRAKE_SHIP')
        self.assertLess(accel[0], 0.)
        self.assertLessEqual(planner.previous_velocity[0], np.sqrt(2.*.1*(150.-98.))+1e-6)

    def test_missing_station_depth_causes_hold_not_blind_entry(self):
        planner = Planner()
        camera = namespace['CameraPerception']()
        planner.command(0., np.zeros(3), np.zeros(3), np.array([100., 0., 0.]),
                        np.zeros(3), [], camera=camera)
        self.assertEqual(planner.last_phase, 'WAIT_SHIP_CAMERA')
        np.testing.assert_allclose(planner.previous_velocity, np.zeros(3))

    def test_replanning_keeps_equivalent_clear_corridor(self):
        corridor = namespace['CorridorPlanner']()
        goal = np.array([500., 0., 0.])
        centers, radii = np.array([[250., 0., 0.]]), np.array([30.])
        corridor.waypoint(0., np.zeros(3), goal, centers, radii)
        first = corridor.points[0].copy()
        corridor.waypoint(6., np.zeros(3), goal, centers+np.array([0., .1, 0.]), radii)
        np.testing.assert_allclose(corridor.points[0], first)

    def test_fresh_depth_must_cover_station_bearing(self):
        camera = namespace['CameraPerception']()
        # Valid depth image looking along +X; it cannot certify the opposite side.
        camera.ingest('top', 'depth', np.full((64, 96), np.inf), 0., 0.,
                      (np.zeros(3), np.eye(3)), 0., 0.)
        self.assertTrue(camera.observes(.5, [100., 0., 0.]))
        self.assertFalse(camera.observes(.5, [-100., 0., 0.]))
        self.assertFalse(camera.observes(3., [100., 0., 0.]))
        planner = Planner()
        planner.command(.5, np.zeros(3), np.zeros(3), np.array([-100., 0., 0.]),
                        np.zeros(3), [], camera=camera)
        self.assertEqual(planner.last_phase, 'WAIT_SHIP_CAMERA')

    def test_waiting_for_station_depth_turns_body_to_reacquire_ship(self):
        attitude = namespace['FlightAttitude']()
        rotation, omega = np.diag([-1., -1., 1.]), np.zeros(3)
        target = np.array([100., 0., 0.])
        camera = namespace['CameraPerception']()
        for tick in range(600):
            hint = namespace['attitude_reference']('WAIT_SHIP_CAMERA', np.zeros(3), np.zeros(3), target)
            command = attitude.command(tick*.1, rotation, np.zeros(3), hint, [0., 0., 1.], omega, .02)
            delta = .1*omega+.005*command
            omega += .1*command
            angle = np.linalg.norm(delta)
            if angle > 1e-12:
                x, y, z = delta/angle
                skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
                rotation = (np.eye(3)+np.sin(angle)*skew+(1.-np.cos(angle))*(skew@skew)).T@rotation
        camera.ingest('top', 'depth', np.full((64, 96), np.inf), 60., 60.,
                      (np.zeros(3), rotation), 0., 0.)
        self.assertTrue(camera.observes(60., target))
        self.assertGreater(rotation[0]@target/100., .99)


if __name__ == '__main__':
    unittest.main()
