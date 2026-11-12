"""Time-dependent chunk planning, guarded launch and pass-through regressions."""
import unittest
import numpy as np
from test_controller import namespace


class ChunkTests(unittest.TestCase):
    def setUp(self):
        self.chunks = namespace['RollingChunks']()
        self.empty = np.empty((0, 3))

    def test_forecast_distinguishes_crossing_from_departing_debris(self):
        location = np.zeros(3)
        path = [np.array([320., 0., 0.])]
        forecast = self.chunks.forecast
        # Now clear, but crosses the flight line near our arrival time.
        crossing = forecast(location, path, 10., 12., np.array([[100., 60., 0.]]),
                            np.array([[0., -6., 0.]]), np.array([10.]))
        # Currently on the line, but has moved safely aside before we arrive.
        departing = forecast(location, path, 10., 12., np.array([[100., 0., 0.]]),
                             np.array([[0., 4., 0.]]), np.array([10.]))
        self.assertLess(crossing, 0.)
        self.assertGreater(departing, 10.)

    def test_chunk_bypasses_predicted_crossing(self):
        # Far enough for a 10 m/s vehicle with finite lateral acceleration to
        # avoid it; an obstacle only 100 m away can already be unavoidable.
        centers, speeds, radii = np.array([[200., 120., 0.]]), np.array([[0., -6., 0.]]), np.array([10.])
        points = self.chunks.plan_chunk(0., np.zeros(3), np.array([10., 0., 0.]),
                                       np.array([2000., 0., 0.]), centers, speeds, radii, 12.)
        self.assertTrue(points)
        self.assertGreater(self.chunks.forecast(np.zeros(3), points, 10., 12., centers, speeds, radii), 2.)
        self.assertGreater(max(np.linalg.norm(p[1:]) for p in points), 10.)

    def test_chunk_is_bounded_and_successor_prepared_before_arrival(self):
        goal = np.array([5000., 0., 0.])
        self.chunks.waypoint(0., np.zeros(3), np.array([8., 0., 0.]), goal,
                             self.empty, self.empty, np.empty(0), 12.)
        self.assertLessEqual(np.linalg.norm(self.chunks.points[-1]), 600.)
        self.chunks.waypoint(1., np.array([150., 0., 0.]), np.array([8., 0., 0.]), goal,
                             self.empty, self.empty, np.empty(0), 12.)
        self.assertTrue(self.chunks.pending)
        self.assertGreater(self.chunks.buffer_m, 600.)
        self.assertLess(self.chunks.points[-1][0], goal[0])

    def test_lookahead_crosses_boundary_without_requesting_a_stop(self):
        self.chunks.points = [np.array([300., 0., 0.])]
        self.chunks.pending = [np.array([900., 0., 0.])]
        self.chunks.anchor = np.zeros(3)
        point = self.chunks.waypoint(0., np.array([220., 0., 0.]), np.array([8., 0., 0.]),
                                     np.array([2000., 0., 0.]), self.empty, self.empty, np.empty(0), 12.)
        self.assertGreater(point[0], 300.)
        self.assertGreater(self.chunks.speed_cap, 8.)

    def test_valid_route_is_retained_with_noise(self):
        centers, radii = np.array([[150., 130., 0.]]), np.array([20.])
        self.chunks.waypoint(0., np.zeros(3), np.array([5., 0., 0.]), np.array([2000., 0., 0.]),
                             centers, np.zeros((1, 3)), radii, 12.)
        first, count = self.chunks.points[0].copy(), self.chunks.replans
        self.chunks.waypoint(3., np.array([15., 0., 0.]), np.array([5., 0., 0.]), np.array([2000., 0., 0.]),
                             centers-np.array([15., -.1, 0.]), np.zeros((1, 3)), radii, 12.)
        np.testing.assert_allclose(self.chunks.points[0], first)
        self.assertEqual(self.chunks.replans, count)

    def test_damage_invalidates_cached_prediction(self):
        planner = namespace['FinalNavigator'](launch=False)
        planner.corridor.points = [np.array([300., 0., 0.])]
        planner.observe_damage(100.)
        self.assertFalse(planner.corridor.points)
        self.assertEqual(planner.corridor.reason, 'damage feedback')

    def test_forecast_uses_supplied_arrival_times(self):
        args = (np.zeros(3), [np.array([320., 0., 0.])], 10., 12.,
                np.array([[100., 60., 0.]]), np.array([[0., -6., 0.]]), np.array([10.]))
        self.assertLess(self.chunks.forecast(*args), 0.)
        self.assertGreater(self.chunks.forecast(*args, times=[48.]), 2.)

    def test_nearby_future_curve_point_is_not_treated_as_passed(self):
        self.chunks.points = [np.array([5., 0., 0.]), np.array([20., 0., 0.]), np.array([400., 0., 0.])]
        self.chunks.anchor = np.zeros(3)
        self.chunks.schedule = np.array([1., 3., 50.])
        self.chunks.waypoint(0., np.zeros(3), np.array([8., 0., 0.]), np.array([5000., 0., 0.]),
                             self.empty, self.empty, np.empty(0), 12.)
        self.assertEqual(self.chunks.points[0][0], 5.)
        self.assertEqual(self.chunks.anchor_time, 0.)

    def test_last_chunk_hands_off_remaining_standoff_gap(self):
        planner = namespace['FinalNavigator'](launch=False)
        planner.corridor.points = [np.zeros(3)]
        planner.corridor.goal = np.array([22., 0., 0.])
        planner.corridor.anchor = np.zeros(3)
        planner.corridor.schedule = np.array([0.])
        accel, mode, _ = planner.command(1., np.zeros(3), np.zeros(3), np.array([122., 0., 0.]),
                                         np.zeros(3), [])
        self.assertEqual(mode, 'BRAKE_SHIP')
        self.assertGreater(accel[0], .1)


class LaunchTests(unittest.TestCase):
    def test_launch_uses_planet_up_and_finishes_after_three_point_five_m(self):
        launch = namespace['GuardedLaunch']()
        origin, target = np.array([8000., 0., 0.]), np.array([9300., 0., 0.])
        np.testing.assert_allclose(launch.goal(origin, target)-origin, [4.5, 0., 0.])
        self.assertIsNotNone(launch.goal(origin+np.array([3., 0., 0.]), target))
        self.assertIsNone(launch.goal(origin+np.array([3.6, 0., 0.]), target))
        self.assertTrue(launch.done)

    def test_launch_still_avoids_an_obstacle_directly_above(self):
        planner = namespace['FinalNavigator']()
        loc = np.array([8000., 0., 0.])
        report = np.array([[15., 0., 0.], [5., 0., 20.], [0., 0., 0.]])
        accel, _, _ = planner.command(0., loc, np.zeros(3), loc+np.array([1300., 0., 0.]),
                                     np.zeros(3), [report])
        self.assertEqual(planner.last_phase, 'LAUNCH')
        self.assertGreaterEqual(accel[0], -1e-9)  # Never escape down into the terrain.
        self.assertLess(accel[0], .01)  # Safety overrides the requested climb.
        self.assertGreater(np.linalg.norm(accel[1:]), .01)


if __name__ == '__main__':
    unittest.main()
