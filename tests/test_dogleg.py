"""Finite-burn dogleg geometry, conservative swept checks and execution guards."""
import unittest
from unittest import mock
import numpy as np
from test_controller import namespace


class DoglegTests(unittest.TestCase):
    def planner(self):
        p = namespace['DoglegNavigator'](launch=False)
        p.lane_aligned = True
        p.control_time = 15.
        p.gravity_at = lambda loc: np.array([-.006, 0., 0.])
        p.lane_location = np.array([9000., 0., 0.])
        return p

    def test_minimal_alignment_uses_body_axis_and_handles_antipode(self):
        rotation = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
        directions = np.array([[1., 0., 0.], [0., -1., 0.], [0., 1., 0.]])
        result, angles = namespace['dogleg_rotation'](rotation, directions)
        np.testing.assert_allclose(result[:, 0], directions, atol=1e-12)
        np.testing.assert_allclose(result@result.transpose(0, 2, 1), np.broadcast_to(np.eye(3), result.shape), atol=1e-12)
        np.testing.assert_allclose(angles, [np.pi/2., np.pi, 0.])

    def test_finite_burn_endpoints_and_straight_has_no_bend(self):
        p = self.planner()
        plan, scores = p.candidates(p.lane_location, np.array([.8, 0., 0.]), np.array([10300., 0., 0.]), np.zeros(3))
        valid = np.flatnonzero(np.isfinite(scores))
        self.assertGreater(len(valid), 100)
        selected = {k: v[valid] for k, v in plan.items()}
        end = selected['origin']+namespace['dogleg_positions'](selected, selected['eta'][:, None])[:, 0]
        np.testing.assert_allclose(end, selected['goal'], atol=1e-7)
        np.testing.assert_array_equal(plan['impulses'][:44, 1], 0.)
        self.assertTrue(np.all(plan['starts'][valid, 1] >= plan['bend_time'][valid, None]))

    def test_checks_distant_crossing_and_curvature_between_samples(self):
        p = self.planner()
        plan, scores = p.candidates(p.lane_location, np.array([.8, 0., 0.]), np.array([10300., 0., 0.]), np.zeros(3))
        i = int(np.argmax(scores)); plan = {k: v[i:i+1] for k, v in plan.items()}
        when = plan['eta'][0]*.65
        crossing = namespace['dogleg_positions'](plan, np.array([when]))[0, 0]
        gap = namespace['dogleg_clearances'](plan, crossing[None], np.zeros((1, 3)), np.array([4.]),
            np.zeros(1), np.zeros((1, 3)), np.array([10300., 0., 0.]), np.zeros(3))
        self.assertLess(gap[0], 0.)

    def test_moving_target_endpoint_includes_slew_time(self):
        p = self.planner()
        p.lane_aligned = False
        p.body_rotation = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
        tv = np.array([.1, .03, -.02])
        plan, scores = p.candidates(p.lane_location, np.array([.8, 0., 0.]), np.array([10300., 0., 0.]), tv)
        rows = np.flatnonzero(np.isfinite(scores))
        selected = {k: v[rows] for k, v in plan.items()}
        end = selected['origin']+namespace['dogleg_positions'](selected, selected['eta'][:, None])[:, 0]
        np.testing.assert_allclose(end, selected['goal']+tv*selected['eta'][:, None], atol=1e-7)

    def test_approaching_obstacle_crosses_only_in_future(self):
        p = self.planner()
        plan, scores = p.candidates(p.lane_location, np.array([.8, 0., 0.]), np.array([10300., 0., 0.]), np.zeros(3))
        i = int(np.argmax(scores)); plan = {k: v[i:i+1] for k, v in plan.items()}
        when = plan['eta'][0]*.65
        crossing = namespace['dogleg_positions'](plan, np.array([when]))[0, 0]
        motion = np.array([0., -2., 0.])
        gap = namespace['dogleg_clearances'](plan, (crossing-when*motion)[None], motion[None], np.array([4.]),
            np.zeros(1), np.zeros((1, 3)), np.array([10300., 0., 0.]), np.zeros(3))
        self.assertLess(gap[0], 0.)

    def test_attitude_settles_once_then_does_not_follow_bend(self):
        p = self.planner()
        p.lane_aligning, p.lane_aligned = True, False
        p.lane_axis = np.array([1., 0., 0.])
        attitude = namespace['DoglegAttitude'](p)
        command = attitude.command(0., np.eye(3), np.zeros(3), np.array([0., 1., 0.]), np.array([0., 0., 1.]), np.zeros(3), .02)
        self.assertTrue(p.lane_aligned)
        self.assertFalse(p.lane_aligning)
        np.testing.assert_array_equal(command, 0.)
        p.lane_axis = np.array([0., 1., 0.])
        np.testing.assert_array_equal(attitude.command(1., np.eye(3), np.zeros(3), p.lane_axis, np.array([0., 0., 1.]), np.zeros(3), .02), 0.)

    def test_no_camera_required_and_search_only_on_events(self):
        p = self.planner()
        loc, vel, target = p.lane_location, np.array([.8, 0., 0.]), np.array([10300., 0., 0.])
        p.command(15., loc, vel, target, np.zeros(3), [], camera=None)
        self.assertIsNotNone(p.lane)
        searches = p.lane_searches
        # Replay the selected ephemeris with a settled completed first burn.
        p.lane_stage = 2
        p.axis_done[:] = True
        p.shooting_active = False
        when = p.lane['durations'][0, 0].sum()+1.
        predicted = namespace['dogleg_positions'](p.lane, np.array([when]))[0, 0]
        impulse = p.lane['impulses'][0, 0]@p.lane['rotation'][0]
        velocity = vel+p.lane['gravity'][0]*when+impulse
        p.command(15.+when, loc+predicted, velocity, target, np.zeros(3), [], camera=None)
        self.assertEqual(p.lane_searches, searches)
        np.testing.assert_array_equal(p.linear_command(velocity, .5), np.zeros(3))

    def test_no_safe_lane_falls_back_without_relaxing_margin(self):
        p = self.planner()
        with mock.patch.dict(namespace, {'dogleg_clearances': lambda plan, *args, **kw: np.full(len(plan['eta']), -1.)}):
            self.assertFalse(p.select_lane(15., p.lane_location, np.array([.8, 0., 0.]), np.array([10300., 0., 0.]),
                                          np.zeros(3), (np.zeros((1, 3)),)*2+(np.ones(1), np.ones(1), np.zeros((1, 3)))))
        self.assertEqual(p.lane_log['event'], 'no_safe_lane')

    def test_wider_grid_is_still_considered_if_it_can_score_higher(self):
        p = self.planner()
        candidates = p.candidates
        def scored(*args, **kwargs):
            plan, score = candidates(*args, **kwargs)
            if args[-1]:
                score += 100.
            return plan, score
        with mock.patch.object(p, 'candidates', side_effect=scored), mock.patch.dict(namespace, {
                'dogleg_clearances': lambda plan, *args, **kw: np.full(len(plan['eta']), 10.)}):
            self.assertTrue(p.select_lane(15., p.lane_location, np.array([.8, 0., 0.]), np.array([10300., 0., 0.]),
                                         np.zeros(3), (np.empty((0, 3)),)*2+(np.empty(0), np.empty(0), np.empty((0, 3)))))
        self.assertGreater(p.lane_log['predicted_score'], 100.)

    def test_adaptive_clearance_bound_is_conservative_against_dense_sampling(self):
        p = self.planner()
        plan, scores = p.candidates(p.lane_location, np.array([.8, 0., 0.]), np.array([10300., 0., 0.]), np.zeros(3))
        rows = np.flatnonzero(np.isfinite(scores))[::250][:10]
        plan = {k: v[rows] for k, v in plan.items()}
        rng = np.random.default_rng(71)
        times = plan['eta'][:, None]*np.linspace(0., 1., 1501)
        paths = namespace['dogleg_positions'](plan, times)
        centers = paths[0, [300, 500, 700, 900]]+rng.normal(0., 12., (4, 3))
        speeds = rng.normal(0., .02, (4, 3))
        acceleration = np.tile([-.005, .0001, 0.], (4, 1))
        radii = np.full(4, 5.)
        bound = namespace['dogleg_clearances'](plan, centers, speeds, radii, np.zeros(4), acceleration,
                                               np.array([1e9, 0., 0.]), np.zeros(3))
        truth = centers[None, None]+times[:, :, None, None]*speeds[None, None]+.5*times[:, :, None, None]**2*acceleration[None, None]
        dense = (np.linalg.norm(truth-paths[:, :, None], axis=3)-radii).min(axis=(1, 2))
        self.assertTrue(np.all(bound <= dense+1e-6))

    def test_entry_and_failed_mission_always_zero_commands(self):
        p = self.planner()
        p.arrival_time = 100.
        np.testing.assert_array_equal(p.linear_command(np.zeros(3), .5), np.zeros(3))
        p.arrival_time, p.failed = None, True
        np.testing.assert_array_equal(p.linear_command(np.zeros(3), .5), np.zeros(3))

    def test_collision_after_capture_does_not_block_a_successful_coast(self):
        p = self.planner()
        target = np.array([10300., 0., 0.])
        plan, scores = p.candidates(p.lane_location, np.array([.8, 0., 0.]), target, np.zeros(3))
        i = int(np.argmax(scores[:44])); plan = {k: v[i:i+1] for k, v in plan.items()}
        center = plan['goal'][0]-p.lane_location
        gap = namespace['dogleg_clearances'](plan, center[None], np.zeros((1, 3)), np.array([1.]),
                                               np.zeros(1), np.zeros((1, 3)), target, np.zeros(3))
        self.assertGreater(gap[0], 0.)

    def test_collision_before_capture_remains_blocked(self):
        p = self.planner()
        target = np.array([10300., 0., 0.])
        plan, scores = p.candidates(p.lane_location, np.array([.8, 0., 0.]), target, np.zeros(3))
        i = int(np.argmax(scores[:44])); plan = {k: v[i:i+1] for k, v in plan.items()}
        point = namespace['dogleg_positions'](plan, np.array([plan['eta'][0]-15.]))[0, 0]
        self.assertGreater(np.linalg.norm(p.lane_location+point-target), 55.)
        gap = namespace['dogleg_clearances'](plan, point[None], np.zeros((1, 3)), np.array([4.]),
                                               np.zeros(1), np.zeros((1, 3)), target, np.zeros(3))
        self.assertLess(gap[0], 0.)

    def test_bend_cost_uses_vector_change_and_sequential_axis_fuel(self):
        p = self.planner()
        p.last_damage = 5000.
        p.policy.previous = (14., np.array([.8, 0., 0.]), 20.)
        plan, scores = p.candidates(p.lane_location, np.array([.8, 0., 0.]), np.array([10300., 0., 0.]), np.zeros(3))
        impulses = plan['impulses']
        l1 = np.abs(impulses).sum(axis=(1, 2))
        norms = np.linalg.norm(impulses, axis=2).sum(axis=1)
        rows = np.flatnonzero(np.isfinite(scores) & (np.abs(impulses[:, 1]).sum(axis=1) > .1))
        i = rows[np.argmax((l1-norms)[rows])]
        self.assertGreater(l1[i], norms[i]+.01)
        fuel = namespace['DOGLEG_FUEL_PER_DV']*l1[i]+plan['slew_fuel'][i]
        expected = 100.*(.5+120./(p.control_time+plan['eta'][i])-(20.+fuel)/75.-2.*5000./209200.)
        self.assertAlmostEqual(scores[i], expected)

    def test_slew_is_paid_and_not_repeated_after_alignment(self):
        p = self.planner()
        p.lane_aligned = False
        p.body_rotation = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
        plan, _ = p.candidates(p.lane_location, np.array([.8, 0., 0.]), np.array([10300., 0., 0.]), np.zeros(3))
        self.assertTrue(np.all(plan['slew'] > 20.))
        self.assertTrue(np.all(plan['slew_fuel'] > 0.))
        p.lane_aligned = True
        settled, _ = p.candidates(p.lane_location, np.array([.8, 0., 0.]), np.array([10300., 0., 0.]), np.zeros(3))
        np.testing.assert_array_equal(settled['slew'], 0.)
        np.testing.assert_array_equal(settled['slew_fuel'], 0.)
