"""Momentum regression checks for fly-through completion and late-burn pricing."""
import unittest
import numpy as np
from types import SimpleNamespace

from test_controller import namespace
import test_sim_adapter


class MomentumCaptureTests(unittest.TestCase):
    def test_active_momentum_finite_burn_still_meets_the_planned_endpoint(self):
        planner = namespace['ShootingNavigator']()
        targets = np.tile([1300., -35., 55.], (6, 1))
        times = np.full(6, 230.)
        impulses, starts, durations, valid = namespace['axis_schedule'](
            targets, times, planner.orders, planner.axis_accels)
        self.assertTrue(valid.all())
        endpoint = namespace['scheduled_positions'](np.array([230.]), np.zeros(3),
            np.zeros(3), np.eye(3), impulses, starts, durations)
        np.testing.assert_allclose(endpoint[:, 0], targets, atol=1e-7)

    def test_fast_sphere_crossing_turns_off_an_unfinished_burn(self):
        planner = namespace['ShootingNavigator'](launch=False)
        planner.shooting_active = True
        planner.axis_done[:] = False
        planner.burn_target = np.array([15., 0., 0.])
        velocity = np.array([8., .5, 0.])
        location, target = np.array([9000., 0., 0.]), np.array([9049.9, 0., 0.])
        self.assertTrue(planner.observe_arrival(150., location, velocity, target, np.zeros(3)))
        np.testing.assert_array_equal(planner.linear_command(velocity, .5), np.zeros(3))
        self.assertFalse(planner.shooting_active)
        self.assertTrue(planner.coasting)
        # Leaving the sphere must not restart jets.
        command, mode, _ = planner.command(155., location+100., velocity, target, np.zeros(3), [])
        self.assertEqual(mode, 'MISSION_COMPLETE_COAST')
        np.testing.assert_array_equal(command, np.zeros(3))
        np.testing.assert_array_equal(planner.linear_command(-velocity, .5), np.zeros(3))
        self.assertEqual(planner.arrival_time, 150.)

    def test_near_miss_does_not_switch_off_guidance(self):
        planner = namespace['ShootingNavigator'](launch=False)
        self.assertFalse(planner.observe_arrival(150., np.array([9000., 0., 0.]),
            np.array([8., 0., 0.]), np.array([9050.1, 0., 0.]), np.zeros(3)))
        self.assertIsNone(planner.arrival_time)

    def test_official_distance_controls_short_crossing(self):
        planner = namespace['ShootingNavigator'](launch=False)
        self.assertTrue(planner.observe_arrival(150., np.array([9000., 0., 0.]),
            np.array([8., 0., 0.]), np.array([9060., 0., 0.]), np.zeros(3), score_distance=49.9))
        np.testing.assert_array_equal(planner.linear_command(np.zeros(3), .5), np.zeros(3))

    def test_a_profitable_late_impulse_is_allowed(self):
        policy = namespace['EconomyPolicy']()
        policy.fuel_per_dv, policy.acceleration = 4.32, .175
        policy.choose(450., 3., 100., 25., 0., .008)
        self.assertGreater(policy.speed, 3.)
        self.assertGreater(policy.burn_seconds, 0.)

    def test_expensive_late_boost_is_rejected_in_favour_of_inertia(self):
        policy = namespace['EconomyPolicy']()
        policy.fuel_per_dv, policy.acceleration = 4.32, .175
        policy.choose(450., 8., 150., 35., 0., .008)
        self.assertEqual(policy.speed, 8.)
        self.assertEqual(policy.burn_seconds, 0.)

    def test_actual_shooting_planner_prices_boost_versus_coast_near_500m(self):
        camera = SimpleNamespace(observes=lambda *a: True, last_uncertainty=np.empty(0),
            obstacles=lambda *a: (np.empty((0, 3)), np.empty((0, 3)), np.empty(0)))
        for elapsed, speed, fuel, expected in ((100., 3., 25., 'IMPULSE_BURN'),
                                              (150., 8., 35., 'IMPULSE_COAST')):
            with self.subTest(speed=speed):
                planner = namespace['ShootingNavigator'](launch=False)
                planner.policy.observe(elapsed, [speed, 0., 0.], fuel)
                command, mode, _ = planner.command(elapsed, np.array([9000., 0., 0.]),
                    np.array([speed, 0., 0.]), np.array([9450., 47., 0.]), np.zeros(3), [], camera=camera)
                self.assertEqual(mode, expected)
                self.assertEqual(np.linalg.norm(command) > 0., expected == 'IMPULSE_BURN')
                self.assertTrue(np.isfinite(planner.plan_score))


class MomentumCaptureAdapterTests(unittest.TestCase):
    run_adapter = test_sim_adapter.AdapterTests.run_adapter

    def test_arrival_stops_translation_and_spin_damping_without_radar(self):
        commands = self.run_adapter([], target_position=[8049.9, 0., 0.], angular_velocity=(.1, 0., 0.))
        self.assertIsNotNone(self.runtime['planner'].arrival_time)
        for _, command in commands:
            np.testing.assert_array_equal(command, np.zeros(3))

    def test_official_frame_entry_stops_jets_after_clock_reset(self):
        commands = self.run_adapter([], target_position=[8050.1, 0., 0.],
            scoring_distance=49.9, reset_clock=True, angular_velocity=(.1, 0., 0.))
        self.assertIsNotNone(self.runtime['planner'].arrival_time)
        for _, command in commands:
            np.testing.assert_array_equal(command, np.zeros(3))


if __name__ == '__main__':
    unittest.main()
