"""Finite-burn endpoint accuracy, execution, sensing and moving-track regression."""
import unittest
import numpy as np
from test_controller import namespace
from test_camera_fusion import Perception, depth_patch


class ShootingTests(unittest.TestCase):
    def test_clutter_ahead_selects_a_bend_while_retaining_forward_momentum(self):
        loc = np.array([9000., 0., 0.])
        vel = np.array([3., 0., 0.])
        target = loc+[1305., 0., 0.]
        column = [np.array([[x, 0., 0.], [x-12., -3., 24.], [100., 0., 0.]])
                  for x in np.arange(225., 376., 30.)]
        for reports, bend in (([], False), (column, True)):
            planner = namespace['ShootingNavigator'](launch=False)
            _, mode, clearance = planner.command(100., loc, vel, target, np.zeros(3), reports)
            self.assertEqual(mode, 'IMPULSE_BURN')
            self.assertGreater(clearance, .5)
            self.assertEqual(np.linalg.norm(planner.last_detour) > .01, bend)
            self.assertGreaterEqual(planner.burn_target[0], vel[0]-.04)

    def test_swept_corridor_counts_a_crossing_between_samples_and_ignores_it_after_arrival(self):
        times = np.array([20., 40.])
        paths = np.array([[[0., 0., 0.], [100., 0., 0.]]]*3)
        paths[1, :, 2] += 50.
        cost = namespace['corridor_cost'](times, paths, np.array([[50., -120., 0.]]),
            np.array([[0., 4., 0.]]), np.zeros((1, 3)), np.array([8.]), np.array([60., 60., 25.]))
        self.assertGreater(cost[0], 0.)
        np.testing.assert_array_equal(cost[1:], [0., 0.])

    def test_empty_corridor_produces_an_active_finite_burn(self):
        planner = namespace['ShootingNavigator'](launch=False)
        command, mode, clearance = planner.command(10., np.array([9000., 0., 0.]),
            np.array([1., 0., 0.]), np.array([10305., 0., 0.]), np.zeros(3), [])
        self.assertEqual(mode, 'IMPULSE_BURN')
        self.assertGreater(np.linalg.norm(command), .1)
        self.assertTrue(planner.shooting_active)
        self.assertGreater(planner.previous_eta, 10.)
        self.assertTrue(np.isfinite(planner.policy.estimated_score))
        self.assertGreater(clearance, 1.)

    def test_finite_burn_schedules_hit_endpoint_for_each_order_and_sign(self):
        planner = namespace['ShootingNavigator'](launch=False)
        for requested in ([100., 40., -20.], [-100., -40., 20.]):
            displacement = np.tile(requested, (6, 1))
            impulse, starts, durations, valid = namespace['axis_schedule'](
                displacement, np.full(6, 80.), planner.orders, planner.axis_accels)
            self.assertTrue(valid.all())
            result = namespace['scheduled_positions'](np.array([80.]), np.zeros(3), np.zeros(3),
                                                      np.eye(3), impulse, starts, durations)
            np.testing.assert_allclose(result[:, 0], displacement, atol=1e-9)
            self.assertTrue((durations.sum(axis=1) < 80.).all())

    def test_unreachable_deadline_is_rejected(self):
        planner = namespace['ShootingNavigator'](launch=False)
        *_, valid = namespace['axis_schedule'](np.array([[100., 0., 0.]]), np.array([1.]),
                                               np.array([[0, 1, 2]]), planner.axis_accels)
        self.assertFalse(valid[0])

    def test_delayed_burns_hit_endpoint_without_early_impulse(self):
        planner = namespace['ShootingNavigator'](launch=False)
        impulse, starts, durations, valid = namespace['axis_schedule'](
            np.array([[100., -30., 20.]]), np.array([90.]),
            np.array([[2, 0, 1]]), planner.axis_accels, np.array([10.]))
        self.assertTrue(valid[0])
        predicted = namespace['scheduled_positions'](np.array([5., 90.]), np.zeros(3), np.zeros(3),
                                                     np.eye(3), impulse, starts, durations)
        np.testing.assert_allclose(predicted[0, 0], np.zeros(3), atol=1e-9)
        np.testing.assert_allclose(predicted[0, 1], [100., -30., 20.], atol=1e-9)

    def test_uncertainty_risk_increases_with_uncertainty_and_speed(self):
        risk = namespace['encounter_risk']
        near_p, energy = risk(2., 2., 3.)
        far_p, _ = risk(8., 2., 3.)
        uncertain_p, fast_energy = risk(8., 6., 6.)
        self.assertGreater(near_p, far_p)
        self.assertGreater(uncertain_p, far_p)
        self.assertAlmostEqual(fast_energy, 4.*energy)

    def test_rotated_schedule_hits_moving_gravity_affected_endpoint(self):
        planner = namespace['ShootingNavigator'](launch=False)
        rotation = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
        velocity, gravity = np.array([2., 0., 0.]), np.array([-.01, 0., 0.])
        desired, time = np.array([300., 30., -10.]), 100.
        needed = rotation@(desired-time*velocity-.5*time*time*gravity)
        impulse, starts, durations, valid = namespace['axis_schedule'](needed[None], np.array([time]),
            np.array([[0, 1, 2]]), planner.axis_accels)
        self.assertTrue(valid[0])
        predicted = namespace['scheduled_positions'](np.array([time]), velocity, gravity, rotation,
                                                       impulse, starts, durations)
        np.testing.assert_allclose(predicted[0, 0], desired, atol=1e-9)

    def test_execution_advances_to_next_axis_without_waiting_for_replan(self):
        planner = namespace['ShootingNavigator'](launch=False)
        planner.shooting_active = True
        planner.burn_target = np.array([.1, .2, 0.])
        planner.burn_order = np.array([0, 1, 2])
        planner.axis_done = np.array([False, False, True])
        planner.burn_signs = np.array([1., 1., 0.])
        np.testing.assert_allclose(planner.linear_command(np.zeros(3), .5), [.5, 0., 0.])
        np.testing.assert_allclose(planner.linear_command(np.array([.1, 0., 0.]), .5), [0., .5, 0.])
        planner.burn_gravity, planner.control_time = np.array([0., 0., -.01]), 2.
        np.testing.assert_allclose(planner.linear_command(np.array([.1, .2, -.02]), .5), np.zeros(3))

    def test_completed_axis_does_not_reverse_after_small_overshoot(self):
        planner = namespace['ShootingNavigator'](launch=False)
        planner.shooting_active = True
        planner.burn_target = np.array([.1, 0., 0.])
        planner.burn_order = np.array([0, 1, 2])
        planner.axis_done = np.array([False, True, True])
        planner.burn_signs = np.array([1., 0., 0.])
        np.testing.assert_array_equal(planner.linear_command(np.array([.101, 0., 0.]), .5), np.zeros(3))
        np.testing.assert_array_equal(planner.linear_command(np.array([.095, 0., 0.]), .5), np.zeros(3))

    def test_waiting_burn_never_fires_before_its_start(self):
        planner = namespace['ShootingNavigator'](launch=False)
        planner.shooting_active = True
        planner.burn_target = np.array([.1, 0., 0.])
        planner.burn_order = np.array([0, 1, 2])
        planner.axis_done = np.array([False, True, True])
        planner.burn_signs = np.array([1., 0., 0.])
        planner.not_before, planner.control_time = 10., 9.
        np.testing.assert_array_equal(planner.linear_command(np.zeros(3), .5), np.zeros(3))
        planner.control_time = 10.
        np.testing.assert_array_equal(planner.linear_command(np.zeros(3), .5), [.5, 0., 0.])

    def test_camera_coverage_uses_actual_field_without_accepting_outside_bearing(self):
        perception = Perception()
        perception.ingest('top', 'depth', depth_patch(100.), 0., 0., (np.zeros(3), np.eye(3)), 0., 0.)
        self.assertTrue(perception.observes(0., np.array([101.27, 0., 74.9])))
        self.assertFalse(perception.observes(0., np.array([101.27, 0., 94.])))
        self.assertFalse(perception.observes(3., np.array([101.27, 0., 74.9])))

    def test_broad_distant_depth_surface_does_not_create_huge_spheres(self):
        perception = Perception()
        depth = np.full((64, 96), 10000.)
        self.assertTrue(perception.ingest('top', 'depth', depth, 0., 0.,
                                         (np.zeros(3), np.eye(3)), 0., 0.))
        volumes = perception.occupied['top'][1]
        self.assertEqual(len(volumes), 384)
        self.assertLess(max(radius for _, radius in volumes), 13.)

    def test_gravity_corrected_track_predicts_stale_report_and_keeps_margin(self):
        planner = namespace['ShootingNavigator'](launch=False)
        location, initial = np.array([9000., 0., 0.]), np.array([9100., 20., 0.])
        velocity = np.array([.1, .4, 0.])
        gravity = -711000.*initial/np.linalg.norm(initial)**3
        report = None
        for stamp in np.arange(0., 29., 4.):
            position = initial+stamp*velocity+.5*stamp*stamp*gravity
            relative = position-location
            radial = (velocity+stamp*gravity)@relative/np.linalg.norm(relative)
            report = np.array([relative, [np.linalg.norm(relative)-4., radial, 8.], [stamp, 0., 0.]])
            planner.obstacles(stamp, location, np.zeros(3), [report])
        centers, speeds, radii = planner.obstacles(31., location, np.zeros(3), [report])
        expected = initial+31.*velocity+.5*31.*31.*gravity-location
        np.testing.assert_allclose(centers[0], expected, atol=.08)
        np.testing.assert_allclose(speeds[0], velocity+31.*gravity, atol=.03)
        self.assertLess(planner.radar_forecast_rates[0], .4)
        self.assertGreater(radii[0], 4.+3.+4.)

    def test_coast_model_learns_drift_but_never_from_a_powered_burn(self):
        model = namespace['CoastDynamics']()
        location = np.array([9000., 0., 0.])
        nominal = -711000.*location/np.linalg.norm(location)**3
        bias = np.array([.001, -.002, .001])
        for stamp in np.arange(0., 25., .1):
            model.observe(stamp, location, stamp*(nominal+bias), False)
        np.testing.assert_allclose(model.residual, bias, atol=2e-5)
        old = model.residual.copy()
        for stamp in np.arange(25., 30., .1):
            model.observe(stamp, location, stamp*np.array([.1, 0., 0.]), True)
        np.testing.assert_array_equal(model.residual, old)

    def test_unmodeled_contact_does_not_corrupt_coast_learning(self):
        model = namespace['CoastDynamics']()
        model.observe(0., np.array([9000., 0., 0.]), np.zeros(3), False)
        model.observe(2., np.array([9000., 0., 0.]), np.array([1., 0., 0.]), False, 5000.)
        np.testing.assert_array_equal(model.residual, np.zeros(3))
        self.assertEqual(model.samples, [])

    def test_fast_arrival_detection_does_not_wait_for_a_plan(self):
        planner = namespace['ShootingNavigator'](launch=False)
        planner.shooting_active = True
        loc, target = np.array([9000., 0., 0.]), np.array([9049.9, 0., 0.])
        self.assertTrue(planner.observe_arrival(10.1, loc, np.array([5., 0., 0.]), target, np.zeros(3)))
        self.assertEqual(planner.arrival_time, 10.1)
        self.assertTrue(planner.terminal.entered)
        self.assertFalse(planner.shooting_active)
        self.assertFalse(planner.observe_arrival(10.2, loc, np.zeros(3), target, np.zeros(3)))

    def test_damage_failure_has_priority_over_arrival(self):
        planner = namespace['ShootingNavigator'](launch=False)
        planner.observe_damage(209201.)
        self.assertFalse(planner.observe_arrival(10., np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)))
        self.assertIsNone(planner.arrival_time)

    def test_official_distance_can_capture_entry_despite_actor_origin_offset(self):
        planner = namespace['ShootingNavigator'](launch=False)
        location, target = np.array([9000., 0., 0.]), np.array([9050.1, 0., 0.])
        self.assertTrue(planner.observe_arrival(10., location, np.array([5., 0., 0.]),
                                               target, np.zeros(3), 49.9))
        self.assertTrue(planner.terminal.entered)
        self.assertTrue(np.isfinite(planner.terminal.exit_goal).all())

    def test_launch_transfers_to_flight_without_stopping_at_clearance_height(self):
        planner = namespace['ShootingNavigator']()
        start = np.array([9000., 0., 0.])
        target = np.array([10300., 0., 0.])
        self.assertIsNotNone(planner.launch.goal(start, target))
        self.assertIsNone(planner.launch.goal(start+np.array([2.6, 0., 0.]), target))
        self.assertTrue(planner.launch.done)

    def test_clear_successful_ballistic_pass_uses_no_thrust(self):
        planner = namespace['ShootingNavigator'](launch=False)
        location, velocity = np.array([9000., 0., 0.]), np.array([7., .3, 0.])
        planner.policy.previous = (100., velocity.copy(), 50.)
        command, mode, _ = planner.command(100., location, velocity, np.array([10000., 0., 0.]),
                                            np.zeros(3), [])
        self.assertEqual(mode, 'IMPULSE_COAST')
        np.testing.assert_array_equal(command, np.zeros(3))

    def completed_burn(self, sideways_speed):
        planner = namespace['ShootingNavigator'](launch=False)
        planner.launch.done = True
        location, velocity = np.array([9000., 0., 0.]), np.array([7., sideways_speed, 0.])
        target = np.array([10000., 0., 0.])
        when, miss = namespace['ballistic_pass'](location, velocity, target, np.zeros(3),
                                                planner.gravity_at(location))
        planner.shooting_active = True
        planner.burn_target = velocity.copy()
        planner.axis_done[:] = True
        planner.burn_gravity = planner.gravity_at(location)
        planner.burn_started = planner.plan_chosen_at = 100.
        planner.previous_eta = 100.+when
        planner.policy.previous = (100., velocity.copy(), 35.)
        planner.terminal.side, planner.terminal.tangent = np.array([-1., 0., 0.]), np.array([0., 1., 0.])
        return planner, location, velocity, target, miss

    def test_completed_burn_cannot_retain_a_coast_that_misses_the_zone(self):
        planner, location, velocity, target, miss = self.completed_burn(.35)
        self.assertGreater(miss, 50.)
        command, mode, _ = planner.command(100., location, velocity, target, np.zeros(3), [])
        self.assertEqual(mode, 'IMPULSE_BURN')
        self.assertGreater(np.linalg.norm(command), .1)
        # The correction starts well before the capture handoff, preserving
        # the opportunity to make a small change instead of a late hard turn.
        self.assertGreater(np.linalg.norm(target-location), 280.)

    def test_retention_does_not_preserve_a_pass_too_close_to_ship_centre(self):
        planner, location, velocity, target, miss = self.completed_burn(.1)
        self.assertLess(miss, 42.)
        command, mode, _ = planner.command(100., location, velocity, target, np.zeros(3), [])
        self.assertEqual(mode, 'IMPULSE_BURN')
        self.assertGreater(np.linalg.norm(command), .1)

    def test_completed_safe_burn_keeps_its_unpowered_pass(self):
        planner, location, velocity, target, miss = self.completed_burn(.28)
        self.assertGreater(miss, 42.)
        self.assertLess(miss, 48.)
        command, mode, _ = planner.command(100., location, velocity, target, np.zeros(3), [])
        self.assertEqual(mode, 'IMPULSE_COAST')
        np.testing.assert_array_equal(command, np.zeros(3))

    def test_camera_reach_respects_the_fixed_gimbal_limits(self):
        aim = namespace['camera_can_aim']
        self.assertTrue(aim(np.eye(3), np.array([100., 0., 0.])))
        self.assertFalse(aim(np.eye(3), np.array([-100., 0., 0.])))
        self.assertFalse(aim(np.eye(3), np.array([0., 0., 100.])))
        self.assertTrue(aim(np.eye(3), np.array([20., 0., 100.])))

    def test_capture_side_stays_visible_without_rotating_the_vehicle(self):
        planner = namespace['ShootingNavigator'](launch=False)
        planner.launch.done = True
        # Nose is sideways during a steep climb. The cheapest unconstrained
        # passage puts the ship behind its gimbals during final approach.
        rotation = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
        planner.body_rotation = rotation.copy()
        location, target = np.array([9000., 0., 0.]), np.array([10300., 0., 0.])
        command, mode, _ = planner.command(100., location, np.array([4., .15, 0.]),
                                           target, np.zeros(3), [])
        self.assertEqual(mode, 'IMPULSE_BURN')
        self.assertGreater(np.linalg.norm(command), .1)
        heading = planner.terminal.inner_goal-location
        heading /= np.linalg.norm(heading)
        for distance in (150., 80.):
            approach = planner.terminal.inner_goal-np.sqrt(distance**2-47.**2)*heading
            self.assertTrue(namespace['camera_can_aim'](rotation, target-approach))
        np.testing.assert_array_equal(planner.body_rotation, rotation)

    def test_free_coast_cannot_bypass_the_camera_reach_check(self):
        planner, location, velocity, target, miss = self.completed_burn(.28)
        planner.body_rotation = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
        self.assertGreater(miss, 42.)
        self.assertLess(miss, 48.)
        command, mode, _ = planner.command(100., location, velocity, target, np.zeros(3), [])
        self.assertEqual(mode, 'IMPULSE_BURN')
        self.assertGreater(np.linalg.norm(command), .1)

    def test_completed_major_burn_can_observe_drift_before_more_trimming(self):
        planner, location, velocity, target, _ = self.completed_burn(.35)
        planner.coast_probe_armed = True
        command, mode, _ = planner.command(100., location, velocity, target, np.zeros(3), [])
        self.assertEqual(mode, 'IMPULSE_COAST')
        np.testing.assert_array_equal(command, np.zeros(3))
        self.assertEqual(planner.coast_probe_until, 103.5)
        self.assertFalse(planner.coast_probe_armed)

    def test_drift_observation_coast_cannot_ignore_incoming_debris(self):
        planner, location, velocity, target, _ = self.completed_burn(.35)
        planner.coast_probe_armed = True
        radar = [np.array([[30., 0., 0.], [20., -7., 20.], [100., 0., 0.]])]
        command, mode, _ = planner.command(100., location, velocity, target, np.zeros(3), radar)
        self.assertNotEqual(mode, 'IMPULSE_COAST')
        self.assertGreater(np.linalg.norm(command), .01)

    def test_initial_drift_measurement_uses_a_safe_climb_only(self):
        loc = np.array([9004., 0., 0.])
        vel = np.array([.7, 0., 0.])
        incoming = [np.array([[19., 0., 0.], [12., -2.7, 14.], [12., 0., 0.]])]
        for reports in ([], incoming):
            planner = namespace['ShootingNavigator'](launch=False)
            planner.launch.up = np.array([1., 0., 0.])
            planner.launch.origin = loc-4.*planner.launch.up
            command, mode, _ = planner.command(12., loc, vel, loc+[1300., 0., 0.], np.zeros(3), reports)
            self.assertEqual(mode == 'IMPULSE_COAST', not reports)
            self.assertEqual(bool(np.linalg.norm(command) > .01), bool(reports))

    def test_fuel_budget_prices_new_spending_and_does_not_penalize_coasting(self):
        price = namespace['fuel_budget_penalty']
        target = namespace['FUEL_PLAN_TARGET']
        self.assertEqual(price(45., 0.), 0.)
        self.assertEqual(price(20., 5.), 0.)
        self.assertGreater(price(target, 4.), price(target, 2.))
        self.assertGreater(price(target+5., 2.), price(target, 2.))

    def test_quiet_observation_preserves_a_pending_return_stage(self):
        planner, location, velocity, target, _ = self.completed_burn(.35)
        planner.coast_probe_armed = True
        planner.last_detour = np.array([0., .15, 0.])
        planner.counter_time = 140.
        command, mode, _ = planner.command(100., location, velocity, target, np.zeros(3), [])
        self.assertEqual(mode, 'IMPULSE_COAST')
        np.testing.assert_array_equal(command, np.zeros(3))
        np.testing.assert_array_equal(planner.last_detour, [0., .15, 0.])
        self.assertEqual(planner.counter_time, 140.)

    def test_due_return_stage_is_not_left_as_a_future_unexecuted_burn(self):
        planner, location, velocity, target, _ = self.completed_burn(.35)
        planner.last_detour = np.array([0., .15, 0.])
        planner.counter_time = 99.
        command, mode, _ = planner.command(100., location, velocity, target, np.zeros(3), [])
        self.assertEqual(mode, 'IMPULSE_BURN')
        self.assertGreater(np.linalg.norm(command), .01)
        self.assertFalse(np.array_equal(planner.last_detour, [0., .15, 0.]) and planner.counter_time <= 100.)

    def test_capture_handoff_keeps_the_ship_viewable_near_the_success_boundary(self):
        approach = namespace['CaptureApproach']()
        approach.body_rotation = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
        location, target = np.array([9000., 0., 0.]), np.array([9300., 0., 0.])
        approach.side, approach.tangent = np.array([-1., 0., 0.]), np.array([0., 1., 0.])
        end, _, _ = approach.goal(100., location, np.array([7., .3, 0.]), target,
                                   np.zeros(3), np.empty((0, 3)), np.empty(0))
        heading = (end-approach.inner_goal)/np.linalg.norm(end-approach.inner_goal)
        for radius in (150., 80., 52.):
            bearing = target-approach.inner_goal+np.sqrt(radius**2-47.**2)*heading
            self.assertTrue(namespace['camera_can_aim'](approach.body_rotation, bearing))

    def test_small_planned_course_correction_is_actually_executed(self):
        planner = namespace['ShootingNavigator'](launch=False)
        planner.launch.done = True
        planner.command(100., np.array([9000., 0., 0.]), np.array([7., .35, 0.]),
                        np.array([10000., 0., 0.]), np.zeros(3), [])
        planner.shooting_active = True
        planner.burn_target = np.array([.01, 0., 0.])
        planner.burn_gravity = np.zeros(3)
        planner.burn_started = planner.control_time
        planner.burn_order = np.array([0, 1, 2])
        planner.axis_done = np.array([False, True, True])
        planner.burn_signs = np.array([1., 0., 0.])
        self.assertGreater(planner.linear_command(np.zeros(3), .5)[0], .1)


if __name__ == '__main__':
    unittest.main()
