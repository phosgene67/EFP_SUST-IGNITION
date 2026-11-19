"""Finite-burn endpoint accuracy, execution, sensing and moving-track regression."""
import unittest
from unittest import mock
from types import SimpleNamespace
import numpy as np
from controller_loader import load_archived_namespace
from test_camera_fusion import Perception, depth_patch

# This module includes opt-in experiments removed by the Momentum restoration.
namespace = load_archived_namespace()


class ShootingTests(unittest.TestCase):
    def test_short_coast_options_are_scored_only_before_600m_with_momentum(self):
        original = namespace['axis_schedule']
        def delays_for(distance, speed):
            observed = []
            def schedule(*args, **kwargs):
                observed.append(np.unique(args[4]).tolist())
                return original(*args, **kwargs)
            planner = namespace['ShootingNavigator'](launch=False, coast_windows=True)
            with mock.patch.dict(namespace, {'axis_schedule': schedule}):
                planner.command(100., np.array([9000., 0., 0.]), np.array([speed, 0., 0.]),
                                np.array([9000.+distance, 0., 0.]), np.zeros(3), [])
            return observed[0]
        self.assertEqual(delays_for(1300., 2.), [0., 1., 2., 4.])
        self.assertEqual(delays_for(1300., 1.), [0.])
        self.assertEqual(delays_for(1300., -2.), [0.])
        self.assertEqual(delays_for(500., 2.), [0.])
        self.assertFalse(namespace['ShootingNavigator']().coast_windows)

    def test_retained_coast_window_keeps_its_absolute_burn_start(self):
        planner = namespace['ShootingNavigator'](launch=False, coast_windows=True)
        location, velocity = np.array([9000., 0., 0.]), np.array([2., 0., 0.])
        target = np.array([10300., 0., 0.])
        planner.command(100., location, velocity, target, np.zeros(3), [])
        duration, gravity = planner.previous_eta-100., planner.gravity_at(location)
        needed = (planner.terminal.inner_goal-location-velocity*duration-.5*gravity*duration**2)[None]
        dv, _, _, valid = namespace['axis_schedule'](
            needed, np.array([duration]), planner.burn_order[None], planner.axis_accels, np.array([4.]))
        self.assertTrue(valid[0])
        planner.burn_target = velocity+dv[0]
        planner.burn_gravity = gravity
        planner.burn_started = planner.plan_chosen_at = 100.
        planner.axis_done, planner.burn_signs = np.abs(dv[0]) < .02, np.sign(dv[0])
        planner.not_before, planner.shooting_active = 104., True
        for now in (100.5, 101.):
            dt = now-100.
            command, mode, _ = planner.command(now, location+dt*velocity+.5*dt**2*gravity,
                velocity+dt*gravity, target, np.zeros(3), [])
            self.assertEqual(mode, 'WAIT_CLEAR_PATH')
            self.assertEqual(planner.not_before, 104.)
            np.testing.assert_array_equal(command, np.zeros(3))

    def test_arc_return_starts_once_then_coasts_after_delivery(self):
        planner = namespace['ShootingNavigator'](launch=False, exact_arcs=True)
        planner.shooting_active = True
        planner.burn_target = np.array([0., 0., 6.])
        planner.burn_gravity = np.zeros(3)
        planner.burn_signs = np.array([0., 0., 1.])
        planner.axis_done[:] = True
        planner.last_detour = np.array([0., .35, 0.])
        planner.counter_time = 100.
        planner.control_time = 99.
        np.testing.assert_array_equal(planner.linear_command(np.array([0., 0., 6.]), .5), np.zeros(3))
        planner.control_time = 100.
        np.testing.assert_allclose(planner.linear_command(np.array([0., 0., 6.]), .5), [0., -.5, 0.])
        self.assertTrue(planner.return_started)
        np.testing.assert_allclose(planner.burn_target, [0., -.7, 6.])
        planner.control_time = 101.
        np.testing.assert_array_equal(planner.linear_command(np.array([0., -.7, 6.]), .5), np.zeros(3))
        np.testing.assert_allclose(planner.burn_target, [0., -.7, 6.])
        self.assertTrue(planner.axis_done.all())

    def test_arc_return_does_not_interrupt_an_unfinished_first_burn(self):
        planner = namespace['ShootingNavigator'](launch=False, exact_arcs=True)
        planner.shooting_active = True
        planner.burn_target = np.array([0., 0., 6.])
        planner.burn_signs = np.array([0., 0., 1.])
        planner.axis_done = np.array([True, True, False])
        planner.burn_order = np.array([2, 0, 1])
        planner.last_detour = np.array([0., .35, 0.])
        planner.counter_time = planner.control_time = 100.
        np.testing.assert_allclose(planner.linear_command(np.array([0., 0., 5.]), .5), [0., 0., .5])
        self.assertFalse(planner.return_started)

    def test_finite_arc_hits_endpoint_with_nonoverlapping_return_burn(self):
        planner = namespace['ShootingNavigator']()
        for requested in ([1300., 30., -50.], [-75., -90., 1300.]):
            target = np.tile(requested, (6, 1))
            times = np.full(6, 230.)
            return_dv = np.tile([0., -1.4, 0.], (6, 1))
            dv, starts, durations, valid, counter = namespace['arc_schedule'](
                target, times, planner.orders, planner.axis_accels, return_dv)
            self.assertTrue(valid.all())
            self.assertTrue((counter >= durations.sum(axis=1)-1e-7).all())
            return_durations = np.abs(return_dv)/planner.axis_accels[
                np.where(return_dv >= 0., 0, 1), np.arange(3)]
            return_starts = np.zeros_like(return_dv)
            for row, order in enumerate(planner.orders):
                cursor = counter[row]
                for axis in order:
                    return_starts[row, axis], cursor = cursor, cursor+return_durations[row, axis]
                self.assertLess(cursor, times[row])
            endpoint = namespace['scheduled_positions'](
                np.array([230.]), np.zeros(3), np.zeros(3), np.eye(3), dv, starts, durations)
            endpoint += namespace['scheduled_positions'](
                np.array([230.]), np.zeros(3), np.zeros(3), np.eye(3),
                return_dv, return_starts, return_durations)
            np.testing.assert_allclose(endpoint[:, 0], target, atol=1e-7)
        self.assertFalse(planner.exact_arcs)

    def test_impossible_return_arc_is_rejected(self):
        planner = namespace['ShootingNavigator']()
        *_, valid, _ = namespace['arc_schedule'](
            np.array([[1300., 30., -50.]]), np.array([230.]), planner.orders[:1],
            planner.axis_accels, np.array([[0., -100., 0.]]))
        self.assertFalse(valid[0])

    def test_vector_burn_hits_endpoint_without_sequential_side_burns(self):
        planner = namespace['ShootingNavigator']()
        displacement = np.array([[-50., -80., 1000.], [15., -20., -300.]])
        times = np.array([220., 100.])
        dv, starts, durations, valid = namespace['vector_schedule'](
            displacement, times, planner.axis_accels, planner.vector_scales)
        self.assertTrue(valid.all())
        for index, time in enumerate(times):
            endpoint = namespace['scheduled_positions'](
                np.array([time]), np.zeros(3), np.zeros(3), np.eye(3),
                dv[index:index+1], starts[index:index+1], durations[index:index+1])[0, 0]
            np.testing.assert_allclose(endpoint, displacement[index], atol=1e-8)
        self.assertTrue((starts == 0.).all())
        self.assertTrue((np.ptp(durations, axis=1) == 0.).all())
        self.assertFalse(planner.vector_burn)

    def test_vector_burn_holds_direction_until_impulse_is_delivered(self):
        planner = namespace['ShootingNavigator'](launch=False, vector_burn=True)
        planner.shooting_active = planner.vector_active = True
        planner.burn_target = np.array([-.5, -.3, 7.])
        planner.axis_done[:] = False
        planner.vector_direction = planner.burn_target/np.linalg.norm(planner.burn_target)
        direction = planner.vector_direction.copy()
        first = planner.linear_command(np.zeros(3), .5)
        # Small transverse error does not cause alternating axis corrections
        # within an already selected burn; planning revalidates it at 2 Hz.
        second = planner.linear_command(np.array([.02, -.02, 1.]), .5)
        np.testing.assert_allclose(first, .5*direction)
        np.testing.assert_allclose(second, first)
        np.testing.assert_array_equal(planner.linear_command(planner.burn_target, .5), np.zeros(3))
        self.assertTrue(planner.axis_done.all())
        # A completed burn stays off during coast. Only a new scored plan can
        # reopen it when gravity-model error later changes the velocity.
        np.testing.assert_array_equal(planner.linear_command(planner.burn_target-.1*direction, .5), np.zeros(3))

    @staticmethod
    def clear_ship_camera(observed=True):
        return SimpleNamespace(observes=lambda now, target: observed,
            obstacles=lambda *args: (np.empty((0, 3)), np.empty((0, 3)), np.empty(0)),
            last_uncertainty=np.empty(0))

    def test_inertial_capture_preserves_a_safe_short_coast(self):
        planner = namespace['ShootingNavigator'](launch=False, inertial_capture=True)
        location, velocity = np.array([9968., -46., 0.]), np.array([7., 0., 0.])
        target = np.array([10000., 0., 0.])
        when, miss = namespace['ballistic_pass'](
            location, velocity, target, np.zeros(3), planner.gravity_at(location))
        self.assertLess(when, 5.)
        self.assertTrue(42. < miss < 48.)
        command, mode, _ = planner.command(180., location, velocity, target, np.zeros(3), [],
                                          camera=self.clear_ship_camera())
        self.assertEqual(mode, 'IMPULSE_COAST')
        np.testing.assert_array_equal(command, np.zeros(3))
        self.assertEqual(planner.last_phase, 'CAPTURE_PASS')

    def test_inertial_capture_corrects_a_miss_before_closest_approach(self):
        planner = namespace['ShootingNavigator'](launch=False, inertial_capture=True)
        location, velocity = np.array([9800., 0., 0.]), np.array([7., 2.1, 0.])
        target = np.array([10000., 0., 0.])
        gravity = planner.gravity_at(location)
        _, miss = namespace['ballistic_pass'](location, velocity, target, np.zeros(3), gravity)
        self.assertGreater(miss, 50.)
        command, mode, _ = planner.command(180., location, velocity, target, np.zeros(3), [],
                                          camera=self.clear_ship_camera())
        self.assertEqual(mode, 'IMPULSE_BURN')
        self.assertGreater(np.linalg.norm(command), .1)
        delta_v = planner.body_rotation@(planner.burn_target-velocity)
        durations = np.abs(delta_v)/planner.axis_accels[np.where(delta_v >= 0., 0, 1), np.arange(3)]
        starts, cursor = np.zeros(3), 0.
        for axis in planner.burn_order:
            starts[axis], cursor = cursor, cursor+durations[axis]
        endpoint = location+namespace['scheduled_positions'](
            np.array([planner.previous_eta-180.]), velocity, gravity, planner.body_rotation,
            delta_v[None], starts[None], durations[None])[0, 0]
        self.assertLess(np.linalg.norm(endpoint-target), 50.)

    def test_inertial_capture_requires_current_station_observation(self):
        planner = namespace['ShootingNavigator'](launch=False, inertial_capture=True)
        planner.command(180., np.array([9800., 0., 0.]), np.array([7., 1.5, 0.]),
                        np.array([10000., 0., 0.]), np.zeros(3), [],
                        camera=self.clear_ship_camera(False))
        self.assertFalse(planner.shooting_active)
        self.assertEqual(planner.last_phase, 'WAIT_SHIP_CAMERA')
        self.assertFalse(namespace['ShootingNavigator']().inertial_capture)

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

    def test_clean_main_burn_is_opt_in_and_preserves_command_limit(self):
        for enabled in (False, True):
            planner = namespace['ShootingNavigator'](launch=False, clean_main_burn=enabled)
            planner.shooting_active = True
            planner.burn_target = np.array([0., 0., 1.])
            planner.burn_order = np.array([2, 0, 1])
            planner.axis_done = np.array([True, True, False])
            planner.burn_signs = np.array([0., 0., 1.])
            command = planner.linear_command(np.zeros(3), .5)
            self.assertLessEqual(np.linalg.norm(command), .500000001)
            self.assertEqual(command[1], 0.)
            self.assertEqual(command[0] > 0., enabled)
            self.assertGreater(command[2], .499999)
        self.assertFalse(namespace['ShootingNavigator']().clean_main_burn)

    def test_clean_main_burn_does_not_bias_other_axes_or_reverse_burns(self):
        for axis, sign in ((0, 1.), (1, 1.), (2, -1.)):
            planner = namespace['ShootingNavigator'](launch=False, clean_main_burn=True)
            planner.shooting_active = True
            planner.burn_target = sign*np.eye(3)[axis]
            planner.burn_order = np.array([axis]+[a for a in range(3) if a != axis])
            planner.axis_done = np.arange(3) != axis
            planner.burn_signs = sign*np.eye(3)[axis]
            np.testing.assert_array_equal(planner.linear_command(np.zeros(3), .5), .5*sign*np.eye(3)[axis])

    def test_arrival_cuts_active_burn_between_planning_updates(self):
        planner = namespace['ShootingNavigator'](launch=False, clean_main_burn=True)
        planner.shooting_active = True
        self.assertTrue(planner.observe_arrival(250., np.array([9049., 0., 0.]), np.array([-5., 0., 0.]),
                         np.array([9000., 0., 0.]), np.zeros(3)))
        np.testing.assert_array_equal(planner.linear_command(np.array([-5., 0., 0.]), .5), np.zeros(3))

    def test_order_diversity_preserves_all_six_nonzero_burn_orders(self):
        from unittest.mock import patch
        original = namespace['scheduled_positions']
        counts = []
        for enabled in (False, True):
            captured = []
            def record(*args):
                if not captured:
                    captured.append((args[5].copy(), args[6].copy()))
                return original(*args)
            planner = namespace['ShootingNavigator'](launch=False, order_diversity=enabled)
            with patch.dict(namespace, scheduled_positions=record):
                planner.command(10., np.array([9000., 0., 0.]), np.array([1., 0., 0.]),
                                np.array([10305., -75., -90.]), np.zeros(3), [])
            starts, durations = captured[0]
            orders = {tuple(np.argsort(start)) for start, duration in zip(starts, durations)
                      if np.all(duration > 1e-8)}
            counts.append(len(orders))
        self.assertEqual(counts, [2, 6])
        self.assertFalse(namespace['ShootingNavigator']().order_diversity)

    def test_recheck_coast_corrects_a_completed_burn_that_predicts_a_miss(self):
        modes = []
        for enabled in (False, True):
            planner = namespace['ShootingNavigator'](launch=False, recheck_coast=enabled)
            location, velocity = np.array([9000., 0., 0.]), np.array([7., .35, 0.])
            target = np.array([10000., 0., 0.])
            when, miss = namespace['ballistic_pass'](
                location, velocity, target, np.zeros(3), planner.gravity_at(location))
            self.assertGreater(miss, 50.)
            planner.shooting_active = True
            planner.burn_target = velocity.copy()
            planner.axis_done[:] = True
            planner.burn_gravity = planner.gravity_at(location)
            planner.burn_started = planner.plan_chosen_at = 100.
            planner.previous_eta = 100.+when
            planner.policy.previous = (100., velocity.copy(), 35.)
            planner.terminal.side, planner.terminal.tangent = np.array([-1., 0., 0.]), np.array([0., 1., 0.])
            command, mode, _ = planner.command(100., location, velocity, target, np.zeros(3), [])
            modes.append(mode)
            self.assertEqual(np.linalg.norm(command) > .1, enabled)
        self.assertEqual(modes, ['IMPULSE_COAST', 'IMPULSE_BURN'])
        self.assertFalse(namespace['ShootingNavigator']().recheck_coast)


if __name__ == '__main__':
    unittest.main()
