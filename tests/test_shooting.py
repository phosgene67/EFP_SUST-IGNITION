"""Finite-burn endpoint accuracy, execution, sensing and moving-track regression."""
import unittest
import numpy as np
from test_controller import namespace
from test_camera_fusion import Perception, depth_patch


class ShootingTests(unittest.TestCase):
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


if __name__ == '__main__':
    unittest.main()
