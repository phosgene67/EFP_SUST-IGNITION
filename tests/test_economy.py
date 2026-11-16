"""Scoring arithmetic, impulse economy and success-boundary capture checks."""
import unittest
from types import SimpleNamespace
import numpy as np
from test_controller import namespace


class EconomyTests(unittest.TestCase):
    def test_free_attitude_does_not_spend_fuel_on_nose_or_upside_down_view(self):
        attitude = namespace['FreeAttitude']()
        for rotation in (np.eye(3), np.diag([1., -1., -1.])):
            command = attitude.command(0., rotation, np.array([0., 5., 0.]), np.array([5., 0., 0.]),
                                       np.array([0., 0., 1.]), np.zeros(3), .02)
            np.testing.assert_allclose(command, np.zeros(3))
            self.assertAlmostEqual(attitude.nose_error_deg, 90.)

    def test_free_attitude_damps_spin_with_hysteresis_and_command_limit(self):
        attitude = namespace['FreeAttitude']()
        def command(rate):
            return attitude.command(0., np.eye(3), np.array([5., 0., 0.]), np.zeros(3),
                                    np.array([0., 0., 1.]), np.array([0., 0., rate]), .02)
        np.testing.assert_allclose(command(.005), np.zeros(3))
        self.assertLess(command(.2)[2], 0.)
        self.assertLessEqual(np.linalg.norm(command(.2)), .020001)
        self.assertLess(command(.005)[2], 0.)
        np.testing.assert_allclose(command(.001), np.zeros(3))

    def test_default_navigation_has_no_nose_alignment_wait(self):
        planner = namespace['EconomyNavigator'](launch=False)
        self.assertTrue(planner.aligned_once)
        self.assertFalse(planner.nose_first)
        planner.command(1., np.array([9000., 0., 0.]), np.array([1., 0., 0.]),
                        np.array([10000., 0., 0.]), np.zeros(3), [])
        self.assertFalse(planner.aligning)
        self.assertNotEqual(planner.last_mode, 'ALIGN_FOR_BURN')

    def test_fuel_ledger_reports_measured_deltas_without_phase_double_counting(self):
        ledger = namespace['FuelLedger']()
        ledger.observe(0., 0., 'LAUNCH')
        ledger.observe(1., 2., 'LAUNCH')
        ledger.observe(2., 5., 'GUIDE')
        ledger.observe(3., 7., 'GUIDE')
        self.assertEqual(ledger.by_phase['LAUNCH'], 2.)
        self.assertEqual(ledger.by_phase['PHASE_TRANSITION'], 3.)
        self.assertEqual(ledger.by_phase['GUIDE'], 2.)
        self.assertEqual(sum(ledger.by_phase.values()), 7.)

    def test_fuel_ledger_freezes_at_arrival_and_resets_for_new_run(self):
        ledger = namespace['FuelLedger']()
        ledger.observe(0., 0., 'LAUNCH')
        ledger.observe(2., 5., 'CAPTURE_PASS', arrived=True)
        ledger.observe(3., 20., 'POST_ENTRY_BRAKE', arrived=True)
        self.assertEqual(sum(ledger.by_phase.values()), 5.)
        self.assertEqual(ledger.by_phase['ARRIVAL_TRANSITION'], 5.)
        ledger.observe(0., 0., 'LAUNCH')
        ledger.observe(1., 3., 'LAUNCH')
        self.assertEqual(sum(ledger.by_phase.values()), 3.)

    def test_official_example_and_score_budgets(self):
        score = namespace['mission_score']
        self.assertAlmostEqual(score(96., 67.5, .05*209200.), 75.)
        self.assertAlmostEqual(score(200., 26.25), 75.)
        self.assertAlmostEqual(score(300., 30.), 50.)
        self.assertAlmostEqual(score(366.68, 163.08, 49.13), -134.7608867, places=5)
        self.assertAlmostEqual(score(200., 30.)-score(200., 30., 1046.), 1.)

    def test_expensive_delta_v_changes_speed_selection(self):
        efficient, expensive = namespace['EconomyPolicy'](), namespace['EconomyPolicy']()
        efficient.fuel_per_dv, expensive.fuel_per_dv = 4.4, 10.
        fast = efficient.choose(1000., 0., 20., 5., 0., .009)
        slow = expensive.choose(1000., 0., 20., 5., 0., .009)
        self.assertLessEqual(slow, fast)
        self.assertTrue(np.isfinite(expensive.estimated_score))

    def test_damage_is_priced_and_not_declared_free(self):
        clean, damaged = namespace['EconomyPolicy'](), namespace['EconomyPolicy']()
        clean.choose(1000., 4., 50., 15., 0., .009)
        damaged.choose(1000., 4., 50., 15., 1046., .009)
        self.assertAlmostEqual(clean.estimated_score-damaged.estimated_score, 1.)

    def test_policy_does_not_request_economy_braking(self):
        policy = namespace['EconomyPolicy']()
        selected = policy.choose(500., 7., 100., 40., 0., .009)
        self.assertGreaterEqual(selected, 7.)
        self.assertEqual(policy.burn_seconds, 0.)

    def test_servo_hysteresis_and_collision_override(self):
        servo = namespace['EconomyServo']()
        zero = np.zeros(3)
        np.testing.assert_allclose(servo.command([.2, 0., 0.], zero, .5), zero)
        self.assertGreater(servo.command([.5, 0., 0.], zero, .5)[0], 0.)
        self.assertGreater(servo.command([.2, 0., 0.], zero, .5)[0], 0.)
        np.testing.assert_allclose(servo.command([0., 0., 0.], zero, .5), zero)
        np.testing.assert_allclose(servo.command([2., 0., 0.], zero, .5, coast=True), zero)
        self.assertGreater(servo.command([2., 0., 0.], zero, .5, urgent=True, coast=True)[0], 0.)

    def test_capture_line_enters_safe_sphere_without_stopping(self):
        approach = namespace['CaptureApproach']()
        location, velocity, target = np.array([1000., 0., 0.]), np.array([-4., 0., 0.]), np.zeros(3)
        goal, phase, cap = approach.goal(0., location, velocity, target, target, np.empty((0, 3)), np.empty(0))
        delta = goal-location
        fraction = np.clip(-location@delta/(delta@delta), 0., 1.)
        self.assertAlmostEqual(np.linalg.norm(location+fraction*delta), 47., places=5)
        self.assertEqual(phase, 'CRUISE')
        self.assertGreater(cap, 4.)

    def test_braking_goal_is_created_only_after_zone_entry(self):
        approach = namespace['CaptureApproach']()
        empty = np.empty((0, 3))
        approach.goal(0., np.array([100., 0., 0.]), np.array([0., 0., 4.]), np.zeros(3), np.zeros(3), empty, [])
        self.assertFalse(approach.entered)
        _, phase, _ = approach.goal(1., np.array([49.8, 0., 0.]), np.array([0., 0., 4.]),
                                     np.zeros(3), np.zeros(3), empty, [])
        self.assertTrue(approach.entered)
        self.assertEqual(phase, 'POST_ENTRY_BRAKE')

    def test_blocked_pass_uses_arrival_first_fallback(self):
        approach = namespace['CaptureApproach']()
        location = np.array([125., 0., 0.])
        approach.goal(0., location, np.zeros(3), np.zeros(3), np.zeros(3),
                      np.array([[-125., 0., 0.]]), np.array([100.]))
        self.assertIsNone(approach.fallback)  # Reobserve before committing to a long detour.
        approach.goal(6., location, np.zeros(3), np.zeros(3), np.zeros(3),
                      np.array([[-125., 0., 0.]]), np.array([100.]))
        self.assertIsNotNone(approach.fallback)

    def test_fuel_calibration_does_not_charge_gravity_loss_twice(self):
        policy = namespace['EconomyPolicy']()
        policy.observe(0., np.zeros(3), 0.)
        policy.observe(1., np.array([.09, 0., 0.]), .432, thrusting=True, gravity=np.array([-.01, 0., 0.]))
        self.assertAlmostEqual(policy.fuel_per_dv, .9*4.5+.1*4.32)

    def test_ballistic_prediction_includes_gravity(self):
        when, miss = namespace['ballistic_pass'](np.array([8000., 0., 0.]), np.array([6., 0., 0.]),
                                                np.array([9000., 47., 0.]), np.zeros(3))
        self.assertGreater(when, 1000./6.)
        self.assertAlmostEqual(miss, 47., places=4)

    def test_damage_limit_before_arrival_cannot_be_relabelled_as_success(self):
        planner = namespace['EconomyNavigator'](launch=False)
        planner.observe_damage(210000.)
        acceleration, mode, _ = planner.command(10., np.array([49., 0., 0.]), np.zeros(3), np.zeros(3), np.zeros(3), [])
        self.assertEqual(mode, 'DAMAGE_LIMIT')
        self.assertIsNone(planner.arrival_time)
        np.testing.assert_allclose(acceleration, np.zeros(3))

    def test_small_damage_does_not_end_run(self):
        planner = namespace['EconomyNavigator'](launch=False)
        planner.observe_damage(1000.)
        self.assertFalse(planner.failed)

    def test_capture_trim_preserves_speed_in_ship_frame(self):
        ship = np.array([1., 2., 0.])
        velocity = ship+np.array([6., 0., 0.])
        trimmed = namespace['course_trim'](velocity, ship, np.array([-2., .2, 0.]))
        self.assertAlmostEqual(np.linalg.norm(trimmed-ship), 6.)
        self.assertGreater(trimmed[1], velocity[1])
        self.assertGreater(trimmed[0]-ship[0], 5.9)

    def test_coasting_continues_until_first_entry_not_closest_approach(self):
        planner = namespace['EconomyNavigator'](launch=False)
        planner.aligned_once = True
        loc, vel = np.array([9000., 0., 0.]), np.array([6., 0., 0.])
        target = np.array([9024., 47., 0.])
        camera = SimpleNamespace(observes=lambda *args: True,
            obstacles=lambda *args: (np.empty((0, 3)), np.empty((0, 3)), np.empty(0)))
        _, mode, _ = planner.command(200., loc, vel, target, np.zeros(3), [], camera=camera)
        self.assertEqual(mode, 'COAST')
        self.assertIsNone(planner.arrival_time)
        np.testing.assert_allclose(planner.linear_command(vel, .5), np.zeros(3))

    def test_coast_arc_checks_moving_obstacle_between_samples(self):
        gap = namespace['coast_path_clearance'](np.array([9000., 0., 0.]), np.array([10., 0., 0.]),
            np.array([[5., 5., 0.]]), np.array([[0., -10., 0.]]), np.array([1.]), 1.)
        self.assertLess(gap, 0.)

    def test_coast_cannot_use_a_future_return_after_receding(self):
        when, miss = namespace['ballistic_pass'](np.array([9000., 0., 0.]), np.array([6., 0., 0.]),
                                                np.array([8900., 0., 0.]), np.zeros(3))
        self.assertTrue(np.isinf(when) and np.isinf(miss))

    def test_arrival_estimate_freezes_before_post_entry_fuel_or_damage(self):
        planner = namespace['EconomyNavigator'](launch=False)
        loc, vel, target = np.array([9000., 0., 0.]), np.array([0., 4., 0.]), np.array([9049., 0., 0.])
        planner.policy.observe(200., vel, 40.)
        planner.command(200., loc, vel, target, np.zeros(3), [])
        frozen = planner.policy.estimated_score
        self.assertIsNotNone(planner.arrival_time)
        planner.policy.observe(210., vel, 100.)
        planner.observe_damage(210000.)
        planner.command(210., loc, vel, target, np.zeros(3), [])
        self.assertEqual(frozen, planner.policy.estimated_score)
        self.assertFalse(planner.failed)


if __name__ == '__main__':
    unittest.main()
