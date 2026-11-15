"""Offline planner checks; AST extraction deliberately avoids connecting to the sim."""
import ast
from pathlib import Path
import unittest
import numpy as np
from controller_loader import load_namespace

SOURCE = Path(__file__).resolve().parents[1] / 'Controller.py'
namespace = load_namespace(SOURCE.read_text())
class Planner(namespace['FinalNavigator']):
    """Navigation-only fixtures; real launch is tested separately and in the adapter."""
    def __init__(self):
        super().__init__(launch=False)


class PlannerTests(unittest.TestCase):
    def test_high_speed_has_separate_station_stopping_guard(self):
        planner = Planner()
        planner.command(0., np.zeros(3), np.array([12., 0., 0.]),
                        np.array([1305.8, 0., 0.]), np.zeros(3), [])
        self.assertGreaterEqual(planner.stopping_distance, 12.**2/(2.*.10))
        self.assertLessEqual(planner.last_horizon, 15)

    def test_retained_command_cannot_bypass_terminal_braking(self):
        planner = Planner()
        planner.previous_velocity = np.array([12., 0., 0.])
        accel, _, _ = planner.command(0., np.zeros(3), np.array([8., 0., 0.]),
                                     np.array([60., 0., 0.]), np.zeros(3), [])
        cap = np.sqrt(2.*namespace['BRAKE_ACCEL']*15.)
        self.assertLessEqual(np.linalg.norm(planner.previous_velocity), cap+1e-6)
        self.assertLess(accel[0], 0.)

    def test_faster_cruise_and_post_arrival_hold(self):
        planner = Planner()
        loc, vel = np.zeros(3), np.zeros(3)
        target = np.array([2000., 0., 0.])
        peak, closest = 0., np.inf
        arrived = False
        for tick in range(1100):
            accel, mode, _ = planner.command(tick*.5, loc, vel, target, np.zeros(3), [])
            # Continue well past the scoring boundary with modest actuator loss.
            actual = .9*accel
            loc += .5*vel + .125*actual
            vel += .5*actual
            peak = max(peak, np.linalg.norm(vel))
            distance = np.linalg.norm(target-loc)
            if distance < 50.:
                arrived = True
            if arrived:
                closest = min(closest, distance)
        self.assertGreater(peak, 8.5)  # Requested cruise is now 9 m/s.
        self.assertTrue(arrived)
        self.assertGreater(closest, 40.)
        self.assertAlmostEqual(np.linalg.norm(target-loc), namespace['HOLD_RADIUS'], delta=.65)
        self.assertLess(np.linalg.norm(vel), .1)
        self.assertEqual(mode, 'HOLD')

    def test_clear_route_converges(self):
        planner = Planner()
        loc, vel = np.zeros(3), np.zeros(3)
        target = np.array([300., 0., 0.])
        for tick in range(1200):
            accel, _, _ = planner.command(tick*.25, loc, vel, target, np.zeros(3), [])
            self.assertLessEqual(np.linalg.norm(accel), namespace['MAX_ACCEL']+1e-6)
            loc += .25*vel + .5*.25**2*accel
            vel += .25*accel
            if np.linalg.norm(target-loc) < 49.8:
                break
        self.assertLess(np.linalg.norm(target-loc), 49.8)

    def test_obstacle_changes_route(self):
        report = np.array([[60., 0., 0.], [50., -4., 20.], [0., 0., 0.]])
        accel, mode, _ = Planner().command(0., np.zeros(3), np.array([4., 0., 0.]),
                                         np.array([300., 0., 0.]), np.zeros(3), [report])
        self.assertNotEqual(mode, 'GUIDE')
        self.assertTrue(np.isfinite(accel).all())

    def test_stale_stationary_report_corrects_shuttle_motion(self):
        planner = Planner()
        report = np.array([[100., 0., 0.], [90., -2., 20.], [0., 0., 0.]])
        planner.obstacles(0., np.zeros(3), np.array([2., 0., 0.]), [])
        centers, _, _ = planner.obstacles(5., np.array([10., 0., 0.]),
                                         np.array([2., 0., 0.]), [report])
        np.testing.assert_allclose(centers[0], [90., 0., 0.])

    def test_closed_loop_static_obstacle(self):
        planner = Planner()
        loc, vel = np.zeros(3), np.zeros(3)
        target, obstacle = np.array([300., 0., 0.]), np.array([120., 0., 0.])
        minimum = np.inf
        for tick in range(1200):
            relative = obstacle-loc
            distance = np.linalg.norm(relative)
            report = np.array([relative, [distance-10., -vel.dot(relative/distance), 20.],
                               [tick*.25, 0., 0.]])
            accel, _, _ = planner.command(tick*.25, loc, vel, target, np.zeros(3), [report])
            loc += .25*vel + .5*.25**2*accel
            vel += .25*accel
            minimum = min(minimum, np.linalg.norm(obstacle-loc))
            if np.linalg.norm(target-loc) < 49.8:
                break
        self.assertGreater(minimum, 13.)  # 10 m obstacle + 3 m shuttle
        self.assertLess(np.linalg.norm(target-loc), 49.8)

    def test_degenerate_and_invalid_reports(self):
        accel, _, _ = Planner().command(0., np.zeros(3), np.zeros(3), np.zeros(3),
                                       np.zeros(3), [np.full((3, 3), np.nan), np.zeros((3, 3))])
        np.testing.assert_allclose(accel, np.zeros(3))

    def test_transverse_velocity_tracking_and_repeated_measurement(self):
        planner = Planner()
        speed = np.array([0., 1., 0.])
        last = None
        for stamp in range(0, 21, 2):
            position = np.array([100., float(stamp), 0.])
            report = np.array([position, [np.linalg.norm(position)-10.,
                               speed.dot(position/np.linalg.norm(position)), 20.], [stamp, 0., 0.]])
            _, estimate, _ = planner.obstacles(float(stamp), np.zeros(3), np.zeros(3), [report])
            last = report
        self.assertGreater(estimate[0, 1], .9)
        count = planner.track_counter
        planner.obstacles(20.5, np.zeros(3), np.zeros(3), [last])
        self.assertEqual(planner.track_counter, count)

    def test_sorting_does_not_swap_tracks(self):
        planner = Planner()
        for stamp in range(0, 11, 2):
            reports = []
            for sign, diameter in ((1, 20.), (-1, 40.)):
                position = np.array([100., sign*(50.+stamp), 0.])
                speed = np.array([0., float(sign), 0.])
                reports.append(np.array([position, [1., speed.dot(position/np.linalg.norm(position)), diameter],
                                         [stamp, 0., 0.]]))
            if stamp % 4:
                reports.reverse()
            _, speeds, _ = planner.obstacles(float(stamp), np.zeros(3), np.zeros(3), reports)
        for report, estimate in zip(reports, speeds):
            sign = 1 if report[1, 2] == 20. else -1
            self.assertGreater(sign*estimate[1], .7)

    def test_noisy_delayed_crossing_debris(self):
        for seed in (7, 19, 43):
            with self.subTest(seed=seed):
                rng = np.random.default_rng(seed)
                planner = Planner()
                loc, vel = np.zeros(3), np.zeros(3)
                target = np.array([350., 0., 0.])
                positions = np.array([[140., 35., 0.], [220., -40., 15.], [100., 80., -30.]])
                speeds = np.array([[0., -.6, 0.], [0., .3, -.1], [.1, -.2, 0.]])
                minimum = np.inf
                for tick in range(600):
                    now = tick*.5
                    if tick % 3 == 0:
                        reports = []
                        for pos, speed in zip(positions, speeds):
                            relative = pos-loc
                            distance = np.linalg.norm(relative)
                            reports.append(np.array([
                                relative+rng.normal(0., 2., 3),
                                [distance-10., (speed-vel).dot(relative/distance)+rng.normal(0., .02),
                                 20.*rng.normal(1., .05)], [now, 0., 0.]]))
                        rng.shuffle(reports)
                    accel, _, _ = planner.command(now, loc, vel, target, np.zeros(3), reports)
                    # Small unmodelled gravity and actuator under-delivery.
                    actual = .9*accel + np.array([0., 0., -.003])
                    loc += .5*vel + .125*actual
                    vel += .5*actual
                    positions += .5*speeds
                    minimum = min(minimum, np.linalg.norm(positions-loc, axis=1).min())
                    if np.linalg.norm(target-loc) < 49.8:
                        break
                self.assertGreater(minimum, 13.)
                self.assertLess(np.linalg.norm(target-loc), 49.8)

    def test_escape_from_inflated_overlap(self):
        # Physical spheres do not intersect, but their uncertainty margins do.
        report = np.array([[15., 0., 0.], [5., 0., 20.], [0., 0., 0.]])
        accel, mode, _ = Planner().command(0., np.zeros(3), np.zeros(3),
                                         np.array([300., 0., 0.]), np.zeros(3), [report])
        self.assertEqual(mode, 'ESCAPE')
        self.assertLess(accel[0], 0.)

    def test_fast_incoming_obstacle_not_filtered_out(self):
        report = np.array([[350., 0., 0.], [340., -15., 20.], [0., 0., 0.]])
        _, mode, clearance = Planner().command(0., np.zeros(3), np.zeros(3),
                                              np.array([500., 0., 0.]), np.zeros(3), [report])
        self.assertNotEqual(mode, 'GUIDE')
        self.assertTrue(np.isfinite(clearance))

    def test_body_connection_prefix_unchanged(self):
        # Official sibling starter shares the protected connection/helper section.
        marker = '# EXAMPLE USER CODE'
        starter = SOURCE.with_name('BasicController.py').read_text()
        self.assertEqual(SOURCE.read_text().split(marker)[0], starter.split(marker)[0])


class AttitudeTests(unittest.TestCase):
    def test_vertical_climb_keeps_roll_continuous_under_lateral_noise(self):
        attitude = namespace['FlightAttitude']()
        # +X nose already points along world +Z; +Y stays right, roof is -X.
        rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
        previous_up = None
        for tick in range(100):
            velocity = np.array([.001*(-1.)**tick, .001*np.sin(tick), 10.])
            angular = attitude.command(tick*.02, rotation, velocity, velocity,
                                       np.array([0., 0., 1.]), np.zeros(3), .02)
            if previous_up is not None:
                self.assertGreater(previous_up@attitude.up, .999)
            previous_up = attitude.up.copy()
            self.assertTrue(attitude.vertical)
            self.assertLess(np.linalg.norm(angular), .001)
            self.assertLess(attitude.nose_error_deg, .02)

    def test_nose_turns_into_vertical_flight_instead_of_roof_first_motion(self):
        attitude = namespace['FlightAttitude']()
        rotation, omega = np.eye(3), np.zeros(3)
        velocity = np.array([0., 0., 10.])
        for tick in range(3000):
            angular = attitude.command(tick*.02, rotation, velocity, velocity,
                                       np.array([0., 0., 1.]), omega, .02)
            delta = .02*omega+.5*.02**2*angular
            omega += .02*angular
            angle = np.linalg.norm(delta)
            if angle > 1e-12:
                x, y, z = delta/angle
                skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
                rotation = (np.eye(3)+np.sin(angle)*skew+(1-np.cos(angle))*(skew@skew)).T@rotation
        self.assertGreater(rotation[0]@velocity/np.linalg.norm(velocity), .999)
        body_velocity = rotation@velocity
        self.assertGreater(body_velocity[0], 9.99)
        self.assertLess(np.linalg.norm(body_velocity[1:]), 10.*np.sin(np.deg2rad(2.)))

    def test_braking_does_not_reverse_nose_and_planet_up_input_is_unchanged(self):
        attitude = namespace['FlightAttitude']()
        up = np.array([0., 0., 5.])
        attitude.command(0., np.eye(3), np.array([10., 0., 0.]), np.zeros(3), up, np.zeros(3), .02)
        np.testing.assert_allclose(attitude.forward, [1., 0., 0.])
        np.testing.assert_allclose(up, [0., 0., 5.])

    def test_upright_alignment_has_zero_command(self):
        command = namespace['attitude_command'](np.eye(3), [1., 0., 0.], [0., 0., 1.],
                                                 np.zeros(3), .02)
        np.testing.assert_allclose(command, np.zeros(3))

    def test_turn_and_upside_down_recovery(self):
        # Independently integrate body angular rates and the passive rotation.
        for initial in (np.diag([1., -1., -1.]),
                        np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])):
            rotation, omega = initial.copy(), np.zeros(3)
            for _ in range(3000):
                command = namespace['attitude_command'](rotation, [1., 0., 0.], [0., 0., 1.], omega, .02)
                self.assertLessEqual(np.linalg.norm(command), .020001)
                delta = .02*omega + .5*.02**2*command
                omega += .02*command
                angle = np.linalg.norm(delta)
                if angle > 1e-12:
                    x, y, z = delta/angle
                    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
                    active = np.eye(3)+np.sin(angle)*skew+(1-np.cos(angle))*(skew@skew)
                    rotation = active.T@rotation
            # Allow the fuel-saving two-degree attitude settling band.
            np.testing.assert_allclose(rotation, np.eye(3), atol=.04)
            self.assertLess(np.linalg.norm(omega), .006)

    def test_vertical_flight_and_stationary_heading(self):
        for forward in ([0., 0., 1.], [0., 0., 0.]):
            command = namespace['attitude_command'](np.eye(3), forward, [0., 0., 1.], np.zeros(3), .02)
            self.assertTrue(np.isfinite(command).all())
            self.assertLessEqual(np.linalg.norm(command), .020001)


if __name__ == '__main__':
    unittest.main()
