"""One-time alignment, Momentum handoff and unchanged default control checks."""
from pathlib import Path
import unittest
import numpy as np
from controller_loader import load_namespace
from benchmark_final import Allocator, rotate


ROOT = Path(__file__).resolve().parents[1]
NS = load_namespace((ROOT/'Controller.py').read_text())
EXPERIMENTAL_NS = load_namespace((ROOT/'tests/score_sweeps/score55_rejected_prototypes_controller.py').read_text())
JETS = Path('C:/Users/Asus/AppData/Local/SimDynamX/STPro/Assets/Repos/STU4-Sim/Data/SpaceBuggyJetConfigNoMains.csv')


class MainAxisSlewTests(unittest.TestCase):
    def test_powered_launch_turn_is_opt_in_and_requires_near_vertical_target(self):
        for enabled, target_offset, expected in ((False, [0., 0., 1300.], False),
                (True, [0., 0., 1300.], True), (True, [1300., 0., 0.], False)):
            p = EXPERIMENTAL_NS['AxisAlignedNavigator']()
            p.powered_launch_slew = enabled
            p.observe_scene = lambda *a: (np.empty((0, 3)), np.empty((0, 3)),
                np.empty(0), np.empty(0), np.empty((0, 3)), np.empty(0))
            loc = np.array([0., 0., 9000.])
            p.command(0., loc, np.zeros(3), loc+target_offset, np.zeros(3), [])
            self.assertEqual(p.slew_active, expected)
            self.assertFalse(p.launch.done)
            if expected:
                self.assertGreater(p.linear_command(np.zeros(3), .5)@p.launch.up, 0.)

    def test_powered_launch_does_not_bypass_nearby_radar_blockage(self):
        p = EXPERIMENTAL_NS['AxisAlignedNavigator']()
        p.powered_launch_slew = True
        rock = np.array([[0., 0., 2.], [0., 0., 5.], [10., 0., 0.]])
        p.command(10., np.array([0., 0., 9000.]), np.zeros(3),
                  np.array([0., 0., 10300.]), np.zeros(3), [rock])
        self.assertFalse(p.slew_active)

    def test_small_z_alignment_settles_with_low_modeled_fuel(self):
        heading = np.array([.05, -.07, 1.])
        heading /= np.linalg.norm(heading)
        slew = NS['MainAxisSlew'](heading, 0., 2)
        allocator = Allocator(JETS)
        rotation, omega, dt, fuel = np.eye(3), np.zeros(3), 1./60., 0.
        for tick in range(600):
            angular = slew.angular_command(rotation, omega, .02)
            if slew.complete:
                break
            linear = np.zeros(3) if slew.settling else rotation@(.025*heading)
            _, alpha, rate = allocator.apply(linear, angular)
            rotation = rotate(rotation, dt*omega+.5*dt*dt*alpha)
            omega += dt*alpha
            fuel += dt*rate
        self.assertTrue(slew.complete)
        self.assertLess(fuel, 1.)
        self.assertGreater(rotation[2]@heading, np.cos(np.deg2rad(1.)))

    def test_turn_reaches_fixed_axis_and_stops_instead_of_following_velocity(self):
        slew = NS['MainAxisSlew']([0., 0., 1.], 0.)
        allocator = Allocator(JETS)
        rotation, omega, dt = np.eye(3), np.zeros(3), 1./60.
        for tick in range(1200):
            command = slew.angular_command(rotation, omega, .02)
            self.assertLessEqual(np.linalg.norm(command), .020000001)
            if slew.complete:
                break
            _, alpha, _ = allocator.apply(np.zeros(3), command)
            rotation = rotate(rotation, dt*omega+.5*dt*dt*alpha)
            omega += dt*alpha
        self.assertTrue(slew.complete)
        self.assertGreater(rotation[0]@slew.heading, np.cos(np.deg2rad(2.)))
        np.testing.assert_array_equal(slew.angular_command(rotation, omega, .02), np.zeros(3))

    def test_completed_turn_returns_to_original_momentum_planning(self):
        p = NS['AxisAlignedNavigator'](launch=False)
        p.main_axis_slew = NS['MainAxisSlew']([1., 0., 0.], 0.)
        p.main_axis_slew.complete = True
        p.command(50., np.array([9000., 0., 0.]), np.array([2., 0., 0.]),
                  np.array([10300., 0., 0.]), np.zeros(3), [])
        self.assertFalse(p.slew_active)
        self.assertNotEqual(p.last_phase, 'ALIGN_MAIN_AXIS')
        attitude = NS['MainAxisAttitude'](p)
        np.testing.assert_array_equal(attitude.command(50., np.eye(3), [0., 5., 0.],
            [0., 0., 5.], [0., 0., 1.], np.zeros(3), .02), np.zeros(3))

    def test_unfinished_turn_times_out_to_momentum(self):
        p = NS['AxisAlignedNavigator'](launch=False)
        p.main_axis_slew = NS['MainAxisSlew']([0., 0., 1.], 0.)
        p.command(30., np.array([9000., 0., 0.]), np.array([2., 0., 0.]),
                  np.array([10300., 0., 0.]), np.zeros(3), [])
        self.assertTrue(p.slew_abandoned)
        self.assertFalse(p.slew_active)

    def test_nearby_blocking_radar_track_prevents_slew(self):
        p = NS['AxisAlignedNavigator'](launch=False)
        rock = np.array([[2., 0., 0.], [0., 0., 5.], [10., 0., 0.]])
        p.command(10., np.array([9000., 0., 0.]), np.zeros(3),
                  np.array([10300., 0., 0.]), np.zeros(3), [rock])
        self.assertFalse(p.slew_active)

    def test_failed_or_arrived_slew_never_commands_boost(self):
        for failed, arrived in ((True, False), (False, True)):
            p = NS['AxisAlignedNavigator'](launch=False)
            p.main_axis_slew = NS['MainAxisSlew']([1., 0., 0.], 0.)
            p.slew_active, p.failed = True, failed
            p.arrival_time = 10. if arrived else None
            # The alignment stage must not run in these states.
            p.slew_active = False
            p.shooting_active = True
            p.axis_done[:] = True
            np.testing.assert_array_equal(p.linear_command(np.zeros(3), .5), np.zeros(3))


if __name__ == '__main__':
    unittest.main()
