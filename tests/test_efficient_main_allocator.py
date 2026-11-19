"""Validate allowed acceleration requests against the unmodified jet allocator."""
from pathlib import Path
import unittest
import numpy as np
from controller_loader import load_namespace
from benchmark_final import Allocator

NS = load_namespace((Path(__file__).resolve().parents[1]/'tests/score_sweeps/score55_rejected_prototypes_controller.py').read_text())
JETS = Path('C:/Users/Asus/AppData/Local/SimDynamX/STPro/Assets/Repos/STU4-Sim/Data/SpaceBuggyJetConfigNoMains.csv')


class EfficientMainAllocatorTests(unittest.TestCase):
    def test_positive_x_uses_allowed_request_with_negligible_transverse_impulse(self):
        allocator = Allocator(JETS, 'revised simplex')
        p = NS['ShootingNavigator'](launch=False)
        p.efficient_x, p.shooting_active = True, True
        p.burn_target, p.burn_signs = np.array([1., 0., 0.]), np.array([1., 0., 0.])
        p.burn_order, p.axis_done = np.array([0, 1, 2]), np.array([False, True, True])
        command = p.linear_command(np.zeros(3), .5)
        accel, angular, rate = allocator.apply(command, np.zeros(3))
        self.assertGreater(accel[0], .192)
        self.assertLess(np.linalg.norm(accel[1:]), .00001)
        self.assertLess(rate/accel[0], 4.33)
        np.testing.assert_allclose(angular, np.zeros(3), atol=1e-10)

    def test_calibrated_launch_preserves_upward_direction_and_fuel_efficiency(self):
        allocator = Allocator(JETS, 'revised simplex')
        accelerations = []
        for enabled in (False, True):
            p = NS['ShootingNavigator']()
            p.calibrated_launch = enabled
            p.last_phase, p.coasting, p.aligning = 'LAUNCH', False, False
            p.body_rotation = np.eye(3)
            p.launch.up = np.array([0., 0., 1.])
            p.previous_velocity = np.array([0., 0., 3.])
            command = p.linear_command(np.zeros(3), .5)
            accel, angular, rate = allocator.apply(command, np.zeros(3))
            self.assertLessEqual(np.linalg.norm(command), .500000001)
            self.assertLess(np.linalg.norm(accel[:2]), .0001)
            self.assertLess(rate/accel[2], 4.33)
            np.testing.assert_allclose(angular, np.zeros(3), atol=1e-10)
            accelerations.append(accel[2])
        self.assertLess(accelerations[0], .10)
        self.assertGreater(accelerations[1], .13)

    def test_positive_z_burn_gains_acceleration_without_more_fuel_per_impulse(self):
        achieved = []
        allocator = Allocator(JETS, 'revised simplex')
        for enabled in (False, True):
            p = NS['ShootingNavigator'](launch=False)
            p.shooting_active, p.efficient_main = True, enabled
            p.burn_target, p.burn_signs = np.array([0., 0., 1.]), np.array([0., 0., 1.])
            p.burn_order, p.axis_done = np.array([2, 0, 1]), np.array([True, True, False])
            command = p.linear_command(np.zeros(3), .5)
            self.assertLessEqual(np.linalg.norm(command), .500000001)
            accel, angular, fuel_rate = allocator.apply(command, np.zeros(3))
            np.testing.assert_allclose(angular, np.zeros(3), atol=1e-10)
            achieved.append(accel[2])
            self.assertLess(fuel_rate/accel[2], 4.33)
            np.testing.assert_array_equal(p.linear_command(np.array([0., 0., 1.]), .5), np.zeros(3))
        self.assertLess(achieved[0], .10)
        self.assertGreater(achieved[1], .13)


if __name__ == '__main__':
    unittest.main()
