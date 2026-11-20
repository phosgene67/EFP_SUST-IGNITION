"""Separate radar geometry from measurement noise without changing camera bounds."""
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest import mock
import numpy as np
from controller_loader import load_namespace

NS = load_namespace((Path(__file__).resolve().parents[1]/'Controller.py').read_text())


class CurrentPhysicalRiskTests(unittest.TestCase):
    def test_position_and_diameter_noise_are_covariance_not_extra_physical_size(self):
        samples = []
        original = NS['gaussian_encounter_risk']
        for enabled in (False, True):
            captured = []
            def risk(gap, sigma, speed):
                captured.append((gap.copy(), sigma.copy()))
                return original(gap, sigma, speed)
            p = NS['ShootingNavigator'](launch=False)
            p.physical_risk, p.expected_risk = enabled, True
            radar = [np.array([[40., 25., 0.], [47., -1., 10.], [20., 0., 0.]])]
            with mock.patch.dict(NS, gaussian_encounter_risk=risk):
                p.command(20., np.array([9000., 0., 0.]), np.array([1., 0., 0.]),
                    np.array([10300., 0., 0.]), np.zeros(3), radar)
            samples.append(captured[0])
            np.testing.assert_array_equal(p.radar_measured_diameters, [10.])
        # Physical radius:5 m measured rock + the unchanged3 m vehicle.
        # The previous core was .575*10+5=10.75 m.
        np.testing.assert_allclose(samples[1][0]-samples[0][0], 2.75)
        np.testing.assert_allclose(samples[1][1]**2-samples[0][1]**2, (.05*10./2.)**2)
        self.assertTrue(np.all(samples[1][1] >= 2.))

    def test_camera_occupied_volume_still_blocks_a_finite_shot(self):
        p = NS['ShootingNavigator'](launch=False)
        p.physical_risk, p.expected_risk = True, True
        camera = SimpleNamespace(observes=lambda *a: True,
            obstacles=lambda *a: (np.array([[20., 0., 0.]]), np.zeros((1, 3)), np.array([500.])))
        p.command(20., np.array([9000., 0., 0.]), np.array([1., 0., 0.]),
            np.array([10300., 0., 0.]), np.zeros(3), [], camera=camera)
        self.assertFalse(p.shooting_active)


if __name__ == '__main__':
    unittest.main()
