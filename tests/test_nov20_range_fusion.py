"""Independent surface range refines center range with unchanged safety bounds."""
from pathlib import Path
import unittest
from unittest import mock
import numpy as np
from controller_loader import load_namespace
import test_sim_adapter

NS = load_namespace((Path(__file__).parent/'score_sweeps'/'nov20_range_fusion_controller.py').read_text())


class RangeFusionTests(unittest.TestCase):
    run_adapter = test_sim_adapter.AdapterTests.run_adapter

    def test_opt_in_survives_simulation_clock_reset(self):
        source = Path(__file__).parent/'score_sweeps'/'nov20_range_fusion_controller.py'
        with mock.patch.object(test_sim_adapter, 'SOURCE', source):
            self.run_adapter([], reset_clock=True,
                runtime_flags={'EFP_RADAR_RANGE_FUSION': '1'})
        self.assertTrue(self.runtime['planner'].range_fusion)

    def test_independent_ranging_reduces_radial_noise_without_reducing_bounds(self):
        rng = np.random.default_rng(20261120)
        count, diameter = 1000, 10.
        measurements = np.array([100., 0., 0.])+rng.normal(0., 2., (count, 3))
        surface = 100.-diameter/2.+rng.normal(0., 2., count)
        sizes = diameter+rng.normal(0., .05*diameter, count)
        reports = [np.array([p, [s, 0., d], [1., 0., 0.]])
                   for p, s, d in zip(measurements, surface, sizes)]
        results = []
        for enabled in (False, True):
            planner = NS['QualificationPlanner']()
            planner.range_fusion = enabled
            results.append(planner.obstacles(1., np.zeros(3), np.zeros(3), reports))
        raw, fused = results
        self.assertLess(np.std(np.linalg.norm(fused[0], axis=1)-100.),
                        .8*np.std(np.linalg.norm(raw[0], axis=1)-100.))
        np.testing.assert_allclose(fused[0]/np.linalg.norm(fused[0], axis=1)[:, None],
                                   raw[0]/np.linalg.norm(raw[0], axis=1)[:, None])
        np.testing.assert_array_equal(raw[1], fused[1])
        np.testing.assert_array_equal(raw[2], fused[2])
        np.testing.assert_array_equal(reports[0][0], measurements[0])

    def test_invalid_surface_range_retains_original_center(self):
        planner = NS['QualificationPlanner']()
        planner.range_fusion = True
        packet = np.array([[30., 0., 0.], [-40., 0., 10.], [1., 0., 0.]])
        centers, _, _ = planner.obstacles(1., np.zeros(3), np.zeros(3), [packet])
        np.testing.assert_array_equal(centers[0], packet[0])


if __name__ == '__main__':
    unittest.main()
