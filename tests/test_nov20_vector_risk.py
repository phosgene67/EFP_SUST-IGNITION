"""Verify the faster evaluator preserves collision probabilities and energies."""
import builtins
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from controller_loader import load_namespace


NS = load_namespace((Path(__file__).resolve().parents[1] / 'Controller.py').read_text())


class VectorRiskTests(unittest.TestCase):
    def test_matches_reference_across_tails_broadcast_shapes_and_energy(self):
        rng = np.random.default_rng(720)
        for shape in ((), (10,), (90, 80)):
            gap = rng.uniform(-5., 150., shape)
            sigma = rng.uniform(.01, 8., shape)
            speed = rng.uniform(0., 20., shape)
            reference = NS['gaussian_encounter_risk'](gap, sigma, speed)
            actual = NS['vectorized_encounter_risk'](gap, sigma, speed)
            np.testing.assert_allclose(actual[0], reference[0], atol=2e-16, rtol=2e-14)
            np.testing.assert_array_equal(actual[1], reference[1])

    def test_uses_original_model_if_scipy_special_is_unavailable(self):
        original_import = builtins.__import__
        def guarded_import(name, *args, **kwargs):
            if name == 'scipy.special':
                raise ImportError('test missing optional accelerator')
            return original_import(name, *args, **kwargs)
        args = (np.array([-1., 0., 2., 20.]), 2., 8.)
        expected = NS['gaussian_encounter_risk'](*args)
        with patch('builtins.__import__', side_effect=guarded_import):
            actual = NS['vectorized_encounter_risk'](*args)
        for a, b in zip(actual, expected):
            np.testing.assert_array_equal(a, b)


if __name__ == '__main__':
    unittest.main()
