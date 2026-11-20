"""Timestamp-aligned radar position estimates; no simulation connection."""
from pathlib import Path
import unittest
import numpy as np
from controller_loader import load_namespace


SOURCE = Path(__file__).resolve().parents[1] / 'Controller.py'
NS = load_namespace(SOURCE.read_text())


class RadarPositionTests(unittest.TestCase):
    def replay(self, noise, velocity):
        planners = [NS['QualificationPlanner'](), NS['QualificationPlanner']()]
        for smooth, planner in zip((False, True), planners):
            planner.smooth_radar_position = smooth
            planner.gravity_tracking = planner.cache_reports = True
        origin = np.array([9500., 100., 80.])
        gravity = -711000.*origin/np.linalg.norm(origin)**3
        velocity = np.asarray(velocity)
        rng = np.random.default_rng(7005)
        errors = [[], []]
        for stamp in np.arange(0., 85., 4.):
            own_position = np.array([9000.+4.*stamp, 0., 0.])
            own_velocity = np.array([4., 0., 0.])
            actual = origin+stamp*velocity+.5*stamp**2*gravity
            relative = actual-own_position
            direction = relative/np.linalg.norm(relative)
            radial = (velocity+stamp*gravity-own_velocity)@direction
            report = np.array([relative+rng.normal(0., noise, 3),
                [np.linalg.norm(relative), radial, 10.], [stamp, 0., 0.]])
            outputs = [p.obstacles(stamp, own_position, own_velocity, [report]) for p in planners]
            for planner, (centers, _, _) , error in zip(planners, outputs, errors):
                if stamp >= 32.:
                    self.assertTrue(planner.radar_track_confidence[0])
                    error.append(centers[0]+own_position-actual)
            np.testing.assert_array_equal(outputs[0][1], outputs[1][1])
            np.testing.assert_array_equal(outputs[0][2], outputs[1][2])
            for key in ('radar_forecast_rates', 'radar_core_radii', 'radar_report_ages'):
                np.testing.assert_array_equal(getattr(planners[0], key), getattr(planners[1], key))
            if stamp < 12.:
                np.testing.assert_array_equal(outputs[0][0], outputs[1][0])
        return planners, np.asarray(errors)

    def test_fitted_position_reduces_noisy_position_error_without_shrinking_margins(self):
        _, errors = self.replay(2., [.15, -.12, .08])
        raw, fitted = np.mean(np.sum(errors**2, axis=2), axis=1)
        self.assertLess(fitted, .7*raw)

    def test_moving_track_is_projected_to_latest_report_not_average_sample_time(self):
        _, errors = self.replay(0., [.5, -.3, .2])
        # The controller recomputes gravity at the latest measured position;
        # the replay uses a fixed launch gravity. Allow that millimetre error,
        # while rejecting the several-metre lag of an unprojected average.
        self.assertLess(np.max(np.abs(errors[1])), .01)

    def test_repeated_cached_reports_do_not_apply_the_position_correction_twice(self):
        planners = [NS['QualificationPlanner'](), NS['QualificationPlanner']()]
        for cached, planner in zip((False, True), planners):
            planner.cache_reports = cached
            planner.gravity_tracking = planner.smooth_radar_position = True
        rng = np.random.default_rng(7200)
        origin = np.array([9500., 100., 80.])
        gravity = -711000.*origin/np.linalg.norm(origin)**3
        previous = None
        for now in np.arange(0., 65., .5):
            stamp = 4.*np.floor(now/4.)
            if stamp != previous:
                measured = origin+.5*stamp**2*gravity-[9000.+4.*stamp, 0., 0.]
                direction = measured/np.linalg.norm(measured)
                report = np.array([measured+rng.normal(0., 2., 3),
                    [np.linalg.norm(measured), (stamp*gravity-[4., 0., 0.])@direction, 10.],
                    [stamp, 0., 0.]])
                previous = stamp
            outputs = [p.obstacles(now, np.array([9000.+4.*now, 0., 0.]),
                np.array([4., 0., 0.]), [report]) for p in planners]
            for a, b in zip(*outputs):
                np.testing.assert_allclose(a, b, atol=1e-12, rtol=0.)


if __name__ == '__main__':
    unittest.main()
