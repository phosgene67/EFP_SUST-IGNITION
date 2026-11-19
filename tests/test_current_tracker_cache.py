"""Caching unchanged radar fits must preserve all obstacle and uncertainty data."""
from unittest import mock
import unittest

import numpy as np
from controller_loader import load_namespace
from pathlib import Path

namespace = load_namespace((Path(__file__).resolve().parents[1]/'Controller.py').read_text())


class TrackerCacheTests(unittest.TestCase):
    def test_fresh_cached_and_missing_history_reports_are_equivalent(self):
        planners = [namespace['QualificationPlanner'](), namespace['QualificationPlanner']()]
        for enabled, planner in zip((False, True), planners):
            planner.cache_reports, planner.gravity_tracking = enabled, True
        rng = np.random.default_rng(9093)
        origins = np.array([[9100.+20.*i, 30.+i, 15.] for i in range(40)])
        rock_velocity = np.array([.2, -.1, .05])
        reports, previous_stamp = None, None
        for now, stamp in ((3., 1.), (5., 5.), (5.25, 5.), (9., 9.), (9.5, 9.),
                           (13., 13.), (13.5, 13.), (17., 17.), (17.5, 17.)):
            if stamp != previous_stamp:
                ego = np.array([9000.+2.*stamp, .1*stamp, 0.])
                relative = origins+stamp*rock_velocity-ego
                reports = [np.array([position+rng.normal(0., .05, 3),
                    [np.linalg.norm(position), (rock_velocity-np.array([2., .1, 0.]))@position/np.linalg.norm(position), 10.],
                    [stamp, 0., 0.]]) for position in relative]
                previous_stamp = stamp
            location = np.array([9000.+2.*now, .1*now, 0.])
            outputs = [p.obstacles(now, location, np.array([2., .1, 0.]), reports) for p in planners]
            for old, cached in zip(*outputs):
                np.testing.assert_array_equal(old, cached)
            for attribute in ('radar_forecast_rates', 'radar_core_radii', 'radar_report_ages', 'radar_track_confidence'):
                np.testing.assert_array_equal(getattr(planners[0], attribute), getattr(planners[1], attribute))
        with mock.patch.object(np, 'interp', wraps=np.interp) as interpolation:
            planners[1].obstacles(17.6, location, np.array([2., .1, 0.]), reports)
            interpolation.assert_not_called()
        # A changed size starts an unmatched track; an old confidence fit must
        # not leak onto the new report or identity.
        reports[0] = reports[0].copy()
        reports[0][1, 2], reports[0][2, 0] = 100., 18.
        old, cached = [p.obstacles(18., location, np.array([2., .1, 0.]), reports) for p in planners]
        for a, b in zip(old, cached):
            np.testing.assert_array_equal(a, b)
        self.assertFalse(planners[1].radar_track_confidence[-1])
        self.assertFalse(namespace['ShootingNavigator']().cache_reports)


if __name__ == '__main__':
    unittest.main()
