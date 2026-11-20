"""Radar epoch estimates, immutable reports, and own-pose reconstruction."""
from pathlib import Path
import unittest
import numpy as np
from controller_loader import load_namespace

NS = load_namespace((Path(__file__).resolve().parents[1]/'Controller.py').read_text())


def report(stamp):
    return np.array([[100., 0., 0.], [95., 0., 10.], [stamp, 0., 0.]])


class RadarTimebaseTests(unittest.TestCase):
    def test_zero_offset_lock_preserves_tracks_but_real_retiming_discards_them(self):
        for offset, retained in ((0., True), (.8, False)):
            p = NS['FinalNavigator'](launch=False)
            p.radar_clock_enabled = True
            p.radar_timebase.offset = offset
            p.radar_timebase.just_locked = True
            p.radar_timebase.adjust = lambda now, reports: reports
            p.report_cache = {'sentinel': object()}
            seen = []
            def obstacles(*args):
                seen.append(bool(p.report_cache))
                return np.empty((0, 3)), np.empty((0, 3)), np.empty(0)
            p.obstacles = obstacles
            p.observe_scene(10., np.array([9000., 0., 0.]), np.zeros(3),
                np.array([10300., 0., 0.]), np.zeros(3), [], None)
            self.assertEqual(seen, [retained])

    def test_locks_lower_envelope_once_without_chasing_later_delays(self):
        clock = NS['RadarTimebase']()
        for now in np.arange(2., 9., .5):
            lag = .8 if int(now*2) % 3 else 1.2
            clock.adjust(now, [report(now-lag)])
        self.assertTrue(clock.locked)
        self.assertAlmostEqual(clock.offset, .75)
        original = report(10.)
        corrected = clock.adjust(20., [original])
        self.assertAlmostEqual(corrected[0][2, 0], 10.75)
        self.assertEqual(original[2, 0], 10.)
        self.assertFalse(clock.just_locked)
        self.assertAlmostEqual(clock.offset, .75)

    def test_normal_publication_latency_does_not_retime_reports(self):
        clock = NS['RadarTimebase']()
        for now in np.arange(2., 9., .5):
            clock.adjust(now, [report(now-.05)])
        self.assertTrue(clock.locked)
        self.assertEqual(clock.offset, 0.)

    def test_no_calibration_from_initial_zero_stamps_or_one_stale_publication(self):
        clock = NS['RadarTimebase']()
        for now in range(20):
            clock.adjust(float(now), [report(0.)])
        self.assertFalse(clock.locked)
        for now in range(20, 40):
            clock.adjust(float(now), [report(1.)])
        self.assertFalse(clock.locked)

    def test_epoch_correction_recovers_correct_pose_for_a_moving_shuttle(self):
        centers = []
        for enabled in (False, True):
            p = NS['ShootingNavigator'](launch=False)
            p.gravity_tracking = False
            p.radar_clock_enabled = enabled
            p.radar_timebase.locked, p.radar_timebase.offset = True, .8
            p.history = [(float(t), np.array([9000.+6*t, 0., 0.]), np.array([6., 0., 0.])) for t in range(21)]
            observation = np.array([[192., 0., 0.], [186., -6., 12.], [17.2, 0., 0.]])
            scene = p.observe_scene(20., np.array([9120., 0., 0.]), np.array([6., 0., 0.]),
                np.array([10300., 0., 0.]), np.zeros(3), [observation], None)
            centers.append(scene[0][0, 0])
        self.assertAlmostEqual(centers[0], 175.2)
        self.assertAlmostEqual(centers[1], 180.)


if __name__ == '__main__':
    unittest.main()
