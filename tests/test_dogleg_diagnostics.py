"""Task 1: relative-frame geometry, camera exclusion and persistent run counters."""
import json
import unittest
from unittest import mock
import numpy as np
from test_controller import namespace


class DoglegDiagnosticTests(unittest.TestCase):
    def test_relative_centers_and_strict_slab_boundaries(self):
        loc = np.array([9000., 120., -30.])
        target = loc+np.array([1000., 0., 0.])
        centers = np.array([[100., 0., 3.], [200., 4., 0.], [500., 6., 8.],
                            [50., 999., 0.], [950., 999., 0.], [-5., 999., 0.],
                            [951., 999., 0.]])
        original = centers.copy()
        result = namespace['DoglegNavigator'].lane_distribution(loc, target, centers)
        self.assertEqual(result['slab_count'], 3)
        self.assertAlmostEqual(result['lat_p50_m'], 4.)
        self.assertAlmostEqual(result['lat_p90_m'], 8.8)
        self.assertAlmostEqual(result['lat_p100_m'], 10.)
        np.testing.assert_array_equal(centers, original)
        self.assertEqual(result['centers'], 'shuttle_relative')

    def test_translation_and_rotation_do_not_change_distribution(self):
        loc = np.array([9000., 120., -30.])
        target = loc+np.array([1000., 0., 0.])
        centers = np.array([[100., 0., 3.], [200., 4., 0.], [500., 6., 8.]])
        rotation = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
        shift = np.array([-5000., 330., 90.])
        distribution = namespace['DoglegNavigator'].lane_distribution
        first = distribution(loc, target, centers)
        second = distribution(rotation@loc+shift, rotation@target+shift, centers@rotation.T)
        for key in ('slab_count', 'lat_p50_m', 'lat_p90_m', 'lat_p100_m'):
            self.assertEqual(first[key], second[key])

    def test_empty_or_short_slab_has_json_null_percentiles(self):
        distribution = namespace['DoglegNavigator'].lane_distribution
        for target, centers in (([1000., 0., 0.], []), ([100., 0., 0.], [[70., 2., 0.]])):
            result = distribution(np.zeros(3), target, centers)
            self.assertEqual(result['slab_count'], 0)
            self.assertIsNone(result['lat_p90_m'])
            json.dumps(result, allow_nan=False)

    def test_failed_search_excludes_camera_and_counts_across_record_flush(self):
        p = namespace['DoglegNavigator'](launch=False)
        p.lane_aligned = True
        p.control_time = 15.
        loc, target = np.array([9000., 0., 0.]), np.array([10300., 0., 0.])
        p.latest_radar_count = 1
        centers = np.array([[200., 6., 8.], [200., 0., 1000.]])
        scene = centers, np.zeros((2, 3)), np.ones(2), np.ones(2), np.zeros((2, 3))
        with mock.patch.dict(namespace, {'dogleg_clearances': lambda plans, *args, **kw: np.full(len(plans['eta']), -1.)}):
            for count in (1, 2):
                self.assertFalse(p.select_lane(15., loc, np.array([.8, 0., 0.]), target, np.zeros(3), scene))
                entry = p.lane_records[-1]
                self.assertEqual(entry['no_safe_lane_events'], count)
                self.assertEqual(entry['debris_distribution']['slab_count'], 1)
                self.assertEqual(entry['debris_distribution']['lat_p100_m'], 10.)
                p.lane_records.clear()
        summary = p.lane_summary(20., fuel=30., record=True)
        self.assertEqual(summary['searches'], 2)
        self.assertEqual(summary['no_safe_lane_events'], 2)
        self.assertIsNone(summary['latest_selected_route'])
        self.assertEqual(len(p.lane_records), 1)

    def test_fuel_snapshot_updates_only_an_active_route(self):
        p = namespace['DoglegNavigator'](launch=False)
        p.lane_last_selected = dict(fuel_at_selection_kg=10., actual_incremental_fuel_kg=3.,
                                   predicted_incremental_fuel_kg=8., waypoint=[1., 2., 3.])
        p.lane = {}
        active = p.lane_summary(100., fuel=15.)
        self.assertEqual(active['latest_selected_route']['actual_incremental_fuel_kg'], 5.)
        self.assertEqual(p.lane_last_selected['actual_incremental_fuel_kg'], 3.)
        p.lane = None
        closed = p.lane_summary(200., fuel=99., record=True)
        self.assertEqual(closed['latest_selected_route']['actual_incremental_fuel_kg'], 3.)
        self.assertEqual(closed['latest_selected_route']['predicted_incremental_fuel_kg'], 8.)
        json.dumps(closed, allow_nan=False)
