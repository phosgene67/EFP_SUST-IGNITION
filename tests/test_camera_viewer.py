import importlib.util
import tempfile
import time
from pathlib import Path
import unittest
from unittest import mock
import numpy as np
import camera_viewer as viewer
from test_controller import namespace, Planner
from test_camera_fusion import CameraAPI, History, Perception, Manager


class PublisherTests(unittest.TestCase):
    def test_all_four_streams_and_metadata_survive_atomic_transport(self):
        with tempfile.TemporaryDirectory() as directory:
            publisher = viewer.PreviewPublisher(launch=False, directory=directory)
            for name in ('top', 'bottom'):
                for kind in ('rgb', 'depth'):
                    pixels = np.zeros((64, 96, 3), np.uint8) if kind == 'rgb' else np.full((64, 96), 2500.)
                    publisher.submit(name, kind, pixels, 1., [dict(xyxy=[1, 2, 3, 4], range_m=25.)], True)
            publisher.update(now=2., speed=12., clearance=np.inf)
            publisher.close()
            frames, metadata = viewer.read_snapshot(publisher.source)
            self.assertEqual(len(frames), 4)
            self.assertEqual(metadata['status']['speed'], 12.)
            self.assertIsNone(metadata['status']['clearance'])
            self.assertFalse(metadata['status']['running'])
            self.assertEqual(metadata['frames']['top_depth']['boxes'][0]['range_m'], 25.)
            self.assertEqual(frames['top_rgb'].dtype, np.uint8)

    def test_reset_removes_previous_run_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            publisher = viewer.PreviewPublisher(launch=False, directory=directory)
            publisher.submit('top', 'depth', np.ones((64, 96)), 1., [], True)
            publisher.reset()
            publisher.close()
            self.assertEqual(len(viewer.read_snapshot(publisher.source)[0]), 0)

    def test_viewer_exit_is_reported_without_stopping_publisher(self):
        with tempfile.TemporaryDirectory() as directory, mock.patch('subprocess.Popen') as popen:
            popen.return_value.poll.return_value = 1
            publisher = viewer.PreviewPublisher(launch=True, directory=directory)
            publisher.update(now=2.)
            self.assertIn('flight continues', publisher.error)
            publisher.close()

    def test_capture_pipeline_forwards_actual_frames_and_detection_boxes(self):
        api = CameraAPI()
        with tempfile.TemporaryDirectory() as directory:
            publisher = viewer.PreviewPublisher(launch=False, directory=directory)
            manager = Manager(api, dict(top='top', bottom='bottom'), lambda *args: None, api.epoch, publisher)
            history, perception = History(), Perception()
            history.add(0., np.zeros(3), np.eye(3))
            manager.tick(0., np.array([1., 0., 0.]))
            manager.drain(0., history, perception)
            publisher.close()
            frames, metadata = viewer.read_snapshot(publisher.source)
            self.assertIn('top_depth', frames)
            self.assertGreater(len(metadata['frames']['top_depth']['boxes']), 0)


@unittest.skipUnless(importlib.util.find_spec('cv2'), 'OpenCV renderer tests use the simulator Python environment')
class RendererTests(unittest.TestCase):
    def test_dashboard_dimensions_colors_and_real_region_annotations(self):
        frames, metadata = viewer.demo_snapshot()
        canvas = viewer.render_dashboard(frames, metadata)
        self.assertEqual(canvas.shape, (950, 1360, 3))
        self.assertEqual(canvas.dtype, np.uint8)
        without_boxes = dict(metadata, frames={key: dict(info, boxes=[]) for key, info in metadata['frames'].items()})
        self.assertGreater(np.count_nonzero(canvas != viewer.render_dashboard(frames, without_boxes)), 1000)

    def test_invalid_depth_is_black_and_distance_changes_color(self):
        result = viewer.depth_bgr(np.array([[np.inf, np.nan, -1., 1000., 10000.]]))
        np.testing.assert_allclose(result[0, :3], 0.)
        self.assertFalse(np.array_equal(result[0, 3], result[0, 4]))

    def test_waiting_and_rejected_frames_can_be_rendered(self):
        waiting = viewer.render_dashboard({}, dict(status={}, frames={}))
        frames, metadata = viewer.demo_snapshot()
        metadata['frames']['top_depth']['accepted'] = False
        metadata['status']['now'] = 50.
        self.assertEqual(viewer.render_dashboard(frames, metadata).shape, waiting.shape)


class FastProfileTests(unittest.TestCase):
    def test_speed_cap_and_requested_cruise_use_fast_envelope(self):
        self.assertEqual(namespace['MAX_SPEED'], 25.)
        policy = namespace['ScorePolicy']()
        for tick in range(30):
            selected = policy.choose(5000., 15., tick*.5, 0.)
            self.assertGreaterEqual(selected, 18.)
        self.assertGreater(policy.choose(5000., 15., 30., 0., maneuvering=True), selected)

    def test_twenty_five_m_s_has_explicit_stopping_distance(self):
        planner = Planner()
        acceleration, _, _ = planner.command(0., np.zeros(3), np.array([25., 0., 0.]),
                                             np.array([20000., 0., 0.]), np.zeros(3), [])
        self.assertGreaterEqual(planner.stopping_distance, 25.**2/(2.*.10))
        self.assertLessEqual(planner.last_horizon, 15)
        self.assertTrue(np.isfinite(acceleration).all())


if __name__ == '__main__':
    unittest.main()
