import datetime
from types import SimpleNamespace
import unittest
import numpy as np
from test_controller import namespace, Planner

Perception = namespace['CameraPerception']
History = namespace['PoseHistory']
Manager = namespace['CameraManager']
HEIGHT, WIDTH = namespace['CAMERA_HEIGHT'], namespace['CAMERA_WIDTH']


def empty_radar():
    return np.empty((0, 3)), np.empty((0, 3)), np.empty(0)


def depth_patch(distance=60., row=None, col=None):
    result = np.zeros((HEIGHT, WIDTH))
    row = HEIGHT//2-2 if row is None else row
    col = WIDTH//2-2 if col is None else col
    result[row:row+4, col:col+4] = distance*100.
    return result


class PerceptionTests(unittest.TestCase):
    def test_non_square_projection_units_and_camera_offset(self):
        perception = Perception()
        self.assertTrue(perception.ingest('top', 'depth', depth_patch(100.), 0., 0.,
                                         (np.zeros(3), np.eye(3)), 0., 0.))
        centers, _, _ = perception.obstacles(0., np.zeros(3), *empty_radar())
        np.testing.assert_allclose(centers[0], [101.27, 0., .9], atol=.01)
        # Right-hand image pixels must project toward camera +Y; upper pixels +Z.
        rays = perception.rays(np.array([0.]), np.array([WIDTH-1.]), WIDTH, HEIGHT, 80.)
        self.assertGreater(rays[0, 1], 1.)
        self.assertGreater(rays[0, 2], 0.)

    def test_body_rotation_and_gimbal_projection(self):
        perception = Perception()
        rotation = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
        perception.ingest('bottom', 'depth', depth_patch(60.), 0., 0.,
                          (np.array([10., 20., 30.]), rotation), 30., 20.)
        centers, _, _ = perception.obstacles(0., np.zeros(3), *empty_radar())
        expected = (np.array([10., 20., 30.]) + rotation.T@np.array([1.675, 0., -1.025])
                    + rotation.T@namespace['camera_rotation'](30., 20.).T@np.array([60., 0., 0.]))
        np.testing.assert_allclose(centers[0], expected, atol=.01)

    def test_invalid_depth_sky_and_missing_pose(self):
        perception = Perception()
        invalid = np.full((HEIGHT, WIDTH), np.inf)
        invalid[0, 0], invalid[0, 1] = -100., np.nan
        perception.ingest('top', 'depth', invalid, 0., 0., (np.zeros(3), np.eye(3)), 0., 0.)
        self.assertEqual(len(perception.obstacles(0., np.zeros(3), *empty_radar())[0]), 0)
        self.assertFalse(perception.ingest('top', 'depth', depth_patch(), 0., 0., None, 0., 0.))
        self.assertFalse(perception.ingest('top', 'depth', depth_patch(), 0., 3.,
                                          (np.zeros(3), np.eye(3)), 0., 0.))

    def test_broad_surface_does_not_create_sphere_around_rover(self):
        perception = Perception()
        perception.ingest('top', 'depth', np.full((HEIGHT, WIDTH), 10000.), 0., 0.,
                          (np.zeros(3), np.eye(3)), 0., 0.)
        centers, _, radii = perception.obstacles(0., np.zeros(3), *empty_radar())
        self.assertGreater(len(centers), 10)
        self.assertTrue((np.linalg.norm(centers, axis=1)-radii > 50.).all())

    def test_camera_occupancy_expires_and_uses_current_location(self):
        perception = Perception()
        perception.ingest('top', 'depth', depth_patch(60.), 0., 0.,
                          (np.zeros(3), np.eye(3)), 0., 0.)
        centers, _, _ = perception.obstacles(1., np.array([10., 0., 0.]), *empty_radar())
        self.assertAlmostEqual(centers[0, 0], 51.27, places=2)
        self.assertEqual(len(perception.obstacles(3., np.zeros(3), *empty_radar())[0]), 0)

    def test_single_empty_depth_frame_does_not_erase_recent_hazard(self):
        perception = Perception()
        pose = np.zeros(3), np.eye(3)
        perception.ingest('top', 'depth', depth_patch(), 0., 0., pose, 0., 0.)
        perception.ingest('top', 'depth', np.zeros((HEIGHT, WIDTH)), .5, .5, pose, 0., 0.)
        self.assertGreater(len(perception.obstacles(.5, np.zeros(3), *empty_radar())[0]), 0)

    def test_rgb_never_invents_range_or_filters_dark_depth(self):
        perception = Perception()
        rgb = np.zeros((HEIGHT, WIDTH, 3))
        rgb[HEIGHT//2-2:HEIGHT//2+2, WIDTH//2-2:WIDTH//2+2] = 180.
        pose = np.zeros(3), np.eye(3)
        perception.ingest('top', 'rgb', rgb, 0., 0., pose, 0., 0.)
        self.assertEqual(len(perception.obstacles(0., np.zeros(3), *empty_radar())[0]), 0)
        radar = (np.array([[60., 0., 0.]]), np.array([[0., 1., 0.]]), np.array([15.]))
        centers, speeds, radii = perception.obstacles(0., np.zeros(3), *radar)
        self.assertEqual(perception.rgb_associations, 1)
        self.assertEqual(len(centers), 1)
        np.testing.assert_allclose(speeds[0], radar[1][0])
        self.assertGreater(radii[0], radar[2][0])
        # A separate dark depth observation remains an obstacle with no RGB match.
        perception.ingest('top', 'rgb', np.zeros_like(rgb), .5, .5, pose, 0., 0.)
        perception.ingest('top', 'depth', depth_patch(), .5, .5, pose, 0., 0.)
        self.assertGreater(len(perception.obstacles(.5, np.zeros(3), *empty_radar())[0]), 0)

    def test_camera_only_hazard_changes_path(self):
        perception = Perception()
        perception.ingest('top', 'depth', depth_patch(60.), 0., 0.,
                          (np.zeros(3), np.eye(3)), 0., 0.)
        _, mode, clearance = Planner().command(0., np.zeros(3), np.array([5., 0., 0.]),
                                              np.array([400., 0., 0.]), np.zeros(3), [], camera=perception)
        self.assertNotEqual(mode, 'GUIDE')
        self.assertTrue(np.isfinite(clearance))

    def test_closed_loop_camera_only_obstacle(self):
        perception, planner = Perception(), Planner()
        loc, vel = np.zeros(3), np.zeros(3)
        obstacle, target = np.array([110., 0., 0.]), np.array([1000., 0., 0.])
        rows, cols = np.indices((HEIGHT, WIDTH))
        rays = Perception.rays(rows.ravel(), cols.ravel(), WIDTH, HEIGHT, namespace['CAMERA_FOV'])
        aa = np.sum(rays*rays, axis=1)
        minimum = np.inf
        for tick in range(600):
            now = tick*.5
            if tick % 2 == 0:
                center = obstacle-loc-np.array([1.27, 0., .9])
                dot = rays@center
                discriminant = dot*dot-aa*(center@center-100.)
                roots = (dot-np.sqrt(np.maximum(discriminant, 0.)))/aa
                image = np.where((discriminant >= 0.) & (roots > 0.), 100.*roots, np.inf)
                perception.ingest('top', 'depth', image.reshape(HEIGHT, WIDTH), now, now,
                                  (loc, np.eye(3)), 0., 0.)
            accel, _, _ = planner.command(now, loc, vel, target, np.zeros(3), [],
                                          camera=perception, cruise_speed=8.)
            loc += .5*vel+.125*accel
            vel += .5*accel
            minimum = min(minimum, np.linalg.norm(obstacle-loc))
            if loc[0] > 200.:
                break
        self.assertGreater(minimum, 13.)
        self.assertGreater(loc[0], 200.)

    def test_pose_interpolation_and_age_rejection(self):
        history = History()
        history.add(0., np.zeros(3), np.eye(3))
        history.add(.1, np.array([1., 0., 0.]), np.eye(3))
        np.testing.assert_allclose(history.sample(.05)[0], [.5, 0., 0.])
        self.assertIsNone(history.sample(2.))
        history.add(-1., np.zeros(3), np.eye(3))
        self.assertEqual(len(history.samples), 1)


class CameraAPI:
    def __init__(self, deliver=True):
        self.OutputMode = SimpleNamespace(Depth_cm='depth', RGB_LDR_sRGB='rgb')
        self.CaptureImageProperties = SimpleNamespace
        self.camera = self
        self.callbacks, self.requests, self.pans = {}, [], []
        self.deliver = deliver
        self.epoch = datetime.datetime(2026, 1, 1)
        self.now = 0.

    def OnImageReceived(self, token, callback):
        self.callbacks[token] = callback

    def CaptureImage(self, camera, props):
        self.requests.append((camera, props))
        if self.deliver:
            self.complete(props.CaptureID, props)
        return props.CaptureID

    def complete(self, token, props, stamp=None):
        stamp = self.now if stamp is None else stamp
        image = SimpleNamespace(properties=props,
            get_timestamp=lambda: SimpleNamespace(as_datetime=lambda: self.epoch+datetime.timedelta(seconds=stamp)),
            as_f32=lambda: SimpleNamespace(Pixels=depth_patch().ravel()),
            as_RGB8=lambda: SimpleNamespace(PixelsR=np.zeros(HEIGHT*WIDTH),
                                           PixelsG=np.zeros(HEIGHT*WIDTH), PixelsB=np.zeros(HEIGHT*WIDTH)))
        self.callbacks.pop(token)(image)


class CaptureTests(unittest.TestCase):
    def make(self, deliver=True):
        api = CameraAPI(deliver)
        manager = Manager(api, dict(top='top', bottom='bottom'),
                          lambda *args: api.pans.append(args), api.epoch)
        return api, manager

    def test_immediate_callback_race_rgb_depth_and_both_cameras(self):
        api, manager = self.make()
        history, perception = History(), Perception()
        for tick in range(16):
            api.now = tick*.25
            history.add(api.now, np.zeros(3), np.eye(3))
            manager.tick(api.now, np.array([1., 0., 0.]))
            manager.drain(api.now, history, perception)
        self.assertEqual(manager.received, 16)
        self.assertEqual(set(camera for camera, _ in api.requests), {'top', 'bottom'})
        self.assertEqual(set(props.output_mode for _, props in api.requests), {'depth', 'rgb'})
        self.assertEqual(len(api.callbacks), 0)

    def test_missing_camera_has_bounded_pending_subscriptions(self):
        api, manager = self.make(False)
        for tick in range(100):
            manager.tick(tick*.25, np.array([1., 0., 0.]))
        self.assertLessEqual(len(api.callbacks), 6)
        self.assertEqual(len(manager.pending), 0)
        self.assertIn('timeout', manager.error)

    def test_camera_api_exception_degrades_without_stopping_control(self):
        api, manager = self.make()
        def broken_pan(*args):
            raise RuntimeError('camera unavailable')
        manager.pan = broken_pan
        for tick in range(12):
            manager.tick(tick*.25, np.array([1., 0., 0.]))
        self.assertEqual(len(api.callbacks), 0)
        self.assertIn('camera unavailable', manager.error)

    def test_late_frame_after_reset_cannot_repopulate_perception(self):
        api, manager = self.make(False)
        manager.tick(0., np.array([1., 0., 0.]))
        _, props = api.requests[0]
        manager.reset()
        api.complete(props.CaptureID, props, stamp=0.)
        self.assertEqual(len(manager.queue), 0)


class EconomyTests(unittest.TestCase):
    def test_coast_band_bypassed_for_emergency(self):
        command = namespace['velocity_command']
        np.testing.assert_allclose(command([.05, 0., 0.], np.zeros(3), .3), np.zeros(3))
        self.assertGreater(command([.05, 0., 0.], np.zeros(3), .3, urgent=True)[0], 0.)

    def test_score_policy_is_finite_bounded_and_observations_are_clamped(self):
        policy = namespace['ScorePolicy']()
        for tick in range(20):
            policy.observe(tick*.5, np.array([tick*.1, 0., 0.]), tick*100.)
            speed = policy.choose(1200.-tick, tick*.1, tick*.5, tick*100.)
            self.assertGreaterEqual(speed, 6.)
            self.assertLessEqual(speed, namespace['MAX_SPEED'])
            self.assertTrue(np.isfinite(policy.estimated_score))
        self.assertLessEqual(policy.fuel_per_dv, 18.)
        self.assertGreaterEqual(policy.acceleration, .06)


if __name__ == '__main__':
    unittest.main()
