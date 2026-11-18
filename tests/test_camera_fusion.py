import datetime
from types import SimpleNamespace
import unittest
import numpy as np
from test_controller import namespace, Planner

Perception = namespace['CameraPerception']
History = namespace['PoseHistory']
Manager = namespace['CameraManager']
HEIGHT, WIDTH = namespace['CAMERA_HEIGHT'], namespace['CAMERA_WIDTH']


class ForecastAssociationTests(unittest.TestCase):
    def test_below_launch_surface_does_not_borrow_a_passing_debris_velocity(self):
        camera = Perception()
        camera.occupied['top'] = (0., [(np.array([5., 0., -3.]), 3.)])
        centers, speeds, radii = np.array([[9., 0., -1.]]), np.array([[1., 0., 0.]]), np.array([6.])
        _, ordinary_speed, _ = camera.obstacles(0., np.zeros(3), centers, speeds, radii)
        np.testing.assert_array_equal(ordinary_speed[0], speeds[0])
        camera.ground_reference = (np.zeros(3), np.array([0., 0., 1.]))
        bound, stationary, size = camera.obstacles(0., np.zeros(3), centers, speeds, radii)
        np.testing.assert_array_equal(stationary[0], np.zeros(3))
        np.testing.assert_array_equal(bound[0], [5., 0., -3.])
        self.assertEqual(size[0], 3.)
        np.testing.assert_array_equal(speeds[0], [1., 0., 0.])

    def test_camera_uses_only_a_mature_unique_track_without_removing_geometry(self):
        planner = namespace['ShootingNavigator'](launch=False)
        planner.radar_track_confidence = np.array([True])
        planner.radar_forecast_rates = np.array([.3])
        planner.obstacles = lambda *args: (np.array([[100., 0., 0.]]), np.array([[0., .2, 0.]]), np.array([14.]))
        camera = SimpleNamespace(
            last_radar_sources=np.array([0, -1]),
            observes=lambda *args: False,
            obstacles=lambda *args: (np.array([[100., 2., 0.], [250., 0., 0.]]),
                np.array([[0., .2, 0.], [0., 0., 0.]]), np.array([7., 9.])))
        args = (10., np.array([9000., 0., 0.]), np.zeros(3), np.array([10300., 0., 0.]), np.zeros(3), [], camera)
        centers, _, radii, rates, _, _ = planner.observe_scene(*args)
        self.assertEqual(len(centers), 3)
        np.testing.assert_array_equal(radii, [14., 7., 9.])
        np.testing.assert_allclose(rates, [.3, .3, .6])
        planner.radar_track_confidence[0] = False
        _, _, _, rates, _, _ = planner.observe_scene(*args)
        np.testing.assert_allclose(rates, [.3, .6, .6])


def empty_radar():
    return np.empty((0, 3)), np.empty((0, 3)), np.empty(0)


def depth_patch(distance=60., row=None, col=None):
    result = np.zeros((HEIGHT, WIDTH))
    row = HEIGHT//2-2 if row is None else row
    col = WIDTH//2-2 if col is None else col
    result[row:row+4, col:col+4] = distance*100.
    return result


class PerceptionTests(unittest.TestCase):
    def test_far_camera_bearings_refine_only_well_resolved_unique_radar_motion(self):
        camera = Perception()
        center = np.array([[500., 0., 0.]])
        history = []
        for t in (0., 2., 4., 6.):
            origin = np.array([t, 0., 0.])
            ray = center[0]+[0., 2.*(t-6.), 0.]-origin
            history.append((t, origin, (ray/np.linalg.norm(ray))[None]))
        camera.rgb_history['top'] = history
        motion = camera.refine_motion(6., np.zeros(3), center, np.zeros((1, 3)), np.array([12.]))
        self.assertAlmostEqual(motion[0, 0], 0.)
        self.assertAlmostEqual(motion[0, 1], .5, places=2)
        duplicate = np.repeat(center, 2, axis=0)
        np.testing.assert_array_equal(camera.refine_motion(6., np.zeros(3), duplicate,
            np.zeros((2, 3)), np.full(2, 12.)), np.zeros((2, 3)))
    def test_depth_frames_refine_transverse_speed_after_compensating_rover_motion(self):
        perception = Perception()
        # Moving the rover alongside the object keeps the patch at the same pixel.
        for stamp in (0., 1.):
            pose = (np.array([0., stamp, 0.]), np.eye(3))
            perception.ingest('top', 'depth', depth_patch(60.), stamp, stamp, pose, 0., 0.)
        centers, speed = np.array([[61., 0., 0.]]), np.array([[.25, 0., 0.]])
        refined = perception.refine_motion(1., np.zeros(3), centers, speed, np.array([12.]))
        np.testing.assert_allclose(refined, [[.25, .2, 0.]])
        ambiguous = perception.refine_motion(1., np.zeros(3), centers.repeat(2, 0),
                                             speed.repeat(2, 0), np.array([12., 12.]))
        np.testing.assert_array_equal(ambiguous, speed.repeat(2, 0))
        np.testing.assert_array_equal(perception.refine_motion(10., np.zeros(3), centers, speed, [12.]), speed)

    def test_non_square_projection_units_and_camera_offset(self):
        perception = Perception()
        self.assertTrue(perception.ingest('top', 'depth', depth_patch(100.), 0., 0.,
                                         (np.zeros(3), np.eye(3)), 0., 0.))
        centers, _, _ = perception.obstacles(0., np.zeros(3), *empty_radar())
        np.testing.assert_allclose(centers.mean(axis=0), [101.27, 0., .9], atol=.01)
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
        np.testing.assert_allclose(centers.mean(axis=0), expected, atol=.01)

    def test_fine_depth_tiles_retain_all_pixel_surface_and_rover_pose_margins(self):
        perception = Perception()
        pixels = depth_patch(100.)
        perception.ingest('top', 'depth', pixels, 0., 0., (np.zeros(3), np.eye(3)), 0., 0.)
        centers, _, radii = perception.obstacles(0., np.zeros(3), *empty_radar())
        rows, cols = np.nonzero(pixels)
        surfaces = np.array([1.27, 0., .9])+100.*perception.rays(rows, cols, WIDTH, HEIGHT, 80.)
        gaps = np.linalg.norm(surfaces[:, None]-centers[None], axis=2)-(radii[None]-6.)
        self.assertTrue((gaps.min(axis=1) <= 0.).all())
        self.assertGreater(len(centers), 1)

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
