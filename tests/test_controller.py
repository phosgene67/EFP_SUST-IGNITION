"""Offline planner checks; AST extraction deliberately avoids connecting to the sim."""
import ast
from pathlib import Path
import unittest
import numpy as np

SOURCE = Path(__file__).resolve().parents[1] / 'Controller.py'
tree = ast.parse(SOURCE.read_text())
nodes = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))
         and n.name in ('limit_vector', 'QualificationPlanner')]
nodes += [n for n in tree.body if isinstance(n, ast.Assign)
          and any(isinstance(t, ast.Name) and t.id in
                  ('MAX_SPEED', 'MAX_ACCEL', 'BRAKE_ACCEL', 'VELOCITY_GAIN', 'HORIZON', 'PLAN_PERIOD')
                  for t in n.targets)]
namespace = {'np': np}
exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), 'exec'), namespace)
Planner = namespace['QualificationPlanner']


class PlannerTests(unittest.TestCase):
    def test_clear_route_converges(self):
        planner = Planner()
        loc, vel = np.zeros(3), np.zeros(3)
        target = np.array([300., 0., 0.])
        for tick in range(1200):
            accel, _, _ = planner.command(tick*.25, loc, vel, target, np.zeros(3), [])
            self.assertLessEqual(np.linalg.norm(accel), .250001)
            loc += .25*vel + .5*.25**2*accel
            vel += .25*accel
            if np.linalg.norm(target-loc) < 45:
                break
        self.assertLess(np.linalg.norm(target-loc), 45)

    def test_obstacle_changes_route(self):
        report = np.array([[60., 0., 0.], [50., -4., 20.], [0., 0., 0.]])
        accel, mode, _ = Planner().command(0., np.zeros(3), np.array([4., 0., 0.]),
                                         np.array([300., 0., 0.]), np.zeros(3), [report])
        self.assertNotEqual(mode, 'GUIDE')
        self.assertTrue(np.isfinite(accel).all())

    def test_stale_stationary_report_corrects_shuttle_motion(self):
        planner = Planner()
        report = np.array([[100., 0., 0.], [90., -2., 20.], [0., 0., 0.]])
        planner.obstacles(0., np.zeros(3), np.array([2., 0., 0.]), [])
        centers, _, _ = planner.obstacles(5., np.array([10., 0., 0.]),
                                         np.array([2., 0., 0.]), [report])
        np.testing.assert_allclose(centers[0], [90., 0., 0.])

    def test_closed_loop_static_obstacle(self):
        planner = Planner()
        loc, vel = np.zeros(3), np.zeros(3)
        target, obstacle = np.array([300., 0., 0.]), np.array([120., 0., 0.])
        minimum = np.inf
        for tick in range(1200):
            relative = obstacle-loc
            distance = np.linalg.norm(relative)
            report = np.array([relative, [distance-10., -vel.dot(relative/distance), 20.],
                               [tick*.25, 0., 0.]])
            accel, _, _ = planner.command(tick*.25, loc, vel, target, np.zeros(3), [report])
            loc += .25*vel + .5*.25**2*accel
            vel += .25*accel
            minimum = min(minimum, np.linalg.norm(obstacle-loc))
            if np.linalg.norm(target-loc) < 45:
                break
        self.assertGreater(minimum, 13.)  # 10 m obstacle + 3 m shuttle
        self.assertLess(np.linalg.norm(target-loc), 45.)

    def test_degenerate_and_invalid_reports(self):
        accel, _, _ = Planner().command(0., np.zeros(3), np.zeros(3), np.zeros(3),
                                       np.zeros(3), [np.full((3, 3), np.nan), np.zeros((3, 3))])
        np.testing.assert_allclose(accel, np.zeros(3))

    def test_transverse_velocity_tracking_and_repeated_measurement(self):
        planner = Planner()
        speed = np.array([0., 1., 0.])
        last = None
        for stamp in range(0, 21, 2):
            position = np.array([100., float(stamp), 0.])
            report = np.array([position, [np.linalg.norm(position)-10.,
                               speed.dot(position/np.linalg.norm(position)), 20.], [stamp, 0., 0.]])
            _, estimate, _ = planner.obstacles(float(stamp), np.zeros(3), np.zeros(3), [report])
            last = report
        self.assertGreater(estimate[0, 1], .9)
        count = planner.track_counter
        planner.obstacles(20.5, np.zeros(3), np.zeros(3), [last])
        self.assertEqual(planner.track_counter, count)

    def test_sorting_does_not_swap_tracks(self):
        planner = Planner()
        for stamp in range(0, 11, 2):
            reports = []
            for sign, diameter in ((1, 20.), (-1, 40.)):
                position = np.array([100., sign*(50.+stamp), 0.])
                speed = np.array([0., float(sign), 0.])
                reports.append(np.array([position, [1., speed.dot(position/np.linalg.norm(position)), diameter],
                                         [stamp, 0., 0.]]))
            if stamp % 4:
                reports.reverse()
            _, speeds, _ = planner.obstacles(float(stamp), np.zeros(3), np.zeros(3), reports)
        for report, estimate in zip(reports, speeds):
            sign = 1 if report[1, 2] == 20. else -1
            self.assertGreater(sign*estimate[1], .7)

    def test_noisy_delayed_crossing_debris(self):
        for seed in (7, 19, 43):
            with self.subTest(seed=seed):
                rng = np.random.default_rng(seed)
                planner = Planner()
                loc, vel = np.zeros(3), np.zeros(3)
                target = np.array([350., 0., 0.])
                positions = np.array([[140., 35., 0.], [220., -40., 15.], [100., 80., -30.]])
                speeds = np.array([[0., -.6, 0.], [0., .3, -.1], [.1, -.2, 0.]])
                minimum = np.inf
                for tick in range(600):
                    now = tick*.5
                    if tick % 3 == 0:
                        reports = []
                        for pos, speed in zip(positions, speeds):
                            relative = pos-loc
                            distance = np.linalg.norm(relative)
                            reports.append(np.array([
                                relative+rng.normal(0., 2., 3),
                                [distance-10., (speed-vel).dot(relative/distance)+rng.normal(0., .02),
                                 20.*rng.normal(1., .05)], [now, 0., 0.]]))
                        rng.shuffle(reports)
                    accel, _, _ = planner.command(now, loc, vel, target, np.zeros(3), reports)
                    # Small unmodelled gravity and actuator under-delivery.
                    actual = .9*accel + np.array([0., 0., -.003])
                    loc += .5*vel + .125*actual
                    vel += .5*actual
                    positions += .5*speeds
                    minimum = min(minimum, np.linalg.norm(positions-loc, axis=1).min())
                    if np.linalg.norm(target-loc) < 45.:
                        break
                self.assertGreater(minimum, 13.)
                self.assertLess(np.linalg.norm(target-loc), 45.)

    def test_escape_from_inflated_overlap(self):
        # Physical spheres do not intersect, but their uncertainty margins do.
        report = np.array([[15., 0., 0.], [5., 0., 20.], [0., 0., 0.]])
        accel, mode, _ = Planner().command(0., np.zeros(3), np.zeros(3),
                                         np.array([300., 0., 0.]), np.zeros(3), [report])
        self.assertEqual(mode, 'ESCAPE')
        self.assertLess(accel[0], 0.)

    def test_fast_incoming_obstacle_not_filtered_out(self):
        report = np.array([[350., 0., 0.], [340., -15., 20.], [0., 0., 0.]])
        _, mode, clearance = Planner().command(0., np.zeros(3), np.zeros(3),
                                              np.array([500., 0., 0.]), np.zeros(3), [report])
        self.assertNotEqual(mode, 'GUIDE')
        self.assertTrue(np.isfinite(clearance))

    def test_body_connection_prefix_unchanged(self):
        # Official sibling starter shares the protected connection/helper section.
        marker = '# EXAMPLE USER CODE'
        starter = SOURCE.with_name('BasicController.py').read_text()
        self.assertEqual(SOURCE.read_text().split(marker)[0], starter.split(marker)[0])


if __name__ == '__main__':
    unittest.main()
