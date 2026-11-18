"""Isolation and mission-transition checks for nonblocking flight planning."""
import unittest
import threading
import numpy as np
from test_controller import namespace
from planning_doubles import ManualExecutor, PlanningWorker


class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.executor = ManualExecutor()
        self.worker = PlanningWorker(self.executor)
        self.planner = namespace['ShootingNavigator'](launch=False)
        self.planner.launch.done = True
        self.loc, self.vel, self.goal = np.array([9000., 0., 0.]), np.zeros(3), np.array([10300., 0., 0.])
        self.camera = namespace['CameraPerception']()

    def submit(self):
        return self.worker.submit(10., self.planner, self.camera, self.loc, self.vel,
                                  self.goal, np.zeros(3), [], 10.)

    def test_pending_job_does_not_prevent_servo_cutoff(self):
        p = self.planner
        p.shooting_active, p.burn_target = True, np.array([.1, 0., 0.])
        p.burn_signs, p.axis_done = np.array([1., 0., 0.]), np.array([False, True, True])
        self.assertTrue(self.submit())
        self.assertIsNone(self.worker.take(10.1, p, self.camera))
        self.assertGreater(np.linalg.norm(p.linear_command(np.zeros(3), .5)), 0.)
        np.testing.assert_array_equal(p.linear_command(np.array([.101, 0., 0.]), .5), np.zeros(3))
        self.assertFalse(self.submit())  # No backlog of stale planning jobs.

    def test_worker_snapshot_does_not_mutate_live_state_or_camera(self):
        self.submit()
        self.executor.finish()
        self.assertFalse(self.planner.shooting_active)
        self.assertEqual(self.camera.depth_associations, 0)
        self.assertIsNotNone(self.worker.take(10.25, self.planner, self.camera))

    def test_adoption_keeps_new_telemetry_and_latest_pose(self):
        self.submit()
        self.planner.policy.previous = (10.2, self.vel.copy(), 12.)
        self.planner.dynamics.residual[:] = [.001, 0., .002]
        self.planner.body_rotation = np.diag([-1., -1., 1.])
        self.executor.finish()
        adopted, _, _ = self.worker.take(10.25, self.planner, self.camera)
        self.assertIs(adopted.dynamics, self.planner.dynamics)
        self.assertEqual(adopted.policy.previous[2], 12.)
        np.testing.assert_array_equal(adopted.body_rotation, self.planner.body_rotation)
        self.assertEqual(adopted.control_time, 10.25)

    def test_completed_old_plan_cannot_undo_arrival_or_failure(self):
        for transition in ('arrival', 'failure'):
            with self.subTest(transition=transition):
                self.setUp()
                self.submit()
                if transition == 'arrival':
                    self.planner.observe_arrival(10.1, self.loc, self.vel, self.loc+49., np.zeros(3), 49.)
                else:
                    self.planner.observe_damage(210000.)
                self.executor.finish()
                self.assertIsNone(self.worker.take(10.25, self.planner, self.camera))

    def test_reset_rejects_an_already_running_old_epoch(self):
        self.submit()
        self.worker.future.set_running_or_notify_cancel()
        self.worker.reset()
        self.executor.finish()
        self.assertIsNone(self.worker.take(10.25, self.planner, self.camera))
        self.assertTrue(self.submit())

    def test_stale_job_cannot_be_used_as_a_fresh_command(self):
        self.submit()
        self.executor.finish()
        self.assertIsNone(self.worker.take(11.01, self.planner, self.camera))
        self.assertEqual(self.worker.last_accepted, -np.inf)

    def test_worker_error_is_reported_and_allows_retry(self):
        self.submit()
        self.worker.future.set_exception(RuntimeError('test failure'))
        self.assertIsNone(self.worker.take(10.25, self.planner, self.camera))
        self.assertIn('test failure', self.worker.error)
        self.assertTrue(self.submit())

    def test_real_math_thread_leaves_the_live_servo_available(self):
        started, release = threading.Event(), threading.Event()
        thread_ids = []
        base = namespace['ShootingNavigator']
        class SlowPlanner(base):
            def command(self, *args, **kwargs):
                thread_ids.append(threading.get_ident())
                started.set()
                if not release.wait(2.):
                    raise RuntimeError('test release timed out')
                return super().command(*args, **kwargs)
        live = SlowPlanner(launch=False)
        live.launch.done = True
        live.shooting_active = True
        live.burn_target = np.array([.1, 0., 0.])
        live.burn_signs, live.axis_done = np.array([1., 0., 0.]), np.array([False, True, True])
        worker = PlanningWorker()
        try:
            worker.submit(10., live, self.camera, self.loc, self.vel, self.goal, np.zeros(3), [], 10.)
            self.assertTrue(started.wait(1.))
            self.assertNotEqual(thread_ids[0], threading.get_ident())
            np.testing.assert_array_equal(live.linear_command(np.array([.101, 0., 0.]), .5), np.zeros(3))
            release.set()
            worker.future.result(timeout=2.)
            self.assertIsNotNone(worker.take(10.25, live, self.camera))
        finally:
            release.set()
            worker.close()
