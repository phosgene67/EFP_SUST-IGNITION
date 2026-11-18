"""Controllable futures for flight-loop tests, without wall-clock sleeps."""
import numpy as np
from concurrent.futures import Future


class ManualExecutor:
    def __init__(self, **kwargs):
        self.jobs = []

    def submit(self, function, *args):
        future = Future()
        self.jobs.append((future, function, args))
        return future

    def finish(self):
        future, function, args = self.jobs.pop(0)
        if future.cancelled():
            return
        if not future.running():
            future.set_running_or_notify_cancel()
        try:
            future.set_result(function(*args))
        except Exception as exc:
            future.set_exception(exc)

    def shutdown(self, **kwargs):
        for future, _, _ in self.jobs:
            future.cancel()


class ImmediateExecutor(ManualExecutor):
    def submit(self, function, *args):
        future = super().submit(function, *args)
        self.finish()
        return future


class PlanningWorker:
    """One isolated math job; the flight thread alone owns the simulator API."""
    def __init__(self, executor=None):
        from concurrent.futures import ThreadPoolExecutor
        self.executor = executor or ThreadPoolExecutor(max_workers=1, thread_name_prefix='efp-plan')
        self.future = None
        self.generation = 0
        self.last_accepted = -np.inf
        self.plan_ms = self.copy_ms = 0.
        self.error = ''

    @property
    def busy(self):
        return self.future is not None

    @staticmethod
    def compute(generation, stamp, planner, camera, vectors, radar, cruise):
        import time as clock
        start = clock.perf_counter()
        _, mode, clearance = planner.command(stamp, *vectors, radar, camera=camera, cruise_speed=cruise)
        return generation, stamp, planner, camera, mode, clearance, 1000.*(clock.perf_counter()-start)

    def submit(self, stamp, planner, camera, loc, vel, goal, goal_velocity, radar, cruise):
        import copy
        import time as clock
        if self.busy:
            return False
        start = clock.perf_counter()
        snapshot, image = copy.deepcopy(planner), copy.deepcopy(camera)
        vectors = tuple(np.asarray(v, float).copy() for v in (loc, vel, goal, goal_velocity))
        reports = [np.asarray(report, float).copy() for report in radar]
        self.copy_ms = 1000.*(clock.perf_counter()-start)
        self.future = self.executor.submit(self.compute, self.generation, float(stamp),
                                           snapshot, image, vectors, reports, float(cruise))
        return True

    def take(self, now, live, camera):
        if self.future is None or not self.future.done():
            return None
        future, self.future = self.future, None
        try:
            generation, stamp, candidate, image, mode, clearance, elapsed = future.result()
        except Exception as exc:
            self.error = 'planning failed: '+str(exc)[:160]
            return None
        self.plan_ms = elapsed
        if generation != self.generation or not 0. <= now-stamp <= 1.:
            self.error = 'discarded obsolete plan'
            return None
        crossed_budget_boundary = getattr(live, 'inner_leg', False) and not getattr(candidate, 'inner_leg', False)
        if live.failed or candidate.arrival_time != live.arrival_time or crossed_budget_boundary:
            self.error = 'discarded plan across mission transition'
            return None
        # These observations continue arriving while the math job runs. Keep
        # their live history rather than restoring its older snapshot.
        candidate.dynamics = live.dynamics
        for name in ('previous', 'fuel_per_dv', 'acceleration'):
            setattr(candidate.policy, name, getattr(live.policy, name))
        candidate.observe_damage(max(live.last_damage, candidate.last_damage))
        candidate.control_time = float(now)
        candidate.body_rotation = live.body_rotation.copy()
        if camera is not None and image is not None:
            camera.rgb_associations, camera.depth_associations = image.rgb_associations, image.depth_associations
        self.last_accepted, self.error = float(stamp), ''
        return candidate, mode, clearance

    def reset(self):
        self.generation += 1
        self.last_accepted, self.error = -np.inf, ''
        if self.future is not None and self.future.cancel():
            self.future = None

    def close(self):
        self.generation += 1
        self.executor.shutdown(wait=False, cancel_futures=True)
