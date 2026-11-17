"""Controllable futures for flight-loop tests, without wall-clock sleeps."""
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
