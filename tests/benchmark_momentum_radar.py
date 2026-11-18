"""Compare radar outputs and processing time against the original Momentum."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np
from controller_loader import load_namespace


def compare(source, baseline, sweeps=80):
    planners = [load_namespace(text)['ShootingNavigator'](launch=False)
                for text in (baseline, source)]
    rng = np.random.default_rng(20261118)
    origin = np.array([8000., 0., 0.])
    positions = origin+rng.uniform([-50., -400., -400.], [1300., 400., 400.], (500, 3))
    velocities = rng.uniform(-.3, .3, (500, 3))
    diameters = rng.uniform(5., 30., 500)
    reports = np.zeros((500, 3, 3))
    timings = [[], []]
    for sweep in range(sweeps):
        now = sweep*.5
        location = origin+np.array([3.*now, 0., 0.])
        measured = np.arange(500) if sweep == 0 else (np.arange(60)+60*sweep) % 500
        for i in measured:
            relative = positions[i]+now*velocities[i]-location
            direction = relative/np.linalg.norm(relative)
            reports[i] = [relative+rng.normal(0., 2., 3),
                          [np.linalg.norm(relative), (velocities[i]-[3., 0., 0.])@direction,
                           diameters[i]], [now, 0., 0.]]
        ordered = sorted(reports.copy(), key=lambda item: np.linalg.norm(item[0]))
        results = [None, None]
        # Alternate call order to reduce first-call timing bias.
        for index in ((0, 1) if sweep % 2 else (1, 0)):
            start = time.perf_counter()
            results[index] = planners[index].obstacles(now, location, np.array([3., 0., 0.]), ordered)
            timings[index].append(time.perf_counter()-start)
        for actual, expected in zip(results[1], results[0]):
            np.testing.assert_array_equal(actual, expected)
        for name in ('radar_forecast_rates', 'radar_track_confidence', 'radar_core_radii'):
            np.testing.assert_array_equal(getattr(planners[1], name), getattr(planners[0], name))
    return dict(sweeps=sweeps, reports_per_sweep=500, outputs_identical=True,
                baseline_median_ms=float(np.median(timings[0])*1000.),
                selected_median_ms=float(np.median(timings[1])*1000.),
                baseline_total_s=sum(timings[0]), selected_total_s=sum(timings[1]))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='tests/momentum_radar_comparison.json')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source = (root/'Controller.py').read_text()
    baseline = subprocess.check_output(['git', 'show', '4a8507e:Controller.py'], cwd=root, text=True)
    result = dict(controller_sha256=hashlib.sha256(source.encode()).hexdigest(),
                  baseline_ref='4a8507e', result=compare(source, baseline))
    (root/args.output).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
