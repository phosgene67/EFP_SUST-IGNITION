"""Compare planners with repeatable synthetic physics, not Space Teams scoring.

Run from the repository root with --baseline-ref <git commit>. Reported impulse
is an ideal translation-only fuel proxy: it excludes jet geometry and rotation.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import time
import numpy as np


def load_planner(source):
    tree = ast.parse(source)
    nodes = [n for n in tree.body if
             (isinstance(n, (ast.FunctionDef, ast.ClassDef)) and
              n.name in ('limit_vector', 'QualificationPlanner')) or
             (isinstance(n, ast.Assign) and all(isinstance(t, ast.Name) and t.id.isupper()
                                              for t in n.targets))]
    namespace = {'np': np}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), '<planner>', 'exec'), namespace)
    return namespace['QualificationPlanner']


def run_case(planner_type, seed, delivery):
    rng = np.random.default_rng(seed)
    planner = planner_type()
    loc, vel = np.zeros(3), np.zeros(3)
    target = np.array([1305.8, 0., 0.])
    if seed == 0:
        positions, speeds = np.empty((0, 3)), np.empty((0, 3))
    else:
        positions = np.column_stack((np.linspace(150., 1100., 8), rng.uniform(-60., 60., 8),
                                     rng.uniform(-40., 40., 8)))
        speeds = rng.uniform(-.3, .3, (8, 3))
    impulse, peak, minimum = 0., 0., np.inf
    durations = []
    reports = []
    success = False
    for tick in range(1000):
        now = tick*.5
        if tick % 3 == 0:
            reports = []
            for position, speed in zip(positions, speeds):
                relative = position-loc
                distance = np.linalg.norm(relative)
                reports.append(np.array([relative+rng.normal(0., 2., 3),
                                         [distance-10., (speed-vel).dot(relative/distance)
                                          +rng.normal(0., .02), 20.*rng.normal(1., .05)],
                                         [now, 0., 0.]]))
            rng.shuffle(reports)
        start = time.perf_counter()
        command, _, _ = planner.command(now, loc, vel, target, np.zeros(3), reports)
        durations.append(time.perf_counter()-start)
        actual = delivery*command
        # Gravity is an external disturbance and consumes no propulsion impulse.
        acceleration = actual+np.array([0., 0., -.003])
        impulse += np.linalg.norm(actual)*.5
        loc += .5*vel+.125*acceleration
        vel += .5*acceleration
        peak = max(peak, np.linalg.norm(vel))
        positions += .5*speeds
        if len(positions):
            minimum = min(minimum, float(np.linalg.norm(positions-loc, axis=1).min()))
        if np.linalg.norm(target-loc) <= 50.:
            success = True
            break
    return dict(seed=seed, thrust_delivery=delivery, arrived=success, time_s=round(now+.5, 2),
                min_physical_gap_m=None if np.isinf(minimum) else round(minimum-13., 2),
                peak_speed_m_s=round(peak, 2), translation_delta_v_m_s=round(impulse, 2),
                plan_p95_ms=round(float(np.percentile(durations, 95))*1000., 2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-ref', required=True)
    parser.add_argument('--output', default='tests/benchmark_results.json')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    baseline = subprocess.check_output(['git', 'show', args.baseline_ref+':Controller.py'], cwd=root, text=True)
    current = (root/'Controller.py').read_text()
    result = {'baseline_ref': args.baseline_ref,
              'current_sha256': hashlib.sha256(current.encode()).hexdigest(),
              'limitations': 'Synthetic translation only; no actual allocator, hulls, or attitude fuel.',
              'cases': []}
    for seed, delivery in ((0, .9), (7, .9), (19, .65)):
        row = {'baseline': run_case(load_planner(baseline), seed, delivery),
               'current': run_case(load_planner(current), seed, delivery)}
        result['cases'].append(row)
        print(json.dumps(row), flush=True)
    (root/args.output).write_text(json.dumps(result, indent=2)+'\n')
