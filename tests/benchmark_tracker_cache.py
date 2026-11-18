"""Paired tracker CPU audit on unchanged mature reports; not a score benchmark."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from controller_loader import load_namespace


def audit(source, count=500, repeats=30):
    namespace = load_namespace(source)
    planners = [namespace['QualificationPlanner'](), namespace['QualificationPlanner']()]
    location, velocity = np.array([9000., 0., 0.]), np.array([2., 0., 0.])
    rng = np.random.default_rng(20261118)
    reports = []
    for index in range(count):
        position = location+np.array([100.+index, 60.+rng.uniform(-30., 30.), rng.uniform(-40., 40.)])
        motion, stamp = np.array([.2, -.1, .05]), 20.
        relative = position-location
        report = np.array([relative, [np.linalg.norm(relative), (motion-velocity)@relative/np.linalg.norm(relative), 10.], [stamp, 0., 0.]])
        reports.append(report)
        for enabled, planner in zip((False, True), planners):
            planner.cache_reports, planner.gravity_tracking = enabled, True
            planner.history = [(0., location-20.*velocity, velocity.copy())]
            samples = [(float(t), position+(t-stamp)*motion) for t in range(0, 21, 3)]
            samples.append((stamp, position.copy()))
            planner.report_cache[tuple(report.ravel())] = dict(
                id=index, stamp=stamp, pos=position.copy(), vel=motion.copy(), diameter=10., samples=samples)
            planner.track_counter = count
    wall, cpu = [[], []], [[], []]
    max_difference, metadata_equal = 0., True
    for repeat in range(repeats):
        outputs = {}
        for index in ((0, 1) if repeat % 2 == 0 else (1, 0)):
            start_wall, start_cpu = time.perf_counter(), time.process_time()
            outputs[index] = planners[index].obstacles(20., location, velocity, reports)
            cpu[index].append(1000.*(time.process_time()-start_cpu))
            wall[index].append(1000.*(time.perf_counter()-start_wall))
        for a, b in zip(outputs[0], outputs[1]):
            max_difference = max(max_difference, float(np.max(np.abs(a-b))))
        for name in ('radar_forecast_rates', 'radar_core_radii', 'radar_report_ages', 'radar_track_confidence'):
            metadata_equal &= np.array_equal(getattr(planners[0], name), getattr(planners[1], name))
    if max_difference != 0. or not metadata_equal:
        raise RuntimeError('Cache changed tracker outputs')
    return dict(kind='measured_compute_on_synthetic_sensor_fixture', objects=count, paired_calls=repeats,
        source_sha256=hashlib.sha256(source.encode()).hexdigest(), max_output_abs_difference=max_difference,
        uncertainty_metadata_equal=bool(metadata_equal),
        timings_ms={name: dict(wall_median=float(np.median(wall[i][1:])), wall_p95=float(np.percentile(wall[i][1:], 95)),
                              cpu_median=float(np.median(cpu[i][1:]))) for i, name in enumerate(('uncached', 'cached'))},
        note='Alternating paired calls, first cold call excluded. All reports unchanged and mature: an upper-bound cache opportunity. Native cycles include fresh reports too. Concurrent workload affects wall time. This does not measure native score.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', type=Path, default=Path(__file__).resolve().parents[1]/'Controller.py')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.controller.read_text())
    args.output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
