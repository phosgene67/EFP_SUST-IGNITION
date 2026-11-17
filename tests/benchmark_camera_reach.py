"""Reproduce the camera-reach stress case without editing Controller.py.

Shortens the sensor chunk at low speed only in the offline namespace. This is
an intentionally rejected tuning condition, not the delivered flight policy.
"""
import argparse
import hashlib
import json
from pathlib import Path

from benchmark_final import Allocator
from benchmark_final_submission import load_scene, run
from controller_loader import load_namespace


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-config', required=True)
    parser.add_argument('--jet-config', required=True)
    parser.add_argument('--seed', type=int, default=101)
    parser.add_argument('--without-view-guard', action='store_true')
    parser.add_argument('--output', default='tests/coast_camera_stress101.json')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source = (root/'Controller.py').read_text()
    original = 'horizon = float(min(20., .45*c_times.min()))'
    replacement = 'horizon = float(min(np.clip(8.+2.*np.linalg.norm(vel-target_vel), 12., 20.), .45*c_times.min()))'
    if source.count(original) != 1:
        raise RuntimeError('Expected one sensor-horizon definition; review this stress test before running it.')
    stressed = source.replace(original, replacement)
    namespace = load_namespace(stressed)
    if args.without_view_guard:
        namespace['camera_can_aim'] = lambda *args: True
    scene = load_scene(args.sim_config)
    record = dict(
        controller_sha256=hashlib.sha256(source.encode()).hexdigest(),
        experimental_source_sha256=hashlib.sha256(stressed.encode()).hexdigest(),
        scenario_sha256=hashlib.sha256(Path(args.sim_config).read_bytes()).hexdigest(),
        seed=args.seed, debris_count=len(scene['positions']), control_hz=60.,
        ground_plane_m=2.5, view_guard_enabled=not args.without_view_guard,
        stress_override='12--20 s speed-adaptive horizon, offline only; delivered horizon remains 20 s',
        limitations='Synthetic cameras and proxy meshes, not a native simulator run.',
        result=run(namespace, Allocator(args.jet_config), scene, 500., args.seed, ground_height=2.5))
    (root/args.output).write_text(json.dumps(record, indent=2)+'\n')
    print(json.dumps(record), flush=True)
