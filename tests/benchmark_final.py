"""Deterministic final-controller experiment with the official rover jet geometry.

Requires NumPy + SciPy (available in the simulator Python environment). This is
an offline approximation: no Space Teams process, hidden live state, or sim edits.
Uses the published allocator equations, HiGHS LP instead of revised simplex,
sphere obstacles, synthetic camera renders, and simplified gravity/rigid dynamics.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from scipy.optimize import linprog
from controller_loader import load_namespace


class Allocator:
    def __init__(self, path):
        jets = np.loadtxt(path, delimiter=',', skiprows=1)
        self.thrust = jets[:, 6]
        self.inertia = np.array([[3025.8, -171.7, 461.7], [-171.7, 5927.5, 313.6],
                                 [461.7, 313.6, 6756.]])
        forces = -jets[:, 3:6]/np.linalg.norm(jets[:, 3:6], axis=1)[:, None]*self.thrust[:, None]
        self.matrix = np.vstack((forces.T/3388.7,
                                 np.linalg.solve(self.inertia, np.cross(jets[:, :3], forces).T)))
        self.cache = {}

    def apply(self, accel, angular):
        command = np.r_[accel, angular]
        if np.linalg.norm(command) < 1e-9:
            return np.zeros(3), np.zeros(3), 0.
        key = tuple(np.round(command, 5))
        if key not in self.cache:
            result = linprog(np.ones(len(self.thrust)), A_eq=self.matrix, b_eq=command,
                             bounds=(0, None), method='highs')
            if not result.success:
                raise RuntimeError('infeasible allocator request: '+result.message)
            on = np.maximum(result.x, 0.)
            # Official remap normalizes the largest on-time to min(dt, max_on).
            # Therefore small continuous requests need not yield small thrust.
            duty = on/max(float(on.max()), 1./60.)
            achieved = self.matrix@duty
            fuel_rate = float(self.thrust@duty)/(80.*9.8)
            self.cache[key] = achieved[:3], achieved[3:], fuel_rate
        return self.cache[key]


def rotate(rotation, delta):
    angle = np.linalg.norm(delta)
    if angle < 1e-12:
        return rotation
    x, y, z = delta/angle
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    active = np.eye(3)+np.sin(angle)*skew+(1.-np.cos(angle))*(skew@skew)
    return active.T@rotation


def render(namespace, location, rotation, name, objects, yaw, pitch):
    height, width = namespace['CAMERA_HEIGHT'], namespace['CAMERA_WIDTH']
    rows, cols = np.indices((height, width))
    rays = namespace['CameraPerception'].rays(rows.ravel(), cols.ravel(), width, height,
                                              namespace['CAMERA_FOV'])
    camera_rotation = namespace['camera_rotation'](yaw, pitch)@rotation
    offset = np.array([1.27, 0., .9]) if name == 'top' else np.array([1.675, 0., -1.025])
    origin = location+rotation.T@offset
    depth = np.full(height*width, np.inf)
    aa = np.sum(rays*rays, axis=1)
    for position in objects:
        center = camera_rotation@(position-origin)
        dot = rays@center
        discriminant = dot*dot-aa*(center@center-10.**2)
        roots = (dot-np.sqrt(np.maximum(discriminant, 0.)))/aa
        depth = np.minimum(depth, np.where((discriminant >= 0.) & (roots > 0.), roots, np.inf))
    # Scene depth is camera-forward X. Infinite pixels represent empty sky.
    return (100.*depth).reshape(height, width)


def run_case(namespace, allocator, seed, final, max_time):
    planner = namespace['QualificationPlanner']()
    rng = np.random.default_rng(seed)
    loc, vel, omega = np.zeros(3), np.zeros(3), np.zeros(3)
    rotation = np.diag([1., -1., -1.])  # Include initial upside-down recovery cost.
    target = np.array([1305.8, 0., 0.])
    if seed == 0:
        objects, speeds = np.empty((0, 3)), np.empty((0, 3))
    else:
        objects = np.column_stack((np.linspace(150., 1100., 8), rng.uniform(-60., 60., 8),
                                   rng.uniform(-40., 40., 8)))
        speeds = rng.uniform(-.3, .3, (8, 3))
    if final:
        policy, vision, history = namespace['ScorePolicy'](), namespace['CameraPerception'](), namespace['PoseHistory']()
    heading = np.array([1., 0., 0.])
    fuel, minimum, peak = 0., np.inf, 0.
    reports, durations = [], []
    pending = []
    mode, clearance, cruise = 'START', np.inf, 12.
    arrived = False
    for tick in range(int(max_time*10)):
        now = tick*.1
        if final:
            history.add(now, loc, rotation)
            for item in pending[:]:
                due, name, kind, pixels, stamp, yaw, pitch = item
                if now >= due:
                    vision.ingest(name, kind, pixels, stamp, now, history.sample(stamp), yaw, pitch)
                    pending.remove(item)
            if tick % 5 == 0:
                name = 'top' if tick % 10 == 0 else 'bottom'
                body_travel = rotation@(vel if np.linalg.norm(vel) > 1. else target-loc)
                yaw = float(np.clip(np.rad2deg(np.arctan2(body_travel[1], body_travel[0])), -89., 89.))
                angle = np.rad2deg(np.arctan2(-body_travel[2], max(np.linalg.norm(body_travel[:2]), 1e-9)))
                pitch = float(np.clip(angle, -50., 0.) if name == 'top' else np.clip(angle, 0., 50.))
                depth = render(namespace, loc, rotation, name, objects, yaw, pitch)
                kind = 'rgb' if tick % 15 == 0 else 'depth'
                pixels = (np.repeat(np.where(np.isfinite(depth), 150., 0.)[:, :, None], 3, axis=2)
                          if kind == 'rgb' else depth)
                pending.append((now+.3, name, kind, pixels, now, yaw, pitch))
        if tick % 15 == 0:
            reports = []
            for position, speed in zip(objects, speeds):
                relative = position-loc
                distance = np.linalg.norm(relative)
                reports.append(np.array([relative+rng.normal(0., 2., 3),
                                         [distance-10., (speed-vel).dot(relative/distance)
                                          +rng.normal(0., .02), 20.*rng.normal(1., .05)], [now, 0., 0.]]))
            rng.shuffle(reports)
        if tick % 5 == 0:
            start = time.perf_counter()
            kwargs = {}
            if final:
                policy.observe(now, vel, fuel)
                cruise = policy.choose(max(0., np.linalg.norm(target-loc)-45.), np.linalg.norm(vel), now, fuel,
                                       maneuvering=mode == 'ESCAPE' or clearance < 25.)
                kwargs = dict(camera=vision, cruise_speed=cruise)
            _, mode, clearance = planner.command(now, loc, vel, target, np.zeros(3), reports, **kwargs)
            durations.append(time.perf_counter()-start)
        if final:
            linear = namespace['velocity_command'](planner.previous_velocity, vel, .3,
                        urgent=planner.urgent or np.linalg.norm(target-loc) < 80.)
        else:
            linear = namespace['limit_vector'](.6*(planner.previous_velocity-vel), .3)
        travel = vel if np.linalg.norm(vel) >= 1. else planner.previous_velocity
        if np.linalg.norm(travel) > .2:
            heading = travel.copy()
        angular = namespace['attitude_command'](rotation, heading, np.array([0., 0., 1.]), omega, .02)
        actual_body, actual_angular, rate = allocator.apply(rotation@linear, angular)
        acceleration = rotation.T@actual_body+np.array([0., 0., -.003])
        loc += .1*vel+.005*acceleration
        vel += .1*acceleration
        rotation = rotate(rotation, .1*omega+.005*actual_angular)
        omega += .1*actual_angular
        fuel += .1*rate
        peak = max(peak, np.linalg.norm(vel))
        objects += .1*speeds
        if len(objects):
            minimum = min(minimum, float(np.linalg.norm(objects-loc, axis=1).min())-13.)
        if np.linalg.norm(target-loc) <= 50.:
            arrived = True
            break
    elapsed = now+.1
    return dict(seed=seed, arrived=arrived, time_s=round(elapsed, 2), fuel_kg=round(fuel, 2),
                min_sphere_clearance_m=None if np.isinf(minimum) else round(minimum, 2),
                collisions=bool(minimum < 0.), peak_speed_m_s=round(peak, 2),
                zero_damage_score_estimate=round(100.*(120./elapsed-fuel/75.+.5), 2),
                camera_frames=vision.frames if final else 0,
                plan_p95_ms=round(float(np.percentile(durations, 95))*1000., 2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-ref', required=True)
    parser.add_argument('--baseline-fusion', action='store_true',
                        help='Run the baseline RGB/depth + score policy, when that commit supports it')
    parser.add_argument('--jet-config', required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[0, 7, 19])
    parser.add_argument('--max-time', type=float, default=400.)
    parser.add_argument('--output', default='tests/final_benchmark_results.json')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    old = subprocess.check_output(['git', 'show', args.baseline_ref+':Controller.py'], cwd=root, text=True)
    source = (root/'Controller.py').read_text()
    report = dict(baseline_ref=args.baseline_ref, current_sha256=hashlib.sha256(source.encode()).hexdigest(),
                  baseline_fusion=args.baseline_fusion,
                  model='38-jet LP + on-time normalization; synthetic sphere/RGB/depth world; not the official simulator',
                  cases=[])
    for seed in args.seeds:
        row = dict(baseline=run_case(load_namespace(old), Allocator(args.jet_config), seed, args.baseline_fusion, args.max_time),
                   final=run_case(load_namespace(source), Allocator(args.jet_config), seed, True, args.max_time))
        report['cases'].append(row)
        print(json.dumps(row), flush=True)
    (root/args.output).write_text(json.dumps(report, indent=2)+'\n')
