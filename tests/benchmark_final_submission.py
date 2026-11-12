"""Offline final-scenario stress test. Never imported by the live controller.

Reads the public scenario as synthetic ground truth, then gives the controller
only noisy cycling radar and rendered camera observations. Station hull is an
explicit ellipsoid PROXY, not the actual Unreal collision mesh.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from benchmark_final import Allocator, rotate
from controller_loader import load_namespace


def quaternion_matrix(quaternion):
    q = np.asarray(quaternion, float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    v = q[:3]
    return (w*w-v@v)*np.eye(3)+2.*np.outer(v, v)-2.*w*skew


def load_scene(path):
    data = json.loads(Path(path).read_text())
    entities = {}
    def visit(node):
        if isinstance(node, dict):
            required = node.get('#Required')
            if isinstance(required, dict) and 'Location' in node:
                entities[required['Name']] = node
            for item in node.values():
                visit(item)
        elif isinstance(node, list):
            for item in node:
                visit(item)
    visit(data)
    rocks = [entities[key] for key in sorted(entities) if key.startswith('Debris') and 'RadarDiameter_m' in entities[key]]
    shuttle, target = entities['SpaceBuggy'], entities['AsteroidCycler']
    return dict(loc=np.array(shuttle['Location'][1:]), rotation=quaternion_matrix(shuttle['Rotation'][1:]),
                target=np.array(target['Location'][1:]), station_rotation=quaternion_matrix(target['Rotation'][1:]),
                positions=np.array([rock['Location'][1:] for rock in rocks]),
                velocities=np.array([rock['Velocity'][1:] for rock in rocks]),
                diameters=np.array([rock['RadarDiameter_m'] for rock in rocks]))


def render_scene(namespace, location, rotation, name, objects, radii, target, station_rotation, yaw, pitch):
    height, width = namespace['CAMERA_HEIGHT'], namespace['CAMERA_WIDTH']
    rows, cols = np.indices((height, width))
    rays = namespace['CameraPerception'].rays(rows.ravel(), cols.ravel(), width, height, namespace['CAMERA_FOV'])
    camera_rotation = namespace['camera_rotation'](yaw, pitch)@rotation
    offset = np.array([1.27, 0., .9]) if name == 'top' else np.array([1.675, 0., -1.025])
    origin = location+rotation.T@offset
    relative = (objects-origin)@camera_rotation.T
    tan_y = np.tan(np.deg2rad(namespace['CAMERA_FOV'])/2.)
    tan_x = tan_y*width/height
    visible = np.flatnonzero((relative[:, 0]+radii > 0.) & (relative[:, 0]-radii < 230.)
        & (np.abs(relative[:, 1]) < relative[:, 0]*tan_x+radii)
        & (np.abs(relative[:, 2]) < relative[:, 0]*tan_y+radii))
    visible = visible[np.argsort(np.linalg.norm(relative[visible], axis=1))[:96]]
    aa = np.sum(rays*rays, axis=1)
    depth = np.full(len(rays), np.inf)
    for index in visible:
        center = relative[index]
        dot = rays@center
        disc = dot*dot-aa*(center@center-radii[index]**2)
        roots = (dot-np.sqrt(np.maximum(disc, 0.)))/aa
        depth = np.minimum(depth, np.where((disc >= 0.) & (roots > 0.), roots, np.inf))
    # Intentionally obstructs the direct arrival side while leaving safe sides
    # within the scoring sphere. Its dimensions are NOT claimed as official.
    hull = np.array([20., 20., 55.])
    local_origin = station_rotation@(origin-target)/hull
    local_rays = rays@camera_rotation@station_rotation.T/hull
    aa_hull = np.sum(local_rays*local_rays, axis=1)
    bb = local_rays@local_origin
    disc = bb*bb-aa_hull*(local_origin@local_origin-1.)
    roots = (-bb-np.sqrt(np.maximum(disc, 0.)))/np.maximum(aa_hull, 1e-12)
    depth = np.minimum(depth, np.where((disc >= 0.) & (roots > 0.), roots, np.inf))
    return (depth*100.).reshape(height, width)


def run(namespace, allocator, scene, max_time, seed):
    rng = np.random.default_rng(seed)
    planner = namespace.get('FinalNavigator', namespace['QualificationPlanner'])()
    attitude = namespace.get('FlightAttitude', lambda: None)()
    camera, history = namespace['CameraPerception'](), namespace['PoseHistory']()
    loc, vel, omega = scene['loc'].copy(), np.zeros(3), np.zeros(3)
    rotation, target = scene['rotation'].copy(), scene['target']
    objects = scene['positions'].copy()
    rock_velocities = scene['velocities'].copy()+rng.normal(0., .02, scene['velocities'].shape)
    diameters = scene['diameters']
    count = len(objects)
    reports = np.zeros((count, 3, 3))
    radar_index = 0
    def measure(index, stamp):
        relative = objects[index]-loc
        distance = np.linalg.norm(relative)
        reports[index] = [relative+rng.normal(0., 2., 3),
            [distance-diameters[index]/2., (rock_velocities[index]-vel).dot(relative/max(distance, 1e-9))
             +rng.normal(0., .02), diameters[index]*rng.normal(1., .05)], [stamp, 0., 0.]]
    for index in range(count):
        measure(index, 0.)
    pending, timings = [], []
    mode, clearance, fuel = 'START', np.inf, 0.
    peak, nearest, hull_gap = 0., np.inf, np.inf
    arrived, heading = False, rotation[0].copy()
    arrival_time = arrival_fuel = None
    crossing_speed = None
    angular_active, linear_active, nose_errors = 0, 0, []
    for tick in range(int(max_time*10)):
        now = tick*.1
        history.add(now, loc, rotation)
        for item in pending[:]:
            due, name, kind, pixels, stamp, yaw, pitch = item
            if now >= due:
                camera.ingest(name, kind, pixels, stamp, now, history.sample(stamp), yaw, pitch)
                pending.remove(item)
        if tick % 5 == 0:
            name = 'top' if tick % 10 == 0 else 'bottom'
            travel = target-loc if np.linalg.norm(target-loc) < 300. or np.linalg.norm(vel) < 1. else vel
            body = rotation@travel
            yaw = float(np.clip(np.rad2deg(np.arctan2(body[1], body[0])), -89., 89.))
            angle = np.rad2deg(np.arctan2(-body[2], max(np.linalg.norm(body[:2]), 1e-9)))
            pitch = float(np.clip(angle, -50., 0.) if name == 'top' else np.clip(angle, 0., 50.))
            depth = render_scene(namespace, loc, rotation, name, objects, diameters/2., target,
                                 scene['station_rotation'], yaw, pitch)
            kind = 'rgb' if tick % 15 == 0 else 'depth'
            pixels = (np.repeat(np.where(np.isfinite(depth), 150., 0.)[:, :, None], 3, axis=2)
                      if kind == 'rgb' else depth)
            pending.append((now+.3, name, kind, pixels, now, yaw, pitch))
        # Radar cycles one object per 1/120 s: 500 objects take ~4.17 s.
        for _ in range(min(12, count)):
            measure(radar_index, now)
            radar_index = (radar_index+1) % count
        if tick % 5 == 0:
            start = time.perf_counter()
            radar = sorted(reports.copy(), key=lambda item: np.linalg.norm(item[0]))
            _, mode, clearance = planner.command(now, loc, vel, target, np.zeros(3), radar,
                                                camera=camera, cruise_speed=namespace.get('CHUNK_CRUISE_SPEED', namespace['MAX_SPEED']))
            timings.append(time.perf_counter()-start)
        urgent = planner.urgent or np.linalg.norm(target-loc) < 80.
        linear = namespace['velocity_command'](planner.previous_velocity, vel, namespace['MAX_ACCEL'], urgent=urgent)
        up = loc/max(np.linalg.norm(loc), 1e-9)
        if attitude is not None:
            budget = namespace.get('attitude_budget', lambda linear, maximum: maximum)(linear, .02)
            reference = namespace.get('attitude_reference', lambda mode, velocity, desired, ship: desired)
            hint = reference(mode, vel, planner.previous_velocity, target-loc)
            angular = attitude.command(now, rotation, vel, hint, up, omega, budget)
        else:
            travel = vel if np.linalg.norm(vel) >= 1. else planner.previous_velocity
            if np.linalg.norm(travel) > .2:
                heading = travel.copy()
            angular = namespace['attitude_command'](rotation, heading, up, omega, .02)
        actual, angular_actual, fuel_rate = allocator.apply(rotation@linear, angular)
        angular_active += np.linalg.norm(angular) > 1e-9
        linear_active += np.linalg.norm(linear) > 1e-9
        if attitude is not None:
            nose_errors.append(attitude.nose_error_deg)
        gravity = -711000.*loc/max(np.linalg.norm(loc)**3, 1.)
        acceleration = rotation.T@actual+gravity
        loc += .1*vel+.005*acceleration
        vel += .1*acceleration
        rotation = rotate(rotation, .1*omega+.005*angular_actual)
        omega += .1*angular_actual
        fuel += .1*fuel_rate
        objects += .1*rock_velocities
        rock_velocities += -.1*711000.*objects/np.maximum(np.linalg.norm(objects, axis=1)**3, 1.)[:, None]
        nearest = min(nearest, float((np.linalg.norm(objects-loc, axis=1)-diameters/2.-3.).min()))
        normalized = scene['station_rotation']@(loc-target)/np.array([23., 23., 58.])
        hull_gap = min(hull_gap, float((np.linalg.norm(normalized)-1.)*23.))
        peak = max(peak, np.linalg.norm(vel))
        distance = np.linalg.norm(target-loc)
        if distance <= 50. and not arrived:
            crossing_speed, arrived = float(np.linalg.norm(vel)), True
            arrival_time, arrival_fuel = now+.1, fuel
        if arrived and now+.1 >= arrival_time+30.:
            break
        if tick % 500 == 0:
            route = planner.corridor if hasattr(planner, 'corridor') else None
            print(f't={now:.0f} phase={mode} d={distance:.0f} speed={np.linalg.norm(vel):.1f} fuel={fuel:.1f} '
                  f'chunks={getattr(route, "chunk_id", 0)} replans={getattr(route, "replans", 0)} '
                  f'buffer={getattr(route, "buffer_m", 0.):.0f} nose={getattr(attitude, "nose_error_deg", 0.):.0f}', flush=True)
    collision = nearest < 0. or hull_gap < 0.
    return dict(arrived=arrived, time_s=round(arrival_time if arrived else now+.1, 2),
                fuel_kg=round(arrival_fuel if arrived else fuel, 2),
                distance_m=round(float(np.linalg.norm(target-loc)), 2), peak_speed_m_s=round(float(peak), 2),
                arrival_speed_m_s=crossing_speed, min_debris_gap_m=round(nearest, 2), min_proxy_hull_gap_m=round(hull_gap, 2),
                collision=collision, post_arrival_test_s=round(now+.1-arrival_time, 1) if arrived else 0.,
                final_speed_m_s=round(float(np.linalg.norm(vel)), 3),
                phase=mode, images=camera.frames, plan_p95_ms=round(float(np.percentile(timings, 95))*1000., 2),
                angular_active_fraction=round(angular_active/(tick+1), 3),
                linear_active_fraction=round(linear_active/(tick+1), 3),
                nose_error_p95_deg=round(float(np.percentile(nose_errors, 95)), 2) if nose_errors else None,
                allocator_retries=allocator.retries,
                zero_damage_score_estimate=(round(100.*(120./arrival_time-arrival_fuel/75.+.5), 2)
                                           if arrived and not collision else None))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-config', required=True)
    parser.add_argument('--jet-config', required=True)
    parser.add_argument('--max-time', type=float, default=500.)
    parser.add_argument('--seed', type=int, default=100)
    parser.add_argument('--output', default='tests/final_submission_results.json')
    parser.add_argument('--baseline-ref')
    parser.add_argument('--clear-corridor', action='store_true',
                        help='Diagnostic only: move debris aside in memory; never edits the official scenario')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source = ((root/'Controller.py').read_text() if not args.baseline_ref else
              subprocess.check_output(['git', 'show', args.baseline_ref+':Controller.py'], cwd=root, text=True))
    scene = load_scene(args.sim_config)
    if args.clear_corridor:
        direction = scene['target']-scene['loc']
        axis = np.eye(3)[np.argmin(np.abs(direction))]
        side = np.cross(direction, axis)
        scene['positions'] += 5000.*side/np.linalg.norm(side)
    result = dict(controller_sha256=hashlib.sha256(source.encode()).hexdigest(),
                  scenario_sha256=hashlib.sha256(Path(args.sim_config).read_bytes()).hexdigest(),
                  debris_count=len(scene['positions']), seed=args.seed,
                  scene_variant='clear-corridor diagnostic' if args.clear_corridor else 'final-scene',
                  limitations='Offline model, simplified gravity and ellipsoid proxy hull; no real Unreal mesh/engine.',
                  result=run(load_namespace(source), Allocator(args.jet_config), scene, args.max_time, args.seed))
    (root/args.output).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
