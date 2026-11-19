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
from collections import defaultdict
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


def load_scene(path, numeric_debris_order=False):
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
    names = [key for key in sorted(entities) if key.startswith('Debris') and 'RadarDiameter_m' in entities[key]]
    if numeric_debris_order:
        names.sort(key=lambda name: int(name[6:]))
    rocks = [entities[key] for key in names]
    shuttle, target = entities['SpaceBuggy'], entities['AsteroidCycler']
    return dict(loc=np.array(shuttle['Location'][1:]), rotation=quaternion_matrix(shuttle['Rotation'][1:]),
                target=np.array(target['Location'][1:]), station_rotation=quaternion_matrix(target['Rotation'][1:]),
                positions=np.array([rock['Location'][1:] for rock in rocks]),
                velocities=np.array([rock['Velocity'][1:] for rock in rocks]),
                diameters=np.array([rock['RadarDiameter_m'] for rock in rocks]))


def render_scene(namespace, location, rotation, name, objects, radii, target, station_rotation, yaw, pitch, ground=None):
    height, width = namespace['CAMERA_HEIGHT'], namespace['CAMERA_WIDTH']
    rows, cols = np.indices((height, width))
    rays = namespace['CameraPerception'].rays(rows.ravel(), cols.ravel(), width, height, namespace['CAMERA_FOV'])
    camera_rotation = namespace['camera_rotation'](yaw, pitch)@rotation
    offset = np.array([1.27, 0., .9]) if name == 'top' else np.array([1.675, 0., -1.025])
    origin = location+rotation.T@offset
    relative = (objects-origin)@camera_rotation.T
    tan_y = np.tan(np.deg2rad(namespace['CAMERA_FOV'])/2.)
    tan_x = tan_y*width/height
    visible = np.flatnonzero((relative[:, 0]+radii > 0.) & (relative[:, 0]-radii < namespace['CAMERA_MAX_RANGE']+10.)
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
    if ground is not None:
        plane, normal = ground
        world_rays = rays@camera_rotation
        denominator = world_rays@normal
        distance = (plane-origin)@normal/np.where(np.abs(denominator) > 1e-9, denominator, np.nan)
        depth = np.minimum(depth, np.where(distance > 0., distance, np.inf))
    return (depth*100.).reshape(height, width)


def run(namespace, allocator, scene, max_time, seed, control_hz=60., attitude_mode='normal', fuel_audit=False, axis_mode=False, ground_height=0., drift_bias=None, plan_latency=0., planning_mode='async', experiments=(), rng_mode='legacy'):
    dt = 1./control_hz
    worker = None
    if plan_latency > 0.:
        from planning_doubles import ImmediateExecutor, PlanningWorker
        worker_type = namespace.get('PlanningWorker', PlanningWorker)
        worker = worker_type(ImmediateExecutor())
    plan_due = -np.inf
    held_body, held_angular = np.zeros(3), np.zeros(3)
    linear, angular = np.zeros(3), np.zeros(3)
    drift_bias = np.zeros(3) if drift_bias is None else np.asarray(drift_bias, float)
    rng = np.random.default_rng(seed)
    planner = namespace.get('ShootingNavigator', namespace.get('EconomyNavigator', namespace.get('FinalNavigator', namespace['QualificationPlanner'])))()
    if 'dogleg' in experiments:
        planner = namespace['DoglegNavigator']()
    if 'ground-lateral' in experiments:
        if 'dogleg' not in experiments:
            raise ValueError('ground-lateral requires dogleg')
        planner.ground_lateral = True
    if 'clean-main-burn' in experiments:
        if not hasattr(planner, 'clean_main_burn'):
            raise ValueError('This controller revision does not implement clean-main-burn')
        planner.clean_main_burn = True
    if 'order-diversity' in experiments:
        if not hasattr(planner, 'order_diversity'):
            raise ValueError('This controller revision does not implement order-diversity')
        planner.order_diversity = True
    if 'recheck-coast' in experiments:
        if not hasattr(planner, 'recheck_coast'):
            raise ValueError('This controller revision does not implement recheck-coast')
        planner.recheck_coast = True
    if 'inertial-capture' in experiments:
        if not hasattr(planner, 'inertial_capture'):
            raise ValueError('This controller revision does not implement inertial-capture')
        planner.inertial_capture = True
    if 'vector-burn' in experiments:
        if not hasattr(planner, 'vector_burn'):
            raise ValueError('This controller revision does not implement vector-burn')
        planner.vector_burn = True
    if 'exact-arcs' in experiments:
        if not hasattr(planner, 'exact_arcs'):
            raise ValueError('This controller revision does not implement exact-arcs')
        planner.exact_arcs = True
    if 'coast-windows' in experiments:
        if not hasattr(planner, 'coast_windows'):
            raise ValueError('This controller revision does not implement coast-windows')
        planner.coast_windows = True
    if 'entry-scope' in experiments:
        if not hasattr(planner, 'entry_scope'):
            raise ValueError('This controller revision does not implement entry-scope')
        planner.entry_scope = planner.terminal.entry_scope = True
    if 'track-cache' in experiments:
        if not hasattr(planner, 'cache_reports'):
            raise ValueError('This controller revision does not implement track-cache')
        planner.cache_reports = True
    gate = None
    fuel_budget = namespace['FuelBudget']() if 'FuelBudget' in namespace else None
    if 'StagedNavigator' in namespace:
        planner = namespace['StagedNavigator']()
        gate = namespace['FlightFuelLimit']()
    attitude_type = (namespace.get('FreeAttitude') if attitude_mode == 'normal' else None)
    if 'camera-recovery' in experiments:
        if 'ViewRecoveryAttitude' not in namespace or attitude_mode != 'normal':
            raise ValueError('camera-recovery requires ViewRecoveryAttitude and normal attitude mode')
        attitude_type = namespace['ViewRecoveryAttitude']
    attitude = (attitude_type or namespace.get('EconomyAttitude', namespace.get('FlightAttitude', lambda: None)))()
    if 'dogleg' in experiments:
        attitude = namespace['DoglegAttitude'](planner)
    if hasattr(planner, 'nose_first') and attitude_mode in ('nose-first', 'nose-only', 'burn-only'):
        planner.nose_first, planner.aligned_once = True, False
    camera, history = namespace['CameraPerception'](), namespace['PoseHistory']()
    loc, vel, omega = scene['loc'].copy(), np.zeros(3), np.zeros(3)
    rotation, target = scene['rotation'].copy(), scene['target']
    objects = scene['positions'].copy()
    rock_rng = np.random.default_rng(8193) if rng_mode == 'native-order' else rng
    rock_velocities = scene['velocities'].copy()+rock_rng.normal(0., .02, scene['velocities'].shape)
    diameters = scene['diameters']
    count = len(objects)
    reports = np.zeros((count, 3, 3))
    radar_index = 0
    def measure(index, stamp):
        relative = objects[index]-loc
        distance = np.linalg.norm(relative)
        position = relative+rng.normal(0., 2., 3)
        closest = distance-diameters[index]/2.
        if rng_mode == 'native-order':
            # Radar.py draws this noise between position and range rate.
            # Even though guidance does not use closest range, skipping the
            # draw changes every later noise sample and track association.
            closest += rng.normal(0., 2.)
        reports[index] = [position,
            [closest, (rock_velocities[index]-vel).dot(relative/max(distance, 1e-9))
             +rng.normal(0., .02), diameters[index]*rng.normal(1., .05)], [stamp, 0., 0.]]
    for index in range(count):
        measure(index, 0.)
    pending, timings = [], []
    mode, clearance, fuel = 'START', np.inf, 0.
    peak, nearest, hull_gap, ground_gap = 0., np.inf, np.inf, np.inf
    arrived, heading = False, rotation[0].copy()
    arrival_time = arrival_fuel = None
    arrival_collision = None
    crossing_speed = None
    milestones = {}
    angular_active, linear_active, nose_errors = 0, 0, []
    first_five, cruise_after_five = None, []
    coast_ticks = 0
    bent_updates, return_events, reserve_ticks = 0, 0, 0
    max_bend_impulse, max_lateral_deviation = 0., 0.
    returning_before = False
    reference_direction = target-scene['loc']
    reference_direction /= np.linalg.norm(reference_direction)
    next_capture = next_plan = next_report = radar_credit = 0.
    capture_index = 0
    up_start = loc/max(np.linalg.norm(loc), 1e-9)
    ground = (loc-ground_height*up_start, up_start) if ground_height > 0. else None
    fuel_segments, fuel_regimes = defaultdict(lambda: dict(time_s=0., fuel_kg=0.)), defaultdict(lambda: dict(time_s=0., fuel_kg=0.))
    audit_allocator = Allocator.__new__(Allocator)
    audit_allocator.__dict__ = dict(allocator.__dict__, cache={}, retries=0)
    shadow = dict(translation_only_kg=0., attitude_only_kg=0.)
    post_fuel = 0.
    for tick in range(int(max_time*control_hz)):
        now = tick*dt
        planner.control_time = now
        if hasattr(planner, 'observe_motion'):
            planner.observe_motion(now, loc, vel, bool(tick and
                (np.linalg.norm(linear) > 1e-9 or np.linalg.norm(angular) > 1e-9)))
            planner.observe_arrival(now, loc, vel, target, np.zeros(3))
        history.add(now, loc, rotation)
        for item in pending[:]:
            due, name, kind, pixels, stamp, yaw, pitch = item
            if now >= due:
                camera.ingest(name, kind, pixels, stamp, now, history.sample(stamp), yaw, pitch)
                pending.remove(item)
        if now+1e-8 >= next_capture:
            name = 'top' if capture_index % 2 == 0 else 'bottom'
            travel = target-loc if np.linalg.norm(target-loc) < 300. or np.linalg.norm(vel) < 1. else vel
            if getattr(attitude, 'viewing', False):
                travel = target-loc
            if 'camera_course' in namespace:
                travel = namespace['camera_course'](loc, vel, target, np.zeros(3))
            body = rotation@travel
            yaw = float(np.clip(np.rad2deg(np.arctan2(body[1], body[0])), -89., 89.))
            angle = np.rad2deg(np.arctan2(-body[2], max(np.linalg.norm(body[:2]), 1e-9)))
            pitch = float(np.clip(angle, -50., 0.) if name == 'top' else np.clip(angle, 0., 50.))
            depth = render_scene(namespace, loc, rotation, name, objects, diameters/2., target,
                                 scene['station_rotation'], yaw, pitch, ground)
            kind = 'rgb' if (capture_index//2) % 3 == 2 else 'depth'
            pixels = (np.repeat(np.where(np.isfinite(depth), 150., 0.)[:, :, None], 3, axis=2)
                      if kind == 'rgb' else depth)
            pending.append((now+.3, name, kind, pixels, now, yaw, pitch))
            capture_index += 1
            next_capture += .25
        # Radar cycles one object per 1/120 s: 500 objects take ~4.17 s.
        radar_credit += 120.*dt
        measurements = int(radar_credit+1e-8)
        radar_credit -= measurements
        for _ in range(min(measurements, count)):
            measure(radar_index, now)
            radar_index = (radar_index+1) % count
        if worker is not None and worker.busy and now+1e-8 >= plan_due:
            planner.body_rotation = rotation
            completed = worker.take(now, planner, camera)
            if completed is not None:
                planner, mode, clearance = completed
        if now+1e-8 >= next_plan and (worker is None or not worker.busy):
            planner.body_rotation = rotation
            start = time.perf_counter()
            radar = sorted(reports.copy(), key=lambda item: np.linalg.norm(item[0]))
            if hasattr(planner, 'policy'):
                planner.policy.observe(now, vel, fuel,
                    thrusting=(np.linalg.norm(linear) > .08 and (getattr(attitude, 'free_attitude', False) or attitude_mode == 'off' or attitude.nose_error_deg < 10.) and np.linalg.norm(angular) < .003) if tick else False,
                    gravity=(planner.gravity_at(loc) if hasattr(planner, 'gravity_at')
                             else -711000.*loc/max(np.linalg.norm(loc)**3, 1.)))
                planner.nose_error = attitude.nose_error_deg
                if attitude_mode == 'off':
                    planner.aligned_once = True
            cap = 10. if hasattr(planner, 'policy') else namespace.get('CHUNK_CRUISE_SPEED', namespace['MAX_SPEED'])
            if worker is None:
                _, mode, clearance = planner.command(now, loc, vel, target, np.zeros(3), radar,
                                                     camera=camera, cruise_speed=cap)
            else:
                worker.submit(now, planner, camera, loc, vel, target, np.zeros(3), radar, cap)
                plan_due = now+plan_latency
            timings.append(time.perf_counter()-start)
            next_plan = now+.5
        bend = float(np.linalg.norm(getattr(planner, 'last_detour', np.zeros(3))))
        max_bend_impulse = max(max_bend_impulse, bend)
        if abs(now-(next_plan-.5)) < 1e-8 and bend > .01:
            bent_updates += 1
        reserve_ticks += getattr(planner, 'last_mode', '') == 'RESERVE_COAST'
        urgent = planner.urgent or np.linalg.norm(target-loc) < 80.
        planner.body_rotation = rotation
        linear = (planner.linear_command(vel, namespace['MAX_ACCEL']) if hasattr(planner, 'linear_command') else
                  namespace['velocity_command'](planner.previous_velocity, vel, namespace['MAX_ACCEL'], urgent=urgent))
        returning_now = getattr(planner, 'return_started', False)
        return_events += returning_now and not returning_before
        returning_before = returning_now
        if axis_mode:
            linear = namespace['axis_thrust'](linear, rotation, namespace['MAX_ACCEL'])
        up = loc/max(np.linalg.norm(loc), 1e-9)
        if attitude is not None:
            if hasattr(attitude, 'configure') and abs(now-(next_plan-.5)) < 1e-8:
                attitude.configure(now, planner, loc, vel, target, np.zeros(3), linear, camera)
            if hasattr(attitude, 'linear_active'):
                attitude.linear_active = np.linalg.norm(linear) > 1e-8
            if hasattr(attitude, 'aligning'):
                attitude.aligning = planner.aligning
            budget = namespace.get('attitude_budget', lambda linear, maximum: maximum)(linear, .02)
            reference = namespace.get('attitude_reference', lambda mode, velocity, desired, ship: desired)
            hint = reference(mode, vel, planner.previous_velocity, target-loc)
            angular = attitude.command(now, rotation, vel, hint,
                                       rotation[2] if attitude_mode == 'nose-only' else up, omega, budget)
            if attitude_mode == 'off':
                angular = np.zeros(3)
            elif attitude_mode == 'burn-only' and not planner.aligning and np.linalg.norm(linear) < .08 and np.linalg.norm(target-loc) > 300. and np.linalg.norm(omega) < .01:
                angular = np.zeros(3)
        else:
            travel = vel if np.linalg.norm(vel) >= 1. else planner.previous_velocity
            if np.linalg.norm(travel) > .2:
                heading = travel.copy()
            angular = namespace['attitude_command'](rotation, heading, up, omega, .02)
        if getattr(planner, 'coast_after_entry', False) and planner.arrival_time is not None:
            angular = np.zeros(3)
        body_command = rotation@linear
        if worker is not None and worker.busy and planning_mode == 'blocking':
            body_command, angular = held_body, held_angular
            linear = rotation.T@body_command
        else:
            held_body, held_angular = body_command.copy(), angular.copy()
        if gate is not None:
            if not gate.allows(np.linalg.norm(target-loc), fuel, planner.advance):
                body_command, angular, linear = np.zeros(3), np.zeros(3), np.zeros(3)
                mode = 'FUEL_LIMIT_COAST'
            planner.inner_leg = gate.crossing_fuel is not None
        if fuel_budget is not None and not fuel_budget.allows(fuel):
            body_command, angular, linear = np.zeros(3), np.zeros(3), np.zeros(3)
            mode = 'FUEL_LIMIT_COAST'
        actual, angular_actual, fuel_rate = allocator.apply(body_command, angular)
        if fuel_audit:
            if arrived:
                post_fuel += dt*fuel_rate
            else:
                phase = getattr(planner, 'last_phase', mode)
                # Disjoint, additive measured buckets. The same fuel is never
                # charged to both a rotation bucket and a translation bucket.
                segment = ('ALIGN_FOR_BURN' if getattr(planner, 'aligning', False) else
                           'CRUISE/'+mode if phase == 'CRUISE' else phase)
                rotation_on, translation_on = np.linalg.norm(angular) > 1e-9, np.linalg.norm(linear) > 1e-9
                regime = ('combined' if rotation_on and translation_on else
                          'attitude_only' if rotation_on else 'translation_only' if translation_on else 'no_command')
                for bucket in (fuel_segments[segment], fuel_regimes[regime]):
                    bucket['time_s'] += dt
                    bucket['fuel_kg'] += dt*fuel_rate
                # These are single-tick counterfactual allocator costs, NOT
                # alternative flight trajectories or additive fuel attribution.
                shadow['translation_only_kg'] += dt*audit_allocator.apply(rotation@linear, np.zeros(3))[2]
                shadow['attitude_only_kg'] += dt*audit_allocator.apply(np.zeros(3), angular)[2]
        coast_ticks += bool(getattr(planner, 'coasting', False))
        angular_active += np.linalg.norm(angular) > 1e-9
        linear_active += np.linalg.norm(linear) > 1e-9
        if attitude is not None:
            nose_errors.append(attitude.nose_error_deg)
        gravity = -711000.*loc/max(np.linalg.norm(loc)**3, 1.)+drift_bias
        acceleration = rotation.T@actual+gravity
        loc += dt*vel+.5*dt*dt*acceleration
        vel += dt*acceleration
        rotation = rotate(rotation, dt*omega+.5*dt*dt*angular_actual)
        omega += dt*angular_actual
        fuel += dt*fuel_rate
        if ground is not None:
            ground_gap = min(ground_gap, float((loc-ground[0])@ground[1]))
        objects += dt*rock_velocities
        rock_velocities += -dt*711000.*objects/np.maximum(np.linalg.norm(objects, axis=1)**3, 1.)[:, None]
        rock_velocities += dt*drift_bias
        nearest = min(nearest, float((np.linalg.norm(objects-loc, axis=1)-diameters/2.-3.).min()))
        normalized = scene['station_rotation']@(loc-target)/np.array([23., 23., 58.])
        hull_gap = min(hull_gap, float((np.linalg.norm(normalized)-1.)*23.))
        peak = max(peak, np.linalg.norm(vel))
        if first_five is None and np.linalg.norm(vel) >= 5.:
            first_five = now+dt
        if first_five is not None and getattr(planner, 'last_phase', None) == 'CRUISE':
            cruise_after_five.append(float(np.linalg.norm(vel)))
        distance = np.linalg.norm(target-loc)
        from_start = loc-scene['loc']
        max_lateral_deviation = max(max_lateral_deviation,
            float(np.linalg.norm(from_start-reference_direction*(from_start@reference_direction))))
        for threshold in (1100, 1000, 600, 550, 500, 200, 50):
            if threshold not in milestones and distance <= threshold:
                milestones[threshold] = dict(time_s=round(now+dt, 2), fuel_kg=round(fuel, 3),
                                             speed_m_s=round(float(np.linalg.norm(vel)), 3))
        if distance <= 50. and not arrived:
            crossing_speed, arrived = float(np.linalg.norm(vel)), True
            arrival_time, arrival_fuel = now+dt, fuel
            arrival_collision = nearest < 0. or hull_gap < 0. or ground_gap < 0.
        post_duration = 90. if hasattr(planner, 'policy') else 30.
        if arrived and (now+dt >= arrival_time+post_duration or
                        (now+dt >= arrival_time+30. and np.linalg.norm(vel) < .05)):
            break
        if now+1e-8 >= next_report:
            next_report += 5. if now < 20. else 10. if distance < 300. else 50.
            route = planner.corridor if hasattr(planner, 'corridor') else None
            print(f't={now:.0f} phase={mode} d={distance:.0f} speed={np.linalg.norm(vel):.1f} fuel={fuel:.1f} '
                  f'chunks={getattr(route, "chunk_id", 0)} replans={getattr(route, "replans", 0)} '
                  f'buffer={getattr(route, "buffer_m", 0.):.0f} nose={getattr(attitude, "nose_error_deg", 0.):.0f}', flush=True)
            if hasattr(planner, 'policy'):
                print(f'  economy peak={planner.policy.speed:.2f} burn={planner.policy.burn_seconds:.1f}s '
                      f'forecast={planner.policy.estimated_score:.1f} kgDV={planner.policy.fuel_per_dv:.1f} '
                      f'coast={planner.coasting} fallback={planner.terminal.fallback is not None} '
                      f'captureGap={planner.terminal.best_gap:.1f}', flush=True)
                when, miss = namespace['ballistic_pass'](loc, vel, target, np.zeros(3))
                print(f'  pass={when:.1f}s miss={miss:.2f}m coastGap={planner.coast_clearance:.1f} '
                      f'dV={np.linalg.norm(planner.previous_velocity-vel):.3f} '
                      f'camera={camera.observes(now, target)}', flush=True)
    collision = nearest < 0. or hull_gap < 0. or ground_gap < 0.
    result = dict(arrived=arrived, time_s=round(arrival_time if arrived else now+dt, 2),
                attitude_policy=type(attitude).__name__,
                fuel_kg=round(arrival_fuel if arrived else fuel, 2),
                distance_m=round(float(np.linalg.norm(target-loc)), 2), peak_speed_m_s=round(float(peak), 2),
                arrival_speed_m_s=crossing_speed, min_debris_gap_m=round(nearest, 2), min_proxy_hull_gap_m=round(hull_gap, 2),
                collision=collision, pre_entry_collision=arrival_collision,
                post_arrival_test_s=round(now+dt-arrival_time, 1) if arrived else 0.,
                min_ground_centre_height_m=round(ground_gap, 2) if ground is not None else None,
                final_speed_m_s=round(float(np.linalg.norm(vel)), 3),
                learned_coast_drift_m_s2=(planner.dynamics.residual.tolist() if hasattr(planner, 'dynamics') else None),
                phase=mode, images=camera.frames, plan_p95_ms=round(float(np.percentile(timings, 95))*1000., 2),
                plan_max_ms=round(float(np.max(timings))*1000., 2),
                angular_active_fraction=round(angular_active/(tick+1), 3),
                linear_active_fraction=round(linear_active/(tick+1), 3),
                nose_error_p95_deg=round(float(np.percentile(nose_errors, 95)), 2) if nose_errors else None,
                allocator_retries=allocator.retries,
                coasting_fraction=round(coast_ticks/(tick+1), 3),
                terminal_fallback=bool(getattr(getattr(planner, 'terminal', None), 'fallback', None)),
                first_5m_s_time=round(first_five, 1) if first_five is not None else None,
                cruise_mean_after_5m_s=round(float(np.mean(cruise_after_five)), 2) if cruise_after_five else None,
                cruise_min_after_5m_s=round(float(np.min(cruise_after_five)), 2) if cruise_after_five else None,
                milestones_by_remaining_metres=milestones,
                bent_plan_updates=bent_updates, return_burn_events=return_events,
                max_bend_impulse_m_s=max_bend_impulse,
                reserve_coast_time_s=round(reserve_ticks*dt, 2),
                max_lateral_deviation_m=round(max_lateral_deviation, 2),
                zero_damage_score_estimate=(round(100.*(120./arrival_time-arrival_fuel/75.+.5), 2)
                                           if arrived and not arrival_collision else None))
    if 'dogleg' in experiments:
        diagnostic = (planner.lane_summary(arrival_time if arrived else now+dt,
                                           arrival_fuel if arrived else fuel, record=True)
                      if hasattr(planner, 'lane_summary') else None)
        result['dogleg'] = dict(planner.lane_log, searches=planner.lane_searches, bends_executed=planner.lane_bends,
                                aligned=planner.lane_aligned, events=list(planner.lane_records),
                                run_summary=diagnostic,
                                no_safe_lane_events=getattr(planner, 'lane_no_safe_events', None))
    if fuel_budget is not None:
        result.update(total_fuel_kg=round(fuel, 3), fuel_limited=fuel_budget.exhausted)
    if gate is not None:
        result.update(fuel_at_600m=gate.crossing_fuel, total_fuel_kg=round(fuel, 3),
                      milestones_by_remaining_metres=milestones)
    if fuel_audit:
        result['fuel_audit'] = dict(
            segments={key: {k: round(v, 4) for k, v in value.items()} for key, value in fuel_segments.items()},
            command_regimes={key: {k: round(v, 4) for k, v in value.items()} for key, value in fuel_regimes.items()},
            counterfactual_same_tick_costs={key: round(value, 4) for key, value in shadow.items()},
            post_arrival_fuel_kg=round(post_fuel, 4),
            note='Buckets sum to pre-arrival fuel. Combined commands cannot be uniquely divided into rotation/translation. Counterfactuals hold commands/pose fixed; only A/B flights measure removal effects.')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-config', required=True)
    parser.add_argument('--jet-config', required=True)
    parser.add_argument('--max-time', type=float, default=500.)
    parser.add_argument('--seed', type=int, default=100)
    parser.add_argument('--output', default='tests/final_submission_results.json')
    parser.add_argument('--baseline-ref')
    parser.add_argument('--baseline-file', help='Saved controller source for a matched local comparison')
    parser.add_argument('--experiment', action='append', default=[],
                        choices=('clean-main-burn', 'order-diversity', 'recheck-coast', 'inertial-capture', 'vector-burn', 'exact-arcs', 'camera-recovery', 'coast-windows', 'entry-scope', 'track-cache', 'dogleg', 'ground-lateral'),
                        help='Opt-in controller experiment, disabled by default')
    parser.add_argument('--attitude-mode', choices=('normal', 'off', 'nose-first', 'nose-only', 'burn-only'), default='normal',
                        help='Offline ablation only; does not change the live controller')
    parser.add_argument('--fuel-audit', action='store_true')
    parser.add_argument('--fuel-plan-target', type=float, help='Offline override of the soft fuel target, not a hard fuel cutoff')
    parser.add_argument('--plan-latency', type=float, default=0., help='Synthetic planning delay in simulation seconds; isolates held-command timing')
    parser.add_argument('--planning-mode', choices=('async', 'blocking'), default='async')
    parser.add_argument('--allocator-solver', choices=('highs', 'native'), default='highs')
    parser.add_argument('--rng-mode', choices=('legacy', 'native-order'), default='legacy',
                        help='Native-order uses numeric debris order, fixed spawn RNG 8193 and all official radar noise draws; seed varies radar noise only')
    parser.add_argument('--axis-thrust', action='store_true', help='Experimental body-axis jet pulses')
    parser.add_argument('--ground-plane', type=float, default=0., help='Diagnostic only: synthetic plane this far below initial centre, metres')
    parser.add_argument('--drift-bias', type=float, nargs=3, default=[0., 0., 0.],
                        help='Unmodeled shared acceleration in m/s^2; stress test, not native calibration')
    parser.add_argument('--control-hz', type=float, default=60., help='Flight servo rate; native controller is 60 Hz')
    parser.add_argument('--clear-corridor', action='store_true',
                        help='Diagnostic only: move debris aside in memory; never edits the official scenario')
    args = parser.parse_args()
    if args.baseline_ref and args.baseline_file:
        parser.error('Choose one baseline source')
    if not 1. <= args.control_hz <= 120.:
        parser.error('--control-hz must be between 1 and 120')
    root = Path(__file__).resolve().parents[1]
    source = ((root/'Controller.py').read_text() if not args.baseline_ref else
              subprocess.check_output(['git', 'show', args.baseline_ref+':Controller.py'], cwd=root, text=True))
    if args.baseline_file:
        source = Path(args.baseline_file).read_text()
    algorithm = load_namespace(source)
    if args.fuel_plan_target is not None:
        algorithm['FUEL_PLAN_TARGET'] = args.fuel_plan_target
    scene = load_scene(args.sim_config, numeric_debris_order=args.rng_mode == 'native-order')
    if args.clear_corridor:
        direction = scene['target']-scene['loc']
        axis = np.eye(3)[np.argmin(np.abs(direction))]
        side = np.cross(direction, axis)
        scene['positions'] += 5000.*side/np.linalg.norm(side)
    result = dict(controller_sha256=hashlib.sha256(source.encode()).hexdigest(),
                  controller_file_sha256=(hashlib.sha256(Path(args.baseline_file).read_bytes()).hexdigest()
                                          if args.baseline_file else
                                          hashlib.sha256((root/'Controller.py').read_bytes()).hexdigest()
                                          if not args.baseline_ref else None),
                  hash_note='controller_sha256 hashes normalized source text; controller_file_sha256 hashes deployed bytes, matching native sourceSHA256 logs.',
                  scenario_sha256=hashlib.sha256(Path(args.sim_config).read_bytes()).hexdigest(),
                  debris_count=len(scene['positions']), seed=args.seed,
                  control_hz=args.control_hz, camera_requests_hz=4., benchmark_version=8 if args.rng_mode == 'native-order' else 7,
                  rng_mode=args.rng_mode,
                  plan_latency_s=args.plan_latency, planning_mode=args.planning_mode,
                  fuel_plan_target_override=args.fuel_plan_target,
                  allocator_solver=args.allocator_solver,
                  experiments=args.experiment,
                  scene_variant='clear-corridor diagnostic' if args.clear_corridor else 'final-scene',
                  attitude_mode=args.attitude_mode, fuel_audit_enabled=args.fuel_audit,
                  axis_thrust=args.axis_thrust, ground_plane_m=args.ground_plane,
                  drift_bias_m_s2=args.drift_bias,
                  limitations='Offline model, simplified gravity and ellipsoid proxy hull; no real Unreal mesh/engine.',
                  result=run(algorithm, Allocator(args.jet_config, 'revised simplex' if args.allocator_solver == 'native' else 'highs'), scene, args.max_time, args.seed, args.control_hz,
                             args.attitude_mode, args.fuel_audit, args.axis_thrust, args.ground_plane, args.drift_bias, args.plan_latency, args.planning_mode, args.experiment, args.rng_mode))
    (root/args.output).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
