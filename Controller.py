#############################################################################################
# DON'T CHANGE ANY OF THE BELOW. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################

import os, sys, time, datetime, traceback
import spaceteams as st
def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    st.logger_fatal(error_message)
sys.excepthook = custom_exception_handler
st.connect_to_sim(sys.argv)

import numpy as np
import numpy.typing as npt


this = st.GetThisSystem()

doSpawning = st.SimGlobals.GetSimEntity().GetParam(st.VarType.bool, "DoSpawning")
if doSpawning:
    while True:
        time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

phobos: st.Entity = this.GetParam(st.VarType.entityRef, "Phobos")
phobosFrame = phobos.GetBodyFixedFrame()
shuttle: st.Entity = this.GetParam(st.VarType.entityRef, "Shuttle")
shuttleFrame = shuttle.GetBodyFixedFrame()
target: st.Entity = this.GetParam(st.VarType.entityRef, "Target")

topCamera: st.Entity = this.GetParam(st.VarType.entityRef, "TopCamera")
topCameraFrame = topCamera.GetBodyFixedFrame()
bottomCamera: st.Entity = this.GetParam(st.VarType.entityRef, "BottomCamera")
bottomCameraFrame = bottomCamera.GetBodyFixedFrame()


def GetNearestDebrisLocations(N: int) -> list[npt.NDArray]:
    allDebrisInfo: list[npt.NDArray] = shuttle.GetParamArray(st.VarType.doubleM3x3, 
                                                            ["Radar", "SortedDebrisData"])
    nearest = allDebrisInfo[:min(N, len(allDebrisInfo))]
    return nearest


def PanTiltCamera(camera: st.Entity, yaw_deg: float, pitch_deg: float) -> None:
    if camera == topCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, -50.0, 0.0)
    elif camera == bottomCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, 0.0, 50.0)
    camera.setRotation_DCM(st.math.Euler321_to_DCM([np.deg2rad(yaw_deg), np.deg2rad(pitch_deg), 0.0]), shuttleFrame)


def SetCommand(accelCommand: npt.NDArray, angAccelCommand: npt.NDArray) -> None:
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAccel"], accelCommand)
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAngAccel"], angAccelCommand)


#############################################################################################
# DON'T CHANGE ANY OF THE ABOVE. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################


#########################################################
# EXAMPLE USER CODE (NOTE: RUNNING A LOOP IS NECESSARY) #
#########################################################


# Qualification baseline. All navigation vectors below use the Phobos frame.
# Tuning values require validation with the official simulator/jet allocator.
MAX_SPEED = 12.0
MAX_ACCEL = 0.30
BRAKE_ACCEL = 0.12
VELOCITY_GAIN = 0.6
HORIZON = 40
PLAN_PERIOD = 0.5
HOLD_RADIUS = 45.0


def limit_vector(vector, maximum):
    vector = np.asarray(vector, dtype=float)
    magnitude = np.linalg.norm(vector)
    return vector * min(1.0, maximum / max(magnitude, 1e-12))


def attitude_command(world_to_body, forward_world, up_world, omega_body, maximum):
    """Point body +X along travel, keep body +Z upright; return body angular accel.

    DCM_WRT is the passive Phobos-to-body transform. The relative active rotation
    below therefore expresses the required physical turn in current body axes.
    """
    current = np.asarray(world_to_body, dtype=float)
    forward = np.asarray(forward_world, dtype=float)
    up = np.asarray(up_world, dtype=float)
    omega = np.asarray(omega_body, dtype=float)
    if (current.shape != (3, 3) or not all(np.isfinite(v).all() for v in
                                         (current, forward, up, omega))):
        return np.zeros(3)
    if np.linalg.norm(forward) < 1e-6:
        forward = current[0].copy()
    forward = forward / np.linalg.norm(forward)
    up = up - forward.dot(up)*forward
    if np.linalg.norm(up) < 1e-6:
        # Keep the existing roll during a near-vertical climb where no unique
        # horizon exists. If that too is parallel, pick a well-conditioned axis.
        up = current[2] - forward.dot(current[2])*forward
        if np.linalg.norm(up) < 1e-6:
            axis = np.eye(3)[np.argmin(np.abs(forward))]
            up = axis - forward.dot(axis)*forward
    up /= np.linalg.norm(up)
    right = np.cross(up, forward)
    desired = np.vstack((forward, right, up))
    relative = current @ desired.T
    angle = np.arccos(np.clip((np.trace(relative)-1.0)*0.5, -1.0, 1.0))
    skew = np.array([relative[2, 1]-relative[1, 2],
                     relative[0, 2]-relative[2, 0], relative[1, 0]-relative[0, 1]])
    if angle < 1e-6:
        rotation = 0.5*skew
    elif np.pi-angle < 1e-4:
        # A cross-product-only attitude error vanishes when exactly upside down.
        _, vectors = np.linalg.eigh((relative+relative.T)*0.5)
        axis = vectors[:, -1]
        if np.linalg.norm(skew) > 1e-8 and axis.dot(skew) < 0.0:
            axis = -axis
        rotation = angle*axis
    else:
        rotation = angle*skew/(2.0*np.sin(angle))
    desired_rate = limit_vector(0.4*rotation, 0.2)
    return limit_vector(desired_rate-omega, maximum)


class QualificationPlanner:
    def __init__(self):
        self.history = []
        self.report_cache = {}
        self.track_counter = 0
        self.approach_direction = None
        self.previous_velocity = np.zeros(3)
        self.last_horizon = HORIZON
        self.last_delta_v = 0.0
        # Deterministic sphere samples plus an explicit goal candidate.
        k = np.arange(48)
        z = 1.0 - 2.0 * (k + 0.5) / len(k)
        angle = k * np.pi * (3.0 - np.sqrt(5.0))
        self.directions = np.column_stack((np.sqrt(1-z*z)*np.cos(angle),
                                          np.sqrt(1-z*z)*np.sin(angle), z))

    def obstacles(self, now, loc, vel, radar):
        self.history.append((now, loc.copy(), vel.copy()))
        self.history = [h for h in self.history if now-h[0] < 60.0]
        times = np.array([h[0] for h in self.history])
        positions = np.array([h[1] for h in self.history])
        velocities = np.array([h[2] for h in self.history])
        centers, speeds, radii = [], [], []
        new_cache = {}
        # Reserve identities for reports that have not changed. Radar sorting is
        # unstable, so list indices must never be used as debris identities.
        used = set()
        valid = []
        for report in radar:
            report = np.asarray(report, dtype=float)
            if report.shape != (3, 3) or not np.isfinite(report).all():
                continue
            valid.append(report)
            cached = self.report_cache.get(tuple(report.ravel()))
            if cached is not None:
                used.add(cached['id'])
        old_tracks = list(self.report_cache.values())
        old_stamps = np.array([track['stamp'] for track in old_tracks])
        old_positions = np.array([track['pos'] for track in old_tracks]).reshape(-1, 3)
        old_speeds = np.array([track['vel'] for track in old_tracks]).reshape(-1, 3)
        old_diameters = np.array([track['diameter'] for track in old_tracks])
        old_ids = np.array([track['id'] for track in old_tracks])
        available = np.array([identity not in used for identity in old_ids], dtype=bool)
        for report in sorted(valid, key=lambda r: r[2, 0]):
            measured = report[0]
            distance = np.linalg.norm(measured)
            if distance < 1e-6:
                continue
            stamp = float(report[2, 0])
            age = max(0.0, now-stamp)
            old_loc = np.array([np.interp(stamp, times, positions[:, i]) for i in range(3)])
            old_vel = np.array([np.interp(stamp, times, velocities[:, i]) for i in range(3)])
            direction = measured / distance
            radial_speed = report[1, 1] + old_vel.dot(direction)
            key = tuple(report.ravel())
            track = self.report_cache.get(key)
            if track is None:
                absolute = measured + old_loc
                matches = []
                dt = stamp-old_stamps
                residuals = np.linalg.norm(absolute-old_positions-dt[:, None]*old_speeds, axis=1)
                eligible = (available & (dt >= 0.4) & (dt <= 8.0)
                            & (np.abs(report[1, 2]-old_diameters)
                               <= np.maximum(3.0, 0.25*np.abs(old_diameters)))
                            & (residuals < 8.0+2.0*dt))
                for index in np.flatnonzero(eligible):
                    matches.append((residuals[index], old_tracks[index]))
                matches.sort(key=lambda m: m[0])
                speed = radial_speed*direction
                identity = self.track_counter
                self.track_counter += 1
                # Ambiguous associations retain the radial-only estimate. A false
                # track is worse than explicitly admitting missing transverse speed.
                if matches and (len(matches) == 1 or matches[1][0] > matches[0][0]+4.0):
                    old = matches[0][1]
                    dt = stamp-old['stamp']
                    measured_speed = (absolute-old['pos'])/dt
                    filtered = 0.7*old['vel'] + 0.3*limit_vector(measured_speed, 4.0)
                    speed = filtered + (radial_speed-filtered.dot(direction))*direction
                    identity = old['id']
                    available[old_ids == identity] = False
                track = dict(id=identity, stamp=stamp, pos=absolute,
                             vel=speed, diameter=report[1, 2])
            used.add(track['id'])
            new_cache[key] = track
            speed = track['vel']
            center = track['pos'] - loc + speed*age
            # Missing pre-start history and clock skew enlarge uncertainty.
            missing_history = max(0.0, times[0]-stamp)
            radius = (0.575 * max(0.0, report[1, 2]) + 3.0 + 8.0
                      + 1.0*age + np.linalg.norm(old_vel)*missing_history)
            centers.append(center)
            speeds.append(speed)
            radii.append(radius)
        self.report_cache = new_cache
        return (np.asarray(centers).reshape(-1, 3),
                np.asarray(speeds).reshape(-1, 3), np.asarray(radii))

    def command(self, now, loc, vel, target_loc, target_vel, radar):
        loc, vel = np.asarray(loc, dtype=float), np.asarray(vel, dtype=float)
        error = np.asarray(target_loc, dtype=float)-loc
        distance = np.linalg.norm(error)
        # Enter the official 50 m success sphere and hold off the ship centre.
        # Lock the approach side for the terminal segment instead of chasing a
        # changing radial offset, which could command motion through the ship.
        approach = -error / max(distance, 1e-9)
        if distance > 1e-6 and distance <= 200.0 and self.approach_direction is None:
            self.approach_direction = approach.copy()
        if self.approach_direction is not None:
            approach = self.approach_direction
        error = error + HOLD_RADIUS*approach
        goal_distance = np.linalg.norm(error)
        direction = error / max(goal_distance, 1e-9)
        speed = min(MAX_SPEED, np.sqrt(2*BRAKE_ACCEL*goal_distance), 0.4*goal_distance)
        desired = np.asarray(target_vel) + speed*direction
        centers, speeds, radii = self.obstacles(now, loc, vel, radar)
        # Retained/coasting commands must obey the current terminal speed limit,
        # rather than carrying an old high-speed command into the holding area.
        previous = np.asarray(target_vel)+limit_vector(self.previous_velocity-target_vel, speed)
        coast = np.asarray(target_vel)+limit_vector(vel-target_vel, speed)
        candidates = [desired, np.asarray(target_vel), previous, coast]
        # Fine alternatives close to the goal direction avoid making a large
        # sideways detour just because the coarse sphere misses a safe corridor.
        if goal_distance > 1e-6:
            reference = np.eye(3)[np.argmin(np.abs(direction))]
            lateral = np.cross(direction, reference)
            lateral /= np.linalg.norm(lateral)
            vertical = np.cross(direction, lateral)
            for angle in (np.deg2rad(12.0), np.deg2rad(25.0)):
                for azimuth in np.arange(8)*np.pi/4:
                    offset = np.cos(azimuth)*lateral + np.sin(azimuth)*vertical
                    candidates.append(np.asarray(target_vel) + speed*(np.cos(angle)*direction+np.sin(angle)*offset))
        for fraction in (1.0, 0.5):
            candidates.extend(np.asarray(target_vel) + fraction*speed*self.directions)
        candidates = np.asarray(candidates)
        # Cover the nominal full-stop time plus reaction time at higher speeds.
        # Retain a bounded horizon so uncertainty/CPU cost cannot grow unbounded.
        horizon = int(np.clip(np.ceil(np.linalg.norm(vel-target_vel)/MAX_ACCEL)+8, HORIZON, 65))
        self.last_horizon = horizon
        # Conservative reachability filter avoids spending rollout time on distant
        # objects; includes current speed so an overspeed shuttle stays protected.
        if len(centers):
            travel = horizon * max(np.linalg.norm(vel), np.linalg.norm(candidates, axis=1).max())
            relevant = np.linalg.norm(centers, axis=1) <= (
                travel + horizon*np.linalg.norm(speeds, axis=1) + radii + horizon)
            centers, speeds, radii = centers[relevant], speeds[relevant], radii[relevant]
        count = len(candidates)
        trial_vel = np.tile(vel, (count, 1))
        trial_pos = np.zeros((count, 3))
        clearance = np.full(count, np.inf)
        risk = np.zeros(count)
        delta_v = np.zeros(count)
        # Acceleration-limited rollout; segment checks prevent tunnelling between steps.
        for step in range(horizon):
            accel = VELOCITY_GAIN*(candidates-trial_vel)
            # Roll out the goal candidate with the same braking guidance used
            # during real flight, so it does not predict flying through arrival.
            remaining = error+step*np.asarray(target_vel)-trial_pos[0]
            length = np.linalg.norm(remaining)
            guided_speed = min(MAX_SPEED, np.sqrt(2*BRAKE_ACCEL*length), 0.4*length)
            guided_velocity = np.asarray(target_vel)+guided_speed*remaining/max(length, 1e-9)
            accel[0] = VELOCITY_GAIN*(guided_velocity-trial_vel[0])
            accel *= np.minimum(1.0, MAX_ACCEL/np.maximum(np.linalg.norm(accel, axis=1), 1e-12))[:, None]
            delta_v += np.linalg.norm(accel, axis=1)
            next_pos = trial_pos + trial_vel + 0.5*accel
            if len(centers):
                start = centers[None, :, :] + step*speeds[None, :, :] - trial_pos[:, None, :]
                end = centers[None, :, :] + (step+1)*speeds[None, :, :] - next_pos[:, None, :]
                delta = end-start
                fraction = np.clip(-np.sum(start*delta, axis=2)/np.maximum(np.sum(delta*delta, axis=2), 1e-12), 0, 1)
                gap = np.linalg.norm(start+fraction[:, :, None]*delta, axis=2)-radii[None, :]-1.0*(step+1)
                clearance = np.minimum(clearance, gap.min(axis=1))
                # Accumulated penetration distinguishes escape directions even
                # when all candidates share the same initially negative clearance.
                risk += np.maximum(0.0, -gap).sum(axis=1)/(1.0+0.1*step)
            trial_pos = next_pos
            trial_vel += accel
        cost = (np.linalg.norm(trial_pos-error-horizon*np.asarray(target_vel), axis=1)
                + 1.5*np.linalg.norm(candidates-self.previous_velocity, axis=1)
                + 1.5*delta_v)
        safe = clearance > 0.0
        if safe.any():
            cost[~safe] = np.inf
            best = int(np.argmin(cost))
            mode = 'GUIDE' if best == 0 else 'AVOID'
            if goal_distance < 3.0 and np.linalg.norm(vel-target_vel) < 0.5:
                mode = 'HOLD'
        else:
            # No safe rollout: minimize accumulated predicted penetration.
            best = int(np.argmin(risk + 0.01*cost))
            mode = 'ESCAPE'
        self.previous_velocity = candidates[best].copy()
        self.last_delta_v = float(delta_v[best])
        return (limit_vector(VELOCITY_GAIN*(candidates[best]-vel), MAX_ACCEL),
                mode, float(clearance[best]))


planner = QualificationPlanner()
heading_world = None
# Radar.py timestamps are relative to its own startup clock, not StartDateTAI.
# Controller and radar start concurrently; their small startup skew is covered by margins.
radar_epoch = st.SimGlobals.SimClock.GetTimeNow().as_datetime()
last_plan = -np.inf
last_log = -np.inf
accel_world = np.zeros(3)
mode, clearance = 'START', np.inf
linear_limit = min(MAX_ACCEL, shuttle.GetParam(st.VarType.double, ['Thrusters', 'MaxAccel_m_s2']))
angular_limit = min(0.02, shuttle.GetParam(st.VarType.double, ['Thrusters', 'MaxAngAccel_rad_s2']))
try:
    while True:
        time.sleep(1.0 / this.GetParam(st.VarType.double, 'LoopFreqHz'))
        now = (st.SimGlobals.SimClock.GetTimeNow().as_datetime()-radar_epoch).total_seconds()
        if now < last_plan:
            planner = QualificationPlanner()
            heading_world = None
            last_plan = last_log = -np.inf
        state = shuttle.getLocVelAcc()
        loc = np.asarray(state.loc_WRT_ExprIn(phobosFrame))
        vel = np.asarray(state.vel_WRT_ExprIn(phobosFrame))
        target_state = target.getLocVelAcc()
        target_loc = np.asarray(target_state.loc_WRT_ExprIn(phobosFrame))
        target_vel = np.asarray(target_state.vel_WRT_ExprIn(phobosFrame))
        if not all(np.isfinite(v).all() for v in (loc, vel, target_loc, target_vel)):
            SetCommand(np.zeros(3), np.zeros(3))
            last_plan = -np.inf
            continue
        if now-last_plan >= PLAN_PERIOD:
            # Read the entire radar list; stale sorting can hide an approaching object.
            radar = GetNearestDebrisLocations(10000)
            radar = [np.asarray(report, dtype=float) for report in radar]
            radar = [report for report in radar if report.shape == (3, 3)
                     and np.isfinite(report).all() and np.linalg.norm(report[0]) > 1e-6
                     and -5.0 <= now-report[2, 0] <= 10.0]
            if len(radar) == 0:
                planner.previous_velocity = target_vel.copy()
                mode, clearance = 'WAIT_RADAR', float('nan')
            else:
                accel_world, mode, clearance = planner.command(
                    now, loc, vel, target_loc, target_vel, radar)
            last_plan = now
        # The planner runs at 2 Hz; the velocity servo runs every control tick.
        # Reusing a fixed acceleration for the whole planning period can overshoot.
        accel_world = limit_vector(VELOCITY_GAIN*(planner.previous_velocity-vel), linear_limit)
        accel_body = st.frames.FramedVector(accel_world, phobosFrame).ExprIn(shuttleFrame)
        omega_body = np.asarray(shuttle.getAngVelocity().WRT(phobosFrame).ExprIn(shuttleFrame))
        # Face the actual direction of travel, not the braking thrust vector.
        # At low speed use the planned direction, retaining heading when holding.
        travel_world = vel-target_vel
        if np.linalg.norm(travel_world) < 1.0:
            travel_world = planner.previous_velocity-target_vel
        if np.linalg.norm(travel_world) >= 0.2:
            heading_world = travel_world.copy()
        body_dcm = np.asarray(shuttle.getRotation().DCM_WRT(phobosFrame))
        if heading_world is None:
            heading_world = body_dcm[0].copy()
        # Phobos-centred position defines the local outward/up direction.
        up_world = loc/max(np.linalg.norm(loc), 1e-9)
        if np.linalg.norm(up_world) < 1e-6:
            up_world = np.array([0.0, 0.0, 1.0])
        angular_accel = attitude_command(body_dcm, heading_world, up_world, omega_body, angular_limit)
        if not np.isfinite(angular_accel).all():
            angular_accel = np.zeros(3)
        SetCommand(accel_body, angular_accel)
        if now-last_log >= 5.0:
            damage = shuttle.GetParam(st.VarType.double, ['Stats', 'Damage'])
            fuel = shuttle.GetParam(st.VarType.double, ['Stats', 'FuelUsed'])
            st.OnScreenLogMessage(
                f'IGNITION {mode} profile=FAST12 t={now:.1f}s distance={np.linalg.norm(target_loc-loc):.1f}m '
                f'speed={np.linalg.norm(vel-target_vel):.2f}m/s clearance={clearance:.1f}m '
                f'damage={damage:.1f}J fuel={fuel:.2f}kg '
                f'lookahead={planner.last_horizon}s planDV={planner.last_delta_v:.2f}m/s '
                f'cmdAccel={np.linalg.norm(accel_world):.3f}m/s2', 'IgnitionController', st.Severity.Info)
            last_log = now
finally:
    try:
        SetCommand(np.zeros(3), np.zeros(3))
    finally:
        st.leave_sim()
