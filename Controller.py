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


# Final-round candidate. All navigation vectors below use the Phobos frame.
# Tuning values require validation with the official simulator/jet allocator.
from collections import deque
from threading import Lock

MAX_SPEED = 12.0
MAX_ACCEL = 0.30
BRAKE_ACCEL = 0.12
VELOCITY_GAIN = 0.6
HORIZON = 40
PLAN_PERIOD = 0.5
HOLD_RADIUS = 45.0
CAMERA_WIDTH = 96
CAMERA_HEIGHT = 64
CAMERA_FOV = 80.0
CAMERA_PERIOD = 0.25
CAMERA_MAX_AGE = 2.0
CAMERA_MAX_RANGE = 220.0


def limit_vector(vector, maximum):
    vector = np.asarray(vector, dtype=float)
    magnitude = np.linalg.norm(vector)
    return vector * min(1.0, maximum / max(magnitude, 1e-12))


class PoseHistory:
    """Own-vehicle poses only; no hidden obstacle state is accessed."""
    def __init__(self):
        self.samples = deque(maxlen=900)

    def add(self, now, location, rotation):
        if self.samples and now < self.samples[-1][0]:
            self.samples.clear()
        self.samples.append((float(now), np.asarray(location).copy(), np.asarray(rotation).copy()))

    def sample(self, stamp):
        if not self.samples:
            return None
        times = np.array([row[0] for row in self.samples])
        index = int(np.argmin(np.abs(times-stamp)))
        # A capture without a nearby known pose cannot safely become occupancy.
        if abs(times[index]-stamp) > 0.15:
            return None
        position = np.array([np.interp(stamp, times, [row[1][axis] for row in self.samples])
                             for axis in range(3)])
        return position, self.samples[index][2]


def camera_rotation(yaw_deg, pitch_deg):
    yaw, pitch = np.deg2rad([yaw_deg, pitch_deg])
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    return np.array([[cp*cy, cp*sy, -sp], [-sy, cy, 0.], [sp*cy, sp*sy, cp]])


def connected_regions(values, valid, relative_jump=0.12):
    """Four-connected depth/brightness regions; bounded by the small image size."""
    height, width = valid.shape
    visited = np.zeros_like(valid, dtype=bool)
    regions = []
    for row, col in zip(*np.nonzero(valid)):
        if visited[row, col]:
            continue
        visited[row, col] = True
        stack, region = [(row, col)], []
        while stack:
            y, x = stack.pop()
            region.append((y, x))
            for yy, xx in ((y-1, x), (y+1, x), (y, x-1), (y, x+1)):
                if 0 <= yy < height and 0 <= xx < width and valid[yy, xx] and not visited[yy, xx]:
                    if abs(values[yy, xx]-values[y, x]) <= max(2., relative_jump*values[y, x]):
                        visited[yy, xx] = True
                        stack.append((yy, xx))
        if len(region) >= 2:
            regions.append(np.asarray(region))
    return regions


class CameraPerception:
    """Metric occupied volumes from depth; RGB supplies bearing-only detections."""
    def __init__(self):
        self.occupied = {}
        self.visual = {}
        self.frames = 0
        self.rejected = 0
        self.rgb_associations = 0
        self.depth_associations = 0
        self.last_uncertainty = np.empty(0)

    @staticmethod
    def rays(rows, cols, width, height, fov):
        focal = height/(2.*np.tan(np.deg2rad(fov)*0.5))
        # Linear scene depth is camera-forward X, not distance along each ray.
        return np.column_stack((np.ones(len(rows)), (cols+0.5-width/2.)/focal,
                                -(rows+0.5-height/2.)/focal))

    def ingest(self, name, mode, pixels, stamp, now, pose, yaw, pitch):
        if pose is None or not np.isfinite(stamp) or not -0.15 <= now-stamp <= CAMERA_MAX_AGE:
            self.rejected += 1
            return False
        location, body_rotation = pose
        camera_to_world = body_rotation.T @ camera_rotation(yaw, pitch).T
        offset = np.array([1.27, 0., .9]) if name == 'top' else np.array([1.675, 0., -1.025])
        origin = location + body_rotation.T @ offset
        if mode == 'rgb':
            rgb = np.asarray(pixels, dtype=float)
            if rgb.shape != (CAMERA_HEIGHT, CAMERA_WIDTH, 3) or not np.isfinite(rgb).all():
                self.rejected += 1
                return False
            gray = rgb.mean(axis=2)
            # Contrast against black space. Dark surfaces remain visible to depth.
            mask = gray > max(10., float(np.percentile(gray, 30))+12.)
            detections = []
            for region in connected_regions(gray, mask, relative_jump=1.):
                if len(region) < 3:
                    continue
                y, x = region.mean(axis=0)
                ray = self.rays(np.array([y]), np.array([x]), CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV)[0]
                bearing = camera_to_world @ (ray/np.linalg.norm(ray))
                spread = np.deg2rad(CAMERA_FOV)*np.sqrt(len(region))/CAMERA_HEIGHT
                detections.append((origin, bearing, spread))
            self.visual[name] = (stamp, detections[:48])
        else:
            depth = np.asarray(pixels, dtype=float)
            if depth.shape != (CAMERA_HEIGHT, CAMERA_WIDTH):
                self.rejected += 1
                return False
            depth = depth*0.01  # OutputMode.Depth_cm returns centimetres.
            valid = np.isfinite(depth) & (depth > 4.) & (depth < CAMERA_MAX_RANGE)
            volumes = []
            for region in connected_regions(depth, valid):
                # Tile broad surfaces instead of enclosing a whole terrain plane
                # in a huge sphere that could incorrectly contain the rover.
                tile_ids = ((region[:, 0]//12)*8 + region[:, 1]//12
                            if np.ptp(region[:, 0]) >= 12 or np.ptp(region[:, 1]) >= 12
                            else np.zeros(len(region), dtype=int))
                for tile in np.unique(tile_ids):
                    pixels_in_tile = region[tile_ids == tile]
                    rows, cols = pixels_in_tile[:, 0], pixels_in_tile[:, 1]
                    rays = self.rays(rows, cols, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV)
                    points = rays*depth[rows, cols, None]
                    lower, upper = points.min(axis=0), points.max(axis=0)
                    center = .5*(lower+upper)
                    # Radius encloses surfaces, pixels, pose error and rover.
                    radius = .5*np.linalg.norm(upper-lower)+3.+3.+np.linalg.norm(center)*.02
                    volumes.append((origin+camera_to_world@center, radius))
            # Keep the closest regions; broad foreground terrain remains occupied.
            volumes.sort(key=lambda item: np.linalg.norm(item[0]-origin)-item[1])
            previous = self.occupied.get(name)
            if volumes or previous is None or stamp-previous[0] > CAMERA_MAX_AGE:
                self.occupied[name] = (stamp, volumes[:96])
        self.frames += 1
        return True

    def obstacles(self, now, location, radar_centers, radar_speeds, radar_radii):
        centers, speeds, radii = [], [], []
        self.rgb_associations = 0
        self.depth_associations = 0
        uncertainty = []
        absolute_radar = np.asarray(radar_centers).reshape(-1, 3)+location
        # RGB detections become metric only through a radar association. They may
        # strengthen an occupied bound, but never create a made-up monocular range.
        for stamp, detections in self.visual.values():
            if not 0. <= now-stamp <= CAMERA_MAX_AGE:
                continue
            for origin, bearing, spread in detections:
                rays = absolute_radar-origin
                ranges = np.linalg.norm(rays, axis=1)
                angles = np.arccos(np.clip((rays@bearing)/np.maximum(ranges, 1e-9), -1., 1.))
                matched = np.flatnonzero(angles < np.minimum(.2, .04+spread*.5))
                if len(matched):
                    index = int(matched[np.argmin(ranges[matched])])
                    self.rgb_associations += 1
                    centers.append(radar_centers[index])
                    speeds.append(radar_speeds[index])
                    radii.append(radar_radii[index]+1.)
                    uncertainty.append(1.)
        for stamp, volumes in self.occupied.values():
            age = now-stamp
            if not 0. <= age <= CAMERA_MAX_AGE:
                continue
            for absolute, radius in volumes:
                relative = absolute-location
                velocity = np.zeros(3)
                rate = 2.
                if len(radar_centers):
                    separation = np.linalg.norm(radar_centers-relative, axis=1)
                    matches = np.flatnonzero(separation < radar_radii+radius)
                    if len(matches):
                        index = int(matches[np.argmin(separation[matches])])
                        velocity = radar_speeds[index]
                        self.depth_associations += 1
                        rate = 1.
                        if separation[index]+radius+2.*age <= radar_radii[index]:
                            continue  # This entire volume is already covered.
                centers.append(relative+age*velocity)
                speeds.append(velocity)
                radii.append(radius+2.*age)
                uncertainty.append(rate)
        self.last_uncertainty = np.asarray(uncertainty)
        return (np.asarray(centers).reshape(-1, 3), np.asarray(speeds).reshape(-1, 3),
                np.asarray(radii))


class CameraManager:
    """Bounded asynchronous capture. The steering thread never waits for images."""
    def __init__(self, api, cameras, pan, epoch):
        self.api, self.cameras, self.pan, self.epoch = api, cameras, pan, epoch
        self.queue = deque(maxlen=8)
        self.lock = Lock()
        self.pending = {}
        self.failures = dict(top=0, bottom=0)
        self.sequence = dict(top=0, bottom=0)
        self.next_capture = -np.inf
        self.token = 30000
        self.generation = 0
        self.received = 0
        self.error = ''
        self.turn = 0

    def reset(self):
        with self.lock:
            self.generation += 1
            self.queue.clear()
            # Retain outstanding requests until completion/timeout, so resets
            # cannot multiply subscriptions. Old-generation images are ignored.
        for request in self.pending.values():
            request['sent'] = -np.inf
        self.next_capture = -np.inf

    def tick(self, now, travel_body):
        for name, item in list(self.pending.items()):
            if now-item['sent'] > 3.:
                self.failures[name] += 1
                del self.pending[name]
                self.error = name+': capture timeout'
        if now < self.next_capture:
            return
        names = ('top', 'bottom')
        name = names[self.turn % 2]
        self.turn += 1
        if name in self.pending or self.failures[name] >= 3:
            self.next_capture = now+CAMERA_PERIOD
            return
        direction = np.asarray(travel_body)
        yaw = float(np.clip(np.rad2deg(np.arctan2(direction[1], direction[0])), -89., 89.))
        pitch = float(np.rad2deg(np.arctan2(-direction[2], max(np.linalg.norm(direction[:2]), 1e-9))))
        pitch = float(np.clip(pitch, -50., 0.) if name == 'top' else np.clip(pitch, 0., 50.))
        mode = 'rgb' if self.sequence[name] % 3 == 2 else 'depth'
        self.sequence[name] += 1
        self.token += 1
        token, generation = self.token, self.generation
        context = dict(token=token, generation=generation, name=name, mode=mode,
                       yaw=yaw, pitch=pitch, sent=now)
        self.pending[name] = context

        def received(image):
            with self.lock:
                if generation == self.generation:
                    self.queue.append((context, image))
        try:
            self.pan(self.cameras[name], yaw, pitch)
            properties = self.api.CaptureImageProperties()
            properties.ResolutionX, properties.ResolutionY = CAMERA_WIDTH, CAMERA_HEIGHT
            properties.FOV, properties.nonphys_EV = CAMERA_FOV, 10.
            properties.output_mode = (self.api.OutputMode.Depth_cm if mode == 'depth'
                                      else self.api.OutputMode.RGB_LDR_sRGB)
            properties.CaptureID = token
            # Subscribe before publishing: the caller's CaptureID is preserved.
            self.api.camera.OnImageReceived(token, received)
            self.api.camera.CaptureImage(self.cameras[name], properties)
        except Exception as exc:
            self.failures[name] += 1
            self.pending.pop(name, None)
            self.error = type(exc).__name__+': '+str(exc)[:100]
        self.next_capture = now+CAMERA_PERIOD

    def drain(self, now, history, perception):
        with self.lock:
            images = list(self.queue)
            self.queue.clear()
        for context, image in images:
            name = context['name']
            active = self.pending.get(name)
            if active is None or active['token'] != context['token']:
                continue
            self.pending.pop(name, None)
            try:
                stamp = (image.get_timestamp().as_datetime()-self.epoch).total_seconds()
                props = image.properties
                if props.ResolutionX != CAMERA_WIDTH or props.ResolutionY != CAMERA_HEIGHT:
                    raise ValueError('unexpected camera resolution')
                if abs(float(props.FOV)-CAMERA_FOV) > .1:
                    raise ValueError('unexpected camera field of view')
                if context['mode'] == 'depth':
                    pixels = np.asarray(image.as_f32().Pixels).reshape(CAMERA_HEIGHT, CAMERA_WIDTH)
                else:
                    rgb = image.as_RGB8()
                    pixels = np.stack([np.asarray(channel).reshape(CAMERA_HEIGHT, CAMERA_WIDTH)
                                       for channel in (rgb.PixelsR, rgb.PixelsG, rgb.PixelsB)], axis=2)
                accepted = perception.ingest(name, context['mode'], pixels, stamp, now,
                                             history.sample(stamp), context['yaw'], context['pitch'])
                if accepted:
                    self.received += 1
                else:
                    self.error = name+': rejected stale/unposed image'
            except Exception as exc:
                self.failures[name] += 1
                self.error = type(exc).__name__+': '+str(exc)[:100]


class ScorePolicy:
    """Bounded time/fuel tradeoff estimates, calibrated from own-vehicle telemetry."""
    def __init__(self):
        self.fuel_per_dv = 6.0
        self.acceleration = .14
        self.previous = None
        self.speed = MAX_SPEED
        self.estimated_score = 0.

    def observe(self, now, velocity, fuel):
        velocity = np.asarray(velocity)
        if self.previous is not None:
            old_time, old_velocity, old_fuel = self.previous
            dt = now-old_time
            dv = np.linalg.norm(velocity-old_velocity)
            spent = fuel-old_fuel
            if .3 <= dt <= 2. and dv > .05 and spent > .01:
                self.fuel_per_dv = .95*self.fuel_per_dv+.05*np.clip(spent/dv, 4.4, 18.)
                self.acceleration = .95*self.acceleration+.05*np.clip(dv/dt, .06, .30)
        self.previous = now, velocity.copy(), float(fuel)

    def choose(self, distance, relative_speed, elapsed, fuel, maneuvering=False):
        choices = np.arange(8., MAX_SPEED+.1, 1.)
        accel = max(.06, min(.20, self.acceleration))
        durations = distance/choices + np.abs(choices-relative_speed)/(2.*accel) + choices/(2.*BRAKE_ACCEL)
        # Useful translation impulse, terminal braking, and a bounded allowance
        # for correcting gravity. Historical fuel is included in the score.
        delta_v = np.abs(choices-relative_speed)+choices+.006*durations
        projected = 100.*(120./np.maximum(elapsed+durations, 1.)
                          -(fuel+self.fuel_per_dv*delta_v)/75.+.5)
        best = int(np.argmax(projected))
        requested = float(choices[best])
        # The simple score forecast omits time/fuel spent dodging moving debris.
        # In a constrained passage retain maneuver progress instead of slowing
        # based on a free-space fuel estimate that has not modelled that passage.
        if maneuvering:
            requested = MAX_SPEED
        # Keep speed decisions stable; route changes should respond to obstacles.
        self.speed += float(np.clip(requested-self.speed, -.5, .5))
        self.estimated_score = float(projected[best])
        return self.speed


def velocity_command(desired, measured, maximum, urgent=False):
    error = np.asarray(desired)-np.asarray(measured)
    if not urgent and np.linalg.norm(error) < .12:
        return np.zeros(3)  # Coast rather than firing jets for tiny corrections.
    return limit_vector(VELOCITY_GAIN*error, maximum)


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
    if angle < np.deg2rad(.5) and np.linalg.norm(omega) < .003:
        return np.zeros(3)
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
        self.urgent = False
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

    def command(self, now, loc, vel, target_loc, target_vel, radar,
                camera=None, cruise_speed=None, maneuver_accel=None, fuel_weight=1.5):
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
        cruise = MAX_SPEED if cruise_speed is None else float(np.clip(cruise_speed, 0., MAX_SPEED))
        predictive_accel = MAX_ACCEL if maneuver_accel is None else float(np.clip(maneuver_accel, .08, MAX_ACCEL))
        speed = min(cruise, np.sqrt(2*BRAKE_ACCEL*goal_distance), 0.4*goal_distance)
        desired = np.asarray(target_vel) + speed*direction
        centers, speeds, radii = self.obstacles(now, loc, vel, radar)
        uncertainty_rates = np.ones(len(centers))
        if camera is not None:
            camera_centers, camera_speeds, camera_radii = camera.obstacles(now, loc, centers, speeds, radii)
            centers = np.vstack((centers, camera_centers))
            speeds = np.vstack((speeds, camera_speeds))
            radii = np.concatenate((radii, camera_radii))
            uncertainty_rates = np.concatenate((uncertainty_rates, camera.last_uncertainty))
        self.urgent = False
        if len(centers):
            ranges = np.linalg.norm(centers, axis=1)
            gaps = ranges-radii
            closing = np.sum((vel-speeds)*centers, axis=1)/np.maximum(ranges, 1e-9)
            self.urgent = bool(np.any((gaps < 3.) | ((closing > 0.) & (gaps < 3.*closing))))
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
        horizon = int(np.clip(np.ceil(np.linalg.norm(vel-target_vel)/predictive_accel)+8, HORIZON, 75))
        self.last_horizon = horizon
        # Conservative reachability filter avoids spending rollout time on distant
        # objects; includes current speed so an overspeed shuttle stays protected.
        if len(centers):
            travel = horizon * max(np.linalg.norm(vel), np.linalg.norm(candidates, axis=1).max())
            relevant = np.linalg.norm(centers, axis=1) <= (
                travel + horizon*np.linalg.norm(speeds, axis=1) + radii + horizon*uncertainty_rates)
            centers, speeds, radii = centers[relevant], speeds[relevant], radii[relevant]
            uncertainty_rates = uncertainty_rates[relevant]
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
            guided_speed = min(cruise, np.sqrt(2*BRAKE_ACCEL*length), 0.4*length)
            guided_velocity = np.asarray(target_vel)+guided_speed*remaining/max(length, 1e-9)
            accel[0] = VELOCITY_GAIN*(guided_velocity-trial_vel[0])
            accel *= np.minimum(1.0, predictive_accel/np.maximum(np.linalg.norm(accel, axis=1), 1e-12))[:, None]
            delta_v += np.linalg.norm(accel, axis=1)
            next_pos = trial_pos + trial_vel + 0.5*accel
            if len(centers):
                start = centers[None, :, :] + step*speeds[None, :, :] - trial_pos[:, None, :]
                end = centers[None, :, :] + (step+1)*speeds[None, :, :] - next_pos[:, None, :]
                delta = end-start
                fraction = np.clip(-np.sum(start*delta, axis=2)/np.maximum(np.sum(delta*delta, axis=2), 1e-12), 0, 1)
                gap = (np.linalg.norm(start+fraction[:, :, None]*delta, axis=2)-radii[None, :]
                       -uncertainty_rates[None, :]*(step+1))
                clearance = np.minimum(clearance, gap.min(axis=1))
                # Accumulated penetration distinguishes escape directions even
                # when all candidates share the same initially negative clearance.
                risk += np.maximum(0.0, -gap).sum(axis=1)/(1.0+0.1*step)
            trial_pos = next_pos
            trial_vel += accel
        cost = (np.linalg.norm(trial_pos-error-horizon*np.asarray(target_vel), axis=1)
                + 1.5*np.linalg.norm(candidates-self.previous_velocity, axis=1)
                + float(np.clip(fuel_weight, 1.5, 30.))*delta_v)
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
        return (velocity_command(candidates[best], vel, MAX_ACCEL,
                                 urgent=self.urgent or distance < 80.),
                mode, float(clearance[best]))


planner = QualificationPlanner()
heading_world = None
# Radar.py timestamps are relative to its own startup clock, not StartDateTAI.
# Controller and radar start concurrently; their small startup skew is covered by margins.
radar_epoch = st.SimGlobals.SimClock.GetTimeNow().as_datetime()
pose_history = PoseHistory()
perception = CameraPerception()
camera_manager = CameraManager(st, dict(top=topCamera, bottom=bottomCamera), PanTiltCamera, radar_epoch)
score_policy = ScorePolicy()
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
            pose_history = PoseHistory()
            perception = CameraPerception()
            score_policy = ScorePolicy()
            camera_manager.reset()
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
        body_dcm = np.asarray(shuttle.getRotation().DCM_WRT(phobosFrame))
        if body_dcm.shape != (3, 3) or not np.isfinite(body_dcm).all():
            SetCommand(np.zeros(3), np.zeros(3))
            continue
        pose_history.add(now, loc, body_dcm)
        camera_manager.drain(now, pose_history, perception)
        sightline = vel-target_vel
        if np.linalg.norm(sightline) < 1.:
            sightline = target_loc-loc
        camera_manager.tick(now, body_dcm@sightline)
        if now-last_plan >= PLAN_PERIOD:
            # Read the entire radar list; stale sorting can hide an approaching object.
            radar = GetNearestDebrisLocations(10000)
            radar = [np.asarray(report, dtype=float) for report in radar]
            radar = [report for report in radar if report.shape == (3, 3)
                     and np.isfinite(report).all() and np.linalg.norm(report[0]) > 1e-6
                     and -5.0 <= now-report[2, 0] <= 10.0]
            fuel = shuttle.GetParam(st.VarType.double, ['Stats', 'FuelUsed'])
            score_policy.observe(now, vel, fuel)
            cruise = score_policy.choose(max(0., np.linalg.norm(target_loc-loc)-HOLD_RADIUS),
                                         np.linalg.norm(vel-target_vel), now, fuel,
                                         maneuvering=mode == 'ESCAPE' or clearance < 25.)
            fresh_depth = any(0. <= now-stamp <= CAMERA_MAX_AGE and len(regions)
                              for stamp, regions in perception.occupied.values())
            if len(radar) == 0 and not fresh_depth:
                planner.previous_velocity = target_vel.copy()
                mode, clearance = 'WAIT_RADAR', float('nan')
            else:
                accel_world, mode, clearance = planner.command(
                    now, loc, vel, target_loc, target_vel, radar, camera=perception,
                    cruise_speed=cruise)
            last_plan = now
        # The planner runs at 2 Hz; the velocity servo runs every control tick.
        # Reusing a fixed acceleration for the whole planning period can overshoot.
        accel_world = velocity_command(planner.previous_velocity, vel, linear_limit,
                                       urgent=planner.urgent or np.linalg.norm(target_loc-loc) < 80.)
        accel_body = st.frames.FramedVector(accel_world, phobosFrame).ExprIn(shuttleFrame)
        omega_body = np.asarray(shuttle.getAngVelocity().WRT(phobosFrame).ExprIn(shuttleFrame))
        # Face the actual direction of travel, not the braking thrust vector.
        # At low speed use the planned direction, retaining heading when holding.
        travel_world = vel-target_vel
        if np.linalg.norm(travel_world) < 1.0:
            travel_world = planner.previous_velocity-target_vel
        if np.linalg.norm(travel_world) >= 0.2:
            heading_world = travel_world.copy()
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
                f'IGNITION {mode} profile=FINAL_FUSION t={now:.1f}s distance={np.linalg.norm(target_loc-loc):.1f}m '
                f'speed={np.linalg.norm(vel-target_vel):.2f}m/s clearance={clearance:.1f}m '
                f'damage={damage:.1f}J fuel={fuel:.2f}kg '
                f'lookahead={planner.last_horizon}s planDV={planner.last_delta_v:.2f}m/s '
                f'cmdAccel={np.linalg.norm(accel_world):.3f}m/s2 cruise={score_policy.speed:.1f}m/s '
                f'images={camera_manager.received} rejected={perception.rejected} '
                f'rgbMatches={perception.rgb_associations} depthMatches={perception.depth_associations} '
                f'cameraFault={camera_manager.error or "none"}',
                'IgnitionController', st.Severity.Info)
            last_log = now
finally:
    try:
        SetCommand(np.zeros(3), np.zeros(3))
    finally:
        st.leave_sim()
