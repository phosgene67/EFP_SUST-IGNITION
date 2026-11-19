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

MAX_SPEED = 25.0
MIN_CRUISE_SPEED = 18.0
MAX_ACCEL = 0.50
BRAKE_ACCEL = 0.12
VELOCITY_GAIN = 0.6
HORIZON = 40
MAX_LOOKAHEAD = 220
PLAN_PERIOD = 0.5
HOLD_RADIUS = 49.0
SHIP_BRAKE_ACCEL = 0.075
CHUNK_CRUISE_SPEED = 9.0
LAUNCH_HEIGHT = 3.5
CAMERA_WIDTH = 96
CAMERA_HEIGHT = 64
CAMERA_FOV = 80.0
CAMERA_PERIOD = 0.25
CAMERA_MAX_AGE = 2.0
CAMERA_MAX_RANGE = 400.0


def limit_vector(vector, maximum):
    vector = np.asarray(vector, dtype=float)
    magnitude = np.linalg.norm(vector)
    return vector * min(1.0, maximum / max(magnitude, 1e-12))


def axis_thrust(acceleration, world_to_body, maximum):
    """Pulse one body axis; velocity feedback supplies the remaining axes later."""
    rotation = np.asarray(world_to_body, float)
    body = rotation@np.asarray(acceleration, float)
    if np.linalg.norm(body) < 1e-9:
        return np.zeros(3)
    axis = int(np.argmax(np.abs(body)))
    pulse = np.zeros(3)
    pulse[axis] = np.copysign(maximum, body[axis])
    return rotation.T@pulse


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
        self.boxes = {}
        self.depth_views = {}

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
            boxes = []
            for region in connected_regions(gray, mask, relative_jump=1.):
                if len(region) < 3:
                    continue
                y, x = region.mean(axis=0)
                ray = self.rays(np.array([y]), np.array([x]), CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV)[0]
                bearing = camera_to_world @ (ray/np.linalg.norm(ray))
                spread = np.deg2rad(CAMERA_FOV)*np.sqrt(len(region))/CAMERA_HEIGHT
                detections.append((origin, bearing, spread))
                boxes.append(dict(xyxy=[int(region[:, 1].min()), int(region[:, 0].min()),
                                        int(region[:, 1].max()), int(region[:, 0].max())], range_m=None))
            self.visual[name] = (stamp, detections[:48])
            self.boxes[(name, mode)] = boxes[:48]
        else:
            depth = np.asarray(pixels, dtype=float)
            if depth.shape != (CAMERA_HEIGHT, CAMERA_WIDTH):
                self.rejected += 1
                return False
            self.depth_views[name] = (stamp, origin.copy(), camera_to_world.copy())
            depth = depth*0.01  # OutputMode.Depth_cm returns centimetres.
            valid = np.isfinite(depth) & (depth > 4.) & (depth < CAMERA_MAX_RANGE)
            volumes = []
            boxes = []
            for region in connected_regions(depth, valid):
                boxes.append(dict(xyxy=[int(region[:, 1].min()), int(region[:, 0].min()),
                                        int(region[:, 1].max()), int(region[:, 0].max())],
                                  range_m=float(depth[region[:, 0], region[:, 1]].min())))
                # Tile broad surfaces instead of enclosing a whole terrain plane
                # in a huge sphere that could incorrectly contain the rover.
                # Distant compact ship/debris surfaces need finer cells: one
                # 12-pixel sphere could falsely cover every clear capture side.
                tile_size = 2 if np.median(depth[region[:, 0], region[:, 1]]) > 40. else 12
                columns = int(np.ceil(CAMERA_WIDTH/tile_size))
                tile_ids = ((region[:, 0]//tile_size)*columns + region[:, 1]//tile_size
                            if np.ptp(region[:, 0]) >= tile_size or np.ptp(region[:, 1]) >= tile_size
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
                self.occupied[name] = (stamp, volumes[:384])
            self.boxes[(name, mode)] = boxes[:64]
        self.frames += 1
        return True

    def observes(self, now, point):
        """A fresh depth capture actually covered this world-space bearing."""
        tan_y = np.tan(np.deg2rad(CAMERA_FOV/2.-3.))
        tan_x = tan_y*CAMERA_WIDTH/CAMERA_HEIGHT
        for stamp, origin, to_world in self.depth_views.values():
            view = to_world.T@(np.asarray(point)-origin)
            if (0. <= now-stamp <= CAMERA_MAX_AGE and 4. < view[0] < CAMERA_MAX_RANGE
                    and abs(view[1]) < view[0]*tan_x and abs(view[2]) < view[0]*tan_y):
                return True
        return False

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
    def __init__(self, api, cameras, pan, epoch, preview=None):
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
        self.preview = preview

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
                if self.preview is not None:
                    try:
                        self.preview.submit(name, context['mode'], pixels, stamp,
                            perception.boxes.get((name, context['mode']), []) if accepted else [],
                            accepted, context['yaw'], context['pitch'])
                    except Exception as exc:
                        self.preview.error = 'Preview publish failed: '+str(exc)
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
        choices = np.arange(MIN_CRUISE_SPEED, MAX_SPEED+.1, 1.)
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
    if np.linalg.norm(error) < (.04 if urgent else .12):
        return np.zeros(3)  # Coast rather than firing jets for tiny corrections.
    return limit_vector(VELOCITY_GAIN*error, maximum)


class FuelLedger:
    """Sampled native fuel accounting; phase changes have their own bucket."""
    def __init__(self):
        self.previous = None
        self.by_phase = {}
        self.frozen = False

    def observe(self, now, fuel, phase, arrived=False):
        if not np.isfinite(now) or not np.isfinite(fuel) or fuel < 0.:
            return
        if self.previous is not None and (now < self.previous[0] or fuel < self.previous[1]-.01):
            self.__init__()
        if self.frozen:
            return
        if self.previous is None:
            label, spent = 'BEFORE_FIRST_SAMPLE', fuel
        else:
            _, old_fuel, old_phase = self.previous
            label = old_phase if old_phase == phase else 'PHASE_TRANSITION'
            spent = max(0., fuel-old_fuel)
        if arrived:
            label = 'ARRIVAL_TRANSITION'
        self.by_phase[label] = self.by_phase.get(label, 0.)+spent
        self.previous = float(now), float(fuel), str(phase)
        self.frozen = bool(arrived)

    def summary(self):
        return ','.join(f'{phase}:{fuel:.2f}' for phase, fuel in
                        sorted(self.by_phase.items(), key=lambda item: -item[1]) if fuel > .005)


def mission_score(elapsed, fuel, damage=0.):
    return 100.*(.5+120./max(float(elapsed), .1)-max(float(fuel), 0.)/75.
                 -2.*max(float(damage), 0.)/209200.)


def arrival_forecast(now, fuel, damage, locations, velocities, goal, effort,
                     kg_per_dv, acceleration, gravity):
    """Best burn/coast continuation per candidate, including its deferred turn."""
    offsets = np.asarray(goal)-np.asarray(locations)
    ranges = np.linalg.norm(offsets, axis=1)
    speeds = np.linalg.norm(velocities, axis=1)
    headings = offsets/np.maximum(ranges, 1e-9)[:, None]
    turn = np.linalg.norm(speeds[:, None]*headings-velocities, axis=1)
    distance = np.maximum(ranges-517., 1.)
    acceleration = float(np.clip(acceleration, .06, .16))
    gravity = float(np.clip(gravity, 0., .02))
    best = np.full(len(speeds), -np.inf)
    for addition in np.arange(0., 8.01, .5):
        peak = np.minimum(10., np.maximum(speeds, 2.)+addition)
        peak = np.maximum(peak, speeds)
        burn = (peak-speeds)/acceleration
        progress = speeds*burn+.5*acceleration*burn*burn
        short = progress >= distance
        burn = np.where(short, (np.sqrt(speeds*speeds+2.*acceleration*distance)-speeds)/acceleration, burn)
        remaining = np.maximum(distance-progress, 0.)
        disc = peak*peak-2.*gravity*remaining
        coast = 2.*remaining/np.maximum(peak+np.sqrt(np.maximum(disc, 0.)), 1e-9)
        duration = burn+np.where(short, 0., coast)
        valid = short | (disc > 0.)
        future_fuel = kg_per_dv*(np.asarray(effort)+turn+(acceleration+gravity)*burn)
        arrival = now+turn/acceleration+duration
        score = 100.*(.5+120./np.maximum(arrival, .1)-(fuel+future_fuel)/75.-2.*damage/209200.)
        best = np.maximum(best, np.where(valid, score, -np.inf))
    return best


class EconomyPolicy:
    """Price prospective burn/coast schedules using the actual winning formula."""
    def __init__(self):
        self.fuel_per_dv, self.acceleration = 4.5, .16
        self.estimated_score, self.speed, self.remaining_time = -np.inf, 6., 300.
        self.burn_seconds = 60.
        self.previous = None

    def observe(self, now, velocity, fuel, thrusting=False, gravity=None):
        velocity = np.asarray(velocity)
        if self.previous is not None:
            stamp, old_velocity, old_fuel = self.previous
            dt, dv, spent = now-stamp, np.linalg.norm(velocity-old_velocity), fuel-old_fuel
            if thrusting and .3 <= dt <= 2. and dv > .02 and spent > .01:
                thrust_dv = np.linalg.norm(velocity-old_velocity-dt*np.asarray(gravity)) if gravity is not None else dv
                self.fuel_per_dv = .9*self.fuel_per_dv+.1*np.clip(spent/max(thrust_dv, .001), 4.32, 14.)
                self.acceleration = .9*self.acceleration+.1*np.clip(dv/dt, .05, .18)
        self.previous = now, velocity.copy(), float(fuel)

    def choose(self, distance, forward_speed, elapsed, fuel, damage, gravity):
        distance = max(float(distance), 1.)
        speed, gravity = max(float(forward_speed), 0.), float(np.clip(gravity, 0., .02))
        acceleration = float(np.clip(self.acceleration, .06, .17))
        options = []
        # Never spend fuel braking merely because a new economy optimum is
        # slower. Excess speed is shed by coasting unless safety requires braking.
        peaks = np.unique(np.r_[speed, np.arange(max(2., speed), 10.01, .25)])
        for peak in peaks:
            burn = max(0., (peak-speed)/acceleration)
            progress = speed*burn+.5*acceleration*burn*burn
            if progress >= distance:
                duration = (np.sqrt(speed*speed+2.*acceleration*distance)-speed)/acceleration
                burn = duration
            else:
                remaining = distance-progress
                discriminant = peak*peak-2.*gravity*remaining
                if peak < .1 or discriminant <= 0.:
                    continue
                coast = (2.*remaining/(peak+np.sqrt(discriminant))) if gravity > 1e-6 else remaining/peak
                duration = burn+coast
            impulse = (acceleration+gravity)*burn
            # A small explicit steering allowance; telemetry revises useful
            # kg/delta-v rather than pretending rotations/corrections are free.
            future_fuel = self.fuel_per_dv*impulse+2.
            score = mission_score(elapsed+duration, fuel+future_fuel, damage)
            options.append((score, peak, burn, duration))
        if options:
            self.estimated_score, self.speed, self.burn_seconds, self.remaining_time = max(options)
        else:
            self.speed, self.burn_seconds = max(speed, 8.), 30.
            self.remaining_time = distance/max(self.speed, 1.)
            self.estimated_score = -np.inf
        return float(self.speed)


def ballistic_pass(location, velocity, target, target_velocity, gravity=None):
    """Closest approach of an unpowered constant-gravity prediction."""
    location = np.asarray(location)
    relative, motion = location-target, velocity-target_velocity
    if relative@motion >= 0.:
        return np.inf, np.inf
    gravity = (-711000.*location/max(np.linalg.norm(location)**3, 1.)
               if gravity is None else np.asarray(gravity))
    coefficients = [.5*(gravity@gravity), 1.5*(motion@gravity), motion@motion+relative@gravity, relative@motion]
    roots = np.roots(np.trim_zeros(coefficients, 'f'))
    times = [float(root.real) for root in roots if abs(root.imag) < 1e-6 and 0. < root.real < 1800.]
    if not times:
        return np.inf, np.inf
    if gravity@motion < 0.:
        apex = -(gravity@motion)/max(gravity@gravity, 1e-12)
        times = [t for t in times if t < apex]
    if not times:
        return np.inf, np.inf
    # The first outbound pass matters. A later mathematical return after the
    # ballistic apex must never justify coasting away from the rescue ship.
    time_to_pass = min(times)
    return time_to_pass, float(np.linalg.norm(relative+time_to_pass*motion+.5*time_to_pass*time_to_pass*gravity))


def ballistic_entry(location, velocity, target, target_velocity, gravity, radius=48.):
    """First inward crossing of an interior scoring sphere, including gravity."""
    r, v, g = np.asarray(location)-target, np.asarray(velocity)-target_velocity, np.asarray(gravity)
    if r@r <= radius*radius:
        return 0.
    coefficients = [.25*(g@g), v@g, v@v+r@g, 2.*(r@v), r@r-radius*radius]
    roots = np.roots(np.trim_zeros(coefficients, 'f'))
    times = [float(t.real) for t in roots if abs(t.imag) < 1e-6 and 0. < t.real < 600.
             and (r+t.real*v+.5*t.real*t.real*g)@(v+t.real*g) < 0.]
    return min(times) if times else np.inf


class EconomyServo:
    """Hysteretic jet pulses; small continuous commands still consume full jets."""
    def __init__(self):
        self.firing = False

    def command(self, desired, measured, maximum, urgent=False, coast=False):
        error = np.asarray(desired)-np.asarray(measured)
        magnitude = np.linalg.norm(error)
        if coast and not urgent:
            self.firing = False
            return np.zeros(3)
        lower, upper = (.015, .035) if urgent else (.12, .32)
        if magnitude < lower:
            self.firing = False
        elif magnitude > upper:
            self.firing = True
        return limit_vector(VELOCITY_GAIN*error, maximum) if self.firing else np.zeros(3)


def course_trim(velocity, target_velocity, correction):
    motion = np.asarray(velocity)-np.asarray(target_velocity)
    speed = np.linalg.norm(motion)
    if speed < .1:
        return np.asarray(velocity).copy()
    heading = motion/speed
    lateral = np.asarray(correction)-(np.asarray(correction)@heading)*heading
    trimmed = motion+limit_vector(lateral, .4)
    return np.asarray(target_velocity)+speed*trimmed/max(np.linalg.norm(trimmed), 1e-9)


def coast_path_clearance(location, velocity, centers, speeds, radii, duration, accelerations=None, gravity=None):
    """Swept unpowered arc, with forecast growth and segment curvature margin."""
    if not len(centers):
        return np.inf
    times = np.linspace(0., min(60., max(duration, 1.)), 31)
    gravity = (-711000.*location/max(np.linalg.norm(location)**3, 1.)
               if gravity is None else np.asarray(gravity))
    arc = times[:, None]*velocity+.5*times[:, None]**2*gravity
    relative = centers[None]+times[:, None, None]*speeds[None]-arc[:, None]
    if accelerations is not None:
        relative += .5*times[:, None, None]**2*np.asarray(accelerations)[None]
    start, delta = relative[:-1], np.diff(relative, axis=0)
    fraction = np.clip(-np.sum(start*delta, axis=2)/np.maximum(np.sum(delta*delta, axis=2), 1e-9), 0., 1.)
    gap = (np.linalg.norm(start+fraction[:, :, None]*delta, axis=2)-radii[None]
           -np.minimum(10., .12*times[1:, None])-.02)
    return float(gap.min())


def attitude_reference(mode, relative_velocity, desired_velocity, ship_direction):
    # Gimbals cannot see behind the rover. While stopped awaiting station depth,
    # turn the body toward the ship instead of retaining the last orbit heading.
    if mode == 'WAIT_SHIP_CAMERA' and np.linalg.norm(relative_velocity) < .4:
        return np.asarray(ship_direction)
    return np.asarray(desired_velocity)


def start_camera_preview(source_file):
    # Load the sibling explicitly: simulator working directories vary.
    from pathlib import Path
    import importlib.util
    viewer_file = Path(source_file).resolve().with_name('camera_viewer.py')
    if not viewer_file.is_file():
        raise FileNotFoundError('Copy camera_viewer.py next to Controller.py to enable the GUI')
    spec = importlib.util.spec_from_file_location('efp_camera_viewer', viewer_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.PreviewPublisher(viewer_script=viewer_file)


def attitude_command(world_to_body, forward_world, up_world, omega_body, maximum, rate_cap=.075, stopping_alpha=.002):
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
    # The real jet allocator can deliver much less torque while translating.
    # Limit turn rate by a conservative angular stopping distance, not merely
    # by the requested angular acceleration. This avoids repeated overshoots.
    turn_rate = min(rate_cap, np.sqrt(2.*stopping_alpha*angle), .35*angle)
    desired_rate = rotation/max(angle, 1e-9)*turn_rate
    if angle < np.deg2rad(2.) and np.linalg.norm(omega) < .006:
        return np.zeros(3)
    return limit_vector(1.5*(desired_rate-omega), maximum)


class FlightAttitude:
    """Nose follows velocity; roll stays continuous through a vertical climb.

    When flight and planet-up are parallel there is no unique horizon. Preserve
    the preceding roll reference instead of amplifying tiny lateral velocity noise.
    """
    def __init__(self):
        self.forward = None
        self.up = None
        self.vertical = False
        self.last_time = None
        self.nose_error_deg = 0.
        self.body_velocity = np.zeros(3)

    def command(self, now, rotation, velocity, desired_velocity, planet_up, omega, maximum):
        rotation = np.asarray(rotation, dtype=float)
        velocity, desired_velocity = np.asarray(velocity), np.asarray(desired_velocity)
        dt = .02 if self.last_time is None else float(np.clip(now-self.last_time, 0., .2))
        self.last_time = float(now)
        travel = velocity if np.linalg.norm(velocity) >= .5 else desired_velocity
        if np.linalg.norm(travel) >= .2:
            self.forward = travel/np.linalg.norm(travel)
        elif self.forward is None:
            self.forward = rotation[0].copy()
        forward = self.forward
        local_up = np.asarray(planet_up, dtype=float).copy()
        local_up /= max(np.linalg.norm(local_up), 1e-9)
        verticality = abs(float(forward@local_up))
        # Hysteresis prevents repeatedly switching roll convention near the pole.
        if self.vertical:
            self.vertical = verticality > np.cos(np.deg2rad(30.))
        else:
            self.vertical = verticality > np.cos(np.deg2rad(20.))
        horizon_up = local_up-forward.dot(local_up)*forward
        transported = rotation[2].copy() if self.up is None else self.up.copy()
        transported -= forward.dot(transported)*forward
        if np.linalg.norm(transported) < .05:
            # Pitch about the existing right axis rather than inventing a roll.
            transported = np.cross(forward, rotation[1])
        if np.linalg.norm(transported) < 1e-6:
            axis = np.eye(3)[np.argmin(np.abs(forward))]
            transported = axis-forward.dot(axis)*forward
        transported /= np.linalg.norm(transported)
        if not self.vertical and np.linalg.norm(horizon_up) > 1e-6:
            horizon_up /= np.linalg.norm(horizon_up)
            if self.up is None:
                transported = horizon_up
            else:
                # Recover planet-relative roll gradually after leaving vertical
                # flight, including the antipodal/upside-down case.
                roll = np.arctan2(forward@np.cross(transported, horizon_up), transported@horizon_up)
                turn = float(np.clip(roll, -.35*dt, .35*dt))
                transported = np.cos(turn)*transported+np.sin(turn)*np.cross(forward, transported)
        self.up = transported
        self.nose_error_deg = float(np.rad2deg(np.arccos(np.clip(rotation[0]@forward, -1., 1.))))
        self.body_velocity = rotation@velocity
        return attitude_command(rotation, forward, transported, omega, maximum)


def segment_clearance(start, end, centers, radii):
    centers = np.asarray(centers).reshape(-1, 3)
    if not len(centers):
        return np.inf
    start, end = np.asarray(start), np.asarray(end)
    delta = end-start
    fraction = np.clip(((centers-start)@delta)/max(float(delta@delta), 1e-9), 0., 1.)
    gaps = np.linalg.norm(centers-start-fraction[:, None]*delta, axis=1)-radii
    # Permit exiting an initially overlapping uncertainty margin, not traversing
    # the object or treating every escape segment as blocked by that same margin.
    initial = np.linalg.norm(centers-start, axis=1)
    escaping = (initial < radii) & ((start-centers)@delta >= 0.)
    gaps[escaping] = np.linalg.norm(centers[escaping]-end, axis=1)-radii[escaping]
    return float(gaps.min())


class CorridorPlanner:
    """Persistent 3-D radar corridor. A* runs occasionally, not every control tick."""
    def __init__(self):
        self.points = []
        self.goal = None
        self.last_plan = -np.inf
        self.expanded = 0
        self.error = ''

    def plan(self, now, location, goal, centers, radii):
        import heapq
        if segment_clearance(np.zeros(3), np.asarray(goal)-location, centers, radii) > 8.:
            self.error = ''
            return [np.asarray(goal).copy()]
        direction = np.asarray(goal)-location
        distance = np.linalg.norm(direction)
        if distance < 1.:
            return [np.asarray(goal).copy()]
        forward = direction/distance
        axis = np.eye(3)[np.argmin(np.abs(forward))]
        side = np.cross(forward, axis)
        side /= np.linalg.norm(side)
        basis = np.vstack((forward, side, np.cross(forward, side)))
        local = np.asarray(centers).reshape(-1, 3)@basis.T
        step = 40.
        lower = np.array([-80., -440., -440.])
        shape = np.array([int(np.ceil((distance+160.)/step))+1, 23, 23])
        occupied = np.zeros(tuple(shape), dtype=bool)
        # Inflate for noisy radar and grid edges. Exact segment checks simplify
        # and validate the chosen polyline against the ungridded radar bounds.
        for center, radius in zip(local, radii):
            radius = float(radius)+26.
            lo = np.maximum(np.floor((center-radius-lower)/step).astype(int), 0)
            hi = np.minimum(np.ceil((center+radius-lower)/step).astype(int)+1, shape)
            if np.any(lo >= hi):
                continue
            axes = [lower[k]+step*np.arange(lo[k], hi[k])-center[k] for k in range(3)]
            sphere = (axes[0][:, None, None]**2+axes[1][None, :, None]**2
                      +axes[2][None, None, :]**2) <= radius*radius
            occupied[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]] |= sphere
        start = tuple(np.rint(-lower/step).astype(int))
        finish = tuple(np.clip(np.rint((np.array([distance, 0., 0.])-lower)/step).astype(int), 0, shape-1))
        occupied[start] = occupied[finish] = False
        offsets = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)
                   if a or b or c]
        weights = {offset: float(np.linalg.norm(offset)) for offset in offsets}
        heap = [(float(np.linalg.norm(np.array(start)-finish)), 0., start)]
        cost, parent, closed = {start: 0.}, {}, set()
        while heap and len(closed) < 22000:
            _, g, node = heapq.heappop(heap)
            if node in closed:
                continue
            if node == finish:
                break
            closed.add(node)
            for offset in offsets:
                neighbor = tuple(node[k]+offset[k] for k in range(3))
                if any(neighbor[k] < 0 or neighbor[k] >= shape[k] for k in range(3)):
                    continue
                if occupied[neighbor] or neighbor in closed:
                    continue
                candidate = g+weights[offset]
                if candidate < cost.get(neighbor, np.inf):
                    cost[neighbor], parent[neighbor] = candidate, node
                    h = float(np.linalg.norm(np.array(neighbor)-finish))
                    heapq.heappush(heap, (candidate+h, candidate, neighbor))
        self.expanded = len(closed)
        if finish not in parent:
            self.error = 'no radar corridor'
            return []
        cells = [finish]
        while cells[-1] != start:
            cells.append(parent[cells[-1]])
        raw = [location+(lower+step*np.asarray(cell))@basis for cell in cells[::-1]]
        raw[0], raw[-1] = location.copy(), np.asarray(goal).copy()
        simplified, index = [raw[0]], 0
        while index < len(raw)-1:
            next_index = index+1
            for candidate in range(len(raw)-1, index, -1):
                if segment_clearance(raw[index]-location, raw[candidate]-location, centers, radii) > 12.:
                    next_index = candidate
                    break
            simplified.append(raw[next_index])
            index = next_index
        self.error = ''
        if any(segment_clearance(a-location, b-location, centers, radii) <= 0.
               for a, b in zip(simplified, simplified[1:])):
            self.error = 'grid route failed continuous clearance check'
            return []
        return simplified[1:]

    def waypoint(self, now, location, goal, centers, radii):
        changed = self.goal is None or np.linalg.norm(goal-self.goal) > 30.
        if changed or not self.points or now-self.last_plan > 5.:
            proposed = self.plan(now, location, goal, centers, radii)
            if proposed:
                old_length = sum(np.linalg.norm(b-a) for a, b in zip([location]+self.points, self.points))
                new_length = sum(np.linalg.norm(b-a) for a, b in zip([location]+proposed, proposed))
                old_clear = bool(self.points) and all(segment_clearance(a-location, b-location, centers, radii) > 10.
                    for a, b in zip([location]+self.points, self.points))
                # Do not swap equivalent corridors every radar refresh because
                # a noisy measurement changed which side won by a few metres.
                if changed or not old_clear or new_length < .9*old_length:
                    self.points = proposed
            elif changed:
                self.points = []
            self.last_plan, self.goal = now, np.asarray(goal).copy()
        while (len(self.points) > 1 and np.linalg.norm(self.points[0]-location) < 65.
               and segment_clearance(np.zeros(3), self.points[1]-location, centers, radii) > 10.):
            self.points.pop(0)
        # Skip already-cleared bends only when the new segment remains clear.
        for index in range(len(self.points)-1, 0, -1):
            if segment_clearance(np.zeros(3), self.points[index]-location, centers, radii) > 12.:
                self.points = self.points[index:]
                break
        return self.points[0] if self.points else np.asarray(goal)


def travel_time(distance, speed, cruise):
    """Nominal arrival time; a forecast, continuously corrected by new sensors."""
    distance = np.maximum(np.asarray(distance, float), 0.)
    cruise = max(float(cruise), .5)
    speed = float(np.clip(speed, .1, cruise))
    accel = .11
    accelerating = max(0., (cruise*cruise-speed*speed)/(2.*accel))
    return np.where(distance <= accelerating,
                    (np.sqrt(speed*speed+2.*accel*distance)-speed)/accel,
                    (cruise-speed)/accel+(distance-accelerating)/cruise)


def path_lookahead(origin, points, distance, position=None):
    """Look through segment boundaries without stopping at each waypoint."""
    path = [np.asarray(origin)]+[np.asarray(point) for point in points]
    if len(path) < 2:
        return np.asarray(origin).copy()
    index, fraction = 0, 0.
    if position is not None:
        nearest = np.inf
        for k, (a, b) in enumerate(zip(path, path[1:])):
            delta = b-a
            along = float(np.clip((position-a)@delta/max(delta@delta, 1e-9), 0., 1.))
            gap = np.linalg.norm(position-a-along*delta)
            if gap < nearest:
                nearest, index, fraction = gap, k, along
    cursor = path[index]+fraction*(path[index+1]-path[index])
    for endpoint in path[index+1:]:
        length = np.linalg.norm(endpoint-cursor)
        if length >= distance:
            return cursor+(endpoint-cursor)*distance/max(length, 1e-9)
        distance -= length
        cursor = endpoint
    return path[-1].copy()


class RollingChunks:
    """Keep one committed route chunk and one predicted successor, never a full mission route."""
    def __init__(self):
        self.points, self.pending = [], []
        self.goal = None
        self.last_check = self.last_plan = -np.inf
        self.last_prefetch = -np.inf
        self.anchor = None
        self.planned_exit = np.zeros(3)
        self.active_exit = self.pending_exit = None
        self.schedule = np.empty(0)
        self.pending_schedule = np.empty(0)
        self.planned_times = np.empty(0)
        self.anchor_time = 0.
        self.chunk_id = self.replans = 0
        self.error = ''
        self.reason = 'initial'
        self.clearance = np.inf
        self.buffer_m = 0.
        self.speed_cap = CHUNK_CRUISE_SPEED
        self.lane_turn = 0.
        self.lane_density = 0.
        self.economy = False
        self.economy_context = None

    def invalidate(self, reason):
        self.points, self.pending = [], []
        self.last_plan = -np.inf
        self.last_prefetch = -np.inf
        self.active_exit = self.pending_exit = None
        self.schedule = self.pending_schedule = np.empty(0)
        self.reason = reason

    @staticmethod
    def length(location, points):
        return sum(np.linalg.norm(b-a) for a, b in zip([location]+list(points), points))

    @staticmethod
    def forecast(location, points, speed, cruise, centers, velocities, radii, times=None):
        """Swept, time-indexed moving-sphere checks along an entire chunk."""
        if not len(centers):
            return np.inf
        centers, velocities, radii = np.asarray(centers), np.asarray(velocities), np.asarray(radii)
        walked, gap, cursor = 0., np.inf, np.asarray(location)
        previous_time = 0.
        for index, endpoint in enumerate(points):
            delta = endpoint-cursor
            length = np.linalg.norm(delta)
            steps = max(1, int(np.ceil(length/20.)))
            for k in range(steps):
                a, b = cursor+k/steps*delta-location, cursor+(k+1)/steps*delta-location
                if times is None:
                    t0, t1 = travel_time(np.array([walked+k/steps*length, walked+(k+1)/steps*length]), speed, cruise)
                else:
                    t0 = previous_time+(times[index]-previous_time)*k/steps
                    t1 = previous_time+(times[index]-previous_time)*(k+1)/steps
                start, end = centers+t0*velocities-a, centers+t1*velocities-b
                change = end-start
                fraction = np.clip(-np.sum(start*change, axis=1)/np.maximum(np.sum(change*change, axis=1), 1e-9), 0., 1.)
                gaps = (np.linalg.norm(start+fraction[:, None]*change, axis=1)-radii-min(10., .12*t1)
                        -.11*(t1-t0)**2/8.)
                # Leaving an inflated initial margin is permitted only while
                # moving away. The independent local guard still checks it.
                escaping = ((np.linalg.norm(start, axis=1) < radii)
                            & (np.sum(start*change, axis=1) >= 0.))
                gaps[escaping] = np.inf
                gap = min(gap, float(gaps.min()))
            walked += length
            if times is not None:
                previous_time = float(times[index])
            cursor = np.asarray(endpoint)
        return gap

    def choose_lane(self, location, velocity, goal, centers, velocities, radii, cruise):
        """Compare broad future corridors, including space outside the direct line.

        This is a soft route preference. Beam and local swept checks remain the
        hard constraints; an attractive distant sector is not certified free.
        """
        offset = goal-location
        distance = np.linalg.norm(offset)
        self.lane_turn = self.lane_density = 0.
        if distance < 350. or not len(centers):
            return goal
        forward = offset/distance
        axis = np.eye(3)[np.argmin(np.abs(forward))]
        side = np.cross(forward, axis)
        side /= np.linalg.norm(side)
        up = np.cross(forward, side)
        headings = [forward]
        headings += [np.cos(tilt)*forward+np.sin(tilt)*(np.cos(angle)*side+np.sin(angle)*up)
                     for tilt in np.deg2rad([15., 30., 45.]) for angle in np.arange(8)*np.pi/4.]
        headings = np.asarray(headings)
        length = min(650., distance)
        samples = np.linspace(70., length, 12)
        times = travel_time(samples, np.linalg.norm(velocity), cruise)
        predicted = centers[:, None]+times[None, :, None]*velocities[:, None]
        relative = predicted[None]-headings[:, None, None]*samples[None, None, :, None]
        gaps = np.linalg.norm(relative, axis=3)-radii[None, :, None]-np.minimum(10., .12*times)[None, None]
        density = np.mean(np.sum((np.maximum(0., 75.-gaps)/75.)**2, axis=1), axis=1)
        endpoints = location+length*headings
        speed = np.linalg.norm(velocity)
        momentum = velocity/max(speed, 1e-9) if speed > 1. else forward
        score = (np.linalg.norm(goal-endpoints, axis=1)+(50. if self.economy else 220.)*density
                 +80.*max(speed, 2.)*(1.-headings@momentum))
        best = int(np.argmin(score))
        self.lane_turn = float(np.rad2deg(np.arccos(np.clip(headings[best]@forward, -1., 1.))))
        self.lane_density = float(density[best])
        return endpoints[best]

    def plan_chunk(self, now, location, velocity, goal, centers, velocities, radii, cruise):
        distance = np.linalg.norm(goal-location)
        if distance < 1.:
            self.planned_exit = np.asarray(velocity).copy()
            self.planned_times = np.array([1.])
            return [np.asarray(goal).copy()]
        heading = (goal-location)/distance
        speed = np.linalg.norm(velocity)
        axis = np.eye(3)[np.argmin(np.abs(heading))]
        side = np.cross(heading, axis)
        side /= np.linalg.norm(side)
        up = np.cross(heading, side)
        lane_goal = self.choose_lane(location, velocity, goal, centers, velocities, radii, cruise)
        horizon = float(np.clip(320./max(speed, 5.), 32., 60.))
        horizon = min(horizon, max(12., .75*distance/max(speed, 1.)))
        if len(centers):
            reach = speed*horizon+.5*.11*horizon*horizon
            relevant = np.linalg.norm(centers, axis=1) <= reach+horizon*np.linalg.norm(velocities, axis=1)+radii+12.
            centers, velocities, radii = centers[relevant], velocities[relevant], radii[relevant]
        # Beam search in position AND velocity. Each edge is integrated with a
        # finite acceleration; no instantaneous turns at geometric waypoints.
        branch = 13
        positions, motion = np.zeros((1, 3)), np.asarray(velocity)[None].copy()
        histories = np.empty((1, 0, 3))
        efforts, exposure, margins = np.zeros(1), np.zeros(1), np.full(1, np.inf)
        elapsed = 0.
        for stage in range(4):
            relative = lane_goal-location-positions
            bearings = relative/np.maximum(np.linalg.norm(relative, axis=1), 1e-9)[:, None]
            requested = min(cruise, np.sqrt(2.*SHIP_BRAKE_ACCEL*max(distance-10., 0.)))
            choices = [requested*bearings, motion.copy(), np.zeros_like(motion)]
            for tilt in (25., 55.):
                for offset in (side, -side, up, -up):
                    d = np.cos(np.deg2rad(tilt))*bearings+np.sin(np.deg2rad(tilt))*offset
                    choices.append(requested*d/np.maximum(np.linalg.norm(d, axis=1), 1e-9)[:, None])
            choices += [requested*motion/np.maximum(np.linalg.norm(motion, axis=1), 1e-9)[:, None],
                        .5*requested*bearings]
            commands = np.stack(choices, axis=1).reshape(-1, 3)
            positions = np.repeat(positions, branch, axis=0)
            motion = np.repeat(motion, branch, axis=0)
            histories = np.repeat(histories, branch, axis=0)
            efforts, margins = np.repeat(efforts, branch), np.repeat(margins, branch)
            exposure = np.repeat(exposure, branch)
            valid = np.ones(len(positions), dtype=bool)
            steps = 8
            dt = horizon/(4.*steps)
            for _ in range(steps):
                remaining = np.linalg.norm(goal-location-positions, axis=1)
                stop_cap = np.minimum(cruise, np.sqrt(2.*SHIP_BRAKE_ACCEL*np.maximum(remaining-5., 0.)))
                bounded = commands*np.minimum(1., stop_cap/np.maximum(np.linalg.norm(commands, axis=1), 1e-9))[:, None]
                acceleration = .5*(bounded-motion)
                acceleration *= np.minimum(1., .11/np.maximum(np.linalg.norm(acceleration, axis=1), 1e-9))[:, None]
                end = positions+dt*motion+.5*dt*dt*acceleration
                if len(centers):
                    start_gap = centers[None]+elapsed*velocities[None]-positions[:, None]
                    end_gap = centers[None]+(elapsed+dt)*velocities[None]-end[:, None]
                    change = end_gap-start_gap
                    fraction = np.clip(-np.sum(start_gap*change, axis=2)/np.maximum(np.sum(change*change, axis=2), 1e-9), 0., 1.)
                    gaps = (np.linalg.norm(start_gap+fraction[:, :, None]*change, axis=2)-radii[None]
                            -min(10., .12*(elapsed+dt))-.11*dt*dt/8.)
                    escaping = ((np.linalg.norm(start_gap, axis=2) < radii[None])
                                & (np.sum(start_gap*change, axis=2) >= 0.))
                    gaps[escaping] = np.inf
                    exposure += dt*np.sum((np.maximum(0., 50.-gaps)/50.)**2, axis=1)
                    margins = np.minimum(margins, gaps.min(axis=1))
                    valid &= gaps.min(axis=1) > 2.
                positions, motion = end, motion+dt*acceleration
                efforts += dt*np.linalg.norm(acceleration, axis=1)
                histories = np.concatenate((histories, positions[:, None]), axis=1)
                elapsed += dt
            # elapsed applies to the whole beam, not each candidate.
            error = lane_goal-location-positions
            ranges = np.linalg.norm(error, axis=1)
            toward = np.sum(motion*error, axis=1)/np.maximum(ranges, 1e-9)
            score = (ranges+.25*np.linalg.norm(goal-location-positions, axis=1)-8.*toward
                     +(8. if self.economy else 1.5)*efforts+(.5 if self.economy else 3.)*exposure
                     +2.*np.maximum(0., (12. if self.economy else 40.)-margins))
            if self.economy_context is not None:
                stamp, spent, damage, kg_per_dv = self.economy_context
                # Compare arrival time and impulse in score points. The capture
                # guide extends 500 m beyond closest approach, not the mission.
                final_offset = goal-location-positions
                final_range = np.linalg.norm(final_offset, axis=1)
                remaining = np.maximum(1., final_range-517.)
                end_speed = np.linalg.norm(motion, axis=1)
                final_motion = end_speed[:, None]*final_offset/np.maximum(final_range, 1e-9)[:, None]
                turn_dv = np.linalg.norm(final_motion-motion, axis=1)
                # A cheap sideways coast can hide a costly turn at the end of
                # this chunk. Price that turn before committing its prefix.
                arrival = stamp+elapsed+turn_dv/.11+remaining/np.maximum(end_speed, 1.)
                projected = 100.*(.5+120./np.maximum(arrival, .1)
                                  -(spent+kg_per_dv*(efforts+turn_dv))/75.-2.*damage/209200.)
                score = -projected+.015*exposure+.1*np.maximum(0., 4.-margins)
            score[~valid] = np.inf
            selected = []
            for candidate in np.argsort(score):
                if not np.isfinite(score[candidate]):
                    break
                # Keep genuinely different trajectories, not eight nearly
                # identical fast prefixes which all hit the same later hazard.
                if all(np.linalg.norm(positions[candidate]-positions[old]) > 4.
                       or np.linalg.norm(motion[candidate]-motion[old]) > .5 for old in selected):
                    selected.append(int(candidate))
                if len(selected) == 8:
                    break
            order = np.asarray(selected, dtype=int)
            if not len(order):
                break
            positions, motion, histories = positions[order], motion[order], histories[order]
            efforts, margins = efforts[order], margins[order]
            exposure = exposure[order]
            if stage == 3:
                # Retain enough curve samples for smooth pursuit. Validation
                # and later replanning still use actual, newly received sensors.
                proposed = [location+p for p in histories[0][3::4]]
                times = np.arange(4, 33, 4)*dt
                if self.forecast(location, proposed, speed, cruise, centers, velocities, radii, times) > 0.:
                    self.planned_exit = motion[0].copy()
                    self.planned_times = times
                    self.error = ''
                    return proposed
        self.error = 'no verified chunk'
        return []

    def waypoint(self, now, location, velocity, goal, centers, velocities, radii, cruise):
        changed = self.goal is not None and np.linalg.norm(goal-self.goal) > 30.
        if changed:
            self.invalidate('destination moved')
        self.goal = np.asarray(goal).copy()
        speed = np.linalg.norm(velocity)
        progress_time = self.anchor_time
        if self.points and self.anchor is not None:
            path = [self.anchor]+self.points
            nearest, index, along = np.inf, 0, 0.
            for k, (a, b) in enumerate(zip(path, path[1:])):
                delta = b-a
                fraction = np.clip((location-a)@delta/max(delta@delta, 1e-9), 0., 1.)
                gap = np.linalg.norm(location-a-fraction*delta)
                if gap < nearest:
                    nearest, index, along = gap, k, float(fraction)
            if len(self.schedule) == len(self.points):
                start_time = self.anchor_time if index == 0 else self.schedule[index-1]
                progress_time = start_time+along*(self.schedule[index]-start_time)
            if index:
                self.anchor = self.points[index-1].copy()
                self.points = self.points[index:]
                if len(self.schedule):
                    self.anchor_time = float(self.schedule[index-1])
                    self.schedule = self.schedule[index:]
            if nearest > 25.:
                self.invalidate('left predicted curve')
        # Prune only passed points. Lookahead blends the next leg before this
        # happens; proximity alone must not cut through a protected corner.
        while len(self.points) > 1 and np.linalg.norm(self.points[0]-location) < 2.:
            self.anchor = self.points.pop(0)
            if len(self.schedule):
                self.anchor_time = float(self.schedule[0])
                self.schedule = self.schedule[1:]
                progress_time = max(progress_time, self.anchor_time)
        times = np.maximum(self.schedule-progress_time, .1) if len(self.schedule) == len(self.points) else None
        if self.points and now-self.last_check >= 1.:
            self.clearance = self.forecast(location, self.points, speed, cruise, centers, velocities, radii, times)
            if self.clearance < 0.:
                self.invalidate('predicted crossing')
            elif self.pending and self.forecast(location, self.points+self.pending, speed, cruise, centers, velocities, radii,
                    np.r_[times, np.maximum(self.pending_schedule-progress_time, .1)] if times is not None and len(self.pending_schedule) else None) < 0.:
                self.pending = []
                self.pending_schedule = np.empty(0)
                self.reason = 'successor crossing'
            self.last_check = now
        if not self.points and now-self.last_plan >= 1.:
            self.points = self.plan_chunk(now, location, velocity, goal, centers, velocities, radii, cruise)
            self.last_plan = now
            self.replans += 1
            if self.points:
                self.anchor = location.copy()
                self.active_exit = self.planned_exit.copy()
                self.schedule = now+self.planned_times
                self.anchor_time = progress_time = now
                self.chunk_id += 1
        remaining = self.length(location, self.points)
        stopping = speed*speed/(2.*SHIP_BRAKE_ACCEL)
        buffered = self.length(location, self.points+self.pending)
        need_successor = (not self.pending and remaining < max(350., stopping+250.))
        need_reserve = bool(self.pending) and len(self.pending) < 16 and buffered < stopping+350.
        endpoint = (self.pending or self.points)[-1] if self.points else location
        if (self.points and (need_successor or need_reserve) and now-self.last_prefetch >= 2.
                and np.linalg.norm(goal-endpoint) > 25.):
            eta = float(travel_time(remaining, speed, cruise))
            queued_schedule = self.pending_schedule if len(self.pending_schedule) else self.schedule
            if len(queued_schedule):
                eta = max(.1, float(queued_schedule[-1]-progress_time))
            next_speed = min(cruise, np.sqrt(speed*speed+2.*.11*remaining))
            next_heading = goal-endpoint
            next_velocity = next_speed*next_heading/max(np.linalg.norm(next_heading), 1e-9)
            exit_velocity = self.pending_exit if self.pending else self.active_exit
            if exit_velocity is not None:
                next_velocity = exit_velocity.copy()
            shifted = centers+eta*velocities-(endpoint-location)
            successor = self.plan_chunk(now+eta, endpoint, next_velocity, goal, shifted, velocities, radii, cruise)
            if successor:
                self.pending += successor
                self.pending_exit = self.planned_exit.copy()
                start_time = queued_schedule[-1] if len(queued_schedule) else now+eta
                self.pending_schedule = np.r_[self.pending_schedule, start_time+self.planned_times]
            self.last_prefetch = now
        lookahead = float(np.clip(40.+8.*speed, 45., 130.))
        if self.pending and remaining < max(220., lookahead*2.):
            proposed = self.points+self.pending
            combined_times = np.r_[self.schedule, self.pending_schedule]-progress_time
            if self.forecast(location, proposed, speed, cruise, centers, velocities, radii,
                             np.maximum(combined_times, .1) if len(combined_times) == len(proposed) else None) > 0.:
                self.points = proposed
                self.schedule = np.r_[self.schedule, self.pending_schedule]
                self.active_exit = self.pending_exit
                self.chunk_id += 1
                self.reason = 'successor ready'
            self.pending = []
            self.pending_schedule = np.empty(0)
        self.buffer_m = self.length(location, self.points+self.pending)
        self.speed_cap = min(cruise, np.sqrt(2.*SHIP_BRAKE_ACCEL*max(self.buffer_m-30., 0.))) if self.points else 2.
        if not self.points:
            return location+min(80., np.linalg.norm(goal-location))*(goal-location)/max(np.linalg.norm(goal-location), 1e-9)
        return path_lookahead(location, self.points, lookahead)


class GuardedLaunch:
    def __init__(self, enabled=True):
        self.enabled, self.done = enabled, not enabled
        self.origin = self.up = None
        self.height = 0.
        self.required_height = LAUNCH_HEIGHT

    def goal(self, location, destination):
        if self.done or np.linalg.norm(destination-location) < 250.:
            self.done = True
            return None
        if self.origin is None:
            self.origin = location.copy()
            self.up = location/max(np.linalg.norm(location), 1e-9)
            if np.linalg.norm(self.up) < .5:
                self.up = np.array([0., 0., 1.])
        self.height = float((location-self.origin)@self.up)
        if self.height >= self.required_height:
            self.done = True
            return None
        lift = 60. if getattr(self, 'speed', .9) > 1. else max(1., LAUNCH_HEIGHT+1.-self.height)
        return location+lift*self.up


class TerminalApproach:
    """Brake outside the ship, then enter the scoring sphere slowly on a clear side."""
    def __init__(self):
        self.side = None
        self.phase = 'CRUISE'
        self.inner_goal = None
        self.survey_goal = None
        self.last_choice = -np.inf

    def goal(self, now, location, velocity, target, target_velocity, centers, radii):
        relative = location-target
        distance = np.linalg.norm(relative)
        if self.side is None:
            self.side = relative/max(distance, 1e-9)
        speed = np.linalg.norm(velocity-target_velocity)
        outward = relative/max(distance, 1e-9)
        closing = max(0., -(velocity-target_velocity)@outward)
        brake_start = max(300., closing*closing/(2.*SHIP_BRAKE_ACCEL)+130.)
        if self.phase == 'CRUISE' and distance < brake_start:
            self.phase = 'BRAKE'
        if self.phase == 'CRUISE':
            return target+100.*self.side, 'CRUISE', MAX_SPEED
        if self.phase == 'BRAKE':
            # Reaching a standoff is compulsory; no ESCAPE candidate may bypass
            # terminal braking and charge through a camera-observed ship surface.
            if distance < 115. and speed < .7:
                self.phase = 'SURVEY'
            else:
                cap = min(8., np.sqrt(2.*SHIP_BRAKE_ACCEL*max(distance-100., 0.)), .15*max(distance-100., 0.))
                return target+100.*self.side, 'BRAKE_SHIP', cap
        if self.inner_goal is not None:
            blocked = segment_clearance(np.zeros(3), self.inner_goal-location, centers, radii) < 0.
            if blocked and distance > 55.:
                self.inner_goal = None
                self.phase = 'SURVEY'
        if self.inner_goal is None and now-self.last_choice >= 2.:
            self.last_choice = now
            # Include the current side and the sphere directions. Known camera
            # geometry is a hard constraint; unknown hull is approached at crawl.
            k = np.arange(72)
            z = 1.-2.*(k+.5)/len(k)
            angle = k*np.pi*(3.-np.sqrt(5.))
            sphere = np.column_stack((np.sqrt(1.-z*z)*np.cos(angle), np.sqrt(1.-z*z)*np.sin(angle), z))
            candidates = [target+HOLD_RADIUS*outward]+[target+HOLD_RADIUS*d for d in sphere]
            available = [point for point in candidates
                         if segment_clearance(np.zeros(3), point-location, centers, radii) > 2.]
            if available:
                self.inner_goal = min(available, key=lambda point: np.linalg.norm(point-location))
                self.phase = 'FINAL'
            else:
                # Orbit the standoff to observe another side, rather than enter
                # a goal which lies inside observed station collision geometry.
                axis = np.eye(3)[np.argmin(np.abs(outward))]
                tangent = np.cross(outward, axis)
                tangent /= np.linalg.norm(tangent)
                options = [target+100.*(np.cos(.55)*outward+sign*np.sin(.55)*tangent)
                           for sign in (1., -1.)]
                self.survey_goal = max(options, key=lambda point:
                    segment_clearance(np.zeros(3), point-location, centers, radii))
        if self.inner_goal is None:
            return (self.survey_goal if self.survey_goal is not None else target+100.*outward), 'SURVEY_SHIP', 1.5
        error = np.linalg.norm(self.inner_goal-location)
        phase = 'HOLD' if error < .6 and speed < .15 else 'FINAL_CREEP'
        # The whole 50 m approach need not be a crawl. Brake along the verified
        # clear segment; only the last few metres use sub-metre/second speeds.
        return self.inner_goal, phase, min(2., np.sqrt(.08*max(error-.5, 0.)), .25*error)


class CaptureApproach:
    """Enter a clear side of the scoring sphere; brake after entry, not before."""
    def __init__(self):
        self.side = self.tangent = None
        self.inner_goal = None
        self.camera_ready = False
        self.fallback = None
        self.entered = False
        self.exit_goal = None
        self.eta = 250.
        self.no_pass_since = None
        self.best_gap = -np.inf

    def enter(self, location, velocity, target_velocity):
        self.entered = True
        self.fallback = None
        motion = np.asarray(velocity)-target_velocity
        speed = np.linalg.norm(motion)
        heading = motion/max(speed, 1e-9)
        if speed < .01:
            heading = self.tangent if self.tangent is not None else np.array([1., 0., 0.])
        self.exit_goal = location+heading*(speed*speed/(2.*SHIP_BRAKE_ACCEL)+15.)

    def goal(self, now, location, velocity, target, target_velocity, centers, radii):
        relative = location-target
        distance = np.linalg.norm(relative)
        motion = velocity-target_velocity
        outward = relative/max(distance, 1e-9)
        if self.side is None:
            self.side = outward.copy()
            axis = np.eye(3)[np.argmin(np.abs(outward))]
            self.tangent = np.cross(outward, axis)
            self.tangent /= np.linalg.norm(self.tangent)
        if self.fallback is not None:
            return self.fallback.goal(now, location, velocity, target, target_velocity, centers, radii)
        if distance <= 50. and not self.entered:
            self.enter(location, velocity, target_velocity)
        if self.entered:
            error = np.linalg.norm(self.exit_goal-location)
            cap = min(6., np.sqrt(2.*SHIP_BRAKE_ACCEL*max(error-.5, 0.)), .3*error)
            return self.exit_goal, 'POST_ENTRY_BRAKE' if error > 1. else 'HOLD', cap
        tangent = self.tangent-outward*(outward@self.tangent)
        tangent /= max(np.linalg.norm(tangent), 1e-9)
        second = np.cross(outward, tangent)
        # A line tangent to a 47 m circle gives several seconds inside the
        # 50 m scoring sphere. Candidate azimuths are verified against sensors.
        ratio = min(.999, 47./max(distance, 47.01))
        candidates = []
        self.best_gap = -np.inf
        for angle in np.arange(16)*np.pi/8.:
            radial = np.cos(angle)*tangent+np.sin(angle)*second
            heading = -np.sqrt(1.-ratio*ratio)*outward+ratio*radial
            along = distance*np.sqrt(1.-ratio*ratio)
            entry = location+along*heading
            end = entry+500.*heading
            clearance = segment_clearance(np.zeros(3), entry+100.*heading-location, centers, radii)
            self.best_gap = max(self.best_gap, clearance)
            if clearance > 1.25 or distance > 260.:
                candidates.append((np.linalg.norm(radial-tangent), radial, entry, end))
        if candidates:
            self.no_pass_since = None
            _, radial, self.inner_goal, goal = min(candidates, key=lambda row: row[0])
            if self.camera_ready or distance > 220.:
                self.tangent = radial
        else:
            if self.no_pass_since is None:
                self.no_pass_since = now
            goal = location+100.*(-outward)
            if distance < 135. and np.linalg.norm(motion) < 1. and now-self.no_pass_since >= 5.:
                self.fallback = TerminalApproach()
                self.fallback.side = self.side.copy()
                return self.fallback.goal(now, location, velocity, target, target_velocity, centers, radii)
            if distance < 260.:
                return target+125.*outward, 'OBSERVE_CAPTURE', min(3., np.sqrt(2.*SHIP_BRAKE_ACCEL*max(distance-125., 0.)))
        # Missing station camera coverage must not brake a clear capture pass.
        # Keep the existing obstacle checks and distant approach speed limit.
        cap = 10. if self.camera_ready or distance < 210. else min(10., np.sqrt(2.*SHIP_BRAKE_ACCEL*max(distance-80., 0.)))
        return goal, 'CAPTURE_PASS' if distance < 260. else 'CRUISE', cap


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
        forecast_rates = []
        core_radii, report_ages, track_confidence = [], [], []
        gravity_tracking = getattr(self, 'gravity_tracking', False)
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
        old_gravity = -711000.*old_positions/np.maximum(np.linalg.norm(old_positions, axis=1)**3, 1.)[:, None]
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
                if gravity_tracking:
                    residuals = np.linalg.norm(absolute-old_positions-dt[:, None]*old_speeds
                                              -.5*dt[:, None]**2*old_gravity, axis=1)
                eligible = (available & (dt >= 0.4) & (dt <= 8.0)
                            & (np.abs(report[1, 2]-old_diameters)
                               <= np.maximum(3.0, 0.25*np.abs(old_diameters)))
                            & (residuals < 8.0+2.0*dt))
                for index in np.flatnonzero(eligible):
                    matches.append((residuals[index], old_tracks[index]))
                matches.sort(key=lambda m: m[0])
                speed = radial_speed*direction
                samples = [(stamp, absolute.copy())]
                identity = self.track_counter
                self.track_counter += 1
                # Ambiguous associations retain the radial-only estimate. A false
                # track is worse than explicitly admitting missing transverse speed.
                if matches and (len(matches) == 1 or matches[1][0] > matches[0][0]+4.0):
                    old = matches[0][1]
                    dt = stamp-old['stamp']
                    samples = [(t, p) for t, p in old.get('samples', [(old['stamp'], old['pos'])])
                               if stamp-t <= 30.]+[(stamp, absolute.copy())]
                    samples = samples[-8:]
                    sample_times = np.array([t for t, _ in samples])
                    sample_positions = np.array([p for _, p in samples])
                    centered_times = sample_times-sample_times.mean()
                    acceleration = -711000.*absolute/max(np.linalg.norm(absolute)**3, 1.)
                    if gravity_tracking:
                        sample_positions -= .5*(sample_times-stamp)[:, None]**2*acceleration
                    # Fit velocity over several radar sweeps. Differencing only
                    # two noisy positions made distant chunk predictions jump.
                    measured_speed = (centered_times[:, None]*sample_positions).sum(axis=0)/max(centered_times@centered_times, 1e-9)
                    gain = .75 if sample_times[-1]-sample_times[0] >= 6. else .3
                    prior_velocity = old['vel']+dt*acceleration if gravity_tracking else old['vel']
                    filtered = (1.-gain)*prior_velocity+gain*limit_vector(measured_speed, 4.)
                    speed = filtered + (radial_speed-filtered.dot(direction))*direction
                    identity = old['id']
                    available[old_ids == identity] = False
                track = dict(id=identity, stamp=stamp, pos=absolute,
                             vel=speed, diameter=report[1, 2], samples=samples)
            used.add(track['id'])
            new_cache[key] = track
            speed = track['vel']
            center = track['pos'] - loc + speed*age
            rate = .6
            confident = False
            if gravity_tracking:
                acceleration = -711000.*track['pos']/max(np.linalg.norm(track['pos'])**3, 1.)
                center += .5*age*age*acceleration
                speed = speed+age*acceleration
                samples = track.get('samples', [])
                if len(samples) >= 4:
                    stamps = np.array([t for t, _ in samples])
                    if stamps[-1]-stamps[0] >= 8.:
                        positions_fit = np.array([p for _, p in samples])-.5*(stamps-stamps[-1])[:, None]**2*acceleration
                        tc = stamps-stamps.mean()
                        slope = (tc[:, None]*positions_fit).sum(axis=0)/max(tc@tc, 1e-9)
                        residual = positions_fit-positions_fit.mean(axis=0)-tc[:, None]*slope
                        rms = np.sqrt(np.mean(np.sum(residual*residual, axis=1)))
                        rate = float(np.clip(2.2*max(2., rms)/max(np.sqrt(tc@tc), 1.)+.1, .25, .6))
                        confident = rms < 5. and rate < .55
            # Missing pre-start history and clock skew enlarge uncertainty.
            missing_history = max(0.0, times[0]-stamp)
            radius = (0.575 * max(0.0, report[1, 2]) + 3.0 + 8.0
                      + (rate if gravity_tracking else 1.)*age + np.linalg.norm(old_vel)*missing_history)
            centers.append(center)
            speeds.append(speed)
            radii.append(radius)
            forecast_rates.append(rate)
            core_radii.append(.575*max(0., report[1, 2])+3.+2.)
            report_ages.append(age)
            track_confidence.append(confident)
        self.report_cache = new_cache
        self.radar_forecast_rates = np.asarray(forecast_rates)
        self.radar_core_radii = np.asarray(core_radii)
        self.radar_report_ages = np.asarray(report_ages)
        self.radar_track_confidence = np.asarray(track_confidence, dtype=bool)
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
        stop_accel = min(predictive_accel, BRAKE_ACCEL)
        horizon = int(np.clip(np.ceil(np.linalg.norm(vel-target_vel)/stop_accel)+8, HORIZON, MAX_LOOKAHEAD))
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
        elapsed = 0.
        while elapsed < horizon:
            # Coarser far-future steps keep long high-speed braking forecasts
            # affordable. Swept checks below include curvature error bounds.
            dt = min(1. if elapsed < 40. else 2., horizon-elapsed)
            accel = VELOCITY_GAIN*(candidates-trial_vel)
            # Roll out the goal candidate with the same braking guidance used
            # during real flight, so it does not predict flying through arrival.
            remaining = error+elapsed*np.asarray(target_vel)-trial_pos[0]
            length = np.linalg.norm(remaining)
            guided_speed = min(cruise, np.sqrt(2*BRAKE_ACCEL*length), 0.4*length)
            guided_velocity = np.asarray(target_vel)+guided_speed*remaining/max(length, 1e-9)
            accel[0] = VELOCITY_GAIN*(guided_velocity-trial_vel[0])
            accel *= np.minimum(1.0, predictive_accel/np.maximum(np.linalg.norm(accel, axis=1), 1e-12))[:, None]
            delta_v += dt*np.linalg.norm(accel, axis=1)
            next_pos = trial_pos + dt*trial_vel + 0.5*dt*dt*accel
            if len(centers):
                start = centers[None, :, :] + elapsed*speeds[None, :, :] - trial_pos[:, None, :]
                end = centers[None, :, :] + (elapsed+dt)*speeds[None, :, :] - next_pos[:, None, :]
                delta = end-start
                fraction = np.clip(-np.sum(start*delta, axis=2)/np.maximum(np.sum(delta*delta, axis=2), 1e-12), 0, 1)
                gap = (np.linalg.norm(start+fraction[:, :, None]*delta, axis=2)-radii[None, :]
                       -uncertainty_rates[None, :]*(elapsed+dt)-predictive_accel*dt*dt/8.)
                clearance = np.minimum(clearance, gap.min(axis=1))
                # Accumulated penetration distinguishes escape directions even
                # when all candidates share the same initially negative clearance.
                risk += dt*np.maximum(0.0, -gap).sum(axis=1)/(1.0+0.1*elapsed)
            trial_pos = next_pos
            trial_vel += dt*accel
            elapsed += dt
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


class FinalNavigator(QualificationPlanner):
    """Guarded launch, rolling predicted chunks, local feedback and ship guard.

    Only the base class's radar tracker is reused. Its long inflated velocity
    forecasts are retained as historical code but are not the active planner.
    """
    def __init__(self, launch=True):
        super().__init__()
        self.corridor = RollingChunks()
        self.launch = GuardedLaunch(launch)
        self.terminal = TerminalApproach()
        self.stopping_distance = 0.
        self.last_phase = 'START'
        self.last_damage = 0.
        self.arrival_time = None
        self.hold_offset = None
        self.flythrough = False
        self.coast_clearance = -np.inf
        self.collision_urgent = False

    def observe_damage(self, damage):
        if np.isfinite(damage) and damage > self.last_damage+.1:
            self.corridor.invalidate('damage feedback')
        self.last_damage = max(self.last_damage, float(damage))

    def observe_scene(self, now, loc, vel, target_loc, target_vel, radar, camera):
        if (getattr(self, 'radar_cruise_only', False) and self.launch.done and len(radar)
                and np.linalg.norm(target_loc-loc) > 300.
                and vel@(loc/max(np.linalg.norm(loc), 1e-9)) > 0.):
            # Radar covers every debris report. Keep vision for launch terrain
            # and terminal hull checks, without duplicate distant depth bounds.
            camera = None
        radar_centers, radar_speeds, radar_radii = self.obstacles(now, loc, vel, radar)
        centers, speeds, radii = radar_centers, radar_speeds, radar_radii
        forecast_rates = (self.radar_forecast_rates.copy() if getattr(self, 'gravity_tracking', False)
                          else np.full(len(centers), .6))
        forecast_accels = (-711000.*(centers+loc)/np.maximum(np.linalg.norm(centers+loc, axis=1)**3, 1.)[:, None]
                           if getattr(self, 'gravity_tracking', False) else np.zeros_like(centers))
        if camera is not None:
            cc, cv, cr = camera.obstacles(now, loc, centers, speeds, radii)
            if getattr(self, 'risk_planning', False) and len(cc) and len(centers):
                # RGB bearing associations copy an existing radar sphere; they
                # are correlated evidence, not another independently ranged hull.
                duplicate = ((np.max(np.abs(cc[:, None]-centers[None]), axis=2) < 1e-8)
                             & (np.max(np.abs(cv[:, None]-speeds[None]), axis=2) < 1e-8)
                             & (np.abs(cr[:, None]-radii[None]-1.) < 1e-8)).any(axis=1)
                cc, cv, cr = cc[~duplicate], cv[~duplicate], cr[~duplicate]
            # Fresh stationary surfaces around the known stationary rescue ship
            # already include pixel/pose/age bounds. Do not inflate them as if
            # they were untracked moving debris. All physical volumes remain.
            ship_surface = (np.linalg.norm(cc+loc-target_loc, axis=1) < 85.) & (np.linalg.norm(cv-target_vel, axis=1) < .3)
            camera_rates = np.where(ship_surface & self.flythrough, .15, .6)
            if getattr(self, 'matched_camera_prediction', False) and len(cc) and len(centers):
                # A depth bound assigned a radar velocity shares that track's
                # prediction uncertainty. Retain its entire occupied volume.
                same_velocity = np.max(np.abs(cv[:, None]-speeds[None]), axis=2) < 1e-8
                separation = np.linalg.norm(cc[:, None]-centers[None], axis=2)
                matches = same_velocity & (separation < cr[:, None]+radii[None])
                associated = matches.any(axis=1) & ~ship_surface
                indices = np.argmin(np.where(matches, separation, np.inf), axis=1)
                camera_rates[associated] = forecast_rates[indices[associated]]
            if self.flythrough and self.launch.origin is not None:
                # Depth surfaces below the original launch plane stay behind an
                # ascending vehicle. Keep their full occupied volumes, without
                # growing them like untracked debris above the flight path.
                below_launch = ((cc+loc-self.launch.origin)@self.launch.up < -1.) & (np.linalg.norm(cv, axis=1) < .1)
                camera_rates[below_launch] = .1
            camera_accels = -711000.*(cc+loc)/np.maximum(np.linalg.norm(cc+loc, axis=1)**3, 1.)[:, None]
            camera_accels[(np.linalg.norm(cv, axis=1) < .1) | ship_surface] = 0.
            forecast_accels = np.vstack((forecast_accels, camera_accels if getattr(self, 'gravity_tracking', False) else np.zeros_like(cc)))
            forecast_rates = np.r_[forecast_rates, camera_rates]
            centers, speeds = np.vstack((centers, cc)), np.vstack((speeds, cv))
            radii = np.r_[radii, cr]
        station_observed = camera is not None and camera.observes(now, target_loc)
        self.latest_scene = centers, speeds, radii
        self.latest_accels = forecast_accels
        self.latest_radar_count = len(radar_centers)
        return centers, speeds, radii, forecast_rates, forecast_accels, station_observed

    def command(self, now, loc, vel, target_loc, target_vel, radar,
                camera=None, cruise_speed=None, maneuver_accel=None, fuel_weight=1.5):
        loc, vel = np.asarray(loc, float), np.asarray(vel, float)
        target_loc, target_vel = np.asarray(target_loc, float), np.asarray(target_vel, float)
        centers, speeds, radii, forecast_rates, forecast_accels, station_observed = self.observe_scene(
            now, loc, vel, target_loc, target_vel, radar, camera)
        goal, phase, phase_cap = self.terminal.goal(now, loc, vel, target_loc, target_vel, centers, radii)
        distance = np.linalg.norm(target_loc-loc)
        relative_velocity = vel-target_vel
        self.stopping_distance = float(np.linalg.norm(relative_velocity)**2/(2.*SHIP_BRAKE_ACCEL))
        if camera is not None and phase in ('FINAL_CREEP', 'SURVEY_SHIP'):
            depth_ready = station_observed
            if not depth_ready:
                goal, phase, phase_cap = target_loc+100.*self.terminal.side, 'WAIT_SHIP_CAMERA', 0.
        launch_goal = self.launch.goal(loc, target_loc)
        if launch_goal is not None:
            goal, phase, phase_cap = launch_goal, 'LAUNCH', getattr(self.launch, 'speed', .9)
        if self.arrival_time is None and ((distance <= 50. and self.flythrough)
                                        or (distance < 49.5 and np.linalg.norm(relative_velocity) < 1.)):
            self.arrival_time = float(now)
            hold = self.terminal.inner_goal if self.terminal.inner_goal is not None else loc
            self.hold_offset = np.asarray(hold).copy()-target_loc
        if self.arrival_time is not None and not self.flythrough:
            goal = target_loc+self.hold_offset
            phase, phase_cap = 'HOLD', min(.5, .25*np.linalg.norm(goal-loc))
        self.last_phase = phase
        cruise = CHUNK_CRUISE_SPEED if cruise_speed is None else float(np.clip(cruise_speed, 0., MAX_SPEED))
        use_chunks = phase == 'CRUISE' or (phase == 'BRAKE_SHIP' and distance > 115.)
        if use_chunks:
            waypoint = self.corridor.waypoint(now, loc, vel, goal, centers, speeds, radii, min(cruise, phase_cap))
            if (phase == 'BRAKE_SHIP' and self.corridor.points
                    and self.corridor.length(loc, self.corridor.points) < 35.
                    and np.linalg.norm(goal-self.corridor.points[-1]) < 35.):
                # A short final curve can end just outside the standoff. Hand
                # the remaining metres to terminal guidance instead of waiting
                # forever for a successor which is intentionally not generated.
                waypoint, use_chunks = goal, False
        else:
            waypoint = goal
        route = waypoint-loc
        length = np.linalg.norm(route)
        direction = route/max(length, 1e-9)
        cap = min(cruise, phase_cap)
        if phase == 'CRUISE':
            cap = min(cap, self.corridor.speed_cap)
            if len(centers) and np.any(np.linalg.norm(centers, axis=1) < radii):
                cap = min(cruise, max(cap, 2.))
        if phase != 'CRUISE':
            cap = min(cap, np.sqrt(2.*SHIP_BRAKE_ACCEL*length), .35*length)
        elif np.linalg.norm(relative_velocity) > 2. and length > 1.:
            cosine = np.clip((relative_velocity@direction)/np.linalg.norm(relative_velocity), -1., 1.)
            sine = np.sqrt(max(0., 1.-cosine*cosine))
            if sine > .1:
                cap = min(cap, np.sqrt(2.*.12*max(length, 80.)/sine))
        desired = target_vel+cap*direction
        previous = target_vel+limit_vector(self.previous_velocity-target_vel, cap)
        coast = target_vel+limit_vector(relative_velocity, cap)
        candidates = [desired, previous, coast, target_vel]
        if (getattr(self, 'fine_avoidance', False) and self.flythrough and phase == 'CRUISE'
                and np.linalg.norm(relative_velocity) > 3.):
            # Preserve the measured momentum and offer a small, servo-sized
            # sidestep before paying for a large change of velocity direction.
            motion_speed = np.linalg.norm(relative_velocity)
            motion_heading = relative_velocity/motion_speed
            reference = np.eye(3)[np.argmin(np.abs(motion_heading))]
            motion_side = np.cross(motion_heading, reference)
            motion_side /= np.linalg.norm(motion_side)
            motion_up = np.cross(motion_heading, motion_side)
            for impulse in (.4, .65, 1.):
                for angle in np.arange(8)*np.pi/4.:
                    trial = relative_velocity+impulse*(np.cos(angle)*motion_side+np.sin(angle)*motion_up)
                    candidates.append(target_vel+motion_speed*trial/np.linalg.norm(trial))
        if length > 1e-6:
            axis = np.eye(3)[np.argmin(np.abs(direction))]
            side = np.cross(direction, axis)
            side /= np.linalg.norm(side)
            vertical = np.cross(direction, side)
            for tilt in (12., 25., 45.):
                for angle in np.arange(8)*np.pi/4.:
                    heading = (np.cos(np.deg2rad(tilt))*direction
                               +np.sin(np.deg2rad(tilt))*(np.cos(angle)*side+np.sin(angle)*vertical))
                    candidates.append(target_vel+cap*heading)
        candidates.extend(target_vel+cap*self.directions)
        candidates = np.asarray(candidates)
        if phase == 'LAUNCH':
            # A blocked takeoff may slide sideways or wait, but must not choose
            # an escape thrust back down into the starting terrain.
            candidates = candidates[((candidates-target_vel)@self.launch.up) >= -1e-9]
        horizon = 15 if phase == 'CRUISE' else 6
        if phase == 'CRUISE' and len(centers):
            ranges = np.linalg.norm(centers, axis=1)
            closing = np.sum((vel-speeds)*centers, axis=1)/np.maximum(ranges, 1e-9)
            contact_times = (ranges-radii)/np.maximum(closing, 1e-9)
            incoming = contact_times[(closing > 1.) & (contact_times > horizon) & (contact_times < 27.)]
            if len(incoming):
                horizon = min(30, int(np.ceil(incoming.min()))+3)
        self.last_horizon = horizon
        predictive_accel = (.085 if self.flythrough else .14) if maneuver_accel is None else float(np.clip(maneuver_accel, .08, .25))
        local_risk = bool(getattr(self, 'local_expected_risk', False) and self.flythrough and phase == 'CRUISE')
        if local_risk:
            nr = self.latest_radar_count
            ranged = np.arange(len(centers)) < nr
            cores = np.r_[self.radar_core_radii, radii[nr:]]
            ages = np.r_[self.radar_report_ages, np.zeros(len(centers)-nr)]
            confident = np.r_[self.radar_track_confidence, np.zeros(len(centers)-nr, dtype=bool)]
        if len(centers):
            # Keep all 500 reports in mapping/tracking, but roll out only volumes
            # physically reachable within this forecast (including incoming ones).
            reach = np.linalg.norm(vel)*horizon+.5*predictive_accel*horizon*horizon
            relevant = np.linalg.norm(centers, axis=1) <= (reach+horizon*np.linalg.norm(speeds, axis=1)
                                                        +radii+.6*horizon+1.)
            centers, speeds, radii = centers[relevant], speeds[relevant], radii[relevant]
            forecast_rates = forecast_rates[relevant]
            forecast_accels = forecast_accels[relevant]
            if local_risk:
                ranged, cores, ages, confident = (a[relevant] for a in (ranged, cores, ages, confident))
        positions = np.zeros((len(candidates), 3))
        velocities = np.tile(vel, (len(candidates), 1))
        clearance, risk, effort = np.full(len(candidates), np.inf), np.zeros(len(candidates)), np.zeros(len(candidates))
        local_economy = self.flythrough and phase == 'CRUISE'
        if local_risk:
            damage_risk = np.zeros((len(candidates), len(centers)))
            fatal_risk, recovery_risk = np.zeros_like(damage_risk), np.zeros_like(damage_risk)
        gravity = self.gravity_at(loc) if local_economy else np.zeros(3)
        for step in range(horizon):
            accel = VELOCITY_GAIN*(candidates-velocities)
            if use_chunks and self.corridor.points:
                trial_location = loc+positions[0]
                aim = path_lookahead(loc, self.corridor.points, max(45., 8.*np.linalg.norm(velocities[0])), trial_location)
                bearing = aim-trial_location
                guided = target_vel+cap*bearing/max(np.linalg.norm(bearing), 1e-9)
                accel[0] = VELOCITY_GAIN*(guided-velocities[0])
            accel *= np.minimum(1., predictive_accel/np.maximum(np.linalg.norm(accel, axis=1), 1e-9))[:, None]
            if local_economy:
                accel[2] = 0.  # A real unpowered coast, not a constant-speed servo.
            effort += np.linalg.norm(accel, axis=1)
            accel += gravity
            end_position = positions+velocities+.5*accel
            if len(centers):
                start = centers[None]+step*speeds[None]+.5*step*step*forecast_accels[None]-positions[:, None]
                end = centers[None]+(step+1)*speeds[None]+.5*(step+1)**2*forecast_accels[None]-end_position[:, None]
                delta = end-start
                fraction = np.clip(-np.sum(start*delta, axis=2)/np.maximum(np.sum(delta*delta, axis=2), 1e-9), 0., 1.)
                separation = np.linalg.norm(start+fraction[:, :, None]*delta, axis=2)
                gap = separation-radii[None]-forecast_rates[None]*(step+1)-.03
                if local_risk:
                    sigma = np.sqrt(4.+((ages+step+1)*forecast_rates/np.where(confident, 2.2, 1.2))**2)
                    relative_speed = np.linalg.norm(delta, axis=2)+.125
                    p, energy = gaussian_encounter_risk(separation-cores[None]-.03, sigma[None], relative_speed)
                    p[:, ~ranged] = 0.
                    damage_risk = np.maximum(damage_risk, p*energy)
                    fatal_risk = np.maximum(fatal_risk, np.where(energy > 209200.-self.last_damage, p, 0.))
                    recovery_risk = np.maximum(recovery_risk, p*(5.+relative_speed/.1))
                    # The same physical core and probabilistic uncertainty as
                    # finite shots, rather than a second, larger hard envelope.
                    gap[:, ranged] = separation[:, ranged]-np.maximum(cores[ranged]-1., 3.)[None]-.03
                if self.flythrough and phase == 'LAUNCH':
                    # Near the surface the vehicle may begin inside an inflated
                    # depth volume. A strictly outward segment may leave that
                    # margin, but cannot move deeper into it or enter another one.
                    initial_overlap = np.linalg.norm(centers, axis=1) < radii
                    departing = (initial_overlap[None] & (np.sum(start*delta, axis=2) > 1e-8))
                    gap[departing] = np.inf
                clearance = np.minimum(clearance, gap.min(axis=1))
                risk += np.maximum(-gap, 0.).sum(axis=1)/(1.+.1*step)
            positions, velocities = end_position, velocities+accel
        progress_goal = self.corridor.points[-1]-loc if use_chunks and self.corridor.points else route
        hint_distance = np.linalg.norm(positions-progress_goal, axis=1)
        cost = hint_distance+.6*np.linalg.norm(candidates-self.previous_velocity, axis=1)+.3*effort
        if local_economy:
            predicted_scores = arrival_forecast(now+horizon, self.last_fuel, self.last_damage,
                loc+positions, velocities-target_vel, goal, effort, self.policy.fuel_per_dv,
                self.policy.acceleration, max(0., -gravity@direction))
            if np.isfinite(predicted_scores).any():
                cost = (-predicted_scores+.005*(hint_distance-hint_distance.min())
                        +.1*np.maximum(0., 3.-clearance))
        safe = clearance > 0.
        if local_risk:
            expected = damage_risk.sum(axis=1)
            recovery = recovery_risk.sum(axis=1)
            eta = max(now+horizon, 1.)+self.policy.remaining_time
            cost += 200.*expected/209200.+12000.*(1./eta-1./(eta+recovery))
            safe &= ((fatal_risk.sum(axis=1) < .01)
                     & (expected < .2*max(0., 209200.-self.last_damage)))
        self.coast_clearance = float(clearance[2]) if phase != 'LAUNCH' else -np.inf
        if local_risk and not safe[2]:
            self.coast_clearance = -np.inf
        if safe.any():
            cost[~safe] = np.inf
            # Retain the committed chunk when safe; use local alternatives for
            # hazards, not tiny progress differences that cause steering churn.
            committed = (not local_economy and (use_chunks or phase == 'CAPTURE_PASS' or
                                               (self.flythrough and phase == 'LAUNCH')))
            best = 0 if committed and safe[0] else int(np.argmin(cost))
            mode = ('GUIDE' if best == 0 else 'AVOID') if phase == 'CRUISE' else phase
        else:
            best = int(np.argmin(risk+.01*cost))
            mode = 'ESCAPE' if phase == 'CRUISE' else phase
        selected = candidates[best].copy()
        if local_economy and best == 2 and safe[best]:
            mode = 'COAST'
            selected = vel.copy()
        if local_economy:
            self.policy.estimated_score = float(predicted_scores[best])
        ship_guard = phase not in ('CRUISE', 'LAUNCH', 'BRAKE_SHIP') or distance < 100.
        if not safe.any() and ship_guard:
            selected = target_vel.copy()
            if distance < 90. and self.arrival_time is None:
                selected += .4*(loc-target_loc)/max(distance, 1e-9)
            if self.arrival_time is None:
                self.terminal.inner_goal = None
            mode = 'SHIP_BLOCKED'
        # The terminal guard is applied AFTER local selection. It cannot be
        # overridden by an ESCAPE choice driving toward the station at speed.
        outward = (loc-target_loc)/max(distance, 1e-9)
        if phase not in ('CRUISE', 'LAUNCH', 'CAPTURE_PASS', 'POST_ENTRY_BRAKE'):
            if phase in ('BRAKE_SHIP', 'WAIT_SHIP_CAMERA', 'SURVEY_SHIP'):
                inward_limit = min(phase_cap, np.sqrt(2.*SHIP_BRAKE_ACCEL*max(distance-100., 0.)))
            else:
                inward_limit = min(phase_cap, np.sqrt(.08*max(distance-HOLD_RADIUS, 0.)))
            inward = -(selected-target_vel)@outward
            if inward > inward_limit:
                selected += (inward-inward_limit)*outward
        self.urgent = bool(distance < 250.)
        self.collision_urgent = False
        if len(centers):
            ranges = np.linalg.norm(centers, axis=1)
            closing = np.sum((vel-speeds)*centers, axis=1)/np.maximum(ranges, 1e-9)
            self.collision_urgent = bool(np.any((ranges-radii < 3.) | ((closing > 0.) & (ranges-radii < 3.*closing))))
            self.urgent |= self.collision_urgent
        self.previous_velocity = selected
        self.last_delta_v = float(effort[best])
        return velocity_command(selected, vel, MAX_ACCEL, urgent=self.urgent), mode, float(clearance[best])


class CoastDynamics:
    """Learn unpowered local drift from own motion, without obstacle truth."""
    def __init__(self):
        self.residual = np.zeros(3)
        self.samples = []
        self.updated = -np.inf
        self.damage = 0.
        self.quiet_after = 0.

    def gravity(self, location):
        location = np.asarray(location, float)
        return -711000.*location/max(np.linalg.norm(location)**3, 1.)+self.residual

    def observe(self, now, location, velocity, powered, damage=0.):
        if damage > self.damage+.1:
            self.quiet_after = now+3.
        self.damage = max(self.damage, float(damage))
        if powered or now < self.quiet_after:
            self.samples.clear()
            return
        nominal = -711000.*np.asarray(location)/max(np.linalg.norm(location)**3, 1.)
        self.samples.append((float(now), np.asarray(velocity).copy(), nominal))
        self.samples = [sample for sample in self.samples if now-sample[0] <= 3.]
        if now-self.samples[0][0] < 1.5 or now-self.updated < .5:
            return
        times = np.array([sample[0] for sample in self.samples])
        centered = times-times.mean()
        velocities = np.array([sample[1] for sample in self.samples])
        measured = (centered[:, None]*velocities).sum(axis=0)/max(centered@centered, 1e-9)
        drift = measured-np.mean([sample[2] for sample in self.samples], axis=0)
        if np.isfinite(drift).all() and np.linalg.norm(drift) < .02:
            self.residual = .8*self.residual+.2*limit_vector(drift, .01)
            self.updated = float(now)


class EconomyNavigator(FinalNavigator):
    def __init__(self, launch=True, nose_first=False):
        super().__init__(launch)
        self.terminal = CaptureApproach()
        self.policy, self.servo = EconomyPolicy(), EconomyServo()
        self.flythrough = True
        self.coasting = False
        self.corridor.economy = True
        self.gravity_tracking = True
        self.launch.speed = 3.
        self.nose_error = 180.
        self.nose_first = bool(nose_first)
        self.aligned_once, self.aligning = not self.nose_first, False
        self.failed = False
        self.dynamics = CoastDynamics()

    def gravity_at(self, location):
        return self.dynamics.gravity(location)

    def observe_motion(self, now, location, velocity, powered):
        # Ground contact and launch settling are not free-flight measurements.
        if self.launch.done:
            self.dynamics.observe(now, location, velocity, powered, self.last_damage)

    def observe_arrival(self, now, location, velocity, target, target_velocity, score_distance=None):
        """Catch a short scoring-sphere crossing between planning updates."""
        distance = np.linalg.norm(location-target) if score_distance is None else float(score_distance)
        if self.failed or self.arrival_time is not None or not np.isfinite(distance) or distance > 50.:
            return False
        self.arrival_time = float(now)
        self.terminal.enter(location, velocity, target_velocity)
        self.shooting_active = False
        self.coasting = False
        self.last_phase = self.last_mode = 'POST_ENTRY_BRAKE'
        motion = velocity-target_velocity
        self.previous_velocity = velocity-.2*motion/max(np.linalg.norm(motion), 1e-9)
        return True

    def observe_damage(self, damage):
        super().observe_damage(damage)
        if self.arrival_time is None and np.isfinite(damage) and damage > 209200.:
            self.failed = True

    def obstacles(self, now, loc, vel, radar):
        centers, speeds, radii = super().obstacles(now, loc, vel, radar)
        # Radar position sigma is 2 m in this scenario. Retain a 4 m position
        # margin plus vehicle radius, diameter inflation and age uncertainty,
        # rather than spending fuel to preserve the former extra 8 m margin.
        return centers, speeds, np.maximum(radii-4., 3.)

    def command(self, now, loc, vel, target_loc, target_vel, radar, camera=None,
                cruise_speed=None, maneuver_accel=None, fuel_weight=1.5):
        loc, vel, target_loc, target_vel = map(lambda v: np.asarray(v, float), (loc, vel, target_loc, target_vel))
        if self.failed:
            self.last_phase = self.last_mode = 'DAMAGE_LIMIT'
            self.previous_velocity = target_vel.copy()
            return np.zeros(3), self.last_mode, float('nan')
        direction = target_loc-loc
        distance = np.linalg.norm(direction)
        direction /= max(distance, 1e-9)
        gravity = self.gravity_at(loc)
        drag = max(0., -gravity@direction)
        forward = np.linalg.norm(vel-target_vel)
        # Near a tangent pass, radial closing speed tends to zero while actual
        # flight speed remains useful. Do not mistake that geometry for a need
        # to brake before scoring. Price distance along the capture line.
        remaining = max(1., np.sqrt(max(distance*distance-47.*47., 0.))-np.sqrt(50.*50.-47.*47.))
        speed = (self.policy.choose(remaining, forward, now, self.last_fuel,
                                    self.last_damage, drag) if self.arrival_time is None else self.policy.speed)
        if cruise_speed is not None:
            speed = min(speed, cruise_speed)
        self.terminal.camera_ready = camera is not None and camera.observes(now, target_loc)
        self.terminal.eta = self.policy.remaining_time
        self.corridor.economy_context = (now, self.last_fuel, self.last_damage, self.policy.fuel_per_dv)
        had_arrived = self.arrival_time is not None
        result, mode, clearance = super().command(now, loc, vel, target_loc, target_vel, radar,
                          camera=camera, cruise_speed=speed, maneuver_accel=maneuver_accel, fuel_weight=fuel_weight)
        if not had_arrived and self.arrival_time is not None:
            self.policy.estimated_score = mission_score(now, self.last_fuel, self.last_damage)
        self.aligning = False
        if (getattr(self, 'entry_coast', False) and self.arrival_time is None
                and self.launch.done and 50. < distance < 300. and self.terminal.camera_ready):
            entry = ballistic_entry(loc, vel, target_loc, target_vel, gravity)
            if np.isfinite(entry):
                entry_gap = coast_path_clearance(loc, vel, *self.latest_scene, entry,
                                                  self.latest_accels, gravity)
                if entry_gap > 2.:
                    # The whole occupied hull is checked up to a point 2 m
                    # inside the score boundary. No post-entry obstacle can
                    # justify cancelling a safe, already acquired trajectory.
                    self.coasting, self.shooting_active = True, False
                    self.collision_urgent = False
                    self.last_phase, self.last_mode = 'CAPTURE_PASS', 'ENTRY_COAST'
                    self.previous_velocity = vel.copy()
                    self.terminal.fallback = None
                    return np.zeros(3), self.last_mode, entry_gap
        if self.launch.done and not self.aligned_once and self.arrival_time is None:
            if self.nose_error < 8.:
                self.aligned_once = True
            elif self.coast_clearance > 2. and not self.collision_urgent:
                self.aligning = True
                self.previous_velocity = vel.copy()
                mode = 'ALIGN_FOR_BURN'
        when, miss = ballistic_pass(loc, vel, target_loc, target_vel, gravity)
        guided = self.previous_velocity.copy()
        direct_trim = None
        if distance > 300. and miss < 100. and np.isfinite(when) and self.terminal.inner_goal is not None:
            predicted = loc+when*(vel-target_vel)+.5*when*when*gravity
            trial = course_trim(vel, target_vel, (self.terminal.inner_goal-predicted)/max(when, 1.))
            if coast_path_clearance(loc, trial, *self.latest_scene, when, self.latest_accels, gravity) > 3.:
                direct_trim = trial
        route_alignment = float((guided-target_vel)@(vel-target_vel)
                                /max(np.linalg.norm(guided-target_vel)*forward, 1e-9))
        far_coast = (distance > 300. and mode == 'GUIDE' and clearance > 2.
                     and route_alignment > np.cos(np.deg2rad(3.)))
        self.coasting = bool(self.last_phase in ('CRUISE', 'CAPTURE_PASS') and self.terminal.fallback is None
                             and self.coast_clearance > 2.
                             and (mode == 'COAST' or (self.policy.burn_seconds < 3. and
                                  ((far_coast and direct_trim is None) or (44. < miss < 49.
                                  and when < self.policy.remaining_time+max(20./max(forward, 1.), 3.)))))
                             and not self.collision_urgent)
        if self.coasting:
            self.previous_velocity = vel.copy()
            mode = 'COAST'
        elif (self.policy.burn_seconds < 3. and self.last_phase in ('CRUISE', 'CAPTURE_PASS')
              and self.terminal.fallback is None and self.terminal.inner_goal is not None
              and self.coast_clearance > 2. and np.isfinite(when)
              and not self.collision_urgent and (distance <= 300. or mode == 'GUIDE')):
            predicted = loc+when*(vel-target_vel)+.5*when*when*gravity
            correction = ((self.terminal.inner_goal-predicted)/max(when, 1.)
                          if distance <= 300. or direct_trim is not None else guided-vel)
            # Correct the pass sideways; do not cancel useful forward speed
            # while the success boundary is still ahead.
            self.previous_velocity = course_trim(vel, target_vel, correction)
            mode = 'COURSE_TRIM'
        self.last_mode = mode
        return self.linear_command(vel, MAX_ACCEL), mode, clearance

    @property
    def last_fuel(self):
        return self.policy.previous[2] if self.policy.previous is not None else 0.

    def linear_command(self, velocity, maximum):
        if self.failed:
            return np.zeros(3)
        if self.aligning and not self.collision_urgent:
            return np.zeros(3)
        precise = self.last_phase != 'CRUISE' or self.collision_urgent or self.last_mode == 'COURSE_TRIM'
        if self.coasting:
            precise = self.collision_urgent
        return self.servo.command(self.previous_velocity, velocity, maximum, urgent=precise, coast=self.coasting)


def axis_schedule(displacements, times, orders, accelerations, delays=None):
    """Exact finite burns on three body axes to meet an endpoint at a set time."""
    displacements, times, orders = np.asarray(displacements), np.asarray(times), np.asarray(orders)
    count = len(times)
    impulses, starts, durations = np.zeros((count, 3)), np.zeros((count, 3)), np.zeros((count, 3))
    elapsed = np.zeros(count) if delays is None else np.asarray(delays).copy()
    valid = np.ones(count, dtype=bool)
    rows = np.arange(count)
    for stage in range(3):
        axes = orders[:, stage]
        distances = displacements[rows, axes]
        signs = np.where(distances >= 0., 0, 1)
        accel = accelerations[signs, axes]
        left = times-elapsed
        disc = left*left-2.*np.abs(distances)/accel
        valid &= (disc >= 0.) & (left >= 0.)
        duration = left-np.sqrt(np.maximum(disc, 0.))
        duration = np.maximum(duration, 0.)
        starts[rows, axes], durations[rows, axes] = elapsed, duration
        impulses[rows, axes] = np.sign(distances)*accel*duration
        elapsed += duration
    return impulses, starts, durations, valid


def scheduled_positions(times, velocity, gravity, rotation, impulses, starts, durations):
    """Relative world positions for finite, sequential body-axis burns."""
    t = np.asarray(times)[None, :, None]
    duration = durations[:, None]
    firing = np.minimum(np.maximum(t-starts[:, None], 0.), duration)
    coast = np.maximum(t-starts[:, None]-duration, 0.)
    acceleration = impulses/np.maximum(durations, 1e-12)
    body_positions = .5*acceleration[:, None]*firing*firing+impulses[:, None]*coast
    return body_positions@rotation+np.asarray(times)[None, :, None]*velocity+.5*np.asarray(times)[None, :, None]**2*gravity


def encounter_risk(gap, sigma, relative_speed):
    """Gaussian half-space probability bound and conservative energy proxy.

    This prices uncertain near passes, not the official contact damage model.
    Twice the vehicle's relative kinetic energy covers severe contact estimates.
    """
    probability = .5*np.exp(-.5*(np.maximum(gap, 0.)/np.maximum(sigma, .1))**2)
    energy = 3388.7*np.asarray(relative_speed)**2
    return probability, energy


def gaussian_encounter_risk(gap, sigma, relative_speed):
    """Normal half-space tail; retain the conservative impact-energy proxy."""
    import math
    z = np.maximum(gap, 0.)/(np.sqrt(2.)*np.maximum(sigma, .1))
    probability = .5*np.fromiter((math.erfc(float(value)) for value in z.ravel()),
                                 dtype=float, count=z.size).reshape(z.shape)
    return probability, 3388.7*np.asarray(relative_speed)**2


class ShootingNavigator(EconomyNavigator):
    """Choose a scored finite burn; check its next motion chunk from sensors."""
    def __init__(self, launch=True, nose_first=False, order_diversity=False, matched_camera_prediction=False):
        super().__init__(launch, nose_first)
        self.order_diversity = bool(order_diversity)
        self.matched_camera_prediction = bool(matched_camera_prediction)
        self.radar_cruise_only = False
        self.expected_risk = self.detour_diversity = False
        self.fine_avoidance = False
        self.efficient_main = self.main_accel_preloaded = False
        self.main_axis = 0
        self.entry_coast = False
        self.local_expected_risk = False
        self.body_rotation = np.eye(3)
        # Published official jet allocator, zero torque, 3388.7 kg reference mass.
        self.axis_accels = np.array([[.1745625, .1431816, .095813], [.096562, .096444, .11655]])
        self.orders = np.array([[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]])
        self.shooting_active = False
        self.selected_axis, self.selected_sign = 0, 1.
        self.selected_remaining = 0.
        self.burn_target = np.zeros(3)
        self.burn_started = 0.
        self.burn_gravity = np.zeros(3)
        self.control_time = 0.
        self.burn_order = np.array([0, 1, 2])
        self.axis_done = np.ones(3, dtype=bool)
        self.burn_signs = np.zeros(3)
        self.plan_chosen_at, self.counter_time = -np.inf, np.inf
        self.not_before = 0.
        self.previous_eta = None
        self.last_detour = np.zeros(3)
        self.measurement = None
        self.plan_score = -np.inf
        self.risk_planning = True
        self.expected_damage = self.contact_probability = 0.
        self.burn_tolerance = .004
        self.launch.required_height = 2.5

    def command(self, now, loc, vel, target_loc, target_vel, radar, camera=None,
                cruise_speed=None, maneuver_accel=None, fuel_weight=1.5):
        self.control_time = float(now)
        loc, vel, target_loc, target_vel = map(lambda value: np.asarray(value, float), (loc, vel, target_loc, target_vel))
        gravity = self.gravity_at(loc)
        rotation = self.body_rotation
        was_active = self.shooting_active
        if self.efficient_main and not self.main_accel_preloaded:
            self.axis_accels[0, 2] = .13467559
            self.main_accel_preloaded = True
        if self.measurement is not None:
            stamp, old_velocity, old_gravity, old_rotation = self.measurement
            axis, sign, remaining, active = self.selected_axis, self.selected_sign, self.selected_remaining, self.shooting_active
            dt = now-stamp
            if active and remaining > .15 and .3 < dt < 2.:
                achieved = old_rotation@(vel-old_velocity-.5*dt*(gravity+old_gravity))/dt
                value = sign*achieved[axis]
                if .025 < value < .3 and np.linalg.norm(np.delete(achieved, axis)) < .25*value:
                    index = 0 if sign > 0. else 1
                    self.axis_accels[index, axis] = .85*self.axis_accels[index, axis]+.15*value
        self.measurement = (now, vel.copy(), gravity.copy(), rotation.copy())
        distance = np.linalg.norm(target_loc-loc)
        self.shooting_active = False
        if (self.failed or not self.launch.done or distance < 280. or self.arrival_time is not None
                or self.nose_first):
            return super().command(now, loc, vel, target_loc, target_vel, radar, camera,
                                   cruise_speed, maneuver_accel, fuel_weight)
        centers, speeds, radii, rates, accels, _ = self.observe_scene(now, loc, vel, target_loc, target_vel, radar, camera)
        self.terminal.camera_ready = camera is not None and camera.observes(now, target_loc)
        _, phase, _ = self.terminal.goal(now, loc, vel, target_loc, target_vel, centers, radii)
        if phase != 'CRUISE' or self.terminal.inner_goal is None:
            return super().command(now, loc, vel, target_loc, target_vel, radar, camera,
                                   cruise_speed, maneuver_accel, fuel_weight)
        outward = (loc-target_loc)/distance
        tangent = self.terminal.tangent-outward*(outward@self.terminal.tangent)
        tangent /= max(np.linalg.norm(tangent), 1e-9)
        second = np.cross(outward, tangent)
        ratio = 47./distance
        radial_choices = np.array([np.cos(angle)*tangent+np.sin(angle)*second for angle in np.arange(16)*np.pi/8.])
        headings = -np.sqrt(1.-ratio*ratio)*outward+ratio*radial_choices
        goals = loc+distance*np.sqrt(1.-ratio*ratio)*headings
        goal_valid = np.ones(16, dtype=bool)
        if self.terminal.camera_ready:
            hull = (np.linalg.norm(centers+loc-target_loc, axis=1) < 85.) & (np.linalg.norm(speeds-target_vel, axis=1) < .1)
            for index in range(16):
                goal_valid[index] = segment_clearance(np.zeros(3), goals[index]+100.*headings[index]-loc,
                                                     centers[hull], radii[hull]) > 1.
        base_times = np.arange(45., 451., 10.)
        if self.previous_eta is not None and self.previous_eta-now > 10.:
            base_times = np.r_[base_times, self.previous_eta-now]
        delay_options = np.array([0.])
        goal_indices = np.repeat(np.arange(16), len(base_times)*6*len(delay_options))
        times = np.tile(np.repeat(base_times, 6*len(delay_options)), 16)
        orders = np.tile(np.repeat(self.orders, len(delay_options), axis=0), (16*len(base_times), 1))
        delays = np.tile(delay_options, 16*len(base_times)*6)
        displacement = (goals[goal_indices]-loc+(target_vel-vel)*times[:, None]-.5*times[:, None]**2*gravity)@rotation.T
        dv, starts, durations, valid = axis_schedule(displacement, times, orders, self.axis_accels, delays)
        end_velocity = vel+dv@rotation+durations.sum(axis=1)[:, None]*gravity
        limit = 10. if cruise_speed is None else cruise_speed
        valid &= np.linalg.norm(end_velocity-target_vel, axis=1) <= max(limit, np.linalg.norm(vel-target_vel))+.05
        valid &= goal_valid[goal_indices]
        # Changing the score optimum must not cancel useful forward momentum.
        # If every such trajectory is blocked, emergency local control can brake.
        forward = (target_loc-loc)/distance
        valid &= (dv@rotation)@forward >= -.04
        kg_per_dv = 3388.7/(80.*9.8)
        fuel_needed = kg_per_dv*np.abs(dv).sum(axis=1)
        scores = 100.*(.5+120./(now+times)-(self.last_fuel+fuel_needed)/75.-2.*self.last_damage/209200.)
        scores[~valid] = -np.inf
        order = np.argsort(-scores)
        chosen = [int(i) for i in order[:4] if np.isfinite(scores[i])]
        for goal_index in range(16):
            rows = np.flatnonzero((goal_indices == goal_index) & valid)
            if len(rows):
                chosen.extend(int(i) for i in rows[np.argsort(-scores[rows])[:1]] if int(i) not in chosen)
        if self.order_diversity:
            # Retain a low-cost lateral-first burn when the main-first shot
            # is blocked, before falling back to the local velocity planner.
            for axis_order in self.orders:
                rows = np.flatnonzero(valid & np.all(orders == axis_order, axis=1))
                if len(rows):
                    index = int(rows[np.argmax(scores[rows])])
                    if index not in chosen:
                        chosen.append(index)
        # Keep alternate arrival times and azimuths as well as the best score.
        for delay in delay_options:
            rows = np.flatnonzero(valid & (delays == delay))
            if len(rows):
                chosen.extend(int(i) for i in rows[np.argsort(-scores[rows])[:3]] if int(i) not in chosen)
        for lower, upper in ((100., 200.), (200., 280.), (280., 360.), (360., 450.)):
            rows = np.flatnonzero(valid & (times >= lower) & (times < upper))
            if len(rows):
                i = int(rows[np.argmax(scores[rows])])
                if i not in chosen:
                    chosen.append(i)
        if self.previous_eta is not None:
            retained = np.flatnonzero(valid & (goal_indices == 0) & (np.abs(now+times-self.previous_eta) < .1)
                                     & np.all(orders == self.burn_order, axis=1))
            chosen.extend(int(i) for i in retained if int(i) not in chosen)
        if not chosen:
            return super().command(now, loc, vel, target_loc, target_vel, radar, camera,
                                   cruise_speed, maneuver_accel, fuel_weight)
        c_dv, c_start, c_duration = [dv[i] for i in chosen], [starts[i] for i in chosen], [durations[i] for i in chosen]
        c_times, c_scores, c_orders = [times[i] for i in chosen], [scores[i] for i in chosen], [orders[i] for i in chosen]
        c_detours = [np.zeros(3) for _ in chosen]
        c_goals = [goal_indices[i] for i in chosen]
        c_counters = [times[i]/2. for i in chosen]
        c_delays = [delays[i] for i in chosen]
        forward = (target_loc-loc)/distance
        axis = np.eye(3)[np.argmin(np.abs(forward))]
        side = np.cross(forward, axis)
        side /= np.linalg.norm(side)
        up = np.cross(forward, side)
        offsets = [m*(np.cos(angle)*side+np.sin(angle)*up)
                   for m in (.15, .35, .7, 1.) for angle in np.arange(8)*np.pi/4.]
        if np.linalg.norm(self.last_detour) > .01:
            offsets.append(self.last_detour)
        detour_bases = chosen[:2]
        if self.detour_diversity and chosen:
            # Earlier order-diversity trials only varied direct shots. Also
            # give a lateral-first schedule access to the detour impulses.
            primary_axes = np.argmax(np.abs(dv), axis=1)
            lateral_first = np.flatnonzero(valid & (orders[:, 0] != primary_axes))
            if len(lateral_first):
                lateral = int(lateral_first[np.argmax(scores[lateral_first])])
                detour_bases = [chosen[0], lateral]
        for base in detour_bases:
            for offset in offsets:
                variant = dv[base]+rotation@offset
                duration = np.abs(variant)/self.axis_accels[np.where(variant >= 0., 0, 1), np.arange(3)]
                start, cursor = np.zeros(3), delays[base]
                for component in orders[base]:
                    start[component], cursor = cursor, cursor+duration[component]
                extra_return = 2.*np.abs(rotation@offset).sum()
                if cursor+extra_return/.08 >= times[base]:
                    continue
                cost = kg_per_dv*(np.abs(variant).sum()+extra_return)
                score = mission_score(now+times[base], self.last_fuel+cost, self.last_damage)
                c_dv.append(variant); c_start.append(start); c_duration.append(duration)
                c_times.append(times[base]); c_scores.append(score); c_orders.append(orders[base])
                c_detours.append(offset)
                c_goals.append(goal_indices[base])
                c_counters.append(max(times[base]/2., cursor))
                c_delays.append(delays[base])
        retained_index = None
        if (was_active and self.previous_eta is not None and self.previous_eta-now > 15.
                and goal_valid[0] and (np.linalg.norm(self.last_detour) < .01 or now < self.counter_time-1.)):
            residual = rotation@(self.burn_target+self.burn_gravity*(now-self.burn_started)-vel)
            residual[self.axis_done] = 0.
            duration = np.abs(residual)/self.axis_accels[np.where(residual >= 0., 0, 1), np.arange(3)]
            delay = max(0., self.not_before-now)
            start, cursor = np.zeros(3), delay
            for component in self.burn_order:
                start[component], cursor = cursor, cursor+duration[component]
            eta = self.previous_eta-now
            if cursor < eta:
                cost = kg_per_dv*(np.abs(residual).sum()+2.*np.abs(rotation@self.last_detour).sum())
                retained_index = len(c_scores)
                c_dv.append(residual); c_start.append(start); c_duration.append(duration)
                c_times.append(eta); c_scores.append(mission_score(now+eta, self.last_fuel+cost, self.last_damage))
                c_orders.append(self.burn_order.copy()); c_detours.append(self.last_detour.copy())
                c_goals.append(0); c_counters.append(self.counter_time-now)
                c_delays.append(delay)
        when, miss = ballistic_pass(loc, vel, target_loc, target_vel, gravity)
        if 42. < miss < 48. and 5. < when < 600.:
            # Preserve an already successful unpowered pass. It need not hit
            # exactly the old 47 m waypoint to earn the same success result.
            c_dv.append(np.zeros(3)); c_start.append(np.zeros(3)); c_duration.append(np.zeros(3))
            c_times.append(when); c_scores.append(mission_score(now+when, self.last_fuel, self.last_damage))
            c_orders.append(self.burn_order.copy()); c_detours.append(np.zeros(3))
            c_goals.append(0); c_counters.append(when/2.); c_delays.append(0.)
        c_dv, c_start, c_duration = np.asarray(c_dv), np.asarray(c_start), np.asarray(c_duration)
        c_times, c_scores = np.asarray(c_times), np.asarray(c_scores)
        return_dv = -2.*np.asarray(c_detours)@rotation.T
        return_duration = np.abs(return_dv)/self.axis_accels[np.where(return_dv >= 0., 0, 1), np.arange(3)]
        return_start = np.zeros_like(return_duration)
        for row, axis_order in enumerate(c_orders):
            cursor = c_counters[row]
            for component in axis_order:
                return_start[row, component] = cursor
                cursor += return_duration[row, component]
        # Receding sensor horizon: at cruise speed this covers 100--200 m.
        # Certifying a whole minute made distant uncertain tracks repeatedly
        # postpone otherwise useful burns. Revalidate this chunk every 0.5 s.
        horizon = float(min(20., .45*c_times.min()))
        samples = np.linspace(0., horizon, 21)
        positions = scheduled_positions(samples, vel, gravity, rotation, c_dv, c_start, c_duration)
        positions += scheduled_positions(samples, np.zeros(3), np.zeros(3), rotation,
                                         return_dv, return_start, return_duration)
        if retained_index is not None:
            row = retained_index
            endpoint = (scheduled_positions(np.array([c_times[row]]), vel, gravity, rotation,
                        c_dv[row:row+1], c_start[row:row+1], c_duration[row:row+1])+
                        scheduled_positions(np.array([c_times[row]]), np.zeros(3), np.zeros(3), rotation,
                        return_dv[row:row+1], return_start[row:row+1], return_duration[row:row+1]))[0, 0]
            mismatch = rotation@(goals[0]-loc+target_vel*c_times[row]-endpoint)
            correction = 2.*np.abs(mismatch).sum()/c_times[row]
            c_scores[row] -= 100.*kg_per_dv*correction/75.
        reach = np.max(np.linalg.norm(positions[:, -1], axis=1))
        relevant = np.linalg.norm(centers, axis=1) < reach+horizon*np.linalg.norm(speeds, axis=1)+radii+15.
        oc, ov, rr, aa, ur = centers[relevant], speeds[relevant], radii[relevant], accels[relevant], rates[relevant]
        nr = self.latest_radar_count
        confident = np.r_[self.radar_track_confidence, np.zeros(len(centers)-nr, dtype=bool)][relevant]
        ranged = (np.arange(len(centers)) < nr)[relevant]
        cores = np.r_[self.radar_core_radii, radii[nr:]][relevant]
        ages = np.r_[self.radar_report_ages, np.zeros(len(centers)-nr)][relevant]
        hard_radii = np.where(ranged, np.maximum(cores-1., 3.), rr)
        clearance = np.full(len(c_scores), np.inf)
        probability = np.zeros((len(c_scores), len(oc)))
        energy_risk = np.zeros_like(probability)
        fatal_risk = np.zeros_like(probability)
        recovery_risk = np.zeros_like(probability)
        for step, (t0, t1) in enumerate(zip(samples[:-1], samples[1:])):
            if not len(oc):
                break
            a = oc[None]+t0*ov[None]+.5*t0*t0*aa[None]-positions[:, step, None]
            b = oc[None]+t1*ov[None]+.5*t1*t1*aa[None]-positions[:, step+1, None]
            delta = b-a
            fraction = np.clip(-np.sum(a*delta, axis=2)/np.maximum(np.sum(delta*delta, axis=2), 1e-9), 0., 1.)
            distance_to_path = np.linalg.norm(a+fraction[:, :, None]*delta, axis=2)
            inflation = np.where(ranged, 0., np.minimum(10., t1*ur))
            curvature = .03+.25*(t1-t0)**2/8.
            gaps = distance_to_path-hard_radii[None]-inflation[None]-curvature
            sigma = np.sqrt(4.+((ages+t1)*ur/np.where(confident, 2.2, 1.2))**2)
            risk_model = gaussian_encounter_risk if self.expected_risk else encounter_risk
            p, energy = risk_model(distance_to_path-cores[None]-curvature, sigma[None],
                                       np.linalg.norm(delta, axis=2)/(t1-t0)+.125*(t1-t0))
            p[:, ~ranged] = 0.
            probability = np.maximum(probability, p)
            energy_risk = np.maximum(energy_risk, p*energy)
            if self.expected_risk:
                recovery_risk = np.maximum(recovery_risk, p*(5.+np.sqrt(energy/3388.7)/.1))
            fatal_risk = np.maximum(fatal_risk, np.where(energy > 209200.-self.last_damage, p, 0.))
            # The same monotonic departure rule as guarded launch; no new entry.
            leaving = (np.linalg.norm(oc, axis=1) < rr)[None] & (np.sum(a*delta, axis=2) > 1e-8)
            gaps[leaving] = np.inf
            clearance = np.minimum(clearance, gaps.min(axis=1))
        expected_damage = energy_risk.sum(axis=1)
        safe = ((clearance > .5) & (fatal_risk.sum(axis=1) < .01)
                & (expected_damage < .2*max(0., 209200.-self.last_damage)))
        c_scores -= 200.*expected_damage/209200.
        if self.expected_risk:
            # Assumed recovery: five seconds plus replacement of lost relative
            # speed at 0.1 m/s^2. Native elapsed time validates the full cost.
            recovery = recovery_risk.sum(axis=1)
            c_scores += 12000.*(1./(now+c_times+recovery)-1./(now+c_times))
        # Distant geometry is a soft preference: future velocity uncertainty is
        # too large to certify the entire mission. The next chunk remains hard.
        future_times = np.linspace(horizon+5., min(float(np.median(c_times)), 200.), 6)
        future_positions = scheduled_positions(future_times, vel, gravity, rotation, c_dv, c_start, c_duration)
        future_positions += scheduled_positions(future_times, np.zeros(3), np.zeros(3), rotation,
                                                return_dv, return_start, return_duration)
        future_risk = np.zeros(len(c_scores))
        for index, when in enumerate(future_times):
            offset = centers+when*speeds+.5*when*when*accels
            gaps = np.linalg.norm(offset[None]-future_positions[:, index, None], axis=2)-radii[None]-5.
            if len(centers):
                future_risk += np.maximum(0., -gaps).max(axis=1)*np.exp(-when/100.)/6.
        c_scores -= np.minimum(future_risk, .75)
        if not safe.any():
            return super().command(now, loc, vel, target_loc, target_vel, radar, camera,
                                   cruise_speed, maneuver_accel, fuel_weight)
        c_scores[~safe] = -np.inf
        best = int(np.argmax(c_scores))
        if self.previous_eta is not None:
            stable = np.flatnonzero(safe & (np.abs(now+c_times-self.previous_eta) < .1)
                                    & (np.asarray(c_goals) == 0)
                                    & np.all(np.asarray(c_orders) == self.burn_order, axis=1)
                                    & (c_scores > c_scores[best]-.5))
            if len(stable):
                best = int(stable[np.argmax(c_scores[stable])])
        if retained_index is not None and safe[retained_index]:
            allowance = 4. if now-self.plan_chosen_at < 4. else 2.
            if c_scores[retained_index] >= c_scores[best]-allowance:
                best = retained_index
        self.previous_eta = now+c_times[best]
        self.terminal.tangent = radial_choices[c_goals[best]].copy()
        self.terminal.inner_goal = goals[c_goals[best]].copy()
        self.last_detour = c_detours[best].copy()
        self.policy.estimated_score = self.plan_score = float(c_scores[best])
        self.expected_damage = float(expected_damage[best])
        self.contact_probability = float(min(1., probability[best].sum()))
        self.policy.remaining_time = float(c_times[best])
        self.policy.burn_seconds = float(c_duration[best].sum())
        self.policy.speed = float(np.linalg.norm(vel+c_dv[best]@rotation-target_vel))
        self.last_phase = 'CRUISE'
        self.burn_tolerance = .02
        self.stopping_distance = float(np.linalg.norm(vel-target_vel)**2/(2.*SHIP_BRAKE_ACCEL))
        self.last_mode = 'IMPULSE_BURN'
        self.last_horizon = int(horizon)
        self.last_delta_v = float(np.abs(c_dv[best]).sum())
        self.urgent = self.collision_urgent = False
        self.coasting = False
        self.shooting_active = True
        if best != retained_index:
            self.burn_target = vel+c_dv[best]@rotation
            self.burn_gravity, self.burn_started = gravity.copy(), float(now)
            self.burn_order = np.asarray(c_orders[best]).copy()
            self.axis_done = np.abs(c_dv[best]) < self.burn_tolerance
            self.burn_signs = np.sign(c_dv[best])
            self.plan_chosen_at, self.counter_time = float(now), now+c_counters[best]
            self.not_before = now+c_delays[best]
        self.selected_remaining = 0.
        for component in c_orders[best]:
            remaining = c_dv[best, component]
            if abs(remaining) > self.burn_tolerance:
                self.selected_axis, self.selected_sign = int(component), float(np.sign(remaining))
                self.selected_remaining = float(abs(remaining))
                break
        self.previous_velocity = self.burn_target.copy()
        if self.selected_remaining == 0.:
            self.last_mode, self.coasting = 'IMPULSE_COAST', True
            self.previous_velocity = vel.copy()
        elif self.not_before > now:
            self.last_mode, self.coasting = 'WAIT_CLEAR_PATH', True
        return self.linear_command(vel, MAX_ACCEL), self.last_mode, float(clearance[best])

    def linear_command(self, velocity, maximum):
        if not self.shooting_active:
            command = super().linear_command(velocity, maximum)
            if self.last_phase == 'LAUNCH':
                pulse = axis_thrust(command, self.body_rotation, maximum)
                return pulse if pulse@self.launch.up >= .9*np.linalg.norm(pulse) else command
            return command
        if self.failed:
            return np.zeros(3)
        if self.control_time < self.not_before:
            return np.zeros(3)
        expected = self.burn_target+self.burn_gravity*max(0., self.control_time-self.burn_started)
        residual = self.body_rotation@(expected-np.asarray(velocity))
        for component in self.burn_order:
            if self.axis_done[component]:
                continue
            if self.burn_signs[component]*residual[component] > self.burn_tolerance:
                self.selected_axis, self.selected_sign = int(component), float(self.burn_signs[component])
                self.selected_remaining = float(abs(residual[component]))
                direction = self.selected_sign*self.body_rotation[component]
                if self.efficient_main and component == 2 and self.selected_sign > 0.:
                    direction = (direction+.0002*self.body_rotation[0])/np.sqrt(1.+.0002**2)
                return maximum*direction
            self.axis_done[component] = True
        self.selected_remaining = 0.
        return np.zeros(3)


class EconomyAttitude(FlightAttitude):
    def __init__(self):
        super().__init__()
        self.adjusting = True
        self.aligning = False

    def command(self, now, rotation, velocity, desired_velocity, planet_up, omega, maximum):
        command = super().command(now, rotation, velocity, desired_velocity, planet_up, omega, maximum)
        if self.aligning:
            return attitude_command(rotation, self.forward, self.up, omega, maximum, rate_cap=.18, stopping_alpha=.03)
        roll_error = float(np.rad2deg(np.arccos(np.clip(np.asarray(rotation)[2]@self.up, -1., 1.))))
        if self.nose_error_deg > 8. or roll_error > 20. or np.linalg.norm(omega) > .015:
            self.adjusting = True
        elif self.nose_error_deg < 3. and roll_error < 8. and np.linalg.norm(omega) < .004:
            self.adjusting = False
        return command if self.adjusting else np.zeros(3)


class FreeAttitude(FlightAttitude):
    """No nose pointing or horizon leveling; damp an actual unwanted spin only."""
    free_attitude = True

    def __init__(self):
        super().__init__()
        self.aligning = self.damping = False

    def command(self, now, rotation, velocity, desired_velocity, planet_up, omega, maximum):
        rotation, velocity, omega = map(lambda value: np.asarray(value, float), (rotation, velocity, omega))
        if rotation.shape != (3, 3) or not all(np.isfinite(value).all() for value in (rotation, velocity, omega)):
            return np.zeros(3)
        self.body_velocity = rotation@velocity
        travel = velocity if np.linalg.norm(velocity) >= .2 else np.asarray(desired_velocity)
        self.forward = travel/max(np.linalg.norm(travel), 1e-9) if np.linalg.norm(travel) > 1e-9 else rotation[0].copy()
        self.up = rotation[2].copy()
        self.nose_error_deg = float(np.rad2deg(np.arccos(np.clip(rotation[0]@self.forward, -1., 1.))))
        self.vertical = abs(self.forward@np.asarray(planet_up)/max(np.linalg.norm(planet_up), 1e-9)) > .94
        rate = np.linalg.norm(omega)
        if rate > .01:
            self.damping = True
        elif rate < .002:
            self.damping = False
        return limit_vector(-.8*omega, maximum) if self.damping else np.zeros(3)


class MainAxisSlew:
    """A single minimal turn toward a fixed burn axis, then no pointing servo."""
    def __init__(self, heading, started, body_axis=0):
        self.heading = np.asarray(heading)/max(np.linalg.norm(heading), 1e-9)
        self.body_axis = int(body_axis)
        self.started, self.complete = float(started), False
        self.settling = False

    def angular_command(self, rotation, omega, maximum):
        body_heading = np.asarray(rotation)@self.heading
        angle = np.arccos(np.clip(body_heading[self.body_axis], -1., 1.))
        self.settling = bool(angle < np.deg2rad(2.))
        if self.settling:
            # Small requests are amplified by the official on-time remap.
            # Settle without translation before starting a fixed-axis burn.
            self.complete = bool(np.linalg.norm(omega) < .0001)
            return np.zeros(3) if self.complete else limit_vector(-1.5*np.asarray(omega)/60., maximum)
        axis = np.cross(np.eye(3)[self.body_axis], body_heading)
        norm = np.linalg.norm(axis)
        axis = axis/norm if norm > 1e-9 else np.eye(3)[(self.body_axis+1)%3]
        rate = min(.4, np.sqrt(2.*.03*max(angle-.01, 0.)))
        desired = axis*rate if angle > .02 else np.zeros(3)
        self.complete = False
        return np.zeros(3) if self.complete else limit_vector(1.5*(desired-omega), maximum)


class AxisAlignedNavigator(ShootingNavigator):
    """Opt-in one-time boost/slew; all subsequent flight uses Momentum."""
    def __init__(self, launch=True, nose_first=False, order_diversity=False, matched_camera_prediction=False):
        super().__init__(launch, nose_first, order_diversity, matched_camera_prediction)
        self.main_axis_slew = None
        self.slew_active, self.slew_abandoned = False, bool(nose_first)

    def command(self, now, loc, vel, target_loc, target_vel, radar, camera=None,
                cruise_speed=None, maneuver_accel=None, fuel_weight=1.5):
        loc, vel, target_loc, target_vel = map(lambda v: np.asarray(v, float), (loc, vel, target_loc, target_vel))
        self.control_time = float(now)
        self.slew_active = False
        if not self.launch.done:
            self.launch.goal(loc, target_loc)
        if self.launch.done and not self.slew_abandoned and self.arrival_time is None and not self.failed:
            if self.main_axis_slew is None:
                heading = target_loc-loc
                self.main_axis_slew = MainAxisSlew(heading, now, self.main_axis)
            slew = self.main_axis_slew
            if now-slew.started > 25. and not slew.complete:
                self.slew_abandoned = True
            elif not slew.complete:
                centers, speeds, radii, rates, accels, _ = self.observe_scene(
                    now, loc, vel, target_loc, target_vel, radar, camera)
                relative = speeds-vel
                times = np.clip(-np.sum(centers*relative, axis=1)
                    /np.maximum(np.sum(relative*relative, axis=1), 1e-9), 0., 4.)
                gap = np.linalg.norm(centers+times[:, None]*relative, axis=1)-radii-1.
                departing = ((np.linalg.norm(centers, axis=1) < radii)
                             & (np.sum(centers*relative, axis=1) > 0.))
                if not np.any((gap < 0.) & ~departing):
                    self.slew_active, self.shooting_active, self.coasting = True, False, False
                    self.previous_velocity = vel.copy()
                    self.last_phase = self.last_mode = 'ALIGN_MAIN_AXIS'
                    return self.linear_command(vel, MAX_ACCEL), self.last_mode, float(gap.min()) if len(gap) else np.inf
        return super().command(now, loc, vel, target_loc, target_vel, radar, camera,
                              cruise_speed, maneuver_accel, fuel_weight)

    def linear_command(self, velocity, maximum):
        if self.slew_active and not self.failed and self.arrival_time is None:
            return (np.zeros(3) if self.main_axis_slew.complete or self.main_axis_slew.settling else
                    min(.025, maximum)*self.main_axis_slew.heading)
        return super().linear_command(velocity, maximum)


class MainAxisAttitude(FreeAttitude):
    def __init__(self, planner):
        super().__init__()
        self.planner = planner

    def command(self, now, rotation, velocity, desired_velocity, planet_up, omega, maximum):
        damping = super().command(now, rotation, velocity, desired_velocity, planet_up, omega, maximum)
        if self.planner.slew_active:
            return self.planner.main_axis_slew.angular_command(rotation, np.asarray(omega), maximum)
        return damping


nose_first_enabled = os.environ.get('EFP_NOSE_FIRST', '0') == '1'
strong_axis_enabled = os.environ.get('EFP_STRONG_AXIS', '1') == '1'
main_axis_y_enabled = os.environ.get('EFP_MAIN_AXIS_Y', '0') == '1'
entry_coast_enabled = os.environ.get('EFP_ENTRY_COAST', '0') == '1'
local_expected_risk_enabled = os.environ.get('EFP_LOCAL_EXPECTED_RISK', '1') == '1'
order_diversity_enabled = os.environ.get('EFP_ORDER_DIVERSITY', '0') == '1'
camera_track_rate_enabled = os.environ.get('EFP_CAMERA_TRACK_RATE', '0') == '1'
radar_cruise_enabled = os.environ.get('EFP_RADAR_CRUISE', '1') == '1'
expected_risk_enabled = os.environ.get('EFP_EXPECTED_RISK', '1') == '1'
detour_diversity_enabled = os.environ.get('EFP_DETOUR_DIVERSITY', '0') == '1'
fine_avoidance_enabled = os.environ.get('EFP_FINE_AVOIDANCE', '1') == '1'
efficient_main_enabled = os.environ.get('EFP_EFFICIENT_MAIN', '0') == '1'
axis_thrust_enabled = os.environ.get('EFP_AXIS_THRUST', '0') == '1'
rescue_frame = target.GetBodyFixedFrame()
planner = (AxisAlignedNavigator if strong_axis_enabled else ShootingNavigator)(nose_first=nose_first_enabled, order_diversity=order_diversity_enabled, matched_camera_prediction=camera_track_rate_enabled)
planner.radar_cruise_only = radar_cruise_enabled
planner.expected_risk, planner.detour_diversity = expected_risk_enabled, detour_diversity_enabled
planner.fine_avoidance = fine_avoidance_enabled
planner.efficient_main = efficient_main_enabled
planner.main_axis = 1 if main_axis_y_enabled else 0
planner.entry_coast = entry_coast_enabled
planner.local_expected_risk = local_expected_risk_enabled
try:
    import hashlib
    from pathlib import Path
    st.logger_info('IGNITION_SOURCE sourceSHA256='+hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
                   +f' strongAxis={strong_axis_enabled} mainAxis={planner.main_axis} orderDiversity={order_diversity_enabled}'
                   +f' cameraTrackRate={camera_track_rate_enabled} radarCruise={radar_cruise_enabled}'
                   +f' expectedRisk={expected_risk_enabled} detourDiversity={detour_diversity_enabled}'
                   +f' fineAvoidance={fine_avoidance_enabled} efficientMain={efficient_main_enabled}'
                   +f' entryCoast={entry_coast_enabled} localExpectedRisk={local_expected_risk_enabled}')
except Exception:
    pass  # Diagnostic logging must never inhibit the flight controller.
flight_attitude = MainAxisAttitude(planner) if strong_axis_enabled else EconomyAttitude() if nose_first_enabled else FreeAttitude()
# Radar.py timestamps are relative to its own startup clock, not StartDateTAI.
# Controller and radar start concurrently; their small startup skew is covered by margins.
radar_epoch = st.SimGlobals.SimClock.GetTimeNow().as_datetime()
pose_history = PoseHistory()
perception = CameraPerception()
preview = None
preview_error = ''
if os.environ.get('EFP_CAMERA_GUI', '1') != '0':
    try:
        preview = start_camera_preview(__file__)
        st.OnScreenLogMessage('Camera GUI snapshots: '+str(preview.source), 'CameraGUI', st.Severity.Info)
    except Exception as exc:
        preview_error = str(exc)
        st.OnScreenLogMessage('Camera GUI unavailable: '+preview_error, 'CameraGUI', st.Severity.Info)
camera_manager = CameraManager(st, dict(top=topCamera, bottom=bottomCamera), PanTiltCamera, radar_epoch, preview)
score_policy = planner.policy
fuel_ledger = FuelLedger()
last_plan = -np.inf
last_log = -np.inf
plan_ms = 0.
accel_world = np.zeros(3)
angular_accel = np.zeros(3)
mode, clearance = 'START', np.inf
damage, fuel = 0., 0.
arrival_announced = False
linear_limit = min(MAX_ACCEL, shuttle.GetParam(st.VarType.double, ['Thrusters', 'MaxAccel_m_s2']))
angular_limit = min(0.02, shuttle.GetParam(st.VarType.double, ['Thrusters', 'MaxAngAccel_rad_s2']))
try:
    while True:
        time.sleep(1.0 / this.GetParam(st.VarType.double, 'LoopFreqHz'))
        now = (st.SimGlobals.SimClock.GetTimeNow().as_datetime()-radar_epoch).total_seconds()
        if now < last_plan:
            planner = (AxisAlignedNavigator if strong_axis_enabled else ShootingNavigator)(nose_first=nose_first_enabled, order_diversity=order_diversity_enabled, matched_camera_prediction=camera_track_rate_enabled)
            planner.radar_cruise_only = radar_cruise_enabled
            planner.expected_risk, planner.detour_diversity = expected_risk_enabled, detour_diversity_enabled
            planner.fine_avoidance = fine_avoidance_enabled
            planner.efficient_main = efficient_main_enabled
            planner.main_axis = 1 if main_axis_y_enabled else 0
            planner.entry_coast = entry_coast_enabled
            planner.local_expected_risk = local_expected_risk_enabled
            pose_history = PoseHistory()
            perception = CameraPerception()
            score_policy = planner.policy
            fuel_ledger = FuelLedger()
            camera_manager.reset()
            if preview is not None:
                preview.reset()
            flight_attitude = MainAxisAttitude(planner) if strong_axis_enabled else EconomyAttitude() if nose_first_enabled else FreeAttitude()
            last_plan = last_log = -np.inf
            arrival_announced = False
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
        planner.body_rotation = body_dcm
        planner.control_time = now
        planner.observe_motion(now, loc, vel,
            np.linalg.norm(accel_world) > 1e-9 or np.linalg.norm(angular_accel) > 1e-9)
        score_distance = np.linalg.norm(loc-target_loc)
        if score_distance < 65.:
            # Match Scoring.py's origin/frame query, including any actor/COM offset.
            score_distance = np.linalg.norm(shuttle.getLocation().WRT_ExprIn(rescue_frame))
        if planner.arrival_time is None and score_distance <= 50.:
            fuel = shuttle.GetParam(st.VarType.double, ['Stats', 'FuelUsed'])
            damage = shuttle.GetParam(st.VarType.double, ['Stats', 'Damage'])
            planner.observe_damage(damage)
            if planner.observe_arrival(now, loc, vel, target_loc, target_vel, score_distance):
                fuel_ledger.observe(now, fuel, 'ARRIVAL', arrived=True)
                planner.policy.estimated_score = mission_score(now, fuel, damage)
        camera_manager.drain(now, pose_history, perception)
        sightline = vel-target_vel
        if np.linalg.norm(sightline) < 1. or np.linalg.norm(target_loc-loc) < 300.:
            sightline = target_loc-loc
        camera_manager.tick(now, body_dcm@sightline)
        if now-last_plan >= PLAN_PERIOD:
            plan_wall_start = time.perf_counter()
            # Read the entire radar list; stale sorting can hide an approaching object.
            radar = GetNearestDebrisLocations(10000)
            radar = [np.asarray(report, dtype=float) for report in radar]
            radar = [report for report in radar if report.shape == (3, 3)
                     and np.isfinite(report).all() and np.linalg.norm(report[0]) > 1e-6
                     and -5.0 <= now-report[2, 0] <= 10.0]
            fuel = shuttle.GetParam(st.VarType.double, ['Stats', 'FuelUsed'])
            damage = shuttle.GetParam(st.VarType.double, ['Stats', 'Damage'])
            score_policy.observe(now, vel, fuel,
                thrusting=np.linalg.norm(accel_world) > .08 and (not nose_first_enabled or flight_attitude.nose_error_deg < 10.) and np.linalg.norm(angular_accel) < .003,
                gravity=planner.gravity_at(loc))
            planner.nose_error = flight_attitude.nose_error_deg
            planner.observe_damage(damage)
            # The economy policy chooses the useful burn/coast peak below this
            # ceiling; sensor checks can require a lower speed or a stop.
            cruise = 10.0
            if len(radar) == 0:
                # Depth only observes a short local range; it cannot justify a
                # 25 m/s approach into unobserved space if radar is unavailable.
                cruise = min(cruise, np.sqrt(2.*BRAKE_ACCEL*max(min(CAMERA_MAX_RANGE, 220.)-40., 0.)))
            fresh_depth = any(0. <= now-stamp <= CAMERA_MAX_AGE and len(regions)
                              for stamp, regions in perception.occupied.values())
            if len(radar) == 0 and not fresh_depth:
                planner.previous_velocity = target_vel.copy()
                mode, clearance = 'WAIT_RADAR', float('nan')
                planner.coasting = False
                planner.last_phase = mode
                planner.last_mode = mode
                planner.shooting_active = False
            else:
                accel_world, mode, clearance = planner.command(
                    now, loc, vel, target_loc, target_vel, radar, camera=perception,
                    cruise_speed=cruise)
                cruise = min(cruise, planner.policy.speed)
            fuel_phase = (planner.last_phase+'/'+mode if planner.last_phase == 'CRUISE' else mode)
            fuel_ledger.observe(now, fuel, fuel_phase, arrived=planner.arrival_time is not None)
            last_plan = now
            plan_ms = 1000.*(time.perf_counter()-plan_wall_start)
        if planner.arrival_time is not None and not arrival_announced:
            st.OnScreenAlert('Arrival zone reached; post-arrival safety control active. Official result is in the Scoring log.',
                             'ArrivalZone', st.Severity.Info)
            arrival_announced = True
        # The planner runs at 2 Hz; the velocity servo runs every control tick.
        # Reusing a fixed acceleration for the whole planning period can overshoot.
        accel_world = planner.linear_command(vel, linear_limit)
        if axis_thrust_enabled:
            accel_world = axis_thrust(accel_world, body_dcm, linear_limit)
        accel_body = st.frames.FramedVector(accel_world, phobosFrame).ExprIn(shuttleFrame)
        omega_body = np.asarray(shuttle.getAngVelocity().WRT(phobosFrame).ExprIn(shuttleFrame))
        # Phobos-centred position defines the local outward/up direction.
        up_world = loc/max(np.linalg.norm(loc), 1e-9)
        if np.linalg.norm(up_world) < 1e-6:
            up_world = np.array([0.0, 0.0, 1.0])
        nose_hint = attitude_reference(mode, vel-target_vel, planner.previous_velocity-target_vel, target_loc-loc)
        flight_attitude.aligning = planner.aligning
        angular_accel = flight_attitude.command(now, body_dcm, vel-target_vel,
            nose_hint, up_world, omega_body, angular_limit)
        if planner.failed:
            angular_accel = np.zeros(3)
        if not np.isfinite(angular_accel).all():
            angular_accel = np.zeros(3)
        SetCommand(accel_body, angular_accel)
        if preview is not None:
            preview.update(running=True, profile='SCORE_MOMENTUM', now=now, mode=mode,
                speed=float(np.linalg.norm(vel-target_vel)), cruise=cruise,
                distance=float(np.linalg.norm(target_loc-loc)), fuel=fuel, damage=damage,
                images=camera_manager.received, rejected=perception.rejected,
                rgb_matches=perception.rgb_associations, depth_matches=perception.depth_associations,
                camera_fault=camera_manager.error, nose_error=flight_attitude.nose_error_deg,
                nose_first=nose_first_enabled,
                body_velocity=flight_attitude.body_velocity.tolist(), vertical_climb=flight_attitude.vertical,
                arrival_zone=planner.arrival_time is not None, arrival_time=planner.arrival_time,
                arrival_action=('PARKED AFTER ENTRY' if planner.terminal.entered and planner.last_phase == 'HOLD'
                                else 'POST-ENTRY BRAKING' if planner.terminal.entered else 'HOLDING POSITION'),
                projected_score=planner.policy.estimated_score,
                scored_fuel=(fuel_ledger.previous[1] if fuel_ledger.frozen else None))
        if now-last_log >= 5.0:
            damage = shuttle.GetParam(st.VarType.double, ['Stats', 'Damage'])
            fuel = shuttle.GetParam(st.VarType.double, ['Stats', 'FuelUsed'])
            st.OnScreenLogMessage(
                f'IGNITION {mode} profile=SCORE_MOMENTUM t={now:.1f}s distance={np.linalg.norm(target_loc-loc):.1f}m '
                f'speed={np.linalg.norm(vel-target_vel):.2f}m/s clearance={clearance:.1f}m '
                f'noseFirst={nose_first_enabled} '
                f'strongAxis={strong_axis_enabled} '
                f'orderDiversity={order_diversity_enabled} '
                f'cameraTrackRate={camera_track_rate_enabled} '
                f'radarCruise={radar_cruise_enabled} '
                f'expectedRisk={expected_risk_enabled} detourDiversity={detour_diversity_enabled} '
                f'fineAvoidance={fine_avoidance_enabled} '
                f'efficientMain={efficient_main_enabled} '
                f'axisThrust={axis_thrust_enabled} '
                f'planMs={plan_ms:.1f} '
                f'coastDrift={np.array2string(planner.dynamics.residual, precision=4, separator=",")}m/s2 '
                f'damage={damage:.1f}J fuel={fuel:.2f}kg '
                f'fuelPhaseKg=[{fuel_ledger.summary()}] '
                f'lookahead={planner.last_horizon}s planDV={planner.last_delta_v:.2f}m/s '
                f'stopDistance={planner.stopping_distance:.1f}m routeNodes={len(planner.corridor.points)} '
                f'chunk={planner.corridor.chunk_id} nextReady={bool(planner.corridor.pending) or len(planner.corridor.points) > 8} '
                f'buffer={planner.corridor.buffer_m:.1f}m replans={planner.corridor.replans} launchHeight={planner.launch.height:.1f}m '
                f'laneTurn={planner.corridor.lane_turn:.0f}deg arrivalZone={planner.arrival_time is not None} '
                f'cmdAccel={np.linalg.norm(accel_world):.3f}m/s2 cruise={cruise:.1f}m/s '
                f'projectedScore={planner.policy.estimated_score:.1f} burnRemaining={planner.policy.burn_seconds:.1f}s '
                f'images={camera_manager.received} rejected={perception.rejected} '
                f'rgbMatches={perception.rgb_associations} depthMatches={perception.depth_associations} '
                f'cameraFault={camera_manager.error or "none"} '
                f'attitude={"NOSE_FOLLOW" if nose_first_enabled else "FREE_ATTITUDE"} noseError={flight_attitude.nose_error_deg:.1f}deg '
                f'travelBody={np.array2string(flight_attitude.body_velocity, precision=2, separator=",")}m/s '
                f'guiFault={(preview.error if preview is not None else preview_error) or "none"}',
                'IgnitionController', st.Severity.Info)
            last_log = now
finally:
    try:
        SetCommand(np.zeros(3), np.zeros(3))
    finally:
        try:
            if preview is not None:
                preview.close()
        finally:
            st.leave_sim()
