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
SHIP_BRAKE_ACCEL = 0.065
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
            self.boxes[(name, mode)] = boxes[:64]
        self.frames += 1
        return True

    def observes(self, now, point):
        """A fresh depth capture actually covered this world-space bearing."""
        tan_y = np.tan(np.deg2rad(CAMERA_FOV/2.-5.))
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
    if angle < np.deg2rad(2.) and np.linalg.norm(omega) < .006:
        return np.zeros(3)
    return limit_vector(desired_rate-omega, maximum)


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
    """500-object final scenario: global corridor + short local control + ship guard.

    Only the base class's radar tracker is reused. Its long inflated velocity
    forecasts are retained as historical code but are not the active planner.
    """
    def __init__(self):
        super().__init__()
        self.corridor = CorridorPlanner()
        self.terminal = TerminalApproach()
        self.stopping_distance = 0.
        self.last_phase = 'START'

    def command(self, now, loc, vel, target_loc, target_vel, radar,
                camera=None, cruise_speed=None, maneuver_accel=None, fuel_weight=1.5):
        loc, vel = np.asarray(loc, float), np.asarray(vel, float)
        target_loc, target_vel = np.asarray(target_loc, float), np.asarray(target_vel, float)
        radar_centers, radar_speeds, radar_radii = self.obstacles(now, loc, vel, radar)
        centers, speeds, radii = radar_centers, radar_speeds, radar_radii
        if camera is not None:
            cc, cv, cr = camera.obstacles(now, loc, centers, speeds, radii)
            centers, speeds = np.vstack((centers, cc)), np.vstack((speeds, cv))
            radii = np.r_[radii, cr]
        goal, phase, phase_cap = self.terminal.goal(now, loc, vel, target_loc, target_vel, centers, radii)
        distance = np.linalg.norm(target_loc-loc)
        relative_velocity = vel-target_vel
        self.stopping_distance = float(np.linalg.norm(relative_velocity)**2/(2.*SHIP_BRAKE_ACCEL))
        if camera is not None and phase in ('FINAL_CREEP', 'SURVEY_SHIP'):
            depth_ready = camera.observes(now, target_loc)
            if not depth_ready:
                goal, phase, phase_cap = target_loc+100.*self.terminal.side, 'WAIT_SHIP_CAMERA', 0.
        self.last_phase = phase
        if phase == 'CRUISE' or (phase == 'BRAKE_SHIP' and distance > 180.):
            waypoint = self.corridor.waypoint(now, loc, goal, radar_centers, radar_radii)
        else:
            waypoint = goal
        route = waypoint-loc
        length = np.linalg.norm(route)
        direction = route/max(length, 1e-9)
        cruise = MAX_SPEED if cruise_speed is None else float(np.clip(cruise_speed, 0., MAX_SPEED))
        cap = min(cruise, phase_cap)
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
        horizon = 15 if phase == 'CRUISE' else 6
        if phase == 'CRUISE' and len(centers):
            ranges = np.linalg.norm(centers, axis=1)
            closing = np.sum((vel-speeds)*centers, axis=1)/np.maximum(ranges, 1e-9)
            contact_times = (ranges-radii)/np.maximum(closing, 1e-9)
            incoming = contact_times[(closing > 1.) & (contact_times > horizon) & (contact_times < 27.)]
            if len(incoming):
                horizon = min(30, int(np.ceil(incoming.min()))+3)
        self.last_horizon = horizon
        predictive_accel = .14 if maneuver_accel is None else float(np.clip(maneuver_accel, .08, .25))
        if len(centers):
            # Keep all 500 reports in mapping/tracking, but roll out only volumes
            # physically reachable within this forecast (including incoming ones).
            reach = np.linalg.norm(vel)*horizon+.5*predictive_accel*horizon*horizon
            relevant = np.linalg.norm(centers, axis=1) <= (reach+horizon*np.linalg.norm(speeds, axis=1)
                                                        +radii+.6*horizon+1.)
            centers, speeds, radii = centers[relevant], speeds[relevant], radii[relevant]
        positions = np.zeros((len(candidates), 3))
        velocities = np.tile(vel, (len(candidates), 1))
        clearance, risk, effort = np.full(len(candidates), np.inf), np.zeros(len(candidates)), np.zeros(len(candidates))
        for step in range(horizon):
            accel = VELOCITY_GAIN*(candidates-velocities)
            accel *= np.minimum(1., predictive_accel/np.maximum(np.linalg.norm(accel, axis=1), 1e-9))[:, None]
            end_position = positions+velocities+.5*accel
            effort += np.linalg.norm(accel, axis=1)
            if len(centers):
                start = centers[None]+step*speeds[None]-positions[:, None]
                end = centers[None]+(step+1)*speeds[None]-end_position[:, None]
                delta = end-start
                fraction = np.clip(-np.sum(start*delta, axis=2)/np.maximum(np.sum(delta*delta, axis=2), 1e-9), 0., 1.)
                gap = np.linalg.norm(start+fraction[:, :, None]*delta, axis=2)-radii[None]-.6*(step+1)-.03
                clearance = np.minimum(clearance, gap.min(axis=1))
                risk += np.maximum(-gap, 0.).sum(axis=1)/(1.+.1*step)
            positions, velocities = end_position, velocities+accel
        hint_distance = np.linalg.norm(positions-route, axis=1)
        cost = hint_distance+.6*np.linalg.norm(candidates-self.previous_velocity, axis=1)+.3*effort
        safe = clearance > 0.
        if safe.any():
            cost[~safe] = np.inf
            best = int(np.argmin(cost))
            mode = ('GUIDE' if best == 0 else 'AVOID') if phase == 'CRUISE' else phase
        else:
            best = int(np.argmin(risk+.01*cost))
            mode = 'ESCAPE' if phase == 'CRUISE' else phase
        selected = candidates[best].copy()
        if not safe.any() and phase != 'CRUISE':
            selected = target_vel.copy()
            if distance < 90.:
                selected += .4*(loc-target_loc)/max(distance, 1e-9)
            self.terminal.inner_goal = None
            mode = 'SHIP_BLOCKED'
        # The terminal guard is applied AFTER local selection. It cannot be
        # overridden by an ESCAPE choice driving toward the station at speed.
        outward = (loc-target_loc)/max(distance, 1e-9)
        if phase != 'CRUISE':
            if phase in ('BRAKE_SHIP', 'WAIT_SHIP_CAMERA', 'SURVEY_SHIP'):
                inward_limit = min(phase_cap, np.sqrt(2.*SHIP_BRAKE_ACCEL*max(distance-100., 0.)))
            else:
                inward_limit = min(phase_cap, np.sqrt(.08*max(distance-HOLD_RADIUS, 0.)))
            inward = -(selected-target_vel)@outward
            if inward > inward_limit:
                selected += (inward-inward_limit)*outward
        self.urgent = bool(distance < 250.)
        if len(centers):
            ranges = np.linalg.norm(centers, axis=1)
            closing = np.sum((vel-speeds)*centers, axis=1)/np.maximum(ranges, 1e-9)
            self.urgent |= bool(np.any((ranges-radii < 3.) | ((closing > 0.) & (ranges-radii < 3.*closing))))
        self.previous_velocity = selected
        self.last_delta_v = float(effort[best])
        return velocity_command(selected, vel, MAX_ACCEL, urgent=self.urgent), mode, float(clearance[best])


planner = FinalNavigator()
flight_attitude = FlightAttitude()
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
score_policy = ScorePolicy()
last_plan = -np.inf
last_log = -np.inf
accel_world = np.zeros(3)
mode, clearance = 'START', np.inf
damage, fuel = 0., 0.
linear_limit = min(MAX_ACCEL, shuttle.GetParam(st.VarType.double, ['Thrusters', 'MaxAccel_m_s2']))
angular_limit = min(0.02, shuttle.GetParam(st.VarType.double, ['Thrusters', 'MaxAngAccel_rad_s2']))
try:
    while True:
        time.sleep(1.0 / this.GetParam(st.VarType.double, 'LoopFreqHz'))
        now = (st.SimGlobals.SimClock.GetTimeNow().as_datetime()-radar_epoch).total_seconds()
        if now < last_plan:
            planner = FinalNavigator()
            pose_history = PoseHistory()
            perception = CameraPerception()
            score_policy = ScorePolicy()
            camera_manager.reset()
            if preview is not None:
                preview.reset()
            flight_attitude = FlightAttitude()
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
        if np.linalg.norm(sightline) < 1. or np.linalg.norm(target_loc-loc) < 300.:
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
            damage = shuttle.GetParam(st.VarType.double, ['Stats', 'Damage'])
            score_policy.observe(now, vel, fuel)
            # The final scenario needs decisive motion through the corridor.
            # TerminalApproach and collision checks impose the actual safe speed.
            cruise = MAX_SPEED
            if len(radar) == 0:
                # Depth only observes a short local range; it cannot justify a
                # 25 m/s approach into unobserved space if radar is unavailable.
                cruise = min(cruise, np.sqrt(2.*BRAKE_ACCEL*max(CAMERA_MAX_RANGE-40., 0.)))
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
        # Phobos-centred position defines the local outward/up direction.
        up_world = loc/max(np.linalg.norm(loc), 1e-9)
        if np.linalg.norm(up_world) < 1e-6:
            up_world = np.array([0.0, 0.0, 1.0])
        nose_hint = attitude_reference(mode, vel-target_vel, planner.previous_velocity-target_vel, target_loc-loc)
        angular_accel = flight_attitude.command(now, body_dcm, vel-target_vel,
            nose_hint, up_world, omega_body, angular_limit)
        if not np.isfinite(angular_accel).all():
            angular_accel = np.zeros(3)
        SetCommand(accel_body, angular_accel)
        if preview is not None:
            preview.update(running=True, profile='FINAL_NAV_500', now=now, mode=mode,
                speed=float(np.linalg.norm(vel-target_vel)), cruise=cruise,
                distance=float(np.linalg.norm(target_loc-loc)), fuel=fuel, damage=damage,
                images=camera_manager.received, rejected=perception.rejected,
                rgb_matches=perception.rgb_associations, depth_matches=perception.depth_associations,
                camera_fault=camera_manager.error, nose_error=flight_attitude.nose_error_deg,
                body_velocity=flight_attitude.body_velocity.tolist(), vertical_climb=flight_attitude.vertical)
        if now-last_log >= 5.0:
            damage = shuttle.GetParam(st.VarType.double, ['Stats', 'Damage'])
            fuel = shuttle.GetParam(st.VarType.double, ['Stats', 'FuelUsed'])
            st.OnScreenLogMessage(
                f'IGNITION {mode} profile=FINAL_NAV_500 t={now:.1f}s distance={np.linalg.norm(target_loc-loc):.1f}m '
                f'speed={np.linalg.norm(vel-target_vel):.2f}m/s clearance={clearance:.1f}m '
                f'damage={damage:.1f}J fuel={fuel:.2f}kg '
                f'lookahead={planner.last_horizon}s planDV={planner.last_delta_v:.2f}m/s '
                f'stopDistance={planner.stopping_distance:.1f}m routeNodes={len(planner.corridor.points)} '
                f'cmdAccel={np.linalg.norm(accel_world):.3f}m/s2 cruise={cruise:.1f}m/s '
                f'images={camera_manager.received} rejected={perception.rejected} '
                f'rgbMatches={perception.rgb_associations} depthMatches={perception.depth_associations} '
                f'cameraFault={camera_manager.error or "none"} '
                f'attitude=NOSE_FOLLOW noseError={flight_attitude.nose_error_deg:.1f}deg '
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
