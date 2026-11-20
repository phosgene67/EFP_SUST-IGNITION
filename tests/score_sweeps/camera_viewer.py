"""Live EFP sensor window. Controller publishes snapshots; this process draws them.

OpenCV is imported only by the viewer, so a missing/closed GUI cannot stop flight.
Run `python camera_viewer.py --demo`, or pass --save preview.png for headless QA.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from threading import Event, Lock, Thread
import time
import numpy as np


def json_safe(value):
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def replace_snapshot(temporary, destination):
    """Bounded retry for a Windows reader/antivirus briefly holding the file."""
    for attempt in range(4):
        try:
            os.replace(temporary, destination)
            return
        except PermissionError:
            if attempt == 3:
                raise
            time.sleep(.005)


class PreviewPublisher:
    """Latest-only, bounded disk transport written on a background thread."""
    def __init__(self, viewer_script=None, launch=True, directory=None):
        self.directory = Path(directory or tempfile.mkdtemp(prefix='efp-cameras-'))
        self.directory.mkdir(parents=True, exist_ok=True)
        self.source = self.directory/'latest.npz'
        self.lock, self.changed, self.stopping = Lock(), Event(), Event()
        self.frames, self.status = {}, {}
        self.error = ''
        self.process = None
        self.log = None
        self.thread = Thread(target=self._writer, name='EFP camera snapshot writer', daemon=True)
        self.thread.start()
        if launch:
            try:
                self.log = open(self.directory/'viewer.log', 'w', encoding='utf-8')
                executable = os.environ.get('EFP_VIEWER_PYTHON') or sys.executable
                if not Path(executable).name.lower().startswith('python'):
                    # Some simulator builds embed Python in a native host.
                    candidates = [Path(sys.prefix)/'Scripts/python.exe', Path(sys.prefix)/'python.exe']
                    versions = Path(os.environ.get('LOCALAPPDATA', ''))/'SimDynamX/STPro/Versions'
                    candidates += sorted(versions.glob('*/py/Scripts/python.exe'), reverse=True)
                    executable = next((str(path) for path in candidates if path.is_file()), executable)
                if not Path(executable).name.lower().startswith('python'):
                    raise RuntimeError('Set EFP_VIEWER_PYTHON to Python with NumPy/OpenCV installed')
                command = [executable, '-u', str(viewer_script or Path(__file__).resolve()),
                           '--source', str(self.source)]
                self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                    stdout=self.log, stderr=self.log,
                    creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            except Exception as exc:
                self.error = 'Viewer launch failed: '+str(exc)

    def submit(self, name, mode, pixels, stamp, boxes, accepted, yaw=0., pitch=0.):
        if name not in ('top', 'bottom') or mode not in ('rgb', 'depth'):
            return
        array = np.asarray(pixels)
        if array.ndim not in (2, 3) or array.size > 1920*1080*3:
            return
        key = name+'_'+mode
        with self.lock:
            self.frames[key] = (array.copy(), dict(stamp=float(stamp), boxes=boxes[:64],
                accepted=bool(accepted), yaw=float(yaw), pitch=float(pitch)))
        self.changed.set()

    def update(self, **status):
        with self.lock:
            self.status.update(status)
        self.changed.set()
        if self.process is not None and self.process.poll() is not None and not self.error:
            self.error = 'Viewer closed/exited; flight continues. Details: '+str(self.directory/'viewer.log')

    def reset(self):
        with self.lock:
            self.frames.clear()
            self.status = dict(mode='RESET', now=0.)
        self.changed.set()

    def _writer(self):
        while True:
            self.changed.wait(.2)
            if self.changed.is_set():
                self.changed.clear()
                with self.lock:
                    arrays = {key: row[0] for key, row in self.frames.items()}
                    metadata = dict(status=self.status.copy(),
                                    frames={key: row[1] for key, row in self.frames.items()},
                                    written_wall=time.time())
                try:
                    arrays['metadata'] = np.array(json.dumps(json_safe(metadata), allow_nan=False))
                    temporary = self.directory/'latest.tmp'
                    with open(temporary, 'wb') as output:
                        np.savez_compressed(output, **arrays)
                    replace_snapshot(temporary, self.source)
                    if self.error.startswith('Snapshot write failed:'):
                        self.error = ''
                except Exception as exc:
                    self.error = 'Snapshot write failed: '+str(exc)
            if self.stopping.is_set() and not self.changed.is_set():
                break
            # Cap disk publication at 5 Hz even when the control loop is 60 Hz.
            self.stopping.wait(.2)

    def close(self):
        self.update(running=False)
        self.stopping.set()
        self.changed.set()
        self.thread.join(timeout=1.)
        if self.log is not None:
            self.log.close()
        # Leave the final image visible. Closing the viewer never stops flight.


def read_snapshot(source):
    # Windows denies replacing a file while another process has it open without
    # delete sharing. Copy bytes and close immediately, before NPZ decompression.
    import io
    payload = Path(source).read_bytes()
    with np.load(io.BytesIO(payload), allow_pickle=False) as data:
        metadata = json.loads(str(data['metadata'].item()))
        frames = {key: data[key].copy() for key in ('top_rgb', 'top_depth', 'bottom_rgb', 'bottom_depth')
                  if key in data}
    return frames, metadata


def depth_bgr(depth_cm, far=400.):
    import cv2
    meters = np.asarray(depth_cm, dtype=float)*.01
    valid = np.isfinite(meters) & (meters > 0.) & (meters < far)
    normalized = np.zeros(meters.shape, dtype=np.uint8)
    normalized[valid] = np.clip(255.*(1.-np.sqrt(meters[valid]/far)), 0., 255.).astype(np.uint8)
    result = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    result[~valid] = 0
    return result


def render_dashboard(frames, metadata):
    """Pure compositor shared by the live window and deterministic screenshot tests."""
    import cv2
    canvas = np.full((950, 1360, 3), (21, 17, 15), dtype=np.uint8)
    white, muted, cyan = (238, 238, 235), (164, 157, 146), (226, 205, 62)
    status = metadata.get('status', {})
    font = cv2.FONT_HERSHEY_SIMPLEX
    def text(value, x, y, color=white, scale=.55):
        cv2.putText(canvas, str(value), (x, y), font, scale, color, 1, cv2.LINE_AA)
    def number(key, digits=1):
        value = status.get(key)
        return '--' if value is None else f'{float(value):.{digits}f}'
    text('EFP  /  CAMERA & DETECTION MONITOR', 24, 36, cyan, .8)
    tag = 'SYNTHETIC DEMO' if status.get('demo') else ('LIVE' if status.get('running', True) else 'RUN ENDED')
    if (not status.get('demo') and status.get('running', True) and frames
            and time.time()-metadata.get('written_wall', time.time()) > 3.):
        tag = 'NO RECENT UPDATES'
    text(tag+'  |  '+str(status.get('profile', 'waiting for controller')), 920, 36, cyan, .53)
    text(f"Mode {status.get('mode', '--')}     t {number('now')} s     Speed {number('speed', 2)} m/s"
         f"     Requested cruise {number('cruise')} m/s     Target {number('distance')} m", 24, 70)
    if status.get('nose_error') is not None:
        error = float(status['nose_error'])
        moving = float(status.get('speed') or 0.) >= .5
        free = status.get('nose_first') is False
        phase = 'FREE ATTITUDE' if free else 'NOSE FIRST' if moving and error < 10. else 'ALIGNING' if error >= 10. else 'HEADING HOLD'
        text(f'{phase}  {error:.1f} deg', 1050, 70, cyan if free else (80, 225, 140) if error < 10. else (80, 170, 255), .47)
    text(f"Fuel {number('fuel', 2)} kg     Damage {number('damage')} J     Accepted frames {status.get('images', 0)}"
         f"     Rejected {status.get('rejected', 0)}     Radar matches: RGB {status.get('rgb_matches', 0)}"
         f" / depth {status.get('depth_matches', 0)}", 24, 99, muted)
    if status.get('projected_score') is not None:
        text('Estimated score '+number('projected_score')+' (unofficial)', 1040, 99, muted, .42)
    frame_info = metadata.get('frames', {})
    for index, key in enumerate(('top_rgb', 'top_depth', 'bottom_rgb', 'bottom_depth')):
        x = 24+(index % 2)*668
        y = 120+(index//2)*365
        width, height = 644, 350
        cv2.rectangle(canvas, (x, y), (x+width, y+height), (53, 45, 38), -1)
        info = frame_info.get(key, {})
        stamp = info.get('stamp')
        age = max(0., status.get('now', stamp or 0.)-(stamp or 0.))
        stale = stamp is not None and age > 2.
        label = key.replace('_', '  /  ').upper()
        label += '   [STALE]' if stale else ''
        text(label, x+12, y+24, (90, 130, 250) if stale else white, .6)
        image = frames.get(key)
        if image is None:
            text('Waiting for '+key.replace('_', ' ')+' capture...', x+80, y+160, muted)
            text('Depth, depth, RGB alternate on each physical camera.', x+35, y+195, muted, .48)
            continue
        if key.endswith('rgb'):
            if image.ndim != 3 or image.shape[2] != 3:
                continue
            display = cv2.cvtColor(np.clip(image, 0, 255).astype(np.uint8), cv2.COLOR_RGB2BGR)
        else:
            display = depth_bgr(image)
        native_height, native_width = display.shape[:2]
        scale = min((width-20)/native_width, 270/native_height)
        dw, dh = int(native_width*scale), int(native_height*scale)
        px, py = x+(width-dw)//2, y+39+(270-dh)//2
        canvas[py:py+dh, px:px+dw] = cv2.resize(display, (dw, dh), interpolation=cv2.INTER_NEAREST)
        for box in info.get('boxes', [])[:64]:
            a, b, c, d = box['xyxy']
            left = px+int(np.clip(a, 0, native_width-1)*scale)
            right = px+int(np.clip(c+1, 1, native_width)*scale)
            top = py+int(np.clip(b, 0, native_height-1)*scale)
            bottom = py+int(np.clip(d+1, 1, native_height)*scale)
            distance = box.get('range_m')
            color = ((60, 80, 255) if distance is not None and distance < 30.
                     else (70, 225, 130) if distance is not None else (70, 220, 240))
            cv2.rectangle(canvas, (left, top), (right, bottom), color, 2)
            label = f'{distance:.1f} m surface' if distance is not None else 'RGB contrast region'
            text(label, left+2, max(py+14, top-5), color, .43)
        caption = f"{native_width} x {native_height}  |  t={stamp:.2f}s  age={age:.2f}s" if stamp is not None else 'No timestamp'
        caption += f"  |  {len(info.get('boxes', []))} regions"
        text(caption, x+12, y+327, muted, .47)
        if not info.get('accepted', True):
            text('REJECTED FOR NAVIGATION: stale frame or missing pose', x+12, y+345, (80, 100, 255), .43)
    text('Depth: warmer = nearer, cooler = farther; black = no usable depth. Scale 0-400 m.', 24, 877, muted, .5)
    if status.get('arrival_zone'):
        if status.get('scored_fuel') is not None:
            text(f"Entry fuel {status['scored_fuel']:.2f} kg (score frozen)",
                 840, 877, muted, .45)
        text('ARRIVAL ZONE REACHED - '+str(status.get('arrival_action', 'HOLDING POSITION'))+'. Official result: Scoring log.',
             24, 903, (80, 225, 140), .55)
    else:
        text('Boxes show actual image regions. RGB boxes have no metric range; depth labels are surface depth.', 24, 903, muted, .5)
    fault = status.get('camera_fault') or 'none'
    text('Camera status: '+str(fault)[:138], 24, 928, (120, 170, 255) if fault != 'none' else muted, .48)
    text('Q / Esc: close viewer only', 1100, 928, cyan, .44)
    return canvas


def demo_snapshot():
    """Clearly marked synthetic inputs; never presented as live simulator frames."""
    height, width = 64, 96
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    rgb[17:34, 22:42] = (128, 116, 104)
    rgb[35:46, 65:76] = (165, 145, 127)
    depth = np.full((height, width), np.inf, dtype=np.float32)
    depth[17:34, 22:42] = 2450.
    depth[35:46, 65:76] = 7800.
    frames = dict(top_rgb=rgb, top_depth=depth, bottom_rgb=np.flipud(rgb).copy(), bottom_depth=np.flipud(depth).copy())
    info = {}
    for key in frames:
        flip = key.startswith('bottom')
        boxes = []
        for rect, distance in (([22, 17, 41, 33], 24.5), ([65, 35, 75, 45], 78.)):
            if flip:
                rect = [rect[0], height-1-rect[3], rect[2], height-1-rect[1]]
            boxes.append(dict(xyxy=rect, range_m=distance if key.endswith('depth') else None))
        info[key] = dict(stamp=42., accepted=True, boxes=boxes)
    metadata = dict(frames=info, written_wall=time.time(), status=dict(demo=True, profile='SCORE_MOMENTUM',
        running=True, now=42.25, mode='AVOID', speed=8.8, cruise=9., distance=680., fuel=28.4,
        damage=0., images=123, rejected=0, rgb_matches=2, depth_matches=2, nose_error=96.8,
        body_velocity=[-1., 8.7, .2], projected_score=35., nose_first=False))
    return frames, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path)
    parser.add_argument('--demo', action='store_true')
    parser.add_argument('--save', type=Path, help='Write a preview PNG without opening a window')
    args = parser.parse_args()
    import cv2
    if args.demo:
        frames, metadata = demo_snapshot()
    elif args.source:
        frames, metadata = {}, dict(status={}, frames={})
    else:
        parser.error('Use --demo or --source <latest.npz>. Controller launches the live viewer automatically.')
    title = 'EFP Camera Interface | '+('DEMO' if args.demo else str(args.source.parent.name))
    last_mtime = None
    if not args.save:
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    while True:
        if args.source and args.source.exists():
            try:
                mtime = args.source.stat().st_mtime_ns
                if mtime != last_mtime:
                    frames, metadata = read_snapshot(args.source)
                    last_mtime = mtime
            except (OSError, ValueError, KeyError, json.JSONDecodeError):
                pass  # Keep the last complete frame during atomic file replacement.
        canvas = render_dashboard(frames, metadata)
        if args.save:
            args.save.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(args.save), canvas):
                raise RuntimeError('Could not write preview PNG')
            return
        cv2.imshow(title, canvas)
        if cv2.waitKey(100) & 0xff in (ord('q'), 27):
            break
        if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
            break
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
