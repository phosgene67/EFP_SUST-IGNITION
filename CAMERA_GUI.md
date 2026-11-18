# Camera interface

The active flight profile is **SCORE_CORRIDOR** for `FINAL_SUBMISSION.simconf`.
See [the controller overview](README.md#current-controller) for current navigation.
After entry, the footer shows **Entry fuel** separately from the current total,
which continues increasing during post-arrival braking.
The FUSION25 comparison at the end is a historical experiment.

## Open the live interface

Copy **both** `Controller.py` and `camera_viewer.py` into the local
`STU4-Solution` folder used by the official simulation. Start the competition
simulation normally. An **EFP Camera Interface** OpenCV window opens automatically.
The controller remains responsible for the camera API; the separate viewer reads
its snapshots and draws the window.

The installed STPro v0.56.0 Python environment has OpenCV 5.0.0 with WIN32UI support.
The installed camera bindings confirm RGB/depth output modes, capture timestamps,
row-major pixel storage, and depth in centimetres. No changes to STU4-Sim are needed.

![Synthetic demonstration of the camera interface](docs/score_eco_interface_demo.png)

This screenshot is a **synthetic demonstration**, not a live mission result.

## What the window shows

| Panel or element | Meaning |
| --- | --- |
| Top / RGB and Bottom / RGB | Actual colour captures from the two physical cameras. Yellow boxes mark the contrast regions used by RGB perception. |
| Top / Depth and Bottom / Depth | Actual depth captures converted from centimetres for display. Colours encode camera-forward surface depth: warmer is nearer, cooler is farther, black is invalid/out of range. |
| Green depth boxes | Detected connected depth surfaces, labelled with the nearest camera-forward depth in metres. Red boxes indicate a surface closer than 30 m. |
| Frame timestamp / age | Each panel's own capture time. RGB and depth are separate captures and need not be synchronized. |
| STALE | A panel's last frame is over two simulation seconds old. |
| REJECTED FOR NAVIGATION | The image arrived but could not be associated with a valid recent pose/time. It is displayed without a detection overlay to aid diagnosis. |
| Header | Controller mode, simulation time, measured speed, requested cruise, target distance, fuel, damage, processed/rejected frames, radar associations, and an explicitly unofficial projected score. |
| FREE ATTITUDE / NOSE FIRST / ALIGNING | Default is FREE ATTITUDE: no fuel is spent forcing the nose along velocity or leveling the horizon. The displayed angle is informational. The old optional nose-first mode retains its alignment labels. |
| Camera status | Capture error information, if present. |
| ARRIVAL ZONE REACHED banner | Persistent controller observation of entry, followed by braking/parking; consult the official Scoring log for the congratulations and score. |

These boxes come from the actual perception code. They are generic contrast/depth
regions, not a trained classifier assigning labels such as "meteoroid" or "ship".
RGB boxes do not invent a metric distance. Depth may include terrain and the rescue
ship as well as debris. The navigation pipeline excludes very close self geometry;
the raw preview can still show it.

The native capture size remains 96×64 to limit simulator/rendering load. The viewer
magnifies those pixels. It refreshes up to 10 times/s, but the capture schedule is
at most four requests/s total, alternating top/bottom with depth, depth, RGB on each
camera. The viewer does not create artificial intermediate frames.

Focus the viewer and press **Q** or **Escape**, or close its window, to close only
the interface. Flight continues. At normal controller shutdown the last available
images remain visible with **RUN ENDED** until you close the viewer.

## Architecture and failure behavior

`CameraPerception` stores image-space detection boxes. `CameraManager.drain()`
forwards each received frame and its corresponding boxes to `PreviewPublisher`.
The publisher keeps only the latest four streams and writes an atomic NumPy
snapshot on a background thread, at most 5 times/s. OpenCV reads that snapshot in
a separate process. Window event handling and disk compression do not wait inside
the flight loop. The viewer never sends thruster or camera commands.

Snapshots and `viewer.log` are stored in a per-run temporary folder named
`efp-cameras-*`. The controller logs its exact `latest.npz` path under `CameraGUI`.
Only the latest snapshot is retained in that folder. It is a diagnostic snapshot,
not a replacement for the official simulation logs or screen recording.

If the helper is missing, OpenCV cannot open a window, or the viewer exits, the
controller reports `guiFault` and continues navigation. The active GUI is optional;
camera perception does not depend on it. In an embedded Python host the publisher
tries to locate a Python executable, and `EFP_VIEWER_PYTHON` can override that choice.

## Troubleshooting and manual preview

1. Check that `camera_viewer.py` is beside the **installed** `Controller.py`, not
   only in the GitHub working copy. Missing-helper errors appear under `CameraGUI`.
2. Check that `images` increases. Waiting panels with `images=0` indicate missing
   camera frames; inspect `cameraFault`. GUI rendering cannot repair a capture API
   timeout or an absent camera.
3. For a viewer that closes immediately, open the per-run `viewer.log` next to the
   snapshot. It records import/window errors. The controller's `guiFault` includes
   its location.
4. For stale/rejected panels, inspect simulation clock/pose errors and capture
   latency. Detection overlays are not copied from a different frame.

Preview the layout without starting a simulation, using Python with NumPy/OpenCV:

```powershell
python camera_viewer.py --demo
```

If your default Python lacks OpenCV, use the simulator environment:

```powershell
& "$env:LOCALAPPDATA\SimDynamX\STPro\Versions\v0.56.0\py\Scripts\python.exe" camera_viewer.py --demo
```

Reopen a running or saved controller snapshot by replacing the source argument with
the exact path printed in the `CameraGUI` log:

```powershell
python camera_viewer.py --source "C:\path\from\CameraGUI\latest.npz"
```

For headless layout validation:

```powershell
python camera_viewer.py --demo --save docs/camera_interface_demo.png
```

To suppress automatic GUI startup, set `EFP_CAMERA_GUI=0` in the environment of the
process launching the controller. Setting it only in an unrelated terminal does
not change an already-running Space Teams process.

## Current speed and depth display

`SCORE_CORRIDOR` selects finite burns and coasting using the score formula, with a
10 m/s ceiling. It checks moving obstacles over each next 20 s chunk and replans
at 2 Hz. Actual speed depends on thrust, fuel cost and sensed clearance. The
historical SCORE_COAST modeled runs peaked at 6.38--6.87 m/s.

The depth display and processing limit are 400 m. This is a software limit,
not a guarantee of usable real depth at that distance. The radar-loss speed
guard still uses its conservative 220 m envelope. A camera-verified tangent
pass enters the safe area before braking; observation and slow-approach
fallbacks remain available when the pass cannot be established.

## Historical nose-first flight and the steep climb

This section records the earlier default. The current default is `FREE ATTITUDE`;
`EFP_NOSE_FIRST=1` enables the older comparison mode.

The earlier selected behavior was to point the nose along the actual flight path, including
a steep climb. The official initial positions give a direct route approximately
**85.7° above the local horizon**. A nose-up exterior appearance is therefore
expected on that route; keeping the body level would be a different orientation
choice. The controller does not rotate the nose toward braking thrust.

`FlightAttitude` now maintains a continuous roll reference near vertical flight.
Planet-up is nearly parallel to the nose during the climb, so projecting it into a
roll reference amplified small lateral velocity changes in the previous version.
The new controller transports the preceding reference through vertical flight,
uses 20°/30° hysteresis around the vertical cone, and gradually recovers the normal
horizon reference as the flight path levels out. It follows measured relative
velocity above 0.5 m/s and the planned direction during low-speed startup.

The GUI shows `NOSE FIRST` below 10° alignment error during motion, or `ALIGNING`
while turning. Logs include `attitude=NOSE_FOLLOW`, `noseError=...deg` and
`travelBody=[X,Y,Z]m/s`. Positive X with small Y/Z means actual forward flight;
a large Z component means the rover is still moving through its roof/bottom while
turning. A turn takes time because torque/rate limits remain active. If `noseError`
stays large after the initial turn, preserve those lines with the recording to
diagnose actual actuator behavior. The controller does not alter the pilot's
free-look camera or the official simulation configuration.

### Historical FUSION25 validation and tradeoff

The earlier profile passed **49 tests**, including transport shutdown/races, all four streams,
OpenCV rendering, actual detection overlays, camera-only fallback, and a 25 m/s
state's braking lookahead. The dashboard was rendered to PNG and visually
checked. New attitude tests integrate a turn into a vertical flight path, verify
that measured motion becomes body +X, check roll continuity under small lateral
noise, and ensure braking does not reverse the nose. Live attitude/window behavior
still needs the next simulator run.

The offline 38-jet comparison against the prior fusion controller is saved in
`tests/fusion25_benchmark_results.json` (recorded before the NOSE_FOLLOW roll update;
its source hash identifies that earlier snapshot):

| Case | Prior fusion time / fuel | FUSION25 time / fuel | Prior estimated score | FUSION25 estimated score |
| --- | --- | --- | ---: | ---: |
| Clear route | 208.7 s / 60.58 kg | 194.1 s / 89.36 kg | 26.72 | −7.33 |
| Moving clutter, seed 7 | 191.3 s / 95.04 kg | 199.8 s / 98.18 kg | −14.00 | −20.84 |
| Moving clutter, seed 19 | 199.6 s / 100.75 kg | 215.3 s / 110.38 kg | −24.21 | −41.44 |

All six compared runs arrived without synthetic sphere contact. The higher cap
shortened the clear case but did not improve estimated score; both clutter cases
were slower. This is an **experimental high-speed profile**, not a demonstrated
score improvement. A score such as −151 can be dominated by fuel consumption or
damage, so the next live test should preserve its complete statistics and the GUI
capture diagnostics. The model remains an approximation, not the official simulator.

Reproduce the comparison with NumPy/SciPy and the official jet configuration:

```powershell
python tests/benchmark_final.py --baseline-ref 090e96b60b81e2717d9ba62c99f244d0cae40447 --baseline-fusion --jet-config "path\to\STU4-Sim\Data\SpaceBuggyJetConfigNoMains.csv" --output tests/fusion25_benchmark_results.json
```
