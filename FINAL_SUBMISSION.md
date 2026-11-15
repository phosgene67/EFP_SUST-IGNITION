# FINAL_NAV_500 controller

> Historical version. The active controller is now **OPEN_LANE_9**. See
> [OPEN_LANE_9.md](OPEN_LANE_9.md) for the guarded launch and rolling
> predicted trajectories. All source hashes and benchmark numbers below refer
> to the earlier controller, not the current implementation.

Use this version with **FINAL_SUBMISSION.simconf**. The protected connection and
camera/command helper prefix in `Controller.py` is unchanged. The installed
official STU4-Sim is read for verification and testing, never edited.

## Why the previous flight was slow and hit the ship

The installed final scene has 500 debris objects. Its radar measures 120 objects
per second, so a full refresh takes about 4.17 seconds. The older planner inflated
all objects across long forecasts, sometimes up to 220 seconds. That frequently
rejected useful routes and caused repeated thrust corrections.

The November 12 live log showed 5.52 m/s at about 60 m from the station, then
102,733 J damage. The scoring log recorded success at 318.66 s, 206.88 kg fuel and
score -286.4. Success means entering the ship's 50 m scoring sphere while below
the damage threshold. It does **not** mean collision-free docking or zero speed.
The scorer does not stop the rover after displaying congratulations.

## Current architecture

All navigation remains in `Controller.py`:

| Component | Responsibility |
| --- | --- |
| `QualificationPlanner.obstacles()` | Timestamped radar tracking, own-motion correction and conservative obstacle bounds; radar list indices are never identities. The old class's trajectory planner is retained but inactive. |
| `CameraManager`, `PoseHistory`, `CameraPerception` | Asynchronous top/bottom RGB and depth, timestamp/pose projection, occupied surface bounds, detection overlays and observation coverage. RGB supplies bearings, depth supplies geometry. |
| `CorridorPlanner` | A 3-D A* route around radar obstacles, simplified with continuous segment clearance checks. Replans about every 5 s and retains an equivalent clear route to reduce steering jitter. |
| `FinalNavigator` | Follows the corridor and evaluates acceleration-limited local trajectories every 0.5 s. Uses 15 s forecasts, extended to at most 30 s for incoming hazards, and swept collision checks. Only physically reachable objects enter those local rollouts; all reports remain in tracking and mapping. |
| `TerminalApproach` | Brakes toward a 100 m standoff, selects a clear side of the 49 m arrival sphere, and surveys around the ship if the observed direct approach is blocked. |
| `velocity_command`, `FlightAttitude` | Velocity control each tick, coasting bands and nose following the measured flight path with stable roll during vertical climbs. |
| `camera_viewer.py` | Separate optional OpenCV process displaying actual frames, detections and telemetry. It sends no flight commands. |

Cruise requests the **25 m/s ceiling**. Translation commands now allow the final
scenario's **0.5 m/s² command limit**, additionally clamped to the live vehicle's
limit. These are requested values: the official finite-thrust jet allocator can
deliver substantially less, especially during combined translation and turning.
Increasing a command limit does not increase jet thrust.

Station braking is independent of the short collision forecast and uses
0.065 m/s² with an additional 130 m allowance. For example, stopping from 25 m/s
requires approximately 4,808 m before that allowance. The starting ship distance
is only about 1,306 m, so this scene cannot sustain 25 m/s and make a safe stop
under the controller's conservative braking model.

The final approach is capped at 2 m/s, with decreasing speed toward the goal and
a separate radial limit approaching 49 m. Near-station forecasts use 6 seconds.
The station speed guard runs **after** local candidate selection, so an avoidance
choice cannot override it. If no terminal trajectory is safe, the controller
stops or retreats instead of charging through the ship. Fresh depth must cover
the station bearing; otherwise `WAIT_SHIP_CAMERA` holds the approach and turns
the stopped rover toward the ship so its limited gimbals can reacquire it. These are
collision precautions, not proof that unseen hull geometry is safe.

## Tests and evidence

The unit/integration suite passes **56 tests**. It covers delayed/noisy moving
radar reports, identity changes after sorting, camera-only obstacles, API errors,
actual runtime frame conversions and command limits, GUI transport/rendering,
nose/roll behavior, global corridor clearance, station braking, an alternate
arrival side, stale or misdirected station depth, and body turning to recover
camera coverage while holding position.

```powershell
python -B -m unittest discover -s tests -v
```

`tests/benchmark_final_submission.py` additionally reads the installed public
final scene's 500 debris positions, initial poses and official 38-jet geometry.
The controller receives only synthetic noisy cycling radar and delayed rendered
camera observations. Scenario ground truth is confined to the test harness.
The model includes a deliberately obstructing ellipsoid **proxy hull**, simplified
gravity and an approximate allocator. It does not reproduce the actual Unreal
collision mesh or native physics. It continues for 30 s after arrival to check
holding behavior, and withholds a zero-damage score estimate after modeled contact
or failed arrival. JSON results include source/scenario hashes.

The previous committed controller (`11d4759`) failed to arrive within 500 s in
the same seed-100 offline setup, ending about 134 m from the ship after using
306.97 kg fuel. That historical result was recorded before the harness added
post-arrival diagnostics; it is not an official simulation score.

Current 500-object model results are saved in
`tests/final_submission_results.json` and
`tests/final_submission_seed101_results.json`:

| Noise seed | Arrival time | Fuel at arrival | Peak speed | Speed entering 50 m | Modeled contact | Hold after arrival |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| 100 | 363.5 s | 202.28 kg | 9.09 m/s | 0.27 m/s | None | 30 s |
| 101 | 382.5 s | 211.74 kg | 9.30 m/s | 0.32 m/s | None | 30 s |

These results establish safe arrival in this limited model, **not a proven live
speed or score improvement**. End-to-end average progress is only about 3.3–3.5 m/s
because braking, scanning and the obstructed ship approach take substantial time.
The model's zero-damage score estimates remain negative (-186.69 and -200.95);
fuel consumption and terminal duration still need optimization using real runs.
Do not present these estimates as competition scores or as evidence of optimality.

Reproduce the current test with NumPy/SciPy (available in the installed simulator
Python), replacing paths as needed:

```powershell
python -B tests/benchmark_final_submission.py --sim-config "path\to\STU4-Sim\Sims\FINAL_SUBMISSION.simconf" --jet-config "path\to\STU4-Sim\Data\SpaceBuggyJetConfigNoMains.csv"
```

Use `--seed 101 --output tests/final_submission_seed101_results.json` for the
second noise/velocity perturbation. This is development evidence, not native
simulator validation or a claim of maximum achievable competition score.

## Deploy and verify

Copy `Controller.py` and `camera_viewer.py` into the installed local
`STU4-Solution` repository and run **FINAL_SUBMISSION.simconf**. Keep the official
scenario unchanged. The GUI and controller log should identify **FINAL_NAV_500**;
seeing `FUSION25` means an older installed controller is running.

The log adds `stopDistance` and `routeNodes`. Expect `GUIDE`/`AVOID` during cruise,
then `BRAKE_SHIP`, possibly `SURVEY_SHIP`, `FINAL_CREEP`, and `HOLD`. Investigate a
persistent `WAIT_SHIP_CAMERA`, `SHIP_BLOCKED` or rising frame rejection count using
the actual RGB/depth panels and `cameraFault` field. An initially steep climb is
correct for this scenario; `NOSE_FOLLOW` aims the front along that flight path.

Record the complete live scoring statistics and matching controller/camera logs.
Check both damage **at success** and whether position remains stable afterward.
The real hull shape, depth calibration, allocator timing and achieved score still
require native simulator verification; an offline proxy cannot certify them.
