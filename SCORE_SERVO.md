# Nonblocking control and fuel planning: SCORE_SERVO

This build addresses control timing, incomplete detour execution and unnecessary
sensor inflation. It targets less than 40 kg **at first entry into the success
zone**. A universal under-40 kg result and a score of 60--70 have not been
demonstrated in the native simulator. Post-entry braking uses additional fuel.

## What the latest native logs establish

Run `261117_191504_FINAL_SUBMISSION__e74` used the exact previous controller
fingerprint `fcef22ba...` and reported **19.3 points, 297.08 s, 53.32 kg and 0 J**.
Its sampled ledger attributed about 3.31 kg to launch, 44.16 kg to cruise impulse
burns, 5.09 kg to phase transitions and the remainder to smaller segments.
Other recent runs were stopped without an official success score; one had
already consumed 81.79 kg by 229.9 s. These are not successful-run comparisons.
Extracted evidence is in `tests/servo_native_baseline.json`.

Planning in those logs commonly took hundreds of milliseconds. Previously the
same thread performed planning and velocity control. During a calculation the
actuator could continue using the last command. This is a verified execution
structure, not proof that timing alone explains every extra kilogram. An
isolated synthetic timing comparison did not establish a fuel saving from
threading alone; route changes also affect fuel and time.

## Execution architecture

`PlanningWorker` runs one pure-math job on a private copy of the planner and
camera observations. It has no simulator handles and makes no simulator API
calls. The main thread reads state, processes camera messages and executes the
velocity servo while that job runs. There is no queue of accumulating plans.

The main thread adopts a completed result only if it is recent and belongs to
the current simulation epoch and mission state. It preserves newer coast-model
observations, fuel calibration and pose. An old result cannot undo arrival or
damage failure. Plans more than one simulation second old are discarded. If
fresh planning is lost, the pre-entry watchdog prevents an old burn from
continuing indefinitely. Existing radar-loss control remains available.

The loop uses a wall-clock deadline for its target period instead of adding
an unconditional full-period sleep after every iteration's work. This targets
60 Hz; actual scheduling must be checked in the native log. `planMs`, `copyMs`,
`controlStepMs`, `maxControlStepMs`, `planBusy` and `planFault` expose computation,
snapshot cost, observed loop gaps and planning failures.

The finite-burn executor now accepts corrections down to 0.004 m/s. The previous
0.02 m/s threshold could discard a small correction that the trajectory forecast
assumed would actually occur. With nonblocking execution, the servo can cut such
a pulse using fresh velocity instead of holding it throughout a planning call.
Native jet timing and API latency still require live validation.

## Guidance and sensor changes

- A detour's scheduled return impulse becomes a current candidate when due.
  It is no longer simply excluded and replaced by another plan promising a
  future return. Sensor checks can still select a different safe maneuver.
- After a substantial burn, a 3.5 s quiet observation candidate gives the drift
  estimator uninterrupted velocity samples. It must pass the existing collision
  and damage-risk checks. Pending detour return stages are preserved, not erased.
- A camera surface uniquely associated with a mature radar track uses that
  track's forecast uncertainty. Every occupied volume remains present. Unknown
  and ambiguous surfaces retain their former uncertainty; station guards remain.
- Depth surfaces below the launch reference plane do not borrow the velocity of
  a passing radar object merely because their inflated bounds overlap. Their
  camera volumes remain occupied. Moving radar tracks are retained separately.
  This uses a local terrain assumption; it is not a general terrain classifier.
- Midcourse and capture guidance check camera reach down to 52 m, close to the
  50 m success boundary. Checking only 150 m and 80 m could leave the ship behind
  a gimbal later and provoke a costly camera-wait stop. The 52 m constraint is
  introduced within 600 m, avoiding a premature route restriction at launch.
  Fresh measured depth is still required; predicted reach does not imply an
  unoccluded view or establish native camera calibration.

## Fuel/time tradeoff

The normal finite-burn search uses a **35 kg soft planning target**, leaving
room below 40 kg for capture and unplanned avoidance. Its extra decision cost is:

```text
0.75 * (max(spent + planned - 35, 0)^2 - max(spent - 35, 0)^2)
```

This prices additional overruns, not historical fuel a second time. Coasting
adds no budget penalty. It is not a hard fuel cutoff, and does not prevent an
emergency maneuver or guarantee total usage. The displayed unofficial plan
forecast includes decision penalties; the official result uses only Scoring.py.
Nose pointing and horizon leveling remain disabled by default. Experimental
one-time turns and angular-allocation variants were not selected for deployment.

For the installed final configuration, the actual score remains:

```text
100 * (0.5 + 120 / arrival_time - entry_fuel / 75 - 2 * entry_damage / 209200)
```

At zero damage and 38 kg, **60 points requires at most 197.80 s**, and **70 points
requires at most 169.81 s**. Saving fuel while extending the trip to 300 s does
not meet either score target. The target has not been declared achieved merely
because a modeled flight uses less fuel.

## Validation and limits

All **142 regression tests pass** using the installed simulator Python runtime.
The suite checks the runtime adapter with a fake API, a real background
planning thread, servo cutoff while planning is pending, stale/failed jobs,
clock reset, preservation of live observations, mission-transition priority,
watchdog cutoff, detour return execution, small impulse execution, camera reach
through the capture handoff, and association uncertainty without deleting bounds.

```powershell
python -m unittest discover -s tests -v
```

Final modeled runs are recorded in `tests/servo_final100.json` through `103.json`.
They use all 500 debris objects, 60 Hz control, synthetic cameras, a 2.5 m ground
plane, a declared 0.25 s planning delay and an unmodeled acceleration bias of
`[0.0024, -0.0008, 0.0016] m/s²`. The bias stresses the drift observer; it is not
a calibrated reproduction of the native engine. `--planning-mode async` keeps
the old servo operating until the delayed result is adopted; `blocking` instead
holds the old actuator command during the declared delay. This isolates timing,
not all effects of native API or rendering delays.

| Seed | Allocator solver | Entry time | Entry fuel | Zero-damage score estimate |
| --- | --- | ---: | ---: | ---: |
| 100 | Native revised simplex | 307.33 s | 38.99 kg | 37.06 |
| 101 | HiGHS | 285.55 s | 37.80 kg | 41.62 |
| 102 | HiGHS | 293.95 s | 41.18 kg | 35.91 |
| 103 | HiGHS | 286.02 s | 42.22 kg | 35.67 |

All four entered the modeled success zone without proxy contacts. Only **two of
four** met the fuel target; **none** met the score target. These are modeled
estimates, not official simulation results. The reported `distance_m` in the
JSON is the final position after the additional braking test, not entry distance.
The archived results match delivered controller byte fingerprint
`876e115c86a67902140c728592527c63f68963ac4b468c34e0ec578854ccb09a`.

The matched seed-101 old-controller reference used 45.81 kg in 255.77 s, scoring
35.83 in the same HiGHS model. The revision saved 8.01 kg but took 29.78 s longer,
raising that modeled score by 5.79 points. Seed 100's old reference used 44.04 kg
and estimated 35.70 points, but its different allocator solver prevents treating
that comparison as an isolated controller improvement. Neither comparison
attributes all savings to threading or establishes performance in the engine.

Seed 100's fuel audit attributes 3.52 kg to launch, 33.61 kg to cruise impulse
burns and 1.86 kg to escape/guidance, with zero attitude-command fuel. Its
post-entry braking consumed another 14.60 kg, excluded from the entry score.
Cruise translation remains the main fuel expense; removing nose pointing again
would not save fuel because it is already inactive. The remaining score gap
requires a faster, shorter trajectory at comparable entry fuel, not simply
raising the requested speed or imposing a lower fuel target.

Reference runs use commit `c89f260` in the blocking-delay model. Their files are
`tests/servo_reference_blocking100.json` and `101.json`. Read each result's solver,
latency, drift and source fingerprints before comparing it. Sphere debris,
ellipsoid station geometry and simplified gravity cannot validate Unreal contact
damage, actual scheduling or camera projection. Modeled contact suppresses the
zero-damage score estimate. Development experiments, including rejected faster
or lower-fuel alternatives, are retained in `tests/servo_tuning_history.json`.

## Deployment

Copy `Controller.py` alongside the existing `camera_viewer.py` in the installed
local `STU4-Solution` folder. No extra live module, SciPy installation or official
simulation change is required. The worker uses Python's standard library.
Leave `EFP_NOSE_FIRST` and `EFP_AXIS_THRUST` unset or `0`.

Confirm **SCORE_SERVO** and the source fingerprint in the next native log.
Compare **official entry fuel**, elapsed time and damage over fresh runs, and
inspect the new control-gap and plan-fault fields. The file connection prefix
and official `STU4-Sim` files are preserved.
