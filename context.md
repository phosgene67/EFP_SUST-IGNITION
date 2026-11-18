# Escape from Phobos project context

Updated 2026-11-18. Read this file before controller changes and update it after
each tested change. It is a maintained summary of the available chat history,
repository history, guides, archived experiments and local simulation logs.
The original chats and logs remain the sources for details.

## Current user direction

Return to SCORE_MOMENTUM at commit `4a8507e`. The user reports this version
can reach about 50 points in their simulation tests. Improve prediction,
path choice and fuel/time balance toward 65+ points. Restore a reliable baseline
before adopting changes. The user runs and deploys the competition simulation.

The latest request supersedes the preceding staged fuel-budget design.
That earlier design allowed 20 kg before 600 m remaining, a conditional 3 kg
advance, then 30 kg afterward under a 50 kg total cap. Its early coast spent too
much time. The active Momentum controller prices fuel through the score again.

Keep the connection prefix and command helpers in Controller.py intact.
Keep the official STU4-Sim repository intact. No push or deployment was performed
in the November 18 restoration task.

## Runtime and architecture

- `Controller.py` connects to Space Teams PRO and contains the live algorithms.
  The simulator executes its bottom flight loop. AST-based test loading excludes
  the connection prefix and flight loop.
- `BasicController.py` is the starter/reference implementation.
  `UserAPI.py` contains provided API examples and bindings.
- `PoseHistory` reconstructs the vehicle pose at delayed sensor timestamps.
  `CameraManager` schedules top/bottom RGB and depth requests.
  `CameraPerception` converts depth in centimetres to occupied world volumes
  and associates RGB bearings with radar tracks.
- Radar positions are relative to the rover. The velocity measurement is radial.
  Multiple associated sweeps estimate transverse motion. A sorted report index
  cannot identify a debris object.
- `ShootingNavigator` compares finite burns along body axes, their execution
  order, arrival times, destination azimuths and lateral detours. It accounts for
  real burn duration, available directional acceleration and fuel per impulse.
- Near-term moving-debris checks cover the next 20 seconds and replan every
  0.5 seconds. Distant uncertain obstacles contribute a soft cost. The fallback
  `RollingChunks` planner retains local avoidance and route lookahead.
- `CoastDynamics` learns local acceleration residuals during unpowered flight.
  Momentum offers an explicit zero-thrust successful pass, retains useful burns
  and discourages reversing forward momentum to follow a changing optimum.
- `GuardedLaunch` hands off after approximately 2.5 m of ascent with sensor
  checks. `FreeAttitude` avoids forced nose pointing and damps unwanted spin.
- `CaptureApproach` plans a camera-checked tangent pass near 47 m. The flight
  loop checks official ship-frame distance between planning updates, marks entry
  within 50 m, freezes entry fuel statistics and starts post-entry braking.
- `camera_viewer.py` is the separate OpenCV viewer. Both it and Controller.py
  must be present in the installed solution folder. The viewer shows RGB/depth,
  detections, flight statistics and entry fuel.
- Files under `tests` provide unit regressions, a simulator API double and offline
  flight models. They are development tools. They are never imported by the
  simulator's live flight controller.

The active configuration is `FINAL_SUBMISSION.simconf`. The local solution
folder must be named `STU4-Solution` because the official configuration loads
`Local/Repos/STU4-Solution/Controller.py`. GitHub remains EFP_SUST-IGNITION.

## Success and scoring

Official success occurs when the vehicle reaches the ship's safe area while
below the damage limit. The scoring clock stops at entry. It does not require
stopping or docking. A collision after success can increase the displayed total
fuel/damage after the official score has already been recorded.

The installed final configuration uses nominal time 120 s, nominal fuel 75 kg
and maximum damage 209200 J.

```text
score = 100 * (0.5 + 120 / arrival_seconds
               - entry_fuel_kg / 75 - 2 * entry_damage_J / 209200)
```

Scores can be negative and can exceed 100. A nominal successful run scores 50.
One kg costs about 1.333 points and 1000 J costs about 0.956 points. A collision
can erase a large fuel saving. With zero damage, 65 points requires arrival
within 194.59 s at 35 kg or 175.61 s at 40 kg. A faster requested speed alone
cannot guarantee that score because the jet allocator saturates and burns fuel.
Small continuous requests can still fire full normalized jet duties.

## Development history

1. Qualification used COMPETITION_SIM, radar avoidance and conservative ship
   capture. Reported successes improved from 249.9 s / 145.4 kg / -95.8 to
   195.98 s / 117.34 kg / -45.2 with zero damage in the latter run.
   `QUALIFICATION.md` explains that historical code and test architecture.
2. The final round added RGB/depth fusion, the OpenCV interface and higher speed
   requests. Logs exposed early excessive fuel, slow debris traversal and a
   station impact. `FINAL_DESIGN.md` and `FINAL_SUBMISSION.md` record these steps.
3. `CHUNK_NAVIGATION.md` and `OPEN_LANE_9.md` introduced predictive local chunks,
   advance planning and less congested routes. Stopping outside the 50 m scoring
   boundary was an identified failure. Actual official-frame entry is essential.
4. `SCORE_SHOOT.md` introduced finite burns.
   `SCORE_ECO.md` developed time/fuel scoring.
   `FUEL_AUDIT.md` compared attitude modes. Forced nose-forward pointing and
   leveling consumed fuel, so the selected default became free attitude.
5. SCORE_MOMENTUM (`4a8507e`) preserved useful inertia and reduced unnecessary
   corrections. `SCORE_MOMENTUM.md` records four offline seeds with mean score
   40.93, mean entry fuel 41.60 kg and no modeled contact. The user subsequently
   reported native scores near 50.
6. SCORE_COAST (`c89f260`) showed only a small modeled gain and worse live
   results. SCORE_SERVO (`0c3f06d`) added asynchronous math and further economy
   control. SCORE_CORRIDOR (`fdf3810`) tried bent routes. Their live regressions
   drove the requested restoration. Existing native logs carrying profile names
   show SCORE_COAST scores -27.0 to 26.8, SCORE_SERVO -18.8 to 31.7 and
   SCORE_CORRIDOR -27.7 to 32.1. These are ranges of available local logs,
   with varying conditions and no matched native baseline.
7. STAGED_COAST (`5267fd2`) enforced the staged fuel allowance. Archived matched
   seed 101 used 40.98 kg in 446.52 s for 22.23 points, versus Momentum's
   43.42 kg in 252.95 s for 39.55 points. Those runs used modeled drift and
   0.25 s planning latency. See `tests/staged_benchmarks.json`.
8. November 18 restores Momentum. A gravity-gradient endpoint correction was
   tested on seeds 100–103. It increased mean entry fuel to about 53.68 kg and
   reduced mean score to 24.76, so it was rejected. A continuous distant-crossing
   cost also scored below the original on average and was rejected.
   Keep those observations when considering similar changes again.

## November 18 selected implementation and verification

The original Momentum burn schedules, coast prediction, route costs, 10 m/s
score-selected cruise ceiling and capture logic are restored. Radar processing
now reuses unchanged report transforms and fitted uncertainty until the next
sweep. New measurements create fresh track state and refresh the fit. Association
innovation is computed once. Missing pose history retains its extra margin.
These changes aim to reduce flight-loop planning stalls while preserving
Momentum's numerical decisions.

Later-profile tests were rolled back with their retired runtime features.
Historical asynchronous-worker regressions use a test-only copy of the worker
in `tests/planning_doubles.py`, which also supports archived latency comparisons.
The camera-reach stress script loads its original SCORE_COAST source revision.

The exact restored baseline was rerun under all 500 published final-scene debris,
60 Hz control, noisy cycling radar, synthetic cameras, a 2.5 m ground plane and
the published jet geometry. Seeds 100–103 reproduced scores 44.93, 43.03,
40.03 and 35.72. These are zero-damage estimates from an offline model with proxy
station geometry. They do not establish a native 65-point result.

Final verification passed all 127 unit and runtime-adapter tests. The selected
controller exactly reproduced the baseline's recorded entry time, fuel, speed,
contact status and score on all four seeds. Mean score is 40.93, mean entry fuel
41.60 kg and mean arrival time 258.75 s.

A separate matched radar replay compared 80 guidance updates with 500 reports
per update. Positions, velocities, radii, fitted uncertainty and confidence
flags were identical. Median radar processing fell from 74.17 ms to 30.05 ms,
about 59.5 percent. This measures the radar portion of planning on this computer.
Actual scheduling and native score effects need simulation verification.
The protected connection prefix also matches the pre-task HEAD.

All 16 flight records, source/scenario hashes, settings, rejected variant results
and radar timing are consolidated in `tests/momentum_restoration_benchmarks.json`.
Reproduce the radar comparison with `python -B tests/benchmark_momentum_radar.py`.
The log profile remains SCORE_MOMENTUM. The normalized source hash is
`1cc462d7734c38f8dc5e78a8b89008eb355de0c6b36c337be50a5a929e00e58e`.

## Verification and future comparisons

Run tests with the simulator Python environment or another Python containing
NumPy. The offline allocator benchmarks additionally require SciPy.

```powershell
python -B -m unittest discover -s tests
```

`tests/benchmark_final_submission.py` accepts the installed official simconf and
jet CSV through `--sim-config` and `--jet-config`. Use `--ground-plane 2.5`,
matched seeds, and `--baseline-ref 4a8507e` for a baseline. Record drift,
planning-delay mode, allocator solver and source hashes. Preserve the score,
entry fuel/time and post-entry statistics as distinct measurements.

Future work should compare changes against this Momentum version and against the
user's official success logs. Avoid treating a predicted score as achieved.
Native meshes, Mars/frame dynamics, sensor rendering and scheduling still need
the user's simulation verification.
