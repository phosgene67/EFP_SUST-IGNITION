> **Historical document — archived 21 November 2026.**
> This describes an earlier revision or experiment. For the delivered implementation, see
> [Final Submission Algorithm](../Final%20Submission%20Algorithm.md). Archived commands and flags refer to their historical revision.
> Original location: `SCORE_SHOOT.md`.

# Finite-burn controller: SCORE_SHOOT

Historical version. The active profile is now `SCORE_MOMENTUM`; see
[SCORE_MOMENTUM.md](SCORE_MOMENTUM.md) for current behavior and measurements.

The active classes are `ShootingNavigator` and `FreeAttitude` in `Controller.py`.
This is a testable candidate, not a verified competition result. The +50/+70 score
targets have not yet been demonstrated by this implementation. Do not confuse
`projectedScore` in the controller log with the official score from `Scoring.py`.

## What the native logs actually show

The local log `261116_050741_FINAL_SUBMISSION__794/FINAL_SUBMISSION.log`
reports **354.9 s, 105.2 kg, 0 J, score -56.5** at 49.91 m. This is the result
reported by the user. A later run, `261116_051602_FINAL_SUBMISSION__9af`, reports
462.34 s, 111.3 kg and zero damage. These logs do not identify the controller
source hash, so they are observations, not a controlled A/B comparison.

In that later run, the launch phase used about **9.08 kg**. At 15.1 s fuel was
9.14 kg; it reached 20.11 kg at 31.5 s during early avoidance. Thus the first
20 kg includes acceleration and avoidance after liftoff. At arrival, sampled
native accounting attributed 28.81 kg to guidance, 23.25 kg to avoidance,
15.01 kg to trimming and 21.47 kg to intervals crossing phase transitions.
The transition bucket cannot be uniquely assigned to either adjacent phase.

The older offline model omitted ground imagery and was too optimistic. The
benchmark now optionally renders a synthetic plane below launch. With that
plane, the previous controller at Git `578fd71`, seed 100, used **106.37 kg**
in **435.37 s**, estimated score **-64.27**. This reproduces the scale of the
problem; it does not validate the real terrain or native physics.

## Why the controller changed

The installed final scenario uses the official `SpaceBuggyJetConfigNoMains.csv`.
The allocator solves for jet on-times and normalizes their largest value. A
small continuous acceleration request can therefore produce a substantial jet
pulse. Repeatedly chasing velocity and switching between mixed body directions
can spend considerable fuel.

The reference mass is 3388.7 kg and the configured specific impulse is 80 s.
For a pure body-axis translation, the modeled fuel cost is about
`3388.7 / (80 * 9.8) = 4.322 kg per m/s` of commanded velocity change. Success
does not require stopping before entering the 50 m safe area, so pre-entry
braking wastes both time and fuel when a clear pass exists.

## Flight sequence and code architecture

1. **Guarded launch.** `GuardedLaunch` lifts roughly 3.5 m while sensor checks
   remain active. When a body axis is sufficiently close to vertical, the new
   launch executor uses that axis rather than a mixed acceleration command.
   A vehicle already inside an inflated sensor volume can leave it while
   separation increases; entering another obstacle volume is still rejected.
2. **Track moving debris.** The radar tracker in `QualificationPlanner.obstacles`
   associates repeated reports, compensates for own-vehicle motion and fits
   several sweeps. `ShootingNavigator` enables gravity compensation for both
   the fitted velocity and stale-report extrapolation. Fit quality and report
   age determine forecast uncertainty. Weak tracks retain larger margins.
3. **Fuse images.** `CameraManager`, `PoseHistory` and `CameraPerception` handle
   asynchronous RGB/depth captures and acquisition-time poses. Depth supplies
   occupied geometry; RGB supplies contrast regions and radar associations,
   not invented metric depth. A copied radar sphere associated with an RGB
   bearing is counted once. Real depth surfaces remain in the obstacle set.
   Static ground and station surfaces are not extrapolated as moving debris.
4. **Choose a finite burn.** `axis_schedule` evaluates arrival times, six body-axis
   firing orders and 16 possible sides of a 47 m tangent pass near the station.
   It solves finite-duration burns, including distance traveled while thrusting,
   rather than treating a velocity change as instantaneous. Candidate scores
   include elapsed time, spent fuel, future body-axis impulse and damage.
   Additional lateral alternatives include a priced return maneuver.
5. **Check the next chunk.** `scheduled_positions` predicts the actual sequential
   burns and coasting. Swept segments check moving obstacles over the next
   **20 seconds**, including gravity and a curvature margin. Farther geometry
   influences ranking without forcing a minute-long uncertain corridor to be
   completely empty. Planning repeats at 2 Hz while execution continues;
   reaching a chunk boundary never commands a stop.
6. **Execute and coast.** At the flight-loop rate, only the current body axis
   fires. Completed axes advance immediately, and overshoot cannot reverse an
   already completed burn. A 0.02 m/s cruise tolerance suppresses tiny repeated
   corrections; the core clearance margin covers the corresponding sub-metre
   deviation over a chunk. Safe existing plans receive a small retention
   allowance to reduce switching. Pure-axis acceleration estimates adapt from
   measured motion. Coast intervals send zero translation command.
7. **Enter safely.** Below 280 m the camera-guarded `EconomyNavigator` and
   `CaptureApproach` handle the pass and lateral course trims. The vehicle must
   enter the 50 m sphere, rather than stop at 50 m. Braking happens after entry.
   If the ship cannot be observed or no pass is clear, conservative observation
   or the slower `TerminalApproach` fallback takes priority over the score.

`EconomyNavigator`, `FinalNavigator` and `RollingChunks` remain the launch,
near-ship and emergency fallback. `BasicController.py` and `UserAPI.py` remain
reference examples; tests load the algorithm without connecting to the simulator.
The protected connection prefix in `Controller.py` remains unchanged.

## Risk and sensor limits

Mature radar tracks use physical/core clearance plus an uncertainty cost.
`encounter_risk` estimates a Gaussian tail and uses twice relative kinetic
energy as a conservative **proxy**, not the official contact damage law.
Candidates still need positive core clearance. Additional gates reject large
estimated damage or potentially fatal encounters. Correlated report errors,
association mistakes and unknown meshes limit these estimates; they are not
certified collision probabilities. Small damage is priced, never assumed free.

The software depth-processing limit is now 400 m, and the viewer uses the same
colour scale. This does not assert that every real surface yields usable depth
at 400 m. The no-radar fallback retains its conservative 220 m envelope.
Camera field of view, gimbal limits, freshness and capture poses still apply.

`FreeAttitude` removes visually motivated nose and horizon control and only
damps unwanted spin. The comparison setting `EFP_NOSE_FIRST=1` selects the older
nose-following economy path. It is not the recommended score configuration.
An isolated one-time alignment experiment also used more fuel and scored worse
than the free-attitude candidate; it was not adopted.

## Score budget

The installed final scoring system specifies nominal time 120 s, nominal fuel
75 kg, damage threshold 209200 J and success radius 50 m. For a successful run:

```text
Score = 100 * (0.5 + 120/T - F/75 - 2*D/209200)
```

For a target score `S`, the fuel budget is:

```text
F <= 75 * (0.5 + 120/T - S/100 - 2*D/209200)
```

| Arrival time | Maximum fuel for +50, zero damage | Maximum fuel for +70, zero damage |
| --- | ---: | ---: |
| 180 s | 50.00 kg | 35.00 kg |
| 200 s | 45.00 kg | 30.00 kg |
| 250 s | 36.00 kg | 21.00 kg |
| 300 s | 30.00 kg | 15.00 kg |

At 250 s, saving 1 kg is worth 1.333 points; saving 10 s is worth about 2 points.
Damage of 1000 J costs about 0.956 points. A fixed 10 or 25 m/s speed target
therefore does not guarantee a better result. The planner chooses the best
predicted time/fuel combination under sensed clearance, with a 10 m/s ceiling.
The score estimates do not prove the requested +70 is physically achievable
with this vehicle and starting configuration.

## Validation and deployment

Four noise seeds were run against all 500 public final-scene debris objects,
with ground imagery enabled 2.5 m below the initial vehicle centre:

| Noise seed | Time at entry | Fuel at entry | Zero-damage score estimate |
| --- | ---: | ---: | ---: |
| 100 | 253.75 s | 42.38 kg | +40.78 |
| 101 | 253.67 s | 40.24 kg | +43.65 |
| 102 | 269.65 s | 43.76 kg | +36.16 |
| 103 | 251.30 s | 46.32 kg | +35.99 |

All four reached the safe area without modeled debris or proxy-hull contact,
then braked to a stop. Their minimum debris gaps were 12.48--14.02 m and their
minimum proxy-hull gaps were 20.99--24.17 m. Peak speeds were 6.26--6.78 m/s;
after first reaching 5 m/s, mean cruise speeds were 5.74--6.20 m/s. Concurrent
benchmark runs measured planning p95 around 247--311 ms; this is not a native
real-time guarantee. Results are recorded in `tests/score_shoot_ground*.json`,
with the exact controller and scenario hashes. Failed and superseded tuning
runs are preserved in `tests/shooting_tuning_history.json`, including the old
ground-imagery baseline and rejected alignment experiments.

The seed 100 fuel audit separates the pre-arrival 42.38 kg:

| Segment | Fuel |
| --- | ---: |
| Short launch, 9.5 s | 3.93 kg |
| Initial escape and guidance | 0.94 kg |
| Main finite burns and corrections | 36.34 kg |
| Cruise/pass coasting | 0.00 kg |
| Extra capture observation | 1.17 kg |

No rotation command was needed in that modeled run. Post-arrival braking used
another 22.88 kg; this is recorded separately and excluded from its score.
These runs show substantial fuel savings compared with the old modeled
baseline. They **do not yet meet +50 or +70**, and no new native run has been
claimed. At roughly 254 s, +50 needs about 35.4 kg total fuel with zero damage,
and +70 needs about 20.4 kg. The score target remains a measurable shortfall.

The 109 passing tests cover finite-burn endpoint accuracy under gravity and rotation,
execution between planning ticks, overshoot, sensing, radar tracking, emergency
behavior, station entry and the connection adapter. Run:

```powershell
python -m unittest discover -s tests -v
```

`tests/benchmark_final_submission.py` reads the public scenario only to generate
synthetic radar and camera observations. The flight controller receives sensor
data, not obstacle ground truth. Benchmark version 3 adds optional ground imagery
and checks whether the vehicle centre crosses that plane. It uses simplified
gravity, sphere debris, an ellipsoid station proxy, ideal servo timing and a
different LP solver. It does not reproduce Unreal terrain or vehicle/hull meshes,
native scheduling, or measured contact damage. No estimated score is emitted
when this model detects a collision or fails to arrive.

Copy **Controller.py and camera_viewer.py** to the installed `STU4-Solution`
folder. Check `profile=SCORE_SHOOT`, `noseFirst=False`, `attitude=FREE_ATTITUDE`,
`planMs`, phase fuel buckets and the final official scoring alert. The phase
ledger freezes at arrival; later braking fuel is outside the official result.
The vehicle may park outside the 50 m sphere after a safe tangent pass because
the mission score has already frozen. Leave official STU4-Sim unchanged.
