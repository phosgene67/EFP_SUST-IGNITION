# Ground-aligned route basis: isolated Task 2

The latest annotated image proposes three legs: an early sideways exit, a climb
outside the dense cloud and a return toward the station. That drawing is a route
hypothesis, not a measured 3D debris boundary. It contains two bends. The existing
one-bend candidate family cannot reproduce all three legs.

The agreed sequence tests one change at a time and stops/reports after each.
This stage implements **Task 2 only**, behind `EFP_GROUND_LATERAL=1`, default off.
It requires Dogleg mode. The user's existing EFP_DOGLEG default-on setting is
preserved. Absolute/early waypoints and a second bend are not added here.

## Verified frames and ascent

Navigation vectors are expressed in the Phobos body-fixed frame. SetCommand
still receives shuttle body-frame acceleration and angular acceleration through
the unchanged DCM conversion. The image's screen-right direction is not a
hard-coded world or body direction.

Source inspection verifies `launch.up = launch.origin / norm(launch.origin)`.
The installed starting position has norm 8,265.895 m and radial up approximately
`[-0.078760, 0.011517, -0.996827]`. Its dot product with launch.origin is positive:
it points away from Phobos's center. Native log 261119_065413_FINAL_SUBMISSION__46a
reports radial launch height 0.0, 0.1, 2.1, 2.8 m at 0.3, 5.3, 10.3, 21.4 s.
Those logged measurements are consistent with outward radial ascent.

This defines a radial tangent plane, **not a camera-measured local terrain
normal**. Crater slopes and terrain clearance remain native verification items.
The software does not infer them from this screenshot.

When the flag is on and launch.up is usable, the generator sets
`side = normalize(cross(forward, up))`, then `tangent = cross(forward, side)`.
Side is perpendicular to both forward and radial up. Missing, invalid, zero or
parallel up retains the old body-derived basis. Candidate count, speed grid,
waypoint fractions, offsets, clearance, finite-burn solver, attitude law, camera
policy and score weights are unchanged. Aligning an offset axis does not force
a horizontal first leg or guarantee a safe lane.

## Spacecraft and camera review

**Verified published configuration/model:** 38 jets, approximately 163.4 N each.
Force is opposite each nozzle normal; their positions also determine torque.
Dominant force-direction counts are +X: 4, -X: 6, +Y: 6, -Y: 6, +Z: 8, -Z: 8. These counts
do not specify usable torque-balanced acceleration. The official selector is
unchanged; no individual jets are commanded.

Canonical pure-axis responses in the published reference-mass allocator model:

| Body command direction | Modeled acceleration |
| --- | ---: |
| +X | 0.17456 m/s² |
| -X | 0.09656 m/s² |
| +Y | 0.14318 m/s² |
| -Y | 0.09644 m/s² |
| +Z | 0.09581 m/s² |
| -Z | 0.11690 m/s² |

These canonical responses are not proven universal maxima over every mixed-axis
command. Eight m/s from rest takes 45.83 s at the canonical +X response, ignoring
gravity. Even the optimistic force-only +X bound, ignoring torque cancellation,
needs about 41.47 s. Thus an instant 8 m/s launch is not a feasible assumption.
Pointing the strong axis toward the burn also costs time/fuel; activating more
jets can spend fuel on opposing force or torque instead of useful translation.

**Modeled score constraints:** at 180 s and zero damage, 55 points requires fuel
at most 46.25 kg; 60 points requires at most 42.50 kg. Clean sequential translation
costs about 4.322 kg per m/s of body-axis impulse. Turns, alignment and corrections
must fit the remaining budget. An unpowered coast is fuel-free, but gravity
changes velocity; constant 8 m/s is not automatically free.

As a conditional example, an 8 m/s right-angle corner needs 11.31 m/s of ideal
vector delta-v, about 48.9 kg at the clean-impulse coefficient. If the initial
and final velocities lie along held X/Z body axes, their sequential L1 change
is 16 m/s, about 69.2 kg before other overhead. This is not a measurement of
the screenshot's 3D angles. It explains why a slow early lateral adjustment
followed by the main climb burn, or a gentle diagonal route, can be cheaper
than demanding 8 m/s on every leg.

The top and bottom cameras are body-mounted pan/tilt sensors, not fixed world-up
views. The helper permits yaw±89°, top pitch[-50°,0°], bottom pitch[0°,50°].
The current asynchronous capture requests96×64 images with an80° FOV parameter,
alternating depth/RGB. A finite aimed image cannot certify the whole future path
or absence of debris outside its view. This stage leaves the existing camera
policy unchanged: Dogleg uses cameras near the final ship approach.

## New measured native evidence, separate from the experiment

The recent log 261119_065413_FINAL_SUBMISSION__46a matches the Task 1 deployed
source hash c347e27a…. Its official result was score -54.1, T357.75 s, F103.26 kg,
D0 J. It did not enable ground-lateral. This is one native run, not a matched
native median. Native lane-search times had median 3.846 s and maximum 9.688 s.

The search runs synchronously; while it computes, command updates can be
delayed. That is a measured latency issue, not proof of exactly how much fuel
it caused. No IGNITION_STATE trace was found in that run. The fixed-step offline
harness does not charge wall-clock search time, so its scores cannot validate
this native behavior. Blocking search must be addressed before competition use.

Frame/actuator/native evidence: tests/ground_lateral_frame_audit.json.

## Completed matched validation

193 tests pass, including radial ascent, orthogonal ground basis, degeneracy
fallback, exact old candidates with the flag off, and runtime reset wiring.
The frozen experiment source is tests/score_sweeps/ground_lateral_controller.py.
Seeds 100–119 ran with `--experiment dogleg --experiment ground-lateral` and
the same legacy radar RNG, 60 Hz, scene, allocator proxy, synthetic camera and
2.5 m ground-plane settings as the 20 Task 1 baseline seeds.

**Modeled flight results, not native scores:**

| Quantity | Dogleg baseline | Ground lateral |
| --- | ---: | ---: |
| Median score | 27.890 | 25.980 |
| Score p10–p90 | 8.882–37.932 | 13.216–36.161 |
| Score range | -0.44–39.77 | -4.32–43.98 |
| Median entry time | 245.845 s | 243.475 s |
| Median entry fuel | 54.890 kg | 53.750 kg |
| Arrivals / modeled contacts | 20 / 0 | 20 / 0 |
| Runs hitting no_safe_lane | 20 / 20 | 20 / 20 |
| Total failed searches / searches | 400 / 467 | 396 / 469 |

Difference of score medians: **-1.910 points**. Rounding-adjusted paired-bootstrap
95% interval: **[-8.270,+7.125]**. This is inconclusive and rejected for promotion.
Mean score rose 0.710 points, but that does not establish the requested median
gain. Paired median time delta was +0.490 s; paired median fuel delta -0.205 kg.
Paired medians need not equal the differences of the separate cohort medians.
Each cohort executed six bends. Neither had post-entry fuel use.

**Modeled mean phase-fuel deltas** (ground minus baseline, additive):

| Phase | Change |
| --- | ---: |
| Alignment | +0.110 kg |
| Dogleg initial burns | -0.146 kg |
| Dogleg bends | -0.333 kg |
| Existing cruise/fallback modes | -0.426 kg |
| Capture and observation | +0.077 kg |
| Camera wait | +0.646 kg |
| Dogleg coasts | -0.037 kg |

Net unrounded phase change was about -0.107 kg/run, versus -0.1065 kg from
rounded entry fields. Savings in some phases were offset by more terminal
camera waiting; these are modeled phase measurements, not native attribution.

**Measured compute under four concurrent benchmarks:** median per-run maximum
planner call was 12.487 s for the Task 1 reference and 12.636 s for the new
cohort; the new maximum was 14.562 s. Concurrent load affects these wall times.
The fixed-step harness excludes them from entry time. The separate native
baseline maximum of 9.688 s independently shows that synchronous search is a
real deployment issue. No native latency or score gain is claimed for this flag.

Full paired results: [tests/score_sweep_ground.json](tests/score_sweep_ground.json).
Source preservation: [tests/ground_lateral_preservation.json](tests/ground_lateral_preservation.json).
Per-seed outputs: tests/score_ground_SEED.json. Reproduce the comparison with:

```powershell
python -B tests/summarize_score_sweep.py --baseline-prefix tests/score_dogleg_diag_ --candidate-prefix tests/score_ground_ --output tests/score_sweep_ground.json
```

**Recommended setting: EFP_GROUND_LATERAL=0.** The user's existing Dogleg
default is preserved; explicitly setting EFP_DOGLEG=0 selects SCORE_MOMENTUM.
No 55+ native score, sub-180 s flight or successful full green route is claimed.
This completes Task 2 only, following the earlier stop/report sequence. Early
absolute offsets and a two-bend route family have not been implemented.
