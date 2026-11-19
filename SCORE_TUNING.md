# Score tuning from the original Momentum controller

The reference is commit `4a8507e`. The current controller restores its
pre-entry guidance and removes the later bypass, staged reserve and fuel ceiling.
All thrust stops after safe-zone entry. Experiments default off.

## Verified scoring

The installed `FINAL_SUBMISSION.simconf`, `Scoring.py` and
`JetSelectionActuator.py` were read directly.

```text
score = 100 * (0.5 + 120 / entry_time_s
               - entry_fuel_kg / 75 - 2 * entry_damage_J / 209200)
```

The sphere radius is 50 m. Damage is checked before success on each scoring
tick. The winning result then freezes. Fuel has no mission-ending limit.
One kg costs 1.3333 points and 1000 J costs 0.9560 points.

At zero damage, a score of 60 requires these maximum times.

| Entry fuel | Maximum time |
| --- | ---: |
| 35 kg | 211.76 s |
| 40 kg | 189.47 s |
| 45 kg | 171.43 s |
| 50 kg | 156.52 s |
| 67 kg | 120.81 s |

The exact score change from a proposed burn is
`12000 * (1/T_new - 1/T_old) - (100/75) * extra_fuel_kg`, before damage.
At 200 s, one kg must save about 4.44 s to pay for itself locally. At 260 s,
it must save about 7.51 s.

At the 3388.7 kg reference mass, Isp 80 s and g0 9.8 m/s², the scalar
fuel cost is 4.32232 kg per m/s of thrust impulse. Pure-axis acceleration
requests reproduce the supplied axis values to rounding, with -Z approximately
0.1169 m/s². These are achieved canonical-command accelerations under the
published LP and duty remap. A universal score ceiling cannot be inferred
without gravity, range, initial velocity and overhead assumptions.
Configuration values, file hashes and allocator calculations are stored in
[the model verification record](tests/score_model_verification.json).

### Where 4.32 comes from, and what 8–9 m/s means

The official actuator's `GetEffectiveImpulseAndFuelUse` adds each jet's
translational impulse magnitude divided by `Isp * g0`. The installed scenario
sets the reference mass to 3388.7 kg. Thus, for a clean translational burn,

```text
fuel = thrust_impulse / (Isp * g0)
     = reference_mass * thrust_delta_v / (80 * 9.8)
     = 4.32232 * thrust_delta_v
```

This is fuel per accumulated thrust impulse, not fuel per second, per metre,
or per measured speed increase. Gravity reduces the speed gained from thrust.
Opposing corrections, rotation and nonparallel simultaneous jets can cost
extra fuel. Eight and nine m/s of clean thrust delta-v would use 34.58 and
38.90 kg before those effects. Coasting requires no thrust fuel.

At 50 kg and zero damage, 60 points requires entry within 156.52 s. The published
initial range minus the sphere radius is about 1255.82 m: this needs roughly
8.02 m/s average along a direct path, including launch and acceleration.
The current planner already considers speeds through 10 m/s. Raising a speed
limit to 8 or 9 cannot fix its slower selected flight by itself.

A separate one-dimensional diagnostic burns to a specified peak speed and then
coasts under inverse-square gravity. These are modeled assumptions, not full
flight trials or attainable guarantees. They omit debris, terrain, camera gates,
launch delays, lateral motion, hull geometry and any cost of axis alignment.

| Assumed thrust axis | Peak speed | Entry time | Fuel | Score |
| --- | ---: | ---: | ---: | ---: |
| Current +Z acceleration | 8 m/s | 211.49 s | 38.66 kg | 55.20 |
| Current +Z acceleration | 9 m/s | 196.16 s | 43.45 kg | 53.24 |
| +X freely aligned with travel | 8 m/s | 193.27 s | 36.74 kg | 63.11 |
| +X freely aligned with travel | 9 m/s | 174.11 s | 41.32 kg | 63.83 |

The +X cases assume free alignment and are not a flyable controller sequence.
Earlier attitude changes added overhead, so these numbers do not justify
restoring nose-first control. Faster acceleration, useful coast and little
correction overhead matter together. Spending the full 50 kg at the +Z 8/9
entry times would instead score only 40.07/44.51. No forced minimum speed or
fuel cutoff is introduced. [Diagnostic records](tests/score_speed_tradeoff.json).
Reproduce with `python -B tests/benchmark_speed_tradeoff.py --sim-config
PATH_TO_FINAL_SUBMISSION --output tests/score_speed_tradeoff.json`.

## Twenty-seed baseline

Seeds 100–119 ran through `tests/benchmark_final_submission.py` against the
exact original source. Conditions were the published 500-debris scene, 60 Hz
control, 2.5 m synthetic ground plane, HiGHS allocator, no injected drift,
zero synthetic planning delay and phase-fuel auditing.

| Modeled quantity | Median | 10th–90th percentile |
| --- | ---: | ---: |
| Zero-contact score estimate | 38.28 | 22.98–43.79 |
| Entry time | 264.97 s | 250.39–280.50 s |
| Entry fuel | 43.73 kg | 39.84–52.35 kg |

All 20 entered, with no modeled pre-entry contact. Score range was 13.79–44.93.
The median cruise impulse-burn fuel was 37.325 kg. Launch used 3.313 kg and
immediate escape used 1.355 kg. These phase medians are descriptive and do
not add up to the median total.

These runs vary radar noise and 0.02 m/s debris velocity jitter. The harness
uses synthetic images, spherical debris and an ellipsoid ship proxy. It
does not reproduce rotating meshes or native contact energy. Zero damage is
modeled from absence of contact. Native damage joules are unavailable.
[Full baseline records](tests/score_sweep_baseline.json).

## Default-off clean main burn

`EFP_CLEAN_MAIN_BURN=1` adds a normalized body-X component of 0.0002 times
the main positive body-Z direction during a shooting burn. The angle is
approximately 0.01146 degrees. Other axes, reverse burns, attitude commands,
path costs and the official jet selector retain their baseline behavior.

Both the published revised-simplex allocator and HiGHS produced approximately
0.13468 m/s² forward acceleration from this request, versus 0.0958134 from
the pure-axis request. Modeled fuel per forward delta-v changed from 4.32233
to 4.32315 kg. This is a numerical actuator result. Native thrust and score
effects still require flight verification.

The difference from the earlier failed angular-trim experiments is the
acceleration-only perturbation with zero added angular command. The archived
`trim_z_highs_trial100` had angular control active for 16.8 percent of its run,
and the broader trim experiment had substantial combined-control overhead.
No angular trim, axis slew or nose pointing is introduced here.

The matched 20-seed experiment was inconclusive and is rejected for promotion.

| Modeled quantity | Baseline median | Clean-main median |
| --- | ---: | ---: |
| Score estimate | 38.28 | 40.34 |
| Entry time | 264.97 s | 227.68 s |
| Entry fuel | 43.73 kg | 47.48 kg |

All 20 candidate runs entered without modeled pre-entry contact. Median paired
time change was -38.03 s and median paired fuel change was +2.35 kg.
The paired bootstrap 95 percent interval for the difference in cohort medians
was -0.81 to +7.63 points, crossing zero.

Median phase-fuel changes were -0.831 kg for cruise impulse burns, +1.021 kg
for avoidance and +1.061 kg for camera waiting. One run used 89.72 kg.
The actuator hypothesis reproduced higher thrust, but did not establish
a reliable score gain. It remains default off.
[Full paired results](tests/score_sweep_clean.json).

## Default-off burn-order diversity

`EFP_ORDER_DIVERSITY=1` preserves the best feasible candidate from each of the
six body-axis burn orders before sensor collision checks. All orders were
already calculated in the reference planner, but the score shortlist could
discard most of them before checking which was clear.

This adds up to six alternatives to the existing collision checks. It changes
candidate pruning, with the original dynamics, risk costs, safety margins,
camera settings and actuator requests. A unit fixture confirms that two
checked orders become six when enabled. Its 20-seed test uses the original
actuator, with clean-main-burn disabled.

All 20 entered without modeled pre-entry contact. The median score stayed at
38.275, with a bootstrap interval for its change of -0.07 to +2.44 points.
The candidate median time was 258.85 s and median fuel was 44.17 kg. Paired
median time, fuel and all phase-fuel changes were zero. The mean paired time
and fuel changes were -3.462 s and -1.082 kg; the score mean rose by 2.027.
The score p10 improved from 22.984 to 29.662, while p90 stayed 43.789.
This lower-tail improvement does not establish the requested median gain.
The experiment is inconclusive, rejected for promotion and remains off.
[Full paired results](tests/score_sweep_order.json).

## Default-off completed-coast recheck

`EFP_RECHECK_COAST=1` changes only the eligibility of a retained completed
burn with no detour or future thrust. The original can credit its old ETA as
an arrival despite the current ballistic prediction missing the safe sphere.
A unit fixture reproduces a 55.4146 m predicted miss being selected as
`IMPULSE_COAST`. With the flag enabled, the planner selects a corrective burn.

The original successful-coast band, 42–48 m and 5–600 s ahead, is retained.
No gravity-gradient endpoint correction, camera reachability constraint,
new camera waiting rule or distant-crossing cost is introduced. This isolates
one retention check from the earlier regressed SCORE_COAST bundle. The
20-seed comparison completed with the other two flags disabled.

All 20 entered without modeled pre-entry contact. Median score was 37.97,
versus 38.275 for the baseline: a -0.305 point difference, with a bootstrap
interval of -0.62 to +0.74. Median entry time was 264.96 s and median fuel
43.54 kg. Paired median changes for time, fuel and every phase were zero;
mean paired changes were +0.730 s and -0.0445 kg. The score p10–p90 was
23.157–43.789. The unit regression is corrected, but this does not establish
a flight-score improvement or prove that it resolves the unidentified native
60 m miss. The experiment remains off and is rejected for score promotion.
[Full paired results](tests/score_sweep_recheck.json).

## Inertia experiments requested after the baseline review

The user reports lower native scores and requests fewer lateral corrections,
30–35 kg or less at 600 m, and consideration of a 9 m/s boost after the dense
region. These are objectives rather than command gates. The baseline's actual
600 m medians are 156.785 s, 41.684 kg and 5.848 m/s; none of its 20 runs was
below 35 kg there. Radar and camera checks remain active throughout the final
leg: the observed debris distribution cannot justify assuming it is empty.

`EFP_INERTIAL_CAPTURE=1` keeps the finite-burn/coast controller in the observed
`CAPTURE_PASS` phase below 280 m. It offers 2–120 s arrival times and retains
a safe coast even when closest approach is less than five seconds ahead.
The original switches to velocity tracking here. Unobserved station, blocked
capture, fallback and damage guards retain their original behavior. No camera
reachability constraint or new waiting rule is added. This is separate from
the old SCORE_COAST and completed-coast-retention experiments.

`EFP_VECTOR_BURN=1` adds simultaneous fixed-direction translation burns to the
original sequential body-axis candidate set. Each new candidate has one burn
followed by coast; its duration is solved against the same destination shell.
Fuel is priced using the L1 body impulse, matching the reference pure-translation
model. An initial component acceleration estimate is calibrated from
observed vehicle acceleration. It does not command jets directly.

The execution direction stays fixed within the selected burn and a completion
latch keeps it off until a new scored plan is selected. The original 2 Hz
radar/camera swept-path checks still reject blocked candidates. No rotation,
artificial acceleration perturbation or altered score weight is introduced.
This differs from CLEAN_MAIN_BURN by delivering the actual lateral impulse
required by the selected path alongside the main impulse. It does not assume
that any allocator thrust increase alone establishes a score improvement.

These flags default off and were tested separately. Source snapshots are
`tests/score_sweeps/inertial_capture_controller.py` and
`tests/score_sweeps/vector_burn_v2_controller.py`. The V1 vector pilot was a
development diagnostic with a missing completion latch; it is excluded from
the V2 cohort.

| Modeled cohort | Arrivals | Arrival-only median score | Median entry time | Median entry fuel |
| --- | ---: | ---: | ---: | ---: |
| Original Momentum | 20/20 | 38.28 | 264.97 s | 43.73 kg |
| Inertial capture | 20/20 | 39.09 | 264.82 s | 42.88 kg |
| Vector burn V2 | 20/20 | 36.38 | 249.76 s | 46.74 kg |
| Exact arc endpoints V1 | 19/20 | 38.68 | 264.87 s | 42.78 kg |
| Exact arc execution V2 | 19/20 | 38.36 | 263.35 s | 43.09 kg |
| Short coast windows | 19/20 | 39.14 | 275.80 s | 40.64 kg |
| Windows + camera recovery | 20/20 | 36.33 | 275.75 s | 42.13 kg |
| Windows + recovery + entry scope | 20/20 | 37.39 | 275.75 s | 41.54 kg |
| Entry scope alone | 20/20 | 38.31 | 264.97 s | 43.60 kg |

No cohort above had modeled pre-entry contact. A failed arrival is excluded
from the arrival-only medians, so the 19-run medians cannot be presented as
20-run score improvements. Those variants are rejected for promotion.

The inertial-capture score change was +0.815 points, with a paired bootstrap
interval +0.01 to +2.89. Allowing for the legacy reports' 0.01-point rounding
makes its lower bound include zero. The signal is marginal and remains off.
Its paired median phase-fuel change was -0.1487 kg in capture and zero in cruise.
[Capture records](tests/score_sweep_capture.json).

The vector-burn median fell by 1.90 points. Its rounding-adjusted interval was
-3.19 to +1.37, and fuel increased. Simultaneous translation did not establish
a useful score gain and is rejected. [Vector records](tests/score_sweep_vector_v2.json).

`EFP_EXACT_ARCS=1` corrects another internal prediction error: original lateral
detours treat an initial offset and -2x return impulse as instantaneous kicks.
Finite-burn fixtures missed their intended endpoints by 13–17 m. `arc_schedule`
solves the first burn after subtracting the return burn's actual displacement.
V2 also explicitly starts a pending return after the first burn completes,
without repeating it or retaining an unnecessary return over a better safe coast.
This changes the existing constant-gravity detour, rather than introducing
gravity-gradient endpoint correction or the old corridor/reserve policy.
Both arc cohorts stalled on seed105 at approximately 95 m in `WAIT_SHIP_CAMERA`.
Neither completed cohort selected a return that reached its execution time.
The executor change therefore did not fix that camera failure. Both are rejected.
[Arc V1 records](tests/score_sweep_arcs.json), [V2 records](tests/score_sweep_arcs_v2.json).

`EFP_COAST_WINDOWS=1` adds 1, 2 and 4 s coast prefixes to the immediate-burn
option, only above 600 m remaining and with at least 1.5 m/s forward closing
speed. The planner predicts debris motion during the coast and subsequent burn,
then compares the full time/fuel score. A selected wait retains its absolute
burn start, avoiding repeated postponement. It does not stop at a waypoint.
The difference from earlier delay trials is an isolated one-radar-sweep option,
without their other guidance/reserve changes, assessed on 20 matched seeds.

The short-window cohort reduced median fuel at 600 m from 41.684 to 39.6915 kg,
at a median crossing time of 162.88 s versus 156.785 s. None crossed below35 kg.
Paired median cruise fuel change was -2.3952 kg, including -2.3386 kg in impulse
burns, on the 19 clean pairs. Seed105 stalled awaiting camera coverage, so the
planner alone is rejected. [Coast-window records](tests/score_sweep_windows.json).

`EFP_CAMERA_RECOVERY=1` addresses the camera stall separately. The old waiting
logic supplied a ship-pointing hint which `FreeAttitude` ignored. The new
`ViewRecoveryAttitude` maintains a 25-degree inner camera-view margin during
a sensor-checked coast. It makes bounded low-duty attitude requests, settles
spin and keeps the camera aimed at the ship while adjusting. It adds no full
nose alignment and does not start view turns during translation thrust.
The next 12 s of coasting must be clear. An unobserved ship retains the 80 m
guard; fresh observed geometry permits continuing the view through the approach.
All thrust stops on entry.

On the failed arc seed105, the V1 view diagnostic finished but still spent
7.28 kg waiting for camera. The V2 margin policy finished in 271.48 s at
51.56 kg, estimated score25.46, with no contact. It used 0.8353 kg of
attitude-only fuel and 1.1787 kg in combined-command ticks; the latter cannot
be uniquely attributed to rotation. These are individual diagnostics, not
median improvement evidence. The full coast-window plus V2-view cohort was
tested as one additional change against the coast-window cohort.

The full coast-window plus V2-view cohort completed 20/20 without modeled
contact. Its median score was36.325, median time275.75 s and fuel42.13 kg.
It resolved the window-only camera timeout, but net median score versus the
original was -1.95 points, rounding-adjusted interval [-2.53,+1.09]. Reliability
recovery is not a score win. [Combined records](tests/score_sweep_coast_view.json),
[isolated view addition](tests/score_sweep_view_added.json).

`EFP_ENTRY_SCOPE=1` tests the mission's actual end condition. The old station-line
check extended 100 m past its 47 m tangent point; this could reject a successful
pass because of later contact. The flag checks that line to its capture point
and clips camera-observed terminal collision sweeps only after predicted entry
to radius48 m. This retains a2 m margin inside the official50 m sphere and
retains all collision checks before it. Unobserved-station guards still apply.
View recovery can similarly certify a known safe coast to closest approach.
It does not set arrival from a prediction: actual frame-relative entry still
stops thrust. Unit fixtures verify that a post-entry obstacle no longer causes
braking, while a pre-entry obstruction still blocks the approach. Independent
20-seed entry-only and combined-plus-entry cohorts completed without contact.
The combined cohort's median score37.39 was below the original38.275; its
rounding-adjusted median-delta interval was[-2.28,+2.02]. Entry-only had median
paired time/fuel changes of zero and a median-delta interval[-0.295,+0.77].
Neither demonstrated a score gain. [Combined entry records](tests/score_sweep_entry_scope.json),
[isolated entry records](tests/score_sweep_entry_only.json).

`EFP_TRACK_CACHE=1` restores a numerical-equivalence tracking optimization from
the later `731c8df` Momentum revision. Unchanged radar reports reuse their
fixed sample fit; new reports recompute it. Sensor geometry, velocity estimates,
uncertainty and planning costs stay the same. On a paired500-object mature-track
fixture, the original tracker took median131.98 ms versus31.58 ms for the
later cache, with zero output difference. These are measured CPU timings under
concurrent offline load, not native score results. Native blocking-time effects
still require verification. [Timing record](tests/tracker_cache_audit.json).

The current flag implementation was separately checked on the same kind of
500-object paired fixture: median107.79 ms versus24.32 ms, with exact numerical
outputs and uncertainty metadata. This measures repeated unchanged samples,
an upper-bound cache opportunity; native cycles also include new samples.
[Current timing record](tests/tracker_cache_current_audit.json).

The final cache-only replay completed all20 matched seeds with zero modeled
contacts. Every reported entry time, fuel, score and phase-fuel total matched
the original. Median score38.275, time264.97 s and fuel43.725 kg are unchanged.
This validates modeled flight equivalence; it establishes no score gain. The
flag remains off until native timing is verified. [Cache replay](tests/score_sweep_cache.json).

The scoring search can choose a faster late burn, including candidates near
9 m/s, when it beats coasting after its fuel cost and sensed path risk. A forced
boost or a fuel cutoff at 600 m is not introduced. A missed safe sphere is not
counted as success; frame-relative entry is still checked every flight tick.

## Native model gap and diagnostic trace

Three recent native runs match the restored baseline's exact CRLF file hash
at commit `b524b36`: scores5.1/9.7/14.2, fuel63.92/59.30/57.90 kg, and zero
entry damage. This is measured native evidence, not a 20-run median. One run
spent an extra12.5 kg between about31 and56 s while speed stayed near2 m/s.
[Native observations](tests/inertia_native_observations.json).

A later complete scan includes all source-matched November18 logs, rather than
only those three diagnostic examples. See [same-source native inventory](tests/inertia_native_inventory.json)
for descriptive medians, full spread and every discovered log. Unknown seeds,
aborted runs and a live run prevent a matched native comparison. The newly
completed143820 run scored24.1 at265.27 s,53.36 kg and zero entry damage.
It still used deployed hash9240452c…, so it did not test the current workspace's
trace or experiment flags. Post-entry station damage is excluded from this result.

The old offline model underestimates these poor native outcomes. An opt-in
`--rng-mode native-order` now reproduces numeric debris ordering, separate
spawn RNG8193 and the radar's closest-point noise draw. Its one baseline pilot
still scored42.24 at267.28 s and39.49 kg. That diagnostic does not explain
the native gap. Legacy mode remains the common setting for the paired cohorts.
Native meshes, control timing, camera calibration and dynamics remain unverified.

`EFP_FLIGHT_TRACE=1` adds file-only `IGNITION_STATE` JSON records every five
seconds. They contain vehicle/target state, body rotation/rate and commands,
burn target/acceleration, up to12 radar reports and8 camera surfaces. Trace
errors disable tracing rather than interrupting flight. The standard controller
log also records body command/rate, selected axis and remaining impulse.
For a diagnostic baseline, keep algorithm experiment flags at zero. Locate the
trace in the new run folder's logs; it need not be in the on-screen alert log.
The extra state is needed before attributing the native gap to a particular cause.

## Active baseline parameters

The later default-off `EFP_DOGLEG=1` experiment completed 20 matched seeds.
Median score fell from38.275 to27.890, with20 arrivals and zero modeled contacts
in each cohort. Paired median time improved18.335s but fuel increased9.445kg.
The rounding-adjusted median-score-delta interval[-17.340,-3.030] rejects it
for score loss. Exhaustive search also had a costly measured latency tail.
The original Momentum default is retained. See [dogleg design and results](DOGLEG.md)
for phase fuel, spread, assumptions and native verification requirements.

The finite-burn search uses 16 capture azimuths, six axis orders and arrival
times from 45 to 450 s, plus retained arrival times. It checks the next 20 s
against sensors and replans at 2 Hz. The scoring-selected cruise ceiling is
10 m/s. The launch handoff is 2.5 m and the nominal capture tangent is 47 m.
An ordinary retained burn has a four-point allowance for its first four seconds,
then two points. The execution tolerance is 0.02 m/s during shooting.

The older 25 m/s constants also appear in legacy code. The active finite-burn
planner's 10 m/s ceiling and score ranking determine its normal speed.

## Reproduction

The installed SDK Python contains NumPy and SciPy. Supply the local official
configuration and jet CSV to the benchmark.

```powershell
python -B tests/benchmark_final_submission.py --sim-config PATH_TO_FINAL_SUBMISSION --jet-config PATH_TO_JET_CSV --ground-plane 2.5 --fuel-audit --baseline-ref 4a8507e --seed 100 --output tests/score_baseline_100.json
python -B tests/benchmark_final_submission.py --sim-config PATH_TO_FINAL_SUBMISSION --jet-config PATH_TO_JET_CSV --ground-plane 2.5 --fuel-audit --experiment clean-main-burn --seed 100 --output tests/score_clean_100.json
```

Repeat for the same seeds through 119. Then run the paired summary.

```powershell
python -B tests/summarize_score_sweep.py --baseline-prefix tests/score_baseline_ --candidate-prefix tests/score_clean_ --output tests/score_sweep_clean.json
```

The summary includes median, IQR, range, paired time/fuel deltas, phase-fuel
differences, contacts and a paired 10,000-resample bootstrap interval.
It includes means so phase fuel can also be compared additively.
An interval overlapping zero is treated as inconclusive. Any new contact
or incomplete arrival rejects a zero-damage candidate.

For native baseline trials, leave every algorithm experiment flag at zero, along
with `EFP_NOSE_FIRST=0` and `EFP_AXIS_THRUST=0`. Confirm the file hash and flags
in the controller log. Optional experimental trials must enable only one flag
and use the corresponding frozen source; no completed trajectory experiment has
earned promotion to the default controller.
Keep the official success alert and entry time/fuel/damage for each run.
Twenty matched native runs are needed to assess the requested native median.
The available [native log inventory](tests/native_score_inventory.json)
contains mixed versions and missing seed/source fields.

Rebuild that inventory with `tests/summarize_native_logs.py --root LOG_ROOT
--output tests/native_score_inventory.json`. It reads official success fields
in either order, including fields preceding the success alert. It leaves
unknown entry fields unknown and excludes later fuel/damage from entry results.
The controller suite passes 163 tests. Completed 20-seed cohorts are recorded
above, with frozen sources, entry metrics and paired phase-fuel deltas. No
trajectory experiment established a median gain beyond its paired seed interval
while preserving all arrivals. The native 60–70 objective has not been achieved
or verified.
