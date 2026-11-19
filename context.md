# Escape from Phobos project context

Updated 2026-11-19. Read this file before controller changes and update it after
each tested change. It is a maintained summary of the available chat history,
repository history, guides, archived experiments and local simulation logs.
The original chats and logs remain the sources for details.

## Current authoritative state: SCORE_MOMENTUM with completion-thrust fix

**Latest user direction, 2026-11-19:** the user manually restored SCORE_MOMENTUM
because subsequent versions performed worse in their testing. Future controller
improvements must start from this Momentum version and preserve its successful
behavior. The subsequent authorized code task asks to remove station-approach
braking and consider a boost near 500 m only when the score tradeoff warrants it.
The earlier documentation-only restriction applied to the completed context
update, not to this code task. Preserve the restored Momentum flight planner.

This section supersedes every older "current", "selected", default-setting,
pending-task and stop/continue instruction in the historical record below.
The Dogleg/ground-lateral sequence is no longer the active development track.
Do not automatically continue wider-lane or data-driven-offset tasks, restore
the experimental controller, or assume the annotated green path is the chosen
implementation. It remains an unproven route hypothesis.

### Verified active files and version identity

- Current committed checkout: `4a75328` (documentation), following the user's
  restoration/archive commit `4210f77`. The completion fix is an uncommitted diff.
- Active entry point: `Controller.py`, constructing `ShootingNavigator` and
  reporting `profile=SCORE_MOMENTUM`.
- Its starting normalized source exactly matched `4a8507e:Controller.py`.
- `temp_controller/score_momentum_controller.py` remains that unmodified
  original Momentum baseline. Preserve it; the active file now includes the
  small completion fix described below. The rejected entry-scope trial exists
  only in its frozen experiment source.
- `temp_controller/currentcontroller.py` contains the later Dogleg/ground-lateral
  implementation. It is an archive, not the active entry point or recommended
  baseline. Preserve it without copying it over Controller.py.
- Original restored baseline normalized-text SHA256:
  `2b0f0f0b5674085d22d0ec5a454ed4ccde49b13e16e8645118210379578089af`.
- Original restored baseline deployed-byte SHA256:
  `b461c96e8db27d760827e9f29ab9b4ec8de0335033080dcdd0f4d399fd073883`.
  Line-ending conversion can change the byte hash without changing normalized
  source text; compare like-for-like when identifying native runs.
- Selected completion-fix normalized source SHA256:
  `5fd084a2121ceb7772e084caa391df7261d39e4dddb1a20bc6d5ab086c0dbaf3`.
- Selected deployed-byte SHA256 at final validation:
  `f1e162886a834c4511fdda4248ce132c21ed312c9a137f22079d774923a32a78`.

### What is active, and what was not carried back

The active Momentum design prices finite body-axis burns and ballistic coasts
using time/fuel/damage score, preserves useful velocity, estimates unpowered
drift, tracks noisy radar reports, uses asynchronous RGB/depth capture and
camera-based terminal checks, and detects safe-sphere crossings between planner
updates. Navigation uses the Phobos frame; SetCommand receives body-frame
acceleration/angular acceleration through the existing conversion.

Verified current settings: 2 Hz planning (`PLAN_PERIOD=0.5`), per-tick velocity
feedback in the 60 Hz loop, a 2.5 m launch handoff and an effective scored
cruise ceiling of 10 m/s. Legacy `MAX_SPEED=25` does not mean that the active
score optimizer commands 25 m/s. There is no active hard fuel ceiling/reserve.
`EFP_NOSE_FIRST=0` and `EFP_AXIS_THRUST=0` are the restored defaults;
`EFP_CAMERA_GUI=1` enables the existing viewer by default.
No new live experiment flag, fuel cap or staged-coast planner is enabled.

The restored file has **no DoglegNavigator or ground-lateral implementation**.
The later EFP_DOGLEG/EFP_GROUND_LATERAL and other archived experiment switches, source-hash
logging, IGNITION_STATE flight trace and lane diagnostics are not present in
this active source. Setting those old environment variables does not add the
missing features. Existing fuelPhaseKg, planMs, speed, damage and camera-status
logs remain available. Older guides describe archived versions and must not
be treated as a feature inventory for the restored file.

### Selected completion fix and rejected terminal experiment, 2026-11-19

**Active correctness fix:** a detected 50 m scoring-sphere entry now latches
`MISSION_COMPLETE_COAST`. Both the normal servo and an unfinished Shooting burn
return zero linear acceleration; the runtime also sends zero angular acceleration.
Losing radar, moving out of the sphere, a camera obstacle after entry or spin
damping cannot restart the jets. The capture approach no longer creates a
post-entry braking/parking command. The official result still comes from Scoring.
The installed Scoring.py uses `distance <= KeepOutRadius_m`; its damage test
runs before success. Damage from a collision before entry remains scored.
Removing fuel burn after success cannot increase an already frozen official score.

**Rejected experiment, frozen source only:** with current station camera
coverage, terminate local swept-obstacle checks at a predicted 48 m entry
(2 m inside the 50 m sphere), and stop rejecting capture lines solely because
they encounter geometry after capture. Obstacles before entry and missing-camera
guards remain. The terminal coast rollout now includes gravity and zero thrust;
it previously simulated a constant-speed velocity servo during CAPTURE_PASS.
The prior entry-scope trial gained only 0.035 modeled median points. This trial
also corrected the coast rollout's gravity/servo approximation, but the new
20-seed result still fails the gain-above-seed-noise requirement. It has been
removed from active Controller.py; EFP_ENTRY_SCOPE is not a live switch.

**Late boost:** inspection and regression fixtures confirm that the restored
Shooting planner already evaluates finite scored burns near 500 m, up to 10 m/s.
No mandatory 500 m kick or assumed debris-free region is added. In a clear
modeled 450 m case, elapsed 100 s, current 3 m/s, fuel 25 kg, it chooses a 13.26 s
burn toward 5.32 m/s; at elapsed 150 s, 8 m/s and 35 kg it chooses zero thrust.
These are planner fixtures, not native flight results or guaranteed speeds.
Nearer than 280 m the original terminal handoff is retained.

The active default settings preserve the original pre-entry Momentum decisions.
Completion thrust is removed by default; no terminal experiment is promoted.
Connection boilerplate, SetCommand, camera helpers, official scoring, physics,
jet selection and STU4-Sim files are unchanged. No commits, pushes or deployment
are performed by this task.

**Measured offline comparison, seeds100–119:** baseline median38.275, candidate
38.310; delta+0.035 points, rounding-adjusted paired-bootstrap95% interval
[-0.295,+0.770]. Both medians264.97 s; fuel43.725 vs43.595 kg. Both20/20 arrivals
with zero modeled contacts;13 entry-result records identical. Paired median
time/fuel deltas are both0; mean fuel delta-0.085 kg. Launch/cruise phase deltas
are exactly0; mean CAPTURE_PASS delta-0.073935 kg, OBSERVE_CAPTURE-0.011095 kg.
The score-gain hypothesis is inconclusive; reject promotion. All20 candidates
used0 kg in their90 s post-entry tests, but that fuel is outside official scoring.
Results: `tests/score_sweep_momentum_terminal_scope.json`,
`tests/momentum_terminal_scope_SEED.json`; tested candidate source:
`tests/score_sweeps/momentum_terminal_scope_controller.py` (normalized SHA
`7c11adeff9d372d1468a9d03d603e2d3a105d2098ad53e219ce32ff145c87f60`).
This source is an archive, not the selected entry point.

**Default completion check, modeled seed100:** same original entry score44.93,
time255.93 s and fuel38.97 kg; post-entry90 s fuel drops23.1234 ->0 kg.
Do not present a single completion check as a new20-run score improvement.
Tests:202 pass, including active Momentum completion/late-burn cases and
explicitly archived experiment cases. See `MOMENTUM_TERMINAL.md` and
`tests/momentum_validation.json`. Native revalidation remains required.

### Evidence to use as the Momentum baseline

**User-reported native experience:** SCORE_MOMENTUM worked better than later
versions and could approach roughly 50 points. This is the user's experimental
judgment, not a verified matched 20-run native median or a guarantee.

**Historical modeled baseline for this exact normalized source:** seeds100–119
in `tests/score_sweep_baseline.json` / `tests/score_baseline_SEED.json`:
median score38.275, p10–p90 22.984–43.789, range13.79–44.93;
median entry time264.97 s, fuel43.725 kg;20 arrivals and zero modeled contacts.
These archived offline estimates are reused as the exact-source baseline for
this task. The model does not provide native damage energy or fully account for
native meshes, rendering, dynamics and controller scheduling.

**Later experiments, retained as rejected historical evidence:** Dogleg median
27.890 at245.845 s/54.890 kg; ground-lateral median25.980 at243.475 s/53.750 kg.
Both had20 modeled arrivals/no contacts, but neither improved the selected
Momentum baseline. Ground-lateral's paired interval against Dogleg included
zero. Reports: `tests/score_sweep_dogleg.json`, `tests/score_sweep_ground.json`,
`DOGLEG.md`, `DOGLEG_DIAGNOSTICS.md`, `GROUND_LATERAL.md`.

The measured native -54.1/357.75 s/103.26 kg result and 9.688 s search stall
belong to the later Dogleg source c347e27a…, not this restored Momentum file.
They demonstrate a problem in that archived version; do not attribute those
numbers or that exact latency to current Momentum without profiling it.

Other rejected experiments and their paired results remain in SCORE_TUNING.md
and the historical notes below. A faster tracker fixture or a single better
seed is not evidence of a native score gain. Do not restore whole failed
bundles to recover one useful idea.

### Continuing direction

Improve **achieved official score** from SCORE_MOMENTUM, initially toward55–60+
with zero scored damage, stretch70+. Do not change the organizers' nominal
references, scoring function or simulation to raise a number. Prioritize useful
coasting, fewer unnecessary lateral/terminal corrections, reliable 50 m entry,
and the actual time-versus-fuel reward. Do not force constant8 m/s, large
high-speed corners, full nose/strong-axis slews or a field-wide bypass without
evidence that the full cost improves score. Gravity still changes a free coast.
Fuel targets remain tradeoffs, not reinstated hard caps.

Profile the restored controller's own phase fuel and planning latency before
assuming that an archived version's bottleneck is identical. Match native logs
to this exact source/settings; the current restoration has no automatic source
hash/IGNITION_STATE logging. Match native evidence to source and actual settings.

Keep the existing connection boilerplate and official jet selector unchanged;
leave STU4-Sim untouched. Use supported acceleration/angular-acceleration
commands, never individual jet commands or live debris truth. Radar reports
are shuttle-relative Phobos vectors with radial velocity only; full velocity
is estimated. Camera coverage is finite, and center-spread percentiles do not
prove a region is empty or establish the required bypass distance.

Continue the evidence standard for future changes: establish/reuse an exact
versioned baseline, compare at least20 identical offline seeds, test one change
at a time behind a default-off flag, report score spread, entry time/fuel,
phase-fuel deltas and contacts, and reject gains within seed noise. Label
modeled, measured native and assumed quantities separately. Native55+ remains
unverified. Official entry results, not post-entry fuel/damage, define a run's
score. The nominal/reference run scores50; improving the controller does not
mean changing nominal time120 s, fuel75 kg, damage threshold209200 J or the
50 m success sphere.

Historical193-test passes belong to the later experimental source. The first
restored-source test audit exposed52 incompatible archived-experiment cases.
Those tests now explicitly load `temp_controller/currentcontroller.py`; entry-scope
fixtures load the rejected trial's frozen source. Adapter
tests do so only when requesting an archived feature. Normal Momentum adapters,
economy, camera, frame and new completion tests still exercise active Controller.py.
This test separation does not deploy or restore the rejected experimental code.
Preserve the frozen experiment sources/results for reproduction.

## Historical experiment archive — superseded by the restoration above

The following record is retained for provenance. Its previous active/default
claims, pending work and test counts describe those versions at those times;
they are not instructions to switch away from the restored SCORE_MOMENTUM.

## Annotated green route: Task 2 completed, not promoted

Latest image/request proposes an early sideways exit, climb outside the dense
region and return to the ship, targeting180s/55–60+ with efficient8+m/s flight.
The sketch has three legs/two bends, which the one-bend planner cannot fully
represent. Prior instruction to stop/report after each stage remains in force.
This continuation implements Task2 only: EFP_GROUND_LATERAL, default0, within
DoglegNavigator. User's EFP_DOGLEG default1 remains unchanged. Do not silently
add early/wider offsets, a second bend, sensor-policy changes or forced speeds.

Source/config/native log verification: launch.up=origin/|origin| in Phobos
coordinates, pointing away from its center; native logged radial launch
height rises0→0.1→2.1→2.8m. This is radial tangent, not an independently
measured local terrain normal. New basis uses normalized cross(forward,up),
with the old body-derived basis preserved for missing/invalid/parallel up.
All offsets/fractions/speeds/costs and actuator/camera helpers stay unchanged.
Runtime start/reset wiring and benchmark --experiment ground-lateral require
dogleg; the feature is default-off.193 tests passed (five basis/launch tests,
including exact candidate equivalence with Task1 when off, plus one runtime
test). Frozen20-seed source:
tests/score_sweeps/ground_lateral_controller.py; results tests/score_ground_SEED.json,
seeds100–119, matched against tests/score_dogleg_diag_SEED.json. Cohort complete.
Results:tests/score_sweep_ground.json; review/assumptions:GROUND_LATERAL.md.
All20 arrivals/zero modeled contacts in both cohorts. Ground median score25.980
vs27.890, delta-1.910, rounding-adjusted paired interval[-8.270,+7.125]:
inconclusive, reject for promotion. Ground scorep10–p90 13.216–36.161,
range-4.32–43.98; medianT243.475s,F53.750kg. Paired medianTdelta+.490s,
Fdelta-.205kg. Mean phase change about-.107kg: alignment+.110,initial-.146,
bend-.333,existing cruise-.426,capture/observe+.077,camera wait+.646,coast-.037.
All20 still hit no_safe_lane:396 of469 searches vs400/467 baseline. Both
executed six bends and had zero post-entry fuel use. Search tail remains
large: new per-run maximum median12.636s,max14.562s under four concurrent
benchmarks; Task1 reference median12.487s. Wall time is not charged in the
fixed-step flight model. Keep EFP_GROUND_LATERAL=0; preserve userDogleg default1.

Review found38 nominal163.4N jets, force opposite nozzle normals. Only the
official acceleration/angular-acceleration selector is commanded. Published
reference-mass +X acceleration.17456m/s² implies at least45.83s to gain8m/s
from rest ignoring gravity; stronger-axis alignment is not free. At180s and
zero damage,60 points needsF≤42.5kg;55 points needsF≤46.25kg. Coasting under
gravity is unpowered but not constant-speed. Cameras see only their aimed
finite FOV and cannot certify a globally debris-free region from one image.

NEW measured native evidence: log261119_065413_FINAL_SUBMISSION__46a matches
Task1 source byteSHA c347e27a… and officially succeeded atscore-54.1,T357.75s,
F103.26kg,D0J. Native search median3.846s,max9.688s; this is one baseline run,
not a matched cohort or evidence for ground-lateral. No IGNITION_STATE trace
was found there. Synchronous search can hold commands while state changes;
do not claim its share of fuel without a state/command trace. Artifact:
tests/ground_lateral_frame_audit.json. The native/offline discrepancy and
blocking-search tail remain unresolved; do not promise55+ from a basis change.
All20 matched seeds are summarized.193 tests pass. Source preservation:
tests/ground_lateral_preservation.json (textSHA32b83a5f…,bytesSHAb819dac3…);
51 other definitions, connection prefix and official assets unchanged. No
native test of the new flag, simulation modification, push or commit was
performed. All benchmark jobs finished. Stop after Task2 per the staged method;
do not claim the full green three-leg route, <180s or55+ has been achieved.

## Task 1 completed: diagnostics only; stop before Task 2

User explicitly orders Tasks1–4 sequentially with a stop/report after each.
Only Task1 is authorized in this turn. Do not implement EFP_GROUND_LATERAL,
EFP_WIDE_LANES or data-driven offsets yet. The current HEAD65f0b41 is clean
at task start and USER-CHANGED EFP_DOGLEG default to1. Preserve this setting
for diagnostic-only work; older notes saying default0 describe the previous
experiment/recommendation, not the current checkout.

Verified geometry: midpoint offsets at1300m are22.7/45.7/69.1m for4/8/12deg,
118.3/187.6m for20/30deg. candidates uses a body-derived transverse basis,
fractions.3/.5/.7 only and no bent routes below300m. Those control methods
are unchanged by Task1. Critical frame correction: observe_scene centers
already are shuttle-relative Phobos vectors, so do NOT subtract loc again.
The failed-search diagnostic uses those vectors and only the radar prefix
(latest_radar_count), excluding appended camera surfaces. It filters strict
50<along<distance-50, records count,p50,p90,max lateral distance and nulls for
empty slabs. It describes nominal centers, not radii/uncertainty or safe gaps.

Added persistent no_safe_lane counter, richer selection/closure records and
run summaries (searches/failures/latest route/predicted versus actual fuel).
Native summaries are logged periodically and on arrival. Closed-route fuel
is frozen at its closure; later fallback fuel is not charged to that route.
The benchmark appends a summary with exact modeled entry fuel. Counters
survive bounded event-log buffer flushes. No decisions, costs, control helper,
camera helper, thrust or physics changes.51 other existing definitions and
candidate/burn/servo methods matchHEAD; official hashes/prefix unchanged.
Audit:tests/dogleg_diagnostic_preservation.json.

187 tests pass (five new diagnostic tests and two runtime logging checks).
Twenty frozen-source diagnostic
replays completed for seeds100–119 with only dogleg enabled, legacy model
settings matching the earlier dogleg cohort. Source:
tests/score_sweeps/dogleg_diagnostic_controller.py (textSHA6b735e26…,
bytesSHAc347e27a…). Outputs:tests/score_dogleg_diag_SEED.json. Compare with
tests/score_dogleg_SEED.json to prove flight metrics/phase fuel unchanged.
Report:tests/dogleg_diagnostics_report.json, generated by
tests/summarize_dogleg_diagnostics.py; human summary:DOGLEG_DIAGNOSTICS.md.
All20 reported entry metrics and complete fuel audits exactly match the
pre-diagnostic dogleg archive at reported precision. Score/time/fuel deltas0.
Modeled current-code baseline: median score27.890,p10–p90 8.882–37.932,
range-.44–39.77; medianT245.845s,F54.890kg;20 arrivals,zero modeled contacts.
All20 runs hit no_safe_lane:400 failures of467 searches (85.7% of searches),
median20 failures/run (range16–24),23.5 searches/run.43 failed-search slabs
were empty;357 nonempty. First failures all at8.5s/distance~1303m:
per-run median count481(range480–481),lat50=244.856m,lat90=346.193m,
latmax=461.962m. First lat90 range343.702–347.524m,all20 above100m.
These are modeled nominal radar-center estimates, not native debris truth,
inflated safety volumes or proof of a required bypass offset. A350m midpoint
offset at8m/s over1300m would ideally cost~32.8kg in bend fuel alone; this
conditional example favors considering internal low-cost gaps, not a claim
that every bypass requires350m. Repeated failure events are correlated.
No native run, push, simulator change or behavioral tuning was performed.
All benchmark jobs finished. Stop/report after Task1 as the user explicitly
requested; do not start ground-aligned axes or wider/earlier lanes yet.

## Completed dogleg experiment: rejected, default off

User now explicitly requests a default-off one-bend dogleg planner, one initial
body+X slew, held attitude at the bend, full-route checks against all radar
tracks and event-triggered search, with cameras only for the final hull check.
EFP_DOGLEG=1 selects DoglegNavigator/DoglegAttitude; the default remains
SCORE_MOMENTUM. SetCommand, camera helpers, connection prefix and simulator are
unchanged. Current baseline is the existing 20-seed original Momentum cohort;
the prior cache replay established identical default entry metrics on all20.

Installed Radar.py/config verify Phobos-expressed shuttle-relative positions,
radial velocity only, sigma2m/.02m/s/5% diameter. Full debris velocities are
tracker estimates, never live simulator truth. The cost model charges sequential
axis L1 impulse at3388.7/(80*9.8)=4.3223214kg/(m/s), plus an explicitly assumed
conservative slew cost. Canonical angular LP audit gave3.98–7.30kg/(rad/s);
actual native phase cost is unverified. No flight trace exists: user answered
“none” to the specific log-folder question.

The planner searches5–10m/s in.5m/s steps, four interior capture points,
30/50/70% waypoints and4/8/12deg then20/30deg wider offsets, eight azimuths.
It includes finite sequential burns, coast during initial slew, gravity,
uncertainty growth, all-track swept checks and predicted48m entry. It widens
the route grid, never relaxes clearance. Expensive search occurs on launch,
new predicted risk, slew settlement, capture miss or a substantial new radar
confidence/distance event after fallback. Retained checks still run at2Hz.
No forced cruise speed, smooth thrust arc, direct jets or fuel cap is introduced.

Current suite180/180:15 focused dogleg tests plus two new runtime adapter
checks. V1 pilot was interrupted without results. V2 pilot and V3 clear-corridor
diagnostic are excluded from the final cohort. Clear diagnostic arrived at
T271.27,F46.30,score32.50 without modeled contact; the one slew cost43.5s and
1.3987kg. Six searches, no bend selected. This exposes overhead, not a win.

The20-seed frozen-source cohort completed from
tests/score_sweeps/dogleg_final_controller.py (normalizedSHA4322889f…,
byteSHA6465a5ef…). Only dogleg was enabled. All20 arrived without modeled
contact; all20 aligned, six executed a bend. Original median score38.275,
dogleg27.890; difference-10.385, rounding-adjusted bootstrap interval
[-17.340,-3.030]. Reject for modeled score loss. Dogleg medianT245.845s,
F54.890kg; paired medianTdelta-18.335s, Fdelta+9.445kg. Scorep10–p90:
8.882–37.932; range-.44–39.77. Native damage joules are not simulated.
Mean phase-fuel delta+9.927kg: main burns-3.240,new initial+8.202,bends+1.826,
alignment+.587,terminal+3.356,other-.804. More burn/terminal fuel outweighed
time savings. No fuel used after entry. This rejects this configuration;
it does not prove that every possible dogleg route is unprofitable.

Per-run maximum planner calls ranged5.093–6.763s (median5.870s) under four
concurrent benchmarks: measured compute, not native flight latency. The
fixed-step harness does NOT charge this wall time, so native held-command
effects remain an additional risk. No source changes occurred during the
cohort; all47 original definitions, connection
prefix and official asset hashes match37bbd54 (tests/dogleg_preservation_audit.json).
Keep EFP_DOGLEG=0 for competition. Results:tests/score_sweep_dogleg.json;
design/assumptions/reproduction/native checks:DOGLEG.md.180 tests pass. No
benchmark jobs remain running. No native run or source push was performed.
Next verification is a native baseline run withEFP_FLIGHT_TRACE=1 (user said
none exists), followed by calibration of state/command/fuel/timing discrepancies.
Do not promote this planner or claim60+ without new matched native evidence.

## Selected state after the inertia experiments

This snapshot supersedes the historical work-in-progress notes below.
Controller.py retains the original 4a8507e SCORE_MOMENTUM flight policy by
default, with mandatory zero thrust after actual safe-zone entry. Every
algorithm experiment remains default-off. No hard fuel budget, forced late
9 m/s burn, full nose pointing or new official simulator configuration is used.
Connection prefix, BasicController.py, UserAPI.py and installed official assets
were checked unchanged. The current suite passes 163 tests.

Twenty matched offline seeds100–119 per trajectory cohort found no reliable
score gain with all arrivals preserved. Original median score38.275,
T264.97 s and fuel43.725 kg are modeled, not native performance. Its median
fuel at600 m remaining was41.6845 kg. The short-coast-window experiment
reduced that to39.6915 kg, but timed out on one camera-observation case.
Adding camera recovery restored20/20 arrivals but median score fell to36.325.
Adding entry-scoped collision checks gave37.39. Entry-scope alone gave38.31,
delta+0.035, rounding-adjusted interval[-0.295,+0.77]: inconclusive. No tested
cohort crossed600 m below35 kg. The native60–70 goal remains unmet.

Rejected/default-off alternatives: simultaneous vector burn (higher fuel,
median36.375), exact finite-burn arcsV1/V2 (each19/20 arrivals), inertial terminal
handoff (marginal+0.815 at rounded-score precision), short coast windows and
their view/entry combinations (above). Read SCORE_TUNING.md and corresponding
tests/score_sweep_*.json before revisiting them. Frozen sources and individual
records preserve reproducibility and distinguish revisions.

EFP_TRACK_CACHE=1 is a separate computation experiment which reuses unchanged
radar report fits. Unit fixtures verify exact output/uncertainty equivalence
through report changes and missing history. A paired500-object unchanged-report
fixture measured median107.79 ms uncached versus24.32 ms cached. This is an
upper-bound cache opportunity under concurrent load, not a native score gain.
Record: tests/tracker_cache_current_audit.json. The final20-seed replay finished:
all20 entry times, fuels, scores, contacts and phase-fuel totals matched the
original exactly at reported precision, with20 arrivals and zero contacts.
Report: tests/score_sweep_cache.json; frozen source:
tests/score_sweeps/final_cache_controller.py. This establishes modeled policy
equivalence, not score improvement. The flag remains off pending native timing
verification. No benchmark jobs remain running.

Three identified native baseline logs scored5.1/9.7/14.2 at57.9–63.92 kg,
zero entry damage, much worse than the model. Native-order radar RNG diagnostic
did not explain this discrepancy. EFP_FLIGHT_TRACE=1 adds bounded file-only
state/command/radar/camera records every5 s; failure disables tracing without
interrupting flight. The user was asked for the new run folder under their
existing Logs root with algorithm flags off. Native timing, camera calibration,
meshes and dynamics remain unverified; do not guess the cause or promote more
offline tuning as a native improvement until those traces are available.
The subsequent full same-source November18 inventory contains13 official
success alerts: descriptive median14.2 points,289.87 s,57.9 kg, zero entry
damage; score range-19.3 to43.2 and fuel40.95–78.05 kg. Two other logs had no
success alert at read time (one live), so no failure rate is inferred. These
are not matched seeds. See tests/inertia_native_inventory.json. Newly completed
143820 run scored24.1,T265.27,F53.36 and still used9240452c… deployed source.
It provides no current experiment/trace validation. Do not cherry-pick the
three diagnostic poor examples as the whole native population.

## Current user direction

Latest continuation: prioritize inertia, predict moving-debris corridors from
radar and RGB/depth, reduce repeated lateral corrections, aim for 30–35 kg or
less by 600 m remaining, and evaluate a late boost toward 9 m/s only when it
pays in score and the sensed corridor is clear. Continuously aim the predicted
trajectory into the 50 m safe sphere. These are performance targets, not fuel
cutoffs or evidence that the last 600 m is always debris-free.

The latest request supersedes all fuel ceilings, staged reserves and forced
bypass work below. Restore SCORE_MOMENTUM from 4a8507e. Optimize the actual
score tradeoff between time, fuel and damage toward a median native 60–70.
Fuel is unlimited. Keep the official jet selector and simulator intact.

Establish a baseline using at least 20 offline seeds. Test one change at a time
behind a default-off flag. Report paired median score delta, spread, entry
time/fuel, phase-fuel differences and contacts. Reject gains within seed noise.
Never brake after entry. Native scores require official logs and source/seed
identity. Offline contacts do not provide damage joules.

### Historical experiment notes (superseded by selected state above)

New isolated experiments are in progress. EFP_INERTIAL_CAPTURE=1 keeps finite
burn/coast control below 280 m only while the existing camera observation and
capture guards permit CAPTURE_PASS. It uses a 2–120 s terminal arrival grid
and allows a safe ballistic coast less than five seconds from closest approach.
It does not change camera reach, gravity, far cruise, margins or score weights.
This differs from the failed SCORE_COAST bundle by removing the controller
handoff, with existing perception guards intact. Its 20-seed cohort uses frozen
tests/score_sweeps/inertial_capture_controller.py with all other flags off.

EFP_VECTOR_BURN=1 adds a fixed-direction simultaneous translation burn followed
by coast to the original sequential-axis candidate set. It estimates finite
burn duration, charges L1 impulse fuel, learns directional acceleration from
telemetry and uses the unchanged radar/camera trajectory checks. It adds no
attitude command, artificial component bias, hard budget or altered jet selector.
Unlike CLEAN_MAIN_BURN it applies the lateral impulse the selected trajectory
actually needs during the same burn. The vector direction is held within the
burn; only a new scored plan can reopen a completed burn. Both new flags are off.
The vector v1 pilot is diagnostic only: a missing completed-burn latch was found
and corrected before the v2 cohort. V1 source/results must not be mixed with v2.
V2 source is tests/score_sweeps/vector_burn_v2_controller.py. Tests pass 140/140.
The 600 m baseline median is 156.785 s, 41.684 kg and 5.848 m/s; none of its
20 seeds crossed 600 m under 35 kg. Full comparisons are pending.

The inertial-capture cohort completed 20/20 with no modeled contact. Median
score 39.09 vs38.275, entry T264.82 s, fuel42.875 kg. The paired bootstrap
interval for the median change was [0.01,2.89], marginal at the legacy score's
0.01-point precision. After allowing median-difference rounding it includes
zero; this is not a robust promotion. Mean paired fuel change -0.5145 kg and
time change -0.2835 s; paired fuel median -0.15 kg, time median zero.

EFP_EXACT_ARCS=1 is a third separate experiment. The original detour adds an
initial lateral impulse and a -2x return impulse as if they were instantaneous.
Finite-burn fixtures showed endpoint errors 13–17 m. arc_schedule instead
subtracts the return burn's actual displacement and solves the first burn to
the existing endpoint, moving the return start after the first burn if needed.
Only the internal constant-gravity detour algebra changes: no gravity-gradient
correction, new route cost, new camera guard or budget. A 20-seed cohort runs
from tests/score_sweeps/exact_arcs_controller.py with other flags off. All 142
tests pass, including endpoints for all six axis orders and infeasible returns.

The arc V1 cohort contains at least one incomplete arrival (seed105), so it is
rejected irrespective of other scores. A second execution mismatch was found:
the old controller did not explicitly start its planned return impulse at the
counter time, relying on a subsequent replan. V2 now starts a pending return
once, after the first burn completes, then retains the remaining return as a
normal finite burn. A better sensor-checked successful coast can cancel an
unnecessary pending return. This completes the two-stage arc's execution;
V1 and V2 source/results are kept separate. V2 frozen source is
tests/score_sweeps/exact_arcs_v2_controller.py, outputs score_arcs_v2_SEED.json.
Tests cover return timing, no interruption of unfinished thrust, and no repeated
return impulse. All 148 tests passed before an additional summary-prefix test.

The vector V2 cohort finished 20/20 with no modeled contact, but median score
fell to36.375, time249.755 s, fuel46.74 kg. Fuel at600m median44.125 kg.
Median score delta -1.90; rounding-adjusted bootstrap interval [-3.19,1.37].
It did not establish improvement and is rejected for promotion. Translational
simultaneous burns changed encounters and increased corrections enough to
outweigh their shorter time; they do not inherently reduce L1 fuel cost.
See tests/score_sweep_vector_v2.json.

Both exact-arc cohorts completed with 19/20 arrivals, zero modeled contacts
and seed105 stalled in WAIT_SHIP_CAMERA at about95m. V1 successful-only median
38.68; V2 successful-only median38.36, with no executed return events in the
final selected trajectories. Both are rejected for an incomplete run. The extra
executor fix did not resolve the actual camera stall. The failed V1 seed used
30.634kg while waiting for camera. That evidence, not a claimed return failure,
motivates a separate guarded camera-recovery experiment.

EFP_CAMERA_RECOVERY=1 selects ViewRecoveryAttitude. During a sensed clear coast
between85 and600m, it can make a limited attitude adjustment that brings a ship
bearing beyond the gimbals into a35deg inner camera cone. It uses bounded
alpha0.003rad/s2 and the published low-duty /60 mapping, then settles rate.
It starts no view turn during translation thrust, imminent ship passage,
missing radar, blocked coast, damage failure or after entry. Radar certifies
the next12s and the unobserved80m ship guard remains. Fresh station coverage
ends the request. This is not continuous nose pointing. A pilot with exact
arcs plus camera recovery on failed seed105 is in progress. All155 tests pass.

EFP_COAST_WINDOWS=1 is another isolated test of the user's early inertia idea.
Before600m only, with forward closing speed>=1.5m/s, the finite-burn grid offers
0/1/2/4s coast prefixes. Full time/fuel scoring and sensed swept-path checks
choose whether waiting for moving debris beats a lateral detour. Retained
plans preserve their absolute burn start; no sliding wait or fuel cutoff.
The bounded one-radar-sweep option differs from older single/two-seed delay
trials that used other guidance/reserve overrides. Its20-seed test uses
tests/score_sweeps/coast_windows_controller.py with all other flags off.
Outputs are tests/score_windows_SEED.json. No variant has been promoted.

New native evidence: three runs with exact CRLF source hash matching committed
baseline b524b36 (9240452c...) score5.1/9.7/14.2, T297.73/309.31/289.87 s,
fuel63.92/59.30/57.90 kg, zero entry damage. The large early lateral steering
cost is measured; its cause is not proven. See tests/inertia_native_observations.json.
It is not a 20-run native median. Body command/rate and remaining impulse
telemetry are now logged to help distinguish control timing and vehicle motion.

An opt-in benchmark --rng-mode native-order now uses numeric debris order,
separate spawn RNG8193, radar seed from --seed, and the official closest-range
noise draw. Legacy mode is unchanged and used for all paired cohorts above.
The native-order baseline pilot100 scored42.24, T267.28 s, fuel39.49 kg. It
does not close the native gap: noise ordering alone is insufficient. No flight
improvement is claimed from that model-only calibration. Do not mix its records
with legacy-mode trials. Native mesh, dynamics, timing and camera calibration
differences still need verification.

Controller.py and its original tests have been restored from 4a8507e.
All bypass and fuel-limiter code has been removed. The required unconditional
change is zero thrust after safe-zone entry. This is causally after the entry
score. A regression confirms that fuel above 50 kg does not inhibit commands.

The first default-off experiment is EFP_CLEAN_MAIN_BURN=1. It adds a body-X
component of 0.0002 times the primary body-Z command direction during positive
Z shooting burns, normalizing to the existing acceleration limit. It does not
change other axes, reverse burns or the native jet allocator.

An independent allocator probe with both HiGHS and the official revised-simplex
method found about 0.13468 m/s² forward acceleration versus 0.0958134 for the
pure Z command. The fuel/forward-delta-v ratio is 4.32315 versus 4.32233 kg.
This is numerical evidence from the published allocator, requiring flight
validation. The small perturbation changes which degenerate LP solution is
normalized by the published max-on-time remap.

Baseline seeds 100–119 ran against the exact original 4a8507e source,
using benchmark_final_submission.py with --ground-plane 2.5 --fuel-audit,
60 Hz, zero synthetic planning delay and HiGHS. Four shell sessions each run
five seeds and write tests/score_baseline_SEED.json. The sweep summary tool
requires 20 completed records and checks source/scenario/seed consistency.
The baseline completed with 20/20 arrivals and no modeled pre-entry contacts.
Median score was 38.275, p10–p90 22.984–43.789, median entry time 264.97 s,
and median fuel 43.725 kg. Full results are in tests/score_sweep_baseline.json.

The clean-main-burn experiment also completed 20/20 with no modeled contact.
Median score was 40.335. Paired median time/fuel changes were -38.025 s and
+2.345 kg. The 95 percent paired bootstrap interval for the cohort-median
score difference was [-0.815, 7.625]. The gain was inconclusive under the
user's rule, so it is rejected for promotion and remains default off.
This differs from the archived angular-trim experiments by using a tiny linear
component with no added angular control. Its field result nevertheless failed
to establish a robust gain. See tests/score_sweep_clean.json.

The second isolated experiment is EFP_ORDER_DIVERSITY=1, preserving one feasible
candidate for each of six axis burn orders before collision checks. It changes
pruning only. Dynamics, camera, score weights and risk margins stay at the
original settings. Its cohort uses no clean-main-burn perturbation.
Seeds 100–119 completed using frozen source tests/score_sweeps/order_controller.py.
The median was unchanged at 38.275, with 20/20 arrivals and no modeled contact.
Its bootstrap interval for median score change was [-0.07, 2.44]. The lower tail
improved, but a median gain was inconclusive and rejected for promotion.
Results are in tests/score_sweep_order.json. Both experiments remain off.

The third isolated experiment is EFP_RECHECK_COAST=1. A regression reproduces
an original completed burn being retained as IMPULSE_COAST despite a predicted
55.4146 m miss. This flag makes that completed coast ineligible unless it meets
the original successful-pass bounds. It changes only retention eligibility;
unlike the regressed SCORE_COAST bundle, camera reachability, guidance and cost
weights are unchanged. The matched 20-seed cohort completed from frozen source
tests/score_sweeps/recheck_controller.py with both other flags off. All 20
entered with no modeled pre-entry contact. Median score was 37.97, delta
-0.305, with bootstrap interval [-0.62, 0.74]. Median time/fuel were 264.96 s
and 43.54 kg. Paired median time/fuel and every phase-fuel delta were zero.
The regression fix did not establish a flight-score gain. It is rejected for
score promotion and remains off. Results: tests/score_sweep_recheck.json.

Selected implementation: restored original pre-entry SCORE_MOMENTUM, no fuel
caps or reserves, all thrust off after entry, all three experiment flags off.
The current full suite passes 135 tests. Eighty modeled flights completed
across four cohorts; no experimental median gain exceeded paired seed noise.
Native 60–70 remains unachieved and unverified. Do not claim completion of the
score target or enable an inconclusive experiment by default.

Latest user question asks where 4.32 kg per m/s comes from and whether 8/9 m/s
with 50 kg can yield 60 points. The official actuator sums each jet's impulse
magnitude divided by Isp*g0, using 80 s and 9.8 m/s²; mass 3388.7 kg gives
4.32232 kg per m/s of clean thrust delta-v. This is not fuel per second or per
net speed gain. Gravity and opposing corrections matter. At 50 kg, zero damage,
60 points requires T<=156.52 s, roughly 8.02 m/s average over 1255.82 m.
The existing speed ceiling is already 10 m/s; a higher forced speed is not
adopted. A modeled 1-D gravity diagnostic of pure +Z burn/coast gives peaks
8/9 m/s at T211.49/196.16 s, fuel38.66/43.45 kg, score55.20/53.24, before
debris or launch overhead. Freely aligned +X models can exceed 60, but omit
all attitude/alignment costs; previous failed nose-first work is not restored.
See tests/score_speed_tradeoff.json and SCORE_TUNING.md for assumptions.

The native log directory provided by the user is
C:/Users/Asus/AppData/Local/SimDynamX/STPro/Logs. The extracted inventory is
tests/native_score_inventory.json. Available successes contain mixed profiles
and missing seed/source fields, so they cannot establish a matched native
20-run baseline. The latest observed native MOMENTUM_BYPASS runs used a
different source hash from the restored controller.
The reproducible read-only native audit is tests/summarize_native_logs.py.
It handles success fields preceding the success message and retains unknown
source/seed identity; matching original runs still requires native provenance.

### Superseded requests and historical direction

Return to SCORE_MOMENTUM at commit `4a8507e`. The user reports this version
can reach about 50 points in their simulation tests. Improve prediction,
path choice and fuel/time balance toward 65+ points. Restore a reliable baseline
before adopting changes. The user runs and deploys the competition simulation.

The latest request supersedes the preceding staged fuel-budget design.
That earlier design allowed 20 kg before 600 m remaining, a conditional 3 kg
advance, then 30 kg afterward under a 50 kg total cap. Its early coast spent too
much time. Momentum prices fuel through the score. The follow-up request adds
a 50 kg total fuel ceiling without reinstating the staged distance allowances.

The next request prioritizes spending useful acceleration fuel before about
500–550 m remaining, then using inertia for the last leg, targeting 40–45 kg.
After safe-zone entry the user accepts continued coasting and later contact.
Post-entry braking is therefore removed from the active ShootingNavigator.

The latest request provides two side-view screenshots and proposes a curved
route around the dense debris band. It also reports missed safe-zone entries
near 60 m. Keep Momentum as the base, evaluate bypass routes from radar,
preserve the 50 kg ceiling, and improve reliability and score toward 60+.

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
  within 50 m and freezes entry fuel statistics. Active ShootingNavigator then
  coasts with all thrust off. The legacy capture classes still contain their
  historical braking behavior for comparison tests.
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

### Radar-ranked bypass and capture corrections

The active profile is MOMENTUM_BYPASS. The screenshots suggest a useful route
shape but provide no world-space metric boundary. bypass_impulses projects the
observed radar band into a plane perpendicular to the destination and compares
12 side directions with five lateral impulse sizes. Constant-velocity forecasts
rank these broad corridors. The actual finite-burn candidates receive the
existing gravity-aware swept checks and expected-damage pricing.

bent_schedule solves both the initial and scheduled return burns so the
constant-gravity endpoint rejoins the destination. The previous detour
construction added a lateral impulse without compensating the endpoint for
finite return-burn duration. The flight loop also lacked an explicit scheduled
return transition. begin_return now executes that transition between planning
updates. A new plan can replace it when a cheaper safe trajectory is found.
The original startup steering is retained while a burn is in progress. Broad
rerouting and the new exact bend schedules require at least 20 mature tracks
and a settled burn, avoiding the startup regressions seen in the early trials.

Both legs count toward projected fuel, and new bent plans must project at
most 47.5 kg cumulative use. When a safe direct candidate exists, a bend must
offer over two points of forecast benefit. Thus the planner can select
an open corridor without forcing an expensive full-field skirt.

Capture targeting moves from 47 m to 43 m inside 550 m. A final coast must
predict penetration at least 6 m inside the 50 m scoring sphere. Finite-burn
planning remains available inside 280 m and adds 5–119 s arrival-time options
for short corrections. Known station surfaces are checked only as far as first
entry, retaining the requested post-entry coast.

At 42 kg, ordinary boosting may pause while outside 550 m if both the handoff
and safe-zone entry are predicted reachable and the next sensor chunk is clear.
This rule preserves fuel while an aligned trajectory coasts. It does not
inhibit corrections for a predicted miss. The total cutoff is still 49.5 kg,
with a 0.5 kg latency allowance. Nose attitude remains free. Success depends on
position and velocity trajectory through the safe sphere.

Native evidence reviewed for this change includes
261118_072411_FINAL_SUBMISSION__294. Its allowance was exhausted at 985.8 m
remaining and 117.7 s. It subsequently coasted to 89.1 m before departing.
261118_071931_FINAL_SUBMISSION__449 recorded official success with 45.0 points;
its later coast and damage are distinct from that recorded result.

The first bypass prototype used 45.58–49.51 kg across four modeled flights,
with scores 29.27–34.69. A trial with a blind 550 m reserve coast plus stronger
fuel/density penalties produced one missed entry and three low-scoring arrivals.
Both tunings were rejected. The reserve gate now requires a successful final
pass, and the two-point benefit threshold suppresses unnecessary bends.
Final guidance also uses the same origin-to-origin vector as Scoring.py inside
550 m, rebased onto the vehicle control-state location. This addresses potential
actor/COM offsets during planning as well as during arrival detection.
The exact API expression is tested in the real-loop double. The native offset
magnitude was not measured in this task.

| Case | Entry fuel | Time | Offline score | Previous score |
| --- | ---: | ---: | ---: | ---: |
| Nominal 100 | 43.09 kg | 261.60 s | 38.42 | 38.05 |
| Nominal 101 | 45.81 kg | 257.27 s | 35.57 | 37.66 |
| Drift 100 | 44.03 kg | 263.50 s | 36.83 | 36.95 |
| Drift 101 | 45.24 kg | 269.83 s | 34.15 | 35.24 |

All four entered without modeled pre-entry contact and stayed below 50 kg.
Mean score is 36.24 versus 36.98 for 731c8df. This is a capture-reliability
update with a small average modeled score decrease. A score improvement or
60+ native result is unverified. Routing remains conditional on clearance,
tracking quality, cost and expected score. The tests do not demonstrate a
complete skirt around the whole debris cloud.

144 tests pass. Coverage includes exact bend endpoints, timed return execution,
open-side radar ranking, reserve reachability, short terminal correction,
grazing-pass rejection, station contact after entry and scoring-origin offsets.
All 20 trial flight records and the four matched baseline records are in
`tests/momentum_bypass_benchmarks.json`. The final source SHA256 is
`7a2fd955b49bef8ed639912893e8b07f8cd674fd816c040037a7763ee3960651`.

### Follow-up fix for the stop at 63 m

The user reported a stop at 63 m and excessive corrections near the ship, and
requested fuel use below 50 kg. Comparing restoration commit 5b6a81a with
4a8507e confirmed that radar caching was the only algorithm change in that
restoration. The original burn, capture and camera settings were preserved.

Native log `261118_064018_FINAL_SUBMISSION__7bf` confirms the failure. At
294 m the rover coasted at 5.97 m/s with fuel stable at 42.02 kg. At 206.6 m
it entered WAIT_SHIP_CAMERA, then burned fuel braking to rest at 62.8 m.
WAIT_SHIP_CAMERA accounted for 22.93 kg by 273.2 s and total fuel was 64.96 kg.
This evidence identifies the camera wait as the large terminal expenditure.

With the vehicle traveling along body +Z, the top gimbal clamps at -50 degrees.
The ship is about 40 degrees off the optical axis. An 80-degree field with
a 3-degree observation margin rejects that bearing. The fix requests a
100-degree field in both RGB and depth, using the same calibrated projection
for ingestion, obstacle mapping and coverage checks.

FuelBudget checks the native FuelUsed counter before every live command.
It latches both translation and angular thrust off at 49.5 kg, reserving
0.5 kg for telemetry and actuator delay. The cutoff applies to post-entry
braking too. Invalid readings also inhibit thrust. A simulation-clock reset
creates a fresh budget. Once cut off, the rover coasts under external forces.
If the budget is spent before entry, it can coast short of the safe area.

All 130 unit and adapter tests pass, including actual-runtime fuel inhibition,
budget latching, body-Z coverage at 63 m, terminal permission after camera
coverage and outside-field rejection. Matched drift replays used the native
observed residual [0.0023, -0.0008, 0.0016] m/s².

| Version / seed | Entry time | Entry fuel | Offline score | Final total fuel |
| --- | ---: | ---: | ---: | ---: |
| Original Momentum / 100 | 280.12 s | 41.99 kg | 36.85 | Uncapped |
| Camera + budget fix / 100 | 265.30 s | 43.71 kg | 36.95 | 49.502 kg |
| Original Momentum / 101 | 271.30 s | 58.41 kg | 16.36 | Uncapped |
| Camera + budget fix / 101 | 265.15 s | 46.08 kg | 33.81 | 49.502 kg |

Both corrected runs entered without modeled contact and then exhausted the
budget during post-entry braking. They continued coasting after cutoff, with
final modeled speeds 2.78 and 3.35 m/s. The original seed 101 replay spent
extended time in WAIT_SHIP_CAMERA, consistent with the native symptom.
Records and source hashes are in
`tests/momentum_camera_budget_benchmarks.json`. This verifies two modeled
runs and the coverage regression. It does not establish native success or 65+.
Latest normalized controller SHA256 is
`47b707eb28dfa9721aee911fba3a1cea51b5b0ec9c7e6f49f0eb6dab18029b0a`.

### Coast-focused final leg

Inside 550 m, ShootingNavigator now offers a zero-thrust FINAL_COAST before
the ordinary velocity/speed replanning. A one-second velocity-Verlet rollout
uses inverse-square Phobos gravity plus the coast observer's local residual
and the ship velocity. It must predict a pass at least 2 m inside the 50 m
boundary. The first entry is interpolated on the crossing segment.

The next 20 seconds still receive moving-obstacle checks. Station surface checks
end at first safe-zone entry. The approach continues to require fresh camera
coverage inside 210 m. If the prediction misses, the path becomes blocked, or
coverage is unavailable, normal guidance remains active. There is no assumption
that a particular seed is debris-free after 500 m. Speed naturally changes under
gravity while the jets are off.

After entry the live loop inhibits attitude thrust and ShootingNavigator returns
zero translational thrust, including between planning updates. Its mode becomes
MISSION_COMPLETE_COAST. The existing 49.5 kg command cutoff remains the total
fuel safeguard. A 40–45 kg target is an optimization goal.

The installed official Scoring.py reads damage before checking success on each
scoring tick and stops updating after missionSuccess. Consequently, damage
observed before the winning check reduces the score or can cause failure.
Damage after recorded success is excluded. A zero-damage 60-point result
requires at most 189.47 s at 40 kg or 171.43 s at 45 kg.

136 tests pass, including predicted entry versus a miss, insufficient outbound
momentum, first-entry path truncation, moving-obstacle rejection, a station
surface beyond entry, and cancellation of a powered burn upon arrival.
Full-scene comparisons are recorded below. The benchmark now
records contact at entry independently of contact during the post-entry test.

| Case | Previous entry fuel | New total fuel | New entry time | Previous score | New score |
| --- | ---: | ---: | ---: | ---: | ---: |
| Nominal seed 100 | 44.95 kg | 43.293 kg | 262.18 s | 35.80 | 38.05 |
| Nominal seed 101 | 45.54 kg | 44.449 kg | 255.70 s | 36.23 | 37.66 |
| Native-drift seed 100 | 43.71 kg | 43.710 kg | 265.30 s | 36.95 | 36.95 |
| Native-drift seed 101 | 46.08 kg | 45.023 kg | 265.05 s | 33.81 | 35.24 |

All four entered without modeled contact. They used 0.235–1.282 kg between
500 m remaining and entry. Recorded entry fuel and total fuel stayed equal
through the 90 s post-entry test. Arrival time changed by at most 0.2 s
against the preceding camera/budget version. The main savings are reduced
terminal corrections and removal of post-entry braking. A 60-point result
remains unachieved in these models.

All eight comparison records, source hashes, stage milestones and saved baseline
source are in `tests/momentum_final_coast_benchmarks.json`. Drift baseline
records reuse the identical previous controller and matched settings from
`momentum_camera_budget_benchmarks.json`. These are offline proxies with
zero synthetic planning delay. The controller source hash for this version is
`463c22f0968f86278dac5c3dcc6079e896a3b19c42291fba01ed87c106476397`.

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
