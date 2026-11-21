# Escape from Phobos project context

Updated 2026-11-21. Read this file before controller changes and update it after
each tested change. It is a maintained summary of the available chat history,
repository history, guides, archived experiments and local simulation logs.
The original chats and logs remain the sources for details.

**Uploaded final-submission evidence, reviewed 2026-11-21 (results addendum):**
The user supplied `finalsubmission2.zip` from the simulator Logs directory and
states it was uploaded as the final competition submission. Two distinct native
successes are present: `261120_043631_FINAL_SUBMISSION__73e` scored48.4 at221.90s,
41.76kg/0J/49.81m; `261120_044029_FINAL_SUBMISSION__ff0` scored47.1 at213.89s,
44.25kg/0J/49.88m. Submitted-pair mean47.75, time217.895s, fuel43.005kg,
both0 scoredJ. Both startup hashes match selected Controller.py4210afa4...32834
and its active defaults; both archived scenario hashes match the protected
official configuration. This pair achieves the requested45+ average, not55+.
The earlier three-run development mean41.2667 below remains historical and
separate; do not pool it into the submitted pair or claim a paired algorithm gain.
Native seeds/background workload are unknown. Logs do not verify organizer
acceptance or ranking. Fuel is unchanged from sampled distance605.4m/595.8m
through entry in the two submitted runs; final coast behavior is supported.
Archive SHA256006ca4a9479e0657a34b0210175330da81823455e5ffe0007782dcd975449e12.
Evidence with canonical Scoring lines, startup flags, member hashes, UUIDs and
selected telemetry: [docs/finalsubmission2_results.json](docs/finalsubmission2_results.json).
[Final Submission Algorithm](Final%20Submission%20Algorithm.md) now leads with
these submitted results, retaining development/offline data separately. No
controller/source/simulator changes or additional native flights were made.

**Documentation reorganization, 2026-11-21; no controller changes:**
The user requested an archive of superseded Markdown and a complete technical
description of the final algorithm. Seventeen historical documents now reside in
[previous works](previous%20works/README.md), including the original Momentum,
Dogleg, qualification, and older tuning descriptions. Their historical status
is explicit; the SCORE_MOMENTUM name alone does not identify current code bytes.
[Final Submission Algorithm](Final%20Submission%20Algorithm.md) is the maintained
architecture/equations/control-flow/validation reference. README setup,
CAMERA_GUI.md, this context, AGENTS.md, and the latest
[November 20 experiment report](tests/nov20_experiment_report.md) remain outside
the archive. README links point to the paper and archive; camera documentation
now correctly describes zero thrust after entry. Source, tests, JSON evidence,
and official simulation files were not changed. This is documentation work,
not a new flight experiment or evidence of a higher native score. The selected
controller and measured results in the finalized entry below remain authoritative.
A read-only check on 2026-11-21 finds the installed STU4-Solution Controller.py
now has the same selected 4210afa4...32834 hash. The older warning below about
the installed ebcc...6285b source is historical; this task did not update it.

**FINALIZED 2026-11-20 at the user's deadline instruction (overrides all earlier snapshots):**
The user asked to finish now with the best completed average. Keep root
Controller.py exactly SHA2564210afa43e5a2570eb3fcd12c6e276c8697a69da631931fb1abeac3613332834.
It matches tests/score_sweeps/nov20_conservative_clock_native_controller.py
byte for byte. Selected SCORE_MOMENTUM uses strong-X/cache/radar cruise/
expected risk/fine avoidance plus confident radar-position smoothing, compiled
identical risk and existing radar clock estimator. Physical geometry OFF.
Zero-offset clock locking preserves correct tracks. No hard fuel limit,
new route planner or braking after actual safe-zone entry was introduced.

Exact-source quiet native scores45.1/40.1/38.6, mean41.2667; time222.17/
232.52/240.13s; fuel44.22/46.10/46.04kg; all0 scoredJ. Mean fuel45.4533kg,
mean time231.6067s. Evidence tests/nov20_final_native_validation.json.
The45+ average minimum and desired55+ were NOT demonstrated. Do not claim
robust improvement over the user's submitted40.5 average or a45-point floor.
All76 targeted tests passed on this exact source. Final hash,2829-byte
protected helper prefix and all six official/installed file hashes verified.
Evidence tests/nov20_finalization.json and tests/nov20_experiment_report.md.
The user's pre-existing temp_controller/currentcontroller.py edits are untouched.

Rejected follow-ups: physical-geometry native46322.1J collision/aborted run;
early fine20 proxy seeds unchanged and native42.7/36.8 mean39.75; range-fusion
pilot45.55 vs50.48; coast-priority20 paired mean−.0395/CI[−.4515,.3000]
despite mean fuel−.8625kg/time+5.1165s; powered-drift assumed-bias pilot100
44.38 vs46.66 (less fuel but17.48s slower). None promoted. Native coast-
priority configuration was prepared but never flown. Wider49m band and
precision-impulse taper are isolated, unflown/unselected prototypes; precision
has no completed tests. Never silently deploy these archived experiments.

Conservative diagnostic-only source ae4d054b...fb9d5 scored37.0/225.37s/
49.67kg/0J. Separate telemetry/counterfactual files distinguish measured own
states from modeled unpowered passes; no debris/hull certification or native
score gain follows from the model. Some corrections were necessary (66.57m
modeled miss at81s). Do not pool the changed-source diagnostic into the
identical-byte cohort or conceal it when discussing native variability.

User-supplied official54.0 excerpt:203.39s/41.24kg/0J, SimRunID
a6a82e84-43ed-2c36-31a3-7eb9da19f958. Exact source/seed/original local file
unknown; nov20_user_supplied_54_result.json. It is an individual result, not
a new selected-source trial/average. User-started old installed baseline
032829...e44 scored12.6/249.50s/64.13kg/0J and is recorded separately.

All benchmark processes finished. Official FINAL_SUBMISSION selection was
restored through the UI; the simulator was left idle at its main menu. Protected official STU4-Sim,
jet allocator/CSV/scoring and installed STU4-Solution source remain unchanged.
No GitHub push/commit or competition submission. User must push local EFP
Controller.py and Pull the solution through the simulator to deploy this source;
the installed checkout still has the earlier ebcc...6285b controller.

**Reopened optimization task, 2026-11-20 (overrides the old finalization below):**
The user submitted a reported 40.5 average and requests a native average above45,
with55+ as the desired target. Reduce unnecessary acceleration/avoidance fuel;
50 kg is an efficiency target, not a hard fuel blocker. Native testing is
authorized. Keep source edits in this EFP repository and official software,
physics/scoring/configuration and the installed solution checkout unchanged.
User's existing uncommitted temp_controller/currentcontroller.py changes are
outside this task and must be preserved.

IN PROGRESS: baseline bytes ebcc0494...6285b are frozen in
tests/score_sweeps/nov20_baseline_controller.py. A fresh20-seed offline baseline
exactly reproduces median45.15, p10-p90 40.485-48.691, time220.33 s,
fuel43.475 kg,20/20 arrivals/no proxy contacts. This is NOT native performance.
Fresh unchanged native261119_212710...95c scored32.1/232.55 s/52.12 kg/0 J.
An expanded audit reads source hashes from the separate PySys_Controller log;
18 completed exact-source records have mean24.53, median28.3, range-9.2..49.6.
This is a historical log sample, not a matched seed cohort or the submitted
average. Evidence: tests/nov20_native_inventory.json.

Two small changes are present in Controller.py, BOTH DEFAULT OFF while tested:
EFP_RADAR_SMOOTH_POSITION uses an existing confident regression projected to
the latest report timestamp, retaining raw associations and all uncertainty/
radius bounds; EFP_FAST_RISK uses scipy.special.erfc for the SAME probability
model, with the original math implementation as ImportError fallback.
Protected connection/SetCommand/camera helper prefix remains byte-equivalent.
Four new sensor/numerical tests,9 capture tests,3 reset/config tests and the
existing tracker-cache equality test pass so far. No promotion/score claim yet.

Native candidates run through isolated copies of the official scenario in
Assets/Sims/EFP_Development, with ONLY Controller Source changed to a frozen
file inside EFP. They do not modify STU4-Sim or the installed solution Git repo.
Local test configs: tests/native_trial/nov20_*.simconf. Official protected file
hashes: tests/nov20_protected_hashes.json. Restoring the launch selection to
the official FINAL_SUBMISSION.simconf is required when native testing ends.

Experiments so far (partial; complete records override these notes):
- Accept-safe-pass alone: two early proxy seeds saved1.93-2.20 kg; matched20
  cohort still running. Native261119_213649...5eb completed-5.3/251.07 s/
  77.29 kg/0 J. No native gain established; do not select it on that result.
- Useful early-lateral candidate screening pilot100:41.65 vs48.79 baseline,
  fuel51.69 vs42.22. Rejected; do not port this prototype into active code.
- Body-axis L1 fallback fuel pricing: seed101 identical;10540.63 vs40.70.
  No useful gain; rejected for selection. The mathematical cost mismatch
  alone does not demonstrate a flight-score improvement.
- Rechecked calibrated+Z pilot10440.74 vs44.96; prior four+Z pilots also
  mixed. No evidence to replace the +X configuration; no new native+Z trial.
- Radar smoothing partial cohort is promising, but is not complete. Native
  attempt261119_214620...c95 failed during lobby startup (jet-selection load
  timeout, no flight/score); retry261119_215046...499 is running. Native
  score and a full matched cohort remain REQUIRED before selection.
- Vectorized risk90x80 microbenchmark:2.7299 ms ->.2128 ms median, maximum
  probability error1.11e-16; tests/nov20_fast_risk_timing.json. This measures
  compute only. Its20-seed equivalence cohort is running, not a score gain.

Progress update: smoothing completed20/20, median46.8 (baseline45.15),
paired median+1.86/bootstrap95%[.385,2.735],0 proxy contacts. Median paired
fuel-.67 kg; mean fuel44.6015->43.799 kg; median phase main-burn delta-1.4462
kg. Median entry time220.36 s. This is a small modeled gain, NOT native55.
Native smoothing499 completed40.8/234.86 s/45.19 kg/0 J. Combined smooth+fast
native261119_220629...35a completed39.2/237.50 s/46.01 kg/0 J. All exploratory
native runs so far had background benchmarks active; final validation should
run WITHOUT benchmarks. Simulator currently idle at its main menu.

Vectorized risk20 seeds exactly match baseline score/time/fuel/contact results;
this confirms policy equivalence in that cohort, not a native score improvement.
Accept-safe-pass20 median45.67, paired median+1.155/bootstrap95%[0,2.43]:
gain is not robust under the specified gate. Keep it OFF; no selection from it.

An additional opt-in EFP_PHYSICAL_RISK (DEFAULT OFF) is now ported alone from
the archived physical-geometry experiment. It records measured diameters,
uses .5*diameter+existing3 m vehicle allowance as physical core, puts5% diameter
noise into variance alongside unchanged2 m position sigma, and preserves
camera volumes, full m*v2 impact energy, .01 fatal probability and .5 m hard
clearance. This is being revisited ONLY on the improved radar-position tracker;
earlier geometry-only native trials were inconsistent. No powered launch,
kinetic-impact relaxation, camera removal or hard fuel limit is added.
Tests/test_nov20_physical_risk.py covers the active source geometry/covariance
and unchanged camera blockage. Experimental source is
tests/score_sweeps/nov20_smooth_physical_controller.py;20 matched seeds are
running (100 pilot +101-119 cohort,113 reused). Native variant is prepared but
NOT flown:nov20_smooth_physical_native_controller.py; native scenario staged
in Assets/Sims/EFP_Development/EFP_NOV20_SMOOTH_PHYSICAL.simconf. All three
new flags remain OFF in active Controller.py pending selection/validation.

Physical prototype pilots:10048.87/210.03 s/43.70 kg vs smoothing50.48;
11346.73/201.55 s/47.11 kg vs smoothing31.77. Do not cherry-pick the latter.
Revisited+Z with smoothing pilot10050.51 vs+X smoothing50.48: no material gain;
no +Z promotion/native trial. The frozen all-options current-source equivalence
pilot100 is running before any final native selection.

Tests:7 new active-source radar/geometry/numerical checks pass; core controller
20, camera fusion17, camera GUI11, capture9, runtime profiles3, risk avoidance3,
tracker-cache1 passed at their stated interim revisions. Re-run appropriate
current checks after final selection. Historical shooting/economy suites show
the SAME15 errors/1 failure on the original baseline and current source;
tests/nov20_historical_test_comparison.json records this. Do not claim all
historical tests pass, or edit the user's temp_controller file to repair them.

FINAL VALIDATION IN PROGRESS: all20 physical/covariance seeds finished with
0 proxy contacts, median48.805/220.975 s/41.495 kg, mean score48.374,
fuel range34.95-47.11 kg. Increment versus smoothing alone: paired median+2.17,
bootstrap95%[1.10,3.44]; versus baseline+4.51,[1.025,5.79]. Median paired fuel
-2.99 kg and time+1.185 s versus baseline. Full results and phase deltas are
tests/nov20_matched_summary.json (all20, including the separately run seed100).
The ported current source exactly reproduces every non-timing result field of
the physical prototype on seed100:nov20_active_source_equivalence.json.

Based on that gate, the staged native candidate now defaults ALL THREE new
flags ON. SHA2561aec9ebf31b037e7693461512798941b9ba941375f87717783b5cb644989800a,
frozen tests/score_sweeps/nov20_selected_controller.py.71 targeted current tests
pass on this exact source:tests/nov20_targeted_tests.json. No further source
edits during the native cohort. Official files/installed solution hashes are
unchanged; protected connection/command prefix verified identical RAW bytes.

All offline benchmarks are finished; process inspection confirmed none running.
REJECTED NATIVE GEOMETRY PROFILE: quiet run222652...d62 took46322.1 J at about
66-71 s, then spun and burned77.71 kg by171.4 s. It was aborted AFTER the
immediate-arrival upper score bound fell to-27.89 (so45 was impossible even
with instant free arrival). NO official finish score exists; do not invent one.
The20 clean proxy runs did NOT establish native safety. EFP_PHYSICAL_RISK was
immediately restored to default0. Keep the implementation opt-in for evidence,
but DO NOT select/re-enable it from the modeled median. Full record retained
in nov20_final_native_validation.json. The old1aec... hash is REJECTED, not final.

New issue under investigation: Radar.py sets t0 AFTER its initial500-object API
scan; Controller has a different epoch. Native fresh-report ages5.8-25.9 s
were1.182-1.487 s (later up to1.7 s), including publication batching/transport.
This is NOT a measured exact epoch offset or proof of collision causality.
Revisit the existing EFP_RADAR_CLOCK estimator on the improved tracker with
CONSERVATIVE geometry, because previous native calibration measured only.258 s
and its results were inconsistent. New small fix: zero-offset locking does not
erase correct tracks; nonzero retiming still clears the wrong-epoch cache.
Five radar-timebase tests pass, including both cache cases.
Clock remains default0 during stress pilots. Frozen source:
tests/score_sweeps/nov20_clock_controller.py. Three pilot processes are running:
nov20_clock_zero100 (clock on,0 skew), nov20_clock_skew_base100 (off,1.3 s
synthetic skew), nov20_clock_skew_fixed100 (on,1.3 s synthetic skew).
These are ASSUMED stress cases, not native clock measurements. Wait for their
results; do not run native trials while those benchmarks are still active.

Quiet native validation uses Assets/Sims/EFP_Development/EFP_NOV20_FINAL.simconf,
with ONLY Controller Source overridden to E:/EFP_SUST-IGNITION-main/Controller.py.
It therefore tests the EXACT user development file, not a simulator-repo copy.
The rejected222652...d62 startup hash matched and flags smoothRadarPosition/
fastRisk/physicalRisk were allTrue. Physical risk is NOW OFF; root hash has
therefore changed. Full impact energy remains1.0, mainAxis0, noseFirstFalse.
Keep simulator focused during flights. Goal is THREE consecutive trials of the
same source, record every score (no selection of best trials). Completion still
requires documenting whether mean>45; desired55+ remains unachieved. Records:
tests/nov20_final_native_validation.json. Source must stay pinned throughout.

Pending: finish clock stress pilots, then stage/verify a conservative candidate
and complete a fresh quiet native cohort. Calculate its average/fuel/damage, update
report and this context, recheck protected hashes, and restore simulator launch
selection to official FINAL_SUBMISSION.simconf at its idle main menu. No push,
competition submission or installed solution update is authorized/needed here.

**Repository workflow repair (2026-11-19):**
The development source of truth is `E:/EFP_SUST-IGNITION-main`. Make future code
changes there. The user commits/pushes that checkout to GitHub, then uses Pull
in Space Teams to update its separate installed `STU4-Solution` checkout.
Do not copy/edit controller files directly in the simulator checkout for normal
development; the user explicitly requested this workflow after native test
copies caused a separate simulator-side commit and Git divergence.
The simulator-only commit f00046b had the SAME Controller.py blob as GitHub
1299799. Its clean main was aligned with origin/main1299799; verified pull says
Already up to date. Backup branch:codex/backup-stpro-f00046b in the simulator
checkout. Complete recovery bundle: `.git/stpro-recovery-20261119.bundle` in
the EFP development repository. Simulator pull.ff=only. No GitHub push or
controller/official-simulation change was made during this repair.

**Live development snapshot (overrides older interim descriptions below):**
USER REVISED COMPLETION TARGET: finish as soon as practical; a native average
of47-50 points across TWO runs is acceptable. Native55 is no longer the
completion gate. Do not continue adding experimental features for55.
FINALIZED AT USER'S EXPLICIT REQUEST. The user chose to finalize the best
verified code now despite the missed47-50 two-run average, ending further tuning.
Controller.py retains the Momentum configuration associated with the earlier
native47.2 result (261119_140256...7b6, source dcf29c02...), plus the verified
zero-thrust behavior after confirmed safe-zone entry. No score gain is attributed
to that post-entry fix. Current byteSHA:
ebcc04946087543faa17bd7caf3b8401c7227aa524041b1cff717fd95f36285b.
Installed solution and frozen tests/score_sweeps/final_momentum_controller.py
match the workspace file. Selected defaults:strong-axis (+X), radar-cruise,
expected-risk, fine-avoidance, exact track-cache. Other flight options off.
Final TWO fresh runs:43.5/211.84 s/47.39 kg/0 J and24.2/239.49 s/56.96 kg/0 J;
mean33.85. Both startup hashes match the selected source. The requested average
was NOT achieved. Full records:tests/final_native_validation.json. Keep this
limitation alongside the historical47.2 best; no stable47-50 claim is justified.
The later local-clearance reduction scored28.8 and was rejected/rolled back.
125 targeted current+archived checks pass for the selected source. Archived
prototype tests explicitly load their frozen snapshot. Historical retired
Dogleg/trace adapter tests remain outside this claim. Protected connection and
camera-command helpers, official simulator/config/scoring/allocator/CSV verified
unchanged. Simulator is stopped at main menu; no benchmark is still running.
No commit, push or external competition submission was performed.
Future work, only if requested: reproduce native performance over more runs,
then investigate the main-burn correction fuel and native/proxy mismatch.

Latest isolated findings:

- Final fresh trial1 `261119_170146...5a8`, exact final hash ebcc0494...:
  official43.5/211.84 s/47.39 kg/0 J. This is BELOW the relaxed average goal.
  Record it without omission. Trial2 will use unchanged source. Validation
  JSON tests/final_native_validation.json stores both rows and eventual mean.
  To average>=47 with this first result, trial2 must score>=50.5.


- Selected controller and explicit archived-experiment regression checks125
  pass after restoration. Tests of rejected features load their frozen archive;
  active runtime reset checks validate ONLY the chosen final configuration.
  Protected connection/camera command helper prefix matches baseline exactly.
  Native-final validation is pending; no average claim yet.


- Native soft+physical trial `261119_164814...dbd` is running, installed hash
  bb7184df...fdd7a02d confirmed in startup log. At207.6 s/204.7 m remaining,
  fuel51.15 kg,0 J: target55 will not be met. Await official entry result.
- Soft+physical pilot10049.50/227.93 s/39.86 kg/no contact versus soft-only
  43.50/255.80 s/40.06 kg. Single seed only; no native gain established.
  Next prepared +Z profile changes body main axis and corresponding allowed
  calibrated +Z request; retains all risk settings. Offline pilot running.
- Signed-axis/torque model found no useful alternative to +X or small +Z turn.
  +X stage15.02 s/6.129 kg/clean modeled47.81; calibrated negative-Y stage
  14.7 s/5.735 kg/48.01. No material gain. No negative-axis flight code added.


- Departure-risk20 seeds complete: median45.215 vs45.15 baseline; paired
  median delta0, bootstrap95%[0,0].19/20 seeds exactly match baseline; seed102
  +.28. No gain beyond seed noise; reject for selection despite unit fix.
  Native549 -13.0/258.72 s/82.03 kg/0 scored J is recorded without assigning
  causality from an unmatched single trial. Station damage happened later.
- +Z soft-risk-gates pilot10050.09/254.05 s/35.36 kg/no contact versus51.11
  baseline: no pilot gain. Opposing angular-request actuator model found
  no main acceleration gain above .134676; rejected, no flight code added.
- NativeX soft-risk gates trial `261119_163424...2ca` is running, installed
  hash96f5125b...6edf0. Next prepared candidate changes ONLY physical-risk,
  no threshold/energy/slew/camera changes. Pilot is running before selection.


- Soft-risk-gates pilot10043.50/255.80 s/40.06 kg/no contact vs48.79:
  no pilot score gain. Native test is exploratory because the main target
  requires actual simulator evidence and proxy behavior has differed markedly.
  Do not promote this flag from that pilot. A matching +Z profile pilot is
  running, changing only soft-risk-gates from prior +Z baseline51.11.
- +Z departure-risk pilot100 is exactly51.11/251.43 s/34.96 kg baseline;
  no demonstrated gain and no native promotion. A mixed-version full native
  log inventory found18 tagged Momentum successes, median25.25, max47.2;
  that is NOT a matched native cohort. No historical native55 result found.


- EFP_SOFT_RISK_GATES default0 changes ONLY finite-shot probabilistic
  eligibility, retaining clearance>.5 and every existing geometric bound.
  Expected damage and recovery-time costs remain conservative m*v2.
  Instead of binary fatal-probability<.01 and expected-damage<.2*max gates,
  it adds expected failure-category cost from the verified official Scoring.py:
  failure replaces120/T with negative remaining/initial distance. Category
  penalty=min(sum(fatal probabilities),1)*(12000/T+100*distance/initial).
  Distance at the present state is a conservative approximation of future
  failure distance. Contact probability/energy/recovery are still MODELS.
  This differs from rejected local-expected-risk: local fallback is unchanged;
  unlike kinetic-impact, impact energy is NOT reduced. Two gate/geometry tests
  and2 runtime-reset tests pass. No native or median improvement is claimed.
  Baseline/source and fixed clearance permit an isolated score test.


- EFP_DEPARTURE_RISK default0 fixes an inconsistency: the existing monotonic
  departure rule ignored geometric overlap but still charged probability,
  energy and recovery. Only radar objects with CURRENT estimated radial
  separation rate>2*forecast_rate are exempted on departing segments. Earlier
  approaching or later returning segments keep their original risk. No hard
  clearance/radius/margin is reduced. A regression test reproduces >40000 J
  phantom expected damage behind the vehicle and a cancelled clear burn;
  the flag removes that charge. Three tests pass, including front obstruction
  and uncertain slow recession retained; runtime-reset test passes. Native
  score benefit remains UNVERIFIED. Seed100 is identical to baseline.
- Lookahead-fuel pilot100 exactly48.79,217.88 s,42.22 kg baseline: no gain
  on this seed. Two path-choice tests pass, but do not promote the prototype.


- EFP_LOOKAHEAD_FUEL default0 is a new PROTOTYPE, not deployed. It differs
  from rejected continuous distant-crossing cost: only confidently fitted
  radar tracks, only25 s beyond the unchanged20 s hard horizon, continuous
  segment closest approach, per-object maximum rather than repeated charges,
  and a modeled later two-pulse repair cost in fuel-score units. Geometry
  guards remain unchanged. Assumed repair delta-v=2*penetration/20 s, weighted
  by a Gaussian-tail proxy, capped10 score points. This cost is assumed,
  NOT measured fuel. Seed100 pilot running. Reject if no paired benefit.
- Focused checks123 passed after departure-risk/lookahead additions. Official config/scoring/actuator/CSV hashes
  reverified unchanged. Connection boilerplate remains unchanged.


- IMPORTANT native powered+physical+kinetic `261119_160345...bc7` hit debris:
  154282.6 J, later106.8 s/53.91 kg and severe spin/recovery. Agent stopped
  the trial because55 was mathematically unattainable even at immediate
  arrival. This is an aborted trial with NO official finish score, not a win
  or a synthetic official loss. Reject powered profile for native selection.
  Offline cohort's zero-contact findings DO NOT predict native safety here.
  Do not promote this combination based on its offline median. Native best47.2.


- Energy-model native206.74 s,49.45 kg scored42.1 versus prior250.54 s,
  41.77 kg scored42.2. Faster flight was paid for by extra fuel. No gain.
- Score-entry-lock on powered physical+kinetic seed10048.82 versus48.96:
  no demonstrated improvement. On physical-only +X seed10051.02 versus
  48.18, saving2.11 kg with essentially identical entry time. This is one
  proxy seed; terminal geometry differs from native. Default remains off.


- +X physical-risk native `261119_154445...99d`:42.2 points,250.54 s,
  41.77 kg,0 J. Best remains47.2; this alone did not improve native score.
  Next installed source2132a18b5117e1d3e835f958d9462d1f4ad048bebe98048916895ebbafe06875
  differs only by EFP_KINETIC_IMPACT default1. All geometry/clearance guards
  identical; modeled contact energy uses0.5*m*v2 instead of m*v2.
- Matching pilot100 physical+kinetic45.21/251.48 s/39.38 kg/no contact
  versus physical-only48.18. Adding powered-launch alone then48.96/
  239.82 s/38.31 kg/no contact. These are hypothesis pilots, not promotion
  evidence or a native55 result. Score-entry-lock pilot on latter is running.
- EFP_SCORE_ENTRY_LOCK default0 checks all sensed obstacles ONLY until a
  predicted49.5 m sphere entry, before legacy terminal guidance can inspect
  its100 m post-tangent extension. Required pre-entry clearance>.25 m.
  Unlike earlier entry-coast it checks radius49.5 rather than48 and avoids
  the helper's old minimum1 s horizon when entry is sooner. Four meaningful
  tests pass: post-entry hull excluded, pre-entry blockage retained,
  subsecond interval truncated, no lock on a missed sphere/departure.
  It does not announce mission success before actual official-frame entry.


- Native physical-risk diagnostics found all-shots rejection at41.705 s with
  8.903 m geometric clearance,2392.76 J expected damage, and summed fatal
  probability .01028 above the .01 limit. Closest modeled object sigma3.918 m,
  rate.339 m/s, report age5.408 s, closest time15.36 s. These are predictions,
  not proof of false alarms. The conservative m*v2 energy gate motivates
  testing existing half-kinetic-energy flag on THIS profile next; actual engine
  damage law is still unavailable in the public scoring/Radar Python files.
- EFP_NEAR_RISK_HORIZON default0 changes20 s finite-shot certification to10 s.
  Pilot10035.23/212.77 s/53.37 kg/no contact vs48.79: hypothesis falsified,
  reject for selection. Two risk-diagnostic tests and2 reset tests pass.
- +X physical-risk pilot101:51.39/241.28 s/36.26 kg/no contact versus43.58
  baseline. Together with seed10048.18 vs48.79 this is insufficient evidence
  of median gain; native run remains exploratory. Native55 not achieved.


- Powered-launch native `261119_153631...634`: official21.0 points,
  241.02 s,59.13 kg,0 J. Start/turn fuel fell, but avoidance/escape used
  another9.04 kg plus corrections hidden in main burns. Rejected.
- Now staged native +X/cache plus ONLY physical-risk and behavior-neutral
  rejection diagnostics, SHA b521f5cb1bba864136d884d12722715c0ead8d0dc528c662de10d0dcfa1f25ea.
  Powered launch, efficient-X, force heading and calibrated launch OFF.
  A diagnostics equivalence test passes. Logs include rejected-shot time,
  clearance, expected/fatal risk, closest obstacle age, radius, sigma, rate,
  confidence and relative position/velocity. Do not infer false alarms without
  examining those measurements. +X physical-risk seed100:48.18/211.0 s/
  44.02 kg/no contact vs48.79 baseline; no demonstrated score improvement.


- Force-aligned-slew pilot100:41.83/230.47 s/45.18 kg/no contact,
  versus original strong-axis48.79. Three focused heading tests pass,
  but the flight hypothesis failed this pilot; default remains off.
- Powered-launch-slew pilot101:45.34/239.95 s/41.00 kg/no contact.
  Both powered pilots are below the existing matched strong-axis scores.
- The first powered_efficient_x100 launch used an older frozen source that
  lacked the powered-launch branch, so that artifact DOES NOT test the
  combination. Exclude it from combination evidence. Harness now rejects
  new experiment flags absent from the selected frozen controller rather
  than silently attaching an unused attribute. No claims based on that run.


- EFP_FORCE_ALIGNED_SLEW default0 is a one-time attitude heading screen:
  price constant-gravity finite burns toward the SAME16 tangent endpoints,
  then freeze the chosen force direction for the initial turn. Sensor route
  checks and terminal goals are unchanged. This differs from the rejected
  repeated gravity-gradient endpoint correction: no gradient or repeated
  endpoint servo is added. Pilot is running; no score claim yet.
- Efficient-X offline seed100:46.83/227.20 s/41.99 kg/no contact versus
  original strong-axis48.79/217.88 s/42.22 kg. Small fuel saving but longer
  route time, so no promotion based on this seed.


- Launch-only calibration native `261119_152937...ef1`: official20.0 points,
  247.44 s,58.91 kg,0 J. Launch itself used4.19 kg versus best-run4.83,
  but dense-zone avoidance erased that saving. Rejected for selection.
- EFP_POWERED_LAUNCH_SLEW default0 starts the existing one-time turn during
  lift only for a target within acos(.98) of launch up and the unchanged
  four-second scene clearance guard. A blocked initial turn uses usual launch.
  This differs from the rejected boost/slew experiment, which started AFTER
  lift. Existing low world translation request remains .025.8 slew tests,
  2 runtime-reset tests pass. Offline seed100 scored43.32/239.77 s/42.55 kg,
  lower than original strong-axis48.79. Native pilot is exploratory because
  the offline harness does not reproduce measured native launch delay.
  Native source82c2b7bc2bacfda2fc75840e9c3b80b5a02871f1285afface06089657726bafb
  retains only the best +X/cache flags plus powered-launch-slew; calibrated
  launch and efficient-X are OFF. Do not claim improvement until measured.


- New isolated EFP_CALIBRATED_LAUNCH default0 applies the already validated
  +Z request bias only to a positive-up launch pulse. Official revised-simplex
  model: .095813 to .134676 m/s2, unchanged ~4.322 kg per m/s impulse,
  negligible transverse force and zero angular acceleration. Two pilot seeds
  scored41.34 and45.21; not a demonstrated multi-seed improvement. Native
  test isolates this flag on the best +X/cache configuration. No simulator edits.
- EFP_EFFICIENT_X default0 separately calibrates the main +X request to
  [1,3.507949e-6,-1.008332e-5]. Official revised-simplex model .192741 m/s2
  with <.00001 transverse acceleration, zero torque,4.32237 kg/(m/s).
  Initial finite-burn prediction is updated consistently. Three allocator
  integration tests pass, runtime reset tests2 pass. Offline pilot is running.
- Offline actuator upper bounds were diagnostic only, never a replacement
  jet selector. Random legal angular-request and direction probes found no
  large efficient +Z gain beyond .135. Broad diagonal directions lose fuel
  efficiency; optimistic score screen found no useful alternative. Larger
  boost/slew requests and higher angular limit alone also failed the modeled
  score comparison. Do not promote these hypotheses without native evidence.


- Small-Z native alignment trial `261119_143200_FINAL_SUBMISSION__d56` scored
  29.5,267.78 s,48.95 kg,0 J, byte SHA d660c32c...62a1b6. Rejected for
  selection. Main-burn fuel included significant avoidance: body Y velocity
  changed from+0.23 to−0.77 to0 m/s during21–61 s, while all fuel was labeled
  IMPULSE_BURN. Do not attribute the whole43.70 kg main bucket to forward thrust.
- EFP_SENSOR_SETTLE default0 offers one bounded startup coast after alignment:
  minimum4 s, until40% of radar tracks confident, maximum12 s, only with
  positive outward velocity and a clear5 s coast. It preserves velocity and
  never forces waiting in immediate danger.3 tests and2 runtime reset tests
  pass. Pilot z_settle49.88/247.73 s/36.42 kg versus Z baseline51.11; no gain.
- Wait-for-gap V2 lowers the initial positive closing gate from1.5 to0.5 m/s;
  all0/3/6/10 s prefixes remain scored, retained ignition time is absolute.
  It differs from V1 and old coast-windows;2 focused tests pass. Z pilot
  z_early_wait48.80/242.07 s/38.08 kg/no contact, worse than baseline51.11.
  Rejected for promotion. No native wait/settle trial has been run.
- EFP_PHYSICAL_RISK default0 removes duplicated uncertainty from finite-shot
  radar core geometry. New core=.5*measured diameter+unchanged3 m vehicle;
  hard clearance still>0.5 m. Position variance remains4 m², and diameter
  radius variance(.025*diameter)² is added. FINAL_SUBMISSION verifies sigma2 m,
  velocity sigma.02 m/s, diameter sigma5%. Camera occupied bounds are unchanged.
  The old core was.575*diameter+5 m plus another covariance term.2 targeted
  tests pass, including covariance accounting and unchanged camera blockage.
  Diameter metadata is cached without changing previous obstacle outputs;
  cache equality test passes. Pilot z_physical_risk51.28/221.78 s/39.62 kg/no
  contact versus51.11 baseline is within seed noise; do NOT claim an offline
  score improvement. Native experiment is exploratory, motivated by the
  measured sideways fuel in the previous native run.

- Native +X/cache trial `261119_140256_FINAL_SUBMISSION__7b6`:47.2 points,
  227.77 s,41.60 kg,0 scored J. No local avoidance fuel in this run. Frozen
  byte SHA dcf29c02...f8300, official result in score55_native_trials.json.
- Native efficient +Z/cache without alignment, `261119_141524...c0e`:16.2,
  261.24 s,59.82 kg,0 J, SHA63f550ca...adf2ed. Rejected. Early ESCAPE cost
  6.71 kg and subsequent main burns45.22 kg. Do not claim that removing the
  +X slew alone is an improvement.
- MainAxisSlew already accepts a body_axis; new EFP_MAIN_AXIS_Z defaults0 and
  selects axis2 only with strong-axis enabled. The isolated official-allocator
  model of this small turn measured5.73 s,0.720 kg,0.112 useful delta-v,
  0.237 kg net overhead and0.32 degree residual angle. A tighter .25 degree
  tuning failed to settle in40 s and burned17.9 kg; rejected, NOT deployed.
  Source `score55_z_alignment_probe.json`; new small-turn integration test passes.
- Offline Z alignment with calibrated main/cache/radar/full-energy risk:
  `score55_z_axis_100.json`51.11/251.43 s/34.96 kg/no pre-entry contact.
  Adding accept-safe-pass was identical on that seed. Adding kinetic impact
  alone: `score55_z_axis_kinetic_100.json`51.67/220.15 s/39.63 kg/no contact.
  One seed is insufficient for promotion; selected native trial uses full energy.
- +Z without alignment plus kinetic impact:
  `score55_efficient_kinetic_100.json`45.94/217.17 s/44.49 kg/no contact,
  versus prior full-energy efficient_risk41.48/216.50 s/47.96 kg. The impact
  assumption helped this pilot by4.46 points but did not help the +X pilot.
- Official revised-simplex allocator integration test confirms the allowed
  calibrated positive-Z request achieves>0.13 m/s2 versus<0.10 nominal,
  with<4.33 kg/(m/s) and zero net angular acceleration. Test1 passes. No
  individual jets are commanded and allocator/configuration remain untouched.

- Radar clock-calibration native trial `261119_120849...7c1` scored37.3,
  248.30 s,45.81 kg,0 J, source5dd44db4...ee396. Its startup estimate locked
  at0.258 s. The larger0.8–1.35 s five-second report-age observations included
  substantial publication latency; do not call them the clock offset or assert
  they caused the regressions. Synthetic0.8 s epoch stress scored48.15 without
  calibration and47.89 with it. No demonstrated score gain; default off.
- Exact tracker cache was ported alone from the archived final_cache source.
  A500-track microbenchmark measured median72.16→14.52 ms, max output error0.
  `tests/score55_tracker_cache_timing.json` and
  `tests/test_current_tracker_cache.py` preserve evidence. New reports still
  refit; unchanged reports reuse pose transforms and forecast quality. Runtime
  EFP_TRACK_CACHE default0. Native effect is being measured in the above trial.
- EFP_KINETIC_IMPACT default0 scales only finite-shot modeled impact energy
  from m*v² to0.5*m*v². Recovery time still uses full relative speed; physical
  separation bounds remain. This is a physical full-stop energy assumption,
  NOT a verified simulator damage law. A focused gate test passes. Source
  `score55_kinetic_risk_controller.py`, successful live-progress pilot
  `tests/score55_kinetic_live_100.json`:48.18/250.85 s/37.24 kg/no contact,
  versus48.79/217.88 s/42.22 kg. No score gain on that seed. Do not promote.
- EFP_ACCEPT_SAFE_PASS default0 retains the original16 tangent endpoints,
  additionally prices any sensed clear unpowered pass within48 m, and checks
  all available geometry up to the first radius48 crossing near the station.
  Unlike near-entry-goal, it does not change the initial endpoint grid. Pilot
  `score55_accept_live_100.json`:51.76/217.70 s/40.02 kg/no contact, identical
  to entry-coast's seed100 outcome. No separate native validation yet.
- In-progress pilot `score55_efficient_kinetic_100.json` uses calibrated +Z,
  radar cruise, normal-tail risk, fine sidesteps, kinetic energy; NO +X slew.
  Uses frozen kinetic-risk source. Earlier efficient-main variants regressed;
  this trial isolates the half-energy assumption against that prior bundle.
  Neither simulator physics nor jet selection has been changed.
- `tests/score55_ballistic_heading_probe.json` is a constant-gravity,
  no-debris calculation, not implemented guidance: modeled fixed-heading52.18
  versus force-aligned54.02 with1.78 degree heading change and fixed measured
  slew cost. It does not prove a native gain or an achievable score.
- Use `tests/run_score55_trial.py` for new single offline runs: it prints
  progress, writes a trace and reports entry metrics. Several older PowerShell
  redirection sessions left empty traces; do not claim results for them.
  Example flags: `--name NAME --source tests/score_sweeps/FROZEN.py` followed
  by repeated `--experiment FLAG`. Seed defaults100; optional native-order RNG.
  Successful newer artifacts have kinetic_live/accept_live names.
-79 core/focused tests passed before the latest calibration/cache/energy
  additions; calibration4, cache1, updated risk3 passed separately. Six old
  adapter cases for retired features still fail as documented below. Run the
  appropriate current checks before selecting a final implementation.


## Active task: continue development until a native score of at least55

Latest explicit user authorization: implement and continuously test improvements
to the time/fuel tradeoff versus remaining distance. The user clarified that
completion requires an official native simulator score >=55; an offline median
does not satisfy the task. Minor nonfatal damage is acceptable if final score
improves, including the time and fuel required to recover lost motion. Fatal
damage and missed entry remain unacceptable. This authorization supersedes
the earlier read-only review and narrowly scoped camera-brake task below.

Keep Momentum as the base and preserve connection boilerplate, SetCommand,
camera helpers and official STU4-Sim/scoring/physics. New experiments are
disabled by default; do not promote a modeled result as native success.

Frozen current baseline is `tests/score_sweeps/momentum_55_baseline.py`,
normalized SHA `a7cb33d7ba02c9d400a0a55bef849e34d2a7c2fed6dc6484916a9a5928c8b2a4`.
A fresh20-seed baseline (100–119) is complete in
`tests/score_55_baseline_SEED.json`, summarized by
`tests/score55_baseline_summary.json`. Median38.275, p10–p90 25.648–43.850,
range13.79–45.86;20 arrivals/no modeled contacts. Use this baseline, not
unmatched old sources, for new comparisons. The offline model is still a proxy.

Native testing is available through the installed Space Teams PRO app using
the computer-use skill. The actual controller path is
`C:/Users/Asus/AppData/Local/SimDynamX/STPro/Assets/Repos/STU4-Solution/Controller.py`,
a separate directory, not a junction to this checkout. It matched the frozen
baseline before testing and has not yet been changed by the agent. Native
candidate copies outside the workspace require filesystem escalation. Do not
change STU4-Sim to select a different controller or alter score parameters.

Native baseline official results observed during this task:22.1 points,
270.36 s,54.24 kg,0 J in `261119_101425_FINAL_SUBMISSION__faf` (agent launched);
36.0 points,266.42 s,44.30 kg,0 J in `261119_102346_FINAL_SUBMISSION__b60`.
`tests/score55_native_trials.json` records these official alerts with source
identity evidence. Their source logs do not contain automatic hashes/seeds;
do not call them a matched-seed cohort. The first flight continued burning
after success and later disconnected; that later damage/fuel is not scored.
Stop a native session after recording its official result. The simulator UI
was returned to its main menu for candidate preparation. Refresh window
selection if its HWND changes after a session/restart.

Experiments screened so far (all offline seed100, not native improvements):

- Calibrated forward command:37.64 /225.08 s /49.26 kg, no contact, versus
  baseline44.93 /255.93 s /38.97 kg. Faster but worse score; rejected. Unlike
  the previous clean-main experiment, this preloaded achieved acceleration
  before the first burn and retained the camera-brake fix. Source and result:
  `tests/score_sweeps/score55_efficient_main_controller.py`,
  `tests/score55_efficient_main_100.json`. Removed from active code.
- One-time +X boost/slew:29.04 /226.43 s /55.47 kg, no contact. Local AVOID
  consumed14.25 kg. Source `score55_axis_align_controller.py`; result
  `tests/score55_axis_align_100.json`. Rejected for promotion.
- Same slew plus all burn-order shortlist alternatives:26.67 /224.33 s /
  57.61 kg, no contact. Source `score55_axis_orders_controller.py`; result
  `tests/score55_axis_orders_100.json`. Rejected for promotion.
- Tighter residual spin settling:21.99 /254.22 s /56.41 kg, no contact.
  Source `score55_axis_settled_controller.py`; result
  `tests/score55_axis_settled_100.json`. It solved large residual orientation
  drift but did not improve score; do not claim otherwise.
- Matched-camera prediction-rate experiment is being screened separately with
  the settled-axis variant. Depth bounds already given a radar velocity retain
  their entire occupied geometry, but inherit that radar track's forecast
  uncertainty rather than growing as unknown motion. Unassociated depth,
  terrain and station handling remain. Source
  `tests/score_sweeps/score55_axis_camera_controller.py`; result when finished
  `tests/score55_axis_camera_100.json`.

Current experimental code includes MainAxisSlew/AxisAlignedNavigator/
MainAxisAttitude and optional shortlist/camera-rate logic. Runtime switches
EFP_STRONG_AXIS, EFP_ORDER_DIVERSITY and EFP_CAMERA_TRACK_RATE all default0;
default flight is still the camera-brake baseline. Native copies must record
which switches are enabled and exact source. The slew is one fixed heading,
not continuous nose/horizon tracking; physical slew tests include allocator
cost. `tests/score55_axis_slew_probe.json` and `tests/score55_command_probe.json`
are actuator diagnostics, not flight scores or official results.

69 focused tests passed after the initial slew addition; the stricter spin
settling subsequently passed its5 focused tests. New camera-rate logic still
needs its targeted tests. Historical full-suite claims are not current:
the user overwrote the old archive file with the prior completion controller,
so unrelated archived-feature fixtures can be incompatible. Do not silently
restore rejected controller bundles just to satisfy those fixtures.

Further measured results (2026-11-19, development still in progress):

- First native candidate, `261119_111837_FINAL_SUBMISSION__84e`: official
  score34.6,235.46 s,49.75 kg,0 J. Exact installed byte SHA
  `706514fdd293dab2da29896ede93b30ff2e380824dcc4d0d37d88e2a59d6aef7`.
  Enabled strong-axis/radar-cruise/expected-risk/fine-avoidance. Frozen native
  trial results in `tests/score55_native_trials.json`. Not a55-point success.
  It coasted the terminal segment without further fuel; early local avoidance
  and recovery, not terminal braking, caused this run's excess fuel.
- Offline seed100, radar transit plus normal-tail/recovery-risk and fine
  sidesteps with +X slew:48.79,217.88 s,42.22 kg,no modeled contact. Source
  `score55_fine_avoidance_controller.py`;20 matched seeds100–119 are running.
  Native34.6 shows that this pilot proxy does not predict native score closely.
- +Y alignment instead:48.04,244.18 s,38.33 kg,no contact. Its slew consumed
  6.24 kg versus6.14 kg for +X; the hoped-for large slew saving was falsified.
  Source `score55_axis_y_controller.py`, switch EFP_MAIN_AXIS_Y default0.
- Entry-coast pilot:51.76,217.70 s,40.02 kg,no contact; +2.97 points against
  the matched48.79 pilot. EFP_ENTRY_COAST default0. Checks a constant-gravity
  first inward crossing of radius48 m and all sensed geometry up to that
  crossing. Preserves any certified pass, including miss distances below44 m.
  This differs from the old entry-scope experiment's tangent47 m trajectory.
  Source `score55_entry_coast_controller.py`;4 new boundary/hull tests pass.
- Local expected-risk pilot:48.79,217.88 s,42.22 kg,no contact; no pilot gain.
  EFP_LOCAL_EXPECTED_RISK default0, separately applies the finite-shot physical
  core/probability/expected damage/recovery cost to local fallback trajectories.
  A failed fatal/expected-damage coast gate cannot be overridden by the coast
  supervisor. Next native trial is staged with this one flight-policy change
  relative to the first native candidate, installed byte SHA
  `7351432f2d92b64dcb8fa534df8201ded30838afcc84766270c0e39c90e97d52`.
  Frozen copy `tests/score_sweeps/score55_native_local_risk_controller.py`.
- `tests/score55_slew_tradeoff.json` models faster isolated turns; it is not
  a flight result. Faster turns saved wall time but most fuel savings reflected
  less useful translation, so do not assume the full fuel difference is gain.

Latest continuation (native target remains unmet):

-20-seed first-candidate comparison finished: median45.15 vs38.275; p10–p90
  40.485–48.691, range29.61–48.94,20 arrivals/no modeled contacts. Median
  T220.33 vs264.97 s; fuel43.475 vs43.725 kg. Median-score delta6.875;
  paired bootstrap95% interval[5.23,10.365]. Summary
  `tests/score55_fine_axis_risk_summary.json`. This is measured offline,
  not a native55 success and not an estimate of a native median.
- Local expected-risk native trials scored4.1 (225.10 s,74.37 kg,0 J) and
  16.5 (243.20 s,62.12 kg,0 J), folders `261119_114238...641` and
  `261119_114648...f8a`. Rejected. Exact sources/settings/official fields
  are in `tests/score55_native_trials.json`; startup source hashes are in
  the sibling `FINAL_SUBMISSION_PySys_Controller.log`, not the main log.
- Near boundary-entry goal plus entry-coast native diagnostic scored−16.6,
  262.30 s,84.30 kg,0 J, `261119_115246_FINAL_SUBMISSION__53b`, byte SHA
  `3ecf2666a0d08b873c63c1ad652a319dc949ba1d6bab7006f94c588f59a6c7bf`.
  Early avoidance/recovery again dominated; no evidence for promotion.
  Offline seed100 was33.41/227.02 s/52.09 kg. EFP_NEAR_ENTRY_GOAL default0.
- EFP_RETAIN_LOCAL_SPEED keeps existing-speed route alternatives despite a
  lower advisory chunk speed, with all existing collision rollouts retained.
  Seed100 was identical to48.79/217.88 s/42.22 kg; no demonstrated gain.
- EFP_WAIT_FOR_GAP prices0/3/6/10 s unpowered prefixes (only beyond600 m,
  closing>1.5 m/s), preserving retained absolute ignition times. Different
  from the old0/1/2/4 s experiment: tested with strong-axis/radar transit/
  normal-tail risk/fine avoidance/entry coast. Pilot43.08/262.35 s/39.50 kg,
  versus entry-coast51.76; rejected. No native trial needed on that evidence.
- Native-order offline diagnostic fixes spawn RNG8193 and varies radar noise
  only: first candidate47.89/214.08 s/43.62 kg; boundary entry39.69/215.77 s/
  49.44 kg. Both have no modeled contacts. This mode still does not reproduce
  native score, timing, camera geometry or engine meshes.
- Investigating radar clock alignment before further trajectory tuning:
  official Radar.py defines t0 only AFTER its500 initial API/report reads.
  Controller uses a separate radar_epoch. No measured offset yet, so do not
  assert it caused the regressions. Diagnostic-only native source installed
  SHA `4e9264be26a0cf1c87090603fc047785c2cc5c6e0b8e8f579b9379402bf7c859`:
  first-candidate flags only, adds radarLag/track counts to five-second logs.
  Frozen `score55_native_radar_timing_controller.py`. Next run measures this.
  Do not silently adjust timestamps until measured.
- User explicitly authorized sole agent operation of the simulator during
  trials and will leave it untouched. Computer-use window is currently977x640;
  refresh observed coordinates. Official config/scoring/allocator/jet CSV
  hashes were rechecked unchanged. No commits or pushes.

All current runtime experiments default off in the workspace Controller.py.
The installed native candidate differs by explicit flag defaults only. The
installed baseline has now been replaced for authorized trials; its frozen
workspace backup remains `momentum_55_baseline.py`. No official sim file changed.
New risk/sidestep tests2 pass. A broad focused run had82 passes and6 failures
in pre-existing adapter tests for retired Dogleg/trace/camera-recovery flags;
those failures predate this task's experimental policies and must not trigger
restoration of rejected controller bundles. Do not claim the full suite passes.

The55 native target is NOT achieved. Continue experimentation and native
verification; do not end this task by claiming proxy success. No commit or
push has been performed.

## Camera-brake baseline — superseded by the active development task above

**Latest user direction, 2026-11-19:** the user again restored original
SCORE_MOMENTUM and explicitly requests only removal of fuel-burning braking
near200 m remaining. This supersedes the previous completion-fix selection.
Do not restore the prior controller or add boost, planner, camera or test-suite
refactors while performing this narrow fix.

At this task's start, HEAD was `31e139c`, the working tree was clean, and
Controller.py exactly matched `temp_controller/score_momentum_controller.py`
and original `4a8507e` (normalized SHA
`2b0f0f0b5674085d22d0ec5a454ed4ccde49b13e16e8645118210379578089af`).
The user has overwritten `temp_controller/currentcontroller.py` with the last
completion-fix version, normalized SHA
`5fd084a2121ceb7772e084caa391df7261d39e4dddb1a20bc6d5ab086c0dbaf3`.
It is no longer the old Dogleg archive described in the historical notes.
Neither saved controller is modified by this task.

**Measured native evidence:** read recent FINAL_SUBMISSION logs under
`C:/Users/Asus/AppData/Local/SimDynamX/STPro/Logs`. In run
`261119_080233_FINAL_SUBMISSION__c70`, at215.8 m remaining the rover was coasting
at4.68 m/s with50.38 kg fuel used. At193.3 m it entered WAIT_SHIP_CAMERA;
by111.7 m it had slowed to0.03 m/s and used68.50 kg. The camera-wait gate,
not a docking requirement, caused that observed pre-entry stop. Run
`261119_075432_FINAL_SUBMISSION__1da` shows the same gate near193.7 m and later
a stop outside the sphere near54 m. These are native observations, not models.

Recent official successes were17.7 /281.61 s /56.19 kg /0 J in
`261119_091430_FINAL_SUBMISSION__6c8`,22.8 /264.58 s /54.43 kg /0 J in
`261119_090934_FINAL_SUBMISSION__35d`, and35.1 /268.49 s /44.70 kg /0 J in
`261119_083947_FINAL_SUBMISSION__467`. The newest folder
`261119_093216_FINAL_SUBMISSION__2f5` has no official success/flight telemetry
in its inspected FINAL_SUBMISSION.log: the lobby timed out waiting60 s for
the ApplyGravity remote to report loaded. Its ApplyGravity log also reports
that server timeout; it does not contain a Python controller traceback. This
folder is a startup failure, not a scored flight. Source hashes and seeds are absent;
do not claim a controlled version comparison from these profile labels.
The17.7 run already used54.12 kg at207 m, then coasted with constant fuel until
near67 m; most of that run's fuel cost predates the terminal region. This patch
does not promise to recover all of the observed score loss.

**Selected patch:** only `CaptureApproach.goal` changes. Remove its automatic
`distance <210 and not camera_ready -> WAIT_SHIP_CAMERA, cap0` branch. Inside
210 m, missing station camera coverage also no longer applies the abort/braking
speed cap. Retain the original distant cap,47 m tangent interception, detected
obstacle checks, blocked-route handling, course trims, burn planner and all
other Momentum behavior. No new flags or parameters. The user's restored
post-entry behavior is deliberately left as restored; the earlier completion
patch is not reapplied. Connection boilerplate, SetCommand, camera helpers and
STU4-Sim remain unchanged.

**Validation:**64 focused tests pass: new `test_momentum_camera_brake`, plus
`test_controller`, `test_final_navigation`, `test_camera_fusion` and
`test_rolling_chunks`. Four new regressions verify the old camera stop versus
the new capture pass, identical distant/camera-observed guidance, retained
known-obstacle handling, and the actual Shooting-to-Economy handoff choosing
zero-thrust COAST toward a predicted pass inside50 m without station coverage.
This is functional/model validation, not a new measured native score or a
20-seed scoring experiment. Older202-test claims below apply to the previous
source and archive contents; the full historical suite is not claimed passing
after the user's rollback. Existing unrelated tests are not changed here.

AST comparison against the saved original finds only CaptureApproach changed;
the connection prefix is identical. Selected normalized controller SHA:
`a7cb33d7ba02c9d400a0a55bef849e34d2a7c2fed6dc6484916a9a5928c8b2a4`;
deployed-byte SHA:
`9dd3102704aff30571ee5b51d519b1ac93ab9cea8b6024b04718e09d2e9c91c3`.
Remaining verification: native run should no longer enter WAIT_SHIP_CAMERA
solely on crossing210 m, should retain enough guidance to enter the50 m sphere,
and should be evaluated by its official entry time/fuel/damage/score. No native
run, deployment, commit or push is performed during this fix.

## Previous completion-fix task — superseded by the user's rollback

**Latest user direction, 2026-11-19:** the user manually restored SCORE_MOMENTUM
because subsequent versions performed worse in their testing. Future controller
improvements must start from this Momentum version and preserve its successful
behavior. The subsequent authorized code task asks to remove station-approach
braking and consider a boost near 500 m only when the score tradeoff warrants it.
The earlier documentation-only restriction applied to the completed context
update, not to this code task. Preserve the restored Momentum flight planner.

In that prior task, this section superseded older "current", "selected", default-setting,
pending-task and stop/continue instructions below. The new section above is now authoritative.
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
