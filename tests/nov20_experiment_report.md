# November 20 score optimization

Status: **finalized at the user's deadline request**. Use repository-root
`Controller.py`, SHA256
`4210afa43e5a2570eb3fcd12c6e276c8697a69da631931fb1abeac3613332834`.
Its best completed identical-source native cohort averaged **41.27**. The45+
minimum average and55+ desired average were **not achieved**. No statistically
established improvement over the user's submitted40.5 average is claimed.
Unfinished experiments are excluded from the production controller.

## Consecutive native validation

Identical controller bytes throughout all three runs:
`4210afa43e5a2570eb3fcd12c6e276c8697a69da631931fb1abeac3613332834`.
SCORE_MOMENTUM + existing strong-X alignment, conservative collision bounds,
confident radar-position smoothing, compiled identical risk and existing
radar-clock estimator. Smaller physical geometry is OFF. No background
benchmarks ran during these trials. Native seeds were not exposed.

| Run suffix | Official score | Entry s | Entry fuel kg | Scored damage J |
| --- | ---: | ---: | ---: | ---: |
|223849...101|45.1|222.17|44.22|0|
|224605...efc|40.1|232.52|46.10|0|
|225615...bd9|38.6|240.13|46.04|0|

Mean score41.2667, range38.6–45.1. This small native cohort does not establish
a45-point minimum or55-point average. Later station contacts occurred after
official scoring stopped; those are not scored debris damage.
Exact records: `nov20_final_native_validation.json`.

Measured fuel buckets near entry: main finite burns26.43–27.20 kg,
initial alignment6.08–6.10, launch4.10–4.92 and local avoidance4.56–7.21.
Transitions contain another1.29–3.69 kg. These are sampled command-phase
buckets, not a unique causal split between rotation and translation.
The second run used no additional fuel from505 m remaining through entry;
its lower score cannot be fixed by removing nonexistent terminal braking.
Evidence: `nov20_native_phase_audit.json`.

## Selection and rejected follow-ups

The selected defaults retain SCORE_MOMENTUM, strong-X alignment, radar cruise,
expected-risk checks, fine avoidance, report cache, confident position smoothing,
compiled identical normal-tail evaluation and the existing clock estimator.
`EFP_PHYSICAL_RISK=0`; conservative collision bounds remain active. No hard
fuel cap or new path planner is enabled. Camera/connection/command helpers are
unchanged. All76 targeted tests passed on these exact controller bytes.
The source also matches the frozen conservative native controller byte for byte.

- Early fine sidesteps: all20 proxy score/time/fuel/contact fields exactly
  matched the smoothing baseline. Two native trials scored42.7 and36.8
  (mean39.75), fuel46.10 and52.22kg, both0 scoredJ. Not promoted.
- Surface-range fusion: three unit/reset tests passed, but pilot100 scored
  45.55 versus50.48, using42.40 versus36.60kg. Rejected; no native claim.
- Coast-priority hysteresis:20/20 proxy arrivals/no contacts. Median score46.675
  versus46.80 smoothing baseline. Paired mean score delta−.0395,
  bootstrap95%[−.4515,.3000]. Mean fuel−.8625kg, mean time+5.1165s.
  Fuel savings did not improve score; rejected. Native candidate prepared,
  never flown. Evidence `nov20_coast_priority_comparison.json`.
- Powered transverse drift: four unit/reset tests passed. Under an ASSUMED
  constant field bias fitted from native calm-coast data, seed100 scored44.38
  versus46.66 baseline, fuel44.02 versus45.64kg, time226.10 versus208.62s.
  This is a modeled stress pilot, not native physics verification. Not promoted.
- Wider49m capture band: isolated prototype only; unflown/unselected. The
  selected controller retains its existing capture band and margins.
- Precision impulse taper: unfinished isolated prototype at the deadline;
  untested/unselected. It is not part of root Controller.py.

A diagnostic-only conservative flight scored37.0/225.37s/49.67kg/0J.
Its different source hash/logging means it is recorded separately from the
three identical-source validation runs; it also illustrates native variability.
Own-state telemetry and a modeled unpowered counterfactual are retained in
`nov20_motion_diagnostic_samples.json` and `nov20_coast_counterfactual.json`.
These counterfactuals do not certify debris/hull clearance or establish a native
score. Some late corrections were necessary: predicted miss was66.57m at81s.

The user supplied an official54.0 excerpt:203.39s/41.24kg/0J, SimRunID
`a6a82e84-43ed-2c36-31a3-7eb9da19f958`. Its original local file and exact
source/seed were unavailable. It proves that individual native result, not
an average or a result of the selected source. Stored separately in
`nov20_user_supplied_54_result.json`. The user-started installed-baseline
run032829...e44 scored12.6/249.50s/64.13kg/0J; it is also recorded separately.

Official FINAL_SUBMISSION selection is restored and the simulator is idle at
its main menu. All development benchmark jobs finished. Final protected-file and connection-
prefix checks passed. The installed solution checkout remains unchanged;
only repository-root EFP/Controller.py is the delivery file. No GitHub push
or competition submission was performed. Push this EFP version, then Pull the
solution in the simulator before submitting; its installed checkout currently
still contains the earlier controller.

## Scope and scoring

The base remains SCORE_MOMENTUM. Connection/command/camera helpers, the official
simulator, jet allocator, jet CSV and official scenario are unchanged. Development
stays in the EFP repository. Native tests use isolated scenario copies that change
only the Controller source path to EFP; the installed solution Git checkout is
untouched. Protected hashes are in `nov20_protected_hashes.json`.

The installed official Scoring.py and FINAL_SUBMISSION.simconf confirm
`100*(0.5 + 120/T - fuel/75 - 2*damage/209200)` and first entry into the50 m
sphere. No braking after entry is required. The existing modeled body-axis fuel
coefficient is `3388.7/(80*9.8)`, approximately4.32 kg per m/s of impulse.
Fuel targets are soft objectives; no50 kg cutoff was introduced.

## Completed matched offline comparisons

All rows below use seeds100-119, the same scenario,60 Hz servo,2 Hz planning,
the official jet geometry, ground-plane proxy2.5 m and zero synthetic planning
delay. These are simplified-physics/synthetic-sensor **proxy** results, not
official native scores. Phase labels classify commands, not unique physical
causes of consumption.

| Profile | Median score | p10-p90 | Median entry s | Median fuel kg | Proxy contacts |
| --- | ---: | --- | ---: | ---: | ---: |
| Frozen baseline |45.15|40.485-48.691|220.33|43.475|0/20|
| Confident radar-position regression |46.80|43.077-48.617|220.36|43.835|0/20|
| Compiled normal-tail evaluator only |45.15|40.485-48.691|220.33|43.475|0/20|
| Existing accept-safe-pass option only |45.67|41.093-50.558|217.65|42.075|0/20|
| Regression + compiled risk + physical radar covariance |48.805|46.497-50.454|220.975|41.495|0/20|

Radar-position regression versus baseline: paired median score+1.86,
bootstrap95%[0.385,2.735]. Median paired fuel-.67 kg; mean fuel44.6015->43.799
kg. Main-burn phase median paired fuel-1.4462 kg, local avoidance-.2726 kg;
launch and alignment unchanged. Medians of phase deltas do not add to the
median total delta. Full records: `score55_nov20_radar_smooth_summary.json`.

The position estimate is projected to the report timestamp. Raw samples and
associations, velocity fitting, uncertainty and camera bounds remain intact.
It uses only confident tracks; immature/uncertain tracks retain raw positions.
Cache replay and moving-track tests guard against duplicate correction or lag.

The compiled evaluator matched every seed's score, entry time, fuel and contact
result exactly. Its90x80 probability microbenchmark measured2.7299->.2128 ms
median, with maximum probability difference1.11e-16. This measures compute,
not a native score improvement. SciPy is already used by the official allocator;
if its special-function module is unavailable, the original math model is used.

Accept-safe-pass: paired median+1.155, bootstrap95%[0,2.43]. This does not clear
the specified robustness gate, so it remains off.

## Exploratory native results

Source identity is checked in each separate PySys_Controller log. Native seeds
are unknown; these runs are not paired seed experiments. They had concurrent
offline benchmark activity. Final consecutive validation must run without that
background load. Every official result, including regressions, is retained in
`nov20_native_trials.json`.

| Run suffix | Profile | Score | Entry s | Fuel kg | Scored damage J |
| --- | --- | ---: | ---: | ---: | ---: |
|212710...95c|Unchanged baseline|32.1|232.55|52.12|0|
|213649...5eb|Accept-safe-pass|-5.3|251.07|77.29|0|
|215046...499|Radar-position regression|40.8|234.86|45.19|0|
|220629...35a|Regression + compiled risk|39.2|237.50|46.01|0|

Attempt214620...c95 timed out in the lobby waiting for the official jet-selection
system to load. It produced no flight or score; it is not counted as a finish.

The read-only historical audit also found18 exact-baseline-source completed
records: mean24.53, median28.3, range-9.2..49.6. This is a historical log sample,
not the user's submitted40.5 average or a matched cohort.

## Rejected pilots and geometry comparison

- Useful early-lateral candidate screening: seed10041.65 versus48.79 baseline,
  fuel51.69 versus42.22 kg. Rejected.
- Body-axis L1 fallback fuel pricing: seed101 identical; seed10540.63 versus
  40.70. The mathematical cost correction did not improve these flights.
- Rechecked calibrated+Z axis: seed10440.74 versus44.96 baseline; adding the
  improved tracker on seed10050.51 versus+X smoothing50.48. No promotion.
- Physical radar geometry/covariance: completed20/20 with no proxy contacts;
  subsequently REJECTED natively. Increment over smoothing alone: paired median
  +2.17, bootstrap95%[1.10,3.44]. Versus original baseline: paired median+4.51,
  bootstrap95%[1.025,5.79], median paired fuel-2.99 kg and time+1.185 s.
  All20 used34.95-47.11 kg; score range44.53-51.03.
  Mean score48.374 versus44.1175 baseline. It uses `.5*diameter + existing3 m vehicle allowance` as the
  modeled core and puts5% diameter measurement noise into covariance alongside
  the unchanged2 m position sigma. Camera volumes, full `m*v^2` impact energy,
  recovery cost, fatal-probability gate and .5 m clearance guard remain.
  Earlier geometry-only trials were inconsistent; this isolated comparison
  differs by using the improved position tracker. Do not select it from the
  worst-seed improvement alone.

The reduced radar geometry was REJECTED natively: quiet run222652...d62 took
46,322.1 J, then reached77.71 kg by171.4 s with an immediate-arrival score
upper bound of-27.89. It was aborted, with no official finish score. This
falsifies native safety inferred from the20 clean proxy runs.
`EFP_PHYSICAL_RISK=0` is restored. Radar-position regression and compiled risk
remain enabled. The current conservative candidate enables the existing radar
epoch estimator, retaining the larger geometry bounds. A zero-offset cache
reset bug was corrected: aligned tracks survive a no-op clock lock.
The new candidate passes76 targeted tests. A single-seed1.3 s synthetic clock
stress improved49.96->50.48, while zero skew exactly reproduced50.48; this
is a small proxy pilot, not evidence of a native gain. Radar code starts its
epoch after the initial500-object scan; publication age alone cannot measure
the exact epoch difference. Native validation remains decisive.
All can be disabled to reproduce the preceding policy. The three-run current
native validation is recorded above; its target is unmet.71 targeted tests passed for the now-rejected geometry source
`1aec9ebf31b037e7693461512798941b9ba941375f87717783b5cb644989800a`.

## Verification limits

Current-source unit checks cover timestamp-aligned tracking, cache equivalence,
normal-tail numerical equivalence and missing-SciPy fallback, camera blockage,
reset configuration and official-frame safe-zone capture. Two historical
suites have the same15 errors and1 failure on the original baseline and current
source; see `nov20_historical_test_comparison.json`. No claim that the entire
historical test archive passes is made. The user's pre-existing changes in
`temp_controller/currentcontroller.py` are preserved.

No GitHub push or competition submission has been performed.
