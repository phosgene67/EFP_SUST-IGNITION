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
20-seed comparison is running with the other two flags disabled.

## Active baseline parameters

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

For native baseline trials, leave all three experiment flags at zero, along
with `EFP_NOSE_FIRST=0` and `EFP_AXIS_THRUST=0`. Confirm the file hash and flags
in the controller log. Optional experimental trials must enable only one flag
and use the corresponding frozen source; neither completed experiment has
earned promotion to the default controller.
Keep the official success alert and entry time/fuel/damage for each run.
Twenty matched native runs are needed to assess the requested native median.
The available [native log inventory](tests/native_score_inventory.json)
contains mixed versions and missing seed/source fields.
