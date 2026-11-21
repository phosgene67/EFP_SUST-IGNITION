> **Historical document — archived 21 November 2026.**
> This describes an earlier revision or experiment. For the delivered implementation, see
> [Final Submission Algorithm](../Final%20Submission%20Algorithm.md). Archived commands and flags refer to their historical revision.
> Original location: `MOMENTUM_TERMINAL.md`.

# SCORE_MOMENTUM completion and terminal validation

2026-11-19. These changes start from the user's restored Momentum controller,
whose normalized source matches `4a8507e`. The saved original in
`temp_controller/score_momentum_controller.py` is unchanged.

## Selected behavior

After an actual scoring-sphere entry, latch `MISSION_COMPLETE_COAST` and send
zero linear and angular acceleration. This applies between planner updates,
during an unfinished axis burn, with no radar, and while spin damping would
otherwise be active. Leaving the sphere cannot reopen the burn. The next
simulation reset constructs a fresh planner normally.

The original pre-entry Momentum guidance and score optimizer remain the default.
There is no mandatory late boost, fixed 8 m/s minimum, new fuel cap, parking
command or new attitude slew. Connection boilerplate, SetCommand, camera
helpers and the official simulator/scoring/jet-selection files are unchanged.

The simulator's own Scoring log determines success. The installed Scoring.py
tests damage first and then accepts `distance <= 50 m`. A collision before
entry can still reduce score; a collision after official success is unscored.
Stopping jets after entry saves subsequent fuel but cannot increase a score
that has already frozen.

## Late-burn tradeoff

The restored Shooting planner already scores finite body-axis burns near 500 m
remaining. It compares predicted arrival time, incremental fuel and sensor risk,
with a 10 m/s cruise ceiling. The original handoff below 280 m is retained.
No region is declared debris-free solely from remaining distance.

For equal damage, a proposed burn must have positive modeled benefit:

`delta_score = 12000 * (1/T_burn - 1/T_coast) - (100/75) * extra_fuel_kg`

Burn time, gravity and current elapsed time must be included. The fuel estimate
uses 3388.7/(80*9.8) = 4.3223 kg per m/s of body-axis delta-v; sequential axis
impulses add by absolute component. This is a model, not a universal measured
fuel cost for combined rotations and translations.

The active planner's new clear-scene regression fixtures give these **modeled**
choices, with the target 450 m forward and 47 m sideways in the Phobos frame,
identity body rotation, a current station camera observation and no obstacles:

| Elapsed | Current speed | Fuel already used | Chosen action |
| --- | --- | --- | --- |
|100 s|3 m/s|25 kg|13.26 s finite burn; planned peak5.32 m/s|
|150 s|8 m/s|35 kg|Zero-thrust coast|

These fixtures verify that a profitable late impulse is allowed and an expensive
one is rejected. They are not native flight results or promised cruise speeds.

Across the exact-source historical 20-seed baseline, median fuel is 41.684 kg at
500 m remaining and 43.725 kg at entry. The median **per-run** incremental fuel
after 500 m is 1.994 kg. The median remaining time there is 89.995 s.
Most modeled fuel expenditure occurs earlier; a forced late boost is not a
general remedy for that cost.

## Rejected pre-entry experiment

The candidate was tested behind a default-off flag. It is now retained only in
the frozen candidate source; EFP_ENTRY_SCOPE is not an active controller switch.
With current camera coverage of the station, the candidate checks swept obstacles
only until predicted 48 m entry, two metres inside the 50 m sphere. It retains
checks before entry, obstacle inflation and missing-camera guards. It removes
the requirement to clear an additional 100 m
beyond capture. Its terminal coast rollout uses gravity and zero thrust.

The previous entry-scope experiment gained only 0.035 median modeled points,
within paired seed noise. This trial additionally fixes the terminal rollout's
constant-speed-servo approximation; it must establish a new gain rather than
inherit a claim from that old experiment.

The new correction produced no robust improvement either. **Modeled results,
matched seeds100–119**, using the historical exact-source baseline and fresh
candidate runs:

| Controller | Median score | Score p10–p90 | Median entry time | Median entry fuel | Arrivals / pre-entry contacts |
| --- | ---: | ---: | ---: | ---: | ---: |
| Original Momentum baseline |38.275|22.984–43.789|264.97 s|43.725 kg|20 /0|
| Entry-scoped candidate |38.310|22.984–44.140|264.97 s|43.595 kg|20 /0|

Median-score delta +0.035; rounding-adjusted paired-bootstrap 95% interval
**[-0.295,+0.770]**. Paired median time and fuel deltas are both zero; mean time
delta -0.005 s and mean fuel delta -0.085 kg. Thirteen of20 reported entry
results are identical. Score ranges: baseline13.79–44.93, candidate13.79–45.24.
No modeled contacts imply modeled zero damage; native damage energy is not
simulated. All20 candidates used0 kg over the following90 s.

| Phase | Paired median fuel delta | Paired mean fuel delta |
| --- | ---: | ---: |
| Launch |0 kg|0 kg|
| All cruise phases |0 kg|0 kg|
| CAPTURE_PASS |0 kg|-0.073935 kg|
| OBSERVE_CAPTURE |0 kg|-0.011095 kg|
| WAIT_SHIP_CAMERA |0 kg|0 kg|

**Decision: reject promotion; the score gain is within seed noise.** The active
controller retains original pre-entry guidance and only the completion fix.
No gain from this trial is claimed. Complete data and paired statistics are in
`tests/score_sweep_momentum_terminal_scope.json`.

## Verification and reproduction

The full suite passes 202 tests. Tests for removed Dogleg, ground-lateral,
camera-recovery, tracker-cache and Shooting experiments explicitly use their
saved source in `temp_controller/currentcontroller.py`. Those passing tests do
not imply these features are active. Entry-scope tests load the frozen rejected
candidate. Normal adapter tests and the new Momentum
capture/late-burn fixtures execute active Controller.py.

The no-brake default was rerun on modeled seed100: entry 255.93 s, 38.97 kg,
score 44.93, no modeled contacts. Those entry metrics exactly match the original
seed100. Its following 90 s uses 0 kg, versus 23.1234 kg in the original.
This single default run verifies the completion fix, not a new 20-run score gain.

The 20-seed baseline is `tests/score_baseline_100.json` through119. Candidate
source is frozen at `tests/score_sweeps/momentum_terminal_scope_controller.py`;
candidate results are `tests/momentum_terminal_scope_SEED.json`. Each run uses:

```powershell
# Set these two paths to the installed, unmodified official assets.
$scenario = 'C:/Users/Asus/AppData/Local/SimDynamX/STPro/Assets/Repos/STU4-Sim/Sims/FINAL_SUBMISSION.simconf'
$jets = 'C:/Users/Asus/AppData/Local/SimDynamX/STPro/Assets/Repos/STU4-Sim/Data/SpaceBuggyJetConfigNoMains.csv'
$pythonExe = 'C:/Users/Asus/AppData/Local/SimDynamX/STPro/Versions/v0.56.0/py/Scripts/python.exe'
& $pythonExe -B tests/benchmark_final_submission.py --sim-config $scenario --jet-config $jets --ground-plane 2.5 --fuel-audit --baseline-file tests/score_sweeps/momentum_terminal_scope_controller.py --experiment entry-scope --seed 100 --output tests/momentum_terminal_scope_100.json
# Repeat with seeds101-119 and corresponding output filenames.
& $pythonExe -B -m unittest discover -s tests
```

All scores here are offline proxies. The harness uses synthetic camera images,
sphere debris, an ellipsoid station hull and simplified dynamics. Native damage
energy and actual Unreal collision geometry are not reproduced. Benchmark
wall-clock compute time is not charged to simulated mission time. Concurrent
test load also affects reported planner timing; it is not native latency evidence.

## Native checks remaining

1. Run active Controller.py with `EFP_NOSE_FIRST=0`, `EFP_AXIS_THRUST=0`;
   no terminal experiment is enabled.
2. Confirm the official success block, entry time/fuel/damage and sphere entry.
3. Confirm `MISSION_COMPLETE_COAST` after entry and no further commanded fuel
   expenditure, including when the rover subsequently leaves the safe sphere.
4. Preserve the full new `FINAL_SUBMISSION.log`, especially IGNITION modes,
   speed/fuel/damage from600 m remaining through entry and the official result.
   A pre-entry slowdown needs these modes to distinguish camera/hull avoidance,
   an intercept correction and unpowered gravitational deceleration.

No new native score,55+ result or60+ median is established by this task.
