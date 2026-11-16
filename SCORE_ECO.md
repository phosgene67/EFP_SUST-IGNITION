# Score-based controller candidate

`Controller.py` now runs `EconomyNavigator` and `FreeAttitude`, logged as
`profile=SCORE_ECO`, for `FINAL_SUBMISSION.simconf`. Copy it together with
`camera_viewer.py` to the installed `STU4-Solution` folder. The connection helpers,
`BasicController.py`, `UserAPI.py`, and official STU4-Sim are unchanged.

Forced nose pointing and horizon leveling are now off by default. See
[FUEL_AUDIT.md](FUEL_AUDIT.md) for the current paired score results and fuel
breakdown. The earlier nose-first measurements below are preserved as history.

## The actual score and useful tradeoffs

The installed official `Systems/Scoring.py` and final scenario specify nominal
time **120 s**, nominal fuel **75 kg**, damage limit **209,200 J**, and success
radius **50 m**. For an arrival:

```text
S = 100 * (0.5 + 120/T - F/75 - 2*D/209200)
```

`T` is seconds, `F` is kilograms, and `D` is joules at success. Scores are
unclamped. A negative number is an actual negative score. A failure uses a
different, distance-based formula; this controller prioritizes arrival over
optimizing an incomplete run.

Small damage is inexpensive, but it is not free:

| Change | Effect on arrival score |
| --- | ---: |
| Use another 1 kg | -1.333 points |
| Take 100 J | -0.096 points |
| Take 1,046 J | -1 point |
| Finish in 180 s instead of 200 s | +6.667 points |
| Finish in 180 s instead of 200 s, spending another 5 kg | No net improvement |

The local value of saving a second is `12000/T²` points. At 200 s, extra thrust
must save at least about 4.44 seconds per extra kilogram just to break even.
At 300 s, that rises to 10 seconds/kg. Consequently, holding the highest possible
speed throughout the flight can give a worse score than accelerating once and
coasting. Past damage and spent fuel cannot be recovered by flying slower.

For target score `S_goal`, the fuel budget at a planned arrival time is:

```text
F_max = 75 * (0.5 + 120/T - 2*D/209200 - S_goal/100)
```

| Arrival time | Maximum fuel for 50, no damage | Maximum fuel for 75, no damage |
| --- | ---: | ---: |
| 120 s | 75 kg | 56.25 kg |
| 150 s | 60 kg | 41.25 kg |
| 180 s | 50 kg | 31.25 kg |
| 200 s | 45 kg | 26.25 kg |
| 240 s | 37.5 kg | 18.75 kg |
| 300 s | 30 kg | 11.25 kg |

Examples of the arithmetic, **not achieved runs**: 150 s / 40 kg / 0 J scores
76.67; 180 s / 30 kg / 1,000 J scores 75.71; 200 s / 40 kg / 1,000 J scores 55.71.
These budgets do not establish that the installed thrusters and debris field can
produce those combinations. Collision energy depends on unknown debris mass and
relative velocity, so a planned impact is not treated as a known small penalty.

## How the executable algorithm works

1. **Launch.** Climb 3.5 m with sensor feedback and then follow the world-space
   route. There is no nose-alignment pause or horizon-leveling burn. Minimal
   spin damping remains. There is no enforced minimum speed during launch or hazards.
2. **Choose a burn/coast schedule.** `EconomyPolicy` evaluates peak speeds up to
   10 m/s using remaining distance, gravity, current speed, elapsed time, measured
   fuel and damage. It selects the highest projected arrival score. Telemetry
   updates the useful acceleration and fuel cost per change in velocity. This is
   an estimate, not an official score or a guaranteed result.
3. **Plan overlapping moving trajectories.** `RollingChunks` retains its finite
   acceleration beam search, radar motion prediction, wide-sector comparison,
   and early successor handoffs. Economy ranking prices time and impulse in
   score points, including the eventual turn back toward the destination.
   A route that appears cheap only because it postpones an expensive turn is
   penalized before committing it. Hard swept collision checks remain active.
4. **Coast and trim.** `EconomyServo` uses hysteretic pulses and genuinely zero
   translation commands between burns. The official jet allocator can saturate
   even small continuous requests. Gravity-aware coast prediction and lateral
   course corrections avoid maintaining speed with constant fuel use or braking
   merely because a revised economy optimum is lower. Long direct trims need a
   swept predicted arc check; other cruise trims follow the verified chunk.
5. **Keep observing.** Radar tracking, RGB associations, both depth cameras,
   pose/time validation, and the optional OpenCV interface stay active. Compact
   distant depth regions use smaller occupancy tiles, with vehicle, pose and
   range margins retained. Radar retains a 4 m position margin rather than the
   previous 8 m, plus vehicle, shape and observation-age inflation. These are
   engineering margins, not a guarantee against every collision.
6. **Enter the safe area on a clear side.** `CaptureApproach` plans a line whose
   closest range is about 47 m, verifies observed obstruction clearance, and
   enters the 50 m sphere without first demanding zero speed. Fresh ship-bearing
   depth coverage is required. If no pass can be verified, reobserve and use the
   slower standoff/survey approach when necessary. Local collision handling
   takes priority over economy coasting or trims.
7. **Brake after entry.** Arrival is latched independently of flight speed.
   Post-entry control decelerates along the exit path and parks. The final parked
   location may be outside 50 m because the clock and scoring already ended at
   first entry. This is not a restart. Radar/depth avoidance continues; the GUI
   arrival banner identifies this as a controller observation. Only the official
   Scoring log confirms a win. Pre-arrival damage above the official limit cannot
   be relabelled as success by the controller.

## Code and tests

The source now passes **97 automated tests**; the new fuel accounting and attitude
checks are described in [FUEL_AUDIT.md](FUEL_AUDIT.md). The following historical
offline results describe the preceding nose-first score controller. The baseline is Git
commit `a15a4c6`, rerun with the same version-2 harness, 60 Hz servo and four
camera requests/s:

| Offline case | Entered safe area | Time at entry | Fuel at entry | Zero-damage score estimate |
| --- | --- | ---: | ---: | ---: |
| Prior OPEN_LANE_9, seed 100 | Yes | 342.88 s | 188.97 kg | -166.96 |
| SCORE_ECO, seed 100 | Yes | 350.55 s | 73.93 kg | -14.35 |
| SCORE_ECO, seed 101 | Yes | 327.28 s | 49.00 kg | 21.34 |
| SCORE_ECO, clear-corridor diagnostic | Yes | 277.08 s | 37.35 kg | 43.50 |

There were no modeled debris or proxy-hull contacts in these cases. Both noisy
cases also decelerated to below 0.05 m/s after entry. Seed 100 used about 61% less
fuel than the prior controller with a slightly longer flight. **Neither noisy
case achieved 50 points; 75 is not validated or promised.** Even the clear-field
diagnostic fell below 50. Remaining losses include launch/orientation overhead,
debris avoidance and corrections near the ship. This is a tested development
candidate, not a claim that the requested score target has been solved.

Historical raw results: [seed 100](tests/score_eco_results.json),
[seed 101](tests/score_eco_seed101_results.json),
[clear-corridor diagnostic](tests/score_eco_clear_results.json), and
[matched baseline](tests/score_eco_baseline_results.json).
`distance_m` in these artifacts is the position at the end of the post-entry
parking test, not the range at which the clock stopped. The archived
[tuning history](tests/score_eco_tuning_history.json) includes failed and worse
earlier experiments, with their own source hashes; those are not results for the
delivered controller. All numbers above are offline estimates, not official
competition results.

Everything required to fly remains in `Controller.py`. The protected prefix
connects to the simulator; the definitions below it implement perception,
tracking, planning, translation and attitude control; the final loop reads the
APIs and sends commands. The runtime uses NumPy and the supplied simulator API.
The viewer is a separate optional process, not the navigation controller.

`tests/test_economy.py` checks the official formula and budgets, damage pricing,
fuel calibration, pulse behavior, ballistic prediction, safe-area fly-through,
speed-preserving course trims, fallback behavior and damage-limit ordering.
The other tests cover the existing tracker, chunk handoffs, cameras, viewer,
body-frame conversions and connection/cleanup contract. They do not run the
Unreal simulator or establish a competition score.

```powershell
python -B -m unittest discover -s tests -v
```

The updated `tests/benchmark_final_submission.py` defaults to the native **60 Hz**
servo, 2 Hz planning, four camera requests/s, noisy cycling radar, official jet
geometry/allocator equations, gravity and a proxy station hull. Result files
record controller/scenario SHA-256, seed and benchmark settings. It never feeds
scenario ground truth into the flight planner. `--clear-corridor` is explicitly
a diagnostic: it moves synthetic debris aside in memory and never changes the
official simulation. Older 10 Hz result files are historical and should not be
compared numerically as if the harness settings were identical.

The model uses synthetic cameras, simplified dynamics, HiGHS instead of the
native LP solver, and an ellipsoid instead of the real station mesh. In
particular, no modeled contact is not proof of zero native damage. Live runs
must confirm time, fuel, damage and official arrival before calling this a
50-point or 75-point controller.
