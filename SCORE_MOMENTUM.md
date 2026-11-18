# Momentum-preserving controller: SCORE_MOMENTUM

This candidate builds on `ShootingNavigator` and `FreeAttitude` in `Controller.py`.
It preserves useful velocity, learns unpowered drift and reduces unnecessary
steering. It does **not** establish a +70 competition result. The official
scoring alert remains authoritative; controller forecasts are estimates.

## The latest native evidence

The local final-scenario logs contain these successful runs:

| Native run directory | Time | Fuel at success | Damage at success | Official score |
| --- | ---: | ---: | ---: | ---: |
| `261116_074433_FINAL_SUBMISSION__45b` | 329.44 s | 59.54 kg | 0 J | +7.0 |
| `261116_073848_FINAL_SUBMISSION__ba6` | 300.68 s | 66.19 kg | 0 J | +1.7 |

These are the preceding `SCORE_SHOOT` runs, not results of this new candidate.
The logs do not contain source hashes. Extracted evidence is preserved in
`tests/momentum_native_baseline.json`, including a third run and phase samples.

The +7.0 run spent about **4.40 kg in launch** and **52.39 kg in cruise impulse
burns**, plus transitions and smaller corrections. Between approximately 31
and 61 seconds, speed remained near 1.4--1.6 m/s while fuel rose from 12.20 to
27.87 kg. This early steering is a larger problem than launch alone.
During the long coast, fuel stayed constant for extended intervals.

That run also registered official success between planner updates, while the
controller never marked arrival and subsequently recorded a collision. The
collision occurred after scoring, so its 21459 J was outside the reported
zero-damage success score. The new arrival handling addresses this timing issue.

## How momentum is used

Inertia preserves velocity when thrust stops; it does not preserve acceleration.
Gravity and other relative-frame effects continue to bend and slow a coast.
The next trajectory chunk uses:

```text
position(t) = position_now + velocity_now*t + 0.5*drift_acceleration*t^2
```

Finite burns add their actual acceleration intervals to that prediction. The
20-second sensor checks and 2 Hz replanning remain active throughout coasting.

The main changes are:

1. **Learn actual coasting dynamics.** `CoastDynamics` fits velocity over up to
   three seconds of unpowered flight. It subtracts nominal Phobos gravity and
   learns a bounded local residual acceleration. It ignores powered intervals,
   launch settling, large outliers and recent damage. The installed final
   configuration enables both Mars and Phobos gravity; the earlier central
   Phobos approximation alone need not explain all measured relative motion.
   The observer learns a local correction without guessing the missing force.
2. **Offer a genuine zero-thrust pass.** An already successful predicted pass
   through the safe sphere can compete directly with new burn plans. A coast
   need not hit the old 47 m waypoint exactly. It still needs acceptable sensed
   clearance and risk; the terminal camera guards remain active.
3. **Keep forward momentum.** Ordinary new finite-burn schedules cannot command
   more than 0.04 m/s net impulse opposite the destination direction merely to
   match a slower optimum. Emergency local control can still brake when the
   forward alternatives are blocked. Safe existing burns receive a stronger
   retention allowance, reducing repeated changes of steering direction.
4. **Price debris uncertainty.** Radar obstacles retain a hard core based on
   estimated debris size and the rover's 3 m physical envelope. Discretionary
   core padding is reduced by 1 m and extra required clearance by 0.5 m. Weak
   tracks use larger forecast uncertainty than mature tracks. Their uncertainty
   is priced through expected damage instead of requiring every possible
   future position to remain empty. Unknown camera surfaces and station guards
   keep their conservative geometry. The potentially fatal encounter gate
   remains in place.
5. **Transfer launch earlier.** The guarded lift transfers to flight planning
   after about 2.5 m of ascent rather than 3.5 m. It continues moving through the
   transition; it does not brake or hover at that height. Sensor checks remain
   active. The earlier height remains the default for the legacy launch class.
6. **Detect entry in the flight loop.** Near the ship, runtime uses the same
   `getLocation().WRT_ExprIn(ship_frame)` distance query as `Scoring.py`.
   Arrival is checked between planning updates, with the damage limit checked
   first. A recorded entry switches to post-arrival braking immediately and
   freezes the sampled phase-fuel ledger. The GUI separately displays entry
   fuel so later braking is not mistaken for scoring fuel.

`encounter_risk` uses an approximate Gaussian tail and a relative-energy proxy.
It is not the native damage law or a certified probability of collision.
Association mistakes, hidden meshes and correlated errors remain limitations.
The controller trades discretionary margin, not a deliberate collision with
the station. Rotation remains disabled apart from damping unwanted spin.

## Time, fuel and damage still compete

For this installed final configuration:

```text
Score = 100 * (0.5 + 120/T - F/75 - 2*D/209200)
```

One kilogram costs 1.333 points; 1000 J costs about 0.956 points. A 20000 J
impact therefore costs about 19.1 points. Damage can be cheaper than a long
detour, but a substantial impact can erase the benefit of saved fuel.

For a +70 target with zero damage:

| Fuel at success | Maximum arrival time |
| --- | ---: |
| 35 kg | 180.00 s |
| 40 kg | 163.64 s |
| 45 kg | 150.00 s |

At 260 seconds, +70 instead requires at most about **19.62 kg**, or +50 at most
**34.62 kg**. Reducing fuel from 60 to 40 kg alone cannot establish +70 at the
current flight times. The finite-burn planner optimizes the actual time/fuel
formula, rather than choosing a fixed speed that spends more fuel than it saves
in time. Available thrust remains unchanged.

## Validation

Four matched noise seeds used all 500 final-scene debris objects, a synthetic
ground plane 2.5 m below the initial centre, and the default 60 Hz servo:

| Seed | Previous fuel | New fuel | New arrival time | Previous score estimate | New score estimate |
| --- | ---: | ---: | ---: | ---: | ---: |
| 100 | 42.38 kg | 38.97 kg | 255.93 s | +40.78 | +44.93 |
| 101 | 40.24 kg | 39.20 kg | 264.87 s | +43.65 | +43.03 |
| 102 | 43.76 kg | 41.65 kg | 263.35 s | +36.16 | +40.03 |
| 103 | 46.32 kg | 46.59 kg | 250.83 s | +35.99 | +35.72 |

All four arrived without modeled debris or proxy-hull contact, then stopped.
Mean fuel fell from **43.18 to 41.60 kg** and mean estimated score rose from
**39.15 to 40.93**. Mean arrival time increased by **1.65 s**; two seeds scored
slightly worse, including one that used more fuel. This is a modest average
improvement, not a demonstrated +70 result or a guaranteed improvement in the
native simulator. The +50 target also remains unmet in these runs.

Seed 100 used 3.31 kg during the 8 s guarded launch, 1.35 kg in immediate escape,
34.08 kg in cruise burns and 0.22 kg during extra capture observation. Its
163.87 s with no translation command consumed no modeled fuel. Launch plus
immediate guidance/escape used about **4.67 kg**, compared with **4.88 kg**
before; part of the launch-bucket reduction is an earlier phase transition.
Post-entry braking used another 23.12 kg, excluded from the score.

The four result files match the delivered controller's source hash. Previous
matched results remain in `tests/score_shoot_ground100.json` through `103.json`.
Planning p95 in concurrent final benchmarks was approximately 263--288 ms;
native scheduling and actual meshes still require live validation.

The **117 passing tests** exercise learned drift, rejection of powered/contact intervals,
unpowered successful passes, finite-burn execution, guarded launch transfer,
fast arrival detection, the official scoring-frame query and damage priority.
The connection adapter executes the real runtime against an API double.

```powershell
python -m unittest discover -s tests -v
```

The offline benchmark renders all 500 public debris objects and optional ground
imagery, but passes only simulated radar/camera observations to the controller.
Benchmark version 4 also supports an unmodeled shared acceleration through
`--drift-bias X Y Z`, to stress the learned coast model. This is a synthetic
stress condition, not a calibration or reproduction of the native engine.

It still uses simplified gravity, sphere debris, an ellipsoid station proxy,
ideal servo scheduling and a different LP solver. It does not validate the
Unreal collision meshes or measure actual damage. Contact or a missed arrival
suppresses its zero-damage score estimate. Current measured artifacts are
`tests/momentum_final100.json` through `momentum_final103.json`; development and
stress results are preserved in `tests/momentum_tuning_history.json`.

## Deployment

Copy `Controller.py` and `camera_viewer.py` together into the installed local
`STU4-Solution` folder. Keep `EFP_NOSE_FIRST` and `EFP_AXIS_THRUST` unset or `0`.
The latter is an experimental override for all modes, not the normal finite-burn
executor. Check `profile=SCORE_MOMENTUM`, `attitude=FREE_ATTITUDE`, `coastDrift`,
`planMs`, and the phase-fuel ledger in the native log.

Use the official success alert's fuel, damage and time for comparison. The
rover can brake and park outside the sphere after entry because scoring has
already frozen. `BasicController.py`, `UserAPI.py`, the protected connection
prefix and official STU4-Sim remain unchanged. No native result is claimed for
this candidate until it is deployed and run.
