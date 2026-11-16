# Fuel audit and removal of forced nose control

The tables below preserve the preceding controller comparison. The current
finite-burn implementation and its separate validation are documented in
[SCORE_SHOOT.md](SCORE_SHOOT.md). Forced nose control remains off by default.

Forced nose pointing and planet-relative horizon leveling are now **off by
default**. The controller still transforms world acceleration into body-frame
thruster commands. It does not need to face its nose along the velocity to move
along the planned path. `FreeAttitude` only damps an actual unwanted spin; it
does not rotate the rover for its camera view or appearance.

This decision follows matched offline experiments, not a claim that all turning
is wasteful. Removing nose control improved three of four tested noise seeds;
one seed used more fuel and scored worse. Native simulation results still need
verification. The 50–75 score target has not been achieved in these experiments.

## Where the previous controller spent fuel

The audit reads the actual per-tick fuel cost produced by the modeled official
jet allocator. The following buckets are disjoint and stop at first entry into
the safe area. They describe the previous nose-first controller, source hash
`9335bb95444e2d4e3d2f0e9eaed6a3c7b45bc967dafab5a5e239c3fb070c2c34`
(Git `412708b`). All runs use the same final scenario, 60 Hz flight control,
2 Hz planning, four camera requests/s and matched noise seeds.

| Flight segment | Seed 100 fuel | Share of its total | Seed 101 fuel |
| --- | ---: | ---: | ---: |
| Debris avoidance | 26.10 kg | 35.3% | 8.67 kg |
| Main guidance/acceleration | 20.00 kg | 27.1% | 26.44 kg |
| Course corrections | 19.48 kg | 26.3% | 5.96 kg |
| Initial 3.5 m launch | 4.75 kg | 6.4% | 4.75 kg |
| Initial nose alignment pause | 1.53 kg | 2.1% | 1.53 kg |
| Cruise coasting, including attitude corrections | 1.31 kg | 1.8% | 0.39 kg |
| Safe-area pass | 0.76 kg | 1.0% | 0.51 kg |
| Additional station observation | 0 kg | 0% | 0.74 kg |
| Total at success | **73.93 kg** | **100%** | **49.00 kg** |

Guidance burns, avoidance and course corrections dominate. The cost of avoiding
debris varies substantially with observations and the resulting trajectory.
Radar, image capture, camera panning and the OpenCV viewer do not directly
consume thruster propellant. Their feedback can cause a necessary maneuver,
which does consume fuel. Disabling those sensors would remove the observations
needed to avoid collisions and verify the station approach.

## Does nose-first control consume extra fuel?

Yes, rotation uses the rover's jets. But direct rotation-only cost is not the
whole effect. For seed 100, a second disjoint accounting of the same 73.93 kg is:

| Commands active at the same instant | Fuel |
| --- | ---: |
| Translation only | 29.68 kg |
| Rotation only | 3.82 kg |
| Translation and rotation together | 40.44 kg |
| Neither | 0 kg |

The 40.44 kg combined bucket is **not** all rotation fuel. Jets can supply force
and torque simultaneously; the allocator scales the entire solution to its
maximum jet duty. A pure body +X request of 0.5 m/s² produces about 0.17456 m/s²
in this model. Adding one tested pitch request of 0.005 rad/s² reduces delivered
forward acceleration to about 0.09653 m/s². It also reduces instantaneous fuel
flow, so that example alone does not demonstrate a total fuel saving. A longer
burn, changed attitude, different observations and changed avoidance trajectory
all affect the final score.

Therefore the audit also performs complete A/B flights. It does not subtract
an imaginary independent rotation fuel tank from the total. Counterfactual
same-tick allocator costs in the JSON files are diagnostics, not additive actual
fuel attribution or alternative-flight scores.

## Complete-flight comparison

These results compare the original controller with an offline ablation that
removes angular commands, the initial nose alignment wait and the nose-angle
filter on useful-thrust calibration. The latter filter is inappropriate when
the body deliberately flies sideways. No path, sensor, speed or collision margin
settings were otherwise changed. Changes in observed images and avoidance paths
are part of the resulting system behavior.

| Noise seed | Nose-first time / fuel | Nose-first score estimate | No forced orientation time / fuel | Score estimate |
| --- | --- | ---: | --- | ---: |
| 100 | 350.55 s / 73.93 kg | -14.35 | 280.60 s / 49.87 kg | 26.27 |
| 101 | 327.28 s / 49.00 kg | 21.34 | 291.68 s / 70.98 kg | -3.49 |
| 102 | 438.90 s / 88.65 kg | -40.87 | 338.23 s / 52.46 kg | 15.53 |
| 103 | 330.92 s / 52.09 kg | 16.81 | 294.85 s / 47.05 kg | 27.96 |
| Mean | 361.91 s / 65.92 kg | **-4.27** | 301.34 s / 55.09 kg | **16.57** |

All eight flights entered the 50 m safe area without modeled debris or proxy
hull contact. Mean fuel decreased about **16.4%**, mean time decreased about
**16.7%**, and mean score increased **20.84 points**. These are four matched
cases, not a statistical guarantee or a native competition score. In seed 100,
the 24.06 kg saving includes changed guidance/avoidance behavior; it is not a
measurement of 24.06 kg of pure rotation fuel. Seed 101 explicitly demonstrates
that removing orientation can also increase fuel use.

Two intermediate alternatives were also tested on seed 100: nose-following with
no horizon-leveling scored -20.23, and keeping orientation only while burning or
near the station scored -19.40. Neither improved that case over the original
-14.35. Their raw records are retained rather than reporting only the winner.

The deployed `FreeAttitude` retains hysteretic rate damping above 0.01 rad/s,
stopping below 0.002 rad/s. This costs fuel if the vehicle is actually spinning.
It does not point the nose or level the horizon. Its exact source is rerun in
`tests/fuel_final_100.json` through `tests/fuel_final_103.json`; the first two runs
also include the full fuel audit. Spin damping is tested separately because the
zero-initial-spin model cases do not exercise that safeguard.

## Post-success fuel does not reduce the frozen score

The old seed 100 flight used another **12.64 kg** while braking and parking
after success; the no-orientation case used **18.06 kg** afterward. These amounts
are excluded from the table's scored fuel. The official scorer freezes its
success result at first entry into the safe area. Removing post-entry braking
would save unscored propellant but would not improve that recorded score.

## What changed in the live code

- `EconomyNavigator` no longer pauses to align the nose by default.
- `FreeAttitude` replaces forced nose/horizon control; acceleration commands
  still use the correct body-frame conversion.
- Useful-thrust fuel calibration works for sideways flight.
- The GUI says **FREE ATTITUDE** rather than repeatedly displaying ALIGNING
  when the nose is intentionally not following the velocity.
- `FuelLedger` uses the existing native fuel readings and adds
  `fuelPhaseKg=[...]` to the controller log every five seconds. It creates no
  extra simulator API polling and cannot command thrusters.

The native ledger is sampled at the planner rate, unlike the exact-tick offline
audit. Fuel spent across a change in mode is reported as `PHASE_TRANSITION`;
the uncertain boundary sample is reported as `ARRIVAL_TRANSITION`. Its ledger
freezes at the controller's first observed entry. That sample may be up to one
planning period later than official success, so the official Scoring log remains
authoritative for scored fuel. Simulation reset clears the ledger.

For controlled native A/B testing, set `EFP_NOSE_FIRST=1` in the environment
inherited by the simulator to restore the preceding behavior. Default is `0`.
Check `noseFirst=False` in the `SCORE_ECO` controller log to confirm the new
behavior. Deploy both `Controller.py` and `camera_viewer.py`; STU4-Sim is unchanged.

## Reproducing and inspecting the evidence

`tests/benchmark_final_submission.py --fuel-audit` adds per-segment and per-command
fuel measurements. `--attitude-mode off`, `nose-first`, `nose-only`, and
`burn-only` are offline experiments; `normal` uses the current live policy.
`--baseline-ref 412708b` loads the preceding source without changing this checkout.
Supply `--sim-config`, `--jet-config`, `--seed` and `--output` as in the existing
benchmark. Runtime control needs NumPy; this offline allocator model also needs
SciPy.

Detailed old-policy records are `tests/fuel_audit_normal_100.json` and
`tests/fuel_audit_normal_101.json`. Their matched no-orientation records are
`tests/fuel_audit_off_100.json` and `tests/fuel_audit_off_101.json`. The extra paired
seeds are `tests/fuel_compare_normal_102.json`, `tests/fuel_compare_off_102.json`,
`tests/fuel_compare_normal_103.json` and `tests/fuel_compare_off_103.json`.
All records identify their source hash, scenario hash and experiment mode.

This model uses synthetic images, simplified dynamics, a different LP solver and
an ellipsoid station hull. Native camera geometry, mesh collisions, external
torques, damage, and real control timing can differ. Neither a zero-contact model
result nor an average score of 16.57 establishes an official win or solves the
50–75 point target.
