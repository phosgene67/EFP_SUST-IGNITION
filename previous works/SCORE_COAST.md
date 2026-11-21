> **Historical document — archived 21 November 2026.**
> This describes an earlier revision or experiment. For the delivered implementation, see
> [Final Submission Algorithm](../Final%20Submission%20Algorithm.md). Archived commands and flags refer to their historical revision.
> Original location: `SCORE_COAST.md`.

# Coast revalidation: SCORE_COAST

This is an incremental correction to `SCORE_MOMENTUM`. It does not establish
a +70 result. The delivered algorithm keeps the working finite-burn planner,
free attitude, guarded launch, radar/RGB/depth feedback and arrival control.

## The change

The planner gives an existing safe burn a score allowance so that small changes
in estimates do not repeatedly reverse its steering. That allowance previously
also applied after all its thrust had finished. A retained zero-thrust plan
could therefore continue even when its predicted closest approach drifted
outside the intended passage around the ship.

A completed burn now has to pass the same ballistic arrival check as a new
coast: predicted closest approach between 42 and 48 metres, still ahead of the
vehicle. If it fails, it competes for a fresh correction instead of receiving
retention credit. An unfinished burn or a planned two-stage detour remains
eligible for retention; those trajectories still have future thrust to apply.
All candidates continue through the existing sensor clearance and damage-risk
checks. Camera checks around the station and the official 50 m success-distance
query remain active.

The approach-side search also checks the actual camera gimbal limits. It
prefers a tangent pass from which a camera can cover the ship at 150 m and
80 m inbound distance, using the existing projection and image-edge margin.
The actual curved zero-thrust coast is checked too, so it cannot bypass this
constraint. This can avoid braking into `WAIT_SHIP_CAMERA` with the ship behind
both cameras. It does not require a body turn. If the current attitude makes
every sampled side unreachable, the original camera hold remains available;
the planner does not assume visibility or bypass fresh depth observations.
Reachability is not proof of an unoccluded view or native camera calibration.

A separate synthetic stress case reproduced the unreachable-camera failure
by shortening only the low-speed sensor horizon in the offline namespace. The
earlier version stopped outside the zone and consumed 73.11 kg by the 500 s
test limit. With the view constraint, the same seed arrived in **257.55 s using
40.37 kg**, estimated score **42.77**, without modeled contact or any attitude
command. This is a robustness check under a rejected tuning condition, not a
native performance result or justification to shorten the delivered horizon.
The successful result is `tests/coast_camera_stress101.json`; the matched
guard-disabled result is `tests/coast_camera_reference101.json`. Reproduce it using
`tests/benchmark_camera_reach.py`; `--without-view-guard` disables just the
view constraint for comparison.

The purpose is to correct a drifting pass while there is still time. For the
same 10 m sideways displacement, an ideal early correction with 100 s remaining
needs about 0.1 m/s, whereas one with 20 s remaining needs about 0.5 m/s. Actual
finite burns, gravity and obstacles also affect the cost.

## Measured result

Matched offline tests used all 500 public final-scene debris objects, the
60 Hz servo, noisy radar, rendered cameras and a synthetic ground plane 2.5 m
below the initial vehicle centre. The reference is commit `4a8507e`.

| Seed | Reference time | New time | Reference fuel | New fuel | Reference score estimate | New score estimate |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 100 | 255.93 s | 259.30 s | 38.97 kg | 38.21 kg | 44.93 | 45.33 |
| 101 | 264.87 s | 264.87 s | 39.20 kg | 39.20 kg | 43.03 | 43.03 |
| 102 | 263.35 s | 263.35 s | 41.65 kg | 41.65 kg | 40.03 | 40.03 |
| 103 | 250.83 s | 250.83 s | 46.59 kg | 46.59 kg | 35.72 | 35.72 |

Mean fuel changed from **41.60 to 41.41 kg** and mean estimated score from
**40.93 to 41.03**. Mean arrival time increased by 0.84 s. This is a small
improvement and a correction to a demonstrated planning failure, not evidence
of a large score increase. All four arrived without modeled contact and then
stopped. Results are in `tests/coast_final100.json` through `103.json`.

The final seed-100 comparison uses the official allocator's **revised simplex**
solver, published equations and jet geometry, without solver retries.
The old controller was checked with revised simplex too; its result is in
`tests/coast_reference_native100.json`. This validates that solver choice did
not create this measured gain. HiGHS development runs agreed on the same rounded
fuel/time results. Neither solver comparison reproduces the native physics engine.

The seed-100 audit records 3.31 kg in launch, 1.35 kg in immediate escape,
31.82 kg in cruise impulse burns and 1.72 kg in capture/observation. Its
169.23 s without a thrust command consumed no modeled fuel. A further
23.43 kg used after entry is excluded from the score. Launch alone is already
a small fraction of total fuel; cruise steering remains the main opportunity.

The model uses simplified gravity, spherical debris and an ellipsoid station
proxy. It does not validate the Unreal collision mesh, native damage law or
real camera calibration. Its zero-damage scores are estimates, suppressed if
the modeled flight fails or makes contact.

## Native evidence and the +70 budget

Recent native logs, before this change, contain:

| Run directory | Time | Fuel at success | Damage | Official score |
| --- | ---: | ---: | ---: | ---: |
| `261117_140232_FINAL_SUBMISSION__73b` | 279.26 s | 42.34 kg | 0 J | 36.5 |
| `261117_152214_FINAL_SUBMISSION__50a` | 260.02 s | 39.63 kg | 0 J | 43.3 |
| `261117_153305_FINAL_SUBMISSION__991` | 291.05 s | 49.32 kg | 0 J | 25.5 |
| `261117_154246_FINAL_SUBMISSION__42b` | 304.83 s | 54.68 kg | 0 J | 16.5 |

These logs lack controller source hashes, so they are observational evidence,
not controlled comparisons. Extracted alerts and telemetry are preserved in
`tests/coast_native_baseline.json`. The new build logs `profile=SCORE_COAST`
and `sourceSHA256=...` so subsequent native results can identify the actual file.
The native fingerprint hashes file bytes; benchmark `controller_sha256` hashes
source with normalized line endings. Final benchmark artifacts also record the
byte hash as `controller_file_sha256` for direct comparison.

The installed score formula is:

```text
Score = 100 * (0.5 + 120/T - F/75 - 2*D/209200)
```

At zero damage, +70 requires approximately these combinations:

| Arrival time | Maximum fuel at success |
| --- | ---: |
| 160 s | 41.25 kg |
| 180 s | 35.00 kg |
| 200 s | 30.00 kg |
| 220 s | 25.91 kg |
| 260 s | 19.62 kg |

One kilogram costs 1.333 points. At 260 s, saving ten seconds is worth about
1.85 points. Spending several extra kilograms for that time saving lowers the
total score. A 40 kg flight near 260 s cannot reach +70 through a small tuning
change. Both the transfer time and corrective impulse need substantial gains.

## Experiments retained as evidence

`tests/coast_tuning_history.json` records development results and experimental
source fragments. Tested variants included a one-time turn toward a stronger
thrust axis, small angular trims that increase available translation through
the unmodified allocator, delayed burns, an extra forecast fuel reserve,
nonlinear gravity prediction, short pulse planning and camera-bound changes.

Several variants shortened the flight while increasing fuel enough to reduce
the score. For example, the +Z trim alone reached seed 100 in 220.27 s but used
50.64 kg, estimated score 36.95, below the reference 44.93. Revised simplex
confirmed the same result. Those trims are not enabled in the delivered
controller. This release retains the existing 20-second sensor horizon.

## Checks and deployment

The **124 passing tests** include regressions for a retained coast that misses
the safe zone, a retained pass too close to the ship centre, preservation of a
successful unpowered coast, camera gimbal reach, approach-side visibility,
rejection of an unviewable free coast, and the runtime source fingerprint. Existing
connection, camera, launch, moving-track and arrival checks also pass:

```powershell
python -m unittest discover -s tests -v
```

`tests/benchmark_final_submission.py --allocator-solver native` selects revised
simplex for an offline cross-check. It requires a SciPy version that supports
that solver, such as the installed simulator Python. Native-solver failure
raises an error rather than silently substituting another solver.

Deploy `Controller.py` alongside the existing `camera_viewer.py` in the local
`STU4-Solution` folder. The live GUI receives the new profile automatically;
its standalone demo remains an example of the preceding profile. Keep
`EFP_NOSE_FIRST` and `EFP_AXIS_THRUST` unset or `0`. Confirm `SCORE_COAST` and
the file fingerprint in the native log, and compare the official success
time, fuel and damage over several fresh runs. No simulator configuration or
official `STU4-Sim` source was changed.
