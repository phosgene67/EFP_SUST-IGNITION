| [Rules & Info](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218) | [Competitor Starter Code (Here)](https://github.com/SimDynamX/STU4-Solution) | [Forum](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/categories/2026-escape-from-phobos)
| ----- | ----- | ----- |

# Escape from Phobos - Starter Code
All info is on the wiki here: https://github.com/SimDynamX/STU4-Solution/wiki

### Current Controller

The selected controller uses **SCORE_MOMENTUM** for
`FINAL_SUBMISSION.simconf`, with a single +X-axis turn, radar-based cruise
checks, scored finite burns, small local avoidance maneuvers, and cached radar
fits. RGB/depth cameras remain active for launch terrain and the station hull.
Thrust stops after confirmed entry into the official 50 m safe zone. There is
no hard fuel cutoff and no continuous nose-pointing servo.

Selected defaults are `EFP_STRONG_AXIS=1`, `EFP_RADAR_CRUISE=1`,
`EFP_EXPECTED_RISK=1`, `EFP_FINE_AVOIDANCE=1`, and `EFP_TRACK_CACHE=1`.
Later experiments were rejected and archived; dogleg is not the active planner.

Finalized at the user's request with the following measured limitation. The selected family has a measured native best of **47.2**. Two fresh native
validation runs scored43.5 and24.2 (mean33.85), both with zero scored damage.
The requested47-50 average was not reproduced; no claim of that average is made.
The matching 20-seed offline cohort had median45.15, entry220.33 s,
fuel43.475 kg and no modeled contacts. Those proxy results do not establish
native performance. See [experiment evidence](tests/score55_experiment_report.md)
and [maintained context](context.md) for source hashes and official results.

Historical experiment flags and their results are documented in
[the experiment record](tests/score55_experiment_report.md). They are not
required to run the selected controller. Native progress and official success
alerts are written to the simulator's `FINAL_SUBMISSION.log`.

[Score tuning and verification](SCORE_TUNING.md) contains the score calculation,
matched results, phase-fuel audit, limitations and reproduction commands.
[Project context](context.md) records current instructions and historical
regressions. [Original Momentum design](SCORE_MOMENTUM.md) documents the base.

**Live camera GUI:** copy **both `Controller.py` and `camera_viewer.py`** into the
installed local `STU4-Solution` folder. The OpenCV window opens automatically during
the next simulation run, showing top/bottom RGB and depth, detection boxes, and
flight statistics. Press Q in the viewer to close it while flight continues.
See [Camera interface setup](CAMERA_GUI.md) for preview commands and
troubleshooting. The economy policy evaluates speeds up to 10 m/s; its selected
speed depends on projected score, measured jet performance and observed clearance.

See [Earlier fusion design](FINAL_DESIGN.md) for sensor implementation details,
historical offline results, and required live camera calibration. The earlier
[qualification guide](QUALIFICATION.md) preserves the FAST12 development record.

Run the selected completion and runtime checks with the simulator's Python:

```powershell
python -m unittest discover -s tests -p test_momentum_capture.py -v
python -m unittest discover -s tests -p test_active_runtime_profiles.py -v
```

The wider test archive includes retired Dogleg and trace adapter expectations.
The final validation uses125 targeted current/archived checks; this is not a
claim that every historical test is compatible with the selected controller.

### Local Repository Setup

The GitHub repository is `EFP_SUST-IGNITION`. Use these two separate checkouts:

- **Development:** `E:\EFP_SUST-IGNITION-main` — edit code here, then commit and push to GitHub.
- **Simulator:** `C:\Users\Asus\AppData\Local\SimDynamX\STPro\Assets\Repos\STU4-Solution` — update this checkout using **Pull** in Space Teams.

The simulator checkout must be named `STU4-Solution` because the official
competition configuration references `Local/Repos/STU4-Solution/Controller.py`.
The development folder can keep its current name. Keep the GitHub repository
name (`EFP_SUST-IGNITION`) unchanged.

After editing: **commit and push from the development repository, then Pull in
the simulator**. Avoid editing files or using Commit & Push in the simulator
checkout; separate local commits there can cause divergence.
