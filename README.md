| [Rules & Info](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218) | [Competitor Starter Code (Here)](https://github.com/SimDynamX/STU4-Solution) | [Forum](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/categories/2026-escape-from-phobos)
| ----- | ----- | ----- |

# Escape from Phobos - Starter Code
All info is on the wiki here: https://github.com/SimDynamX/STU4-Solution/wiki

### Current Controller

The controller restores **SCORE_MOMENTUM** from commit `4a8507e` for
`FINAL_SUBMISSION.simconf`. It evaluates finite burns and coast trajectories
against the time, fuel and damage score. Fuel use has no command cutoff or
staged reserve. All thrust stops when the scoring safe zone is reached.

The original 20-seed offline baseline had median score 38.28, median entry time
264.97 s and median entry fuel 43.73 kg. All 20 reached the safe zone without
modeled pre-entry contact. These are proxy results. A native 60–70 median
remains unverified.

Experiments are isolated behind default-off environment flags.

| Flag | Experiment | Status |
| --- | --- | --- |
| `EFP_CLEAN_MAIN_BURN=1` | Small transverse acceleration component during the main burn | Twenty-seed median gain +2.06, 95% interval -0.81 to +7.63. Inconclusive and rejected for promotion. |
| `EFP_ORDER_DIVERSITY=1` | Preserve a candidate for each body-axis burn order before collision checks | Twenty-seed median gain 0.00, 95% interval -0.07 to +2.44. Inconclusive and rejected for promotion. |
| `EFP_RECHECK_COAST=1` | Revalidate a completed retained coast against the safe passage requirement | Twenty-seed median change -0.31, 95% interval -0.62 to +0.74. Unit regression fixed; no demonstrated flight-score gain, so it stays off. |
| `EFP_INERTIAL_CAPTURE=1` | Keep finite-burn/coast planning through a camera-observed terminal approach | Twenty-seed median +0.815 points; marginal at the reported score precision. Stays off pending stronger evidence. |
| `EFP_VECTOR_BURN=1` | Offer a single fixed-direction translation burn followed by coast | Twenty-seed median -1.90 points with higher fuel. Rejected for promotion. |
| `EFP_EXACT_ARCS=1` | Solve and execute both finite burns of a curved detour | Both 20-seed revisions had one incomplete arrival. Rejected. |
| `EFP_COAST_WINDOWS=1` | Predict short coasts before burning through moving debris | Reduced fuel at 600 m by about 2 kg, but one arrival timed out. Rejected. |
| `EFP_CAMERA_RECOVERY=1` | Maintain station camera coverage during a clear coast | With coast windows, restored 20/20 arrivals but reduced median score. Stays off. |
| `EFP_ENTRY_SCOPE=1` | Evaluate collision checks up to predicted safe-zone entry | Twenty-seed isolated trial showed no meaningful median gain. Stays off. |
| `EFP_TRACK_CACHE=1` | Reuse fits for unchanged radar samples | All 20 modeled entry results matched the original; faster tracker fixture. Native score benefit unverified. Stays off. |

Copying this controller alone keeps the baseline flight policy: every experiment
above defaults to off. No tested trajectory change has established the requested
60-point native result or 30–35 kg fuel use by 600 m remaining.

For a native diagnostic run, set `EFP_FLIGHT_TRACE=1` before launching the
simulator, leaving algorithm flags off. Bounded `IGNITION_STATE` JSON records
are written to simulator logs every five seconds. They include pose, commands,
fuel and sensor samples needed to investigate the native/offline fuel gap.

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

Run the offline tests with Python and NumPy:

```powershell
python -m unittest discover -s tests -v
```

### Local Repository Setup

Our GitHub repository is named `EFP_SUST-IGNITION`, but the **local repository folder must be renamed to `STU4-Solution`** because the official `STU4-Sim` competition configuration references:

`Local/Repos/STU4-Solution/Controller.py`

This allows the simulator to execute our custom `Controller.py` without modifying the official simulation configuration.

**Note:** Only rename the local folder. Keep the GitHub repository name (`EFP_SUST-IGNITION`) unchanged.
