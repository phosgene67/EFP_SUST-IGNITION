| [Rules & Info](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218) | [Competitor Starter Code (Here)](https://github.com/SimDynamX/STU4-Solution) | [Forum](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/categories/2026-escape-from-phobos)
| ----- | ----- | ----- |

# Escape from Phobos - Starter Code
All info is on the wiki here: https://github.com/SimDynamX/STU4-Solution/wiki

### Current Controller

The current controller is **MOMENTUM_BYPASS**, based on SCORE_MOMENTUM
at commit `4a8507e`, for `FINAL_SUBMISSION.simconf`. It compares finite burns and
unpowered passes using the official time, fuel and damage formula. Radar and
RGB/depth observations update predicted debris motion. Near-term collision
checks cover the next 20 seconds and replan at 2 Hz. Existing burns retain useful
momentum, with a camera-checked tangent pass into the ship's 50 m safe area.

The restoration removes the staged 600 m fuel handoff. Fuel is priced in the
score calculation. Free attitude remains the default. The controller checks
official ship-frame distance between planning updates and records entry fuel
at the safe-zone crossing.
After entry, the active controller now disables all thrust and coasts.
Within 550 m, it prioritizes a gravity-aware coast that predicts safe-zone entry,
with continuing debris and camera checks. A predicted miss or obstruction
returns control to ordinary guidance.

Radar observations rank curved corridors around the debris band. Broader route
changes require at least 20 mature tracks and a settled burn. Both initial and
return burns count toward cost. A curve needs more than two points of forecast
benefit over an available safe direct plan.

Inside 550 m, capture targeting moves inward to 43 m and guidance uses the
official scoring-origin vector. Short finite corrections remain available near
the ship. Surface checks end at first safe-zone entry.

The camera requests a 100-degree field so the station remains observable during
a body-Z climb with the gimbal at its limit. This addresses the camera-wait
stop reported near 63 m. A latched command cutoff disables translation and
angular thrust at 49.5 kg total fuel, leaving a 0.5 kg delay allowance against
the requested 50 kg ceiling. The cutoff also applies after entry. The vehicle
coasts once the allowance is exhausted.
144 tests pass. Four full-scene offline runs entered without modeled pre-entry
contact using 43.09–45.81 kg. Estimated scores were 34.15–38.42, averaging
36.24 versus 36.98 for the preceding controller. This is a capture-reliability
update with a small modeled score decrease. Native verification and the
60-point target remain pending.
[Bypass comparisons](tests/momentum_bypass_benchmarks.json) include the
rejected wider-route tunings. [Final-coast comparisons](tests/momentum_final_coast_benchmarks.json)
preserve the preceding release.

Radar processing reuses unchanged report transforms and fitted uncertainty until
fresh measurements arrive. In the earlier matched 500-report replay, median radar
processing fell from 74.17 ms to 30.05 ms with identical outputs.
[Restoration comparisons](tests/momentum_restoration_benchmarks.json) and
[camera-fix comparisons](tests/momentum_camera_budget_benchmarks.json)
preserve the preceding experiments.

Read [project context](context.md) before future changes. It records the active
architecture, decisions, historical regressions and current verification.
[Momentum design](SCORE_MOMENTUM.md) describes the restored algorithm.
[Fuel audit](FUEL_AUDIT.md) explains the attitude and thruster tradeoffs.
Earlier experiments remain in the repository for comparison.

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
