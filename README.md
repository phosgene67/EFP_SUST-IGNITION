| [Rules & Info](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218) | [Competitor Starter Code (Here)](https://github.com/SimDynamX/STU4-Solution) | [Forum](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/categories/2026-escape-from-phobos)
| ----- | ----- | ----- |

# Escape from Phobos - Starter Code
All info is on the wiki here: https://github.com/SimDynamX/STU4-Solution/wiki

### Current Controller

The current controller is **SCORE_MOMENTUM**, restored from commit
`4a8507e` for `FINAL_SUBMISSION.simconf`. It compares finite burns and
unpowered passes using the official time, fuel and damage formula. Radar and
RGB/depth observations update predicted debris motion. Near-term collision
checks cover the next 20 seconds and replan at 2 Hz. Existing burns retain useful
momentum, with a camera-checked tangent pass into the ship's 50 m safe area.

The restoration removes the staged 600 m fuel handoff. Fuel is priced in the
score calculation. Free attitude remains the default. The controller checks
official ship-frame distance between planning updates and records entry fuel
before post-entry braking.

Radar processing now reuses unchanged report transforms and fitted uncertainty
until fresh measurements arrive. This reduces repeated work during guidance.
In an 80-update matched replay with 500 reports, median radar processing fell
from 74.17 ms to 30.05 ms with identical outputs. All 127 tests passed.
The exact restored baseline reproduced offline scores 44.93, 43.03, 40.03 and
35.72 on seeds 100–103. The user's live Momentum runs reportedly reach about 50.
A native 65+ result remains unverified.
The restored controller with caching reproduced those four scores and the same
entry time and fuel. [Full comparisons](tests/momentum_restoration_benchmarks.json)
include the rejected prediction experiments.

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
