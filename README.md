| [Rules & Info](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218) | [Competitor Starter Code (Here)](https://github.com/SimDynamX/STU4-Solution) | [Forum](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/categories/2026-escape-from-phobos)
| ----- | ----- | ----- |

# Escape from Phobos - Starter Code
All info is on the wiki here: https://github.com/SimDynamX/STU4-Solution/wiki

### Current Controller

`Controller.py` contains our **SCORE_SHOOT** candidate for
`FINAL_SUBMISSION.simconf`: finite body-axis burns selected using the score formula,
coasting, gravity-aware moving-debris tracking, continuous radar/RGB/depth feedback,
and a guarded pass into the safe area followed by braking.
See [Finite-burn controller](SCORE_SHOOT.md) for the current architecture, fuel
measurements, score budgets and validation results. The +50/+70 targets remain
unverified in the native simulator.
Forced nose pointing and horizon leveling are now disabled by default following
matched fuel/score comparisons. The GUI shows **FREE ATTITUDE**. See
[Fuel audit and orientation comparison](FUEL_AUDIT.md) for the fuel breakdown,
all paired results, native fuel logging and the optional comparison setting.
[SCORE_ECO.md](SCORE_ECO.md) records the preceding economy controller.
[OPEN_LANE_9.md](OPEN_LANE_9.md) records the preceding controller.
[CHUNK_NAVIGATION.md](CHUNK_NAVIGATION.md) records the earlier chunk design.

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
