| [Rules & Info](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218) | [Competitor Starter Code (Here)](https://github.com/SimDynamX/STU4-Solution) | [Forum](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/categories/2026-escape-from-phobos)
| ----- | ----- | ----- |

# Escape from Phobos - Starter Code
All info is on the wiki here: https://github.com/SimDynamX/STU4-Solution/wiki

### Current Controller

`Controller.py` contains our final-round candidate: radar tracking,
asynchronous RGB/depth perception from both cameras, local trajectory planning,
adaptive cruise selection, and coasting/attitude control.

**Live camera GUI:** copy **both `Controller.py` and `camera_viewer.py`** into the
installed local `STU4-Solution` folder. The OpenCV window opens automatically during
the next simulation run, showing top/bottom RGB and depth, detection boxes, and
flight statistics. Press Q in the viewer to close it while flight continues.
See [Camera interface and FUSION25 setup](CAMERA_GUI.md) for preview commands and
troubleshooting. The requested 25 m/s cap remains subject to braking and avoidance;
the offline comparison did not show a score improvement.

See [Final system design and validation](FINAL_DESIGN.md) for implementation details,
offline results, and required live camera calibration. The earlier
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
