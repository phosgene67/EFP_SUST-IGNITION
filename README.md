| [Rules & Info](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218) | [Competitor Starter Code (Here)](https://github.com/SimDynamX/STU4-Solution) | [Forum](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/categories/2026-escape-from-phobos)
| ----- | ----- | ----- |

# Escape from Phobos - Starter Code
All info is on the wiki here: https://github.com/SimDynamX/STU4-Solution/wiki

### Current Controller

The current `STAGED_COAST` controller targets `FINAL_SUBMISSION.simconf`. After
the guarded launch, it compares coasting with forward and lateral burns that
can reach the 600 m boundary. It projects each candidate under changing Phobos
gravity and uses predicted debris encounters and the fuel needed after the
boundary to choose a route. Radar tracks provide range and closing speed.
Compact depth detections and sequences of RGB bearings can refine sideways
motion when a single radar track agrees. The extended RGB calculation accepts
observations between 80 and 900 m and requires at least three matched frames.
Actual camera visibility depends on range, resolution, occlusion and gimbal aim.

The outer leg renews thrust authority for one second at a time while the
velocity servo cuts each burn when it achieves the selected velocity change.
Consecutive plans can extend a useful boost. Crossing 600 m switches control
to the existing station-pass planner. The flight loop checks the measured fuel
counter before every thruster command, including angular thrust and post-entry
braking. The log identifies the active budget as `fuelCeiling`.

| Flight stage | Cumulative fuel ceiling |
| --- | --- |
| Before first crossing 600 m | 20 kg. When less than 0.25 kg of planned thrust remains and the measured coast predicts a miss, the planner can use up to 3 kg from the later allowance. |
| From first crossing onward | The smaller of 50 kg and fuel measured at the crossing plus 30 kg. |
| At the limit | Both thrust commands go to zero with a 0.4 kg allowance for telemetry and actuator delays. |

The total includes fuel spent after the official score is recorded. If the
remaining allowance runs out during braking, the vehicle continues to coast.
An external controller can cut new commands but the simulator controls the
timing of a command already firing.

In the installed simulator logs, the preceding corridor controller used 48.08 kg
at 992.5 m in one flight and 38.41 kg at 993.9 m in another. Their official
scores were −6.0 and 13.0. Those logs locate the early expenditure but do not
distinguish useful acceleration from every avoidance correction.

The staged controller used 22.59 kg at both 1,000 m and 600 m in two
offline final-scenario runs. A 47 kg soft planning target after the handoff gave
the following results with the separate 50 kg command limit enabled.

| Controller and seed | Time to safe zone | Entry fuel | Total including post-entry test | Estimated score |
| --- | ---: | ---: | ---: | ---: |
| STAGED_COAST 101 | 446.52 s | 40.98 kg | 49.61 kg | 22.23 |
| STAGED_COAST 102 | 456.25 s | 41.18 kg | 49.61 kg | 21.40 |
| SCORE_MOMENTUM 101 | 252.95 s | 43.42 kg | — | 39.55 |

These runs used the published initial layout, synthetic cameras, noisy radar,
a 0.25 s planning delay, HiGHS jet allocation and the same modeled drift.
All three entered without modeled contact. On seed 101 the staged policy
saved 2.44 kg by entry but added 193.57 s. It is a fuel-control candidate.
The 55 and 70 point targets remain unverified. For example, at 41 kg and
zero damage the scoring formula requires entry within about 201 s for 55
points and 161 s for 70. In these two staged runs reaching 600 m alone took
233.48 s and 243.58 s. [Benchmark records](tests/staged_benchmarks.json).

See [Nonblocking control and fuel planning](SCORE_SERVO.md) for the previous profile.
[Coast revalidation](SCORE_COAST.md) documents the previous release. The
[Momentum-preserving controller](SCORE_MOMENTUM.md) documents the underlying architecture.
Forced nose pointing and horizon leveling are now disabled by default following
matched fuel/score comparisons. The GUI shows **FREE ATTITUDE**. See
[Fuel audit and orientation comparison](FUEL_AUDIT.md) for the fuel breakdown,
all paired results, native fuel logging and the optional comparison setting.
[SCORE_ECO.md](SCORE_ECO.md) records the preceding economy controller.
[SCORE_SHOOT.md](SCORE_SHOOT.md) records the first finite-burn version.
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
