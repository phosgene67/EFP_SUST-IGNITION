# OPEN_LANE_9: wider routes, continuous handoffs and arrival hold

Use this controller with **FINAL_SUBMISSION.simconf**. Deploy both
`Controller.py` and `camera_viewer.py` and check for `profile=OPEN_LANE_9`.
The protected connection prefix and the official STU4-Sim remain unchanged.

## What the latest live run actually recorded

The November 15 run `261115_175654_FINAL_SUBMISSION__b3c` contains the official
congratulations in `FINAL_SUBMISSION_PySys_Scoring.log`, line 144:

- Score **-134.8**, elapsed **366.68 s**, fuel **163.08 kg**.
- Damage **49.13 J**, ship distance **49.98 m** at success.
- The later controller hold near **49.5 m** happened after that success.

The alert can disappear from the screen while the rover continues holding. The
November 12 `131930...a09` run with no success alert ended with controller distance
about **55.2 m**; its available log does not show entry into the 50 m scoring zone.
Do not infer an official success solely from a rounded controller display or a
mode named HOLD. The official Scoring log is authoritative.

## Changes

**Open-space preference.** The old beam score mainly rewarded progress and
penalized only very small clearance. A passable straight tube through many debris
objects could therefore beat a slightly longer open route. The planner now
compares the forward direction with sectors 15°, 30° and 45° away, looking up to
650 m ahead. It evaluates predicted debris positions at estimated arrival times,
soft congestion and momentum/turn costs. The resulting sector biases the bounded
position/velocity beam search. Integrated proximity cost also penalizes spending
time near clusters, rather than treating every non-colliding trajectory equally.
These soft preferences do not replace the hard swept collision checks.

**Successors before boundaries.** Prepare the next chunk when the remaining
current route falls below the larger of 350 m and stopping distance plus 250 m.
Prepare reserve space earlier too, and commit the successor at least 220 m or
two steering lookahead distances before the boundary. Ordinary chunk endpoints
are pass-through points. Slow down only when observations cannot justify enough
clear space, a turn/hazard requires it, or terminal braking has begun.
`nextReady` also counts successor samples already merged into the committed curve.

**A 9 m/s cruise target.** Request 9 m/s instead of the earlier 12 m/s request.
The aim is useful 8–9 m/s travel with less unnecessary thrust and replanning.
This is not a forced minimum: launch, acceleration, collision avoidance, missing
sensors and station braking can require less than 5 m/s. Forcing 5 m/s in those
conditions would recreate the station-impact problem. The hard 25 m/s ceiling,
live acceleration clamp and independent braking guard remain active.

**Latch arrival and keep the safe hold point.** Once measured range is below
49.5 m and relative speed below 1 m/s, remember the valid hold point relative to
the ship. Camera coverage loss or a newly inflated hull margin no longer clears
that point and starts another outward survey/retreat. Local collision checking
continues; drift produces restoring guidance. The final target remains around
49 m, not the station center. A regression checks that a later hull-margin
observation cannot send an arrived rover outward.

**Persistent indication.** The camera GUI retains an **ARRIVAL ZONE REACHED**
banner, and the controller emits its own one-time arrival-zone notice. This is
explicitly a controller position observation, **not a replacement or fabricated
copy of the official congratulations or score**. Consult the Scoring log for
official success. If scoring is absent or fails, the controller cannot repair the
official scorer by displaying its own win message.

**Windows viewer file locking.** The November 15 log also reported access denied
while replacing the snapshot. The viewer now closes the file before NPZ
decompression; the background publisher retries brief replacement locks and clears
recovered snapshot faults. Camera perception continues independently of the GUI.

![Synthetic preview of persistent arrival indication](docs/open_lane_arrival_demo.png)

This is a synthetic layout preview, not a live flight image.

## Validation

All **74 tests** pass using the installed STPro v0.56.0 Python environment. New
regressions cover a clear-but-crowded direct tube versus open space, early
successor preparation/commit, post-arrival drift and false hull margins, the
persistent banner, and transient Windows file locks. Existing noisy radar,
camera failures, collision avoidance, runtime adapter and protected-prefix checks
remain active.

Two full offline final-scene flights, each with 500 noisy cycling radar tracks,
delayed synthetic camera observations and the official jet geometry, produced:

| Seed | Arrival | Fuel | Peak | Mean cruise after first reaching 5 m/s | Minimum in that cruise interval | Minimum debris clearance |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 100 | 357.1 s | 201.94 kg | 8.99 m/s | 7.88 m/s | 5.00 m/s | 35.82 m |
| 101 | 347.2 s | 205.73 kg | 8.99 m/s | 7.71 m/s | 5.00 m/s | 25.98 m |

Both arrived without modeled contact and remained held for 30 further seconds.
The previous chunk model took 405.9/388.1 s with minimum debris clearances of
22.44/20.99 m for those seeds. Fuel/score changes are mixed, so faster arrival is
not a claim of maximum competition score. Full-mission average progress remains
lower than cruise speed because launch, braking and ship approach are included.

The model uses simplified dynamics and an ellipsoid **proxy** ship hull. It does
not certify real mesh clearance or native speed. Saved results contain the exact
controller/scenario hashes:

- `tests/open_lane_results.json`
- `tests/open_lane_seed101_results.json`

Reproduce with NumPy/SciPy:

```powershell
python -B -m unittest discover -s tests -v
python -B tests/benchmark_final_submission.py --sim-config "path\to\STU4-Sim\Sims\FINAL_SUBMISSION.simconf" --jet-config "path\to\STU4-Sim\Data\SpaceBuggyJetConfigNoMains.csv" --output tests/open_lane_results.json
```

For the second flight add `--seed 101 --output tests/open_lane_seed101_results.json`.
In the next native run retain `laneTurn`, `nextReady`, `buffer`, `replans`,
`arrivalZone`, measured speed, damage and the complete official scoring output.
