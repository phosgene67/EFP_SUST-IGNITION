# Dogleg lane experiment

`EFP_DOGLEG=1` selects the new planner in Controller.py. It defaults to **off**.
The baseline remains SCORE_MOMENTUM. This is an experiment, not a verified
60-point native controller. No native flight trace was available for this task.

## Frames and evidence

**Verified installed-source facts, not native flight measurements:** Radar.py
reports shuttle-relative positions expressed in the Phobos body-fixed frame.
It reports only line-of-sight relative speed. Full three-dimensional debris
velocity is estimated by the existing timestamped tracker. Neither list order
nor a single radial measurement identifies a complete velocity vector.
FINAL_SUBMISSION configures position sigma 2 m, radial-speed sigma 0.02 m/s,
and diameter sigma 5%. Individual objects refresh on a rotating sweep.

Navigation and waypoint vectors use the Phobos frame. SetCommand takes shuttle
body-frame acceleration and angular acceleration. The existing world-to-body
DCM conversion is preserved. Body +X is the nose/strong translation axis; it
is not a hard-coded world direction. Every alignment uses the measured DCM
and a computed first-burn direction. Camera helpers and SetCommand are unchanged.

**Modeled allocator facts:** at reference mass 3388.7 kg, Isp 80 s and g0 9.8,
clean translational impulse costs `3388.7/(80*9.8) = 4.3223214 kg/(m/s)`.
The supplied axis accelerations agree with the published allocator calculation,
not an independent native calibration. Sequential axis burns cost the **L1 sum**
of their velocity increments, which can exceed the vector norm. Gravity,
opposing corrections and attitude control consume additional impulse.
Native fuelPhaseKg alone cannot validate kg/delta-v: the corresponding state,
command, body rotation and gravity history are needed.

**Modeled physics corrections:** an ideal equal-speed corner costs
`2*v*sin(theta/2)`; ideal continuous turning costs `v*theta`. At 20 degrees their
difference is only about 0.5%, so changing curve shape alone cannot explain a
large fuel saving. A bend allowance of 15–20% of cruise speed corresponds to
about **8.6–11.5 degrees**, not 20 degrees. Finite thrusters cannot produce an
instantaneous corner; the planner checks the actual finite burn instead.
Unpowered coasting uses no commanded thrust fuel but speed changes under
gravity. The profitable speed depends on acceleration, gravity and overhead;
9 m/s is not a universal physical cutoff.

## Implementation and assumptions

**Implemented policy:** straight routes and one-bend routes through waypoints
at 30%, 50% and 70% progress. Eight lateral azimuths cover both signs of both
transverse directions. The first grid uses nominal 4/8/12-degree bend offsets;
the wider grid uses 20/30 degrees. Four goals lie 44.72 m from the known ship
center. Candidate cruise parameters span 5–10 m/s in 0.5 m/s increments.
Finite burn dynamics determine actual speeds and bend angles, which are logged.

Every evaluated candidate checks **every current radar track** over the full
predicted route. A score upper bound skips candidates that cannot outscore an
already certified safe candidate. No debris entity/state query is made. The
wider grid is checked whenever its modeled score could beat the current best;
clearance is never reduced to force a solution. If none is certified, the
existing planner supplies the fallback. Cameras are ignored for lane decisions
outside 300 m; available camera geometry checks the final approach. Camera
availability does not gate a dogleg thrust request. Fallback retains the existing
terminal guard and can still wait for camera coverage.

**Modeled prediction:** existing own-vehicle gravity, constant local debris
gravity and tracked velocities. Uncertainty radii and velocity-growth bounds
come from the existing tracker. The additional route acceptance margin is 2 m.
A coarse swept chord checks all pairs with an acceleration-curvature bound;
ambiguous pairs are refined to at most 6 s intervals. Unit tests compare this
bound with dense finite-burn sampling and exercise moving future crossings.
Only fine sweeps may truncate after predicted 48 m entry. Actual frame-relative
50 m entry, checked by the flight loop, stops all commands. Predictions never
declare a mission complete.

**Attitude assumptions:** one initial minimum-angle +X alignment, then hold
attitude and damp spin. The bend uses sequential existing body axes with no new
slew. The slew model uses 0.006 rad/s² and a 0.12 rad/s cap. Its conservative
fuel allowance is `2*peak_rate*8*sqrt(3)` kg. A synthetic canonical-axis allocator
audit gave 3.98–7.30 kg/(rad/s); the allowance is an assumption for arbitrary
rotation, not a proven native bound. A risk during slew aborts the turn rather
than starting repeated pointing maneuvers. After alignment, plans are rebuilt
using actual pose and fuel; alignment is not charged a second time.

**Event policy:** search at launch completion, initial slew settlement, a new
predicted collision, or a predicted capture miss. After no safe lane is found,
retry requires at least 25 additional confident tracks or 100 m of progress.
Retained-route safety is checked on the existing 2 Hz sensor cycle. There is
no periodic route optimization or velocity servo maintaining a constant cruise
speed. Finite-burn completion feedback latches each axis off, then coasts.

This differs from the rejected rolling-corridor/bypass bundles: no hard reserve,
forced bypass side, continuously steered curve, endpoint gravity-gradient patch,
or distant-crossing score cost. It revisits initial axis alignment explicitly,
with its time and fuel included rather than assuming a free strong axis.

## Evidence so far

**Modeled baseline:** seeds 100–119, original 4a8507e, 60 Hz, published allocator
proxy, legacy radar RNG, 500 debris and ground-plane diagnostic at 2.5 m.
The prior cache-only replay reproduced all 20 baseline entry results. Baseline
median score 38.275; p10–p90 22.984–43.789; median time 264.97 s; median fuel
43.725 kg; 20 arrivals; zero modeled contacts. Native damage joules are not
simulated, so modeled contact is reported separately from native damage.

**Modeled diagnostic only:** V3 clear-corridor pilot seed100 arrived in 271.27 s
at 46.30 kg, estimated score 32.50 and no pre-entry contact. The initial slew
used 43.5 s and 1.3987 kg; initial dogleg burn used 31.7745 kg; coasting for
133.5 s used zero fuel. Later camera/terminal corrections consumed more fuel.
Six lane searches occurred; no bend was selected in this cleared scene. This
tests execution and exposes overhead; it does not establish a median gain.
See tests/score_dogleg_clear_pilot100.json. V1 was interrupted without results.
V2 pilot and V3 diagnostic are excluded from the frozen-source cohort.

**Completed modeled comparison:** seeds100–119 used the frozen
tests/score_sweeps/dogleg_final_controller.py with only dogleg enabled.
All results are offline proxies, not native measurements.

| Quantity | Original Momentum | Dogleg |
| --- | ---: | ---: |
| Median score | 38.275 | 27.890 |
| Score p10–p90 | 22.984–43.789 | 8.882–37.932 |
| Score range | 13.79–44.93 | -0.44–39.77 |
| Median entry time | 264.97 s | 245.845 s |
| Median entry fuel | 43.725 kg | 54.890 kg |
| Arrivals / modeled contacts | 20 / 0 | 20 / 0 |
| Native damage joules | Not simulated | Not simulated |

**Modeled paired result:** difference of score medians **-10.385 points**;
rounding-adjusted paired-bootstrap 95% interval **[-17.340,-3.030]**.
Median paired time delta **-18.335 s**; fuel delta **+9.445 kg**.
These paired medians need not equal the difference of the separate medians.
The result is rejected for score loss. All20 aligned; six executed one bend.
Maximum post-entry thrust fuel was0 kg. Median lane searches per run:23.5.

**Modeled phase-fuel changes, paired means** (means are additive):

| Phase | Dogleg minus baseline |
| --- | ---: |
| Existing main impulse burns | -3.240 kg |
| New dogleg initial burns | +8.202 kg |
| Dogleg bends | +1.826 kg |
| Alignment | +0.587 kg |
| Capture / observation / camera wait | +3.356 kg |
| Other phases combined | -0.804 kg |
| Total from unrounded phase ledger | +9.927 kg |

Mean total fuel difference from rounded entry fields is+9.930 kg. More burn
and terminal fuel outweighed the time saving. This falsifies the claimed gain
for **this implementation/configuration**, not every possible dogleg policy.
The archive preserves its rejected configuration for review; it is not promoted.
Full per-seed metrics, raw phase buckets, bootstrap interval and source hashes:
[paired report](tests/score_sweep_dogleg.json).

**Measured compute limitation:** per-run maximum planner calls ranged from
5.093 to6.763 s, median5.870 s, under four concurrent benchmarks. The individual-pair refinement is
faster, but that does not make exhaustive search suitable for the flight loop.
The harness integrates at fixed simulated time and does not charge this wall
time to entry time. Native held-command effects are therefore unverified.
Do not recommend enabling this version for competition based on proxy scores.

**Recommended configuration:** `EFP_DOGLEG=0`; leave other algorithm experiments
off. Keep SCORE_MOMENTUM active. Use `EFP_FLIGHT_TRACE=1` for a baseline native
diagnostic before tuning model discrepancies. The original native60+ goal
remains unmet and unverified. The180-test suite passes; preservation audit
confirms all47 original definitions and the official assets are unchanged.

Reproduce each seed using the installed SDK Python (paths abbreviated here):

```powershell
python -B tests/benchmark_final_submission.py --sim-config FINAL_CONFIG --jet-config JET_CSV --ground-plane 2.5 --fuel-audit --baseline-file tests/score_sweeps/dogleg_final_controller.py --experiment dogleg --seed 100 --output tests/score_dogleg_100.json
python -B tests/summarize_score_sweep.py --baseline-prefix tests/score_baseline_ --candidate-prefix tests/score_dogleg_ --output tests/score_sweep_dogleg.json
```

Repeat the flight command for100–119 with matching filenames before summarizing.

## Logs and native verification

Dogleg mode logs `profile=SCORE_DOGLEG`, `dogleg=True`, and the deployed source
hash. File-only IGNITION_DOGLEG_EVENT / IGNITION_DOGLEG records contain the route,
goal, speed, actual bend angle, initial/bend delta-v, assumed slew fuel,
predicted versus actual incremental fuel, clearance, event and search time.
fuelPhaseKg remains the native additive phase ledger. The camera GUI retains
its existing profile label; use the flight log to identify this experiment.

Only for a prototype diagnostic native trial, enable `EFP_DOGLEG=1` and
`EFP_FLIGHT_TRACE=1`; leave nose-first, axis-thrust and other experiment flags
off. Deploy Controller.py and retain the existing camera_viewer.py. Native
verification must establish actual slew time/fuel, initial and bend impulse
efficiency, search latency/held commands, moving-debris clearance and official
safe-zone entry with zero scored damage. Preserve the official success alert
and entry time/fuel/damage, not post-entry collision statistics.

Provide the new run folder name under the existing Logs root, including
FINAL_SUBMISSION.log and FINAL_SUBMISSION_PySys_Controller.log (plus the jet
actuator log if a command/dynamics discrepancy occurs). Confirm the source
hash, flags and IGNITION_STATE records. Twenty matched native runs remain
necessary to support a 60+ median claim; no such result is currently claimed.
