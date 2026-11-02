# Qualification controller baseline

`Controller.py` is self-contained and keeps the original protected connection and
helper section. No STU4-Sim files or simulator configuration have been edited.

The controller uses target-relative velocity guidance (12 m/s cruise cap), slows near
the destination, and chooses among up to 116 candidate velocities with acceleration-limited
40–65-second rollouts. The lookahead covers nominal stopping time at the requested
0.30 m/s² acceleration plus 8 seconds, subject to the 65-second cap. Achieved thrust
can be lower than requested; the lookahead is not a stopping guarantee.
It checks swept segments against inflated radar spheres,
including a 3 m shuttle radius, measurement/diameter margins, and increasing drift
uncertainty. Planning runs at most twice per simulation second. Commands are
transformed from the Phobos frame into the shuttle frame. Attitude guidance turns
body +X (front) toward actual travel and body +Z (top) toward local outward/up.
At low speed it follows the planned travel direction, then retains heading while
holding. Rate and angular-acceleration limits make turns gradual. It recovers from
an upside-down attitude and handles vertical-flight singularities. Translation
remains independent during turns so attitude alignment cannot delay avoidance.
Status logs appear every
five simulation seconds. Blocking camera captures have been removed. A velocity
servo refreshes thrust every control tick between planning updates. Commands respect
the configured actuator limits. Missing, invalid, or more than 10-second-old radar
causes braking toward target velocity (`WAIT_RADAR`) rather than blind navigation.

Terminal guidance locks an approach-side waypoint 45 m from the rescue ship centre
inside the official 50 m success sphere. Its rollout includes braking toward that
waypoint. `HOLD` indicates it is within 3 m of the waypoint with relative speed below
0.5 m/s; this is controller telemetry, not the official success alert. It continues
active position guidance after arrival instead of flying toward the ship centre.
The ship's full hull geometry is not supplied to the planner, so the offset still
needs visual verification in the simulator. A 12 m/s cap does not imply constant
12 m/s flight: acceleration, braking and obstacle avoidance reduce speed as needed.
Braking guidance retains the conservative 0.12 m/s² stopping-distance parameter.
Previously selected velocities and coasting choices are capped by the current
terminal speed envelope so they cannot bypass approach braking.

The FAST12 profile adds 12° and 25° steering alternatives near the goal direction
and an explicit coasting candidate. It penalizes accumulated predicted acceleration
(a translation-impulse proxy) and changes of velocity command to discourage
unnecessary burns and abrupt route switching. Collision constraints take priority
over these costs. `profile=FAST12`, `lookahead`, `planDV`, and `cmdAccel` in status
logs identify the version and requested maneuver; `planDV` is neither measured
fuel nor the total run's delta-v.

Radar timestamps are referenced to Radar.py startup. The controller approximates
that epoch with its own startup clock and reconstructs shuttle motion from history.
Successive reports are associated using predicted absolute position, timestamp,
and diameter, independent of radar ordering. Unambiguous matches estimate transverse
velocity with a low-pass filter; ambiguous matches retain a radial-only estimate.
Repeated reports are cached without treating them as fresh measurements. Prediction
margins grow by 1 m per second of measurement age and lookahead time.
Startup skew, association errors, unobserved transverse debris velocity, gravity,
and actual jet allocation remain sources of prediction error. Margins are heuristics,
not collision guarantees.
`ESCAPE` means every sampled trajectory intersected an inflated obstacle; it chooses
the least accumulated predicted penetration, allowing it to leave overlapping
uncertainty margins. Local planning can stall in dense fields.

## Offline validation

With Python and NumPy installed:

```powershell
python -m unittest discover -s tests -v
```

Twenty tests cover clear-route arrival, complete static-obstacle avoidance,
immediate obstacle response, stale measurement correction, transverse tracking,
radar reordering, repeated reports, escape from inflated overlap, fast incoming
debris, and preservation of the protected boilerplate. Three fixed-seed crossing
scenarios include 2 m position noise, noisy range rate/diameter, 1.5-second radar
updates, moving obstacles, a small gravity disturbance, and 10% thrust under-delivery.
Each enters a 48 m arrival threshold without physical-sphere contact. Additional
tests integrate attitude dynamics through a 90-degree turn and a 180-degree
upside-down recovery, check stationary/vertical guidance, and run a 1 km route
through arrival and prolonged holding with 10% thrust under-delivery. That route
exceeds 11 m/s, stays outside 40 m after arrival, and settles near the 45 m waypoint.
Additional checks cover high-speed lookahead and retained-command terminal braking.

Planner tests extract pure code using AST. Adapter tests execute the actual
controller against a mock Space Teams API and check a rotated body frame, command
limits, missing/invalid radar, connection, and zero-command cleanup. No tests connect
to Space Teams PRO. These simplified dynamics and API doubles do not reproduce the
real allocator, collisions, irregular geometry, or complete gravity field. Passing
is not qualification proof.

## Speed comparison

The user's latest live baseline was 195.98 s, 117.34 kg fuel, zero damage, score
−45.2. FAST12 has not yet been tested in Space Teams PRO.

Reproduce the synthetic comparison against the prior 10 m/s controller:

```powershell
python tests/benchmark_controller.py --baseline-ref 74f9f06f07fc47e412ac7d2e8cf52fb332f59cfb
```

Results are saved to `tests/benchmark_results.json`. Routes are 1305.8 m long;
cluttered cases use eight moving spheres, noisy 1.5-second radar updates, gravity
disturbance, and either 90% or 65% thrust delivery. All six compared runs arrived;
the cluttered runs retained positive physical sphere clearance.

| Synthetic case | Previous time | FAST12 time | Previous translation delta-v | FAST12 delta-v |
| --- | ---: | ---: | ---: | ---: |
| Clear, 90% thrust | 172.0 s | 164.0 s | 19.16 m/s | 22.75 m/s |
| Clutter seed 7, 90% thrust | 182.0 s | 165.5 s | 32.71 m/s | 27.09 m/s |
| Clutter seed 19, 65% thrust | 183.5 s | 174.5 s | 24.06 m/s | 27.05 m/s |

Higher speed improved time in these cases but increased translation effort in two
of them. This benchmark omits attitude fuel, real jet allocation and full hull
geometry, so it does not predict the official score or guarantee lower fuel use.
For the next live run, compare time, fuel, damage and score together against the
195.98 s / 117.34 kg / zero-damage baseline. Retain a higher-speed profile only if
the repeated simulator results justify the tradeoff.

## Required Space Teams PRO validation

1. Back up the installed `Local/Repos/STU4-Solution/Controller.py`, then copy this
   repository's `Controller.py` there. Keep the installed folder named
   `STU4-Solution`: the official configuration references that exact path.
2. Launch the official `STU4-Sim/Sims/COMPETITION_SIM.simconf` in Space Teams PRO.
   Confirm the `IGNITION` status log appears and no Python/allocator errors occur.
3. Check distance falls, body-frame thrust responds correctly at different
   orientations, the front faces travel, the top stays upright relative to Phobos,
   angular rates settle, and radar clearance behaves plausibly. Confirm `HOLD`
   occurs near 45 m after the official success alert, without touching the ship.
   Check planning latency with the actual simulator running.
4. Run the official environment repeatedly. Record the simulator's success/failure,
   score, elapsed time, damage, fuel, and any sustained `ESCAPE`/stalled behavior.
   The official scoring system alone decides success (currently a 50 m safe sphere).
5. Tune the speed/acceleration and uncertainty margins only after inspecting those
   runs. Capture the required submission evidence and freeze the tested controller.

No live simulator run or qualification success has been verified in this change.
The baseline does not alter scoring, debris entities, or the official simulation.
