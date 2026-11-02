# Qualification controller baseline

`Controller.py` is self-contained and keeps the original protected connection and
helper section. No STU4-Sim files or simulator configuration have been edited.

The controller uses target-relative velocity guidance (6 m/s cruise), slows near
the destination, and chooses among 99 candidate velocities with acceleration-limited
30-second rollouts. It checks swept segments against inflated radar spheres,
including a 3 m shuttle radius, measurement/diameter margins, and increasing drift
uncertainty. Planning runs at most twice per simulation second. Commands are
transformed from the Phobos frame into the shuttle frame; angular-rate damping
does not require pointing the shuttle at the destination. Status logs appear every
five simulation seconds. Blocking camera captures have been removed. A velocity
servo refreshes thrust every control tick between planning updates. Commands respect
the configured actuator limits. Missing, invalid, or more than 10-second-old radar
causes braking toward target velocity (`WAIT_RADAR`) rather than blind navigation.

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

Fourteen tests cover clear-route arrival, complete static-obstacle avoidance,
immediate obstacle response, stale measurement correction, transverse tracking,
radar reordering, repeated reports, escape from inflated overlap, fast incoming
debris, and preservation of the protected boilerplate. Three fixed-seed crossing
scenarios include 2 m position noise, noisy range rate/diameter, 1.5-second radar
updates, moving obstacles, a small gravity disturbance, and 10% thrust under-delivery.
Each reaches the 45 m arrival threshold without physical-sphere contact.

Planner tests extract pure code using AST. Adapter tests execute the actual
controller against a mock Space Teams API and check a rotated body frame, command
limits, missing/invalid radar, connection, and zero-command cleanup. No tests connect
to Space Teams PRO. These simplified dynamics and API doubles do not reproduce the
real allocator, collisions, irregular geometry, or complete gravity field. Passing
is not qualification proof.

## Required Space Teams PRO validation

1. Back up the installed `Local/Repos/STU4-Solution/Controller.py`, then copy this
   repository's `Controller.py` there. Keep the installed folder named
   `STU4-Solution`: the official configuration references that exact path.
2. Launch the official `STU4-Sim/Sims/COMPETITION_SIM.simconf` in Space Teams PRO.
   Confirm the `IGNITION` status log appears and no Python/allocator errors occur.
3. Check distance falls, body-frame thrust responds correctly at different
   orientations, angular rates settle, and radar clearance behaves plausibly.
   Check planning latency with the actual simulator running.
4. Run the official environment repeatedly. Record the simulator's success/failure,
   score, elapsed time, damage, fuel, and any sustained `ESCAPE`/stalled behavior.
   The official scoring system alone decides success (currently a 50 m safe sphere).
5. Tune the speed/acceleration and uncertainty margins only after inspecting those
   runs. Capture the required submission evidence and freeze the tested controller.

No live simulator run or qualification success has been verified in this change.
The baseline does not alter scoring, debris entities, or the official simulation.
