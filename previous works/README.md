# Previous works

These 17 documents preserve superseded algorithms, earlier controller revisions,
and historical experiment reports. They were archived on **21 November 2026**.
They are retained as development evidence; their descriptions of the "current"
controller, flags, speed settings, and scores belong to their original revision.

The maintained implementation reference is
[Final Submission Algorithm](../Final%20Submission%20Algorithm.md).
The latest validation record remains
[the November 20 experiment report](../tests/nov20_experiment_report.md).

## Archived document list

| Document | Why it is historical |
| --- | --- |
| [QUALIFICATION.md](QUALIFICATION.md) | Qualification/FAST12 architecture and tests, superseded by the final-round controller. |
| [FINAL_DESIGN.md](FINAL_DESIGN.md) | Earlier radar/camera architecture and cruise design. |
| [FINAL_SUBMISSION.md](FINAL_SUBMISSION.md) | The earlier FINAL_NAV_500 implementation, rather than the delivered submission algorithm. |
| [CHUNK_NAVIGATION.md](CHUNK_NAVIGATION.md) | CHUNK_NAV_12 development and its earlier launch/cruise settings. |
| [OPEN_LANE_9.md](OPEN_LANE_9.md) | Earlier lane-selection and arrival-hold design. |
| [SCORE_ECO.md](SCORE_ECO.md) | Preceding economy-controller revision and historical comparisons. |
| [SCORE_SHOOT.md](SCORE_SHOOT.md) | Initial finite-burn implementation and preceding native runs. |
| [SCORE_MOMENTUM.md](SCORE_MOMENTUM.md) | Original Momentum revision; the current family name is retained, but its delivered configuration and evidence are documented in the new paper. |
| [SCORE_COAST.md](SCORE_COAST.md) | Superseded coast-revalidation experiment. |
| [SCORE_SERVO.md](SCORE_SERVO.md) | Earlier asynchronous-planner/servo experiment; its worker architecture is not the default delivered control loop. |
| [MOMENTUM_TERMINAL.md](MOMENTUM_TERMINAL.md) | Earlier terminal/completion change record, consolidated in the new implementation reference. |
| [FUEL_AUDIT.md](FUEL_AUDIT.md) | Earlier nose-control and phase-fuel comparisons. |
| [SCORE_TUNING.md](SCORE_TUNING.md) | Historical tuning sequence and experiment settings; its opening defaults are not the delivered defaults. |
| [DOGLEG.md](DOGLEG.md) | Experimental dogleg planner, not selected for the delivered controller. |
| [DOGLEG_DIAGNOSTICS.md](DOGLEG_DIAGNOSTICS.md) | Diagnostic work on that experimental planner. |
| [GROUND_LATERAL.md](GROUND_LATERAL.md) | Experimental ground-aligned dogleg basis. |
| [tests/score55_experiment_report.md](tests/score55_experiment_report.md) | Earlier tuning/finalization report; the later November 20 evidence remains under the active tests directory. |

## Documents kept outside the archive

| Document | Current purpose |
| --- | --- |
| [README.md](../README.md) | Team information, setup, dependency installation, and deployment. |
| [Final Submission Algorithm.md](../Final%20Submission%20Algorithm.md) | Complete implementation architecture, equations, control flow, and evidence. |
| [CAMERA_GUI.md](../CAMERA_GUI.md) | Current camera-interface operation and troubleshooting. |
| [context.md](../context.md) | Maintained development context, selected implementation, and prior decisions. |
| [AGENTS.md](../AGENTS.md) | Repository working instructions. |
| [tests/nov20_experiment_report.md](../tests/nov20_experiment_report.md) | Latest finalized comparison and native-validation record. |

Archiving documentation does not change `Controller.py`, the simulator, Python
tests, experiment snapshots, or saved JSON evidence. Historical reproduction
commands assume their documented code revision and a terminal at the repository
root; an archived flag is not a recommendation to enable it in the final version.
