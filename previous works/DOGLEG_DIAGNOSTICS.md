> **Historical document — archived 21 November 2026.**
> This describes an earlier revision or experiment. For the delivered implementation, see
> [Final Submission Algorithm](../Final%20Submission%20Algorithm.md). Archived commands and flags refer to their historical revision.
> Original location: `DOGLEG_DIAGNOSTICS.md`.

# Task 1: dogleg diagnostics, no flight-policy change

Scope: Task 1 only. Ground-aligned axes, absolute offsets, early waypoints and
data-driven offsets are not implemented. The user requested a stop/report after
each task. The current checkout defaults EFP_DOGLEG to **1**, following the
user's commit `65f0b41`; this task preserves that choice. Earlier default-off notes
refer to the original experiment and its recommendation, not this checkout.

## Verified code findings

The current generator uses fractions 0.3/0.5/0.7 and an added lateral radius of
`distance*fraction*(1-fraction)*tan(angle)`. At 1,300 m and fraction 0.5 these radii
are 22.7/45.7/69.1 m for 4/8/12 degrees and 118.3/187.6 m for 20/30 degrees. These
are the explicit added offsets; the off-center capture aim also contributes
to the waypoint's total displacement. Finite burns determine the actual turn.

The transverse basis is derived from the shuttle DCM. Its eight azimuths span
the transverse plane, but an exactly surface-parallel direction is not assured
to lie on that discrete grid. No 0.2 fraction is generated. These facts verify
the proposed representation limitations; they do not establish that offset
range alone causes every rejected route.

Frame correction: scene centers supplied to select_lane already are
**shuttle-relative vectors expressed in the Phobos frame**. The tracker computes
`track_position - shuttle_location` and propagates measurement age. Therefore
the diagnostic uses `rel = centers`, not `centers - loc`. Camera-derived
surfaces appended after the radar prefix are excluded from debris statistics.

## Logged data

Every no_safe_lane event now contains:

- simulated timestamp, total searches and persistent no_safe_lane count;
- distance and the strict slab limits 50 m and distance-50 m;
- the count and lateral p50/p90/p100 of nominal radar centers inside that slab;
- explicit frame/relative-coordinate labels and the existing search_ms value.

Empty slabs have count 0 and JSON null percentiles. No radii or uncertainty
inflation is folded into these percentiles. No simulator debris truth is read.

Selection records retain origin, waypoint, goal, speed, bend angle, per-burn L1
delta-v, predicted incremental fuel, selection fuel and observed incremental
fuel. Closure records freeze observed fuel at route closure. A run_summary
records lifetime searches/failures and the last selected route, using current
fuel only while that route remains active. Later fallback fuel is not charged
to an already closed route.

The lifetime counters survive the bounded lane_records buffer being flushed
to native logs. Native summaries use IGNITION_DOGLEG_SUMMARY periodically and
on arrival; event records remain IGNITION_DOGLEG_EVENT. The benchmark appends
its final summary using modeled entry fuel. JSON records are available in the
controller's simulator log, not only the on-screen display.

## Model and comparison

The diagnostic cohort uses seeds 100–119, EFP_DOGLEG enabled, legacy radar RNG,
60 Hz, the same installed FINAL_SUBMISSION scene, published allocator proxy,
synthetic cameras and the 2.5 m ground-plane diagnostic. Compare it with the
pre-diagnostic dogleg cohort, not with an unrelated Momentum version, to verify
that instrumentation preserves decisions and phase fuel.
The reference archive predates the default-on commit, but both benchmark
cohorts explicitly enable dogleg; that default-setting difference does not
change the tested policy.

The diagnostic frozen source is
tests/score_sweeps/dogleg_diagnostic_controller.py. Per-seed outputs are
tests/score_dogleg_diag_SEED.json. The source audit confirms unchanged candidate,
burn and servo methods, 51 other existing definitions, connection prefix and
official simulation files. Five new tests cover relative frames, rigid-transform
invariance, empty slabs, camera exclusion, persistent counters and route-fuel
endpoints. Two runtime checks cover native-summary emission and logging failure
without interrupted arrival commands. The full suite passes 187 tests.

## Completed 20-seed baseline

**Modeled results, current Dogleg-enabled policy:**

| Quantity | Result |
| --- | ---: |
| Median score | 27.890 |
| Score p10–p90 | 8.882–37.932 |
| Score range | -0.44–39.77 |
| Median entry fuel | 54.890 kg |
| Fuel p10–p90 | 46.689–65.790 kg |
| Median entry time | 245.845 s |
| Entry time p10–p90 | 231.272–254.910 s |
| Arrivals / modeled contacts | 20 / 0 |
| Runs hitting no_safe_lane | 20/20 (100%) |
| Failed lane searches | 400 of 467 (85.7%) |
| Median searches per run | 23.5 |
| Median failed searches per run | 20 (range 16–24) |

**Modeled diagnostic, first failed-search slab per run:** all first failures
occurred at simulated 8.5 s, about 1,303 m from the station. Summaries below
are medians across the 20 run-wise values, not pooled debris percentiles.

| Slab quantity | Median across runs | Range across runs |
| --- | ---: | ---: |
| Tracked-center count | 481 | 480–481 |
| Lateral p50 | 244.86 m | 243.40–246.38 m |
| Lateral p90 | 346.19 m | 343.70–347.52 m |
| Lateral maximum | 461.96 m | 458.35–465.85 m |

Every run's first p90 exceeds 100 m. Of all 400 failed searches, 357 had a
nonempty slab; 43 had no eligible centers. Thus not every failure can be
explained by nominal center spread in the specified slab. Later failures
include a different approach geometry and terminal checks.

**Measured validation of the offline replay:** all 20 reported entry scores,
times, fuels, contact indicators and complete fuel-audit dictionaries match
the pre-diagnostic archive at its recorded precision. Paired time, fuel and
score deltas are zero. This confirms the expected diagnostic-only policy
equivalence in the harness; no native equivalence/performance claim is made.
Native damage energy is not simulated. Fixed-step offline flight time does
not charge wall-clock planning/logging time, so native timing remains unverified.

Full results and per-run failure distributions:
[diagnostic report](../tests/dogleg_diagnostics_report.json).
Source preservation: [audit](../tests/dogleg_diagnostic_preservation.json).
**Task 1 is complete. Tasks 2–4 have not started.**

Reproduce the report after generating all 20 seeds with the same benchmark
arguments as DOGLEG.md:

```powershell
python -B tests/summarize_dogleg_diagnostics.py --prefix tests/score_dogleg_diag_ --reference-prefix tests/score_dogleg_ --output tests/dogleg_diagnostics_report.json
```

The report gives the requested current-code baseline and no_safe_lane run
fraction. It reports each run's first failed-search slab separately; subsequent
failures are correlated observations. It does not pool event-wise percentiles
and claim they are a single debris percentile.

## Interpretation limits

A large lateral p90 is not the minimum safe bypass offset and does not prove
the absence of internal gaps. It describes centers at one time across a long
slab, not occupied cross-sections, future encounter times or inflated safety
volumes. Choosing to skirt the whole spread requires a separate time-aware
route test. Around-field and through-gap hypotheses remain to be tested.

The unchanged modeled fuel coefficient is `3388.7/(80*9.8) = 4.3223214` kg per m/s
of accumulated sequential body-axis impulse, or 5.7631 score points per m/s.
An ideal equal-speed bend has vector delta-v `2*v*sin(theta/2)`; body-axis L1
cost can be larger. The suggested offset-to-final-score examples require
time, gravity and overhead assumptions and are not verified native scores.

As a modeled illustration, a 350 m midpoint offset over 1,300 m at 8 m/s gives
an ideal 56.6-degree bend, 7.59 m/s of bend delta-v and 32.8 kg of bend fuel.
That fuel component alone costs about 44 score points. This is a conditional
example, not evidence that a safe route must use that offset.
