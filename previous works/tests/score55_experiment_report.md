> **Historical document — archived 21 November 2026.**
> This describes an earlier revision or experiment. For the delivered implementation, see
> [Final Submission Algorithm](../../Final%20Submission%20Algorithm.md). Archived commands and flags refer to their historical revision.
> Original location: `tests/score55_experiment_report.md`.

Finalized at the user's explicit request; no further tuning is pending. See final_native_validation.json for the accepted limitation.

Current selected Momentum source:ebcc04946087543faa17bd7caf3b8401c7227aa524041b1cff717fd95f36285b.
User relaxed target to47-50 native average over two runs and urgent completion.
Final validation scored43.5 and24.2, mean33.85, both zero damage. Startup source
hashes independently match the selected file for BOTH runs. Target NOT met.
The extra clearance adjustment scored28.8 and was rejected/rolled back.
Best earlier official native score remains47.2. No higher average is claimed.

# Score55 development evidence

Native target55 remains unachieved. These are official single-run results, not matched native seed cohorts.

| Native folder | Score | Entry s | Fuel kg | Scored damage J |
|---|---:|---:|---:|---:|
| 261119_101425_FINAL_SUBMISSION__faf | 22.1 | 270.36 | 54.24 | 0.0 |
| 261119_102346_FINAL_SUBMISSION__b60 | 36.0 | 266.42 | 44.3 | 0.0 |
| 261119_111837_FINAL_SUBMISSION__84e | 34.6 | 235.46 | 49.75 | 0.0 |
| 261119_114238_FINAL_SUBMISSION__641 | 4.1 | 225.1 | 74.37 | 0.0 |
| 261119_114648_FINAL_SUBMISSION__f8a | 16.5 | 243.2 | 62.12 | 0.0 |
| 261119_115246_FINAL_SUBMISSION__53b | -16.6 | 262.3 | 84.3 | 0.0 |
| 261119_115935_FINAL_SUBMISSION__4bf | 37.8 | 235.32 | 47.43 | 0.0 |
| 261119_120849_FINAL_SUBMISSION__7c1 | 37.3 | 248.3 | 45.81 | 0.0 |
| 261119_140256_FINAL_SUBMISSION__7b6 | 47.2 | 227.77 | 41.6 | 0.0 |
| 261119_141524_FINAL_SUBMISSION__c0e | 16.2 | 261.24 | 59.82 | 0.0 |
| 261119_143200_FINAL_SUBMISSION__d56 | 29.5 | 267.78 | 48.95 | 0.0 |
| 261119_145343_FINAL_SUBMISSION__e5a | 33.3 | 226.53 | 52.27 | 0.0 |
| 261119_152937_FINAL_SUBMISSION__ef1 | 20.0 | 247.44 | 58.91 | 0.0 |
| 261119_153631_FINAL_SUBMISSION__634 | 21.0 | 241.02 | 59.13 | 0.0 |
| 261119_154445_FINAL_SUBMISSION__99d | 42.2 | 250.54 | 41.77 | 0.0 |
| 261119_155556_FINAL_SUBMISSION__4cf | 42.1 | 206.74 | 49.45 | 0.0 |
| 261119_161020_FINAL_SUBMISSION__335 | 35.0 | 228.49 | 50.67 | 0.0 |

Powered/physical/kinetic native bc7 was aborted after154282.6 J damage; it has no official finish score. It is rejected.

| Offline cohort (20 seeds100-119) | Median score | p10-p90 | Entry median s | Fuel median kg | Proxy contacts |
|---|---:|---|---:|---:|---:|
| Original task baseline | 38.275 | 25.648-43.85 | 264.97 | 43.725 | 0 |
| Strong-axis/radar/normal-tail/fine | 45.15 | 40.485-48.691 | 220.33 | 43.475 | 0 |
| Powered/physical/kinetic (rejected natively) | 48.58 | 44.709-51.358 | 207.97 | 43.745 | 0 |

Powered profile vs strong-axis: paired median score+1.85 (bootstrap95% .455-6.8), time-6.79 s, fuel-.605 kg. Median paired fuel deltas by phase: launch-2.2204 kg, alignment-.5604, main+2.2881, local avoid-.80875. Phase medians do not sum to the median total. See saved JSON for every seed.

The native contact falsifies the inference that zero proxy contacts predicts native safety. No controller promotion from this cohort.

Departure-risk cohort completed20 seeds: median45.215, paired median change0, bootstrap95%[0,0],19/20 seeds identical to baseline. Rejected for score selection. Departure risk has three tests for confirmed recession, uncertain recession, and approaching blockage. Two lookahead path-choice tests pass; its pilot is also exactly baseline, so no benefit claim.

Fuel-phase labels classify commanded mode, not physical cause. Avoidance can be charged to IMPULSE_BURN. Official configuration/scoring/actuator/jet CSV remain unchanged.
