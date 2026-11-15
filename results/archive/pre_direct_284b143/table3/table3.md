# Table III reproduction

Recorded trials: 60/60. Missing: 0.

Primary collision model: EGO rotor enclosing sphere, radius 0.322 m. Goal radius: 0.2 m.
Flight time, speed and clearance average the successful trials; counts and all failed trajectories remain in trials.csv.
Clearance uses original solid geometry without planner inflation. Paper clearance geometry/aggregation is unspecified.

| Mode | Scene | Trials | Success | Flight s | Speed m/s | Body clearance m | Center clearance m | Online replan ms | All plan ms |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Paper | sparse | 10 | 100% | 5.15 | 4.47 | 0.11 (paper definition unspecified) | — | 43.1 | — |
| Paper | medium | 10 | 100% | 5.23 | 4.42 | 0.07 (paper definition unspecified) | — | 57.9 | — |
| Paper | dense | 10 | 100% | 4.24 | 4.73 | 0.15 (paper definition unspecified) | — | 59.1 | — |
| nominal | sparse | 10/10 | 0% | — | — | — | — | 122.40 | 122.81 |
| nominal | medium | 10/10 | 0% | — | — | — | — | 145.19 | 138.36 |
| nominal | dense | 10/10 | 0% | — | — | — | — | 125.52 | 123.13 |
| so3 | sparse | 10/10 | 0% | — | — | — | — | — | 122.80 |
| so3 | medium | 10/10 | 0% | — | — | — | — | — | 145.12 |
| so3 | dense | 10/10 | 0% | — | — | — | — | — | 121.20 |

## Clearance sensitivity on terminated trajectories

These are clearances of the same truncated trajectories, not counterfactual success rates for smaller vehicles.

| Mode | Scene | Center clearance m | 0.1 m sphere clearance m | 0.322 m envelope clearance m |
|---|---|---:|---:|---:|
| nominal | sparse | 0.434 | 0.334 | 0.112 |
| nominal | medium | 0.272 | 0.172 | -0.050 |
| nominal | dense | 0.361 | 0.261 | 0.039 |
| so3 | sparse | 4.854 | 4.754 | 4.532 |
| so3 | medium | 3.291 | 3.191 | 2.969 |
| so3 | dense | 1.264 | 1.164 | 0.942 |

## Reached-goal and observation metrics

Formal trials stop at the first terminal condition. Observation spans exclude the post-terminal audit tail for flight metrics.

| Mode | Scene | Reached / declared | Reached flight s | Reached speed m/s | All-run observed flight s | All-run recorded sim s |
|---|---|---:|---:|---:|---:|---:|
| nominal | sparse | 0/10 | — | — | 1.99 | 2.27 |
| nominal | medium | 0/10 | — | — | 1.07 | 1.39 |
| nominal | dense | 0/10 | — | — | 1.26 | 1.53 |
| so3 | sparse | 0/10 | — | — | 0.43 | 5.48 |
| so3 | medium | 0/10 | — | — | 0.45 | 5.78 |
| so3 | dense | 0/10 | — | — | 0.46 | 6.01 |

## Missing trials

None.
