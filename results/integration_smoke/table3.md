# Table III reproduction

Recorded trials: 2/60. Missing: 58.

Primary collision model: EGO rotor enclosing sphere, radius 0.322 m. Goal radius: 0.2 m.
Flight time, speed and clearance average the successful trials; counts and all failed trajectories remain in trials.csv.
Clearance uses original solid geometry without planner inflation. Paper clearance geometry/aggregation is unspecified.

| Mode | Scene | Trials | Success | Flight s | Speed m/s | Body clearance m | Center clearance m | Online replan ms | All plan ms |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Paper | sparse | 10 | 100% | 5.15 | 4.47 | 0.11 (paper definition unspecified) | — | 43.1 | — |
| Paper | medium | 10 | 100% | 5.23 | 4.42 | 0.07 (paper definition unspecified) | — | 57.9 | — |
| Paper | dense | 10 | 100% | 4.24 | 4.73 | 0.15 (paper definition unspecified) | — | 59.1 | — |
| nominal | sparse | 1/10 | pending | — | — | — | — | 0.66 | 0.77 |
| nominal | medium | 0/10 | pending | — | — | — | — | — | — |
| nominal | dense | 0/10 | pending | — | — | — | — | — | — |
| so3 | sparse | 1/10 | pending | — | — | — | — | 97.25 | 99.56 |
| so3 | medium | 0/10 | pending | — | — | — | — | — | — |
| so3 | dense | 0/10 | pending | — | — | — | — | — | — |

## Collision-model sensitivity

| Mode | Scene | Center point | Original 0.1 m sphere | EGO 0.322 m envelope |
|---|---|---:|---:|---:|
| nominal | sparse | pending | pending | pending |
| nominal | medium | pending | pending | pending |
| nominal | dense | pending | pending | pending |
| so3 | sparse | pending | pending | pending |
| so3 | medium | pending | pending | pending |
| so3 | dense | pending | pending | pending |

## Reached-goal and observation metrics

Arrival below includes trajectories that collided before reaching the goal. These are failures in the primary table.

| Mode | Scene | Reached / declared | Reached flight s | Reached speed m/s | All-run observed flight s | All-run recorded sim s |
|---|---|---:|---:|---:|---:|---:|
| nominal | sparse | 0/10 | — | — | 60.00 | 60.25 |
| nominal | medium | 0/10 | — | — | — | — |
| nominal | dense | 0/10 | — | — | — | — |
| so3 | sparse | 1/10 | 42.12 | 1.95 | 42.18 | 47.33 |
| so3 | medium | 0/10 | — | — | — | — |
| so3 | dense | 0/10 | — | — | — | — |

## Missing trials

nominal_sparse_01, nominal_sparse_02, nominal_sparse_03, nominal_sparse_04, nominal_sparse_05, nominal_sparse_06, nominal_sparse_07, nominal_sparse_08, nominal_sparse_09, nominal_medium_00, nominal_medium_01, nominal_medium_02, nominal_medium_03, nominal_medium_04, nominal_medium_05, nominal_medium_06, nominal_medium_07, nominal_medium_08, nominal_medium_09, nominal_dense_00, nominal_dense_01, nominal_dense_02, nominal_dense_03, nominal_dense_04, nominal_dense_05, nominal_dense_06, nominal_dense_07, nominal_dense_08, nominal_dense_09, so3_sparse_01, so3_sparse_02, so3_sparse_03, so3_sparse_04, so3_sparse_05, so3_sparse_06, so3_sparse_07, so3_sparse_08, so3_sparse_09, so3_medium_00, so3_medium_01, so3_medium_02, so3_medium_03, so3_medium_04, so3_medium_05, so3_medium_06, so3_medium_07, so3_medium_08, so3_medium_09, so3_dense_00, so3_dense_01, so3_dense_02, so3_dense_03, so3_dense_04, so3_dense_05, so3_dense_06, so3_dense_07, so3_dense_08, so3_dense_09
