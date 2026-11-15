# Formal trial data audit

- Trials: 60/60; all six groups contain ten predeclared seeds.
- Frozen implementation: `ef8e8e472dbbac40f4b91581582d848a83697234`.
- Manifest SHA-256: `755f64c65f9688425630bd45e90d75e952116c970ec273983255dc70b8d63803`.
- Data integrity and statistical reconstruction: PASS.
- Clean process shutdown: FAIL; 20 trials contain shutdown log anomalies.
- Trials with process errors before runner SIGINT: 0.

Metadata, JSON/PCD hashes, required raw files, record counts, finite values, terminal events, planning counts and means were checked.
Planning means were recomputed from raw events with both reception-order and terminal-simulation-time cutoffs. Failed calls remain included.
Grouped means and the fixed denominator of ten were checked against report.json. No raw data or summaries were changed.

| Mode | Scene | Trials | Successes | Terminal reasons | Statistics audit |
|---|---|---:|---:|---|---|
| nominal | sparse | 10 | 0 | {"collision": 8, "out_of_bounds": 2} | PASS |
| nominal | medium | 10 | 0 | {"collision": 10} | PASS |
| nominal | dense | 10 | 0 | {"out_of_bounds": 4, "collision": 6} | PASS |
| so3 | sparse | 10 | 0 | {"out_of_bounds": 10} | PASS |
| so3 | medium | 10 | 0 | {"out_of_bounds": 10} | PASS |
| so3 | dense | 10 | 0 | {"out_of_bounds": 10} | PASS |

## Shutdown anomalies

The JSON audit records the exact launch-log lines and child exit codes for each anomaly. Errors after runner SIGINT are distinguished from errors during recorded flight.
Known shutdown message: `context cannot be slept with because it's invalid`; affected CORTO processes abort with exit code -6 after collection completes.

## Issues affecting the data or statistics

None found in the checks above.
