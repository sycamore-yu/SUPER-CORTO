# Implementation review

Review baseline: official SUPER `2ad3419c127a617c6d7df6925e81a14175a9c096`.
Specification: the requested CORTO Table III migration and acceptance criteria.
The review inspected the source diff and newly added implementation files.

## Standards

Two reproducibility findings were addressed before formal experiments:

1. NLopt and GLFW downloads now pin the exact validated Ubuntu package versions. `DEPENDENCIES.json` records their SHA-256 checksums, MOSEK version, compiler, system libraries, ROS version and hardware.
2. Formal experiments use a committed implementation. Every trial records the code commit, tracked-source modification state, map checksum and manifest checksum.

The inherited CORTO implementation contains concurrent `vector<bool>` writes and a pending-plan flag ordering that deserve a separate upstream algorithm review. This port preserves those source behaviours as requested. New simulation state and command access use a common mutex; the control executor and physical state publisher are independent of rendering.

## Spec

Three functional findings were fixed:

1. An added `mission_end` after initial planning failure could stop the benchmark before the original retry. It was removed. Failed planning calls remain recorded; the source returns to `WAIT_TARGET` and retries within the declared trial timeout.
2. The nominal control timestamp initially used the simulator clock while reference states used cumulative RK4 time. Nominal controls now use the same model time as nominal state and events. SO(3) controls use the physical simulation clock.
3. A stopped ROS context could make the temporary executor in `rclcpp::spin_some(node)` throw during shutdown. The node now reuses one executor and joins its planning thread on scope exit. Only an `RCLError` observed after context shutdown is accepted as normal shutdown; runtime errors remain visible. The numerical and control algorithms are unchanged.

The corresponding ROS node was rebuilt after these fixes. Map semantics and numerical-regression evidence are retained in `results/validation/` and `results/numerical_regression/`. Development integration logs, including the observed shutdown failures, are retained separately from formal experiments.
