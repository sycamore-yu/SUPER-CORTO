# EGO-Planner SO(3) dynamics

Upstream: https://github.com/ZJU-FAST-Lab/ego-planner
Commit: `bfda51284c8c1b476043255a8145ef925a3778a5`

Vendored files:

- `src/uav_simulator/so3_quadrotor_simulator/include/quadrotor_simulator/Quadrotor.h`
- `src/uav_simulator/so3_quadrotor_simulator/src/dynamics/Quadrotor.cpp`
- Repository `LICENSE` (GPL-3.0).

Only the unused ROS 1 include was removed from Quadrotor.cpp, and the bundled
Boost odeint include was replaced by the system Boost include. The force,
moment, drag, motor lag, limits, integration call, and rotation normalization
are unchanged.

Parameters are the upstream defaults: mass 0.98 kg, gravity 9.81 m/s²,
inertia diag(0.00264, 0.00264, 0.00496) kg·m², arm length 0.26 m,
propeller radius 0.062 m, thrust coefficient 8.98132e-9 N/rpm²,
moment coefficient 0.07*(3*0.062)*8.98132e-9 N·m/rpm², motor time constant
1/30 s, RPM range 1200–35000. Quadratic translational drag is retained.

The original simulator's allocation is reused by `so3_dynamics.hpp`.
CORTO sends normalized thrust and body rates instead of attitude/force.
The body-rate controller uses EGO's base Hummingbird angular gains
(0.07, 0.07, 0.02) N·m/(rad/s), plus gyroscopic compensation.
It starts in airborne hover with equilibrium motor RPM.

EGO contains no obstacle collision response or collision mesh. Its only contact
handling clamps negative altitude while descending. Collision evaluation must
use the separately documented benchmark geometry and must not infer a
physical body radius from this dynamics code.

