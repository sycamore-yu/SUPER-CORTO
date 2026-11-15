# Official source provenance and migration boundary

SUPER-CORTO is an independent Git repository cloned from official SUPER. Its local branch is `main`; `super-upstream` records the official source URL. No existing SUPER or CORTO working-tree changes were copied.

| Component | Official revision | Use |
| --- | --- | --- |
| [SUPER](https://github.com/hku-mars/SUPER) | `2ad3419c127a617c6d7df6925e81a14175a9c096` | ROS 2 ROG-Map, PerfectDrone, MARSIM, messages and visualization |
| [CORTO-Planner](https://github.com/yzpd/CORTO-Planner) | `e05a2e60f78d9ae4425981c1343913c3c67d3bb3` | Exact source snapshot in `vendor/corto`; ROS 2 port in `corto_ros2` |

The CORTO snapshot was extracted with `git archive` from the pinned official commit object. Its working-tree modifications in other projects are irrelevant to this snapshot. `vendor/corto/COLCON_IGNORE` prevents accidental ROS 1 package discovery. The upstream SUPER planner and mission planner are also excluded from colcon; no CIRI, LOONG, or MPCC planner runs in this experiment.

## Coordinate and occupancy contract

- All ROS poses, PCD points and physical states use the `world` frame, metres, and ENU axes.
- The application domain is `[-15,15) × [-7.5,7.5) × [0,3)` metres. CORTO's local grid coordinate is `world - [-15,-7.5,0]`.
- Queries outside this half-open domain are occupied. This explicit check is needed because ROG-Map's inflated-map query returns free outside allocated storage.
- Static PCD loading uses the official ROG-Map insertion and inflation. Map sliding and live occupancy updates are disabled for Table III. LiDAR rendering remains enabled.
- Unknown voxels inside the complete static scene are treated as free, matching CORTO's original static occupancy convention.
- ROG-Map uses `ORIGIN_AT_CORNER`, floor indexing and cell-centre queries. Its internally padded allocation does not change the evaluation boundary.
- Original CORTO uses a `-0.01 m` vertical grid origin. The port makes the scene origin explicit at `z=0`; numerical fixtures state their coordinate convention. Original launch start/goal are retained as world positions `[-14,-6.5,0.49]` and `[6.8,5.2,0.49]`.

## Source behaviours preserved

The source's L-BFGS corridor acceptance permits a projected-obstacle violation fraction of 25%. The control loop shifts the optimized input sequence before executing row zero, so the executed command is originally the second optimized row. The asynchronous SOCP call passes `snap_v` to the original speed-boundary argument. These behaviours remain unchanged and are logged as source characteristics, not repaired by this migration.

The original ROS control message is `WrenchStamped`: `force.xyz` are body rates in rad/s; `torque.x` is normalized collective thrust. `0.6` corresponds to `9.81 m/s²`. The same CTBR action directly enters the original 10-state RK4 model.

The CORTO node owns state propagation and simulation time. PerfectDrone receives `/corto/odom` and renders the exact state without reintegration or flatness conversion. There is one execution path.

## Paper and reconstruction

[Corridor-Driven Topological Planning With Nonlinear MPC for Agile Quadrotor Flight](https://doi.org/10.1109/LRA.2026.3700428), Table III, provides the target metrics in the task. The official repository only includes a Medium JSON with 100 obstacles (20 rings, 80 cylinders), while the requested paper condition has 90. The experiment therefore records reconstructed 45/90/180-obstacle scenes and predeclared seeds, not an author-identical map claim.

The experiment manifest specifies reconstructed scenes, seeds, endpoints and scoring. Collisions are sampled center-point events; they do not apply contact forces or stop the simulation. Arrival and collision-free, in-bounds success are reported separately.

The author-uploaded [paper text](https://www.researchgate.net/publication/406027365_Corridor-Driven_Topological_Planning_With_Nonlinear_MPC_for_Agile_Quadrotor_Flight), Sections V-A and V-C/D, specifies the 20 Hz controller, 10 prediction steps, 0.05 s step, 100 time-allocation samples, and defines Online T as online replanning computation time. Our report therefore keeps replanning wall time separate from NMPC wall time. The publication does not fully specify the failure-call aggregation policy; this experiment includes every completed replanning call before the terminal event, including failed calls.
