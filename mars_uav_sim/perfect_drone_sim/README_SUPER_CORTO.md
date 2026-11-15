# SUPER-CORTO simulator

`PerfectDrone` keeps the official MARSIM rendering, Odometry, TF, pose,
robot mesh and flown-path publishers. The configuration accepts
`dynamics_mode: perfect | so3`; a ROS parameter of the same name overrides it.

## Modes

- `perfect`: official SUPER `PositionCommand` tracking. Position and velocity
  are copied from the command. Attitude is reconstructed from acceleration and
  yaw using the original flatness routine (gravity 9.80 m/s²).
- `so3`: EGO-Planner rigid-body and first-order rotor dynamics. It starts at
  the configured airborne pose with equilibrium rotor RPM. The CORTO CTBR
  command passes through the body-rate controller and four-rotor allocation.

Source provenance, physical parameters and the exact two portability edits are
in `third_party/ego/UPSTREAM.md`. EGO's obstacle collision response is absent
from the upstream model; benchmark collision detection runs separately.

## Interfaces

| Topic | Type | Meaning |
| --- | --- | --- |
| `/corto/control` | `geometry_msgs/msg/WrenchStamped` | Official CORTO encoding: `wrench.force.xyz` are desired body rates in rad/s; `wrench.torque.x` is normalized collective thrust. 0.6 yields 9.81 m/s² before motor lag. |
| `/planning/pos_cmd` | `mars_quadrotor_msgs/msg/PositionCommand` | Original position/velocity/acceleration/yaw command, used in `perfect`. |
| `/lidar_slam/odom` | `nav_msgs/msg/Odometry` | Actual simulated position, orientation, world velocity, and body angular velocity. |
| `/simulator/motor_rpm` | `std_msgs/msg/Float64MultiArray` | Four actual rotor RPM values in upstream rotor order, in `so3`. |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time. |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Original MARSIM LiDAR points in `world`. |
| `/global_pc` | `sensor_msgs/msg/PointCloud2` | Original simulation PCD. |
| `/tf` | `tf2_msgs/msg/TFMessage` | `world -> perfect_drone`. |
| `/robot`, `/path` | Marker, Path | Original RViz outputs. |

The simulator advances with a fixed 0.001 s step on a dedicated wall-clock
thread. Control subscriptions also run on that thread. Odometry and clock
publish at 100 Hz. The main ROS executor performs MARSIM rendering.
Rendering, NMPC solving and planning do not stop the physics thread.
LiDAR timestamps identify the captured pose before rendering.

Both modes publish one common simulation clock. Run one simulator per
`ROS_DOMAIN_ID`; ROS clients use `use_sim_time:=true`. Planner timing uses
a separate monotonic wall clock.

## Run

From the repository root after building:

```bash
source scripts/env.sh
source install/setup.bash
ROS_DOMAIN_ID=185 xvfb-run -a vglrun -d egl ros2 run perfect_drone_sim perfect_drone_node \
  --ros-args -p config_name:=/absolute/path/to/simulator.yaml \
  -p dynamics_mode:=so3 -p use_sim_time:=true
```

`config_name` accepts an absolute path or a file in the original config
directory. `pcd_name` in the YAML accepts an absolute PCD path or an original
map filename. Set `init_position`, `init_yaw`, and the original LiDAR fields
in the same YAML.

## Verification

`ctest --test-dir build/perfect_drone_sim --output-on-failure` verifies
hover thrust units, motor-lag response, rotor allocation signs, angular-rate
tracking and rotation normalization.

`test/simulator_ros_smoke.py` starts the actual node with VirtualGL's
EGL GPU backend, sends commands,
and verifies state response, clock progression, rendered LiDAR, TF, robot mesh
and path messages. It writes result JSON, complete sampled Odometry JSONL and
the simulator log, including clean process exit. Keep Xvfb outside the node's
process group:

```bash
ROS_DOMAIN_ID=186 xvfb-run -a python3 \
  mars_uav_sim/perfect_drone_sim/test/simulator_ros_smoke.py \
  --mode so3 --config dense.yaml \
  --simulator "$PWD/install/perfect_drone_sim/lib/perfect_drone_sim/perfect_drone_node" \
  --output results/simulator_smoke
```

