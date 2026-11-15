# CORTO 状态与 MARSIM 渲染

本仓库的 ROS 2 PerfectDrone 是 CORTO 状态显示节点。
它订阅 `/corto/odom`，将完整消息转发至 `/lidar_slam/odom`。
位置、四元数、线速度、角速度和时间戳全部保持原值。
TF、Pose、机器人模型、轨迹和 LiDAR 均使用该状态。

`/corto/odom` 使用 reliable、transient_local、depth=100。
节点不订阅控制动作，不积分动力学，不发布时钟。
`/clock` 由 CORTO 随每个 0.05 s 模型步发布。
LiDAR 定时器按配置 `sensing_rate` 渲染最新完整状态。
全局 PCD 以 transient_local 发布，支持晚加入的 RViz。

## 输入与输出

| 话题 | 类型 | 用途 |
|---|---|---|
| `/corto/odom` | nav_msgs/Odometry | 唯一状态输入 |
| `/lidar_slam/odom` | nav_msgs/Odometry | 原样转发 |
| `/lidar_slam/pose` | geometry_msgs/PoseStamped | 位姿 |
| `/tf` | tf2_msgs/TFMessage | world → perfect_drone |
| `/robot`、`/path` | Marker、Path | 可视化 |
| `/cloud_registered` | sensor_msgs/PointCloud2 | 当前状态渲染的 LiDAR |
| `/global_pc` | sensor_msgs/PointCloud2 | 固定全局地图 |

`test/simulator_ros_smoke.py` 检查完整状态一致、TF、非空 LiDAR、
单一时钟，以及控制指令没有被该节点消费。
