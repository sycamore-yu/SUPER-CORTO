# SUPER 与 CORTO · 相同地图任务对照

两种算法使用相同 PCD、起终点、目标半径 0.2 m 和最长 60 s 任务。
评分均采用 20 Hz 位置中心点到实体障碍的距离，以及未内缩的地图边界。
SUPER 的原生 100 Hz 里程计插值为 20 Hz；保留实际终止状态。

| 场景 | CORTO 安全成功 | SUPER 安全成功 | CORTO 到达 | SUPER 到达 | CORTO 碰撞 / 越界 | SUPER 碰撞 / 越界 |
|---|---:|---:|---:|---:|---:|---:|
| sparse | 7/10 | 5/10 | 10/10 | 5/10 | 1 / 3 | 0 / 0 |
| medium | 4/10 | 6/10 | 10/10 | 6/10 | 2 / 6 | 0 / 0 |
| dense | 5/10 | 0/10 | 10/10 | 0/10 | 1 / 4 | 0 / 0 |

| 场景 | CORTO 安全成功平均飞行时间 | SUPER 安全成功平均飞行时间 |
|---|---:|---:|
| sparse | 6.13 s | 6.15 s |
| medium | 5.89 s | 8.12 s |
| dense | 9.40 s | — |

## 实验解释

- CORTO：NMPC → CTBR → 原版 10 状态 RK4，20 Hz 状态更新。
- SUPER：原版拓扑搜索 / CIRI / 轨迹优化 → PositionCommand → PerfectDrone 理想位置跟踪。
- SUPER 内部 100 Hz 里程计、10 Hz LiDAR、ROG-Map ROS 回调仍保留；其控制模型不与 CORTO 等价。
- SUPER 运行的是现有 ROS 2 编译产物，其二进制 SHA256 记录在 provenance.json；不是直接执行官方 ROS 1 构建。
- SUPER 不提供与 CORTO planner_seed 相同的采样种子接口；每组 10 次是独立重复运行，不能作配对统计。
- SUPER 仍保留自身 0.2 m 机器人半径、7 m 局部规划范围和 CIRI；统一的是地图、任务和最终中心点评分。
- 两者的规划时间计量范围不同，不能用原始均值直接声称某算法更快。

SUPER 已完成 30/30 回合。

完整原始数据及每回合日志见 SUPER 对照结果目录。
