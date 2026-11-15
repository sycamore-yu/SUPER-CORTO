# SUPER 与 CORTO · 同地图闭环对照

2026-11-15：使用 CORTO 原实验的 45/90/180 障碍 PCD、起点
`[-14,-6.5,0.49]`、终点 `[6.8,5.2,0.49]`、目标半径 0.2 m。
单回合最多运行 60 s，按 20 Hz 中心点与未内缩的地图边界评分。

| 场景 | CORTO 原修复版 | CORTO 轮廓代价修复版 | SUPER 安全到达 | SUPER 到达 |
|---|---:|---:|---:|---:|
| Sparse | 3/10 | **7/10** | 5/10 | 5/10 |
| Medium | 4/10 | **4/10** | 6/10 | 6/10 |
| Dense | 1/10 | **5/10** | 0/10 | 0/10 |

上表 CORTO 两列均 10/10 到达。
新增的 NMPC 轮廓代价实验保存于 `results/table3_centerline/`，
其源码为 `1e69749`。原修复版保留在
`results/table3_repaired/`。

SUPER 30 回合的中心点碰撞和越界均为 0；
未到达的 19 回合在 60 s 时仍停留在目标外。
其进程日志包含大量 `PlanFromRest failed` 和
`CIRI maxVolInsEllipsoid failed`。
原版 SUPER 的任务终点失败主要发生在 CIRI/轨迹前端。

## 比较边界

SUPER 使用现有 `/home/tong/tongworkspace/projects/aerial/SUPER`
仓库的 ROS 2 编译产物，保留其 CIRI、轨迹优化、
`PositionCommand` 与 PerfectDrone 理想位置跟踪。
其二进制 SHA256 已记录在
`results/super_same_scenes/formal30/provenance.json`。
这不是未修改的官方 ROS 1 版本；ROS 2 源码分支存在本地改动。

CORTO 则执行 NMPC → 4D CTBR → 原版 10 状态 RK4。
SUPER 100 Hz 原始里程计插值为 20 Hz 后，按同一中心点规则评分。
相同的是地图、起终点、任务时限和评分，而非执行动力学。
SUPER 使用自身 CIRI、机器人半径 0.2 m、ROG-Map 在线回调、
7 m 局部规划距离；CORTO 使用 PTF 椭圆走廊与其他局部规划参数。
SUPER 也没有 CORTO `planner_seed` 的对应固定采样种子接口，
所以 10 次独立重复运行不能作逐种子配对比较。
两套规划耗时的计时范围不同，此处不直接比较耗时。

## 文件

- 原始数据：`results/super_same_scenes/formal30/direct_<scene>_<seed>/`
- 逐场景对照：`results/super_same_scenes/formal30/comparison.md`
- SUPER 二进制与 PCD 来源：`results/super_same_scenes/formal30/provenance.json`
- 运行脚本：`scripts/benchmark_super_same_scenes.py`
- 评分脚本：`scripts/compare_super_corto.py`

本地复现先加载 ROS 2 环境，再执行：

```bash
source scripts/env.sh
source /home/tong/tongworkspace/projects/aerial/SUPER/scripts/ros2_env.sh
python3 scripts/benchmark_super_same_scenes.py --results results/super_reproduce
python3 scripts/compare_super_corto.py --super results/super_reproduce
```

对照不改变 CORTO 项目的控制器，也不修改 SUPER 规划算法源码。
