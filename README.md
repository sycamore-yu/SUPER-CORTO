# SUPER-CORTO

基于官方 SUPER 仿真组件，移植 CORTO-Planner 原版 C++ 算法。
目标环境为 ROS 2 Jazzy。

CORTO 保留 Topo-PRM、弧长三次样条、MOSEK SOCP、PTF 连续椭圆安全飞行走廊、NLopt SLSQP 非线性模型预测控制（NMPC），以及异步重规划和计划交接。
实际执行使用 EGO-Planner 的 SO(3) 刚体、电机滞后、阻力和旋翼限制。
地图、LiDAR、状态发布和可视化来自官方 SUPER。

已完成数值回归和 60 个正式回合。
固定输入的 1479 个数值与原版完全一致。
本次固定配置的飞行成功率为 0%，未复现论文 Table III 的性能。
结果、定义和差距分析见 [实验报告](docs/RESULTS.md)。

## 源码与接口

| 组件 | 固定提交 | 目录 |
| --- | --- | --- |
| SUPER | `2ad3419c127a617c6d7df6925e81a14175a9c096` | `rog_map/`、`mars_uav_sim/` |
| CORTO-Planner | `e05a2e60f78d9ae4425981c1343913c3c67d3bb3` | 原版 `vendor/corto/`，移植 `corto_ros2/` |
| EGO-Planner | `bfda51284c8c1b476043255a8145ef925a3778a5` | `mars_uav_sim/perfect_drone_sim/third_party/ego/` |

`PerfectDrone` 支持 `dynamics_mode: perfect | so3`。
`perfect` 保留原版位置命令执行。
`so3` 接收 `/corto/control`，通过角速度内环和旋翼分配驱动物理模型。
`/lidar_slam/odom` 向 CORTO 反馈实际状态。

实验模式 `nominal` 使用原版 RK4 执行状态，并通过 `perfect` 显示轨迹。
实验模式 `so3` 使用外部物理状态。
物理步长为 1 ms，NMPC 目标控制频率为 20 Hz。
状态反馈与仿真时钟的目标发布频率为 100 Hz。
实际控制间隔、时钟偏差和采样缺口保留在原始数据中。

## 编译

```bash
cd /home/tong/tongworkspace/projects/aerial/SUPER-CORTO
bash scripts/setup_deps.sh
source scripts/env.sh
bash scripts/build.sh
ctest --test-dir build/corto_ros2 --output-on-failure
ctest --test-dir build/perfect_drone_sim --output-on-failure
```

依赖 MOSEK 11.2.2 C++ SDK 和有效许可证。
NLopt 与 GLFW 的版本及下载包校验值见 [依赖记录](docs/DEPENDENCIES.json)。
详细运行命令、GPU 无窗口渲染、RViz 和完整实验入口见 [运行说明](docs/RUNNING.md)。

## 固定实验

场景尺寸为 30×15×3 m。
Sparse、Medium、Dense 分别有 45、90、180 个圆柱和圆环障碍。
作者公开 Medium 地图有 100 个障碍，因此三张实验图按已记录规则重建。
生成种子为 20261115，规划种子为 0–9。
两个执行条件各 30 回合，共 60 回合。

速度上限为 10 m/s，加速度上限为 5 m/s²。
NMPC 预测 10 步，每步 0.05 s；SOCP 使用 100 个节点。
成功率分母包含全部预定回合。
所有碰撞、越界、求解失败和规划失败均保留。

主碰撞模型为半径 0.322 m 的 EGO 旋翼保守包络球。
同时报告中心点和 0.1 m 球体的净空。
原版 EGO 不提供障碍接触求解或整机碰撞网格。
净空从未膨胀的原始障碍几何计算。

- [场景与种子清单](experiments/manifest.json)
- [评测协议](experiments/PROTOCOL.md)
- [固定输入数值回归](results/numerical_regression/regression.json)
- [Table III 对照结果](results/table3/table3.md)
- [全部回合指标](results/table3/trials.csv)
- [源码迁移边界](docs/UPSTREAM.md)
- [CORTO 原版行为记录](corto_ros2/PORT_NOTES.md)

## 来源与版权

复用源码保留各自许可证和版权声明。
官方项目：[SUPER](https://github.com/hku-mars/SUPER)、[CORTO-Planner](https://github.com/yzpd/CORTO-Planner)、[EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner)。
原 SUPER 项目说明与引用保存在 [SUPER_UPSTREAM_README.md](docs/SUPER_UPSTREAM_README.md)。
