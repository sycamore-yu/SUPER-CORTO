# SUPER-CORTO

在 ROS 2 Jazzy 中运行原版 CORTO 规划与 NMPC。
NMPC 的 CTBR 动作直接进入原版 10 状态 RK4 模型，每步 0.05 s。
SUPER 提供 ROG-Map、MARSIM LiDAR、TF、RViz 和轨迹显示。

```text
CORTO 规划 → NMPC → 原版 RK4 → /corto/odom
                      ↑             ├→ 下一次 NMPC
                      │             └→ PerfectDrone → LiDAR / TF / RViz
                  原版状态反馈
```

PerfectDrone 直接读取完整位姿和速度，不重新计算姿态。
仿真时间由 CORTO 模型步推进，求解时间使用墙钟。

## 编译与运行

```bash
cd /home/tong/tongworkspace/projects/aerial/SUPER-CORTO
bash scripts/setup_deps.sh
source scripts/env.sh
bash scripts/build.sh

# 有桌面时直接启动；无桌面时在命令前加 xvfb-run -a。
ros2 launch "$PWD/launch/table3.launch.py" scene:=medium rviz:=true

# 固定场景、固定种子的全部 30 回合。
xvfb-run -a python3 scripts/benchmark_run.py --results results/table3_reproduce
```

MOSEK 需要有效许可证。依赖配置和单回合命令见 [RUNNING.md](docs/RUNNING.md)。

## 评测

Sparse、Medium、Dense 各运行 10 回合。
碰撞与越界只记录事件，继续采集到到达、原版任务结束或超时。
到达率与无碰撞且未越界的成功率分别统计。
几何评测使用 20 Hz 原始状态的中心点；地图、起终点及种子保持固定。

当前协议见 [PROTOCOL.md](experiments/PROTOCOL.md)，验收结果见 [RESULTS.md](docs/RESULTS.md)。
原版数值行为与 ROS 接口见 [PORT_NOTES.md](corto_ros2/PORT_NOTES.md)。
旧实验保存在 `results/archive/pre_direct_284b143/`。
