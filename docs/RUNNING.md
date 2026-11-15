# 编译、运行与复现实验

工作目录：`/home/tong/tongworkspace/projects/aerial/SUPER-CORTO`。

## 依赖

目标环境为 Ubuntu 24.04、ROS 2 Jazzy、GCC 13、Eigen 3、PCL 1.14、Boost、OpenCV、yaml-cpp。
优化器使用 MOSEK 11.2.2 的 C++ Fusion API 和 NLopt 2.7.1。
MOSEK 需要有效许可证；算法没有替代求解器。
本机 SDK 默认路径为 `/home/tong/.local/opt/mosek-11.2.2/mosek/11.2`。
其他安装位置通过 `MOSEK_ROOT` 指定。
许可证通过 `MOSEKLM_LICENSE_FILE` 指定，默认 `$HOME/mosek/mosek.lic`。

无窗口 MARSIM 使用 Xvfb 和 VirtualGL 的 EGL 后端。
`vglrun -d egl` 在本机使用 NVIDIA RTX 4090；不需要桌面 X11 会话。
启动文件对仿真进程设置该前缀，外层只需运行一个 `xvfb-run -a`。

## 编译和必要检查

```bash
cd /home/tong/tongworkspace/projects/aerial/SUPER-CORTO
bash scripts/setup_deps.sh
source scripts/env.sh
bash scripts/build.sh
ctest --test-dir build/corto_ros2 --output-on-failure
ctest --test-dir build/perfect_drone_sim --output-on-failure
```

`setup_deps.sh` 将 NLopt、GLFW 解包到本仓库 `.deps/`。
其余 ROS 与系统依赖使用本机 Jazzy 安装。
`build.sh` 只构建 CORTO、ROG-Map、MARSIM、PerfectDrone 和消息包。
SUPER 原规划器与任务节点使用 `COLCON_IGNORE` 排除。

数值回归分别编译固定提交的原版算法和 ROS 2 移植算法。
原版测试入口仅提供参数读取接口，数值函数直接来自 `vendor/corto`。
回归覆盖 Topo-PRM、样条、SOCP、PTF SFC、NMPC、RK4 与解析 Jacobian。
真实 ROG-Map 的加载、坐标、膨胀、未知与边界语义另有契约检查。

## 交互启动

```bash
source scripts/env.sh
ROS_DOMAIN_ID=185 xvfb-run -a ros2 launch \
  "$SUPER_CORTO_ROOT/launch/table3.launch.py" \
  mode:=so3 scene:=medium planner_seed:=0 \
  output_dir:="$SUPER_CORTO_ROOT/results/manual_so3" rviz:=false
```

`mode:=nominal` 使用 CORTO 原版 RK4 执行条件。
它向 PerfectDrone 的 `perfect` 模式发送位置命令，供 MARSIM 和可视化使用。
参考状态发布到 `/corto/nominal_odom`。

`mode:=so3` 使用 PerfectDrone 的 `so3` 模式。
CORTO 发布 CTBR，角速度内环和旋翼分配驱动 EGO 刚体、电机模型。
NMPC 读取 `/lidar_slam/odom` 的实际状态。
独立物理线程持续推进 1 ms 时间步，规划与渲染不会暂停物理时间。

在有显示器的终端移除 `xvfb-run -a` 并设置 `rviz:=true`，可查看地图、LiDAR、参考路径、连续走廊、NMPC 预测和执行轨迹。

上述命令用于交互观察，按 Ctrl+C 停止。
若需按正式规则终止并保存单回合数据，使用记录器入口：

```bash
source scripts/env.sh
xvfb-run -a python3 scripts/benchmark_run.py \
  --results results/single_reproduce --run-id so3_medium_00 \
  --launch-command "ros2 launch $SUPER_CORTO_ROOT/launch/table3.launch.py mode:={mode} scene:={scene} planner_seed:={planner_seed} output_dir:={output_dir} rviz:=false"
```

## 完整 60 回合

固定场景和种子已保存到 `experiments/manifest.json`。
需重建完全相同的地图时运行 `python3 scripts/benchmark_generate.py`。
正式清单包括每个条件的 Sparse、Medium、Dense 各 10 回合。

```bash
source scripts/env.sh
xvfb-run -a python3 scripts/benchmark_run.py \
  --results results/table3_reproduce \
  --launch-command "ros2 launch $SUPER_CORTO_ROOT/launch/table3.launch.py mode:={mode} scene:={scene} planner_seed:={planner_seed} output_dir:={output_dir} rviz:=false"
python3 scripts/benchmark_report.py --results results/table3_reproduce
```

`--mode nominal` 或 `--mode so3` 只运行一个条件。
`--run-id so3_medium_00` 只运行预先声明的一回合。
`--resume` 保留所有已完成回合，包括失败回合，并继续缺少的回合。
已有原始数据不会覆盖。

每回合保留原始状态、控制、求解事件、规划事件、进程日志、配置和评测结果。
结果目录中的 `trials.csv` 为全部逐回合指标。
`table3.csv`、`table3.md` 为论文对照表。
指标定义、重建条件与统计分母见 `experiments/PROTOCOL.md`。

## 已有结果与离线复算

正式数据使用提交 `ef8e8e472dbbac40f4b91581582d848a83697234`。
全部 60 个 run 记录的源码修改状态均为 false。
当前版本随后只修复了一个停止阶段的 ROS 休眠异常。
该修复没有改变规划、控制、动力学或评分逻辑。
正式数据保留在 `results/table3/`，已有失败回合不会被后续检查替换。

```bash
python3 scripts/benchmark_report.py --results results/table3
python3 -B results/table3/nominal_diagnostics.py
python3 -B results/table3/analyze_runtime.py
python3 -B results/table3/plot_diagnostics.py
```

绘图脚本使用 Matplotlib；本机验证版本为 3.6.3。
回归向量、所有原始数据、汇总表和图均保存在仓库中。
