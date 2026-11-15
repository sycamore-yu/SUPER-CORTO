# 编译、验证与运行

## 环境

使用 Ubuntu 24.04、ROS 2 Jazzy、MOSEK 11.2.2、NLopt 2.7.1。
`scripts/env.sh` 设置当前仓库、ROS、MOSEK 和本地依赖路径。
MOSEK 路径可通过 `MOSEK_ROOT` 指定，许可证由 `MOSEKLM_LICENSE_FILE` 指定。
MARSIM 在本机使用 `vglrun -d egl`。无桌面时外层加 `xvfb-run -a`。

```bash
cd /home/tong/tongworkspace/projects/aerial/SUPER-CORTO
bash scripts/setup_deps.sh
source scripts/env.sh
bash scripts/build.sh
```

## 验证

```bash
source scripts/env.sh
ctest --test-dir build/corto_ros2 --output-on-failure
python3 -m unittest discover -s experiments -p test_protocol.py -v
python3 experiments/test_recorder_ros.py
```

状态转发与 LiDAR 检查使用某次启动生成的 `sim_config.yaml`：

```bash
ROS_DOMAIN_ID=193 xvfb-run -a python3 \
  mars_uav_sim/perfect_drone_sim/test/simulator_ros_smoke.py \
  --simulator "$PWD/install/perfect_drone_sim/lib/perfect_drone_sim/perfect_drone_node" \
  --config "$PWD/results/table3/direct_sparse_00/sim_config.yaml" \
  --output results/renderer_recheck
```

## 交互观察

```bash
ros2 launch "$PWD/launch/table3.launch.py" \
  scene:=medium planner_seed:=0 output_dir:="$PWD/results/manual" rviz:=true
```

按 Ctrl+C 结束交互观察。
`scene` 可选 sparse、medium、dense。

## 固定回合评测

```bash
# 全部 30 回合，另建结果目录。
xvfb-run -a python3 scripts/benchmark_run.py --results results/table3_reproduce

# 仅运行预定清单中的一个回合。
xvfb-run -a python3 scripts/benchmark_run.py \
  --results results/single_reproduce --run-id direct_medium_00

# 恢复未完成回合，已有失败结果也保留。
xvfb-run -a python3 scripts/benchmark_run.py \
  --results results/table3_reproduce --resume

# 从同一版本原始数据重算统计。
python3 scripts/benchmark_report.py --results results/table3_reproduce

# 核对 30 回合原始状态与 CTBR，逐步重放 CORTO RK4。
python3 scripts/benchmark_audit.py --results results/table3_reproduce
```

默认评测 ROS_DOMAIN_ID=185；并行诊断使用不同 `--ros-domain-id`。
完整评测规则见 `experiments/PROTOCOL.md`。
