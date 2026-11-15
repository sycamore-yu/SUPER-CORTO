# CORTO direct · Table III 复现验收

正式实验源码提交：`1796c381772cd22df1e24e99a7d73343afa66acd`。
原版 CORTO：`e05a2e60f78d9ae4425981c1343913c3c67d3bb3`。
实验日期：2026-11-15。

## 执行链与验收

唯一控制链为 CORTO Topo-PRM / SOCP / PTF 走廊 → 原版 NMPC →
原版 10 状态 RK4 → 同一状态反馈 / Odometry / PerfectDrone 可视化。
已从活动代码删除 EGO 完整动力学、SO(3) 控制器、角速度内环、
旋翼分配、电机模型及模式开关。PerfectDrone 不积分第二套状态。

已取消 0.322 m 球包络、边界内缩、扫掠碰撞强制终止。
记录器在发生碰撞或越界后继续飞行，仅在到达、超时或原版任务结束时停止。
采样中心点碰撞、越界仍计入安全失败。

测试：6 项协议测试、ROS 消息端到端测试、2 项 CTest 全部通过。
上游数值回归通过，固定输入和移植模块输出一致。
PerfectDrone 的 61 个状态样本和 61 个 TF 一致，状态误差 0，
LiDAR 44 帧均非空（`results/validation_direct/renderer/renderer.json`）。

## 正式 30 回合结果

固定场景与规划种子 0–9，Sparse / Medium / Dense 各 10 回合。
每个回合保存原始 Odometry、CTBR、求解事件和进程日志。

| 场景 | 到达率 | 安全成功率 | 碰撞回合 | 越界回合 | 成功飞行时间 | 成功平均速度 | 成功最小净空 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Sparse | 8/10 | 0/10 | 7 | 8 | — | — | — |
| Medium | 10/10 | 2/10 | 6 | 7 | 7.75 s | 3.35 m/s | 0.057 m |
| Dense | 10/10 | 0/10 | 10 | 10 | — | — | — |

安全成功要求到达并且此前未发生中心点碰撞或越界。
两个安全成功回合都在 Medium；其余组没有成功样本，因此飞行指标不计算均值。
论文 Table III 三场景报告的成功率均为 100%。
本实验使用重建的 45/90/180 障碍场景；作者发布的 Medium JSON 有 100 个障碍，
因此该实验不是作者原始场景的逐回合复刻。

## 求解与失败诊断

| 场景 | 在线重规划均值（全部完成调用） | 成功重规划均值 | 成功/失败重规划次数 | NMPC 平均耗时 | NMPC 失败次数 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Sparse | 2.95 ms | 106.84 ms | 73 / 2633 | 58.78 ms | 748 |
| Medium | 11.77 ms | 121.85 ms | 74 / 700 | 50.42 ms | 89 |
| Dense | 2.16 ms | 142.75 ms | 56 / 3788 | 66.13 ms | 1075 |

全部调用均值包含 Topo-PRM 立即失败的调用，不能当作成功规划的计算时延。
本次成功重规划均值均高于论文的在线规划时间 43.1 / 57.9 / 59.1 ms。
Dense 10 回合中的首次 Topo-PRM 失败均早于首次越界，
说明规划失败不能全部解释为飞行越界后的连带现象。
NMPC 求解平均耗时接近或超过 50 ms 的模型控制周期。

此次验收的主要未解决问题是安全成功率低和在线拓扑重规划失败频繁。
保持 CORTO 数值算法不变，没有修改原版 NMPC 约束或 Topo-PRM 搜索逻辑以提高结果。

## 原始数据完整性

`results/table3/execution_audit.json` 记录：30/30 回合、10,826 个状态样本、
10,804 条 CTBR 记录、10,796 次状态转移，最大 RK4 回放误差为 0。
全部状态转移输入、源码版本、场景 SHA 校验和进程退出检查通过。
此次新记录器已修复跨话题末条控制消息丢失问题。

实验两次运行使用相同场景和规划种子，但部分回合到达结果不同。
原版异步重规划使用实际求解时长与交接时序；固定随机种子不能固定这些运行时条件。
首次运行保存在本地 `results/validation_direct/first_pass_incomplete/`，
不纳入本次正式结果。

## 复现

```bash
source scripts/env.sh
ctest --test-dir build/corto_ros2 --output-on-failure
python3 -m unittest discover -s experiments -p test_protocol.py -v
python3 experiments/test_recorder_ros.py
xvfb-run -a python3 scripts/benchmark_run.py --results results/table3_reproduce
python3 scripts/benchmark_audit.py --results results/table3_reproduce
```

完整统计：`results/table3/table3.md`、`trials.csv`、`report.json`。
每个回合的原始记录位于 `results/table3/direct_<scene>_<seed>/`。
