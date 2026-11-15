# CORTO ROS 2 移植契约

上游固定为 `yzpd/CORTO-Planner@e05a2e60f78d9ae4425981c1343913c3c67d3bb3`。原始源码位于 `../vendor/corto`。本包运行于 ROS 2 Jazzy，修复后的算法差异见 `docs/repair-pipeline.md`。

## 算法文件

| 阶段 | 文件 | 当前行为 |
| --- | --- | --- |
| Topo-PRM | `modules/path_searching/topo_prm.cpp` | 320 次采样、ROG-Map 膨胀体素射线、Guard/Connector 和安全路径缩短 |
| 弧长样条 | `modules/cubic_spline/` | 原版三次样条、重采样与导数 |
| SOCP | `modules/optimization/ref_traj_opt.cpp` | MOSEK Fusion、100 节点、速度平方变量、原加速度/jerk约束 |
| PTF SFC | `modules/corridor/` | 弧长投影、连续潜变量椭圆、L-BFGS 分阶段惩罚、体积计算 |
| NMPC | `modules/nmpc/` | SLSQP、10×0.05 s、RK4、原速度/走廊约束；修复版启用已声明的 PTF 横向轮廓成本 |
| 异步规划与交接 | `src/corto_node.cpp` | 双走廊池、耗时EMA、预测接续点、候选选择、交接距离门限 |

候选先保留预计时间小于最快候选加 0.2 s 的路径，再选走廊体积最大者。局部规划长度为 8 m；进度时间超过 1 s 或剩余时间小于 0.8 s 时触发重规划。交接距离门限为 0.5 m。以上均沿用上游代码。

## 原版数值语义

以下区分上游行为和修复版行为：

- SFC 使用 `NLP_LBFGS()`。最后一次优化返回成功或达到迭代上限，且原始投影点违反数量不超过整数除法 `n_all_raw / 4` 时，原版允许该候选进入选择。
- 上游先移动 U 再执行，实际执行第 2 个优化节点；修复版先执行 `U.row(0)`，再移动热启动序列。
- SOCP 的 w 变量表示速度平方；修复版将 `snap_v` 平方后用作入口值。
- 参考节点仍使用最近的离散累计时间节点；SFC 改为与 L-BFGS 完全一致的潜变量椭圆插值。
- NMPC 原版硬约束只有速度范数与横截面椭圆约束。5 m/s² 是 SOCP 的设置；未另加 NMPC 加速度硬约束。
- 后续安全实验将原版已有变量 `contouring_cost` 和 `cost_w(8)` 接入目标函数及梯度；该成本修复不改变原有约束与执行动力学，详见 `docs/RESULTS.md`。
- 状态顺序为 `[px,py,pz,vx,vy,vz,qw,qx,qy,qz]`。原版初始化尾部 `[0,0,0,1]` 对应偏航 π。本包保留该值。
- NMPC 的机体角速度范围为 ±1.8π rad/s。归一化推力范围为 [0,0.8]，悬停推力为 0.6。
- 原版 nominal RK4 不额外归一化四元数。本包数值基线保持该行为。

## 接口变化

NMPC 输出直接进入原版 RK4，每步推进 0.05 s。
`/corto/odom` 发布执行后的完整状态，`/clock` 发布同一模型时间。
下一次 NMPC 直接使用该状态。PerfectDrone 只转发和渲染。
状态不再经过 PositionCommand 和平坦性姿态重建。

`/corto/control` 类型为 `geometry_msgs/msg/WrenchStamped`。字段沿用原版编码：

| 字段 | 含义 |
| --- | --- |
| `wrench.force.x/y/z` | 机体角速度指令，rad/s |
| `wrench.torque.x` | 归一化总推力 |
| `wrench.torque.y/z` | 0 |
| `header.frame_id` | body |

归一化推力在原版动力学中换算为 `9.81 × thrust / 0.6`，单位 m/s²。
机体系角速度直接用于四元数导数。

start_pos_x/y/z 和 goal_pos_x/y/z 直接使用 world 坐标。默认起终点为 [-14,-6.5,0.49] 和 [6.8,5.2,0.49]。原 demo 的局部坐标偏移不再重复施加。地图坐标转换由 GridMap::origin() 明确提供。Topo-PRM 不再写死原版的 z=-0.01 偏移。

地图实际查询委托给 ROG-Map。SFC 点云从同一膨胀占据查询按网格中心采样生成，包含靠近地面和天花板的占据体素。MARSIM 输出仍由 PerfectDrone 提供。

`ros::NodeHandle` 参数读取直接替换为 rclcpp 参数读取。参数中的斜杠改为点，例如 `nmpc.maxVel`。Topo-PRM 新增 `planner_seed`：-1 保留随机设备种子，非负值固定随机输入。线程在销毁前 join，避免后台引用悬空。头文件 `Pre4NMPC_Robust` 增加 inline，解决重复定义；函数体未改。

## 事件与可视化

`/corto/events` 为 `std_msgs/msg/String`，每条为 JSON，含 event、sim_time_s、wall_time_s。

- mission_start：第一条控制下发前记录，先发布 t=0 初态。
- plan_result：每次规划恰好一条。含 initial、success、elapsed_ms、stage、plan_id。失败阶段也记录。
- plan_handoff：含 accepted、stitch_error_m、plan_id。
- nmpc_result：每次求解记录 solver_code、success、elapsed_ms。
- mission_end：含 reason；goal_reached 与 topology_failures 分开记录。初始规划失败只记录 plan_result，并保留原版重试流程。

sim_time_s 来自累计 RK4 时间。wall_time_s、规划和 NMPC elapsed_ms 使用 steady_clock。

状态发布到 `/corto/status`。轨迹、候选路径、拓扑图和走廊分别发布到 `/corto/predict_traj`、`/corto/filter_path`、`/corto/graph_nodes`、`/corto/corridor_markers`。实际机体、TF、LiDAR 与路径显示由 SUPER 保留。

## 数值回归

`corto_upstream_oracle` 直接编译 vendor 中未修改的算法源文件。`corto_ported_fixture` 编译本包数值源。二者使用相同 Eigen/PCL/NLopt/MOSEK。

oracle 的测试专用 ROS 头只提供原核心用到的参数读取。两个未被数值核心使用的 ROS 消息 include 使用空头。它们不实现或替换任何规划算法。Topo-PRM 测试输入为固定种子和固定占据列；生产 ROG-Map 查询由 map_contract 另测。

测试分别保存两套原始向量，逐元素比较：

- Topo-PRM 路径数量与全部路径点。
- 样条长度、100 个位置及一阶、二阶导数。
- SOCP 总时间及 100 个离散时间。
- PTF SFC 成功状态、体积与全部输出系数。
- NMPC 求解状态、40 个控制量和 100 个预测状态量。
- RK4 状态、状态/控制 Jacobian、加速度及其 Jacobian。

容差为 `1e-9 + 1e-8 × max_abs(reference)`。回归同时要求 SOCP 生成 100 个节点、SFC 成功和 NMPC 返回正状态码。不能仅凭两次相同失败判定通过。

在仓库根目录运行：

```bash
source scripts/env.sh
ctest --test-dir build/corto_ros2 --output-on-failure
```

原始向量、进程日志和 `regression.json` 写入 `results/numerical_regression/`。
