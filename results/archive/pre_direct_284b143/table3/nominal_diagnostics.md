# nominal 正式回合离线诊断

已分析 30 个固定回合，共核对 **835 个完整状态转移**。
状态分量最大绝对残差为 **1.776357e-15**。
源码提交：ef8e8e472dbbac40f4b91581582d848a83697234。

## 分场景汇总

| 场景 | 碰撞首次终止 | 越界首次终止 | 成功 | 核对转移数 | 最大残差 | NMPC 正返回 | NMPC 负返回 | 有负返回回合 | 交接拒绝 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| sparse | 8 | 2 | 0/10 | 389 | 1.776e-15 | 393 | 1 | 1 | 24 |
| medium | 10 | 0 | 0/10 | 205 | 8.882e-16 | 205 | 0 | 0 | 1 |
| dense | 6 | 4 | 0/10 | 241 | 8.882e-16 | 242 | 3 | 2 | 4 |

## 核对口径

状态顺序为 [位置 3 维，速度 3 维，qw，qx，qy，qz]。控制顺序为 [ωx，ωy，ωz，归一推力]。
每段直接使用记录的起始状态与控制，计算原版 RK4 的 0.05 s 转移，再与下一条状态比较。未重跑仿真或优化器。
只核对终点不晚于首次 terminal_time_s 的完整区间。跨越首次终止的最后一个区间被排除。
主事件统计同时满足两个条件：在首次 collector 终止事件前接收，且自身 sim_time_s 不晚于 terminal_time_s。
迟到但时间戳在终止前的消息仅保留在 late_preterminal_events 审计字段，不纳入 NMPC、规划及交接主计数。
逐回合核对 NMPC、规划、重规划的调用数和失败数，共 6 项；与既有 summary 全部一致的回合为 30/30。
首次终止原因及成功标记直接读取原始事件与既有 summary。碰撞采用既定机体包络判据，未重新判定接触。
状态残差判据为 1e-10。详细分量残差、时间戳检查及输入 SHA256 保存在 nominal_diagnostics.json。

## 两类证据

执行模型一致性检验回答：记录的 CTBR 是否按原版状态方程生成下一步状态。成功率检验回答：飞行是否在触发碰撞或越界前到达目标。
两类证据分别成立。较小的 RK4 残差与飞行失败可以同时出现。
负返回次数和交接拒绝次数是时序统计。原始记录缺少逐步参考轨迹及走廊系数，本诊断未据此确定单一失效原因。

## 每回合明细

| 回合 | 首次终止 | 时间 s | 转移数 | 最大残差 | NMPC 正/负 | 接纳/拒绝交接 |
|---|---|---:|---:|---:|---:|---:|
| nominal_sparse_00 | collision | 1.728571 | 34 | 2.220e-16 | 35/0 | 1/2 |
| nominal_sparse_01 | collision | 1.728571 | 34 | 2.220e-16 | 34/0 | 1/2 |
| nominal_sparse_02 | collision | 1.524000 | 30 | 8.882e-16 | 30/0 | 1/1 |
| nominal_sparse_03 | out_of_bounds | 1.969708 | 39 | 4.441e-16 | 39/0 | 1/1 |
| nominal_sparse_04 | collision | 1.410345 | 28 | 1.776e-15 | 29/0 | 0/3 |
| nominal_sparse_05 | collision | 1.433333 | 28 | 1.110e-16 | 28/1 | 0/3 |
| nominal_sparse_06 | collision | 2.489000 | 49 | 8.882e-16 | 50/0 | 1/3 |
| nominal_sparse_07 | collision | 2.624324 | 52 | 8.882e-16 | 52/0 | 2/3 |
| nominal_sparse_08 | collision | 2.334247 | 46 | 8.882e-16 | 46/0 | 1/3 |
| nominal_sparse_09 | out_of_bounds | 2.480277 | 49 | 8.882e-16 | 50/0 | 1/3 |
| nominal_medium_00 | collision | 1.118852 | 22 | 8.882e-16 | 22/0 | 0/1 |
| nominal_medium_01 | collision | 1.118852 | 22 | 8.882e-16 | 22/0 | 0/0 |
| nominal_medium_02 | collision | 1.048507 | 20 | 1.110e-16 | 20/0 | 0/0 |
| nominal_medium_03 | collision | 1.040909 | 20 | 4.441e-16 | 20/0 | 0/0 |
| nominal_medium_04 | collision | 1.037879 | 20 | 4.441e-16 | 20/0 | 0/0 |
| nominal_medium_05 | collision | 1.041538 | 20 | 2.220e-16 | 20/0 | 0/0 |
| nominal_medium_06 | collision | 1.040152 | 20 | 2.220e-16 | 20/0 | 0/0 |
| nominal_medium_07 | collision | 1.036719 | 20 | 8.882e-16 | 20/0 | 0/0 |
| nominal_medium_08 | collision | 1.067187 | 21 | 8.882e-16 | 21/0 | 0/0 |
| nominal_medium_09 | collision | 1.032308 | 20 | 4.441e-16 | 20/0 | 0/0 |
| nominal_dense_00 | out_of_bounds | 1.485295 | 29 | 4.441e-16 | 29/0 | 1/0 |
| nominal_dense_01 | out_of_bounds | 1.485295 | 29 | 4.441e-16 | 30/0 | 1/0 |
| nominal_dense_02 | out_of_bounds | 1.820008 | 36 | 4.441e-16 | 34/2 | 0/3 |
| nominal_dense_03 | collision | 1.070290 | 21 | 4.441e-16 | 21/0 | 0/0 |
| nominal_dense_04 | collision | 1.089552 | 21 | 8.882e-16 | 20/1 | 0/0 |
| nominal_dense_05 | collision | 0.737143 | 14 | 2.220e-16 | 15/0 | 0/0 |
| nominal_dense_06 | out_of_bounds | 1.313065 | 26 | 2.220e-16 | 27/0 | 0/1 |
| nominal_dense_07 | collision | 1.112121 | 22 | 4.441e-16 | 22/0 | 0/0 |
| nominal_dense_08 | collision | 1.085507 | 21 | 4.441e-16 | 21/0 | 0/0 |
| nominal_dense_09 | collision | 1.123684 | 22 | 8.882e-16 | 23/0 | 0/0 |

## 迟到事件审计

下表仅记录终止后接收、时间戳在终止前的事件。主表已排除这些事件。

| 回合 | 原始事件行号 | 事件 | 模型时间 s | 求解返回值 |
|---|---:|---|---:|---:|
| nominal_sparse_01 | 44 | nmpc_result | 1.700000 | 3 |
| nominal_sparse_02 | 38 | nmpc_result | 1.500000 | 3 |
| nominal_sparse_03 | 47 | nmpc_result | 1.950000 | 3 |
| nominal_sparse_07 | 66 | nmpc_result | 2.600000 | 3 |
| nominal_sparse_08 | 58 | nmpc_result | 2.300000 | 3 |
| nominal_medium_00 | 28 | nmpc_result | 1.100000 | 3 |
| nominal_medium_01 | 27 | nmpc_result | 1.100000 | 3 |
| nominal_medium_02 | 24 | nmpc_result | 1.000000 | 3 |
| nominal_medium_03 | 24 | nmpc_result | 1.000000 | -1 |
| nominal_medium_04 | 24 | nmpc_result | 1.000000 | 3 |
| nominal_medium_05 | 24 | nmpc_result | 1.000000 | 3 |
| nominal_medium_06 | 24 | nmpc_result | 1.000000 | 3 |
| nominal_medium_07 | 25 | nmpc_result | 1.000000 | 3 |
| nominal_medium_08 | 25 | nmpc_result | 1.050000 | 3 |
| nominal_medium_09 | 24 | nmpc_result | 1.000000 | 3 |
| nominal_dense_00 | 36 | nmpc_result | 1.450000 | 3 |
| nominal_dense_02 | 49 | nmpc_result | 1.800000 | 3 |
| nominal_dense_03 | 26 | nmpc_result | 1.050000 | 3 |
| nominal_dense_04 | 25 | nmpc_result | 1.050000 | 3 |
| nominal_dense_07 | 27 | nmpc_result | 1.100000 | 3 |
| nominal_dense_08 | 29 | nmpc_result | 1.050000 | 3 |

复算命令：

```bash
python3 results/table3/nominal_diagnostics.py
```

该脚本只更新本目录的 nominal_diagnostics.json 和 nominal_diagnostics.md。
