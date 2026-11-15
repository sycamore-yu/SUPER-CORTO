# Table III · CORTO direct

回合：1/30。

碰撞与越界只记录。评测使用原始 20 Hz 状态采样点，机体中心不加包络，地图边界不内缩。
三张重建地图、起终点和种子沿用前次实验。
成功要求到达目标且此前未发生采样点碰撞或越界。飞行指标按成功回合平均。

| 条件 | 场景 | 成功率 % | 飞行时间 s | 平均速度 m/s | 净空 m | 在线重规划 ms |
|---|---|---:|---:|---:|---:|---:|
| 论文 | sparse | 100.00 | 5.15 | 4.47 | 0.11 | 43.10 |
| 论文 | medium | 100.00 | 5.23 | 4.42 | 0.07 | 57.90 |
| 论文 | dense | 100.00 | 4.24 | 4.73 | 0.15 | 59.10 |
| CORTO direct | sparse | — | — | — | — | 137.16 |
| CORTO direct | medium | — | — | — | — | — |
| CORTO direct | dense | — | — | — | — | — |

## 完整任务与失败记录

下表包含途中碰撞或越界后到达的回合。它们计入到达率，仍计为安全失败。

| 场景 | 到达/总回合 | 无碰撞且未越界成功 | 碰撞回合 | 越界回合 | 到达用时 s | 到达平均速度 m/s | 到达轨迹净空 m |
|---|---:|---:|---:|---:|---:|---:|---:|
| sparse | 1/10 | 0 | 1 | 0 | 6.35 | 3.79 | -0.03 |
| medium | 0/10 | 0 | 0 | 0 | — | — | — |
| dense | 0/10 | 0 | 0 | 0 | — | — | — |

缺少回合：direct_sparse_01, direct_sparse_02, direct_sparse_03, direct_sparse_04, direct_sparse_05, direct_sparse_06, direct_sparse_07, direct_sparse_08, direct_sparse_09, direct_medium_00, direct_medium_01, direct_medium_02, direct_medium_03, direct_medium_04, direct_medium_05, direct_medium_06, direct_medium_07, direct_medium_08, direct_medium_09, direct_dense_00, direct_dense_01, direct_dense_02, direct_dense_03, direct_dense_04, direct_dense_05, direct_dense_06, direct_dense_07, direct_dense_08, direct_dense_09。

逐回合状态、控制、事件和失败结果见 trials.csv 及各回合目录。
