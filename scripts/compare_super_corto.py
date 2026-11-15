#!/usr/bin/env python3
"""Compare the shared-scene scores while retaining execution-model differences."""
import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--super", type=Path, required=True)
    parser.add_argument("--corto", type=Path, default=ROOT / "results/table3_centerline")
    args = parser.parse_args()
    super_root, corto_root = args.super.resolve(), args.corto.resolve()
    sr = json.loads((super_root / "report.json").read_text())
    cr = json.loads((corto_root / "report.json").read_text())
    cg = {g["scene"]: g for g in cr["groups"]}
    sg = {g["scene"]: g for g in sr["groups"]}
    scenes = ["sparse", "medium", "dense"]
    summary = {"super": sr, "corto": cr, "comparison": []}
    lines = ["# SUPER 与 CORTO · 相同地图任务对照", "",
             "两种算法使用相同 PCD、起终点、目标半径 0.2 m 和最长 60 s 任务。",
             "评分均采用 20 Hz 位置中心点到实体障碍的距离，以及未内缩的地图边界。",
             "SUPER 的原生 100 Hz 里程计插值为 20 Hz；保留实际终止状态。",
             "", "| 场景 | CORTO 安全成功 | SUPER 安全成功 | CORTO 到达 | SUPER 到达 | CORTO 碰撞 / 越界 | SUPER 碰撞 / 越界 |",
             "|---|---:|---:|---:|---:|---:|---:|"]
    for scene in scenes:
        c, s = cg[scene], sg[scene]
        summary["comparison"].append({"scene": scene, "corto": c, "super": s})
        lines.append(f"| {scene} | {c['successes']}/{c['expected']} | "
                     f"{s['safe_success']}/{s['completed']} | "
                     f"{c['reached_goal_count']}/{c['expected']} | {s['arrival']}/{s['completed']} | "
                     f"{c['collision_runs']} / {c['out_of_bounds_runs']} | "
                     f"{s['collision']} / {s['out_of_bounds']} |")
    lines += ["", "| 场景 | CORTO 安全成功平均飞行时间 | SUPER 安全成功平均飞行时间 |",
              "|---|---:|---:|"]
    for scene in scenes:
        c, s = cg[scene], sg[scene]
        ct = "—" if c["flight_time_s"] is None else f"{c['flight_time_s']:.2f} s"
        st = "—" if s["mean_safe_flight_s"] is None else f"{s['mean_safe_flight_s']:.2f} s"
        lines.append(f"| {scene} | {ct} | {st} |")
    lines += ["", "## 实验解释", "",
              "- CORTO：NMPC → CTBR → 原版 10 状态 RK4，20 Hz 状态更新。",
              "- SUPER：原版拓扑搜索 / CIRI / 轨迹优化 → PositionCommand → PerfectDrone 理想位置跟踪。",
              "- SUPER 内部 100 Hz 里程计、10 Hz LiDAR、ROG-Map ROS 回调仍保留；其控制模型不与 CORTO 等价。",
              "- SUPER 运行的是现有 ROS 2 编译产物，其二进制 SHA256 记录在 provenance.json；不是直接执行官方 ROS 1 构建。",
              "- SUPER 不提供与 CORTO planner_seed 相同的采样种子接口；每组 10 次是独立重复运行，不能作配对统计。",
              "- SUPER 仍保留自身 0.2 m 机器人半径、7 m 局部规划范围和 CIRI；统一的是地图、任务和最终中心点评分。",
              "- 两者的规划时间计量范围不同，不能用原始均值直接声称某算法更快。",
              "", f"SUPER 已完成 {sum(sg[x]['completed'] for x in scenes)}/30 回合。",
              "", "完整原始数据及每回合日志见 SUPER 对照结果目录。"]
    (super_root / "comparison.md").write_text("\n".join(lines) + "\n")
    (super_root / "comparison.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(super_root / "comparison.md")


if __name__ == "__main__":
    main()
