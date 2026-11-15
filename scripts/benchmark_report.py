#!/usr/bin/env python3
"""Score all predeclared CORTO direct trials from the recorded model states."""
import argparse
import csv
import json
from pathlib import Path
import statistics

import numpy as np
from benchmark_geometry import SceneGeometry

ROOT = Path(__file__).resolve().parents[1]
PAPER = {"sparse": [100, 5.15, 4.47, 0.11, 43.1],
         "medium": [100, 5.23, 4.42, 0.07, 57.9],
         "dense": [100, 4.24, 4.73, 0.15, 59.1]}


def mean(values):
    values = [x for x in values if x is not None]
    return statistics.fmean(values) if values else None


def score_trial(run_dir, manifest, run, scene):
    events = [json.loads(x) for x in (run_dir / "events.jsonl").read_text().splitlines() if x.strip()]
    with (run_dir / "trajectory.csv").open() as stream:
        recorded = list(csv.DictReader(stream))
    terminal_index = next((i for i, x in enumerate(events)
                           if x.get("event") == "mission_end" and x.get("origin") == "collector"), len(events))
    terminal = events[terminal_index] if terminal_index < len(events) else None
    end = float(terminal["sim_time_s"]) if terminal else float("inf")
    measured = [x for x in events[:terminal_index] if float(x.get("sim_time_s", 0)) <= end + 1e-9]
    plans = [x for x in measured if x["event"] == "plan_result"]
    replans = [x for x in plans if not x["initial"]]
    solves = [x for x in measured if x["event"] == "nmpc_result"]
    queries = [x for x in measured if x["event"] == "topo_query"]
    handoffs = [x for x in measured if x["event"] == "plan_handoff"]
    successful_replans = [x for x in replans if x["success"]]
    infeasible_positive = [x for x in solves if x["success"] and
                           x.get("maximum_corridor_violation", 0) > 1e-3]
    predicted_outside_floor = [x for x in solves if x["success"] and
                               x.get("minimum_predicted_z_m", 0) < 0]
    result = dict(run, recorded_odom_samples=len(recorded), event_count=len(events),
                  termination_reason=terminal["reason"] if terminal else "missing_mission_end",
                  terminal_time_s=end if terminal else None,
                  reached_goal=False, success=False, collision=False, out_of_bounds=False,
                  flight_time_s=None, path_length_m=None, mean_speed_mps=None, clearance_m=None,
                  first_collision_time_s=None, first_collision_position=None,
                  first_out_of_bounds_time_s=None, first_out_of_bounds_position=None,
                  planning_calls=len(plans), planning_failures=sum(not x["success"] for x in plans),
                  replanning_calls=len(replans), replanning_failures=sum(not x["success"] for x in replans),
                  nmpc_calls=len(solves), nmpc_failures=sum(not x["success"] for x in solves),
                  nmpc_infeasible_positive_calls=len(infeasible_positive),
                  nmpc_predicted_floor_violations=len(predicted_outside_floor),
                  topo_start_outside=sum(x.get("reason") == "start_outside_map" for x in queries),
                  topo_start_occupied=sum(x.get("reason") == "start_occupied" for x in queries),
                  topo_goal_occupied=sum(x.get("reason") == "goal_occupied" for x in queries),
                  topo_no_connection=sum(x.get("reason") == "no_connection" for x in queries),
                  rejected_plan_handoffs=sum(not x["accepted"] for x in handoffs),
                  successful_replans=len(successful_replans),
                  successful_replanning_ms=mean([x["elapsed_ms"] for x in successful_replans]),
                  planning_all_ms=mean([x["elapsed_ms"] for x in plans]),
                  online_replanning_ms=mean([x["elapsed_ms"] for x in replans]),
                  nmpc_mean_ms=mean([x["elapsed_ms"] for x in solves]))
    start = next((float(x["sim_time_s"]) for x in events if x["event"] == "mission_start"), None)
    if start is None or not recorded:
        result["evaluation_reason"] = "missing_mission_start_or_odometry"
        return result
    samples = {float(x["sim_time_s"]): x for x in recorded
               if start - 1e-9 <= float(x["sim_time_s"]) <= end + 1e-9}
    times = np.array(sorted(samples))
    points = np.array([[float(samples[t][key]) for key in ("x", "y", "z")] for t in times])
    if not len(times) or not np.isfinite(points).all():
        result["evaluation_reason"] = "nonfinite_or_missing_flight_state"
        return result
    reached = np.flatnonzero(np.linalg.norm(points - np.asarray(scene["goal"]), axis=1) < manifest["goal_tolerance_m"])
    if len(reached):
        count = int(reached[0]) + 1
        points, times = points[:count], times[:count]
        result["reached_goal"] = True
    geometry = SceneGeometry(scene)
    clearance = geometry.clearance(points)
    collisions = np.flatnonzero(clearance <= 0)
    outside = np.flatnonzero(geometry.out_of_bounds(points))
    duration = float(times[-1] - start)
    distance = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
    result.update(flight_time_s=duration, path_length_m=distance,
                  mean_speed_mps=distance / duration if duration > 0 else None,
                  clearance_m=float(clearance.min()) if np.isfinite(clearance.min()) else None,
                  collision=bool(len(collisions)), out_of_bounds=bool(len(outside)),
                  evaluation_samples=len(times),
                  final_goal_distance_m=float(np.linalg.norm(points[-1] - scene["goal"])))
    for name, indices in (("collision", collisions), ("out_of_bounds", outside)):
        if len(indices):
            index = int(indices[0])
            result[f"first_{name}_time_s"] = float(times[index])
            result[f"first_{name}_position"] = points[index].tolist()
    result["success"] = result["reached_goal"] and not result["collision"] and not result["out_of_bounds"]
    reasons = [name for name in ("collision", "out_of_bounds") if result[name]]
    if not result["reached_goal"]:
        reasons.append(result["termination_reason"])
    result["evaluation_reason"] = "success" if result["success"] else "+".join(reasons)
    return result


def format_value(value, digits=2):
    return "—" if value is None else f"{value:.{digits}f}"


def report(experiments, results):
    manifest = json.loads((experiments / "manifest.json").read_text())
    trials, missing = [], []
    for run in manifest["runs"]:
        directory = results / run["run_id"]
        if not (directory / "run.json").exists():
            missing.append(run["run_id"])
            continue
        scene = json.loads((experiments / "scenes" / f"{run['scene']}.json").read_text())
        result = score_trial(directory, manifest, run, scene)
        (directory / "summary.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        trials.append(result)
    results.mkdir(parents=True, exist_ok=True)
    with (results / "trials.csv").open("w") as stream:
        keys = list(dict.fromkeys(k for result in trials for k in result))
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(trials)
    groups = []
    for scene in PAPER:
        observed = [x for x in trials if x["scene"] == scene]
        successes = [x for x in observed if x["success"]]
        reached = [x for x in observed if x["reached_goal"]]
        expected = sum(x["scene"] == scene for x in manifest["runs"])
        calls = sum(x["replanning_calls"] for x in observed)
        group = dict(scene=scene, completed=len(observed), expected=expected,
                     reached_goal_count=len(reached), successes=len(successes),
                     success_rate_percent=100 * len(successes) / expected if len(observed) == expected else None,
                     arrival_rate_percent=100 * len(reached) / expected if len(observed) == expected else None,
                     flight_time_s=mean([x["flight_time_s"] for x in successes]),
                     mean_speed_mps=mean([x["mean_speed_mps"] for x in successes]),
                     clearance_m=mean([x["clearance_m"] for x in successes]),
                     reached_flight_time_s=mean([x["flight_time_s"] for x in reached]),
                     reached_mean_speed_mps=mean([x["mean_speed_mps"] for x in reached]),
                     reached_clearance_m=mean([x["clearance_m"] for x in reached]),
                     collision_runs=sum(x["collision"] for x in observed),
                     out_of_bounds_runs=sum(x["out_of_bounds"] for x in observed),
                     planning_failures=sum(x["planning_failures"] for x in observed),
                     nmpc_failures=sum(x["nmpc_failures"] for x in observed),
                     nmpc_infeasible_positive_calls=sum(x["nmpc_infeasible_positive_calls"] for x in observed),
                     nmpc_predicted_floor_violations=sum(x["nmpc_predicted_floor_violations"] for x in observed),
                     topo_start_outside=sum(x["topo_start_outside"] for x in observed),
                     topo_start_occupied=sum(x["topo_start_occupied"] for x in observed),
                     topo_goal_occupied=sum(x["topo_goal_occupied"] for x in observed),
                     topo_no_connection=sum(x["topo_no_connection"] for x in observed),
                     rejected_plan_handoffs=sum(x["rejected_plan_handoffs"] for x in observed),
                     successful_replans=sum(x["successful_replans"] for x in observed),
                     successful_replanning_ms=(
                         sum(x["successful_replanning_ms"] * x["successful_replans"] for x in observed
                             if x["successful_replanning_ms"] is not None)
                         / sum(x["successful_replans"] for x in observed)
                         if sum(x["successful_replans"] for x in observed) else None),
                     replanning_calls=calls,
                     online_replanning_ms=sum(x["online_replanning_ms"] * x["replanning_calls"] for x in observed
                                              if x["online_replanning_ms"] is not None) / calls if calls else None)
        groups.append(group)
    with (results / "table3.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=groups[0])
        writer.writeheader()
        writer.writerows(groups)
    lines = ["# Table III · CORTO direct", "", f"回合：{len(trials)}/{len(manifest['runs'])}。",
             "", "碰撞与越界只记录。评测使用原始 20 Hz 状态采样点，机体中心不加包络，地图边界不内缩。",
             "三张重建地图、起终点和种子沿用前次实验。",
             "成功要求到达目标且此前未发生采样点碰撞或越界。飞行指标按成功回合平均。",
             "", "| 条件 | 场景 | 成功率 % | 飞行时间 s | 平均速度 m/s | 净空 m | 在线重规划 ms |",
             "|---|---|---:|---:|---:|---:|---:|"]
    for scene, values in PAPER.items():
        lines.append(f"| 论文 | {scene} | " + " | ".join(format_value(x) for x in values) + " |")
    for group in groups:
        lines.append(f"| CORTO direct | {group['scene']} | " + " | ".join(format_value(group[k]) for k in
                     ("success_rate_percent", "flight_time_s", "mean_speed_mps", "clearance_m", "online_replanning_ms")) + " |")
    lines += ["", "## 完整任务与失败记录", "",
              "下表包含途中碰撞或越界后到达的回合。它们计入到达率，仍计为安全失败。",
              "", "| 场景 | 到达/总回合 | 无碰撞且未越界成功 | 碰撞回合 | 越界回合 | 到达用时 s | 到达平均速度 m/s | 到达轨迹净空 m |",
              "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for g in groups:
        lines.append(f"| {g['scene']} | {g['reached_goal_count']}/{g['expected']} | {g['successes']} | {g['collision_runs']} | {g['out_of_bounds_runs']} | "
                     + " | ".join(format_value(g[k]) for k in ("reached_flight_time_s", "reached_mean_speed_mps", "reached_clearance_m")) + " |")
    lines += ["", "## 拓扑与 NMPC 诊断", "",
              "成功求解与约束可行分别统计；椭圆约束正违反量超过 0.001 时记录为非可行正返回。",
              "", "| 场景 | 地图外规划起点 | 被占据局部目标 | 无图连通 | 交接拒绝 | 成功重规划 ms | 正返回但走廊非可行 | 预测负高度 |",
              "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for g in groups:
        lines.append(f"| {g['scene']} | {g['topo_start_outside']} | {g['topo_goal_occupied']} | {g['topo_no_connection']} | "
                     f"{g['rejected_plan_handoffs']} | {format_value(g['successful_replanning_ms'])} | "
                     f"{g['nmpc_infeasible_positive_calls']} | {g['nmpc_predicted_floor_violations']} |")
    lines += ["", f"缺少回合：{', '.join(missing) if missing else '无'}。", "",
              "逐回合状态、控制、事件和失败结果见 trials.csv 及各回合目录。"]
    (results / "table3.md").write_text("\n".join(lines) + "\n")
    (results / "report.json").write_text(json.dumps({"groups": groups, "missing_runs": missing}, indent=2) + "\n")
    print(json.dumps({"recorded": len(trials), "missing": len(missing), "report": str(results / "table3.md")}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--experiments", type=Path, default=ROOT / "experiments")
    parser.add_argument("--results", type=Path, required=True)
    args = parser.parse_args()
    report(args.experiments, args.results)
