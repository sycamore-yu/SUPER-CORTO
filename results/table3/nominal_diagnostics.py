#!/usr/bin/env python3
"""Audit frozen nominal records; write only this directory's diagnostic outputs."""
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


RESULTS = Path(__file__).resolve().parent
SCENES = ("sparse", "medium", "dense")
STATE_FIELDS = ("x", "y", "z", "vx", "vy", "vz", "qw", "qx", "qy", "qz")
CONTROL_FIELDS = ("body_rate_x", "body_rate_y", "body_rate_z", "normalized_thrust")
DT = 0.05
TIME_TOLERANCE = 1e-9
RESIDUAL_TOLERANCE = 1e-10


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def derivative(state, control):
    _, _, _, vx, vy, vz, qw, qx, qy, qz = state
    rx, ry, rz, normalized_thrust = control
    thrust = normalized_thrust * 9.81 / 0.6
    return np.array([
        vx, vy, vz,
        2 * (qx * qz + qw * qy) * thrust,
        2 * (qy * qz - qw * qx) * thrust,
        (1 - 2 * (qx * qx + qy * qy)) * thrust - 9.81,
        0.5 * (-rx * qx - ry * qy - rz * qz),
        0.5 * (rx * qw + rz * qy - ry * qz),
        0.5 * (ry * qw - rz * qx + rx * qz),
        0.5 * (rz * qw + ry * qx - rx * qy),
    ])


def rk4(state, control):
    k1 = derivative(state, control)
    k2 = derivative(state + DT * k1 / 2, control)
    k3 = derivative(state + DT * k2 / 2, control)
    k4 = derivative(state + DT * k3, control)
    return state + DT * (k1 / 6 + k2 / 3 + k3 / 3 + k4 / 6)


def first_event(events):
    return min(events, key=lambda event: event["sim_time_s"]) if events else None


def analyze_run(scene, seed):
    run_id = f"nominal_{scene}_{seed:02d}"
    folder = RESULTS / run_id
    summary = json.loads((folder / "summary.json").read_text())
    metadata = json.loads((folder / "run.json").read_text())
    events = [json.loads(line) for line in (folder / "events.jsonl").read_text().splitlines() if line]
    start = next(event for event in events if event["event"] == "mission_start")
    terminal_index = next(index for index, event in enumerate(events)
                          if event["event"] == "mission_end" and event.get("origin") == "collector")
    terminal = events[terminal_index]
    start_time = float(start["sim_time_s"])
    terminal_time = float(terminal["terminal_time_s"])
    if abs(float(summary["terminal_time_s"]) - terminal_time) > TIME_TOLERANCE:
        raise ValueError(f"Terminal event and summary disagree: {run_id}")
    trajectory = read_csv(folder / "trajectory.csv")
    controls = read_csv(folder / "control.csv")
    control_by_time = {}
    for row in controls:
        key = round(float(row["sim_time_s"]) * 1e9)
        control_by_time.setdefault(key, []).append(row)

    residuals, alignment_errors, step_errors, issues = [], [], [], []
    eligible_intervals = 0
    for index, (current, following) in enumerate(zip(trajectory, trajectory[1:])):
        time = float(current["sim_time_s"])
        next_time = float(following["sim_time_s"])
        if time < start_time - TIME_TOLERANCE or next_time > terminal_time + TIME_TOLERANCE:
            continue
        eligible_intervals += 1
        matching = control_by_time.get(round(time * 1e9), [])
        if len(matching) != 1 or abs(next_time - time - DT) > TIME_TOLERANCE:
            issues.append({"trajectory_interval_index": index, "start_s": time,
                           "end_s": next_time, "matching_controls": len(matching)})
            continue
        state = np.array([float(current[key]) for key in STATE_FIELDS])
        next_state = np.array([float(following[key]) for key in STATE_FIELDS])
        control = np.array([float(matching[0][key]) for key in CONTROL_FIELDS])
        if not all(np.isfinite(array).all() for array in (state, next_state, control)):
            issues.append({"trajectory_interval_index": index, "reason": "nonfinite_input"})
            continue
        residuals.append(np.abs(rk4(state, control) - next_state))
        alignment_errors.append(abs(float(matching[0]["sim_time_s"]) - time))
        step_errors.append(abs(next_time - time - DT))

    component_max = np.max(residuals, axis=0) if residuals else None
    before_terminal = [
        event for event in events[:terminal_index]
        if start_time - TIME_TOLERANCE <= float(event["sim_time_s"]) <= terminal_time + TIME_TOLERANCE
    ]
    late_preterminal = [
        dict(event, events_jsonl_line=index + 1)
        for index, event in enumerate(events)
        if index > terminal_index
        and start_time - TIME_TOLERANCE <= float(event["sim_time_s"]) <= terminal_time + TIME_TOLERANCE
    ]
    nmpc = [event for event in before_terminal if event["event"] == "nmpc_result"]
    handoffs = [event for event in before_terminal if event["event"] == "plan_handoff"]
    plans = [event for event in before_terminal if event["event"] == "plan_result"]
    negative = [event for event in nmpc if event["solver_code"] < 0]
    rejected = [event for event in handoffs if event["accepted"] is False]
    max_residual = float(np.max(component_max)) if component_max is not None else None
    replans = [event for event in plans if event.get("initial") is False]
    main_counts = {
        "nmpc_calls": len(nmpc),
        "nmpc_failures": sum(event["success"] is False for event in nmpc),
        "planning_calls": len(plans),
        "planning_failures": sum(event["success"] is False for event in plans),
        "replanning_calls": len(replans),
        "replanning_failures": sum(event["success"] is False for event in replans),
    }
    count_checks = {key: {"diagnostic": value, "summary": summary[key], "equal": value == summary[key]}
                    for key, value in main_counts.items()}
    return {
        "run_id": run_id, "scene": scene, "planner_seed": seed,
        "code_commit": metadata["code_commit"],
        "first_terminal_reason": terminal["reason"],
        "terminal_time_s": terminal_time,
        "terminal_observation_time_s": terminal["sim_time_s"],
        "first_collector_terminal_events_jsonl_line": terminal_index + 1,
        "success_in_existing_summary": summary["success"],
        "eligible_complete_intervals": eligible_intervals,
        "checked_complete_intervals": len(residuals),
        "max_absolute_state_residual": max_residual,
        "max_absolute_position_residual_m": float(np.max(component_max[:3])) if component_max is not None else None,
        "max_absolute_velocity_residual_mps": float(np.max(component_max[3:6])) if component_max is not None else None,
        "max_absolute_quaternion_residual": float(np.max(component_max[6:])) if component_max is not None else None,
        "component_max_residual": dict(zip(STATE_FIELDS, component_max.tolist())) if component_max is not None else {},
        "max_control_state_timestamp_difference_s": max(alignment_errors, default=None),
        "max_step_difference_s": max(step_errors, default=None),
        "transition_integrity_issues": issues,
        "consistent_with_recorded_rk4": bool(residuals) and not issues and max_residual <= RESIDUAL_TOLERANCE,
        "nmpc_calls_before_terminal": len(nmpc),
        "nmpc_positive_returns_before_terminal": sum(event["solver_code"] > 0 for event in nmpc),
        "nmpc_negative_returns_before_terminal": len(negative),
        "nmpc_zero_returns_before_terminal": sum(event["solver_code"] == 0 for event in nmpc),
        "nmpc_return_codes_before_terminal": dict(Counter(str(event["solver_code"]) for event in nmpc)),
        "first_negative_nmpc_before_terminal": first_event(negative),
        "plan_calls_before_terminal": len(plans),
        "plan_failures_before_terminal": sum(event["success"] is False for event in plans),
        "handoffs_accepted_before_terminal": sum(event["accepted"] is True for event in handoffs),
        "handoffs_rejected_before_terminal": len(rejected),
        "handoff_rejections": rejected,
        "late_preterminal_events": late_preterminal,
        "late_preterminal_event_counts": dict(Counter(event["event"] for event in late_preterminal)),
        "summary_count_checks": count_checks,
        "summary_counts_match": all(check["equal"] for check in count_checks.values()),
        "files_sha256": {name: hashlib.sha256((folder / name).read_bytes()).hexdigest()
                         for name in ("trajectory.csv", "control.csv", "events.jsonl", "summary.json", "run.json")},
    }


def main():
    expected = [RESULTS / f"nominal_{scene}_{seed:02d}" / "summary.json"
                for scene in SCENES for seed in range(10)]
    missing = [str(path) for path in expected if not path.exists()]
    if missing:
        raise FileNotFoundError("All 30 frozen nominal summaries are required: " + ", ".join(missing))
    runs = [analyze_run(scene, seed) for scene in SCENES for seed in range(10)]
    groups = []
    for scene in SCENES:
        selected = [run for run in runs if run["scene"] == scene]
        groups.append({
            "scene": scene, "runs": len(selected),
            "first_terminal_reasons": dict(Counter(run["first_terminal_reason"] for run in selected)),
            "successful_runs": sum(run["success_in_existing_summary"] for run in selected),
            "rk4_consistent_runs": sum(run["consistent_with_recorded_rk4"] for run in selected),
            "checked_complete_intervals": sum(run["checked_complete_intervals"] for run in selected),
            "max_absolute_state_residual": max(run["max_absolute_state_residual"] for run in selected),
            "nmpc_positive_returns_before_terminal": sum(run["nmpc_positive_returns_before_terminal"] for run in selected),
            "nmpc_negative_returns_before_terminal": sum(run["nmpc_negative_returns_before_terminal"] for run in selected),
            "runs_with_negative_nmpc_before_terminal": sum(run["nmpc_negative_returns_before_terminal"] > 0 for run in selected),
            "plan_failures_before_terminal": sum(run["plan_failures_before_terminal"] for run in selected),
            "handoffs_accepted_before_terminal": sum(run["handoffs_accepted_before_terminal"] for run in selected),
            "handoffs_rejected_before_terminal": sum(run["handoffs_rejected_before_terminal"] for run in selected),
            "summary_counts_matching_runs": sum(run["summary_counts_match"] for run in selected),
            "late_preterminal_event_counts": dict(Counter(event["event"] for run in selected
                                                          for event in run["late_preterminal_events"])),
        })
    output = {
        "analysis": "Offline audit of all 30 nominal trials, truncated at each first terminal event",
        "code_commits": sorted({run["code_commit"] for run in runs}),
        "method": {
            "state_order": STATE_FIELDS, "control_order": CONTROL_FIELDS, "nominal_dt_s": DT,
            "rk4_source": "vendor/corto/src/modules/nmpc/nominal_quad_dynamic.hpp",
            "interval_scope": "Only complete recorded intervals ending at or before terminal_time_s; the interval crossing termination is excluded.",
            "event_scope": "Main counts require both reception before the first collector mission_end and event sim_time_s at or before terminal_time_s.",
            "late_event_scope": "Events received after the collector mission_end but timestamped at or before terminal_time_s are retained only in late_preterminal_events audit fields; excluded from all main counts.",
            "terminal_definition": "Existing collector event and summary; no collision, bounds or success classification is recomputed.",
            "time_matching_tolerance_s": TIME_TOLERANCE,
            "absolute_state_residual_tolerance": RESIDUAL_TOLERANCE,
            "interpretation": "Execution-model consistency and collision-free goal-reaching success are separate evidence. This audit does not identify a single cause of collision or out-of-bounds failure.",
            "no_simulation_or_optimization_rerun": True,
            "raw_records_and_summaries_modified": False,
        },
        "total_checked_complete_intervals": sum(run["checked_complete_intervals"] for run in runs),
        "max_absolute_state_residual": max(run["max_absolute_state_residual"] for run in runs),
        "summary_counts_matching_runs": sum(run["summary_counts_match"] for run in runs),
        "late_preterminal_event_counts": dict(Counter(event["event"] for run in runs
                                                      for event in run["late_preterminal_events"])),
        "groups": groups, "runs": runs,
    }
    (RESULTS / "nominal_diagnostics.json").write_text(json.dumps(output, indent=2, ensure_ascii=False, allow_nan=False) + "\n")

    lines = [
        "# nominal 正式回合离线诊断", "",
        f"已分析 30 个固定回合，共核对 **{output['total_checked_complete_intervals']} 个完整状态转移**。",
        f"状态分量最大绝对残差为 **{output['max_absolute_state_residual']:.6e}**。",
        "源码提交：" + ", ".join(output["code_commits"]) + "。", "",
        "## 分场景汇总", "",
        "| 场景 | 碰撞首次终止 | 越界首次终止 | 成功 | 核对转移数 | 最大残差 | NMPC 正返回 | NMPC 负返回 | 有负返回回合 | 交接拒绝 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for group in groups:
        reasons = group["first_terminal_reasons"]
        lines.append(f"| {group['scene']} | {reasons.get('collision', 0)} | {reasons.get('out_of_bounds', 0)} | "
                     f"{group['successful_runs']}/10 | {group['checked_complete_intervals']} | {group['max_absolute_state_residual']:.3e} | "
                     f"{group['nmpc_positive_returns_before_terminal']} | {group['nmpc_negative_returns_before_terminal']} | "
                     f"{group['runs_with_negative_nmpc_before_terminal']} | {group['handoffs_rejected_before_terminal']} |")
    lines += [
        "", "## 核对口径", "",
        "状态顺序为 [位置 3 维，速度 3 维，qw，qx，qy，qz]。控制顺序为 [ωx，ωy，ωz，归一推力]。",
        "每段直接使用记录的起始状态与控制，计算原版 RK4 的 0.05 s 转移，再与下一条状态比较。未重跑仿真或优化器。",
        "只核对终点不晚于首次 terminal_time_s 的完整区间。跨越首次终止的最后一个区间被排除。",
        "主事件统计同时满足两个条件：在首次 collector 终止事件前接收，且自身 sim_time_s 不晚于 terminal_time_s。",
        "迟到但时间戳在终止前的消息仅保留在 late_preterminal_events 审计字段，不纳入 NMPC、规划及交接主计数。",
        f"逐回合核对 NMPC、规划、重规划的调用数和失败数，共 6 项；与既有 summary 全部一致的回合为 {output['summary_counts_matching_runs']}/30。",
        "首次终止原因及成功标记直接读取原始事件与既有 summary。碰撞采用既定机体包络判据，未重新判定接触。",
        f"状态残差判据为 {RESIDUAL_TOLERANCE:g}。详细分量残差、时间戳检查及输入 SHA256 保存在 nominal_diagnostics.json。",
        "", "## 两类证据", "",
        "执行模型一致性检验回答：记录的 CTBR 是否按原版状态方程生成下一步状态。成功率检验回答：飞行是否在触发碰撞或越界前到达目标。",
        "两类证据分别成立。较小的 RK4 残差与飞行失败可以同时出现。",
        "负返回次数和交接拒绝次数是时序统计。原始记录缺少逐步参考轨迹及走廊系数，本诊断未据此确定单一失效原因。",
        "", "## 每回合明细", "",
        "| 回合 | 首次终止 | 时间 s | 转移数 | 最大残差 | NMPC 正/负 | 接纳/拒绝交接 |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for run in runs:
        lines.append(f"| {run['run_id']} | {run['first_terminal_reason']} | {run['terminal_time_s']:.6f} | "
                     f"{run['checked_complete_intervals']} | {run['max_absolute_state_residual']:.3e} | "
                     f"{run['nmpc_positive_returns_before_terminal']}/{run['nmpc_negative_returns_before_terminal']} | "
                     f"{run['handoffs_accepted_before_terminal']}/{run['handoffs_rejected_before_terminal']} |")
    lines += ["", "## 迟到事件审计", "",
              "下表仅记录终止后接收、时间戳在终止前的事件。主表已排除这些事件。", "",
              "| 回合 | 原始事件行号 | 事件 | 模型时间 s | 求解返回值 |",
              "|---|---:|---|---:|---:|"]
    for run in runs:
        for event in run["late_preterminal_events"]:
            lines.append(f"| {run['run_id']} | {event['events_jsonl_line']} | {event['event']} | "
                         f"{event['sim_time_s']:.6f} | {event.get('solver_code', '—')} |")
    lines += ["", "复算命令：", "", "```bash", "python3 results/table3/nominal_diagnostics.py", "```", "",
              "该脚本只更新本目录的 nominal_diagnostics.json 和 nominal_diagnostics.md。"]
    (RESULTS / "nominal_diagnostics.md").write_text("\n".join(lines) + "\n")
    print(json.dumps({"trials": len(runs), "intervals": output["total_checked_complete_intervals"],
                      "max_residual": output["max_absolute_state_residual"],
                      "summary_counts_matching_runs": output["summary_counts_matching_runs"],
                      "late_preterminal_event_counts": output["late_preterminal_event_counts"],
                      "groups": groups}, ensure_ascii=False))


if __name__ == "__main__":
    main()
