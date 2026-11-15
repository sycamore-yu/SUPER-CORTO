#!/usr/bin/env python3
"""Analyze frozen trial CSV/JSON data; never launch ROS or modify raw files.

Run after the suite:
    python3 results/table3/analyze_runtime.py
The default input/output directory is this script's parent directory.
"""
import argparse
import collections
import csv
import datetime
import hashlib
import json
import math
from pathlib import Path
import statistics


DEFINITIONS = {
    "terminal": "首个 origin=collector 的 mission_end；优先 terminal_time_s，否则 sim_time_s。",
    "event_filter": "仅保留终止事件之前的 JSONL 行，且事件 sim_time_s <= 首次终止时间。包含首次控制对应的启动求解。与正式评分的 NMPC 过滤一致。",
    "sample_filter": "保留 mission_start.sim_time_s <= sample.sim_time_s <= terminal_time_s，且 sample.wall_receive_s <= collector terminal.wall_receive_s。浮点时间端点容差 1e-9 s。",
    "audit_tail": "触发帧若在插值终止时刻之后，和随后 0.3 s 审计尾部一样，不进入运行周期统计。原始文件保持不变。",
    "intervals": "按原始接收行顺序，对过滤后的相邻样本分别计算 sim_time_s 和 wall_receive_s 差。没有排序、去重或连接不同回合。",
    "interval_roundoff": "周期阈值计数使用 1e-6 ms 浮点容差。NMPC 的 50 ms 超期仍按实测 elapsed_ms>50 严格计算。",
    "frequency": "N>=2 且首末跨度>0 时，频率=(N-1)/(最后时间-第一时间)。组频率=总间隔数/所有回合有效间隔时长之和。",
    "wall_cadence": "wall_receive_s 是同一记录器单调墙钟。收包周期近似发布周期，包含 DDS 与记录器调度，不能解释为执行器消费时刻。",
    "nominal_clock": "nominal 每个模型步人为增加 0.05 s。其 sim 频率不能证明墙钟达到 20 Hz；不拿 nominal 仿真/墙钟比解释真实物理速度。",
    "so3_clock": "SO3 固定物理 dt=0.001 s，独立墙钟线程；odom/clock 配置为 100 Hz。记录的状态时间与收包墙钟跨度可检查求解期间时钟是否继续推进。",
    "nmpc": "elapsed_ms 使用求解器外层 steady_clock 计时，包含负状态调用。超期定义严格为 elapsed_ms>50 ms。",
    "quantiles": "P50/P95/P99 用排序样本的线性插值分位数，位置=(N-1)*p。空集合返回 null。",
    "groups": "mode/scene 和 mode 组按全部终止前间隔及求解调用合并。统计不筛选成功种子；倾角与末次推力的组分布按回合计权。",
    "tilt": "倾角=acos((qw^2+qz^2-qx^2-qy^2)/||q||^2)，单位度。只为解释姿态而归一化，不改原始状态。",
    "boundary": "仅 collector reason=out_of_bounds 的回合报告首次越界前状态。其他回合不推算中止后的反事实越界。",
    "boundary_face": "方向由 terminal_position、launch_inputs 的 map_origin/map_size 和 manifest 的 body_radius_m 确定；z_lower 表示包络触及场景底面。接触面容差 1e-7 m。",
    "odom_gaps": "SO3 时间戳间隔若为 10 ms 的整数倍，记录缺少的 10 ms 槽位；这表示未观测样本，不能单独区分发布、DDS 或记录器丢失。",
    "last_command": "末次指令是终止前按接收顺序最后一条满足 sim/wall 双重条件的 CTBR；不宣称它的执行器消费时间已被记录。",
    "censored_hold": "最后 CTBR 到终止事件接收时刻的间隔单列为右删失等待时间。它不进入完整相邻 CTBR 间隔或频率。",
    "causality": "超期、模型差异和大倾角只能作为观测与关联；这些数据不能确定唯一失效原因。",
}


def load_json(path):
    return json.loads(path.read_text())


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, list):
        return [clean(item) for item in value]
    return value


def quantile(values, probability):
    if not values:
        return None
    xs = sorted(values)
    index = (len(xs) - 1) * probability
    lo, hi = math.floor(index), math.ceil(index)
    return xs[lo] + (xs[hi] - xs[lo]) * (index - lo)


def stats(values):
    valid = [x for x in values if finite(x)]
    return {
        "count": len(valid),
        "nonfinite_count": len(values) - len(valid),
        "mean": statistics.fmean(valid) if valid else None,
        "min": min(valid) if valid else None,
        "p50": quantile(valid, .5),
        "p95": quantile(valid, .95),
        "p99": quantile(valid, .99),
        "max": max(valid) if valid else None,
    }


def interval_metrics(intervals_ms):
    valid = [x for x in intervals_ms if finite(x)]
    duration = sum(valid)
    return {
        **stats(intervals_ms),
        "pooled_frequency_hz": len(valid) * 1000 / duration if duration > 0 else None,
        "zero_count": sum(x == 0 for x in valid),
        "negative_count": sum(x < 0 for x in valid),
        "over_50_ms_count": sum(x > 50 + 1e-6 for x in valid),
        "over_100_ms_count": sum(x > 100 + 1e-6 for x in valid),
    }


def read_csv(path):
    with path.open(newline="") as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def intervals(rows, field):
    return [(right[field] - left[field]) * 1000 for left, right in zip(rows, rows[1:])]


def sequence_metrics(rows):
    sim = intervals(rows, "sim_time_s")
    wall = intervals(rows, "wall_receive_s")
    return {
        "sample_count": len(rows),
        "sim_interval_ms": interval_metrics(sim),
        "wall_receive_interval_ms": interval_metrics(wall),
        "sim_span_s": rows[-1]["sim_time_s"] - rows[0]["sim_time_s"] if len(rows) >= 2 else None,
        "wall_receive_span_s": rows[-1]["wall_receive_s"] - rows[0]["wall_receive_s"] if len(rows) >= 2 else None,
    }


def clock_comparison(rows):
    result = sequence_metrics(rows)
    sim, wall = result["sim_span_s"], result["wall_receive_span_s"]
    result["sim_elapsed_per_wall_elapsed"] = sim / wall if finite(wall) and wall > 0 else None
    result["sim_minus_wall_elapsed_ms"] = (sim - wall) * 1000 if finite(sim) and finite(wall) else None
    gaps = intervals(rows, "sim_time_s")
    result["above_10ms_sim_interval_count"] = sum(x > 10 + 1e-6 for x in gaps)
    result["unobserved_nominal_10ms_slots"] = sum(
        max(0, round(x / 10) - 1) for x in gaps
        if finite(x) and abs(x - round(x / 10) * 10) <= 1e-6)
    return result


def nmpc_metrics(events):
    durations = [event["elapsed_ms"] for event in events]
    valid = [x for x in durations if finite(x)]
    return {
        "elapsed_ms": stats(durations),
        "over_50_ms_count": sum(x > 50 for x in valid),
        "over_50_ms_fraction": sum(x > 50 for x in valid) / len(valid) if valid else None,
        "negative_status_count": sum(event.get("solver_code", 0) < 0 for event in events),
        "solver_code_counts": dict(collections.Counter(str(event.get("solver_code")) for event in events)),
    }


def tilt_deg(row):
    q = [row[key] for key in ("qw", "qx", "qy", "qz")]
    if not all(finite(x) for x in q):
        return None
    norm2 = sum(x * x for x in q)
    if norm2 <= 0:
        return None
    qw, qx, qy, qz = q
    cosine = (qw * qw + qz * qz - qx * qx - qy * qy) / norm2
    return math.degrees(math.acos(max(-1., min(1., cosine))))


def describe_state(row):
    if row is None:
        return None
    return {
        "sim_time_s": row["sim_time_s"], "wall_receive_s": row["wall_receive_s"],
        "position_m": [row[key] for key in ("x", "y", "z")],
        "velocity_world_mps": [row[key] for key in ("vx", "vy", "vz")],
        "quaternion_wxyz": [row[key] for key in ("qw", "qx", "qy", "qz")],
        "body_rate_odom_radps": [row[key] for key in ("wx", "wy", "wz")],
        "tilt_deg": tilt_deg(row),
    }


def describe_control(row):
    if row is None:
        return None
    return {
        "sim_time_s": row["sim_time_s"], "wall_receive_s": row["wall_receive_s"],
        "body_rate_command_radps": [row[key] for key in ("body_rate_x", "body_rate_y", "body_rate_z")],
        "normalized_thrust": row["normalized_thrust"],
    }


def analyze_trial(results, declared, manifest):
    run_dir = results / declared["run_id"]
    result = dict(declared)
    required = ["run.json", "summary.json", "events.jsonl", "trajectory.csv", "control.csv", "launch_inputs.json"]
    missing = [name for name in required if not (run_dir / name).exists()]
    if missing:
        result.update(analysis_status="incomplete", missing_files=missing)
        return result, None
    metadata = load_json(run_dir / "run.json")
    summary = load_json(run_dir / "summary.json")
    events = [json.loads(line) for line in (run_dir / "events.jsonl").read_text().splitlines() if line.strip()]
    terminal_index = next((i for i, event in enumerate(events)
                           if event.get("event") == "mission_end" and event.get("origin") == "collector"), None)
    start = next((event for event in events if event.get("event") == "mission_start"), None)
    if terminal_index is None or start is None:
        result.update(analysis_status="missing_collector_terminal_or_mission_start")
        return result, None
    terminal = events[terminal_index]
    end_sim = float(terminal.get("terminal_time_s", terminal["sim_time_s"]))
    end_wall = float(terminal["wall_receive_s"])
    start_sim = float(start["sim_time_s"])
    odom_raw, control_raw = read_csv(run_dir / "trajectory.csv"), read_csv(run_dir / "control.csv")

    def within(row, lower=start_sim):
        return (lower - 1e-9 <= row["sim_time_s"] <= end_sim + 1e-9
                and row["wall_receive_s"] <= end_wall)

    odom = [row for row in odom_raw if within(row)]
    control = [row for row in control_raw if within(row)]
    measured_events = [event for event in events[:terminal_index]
                       if float(event.get("sim_time_s", float("inf"))) <= end_sim]
    solves = [event for event in measured_events if event.get("event") == "nmpc_result"]
    angles = [tilt_deg(row) for row in odom]
    valid_angles = [angle for angle in angles if finite(angle)]
    last_state = describe_state(odom[-1] if odom else None)
    last_control = describe_control(control[-1] if control else None)
    terminal_motion = {
        "last_state": last_state, "last_control": last_control,
        "tilt_deg": stats(angles),
        "normalized_thrust": stats([row["normalized_thrust"] for row in control]),
        "last_sample_to_terminal_sim_s": end_sim - odom[-1]["sim_time_s"] if odom else None,
        "last_command_to_terminal_sim_s": end_sim - control[-1]["sim_time_s"] if control else None,
        "last_command_to_terminal_receive_ms": (end_wall - control[-1]["wall_receive_s"]) * 1000 if control else None,
    }
    boundary_faces = []
    if terminal.get("reason") == "out_of_bounds":
        params = load_json(run_dir / "launch_inputs.json")["corto_parameters"]
        origin, size = params["map_origin"], params["map_size"]
        position = terminal.get("terminal_position")
        if position is not None:
            radius = manifest["body_radius_m"]
            margins = {}
            for axis, name in enumerate("xyz"):
                margins[name + "_lower"] = position[axis] - origin[axis] - radius
                margins[name + "_upper"] = origin[axis] + size[axis] - radius - position[axis]
            boundary_faces = [name for name, margin in margins.items() if margin <= 1e-7]
            terminal_motion["body_boundary_margin_by_face_m"] = margins
        terminal_motion["body_boundary_faces"] = boundary_faces
    actual_nmpc = nmpc_metrics(solves)
    summary_mean = summary.get("nmpc_mean_ms")
    actual_mean = actual_nmpc["elapsed_ms"]["mean"]
    result.update({
        "analysis_status": "analyzed", "metadata": metadata,
        "input_sha256": {name: sha256(run_dir / name) for name in required},
        "terminal_event": terminal, "terminal_event_index_zero_based": terminal_index,
        "mission_start_sim_time_s": start_sim, "terminal_sim_time_s": end_sim,
        "flight_window_sim_s": end_sim - start_sim,
        "termination_reason": terminal.get("reason"), "success_from_summary": summary.get("success"),
        "raw_counts": {"odom": len(odom_raw), "control": len(control_raw), "nmpc_events": sum(e.get("event") == "nmpc_result" for e in events)},
        "odom": sequence_metrics(odom), "ctbr": sequence_metrics(control), "nmpc": actual_nmpc,
        "terminal_motion": terminal_motion,
        "before_first_body_boundary": terminal_motion if terminal.get("reason") == "out_of_bounds" else None,
        "body_boundary_faces": boundary_faces,
        "summary_consistency": {
            "nmpc_count_matches": len(solves) == summary.get("nmpc_calls"),
            "nmpc_mean_matches": (actual_mean is None and summary_mean is None)
            or (finite(actual_mean) and finite(summary_mean) and math.isclose(actual_mean, summary_mean, abs_tol=1e-6)),
        },
    })
    if declared["mode"] == "so3":
        before_end = [row for row in odom_raw if within(row, lower=float("-inf"))]
        result["so3_clock_comparison"] = {
            "flight_window": clock_comparison(odom),
            "all_before_terminal_including_preflight": clock_comparison(before_end),
        }
    else:
        result["so3_clock_comparison"] = None
    bucket = {
        "odom_sim_ms": intervals(odom, "sim_time_s"),
        "odom_wall_ms": intervals(odom, "wall_receive_s"),
        "ctbr_sim_ms": intervals(control, "sim_time_s"),
        "ctbr_wall_ms": intervals(control, "wall_receive_s"),
        "solves": solves,
        "censored_hold_ms": [terminal_motion["last_command_to_terminal_receive_ms"]],
        "boundary_last_tilt_deg": [last_state["tilt_deg"]] if result["before_first_body_boundary"] and last_state else [],
        "boundary_max_tilt_deg": [max(valid_angles)] if result["before_first_body_boundary"] and valid_angles else [],
        "boundary_last_thrust": [last_control["normalized_thrust"]] if result["before_first_body_boundary"] and last_control else [],
    }
    return result, bucket


def aggregate(trials, buckets, declared, mode, scene=None):
    members = [run for run in trials if run["mode"] == mode and (scene is None or run["scene"] == scene)]
    complete = [run for run in members if run["analysis_status"] == "analyzed"]
    selected = [buckets[run["run_id"]] for run in complete]
    def pooled(key):
        return [item for bucket in selected for item in bucket[key]]
    return {
        "mode": mode, "scene": scene,
        "expected_trials": sum(run["mode"] == mode and (scene is None or run["scene"] == scene) for run in declared),
        "analyzed_trials": len(complete),
        "unavailable_runs": [run["run_id"] for run in members if run["analysis_status"] != "analyzed"],
        "termination_reasons": dict(collections.Counter(run["termination_reason"] for run in complete)),
        "body_boundary_face_counts": dict(collections.Counter(face for run in complete for face in run["body_boundary_faces"])),
        "odom_sim_interval_ms": interval_metrics(pooled("odom_sim_ms")),
        "odom_wall_receive_interval_ms": interval_metrics(pooled("odom_wall_ms")),
        "ctbr_sim_interval_ms": interval_metrics(pooled("ctbr_sim_ms")),
        "ctbr_wall_receive_interval_ms": interval_metrics(pooled("ctbr_wall_ms")),
        "nmpc": nmpc_metrics(pooled("solves")),
        "right_censored_last_control_hold_ms": stats(pooled("censored_hold_ms")),
        "first_body_boundary_last_tilt_deg": stats(pooled("boundary_last_tilt_deg")),
        "first_body_boundary_max_tilt_deg": stats(pooled("boundary_max_tilt_deg")),
        "first_body_boundary_last_normalized_thrust": stats(pooled("boundary_last_thrust")),
        "so3_flight_sim_per_wall_ratios": stats([
            run["so3_clock_comparison"]["flight_window"]["sim_elapsed_per_wall_elapsed"]
            for run in complete if run["so3_clock_comparison"]]),
        "so3_unobserved_flight_10ms_slots": sum(
            run["so3_clock_comparison"]["flight_window"]["unobserved_nominal_10ms_slots"]
            for run in complete if run["so3_clock_comparison"]),
    }


def display(value, digits=3):
    return "—" if value is None else f"{value:.{digits}f}"


def markdown(report):
    hardware = report["hardware"]
    lines = [
        "# 正式回合运行时诊断", "",
        f'已分析 {report["analyzed_trials"]}/{report["expected_trials"]} 回合。'
        "统计仅使用首次终止前的数据，未筛选成功种子。", "",
        "本报告分别报告仿真时间和收包墙钟。nominal 的 0.05 s 时间步不能证明墙钟达到 20 Hz。", "",
        "## 本机硬件", "",
        f'- CPU：{hardware.get("cpu_model", "未记录")}。',
        f'- 逻辑 CPU：{hardware.get("logical_cpus", "未记录")}；插槽：{hardware.get("sockets", "未记录")}。',
        f'- GPU：{hardware.get("gpu", "未记录")}。',
        f'- ROS：{hardware.get("ros_distro", "未记录")}；编译器：{hardware.get("compiler", "未记录")}。',
        "- 来源为本次套件的 dependencies.json。未据论文真实飞行实验硬件推断仿真硬件。",
        "- 本数据不记录逐时 CPU 负载、频率调度及执行器命令消费时间。", "",
        "## 计算定义", "",
    ]
    lines.extend(f'- **{name}**：{text}' for name, text in DEFINITIONS.items())
    lines.extend([
        "", "## 模式和场景汇总", "",
        "周期与求解分位数合并终止前样本。不同回合之间没有构造时间间隔。", "",
        "| 模式 | 场景 | 回合 | odom sim 均值 ms | odom wall 均值 ms | CTBR sim Hz | CTBR wall Hz | 最长 wall 间隔 ms | NMPC 均值 ms | NMPC P95 ms | >50 ms % |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ])
    for group in report["groups"] + report["mode_groups"]:
        solve = group["nmpc"]
        ratio = solve["over_50_ms_fraction"]
        values = [group["odom_sim_interval_ms"]["mean"], group["odom_wall_receive_interval_ms"]["mean"],
                  group["ctbr_sim_interval_ms"]["pooled_frequency_hz"], group["ctbr_wall_receive_interval_ms"]["pooled_frequency_hz"],
                  group["ctbr_wall_receive_interval_ms"]["max"], solve["elapsed_ms"]["mean"], solve["elapsed_ms"]["p95"],
                  ratio * 100 if ratio is not None else None]
        lines.append(f'| {group["mode"]} | {group["scene"] or "全部"} | {group["analyzed_trials"]}/{group["expected_trials"]} | '
                     + " | ".join(display(value) for value in values) + " |")
    lines.extend([
        "", "## 每回合周期和求解", "",
        "| 回合 | 首次终止 | odom/CTBR 样本 | CTBR sim Hz | CTBR wall Hz | 最长 wall 间隔 ms | 末指令至终止接收 ms（删失） | NMPC 次数 | 均值 ms | P95 ms | >50 ms % |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ])
    for run in report["trials"]:
        if run["analysis_status"] != "analyzed":
            lines.append(f'| {run["run_id"]} | {run["analysis_status"]} | — | — | — | — | — | — | — | — | — |')
            continue
        solve = run["nmpc"]; ratio = solve["over_50_ms_fraction"]
        values = [run["ctbr"]["sim_interval_ms"]["pooled_frequency_hz"],
                  run["ctbr"]["wall_receive_interval_ms"]["pooled_frequency_hz"],
                  run["ctbr"]["wall_receive_interval_ms"]["max"],
                  run["terminal_motion"]["last_command_to_terminal_receive_ms"],
                  solve["elapsed_ms"]["count"], solve["elapsed_ms"]["mean"], solve["elapsed_ms"]["p95"],
                  ratio*100 if ratio is not None else None]
        lines.append(f'| {run["run_id"]} | {run["termination_reason"]} | {run["odom"]["sample_count"]}/{run["ctbr"]["sample_count"]} | '
                     + " | ".join(display(value) for value in values) + " |")
    lines.extend([
        "", "## 首次包络越界前的姿态与指令", "",
        "仅列出首次终止原因为 out_of_bounds 的回合。last/max 分别为最后一帧和整个有效飞行窗口最大值。", "",
        "| 回合 | 边界方向 | 终止 sim s | 最后状态 sim s | 最后 z m | last 倾角 ° | max 倾角 ° | 最后 CTBR sim s | 最后归一化推力 | 角速度指令 rad/s |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ])
    for run in report["trials"]:
        motion = run.get("before_first_body_boundary")
        if not motion:
            continue
        state, control = motion["last_state"], motion["last_control"]
        values = [run["terminal_sim_time_s"], state["sim_time_s"] if state else None,
                  state["position_m"][2] if state else None, state["tilt_deg"] if state else None,
                  motion["tilt_deg"]["max"], control["sim_time_s"] if control else None,
                  control["normalized_thrust"] if control else None]
        rates = ", ".join(display(x) for x in control["body_rate_command_radps"]) if control else "—"
        faces = ", ".join(run["body_boundary_faces"]) or "未解析"
        lines.append(f'| {run["run_id"]} | {faces} | ' + " | ".join(display(value) for value in values) + f" | {rates} |")
    lines.extend([
        "", "## SO3 时钟与墙钟对照", "",
        "两种窗口都止于首次终止。预飞行悬停仅用于检查持续推进的时钟，不进入主飞行周期汇总。", "",
        "| 回合 | 飞行 odom 数 | 飞行 sim 跨度 s | 飞行 wall 跨度 s | sim/wall | 全终止前 sim 跨度 s | 全终止前 wall 跨度 s | 最大 sim 间隔 ms | 最大 wall 间隔 ms | 未观测 10 ms 槽位 |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ])
    for run in report["trials"]:
        comp = run.get("so3_clock_comparison")
        if not comp:
            continue
        flight, full = comp["flight_window"], comp["all_before_terminal_including_preflight"]
        values = [flight["sample_count"], flight["sim_span_s"], flight["wall_receive_span_s"],
                  flight["sim_elapsed_per_wall_elapsed"], full["sim_span_s"], full["wall_receive_span_s"],
                  flight["sim_interval_ms"]["max"], flight["wall_receive_interval_ms"]["max"],
                  flight["unobserved_nominal_10ms_slots"]]
        lines.append(f'| {run["run_id"]} | ' + " | ".join(display(value, 6) for value in values) + " |")
    inconsistent = [run["run_id"] for run in report["trials"]
                    if run.get("summary_consistency") and not all(run["summary_consistency"].values())]
    lines.extend(["", "## 完整性和解释范围", "",
                  f'- 未完成或缺失：{", ".join(report["unavailable_runs"]) or "无"}。',
                  f'- 与正式 summary 的 NMPC 次数或均值不一致：{", ".join(inconsistent) or "无"}。',
                  "- 统计保留负状态求解。code>0 的原实现标记不等同于所有调用都收敛到相同精度。",
                  "- 超期、大倾角和模型差异可作为差距分析证据，不能由这些记录断言唯一因果。",
                  "- 精确筛选规则、输入哈希、完整分位数和每回合末状态见 runtime_diagnostics.json。", ""])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    results = args.results.resolve()
    manifest_path = results / "manifest.json"
    manifest = load_json(manifest_path)
    dependencies = load_json(results / "dependencies.json") if (results / "dependencies.json").exists() else {}
    cpu = {entry["field"].rstrip(":"): entry["data"]
           for entry in dependencies.get("cpu", {}).get("lscpu", [])}
    trials, buckets = [], {}
    for declared in manifest["runs"]:
        trial, bucket = analyze_trial(results, declared, manifest)
        trials.append(trial)
        if bucket is not None:
            buckets[trial["run_id"]] = bucket
    modes = list(dict.fromkeys(run["mode"] for run in manifest["runs"]))
    scenes = list(dict.fromkeys(run["scene"] for run in manifest["runs"]))
    report = {
        "schema_version": 1,
        "generated_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "source": "existing trial data only; no ROS, simulation, solver or raw-data mutation",
        "manifest_sha256": sha256(manifest_path), "protocol_version": manifest.get("protocol_version"),
        "script_sha256": sha256(Path(__file__)),
        "source_commits": sorted({run["metadata"]["code_commit"] for run in trials if "metadata" in run}),
        "expected_trials": len(manifest["runs"]), "analyzed_trials": len(buckets),
        "unavailable_runs": [run["run_id"] for run in trials if run["analysis_status"] != "analyzed"],
        "definitions": DEFINITIONS,
        "hardware": {"cpu_model": cpu.get("Model name"), "logical_cpus": cpu.get("CPU(s)"),
                     "sockets": cpu.get("Socket(s)"), "gpu": dependencies.get("gpu"),
                     "ros_distro": dependencies.get("ros_distro"), "compiler": dependencies.get("compiler")},
        "groups": [aggregate(trials, buckets, manifest["runs"], mode, scene) for mode in modes for scene in scenes],
        "mode_groups": [aggregate(trials, buckets, manifest["runs"], mode) for mode in modes],
        "trials": trials,
    }
    report = clean(report)
    (results / "runtime_diagnostics.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    (results / "runtime_diagnostics.md").write_text(markdown(report))
    print(json.dumps({"analyzed": len(buckets), "expected": len(manifest["runs"]),
                      "json": str(results / "runtime_diagnostics.json"),
                      "markdown": str(results / "runtime_diagnostics.md")}, ensure_ascii=False))


if __name__ == "__main__":
    main()

