#!/usr/bin/env python3
"""Score every recorded trial using the frozen manifest; never select successful seeds."""
import argparse
import csv
import json
from pathlib import Path
import statistics

import numpy as np

from benchmark_geometry import SceneGeometry, densify


ROOT = Path(__file__).resolve().parents[1]
PAPER = {"sparse": [100, 5.15, 4.47, 0.11, 43.1],
         "medium": [100, 5.23, 4.42, 0.07, 57.9],
         "dense": [100, 4.24, 4.73, 0.15, 59.1]}


def mean(values):
    values = [x for x in values if x is not None]
    return statistics.fmean(values) if values else None


def first_goal_entry(times, points, goal, radius):
    for i, point in enumerate(points):
        if np.linalg.norm(point - goal) < radius:
            return i, float(times[i]), point
        if i + 1 == len(points):
            break
        offset, delta = point - goal, points[i + 1] - point
        a = float(delta @ delta)
        b = 2 * float(offset @ delta)
        c = float(offset @ offset) - radius * radius
        discriminant = b * b - 4 * a * c
        if a > 0 and discriminant > 0:
            root = (-b - np.sqrt(discriminant)) / (2 * a)
            if 0 <= root < 1:
                return i + 1, float(times[i] + root * (times[i + 1] - times[i])), point + root * delta
    return None


def score_trial(run_dir, manifest, run, scene):
    events_path = run_dir / "events.jsonl"
    events = [json.loads(x) for x in events_path.read_text().splitlines() if x.strip()] if events_path.exists() else []
    recorded = []
    if (run_dir / "trajectory.csv").exists():
        with (run_dir / "trajectory.csv").open() as stream:
            recorded = list(csv.DictReader(stream))
    terminal_index = next((i for i, x in enumerate(events)
                           if x.get("event") == "mission_end" and x.get("origin") == "collector"), None)
    if terminal_index is None:
        terminal_index = next((i for i, x in enumerate(events) if x.get("event") == "mission_end"), None)
    terminal = events[terminal_index] if terminal_index is not None else None
    measured_events = events[:terminal_index] if terminal_index is not None else events
    if terminal:
        statistics_end = float(terminal.get("terminal_time_s", terminal["sim_time_s"]))
        measured_events = [x for x in measured_events
                           if float(x.get("sim_time_s", float("inf"))) <= statistics_end]
    start_events = [x for x in events if x.get("event") == "mission_start"]
    plans = [x for x in measured_events if x.get("event") == "plan_result"]
    solves = [x for x in measured_events if x.get("event") == "nmpc_result"]
    replan = [x for x in plans if x.get("initial") is False]
    result = dict(run)
    result.update({"recorded_odom_samples": len(recorded), "event_count": len(events),
                   "termination_reason": terminal.get("reason", "unknown") if terminal else "missing_mission_end",
                   "terminal_time_s": terminal.get("terminal_time_s", terminal.get("sim_time_s")) if terminal else None,
                   "after_terminal_plan_results": sum(x.get("event") == "plan_result" for x in events) - len(plans),
                   "collision_model": manifest["collision_model"], "body_radius_m": manifest["body_radius_m"],
                   "reached_goal": False, "success": False,
                   "flight_time_s": None, "flight_observed_duration_s": None,
                   "recording_span_sim_s": (max(float(x["sim_time_s"]) for x in recorded) - min(float(x["sim_time_s"]) for x in recorded)) if recorded else None,
                   "path_length_m": None, "mean_speed_mps": None,
                   "center_clearance_m": None, "body_clearance_m": None, "original_r0p1_clearance_m": None,
                   "center_clearance_lower_m": None, "body_clearance_lower_m": None,
                   "body_collision": False, "center_collision": False, "original_r0p1_collision": False,
                   "center_out_of_bounds": False, "body_out_of_bounds": False,
                   "first_body_collision_time_s": None, "first_body_collision_position": None,
                   "planning_calls": len(plans), "planning_phase_unlabelled_calls": sum("initial" not in x for x in plans),
                   "planning_failures": sum(not x.get("success", False) for x in plans),
                   "replanning_calls": len(replan), "replanning_failures": sum(not x.get("success", False) for x in replan),
                   "nmpc_calls": len(solves), "nmpc_failures": sum(not x.get("success", False) for x in solves),
                   "planning_all_ms": mean([x.get("elapsed_ms") for x in plans]),
                   "online_replanning_ms": mean([x.get("elapsed_ms") for x in replan]),
                   "nmpc_mean_ms": mean([x.get("elapsed_ms") for x in solves])})
    if not start_events or not recorded:
        result["evaluation_reason"] = "missing_mission_start" if not start_events else "missing_odometry"
        return result
    start_time = float(start_events[0]["sim_time_s"])
    raw_times = np.array([float(x["sim_time_s"]) for x in recorded])
    result["out_of_order_odom_samples"] = int(np.sum(np.diff(raw_times) < 0))
    end_time = float(terminal.get("evaluation_end_sim_time_s", terminal["sim_time_s"])) if terminal else float("inf")
    unique = {float(x["sim_time_s"]): x for x in recorded
              if start_time - 1e-9 <= float(x["sim_time_s"]) <= end_time + 1e-9}
    result["duplicate_odom_samples"] = len(recorded) - len(set(raw_times.tolist()))
    times = np.array(sorted(unique))
    points = np.array([[float(unique[t][key]) for key in ("x", "y", "z")] for t in times])
    if not len(times) or not np.isfinite(points).all():
        result["evaluation_reason"] = "nonfinite_or_missing_flight_state"
        return result
    result["flight_observed_duration_s"] = float(times[-1] - start_time)
    entry = first_goal_entry(times, points, np.asarray(scene["goal"]), manifest["goal_tolerance_m"])
    if entry and terminal and entry[1] > float(terminal.get("terminal_time_s", end_time)) + 1e-9:
        entry = None
    if entry:
        index, arrival, position = entry
        times = np.r_[times[:index], arrival]
        points = np.vstack((points[:index], position))
        result["reached_goal"] = True
        result["goal_entry_time_s"] = arrival
    duration = float(times[-1] - start_time)
    path_length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
    dense_times, dense_points = densify(times, points, manifest["continuous_sample_spacing_m"])
    geometry = SceneGeometry(scene)
    clearance = geometry.clearance(dense_points)
    # Distance is 1-Lipschitz. This bound includes ellipse and segment sampling error.
    margin = geometry.error_bound_m + manifest["continuous_sample_spacing_m"] / 2
    lower = clearance - margin
    body_radius = manifest["body_radius_m"]
    reference_radius = manifest["original_reference_radius_m"]
    boundary = np.min(np.minimum(dense_points - geometry.bounds[0], geometry.bounds[1] - dense_points), axis=1)
    body_collisions = np.flatnonzero(lower <= body_radius)
    result.update({"flight_time_s": duration, "path_length_m": path_length,
                   "mean_speed_mps": path_length / duration if duration > 0 else None,
                   "center_clearance_m": float(np.min(clearance)),
                   "body_clearance_m": float(np.min(clearance) - body_radius),
                   "original_r0p1_clearance_m": float(np.min(clearance) - reference_radius),
                   "center_clearance_lower_m": float(np.min(lower)),
                   "body_clearance_lower_m": float(np.min(lower) - body_radius),
                   "clearance_error_bound_m": margin,
                   "center_collision": bool(np.min(lower) <= 0),
                   "body_collision": bool(len(body_collisions)),
                   "original_r0p1_collision": bool(np.min(lower) <= reference_radius),
                   "center_out_of_bounds": bool(np.min(boundary) <= 0),
                   "body_out_of_bounds": bool(np.min(boundary) <= body_radius),
                   "min_body_boundary_margin_m": float(np.min(boundary) - body_radius),
                   "continuous_check_samples": len(dense_times)})
    if len(body_collisions):
        index = body_collisions[0]
        result["first_body_collision_time_s"] = float(dense_times[index])
        result["first_body_collision_position"] = dense_points[index].tolist()
    if terminal and terminal.get("reason") == "collision":
        result["body_collision"] = True
        result["first_body_collision_time_s"] = terminal.get("first_contact_time_s", result["first_body_collision_time_s"])
        result["first_body_collision_position"] = terminal.get("first_contact_position", result["first_body_collision_position"])
    if terminal and terminal.get("reason") == "out_of_bounds":
        result["body_out_of_bounds"] = True
    result["success"] = result["reached_goal"] and not result["body_collision"] and not result["body_out_of_bounds"]
    reasons = []
    for key in ("body_collision", "body_out_of_bounds"):
        if result[key]:
            reasons.append(key)
    if not result["reached_goal"]:
        reasons.append(result["termination_reason"])
    result["evaluation_reason"] = "success" if result["success"] else "+".join(reasons)
    return result


def format_value(value, digits=2):
    return "—" if value is None else f"{value:.{digits}f}"


def report(experiments, results):
    manifest = json.loads((experiments / "manifest.json").read_text())
    trials = []
    missing = []
    for run in manifest["runs"]:
        run_dir = results / run["run_id"]
        if not (run_dir / "run.json").exists():
            missing.append(run["run_id"])
            continue
        scene = json.loads((experiments / "scenes" / f"{run['scene']}.json").read_text())
        result = score_trial(run_dir, manifest, run, scene)
        (run_dir / "summary.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        trials.append(result)
    results.mkdir(parents=True, exist_ok=True)
    keys = list(dict.fromkeys(k for result in trials for k in result))
    with (results / "trials.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(trials)
    groups = []
    for mode in ("nominal", "so3"):
        for scene in PAPER:
            observed = [x for x in trials if x["mode"] == mode and x["scene"] == scene]
            success = [x for x in observed if x["success"]]
            reached = [x for x in observed if x["reached_goal"]]
            # Denominator is always the ten predeclared trials, including failures.
            group = {"mode": mode, "scene": scene, "completed": len(observed), "expected": 10,
                     "successes": len(success),
                     "reached_goal_count": len(reached),
                     "reached_goal_rate_percent": 100 * len(reached) / 10 if len(observed) == 10 else None,
                     "reached_run_flight_time_s": mean([x["flight_time_s"] for x in reached]),
                     "reached_run_mean_speed_mps": mean([x["mean_speed_mps"] for x in reached]),
                     "all_run_observed_flight_duration_s": mean([x["flight_observed_duration_s"] for x in observed]),
                     "all_run_recording_span_sim_s": mean([x["recording_span_sim_s"] for x in observed]),
                     "success_rate_percent": 100 * len(success) / 10 if len(observed) == 10 else None,
                     "flight_time_s": mean([x["flight_time_s"] for x in success]),
                     "mean_speed_mps": mean([x["mean_speed_mps"] for x in success]),
                     "body_clearance_m": mean([x["body_clearance_m"] for x in success]),
                     "center_clearance_m": mean([x["center_clearance_m"] for x in success]),
                     "all_run_center_clearance_m": mean([x["center_clearance_m"] for x in observed]),
                     "all_run_r0p1_clearance_m": mean([x["original_r0p1_clearance_m"] for x in observed]),
                     "all_run_body_clearance_m": mean([x["body_clearance_m"] for x in observed]),
                     "failed_run_min_body_clearance_m": min((x["body_clearance_m"] for x in observed if x["body_clearance_m"] is not None), default=None),
                     "planning_all_ms": (sum(x["planning_all_ms"] * x["planning_calls"] for x in observed if x["planning_all_ms"] is not None)
                                         / sum(x["planning_calls"] for x in observed)) if sum(x["planning_calls"] for x in observed) else None,
                     "online_replanning_ms": (sum(x["online_replanning_ms"] * x["replanning_calls"] for x in observed if x["online_replanning_ms"] is not None)
                                              / sum(x["replanning_calls"] for x in observed)) if sum(x["replanning_calls"] for x in observed) else None,
                     "planning_failures": sum(x["planning_failures"] for x in observed),
                     "nmpc_failures": sum(x["nmpc_failures"] for x in observed)}
            groups.append(group)
    with (results / "table3.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=groups[0])
        writer.writeheader()
        writer.writerows(groups)
    lines = ["# Table III reproduction", "", f"Recorded trials: {len(trials)}/60. Missing: {len(missing)}.", "",
             "Primary collision model: EGO rotor enclosing sphere, radius 0.322 m. Goal radius: 0.2 m.",
             "Flight time, speed and clearance average the successful trials; counts and all failed trajectories remain in trials.csv.",
             "Clearance uses original solid geometry without planner inflation. Paper clearance geometry/aggregation is unspecified.", "",
             "| Mode | Scene | Trials | Success | Flight s | Speed m/s | Body clearance m | Center clearance m | Online replan ms | All plan ms |",
             "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for scene, values in PAPER.items():
        lines.append(f"| Paper | {scene} | 10 | 100% | {values[1]:.2f} | {values[2]:.2f} | {values[3]:.2f} (paper definition unspecified) | — | {values[4]:.1f} | — |")
    for group in groups:
        rate = format_value(group['success_rate_percent'], 0) + "%" if group['success_rate_percent'] is not None else "pending"
        lines.append(f"| {group['mode']} | {group['scene']} | {group['completed']}/10 | {rate} | "
                     + " | ".join(format_value(group[key]) for key in ("flight_time_s", "mean_speed_mps", "body_clearance_m", "center_clearance_m", "online_replanning_ms", "planning_all_ms")) + " |")
    lines += ["", "## Clearance sensitivity on terminated trajectories", "",
              "These are clearances of the same truncated trajectories, not counterfactual success rates for smaller vehicles.", "",
              "| Mode | Scene | Center clearance m | 0.1 m sphere clearance m | 0.322 m envelope clearance m |", "|---|---|---:|---:|---:|"]
    for group in groups:
        clearances = [format_value(group[key], 3) for key in ("all_run_center_clearance_m", "all_run_r0p1_clearance_m", "all_run_body_clearance_m")]
        lines.append(f"| {group['mode']} | {group['scene']} | " + " | ".join(clearances) + " |")
    lines += ["", "## Reached-goal and observation metrics", "",
              "Formal trials stop at the first terminal condition. Observation spans exclude the post-terminal audit tail for flight metrics.", "",
              "| Mode | Scene | Reached / declared | Reached flight s | Reached speed m/s | All-run observed flight s | All-run recorded sim s |",
              "|---|---|---:|---:|---:|---:|---:|"]
    for group in groups:
        lines.append(f"| {group['mode']} | {group['scene']} | {group['reached_goal_count']}/10 | "
                     + " | ".join(format_value(group[key]) for key in ("reached_run_flight_time_s", "reached_run_mean_speed_mps", "all_run_observed_flight_duration_s", "all_run_recording_span_sim_s")) + " |")
    lines += ["", "## Missing trials", "", ", ".join(missing) if missing else "None."]
    (results / "table3.md").write_text("\n".join(lines) + "\n")
    (results / "report.json").write_text(json.dumps({"groups": groups, "missing_runs": missing}, indent=2) + "\n")
    print(json.dumps({"recorded": len(trials), "missing": len(missing), "report": str(results / "table3.md")}))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--experiments", type=Path, default=ROOT / "experiments")
    parser.add_argument("--results", type=Path, required=True)
    args = parser.parse_args()
    report(args.experiments, args.results)


if __name__ == "__main__":
    main()
