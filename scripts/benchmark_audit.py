#!/usr/bin/env python3
"""Audit frozen raw trials and replay their direct CTBR/RK4 state transitions."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FIELDS = ("x", "y", "z", "vx", "vy", "vz", "qw", "qx", "qy", "qz")
CONTROLS = ("body_rate_x", "body_rate_y", "body_rate_z", "normalized_thrust")


def derivative(x, u):
    w, a, b, c = x[6:]
    rx, ry, rz, thrust = u
    thrust = thrust * 9.81 / 0.6
    return np.r_[x[3:6],
                 2 * (a*c + w*b) * thrust, 2 * (b*c - w*a) * thrust,
                 (1 - 2 * (a*a + b*b)) * thrust - 9.81,
                 .5 * (-rx*a - ry*b - rz*c), .5 * (rx*w + rz*b - ry*c),
                 .5 * (ry*w - rz*a + rx*c), .5 * (rz*w + ry*a - rx*b)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--experiments", type=Path, default=ROOT / "experiments")
    args = parser.parse_args()
    results = args.results.resolve()
    manifest_path = args.experiments / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    commit = (results / "SOURCE_COMMIT").read_text().strip()
    scene_hashes = {x["scene"]: x["scene_sha256"] for x in manifest["scenes"]}
    manifest_hash = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    trials = []
    for run in manifest["runs"]:
        directory = results / run["run_id"]
        metadata = json.loads((directory / "run.json").read_text())
        summary = json.loads((directory / "summary.json").read_text())
        exits = json.loads((directory / "process_exit.json").read_text())
        with (directory / "trajectory.csv").open() as stream:
            states = list(csv.DictReader(stream))
        with (directory / "control.csv").open() as stream:
            controls = list(csv.DictReader(stream))
        events = [json.loads(x) for x in (directory / "events.jsonl").read_text().splitlines()]
        action = {round(float(row["sim_time_s"]), 9): np.array([float(row[k]) for k in CONTROLS]) for row in controls}
        errors, gaps, missing = [], [], []
        for before, after in zip(states, states[1:]):
            time = float(before["sim_time_s"])
            gaps.append(float(after["sim_time_s"]) - time)
            u = action.get(round(time, 9))
            if u is None:
                missing.append(time)
                continue
            x = np.array([float(before[k]) for k in FIELDS])
            target = np.array([float(after[k]) for k in FIELDS])
            k1 = derivative(x, u)
            k2 = derivative(x + .025*k1, u)
            k3 = derivative(x + .025*k2, u)
            k4 = derivative(x + .05*k3, u)
            prediction = x + .05/6 * (k1 + 2*k2 + 2*k3 + k4)
            errors.append(float(np.max(np.abs(prediction - target))))
        terminal = next(x for x in events if x["event"] == "mission_end" and x.get("origin") == "collector")
        violations = [x for x in events if x["event"] in ("collision", "out_of_bounds")]
        intervals = np.diff([float(x["wall_receive_s"]) for x in controls])
        log = (directory / "launch.log").read_text()
        errors_in_log = [line for line in log.splitlines() if "process has died" in line or "terminate called" in line]
        result = dict(run_id=run["run_id"], states=len(states), controls=len(controls),
                      transitions=len(errors), max_rk4_error=max(errors, default=0),
                      max_step_error=max((abs(x-.05) for x in gaps), default=0), missing_controls=missing,
                      wall_control_interval_mean_ms=float(np.mean(intervals)*1000) if len(intervals) else None,
                      wall_control_interval_max_ms=float(np.max(intervals)*1000) if len(intervals) else None,
                      source_clean=metadata["code_commit"] == commit and not metadata["tracked_source_dirty"],
                      inputs_match=metadata["manifest_sha256"] == manifest_hash and metadata["scene_sha256"] == scene_hashes[run["scene"]],
                      termination=terminal["reason"], reached_goal=summary["reached_goal"], safe_success=summary["success"],
                      first_violation_sim_time=min((x["sim_time_s"] for x in violations), default=None),
                      terminal_sim_time=terminal["sim_time_s"],
                      clean_exit=exits["launch"] == 0 and exits["recorder"] == 0 and not errors_in_log,
                      process_errors=errors_in_log)
        result["replay_passed"] = not missing and result["max_step_error"] < 1e-9 and result["max_rk4_error"] < 1e-9
        trials.append(result)
    result = {"source_commit": commit, "expected": len(manifest["runs"]), "completed": len(trials),
              "total_states": sum(x["states"] for x in trials), "total_controls": sum(x["controls"] for x in trials),
              "transitions": sum(x["transitions"] for x in trials),
              "max_rk4_error": max(x["max_rk4_error"] for x in trials),
              "source_and_inputs_passed": all(x["source_clean"] and x["inputs_match"] for x in trials),
              "replay_passed": all(x["replay_passed"] for x in trials),
              "clean_exits": all(x["clean_exit"] for x in trials),
              "trials": trials}
    (results / "execution_audit.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key != "trials"}))
    assert result["source_and_inputs_passed"] and result["replay_passed"] and result["clean_exits"]


if __name__ == "__main__":
    main()
