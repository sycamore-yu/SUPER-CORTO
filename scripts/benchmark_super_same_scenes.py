#!/usr/bin/env python3
"""Run the existing SUPER ROS 2 planner on CORTO's three frozen scenes.

The original SUPER binary and PerfectDrone remain in the separate SUPER repo.
Only the observation/termination and scoring policy is adapted to CORTO's
20-Hz sampled center-point protocol. No SUPER planner source is modified.
"""
import argparse
from contextlib import contextmanager
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

from benchmark_geometry import SceneGeometry


ROOT = Path(__file__).resolve().parents[1]
SUPER = Path(os.environ.get("SUPER_ROS2_ROOT", "/home/tong/tongworkspace/projects/aerial/SUPER"))
MANIFEST = ROOT / "experiments/manifest.json"


@contextmanager
def compiled_super_path():
    """Restore the compiled ROS 2 binary's former source path while it runs."""
    original = Path("/home/tong/tongworkspace/develop/SUPER")
    made_link = False
    if not original.exists() and not original.is_symlink():
        original.parent.mkdir(parents=True, exist_ok=True)
        original.symlink_to(SUPER, target_is_directory=True)
        made_link = True
    elif original.resolve() != SUPER.resolve():
        raise RuntimeError(f"SUPER executable expects {original}, which points elsewhere")
    try:
        yield
    finally:
        if made_link:
            original.unlink()
            original.parent.rmdir()


def prepare(run, output):
    scene_name = run["scene"]
    scene = json.loads((ROOT / "experiments/scenes" / f"{scene_name}.json").read_text())
    pcd = (ROOT / "experiments/scenes" / f"{scene_name}.pcd").resolve()
    inputs = output / "inputs" / run["run_id"]
    inputs.mkdir(parents=True, exist_ok=True)

    planner = yaml.safe_load((SUPER / "super_planner/config/static_dense.yaml").read_text())
    planner["fsm"].update(click_height=scene["goal"][2], visualization_en=False)
    planner["super_planner"].update(visualization_en=False, visual_process=False)
    planner["traj_opt"]["boundary"].update(max_vel=10.0, max_acc=5.0)
    rog = planner["rog_map"]
    rog.update(resolution=0.1, inflation_resolution=0.1, inflation_step=1,
               map_size=[30.0, 15.0, 3.0], fix_map_origin=[0.0, 0.0, 1.5],
               virtual_ground_height=0.0, virtual_ceil_height=3.0,
               load_pcd_en=True, pcd_name=str(pcd))
    # SUPER receives odometry through ROG-Map's ROS callbacks; disabling
    # callbacks makes its original FSM report "No odom" forever.
    rog["ros_callback"]["enable"] = True
    rog["visualization"]["enable"] = False
    planner_path = inputs / "super_planner.yaml"
    planner_path.write_text(yaml.safe_dump(planner, sort_keys=False))

    sim = yaml.safe_load((SUPER / "mars_uav_sim/perfect_drone_sim/config/dense.yaml").read_text())
    sim.update(pcd_name=str(pcd), init_position=dict(zip(("x", "y", "z"), scene["start"])),
               init_yaw=float(json.loads(MANIFEST.read_text())["initial_yaw_rad"]),
               sensing_horizon=15, sensing_rate=10)
    sim_path = inputs / "super_sim.yaml"
    sim_path.write_text(yaml.safe_dump(sim, sort_keys=False))

    trial = dict(config_id="super_same_corto_scenes", trial_id=run["planner_seed"],
                 run_id=run["run_id"], scene=scene_name, planner="SUPER", executor="original_super_perfectdrone",
                 start=scene["start"], goal=scene["goal"], bounds=scene["bounds"],
                 yaw=json.loads(MANIFEST.read_text())["initial_yaw_rad"],
                 map_file=str(pcd), map_sha256=hashlib.sha256(pcd.read_bytes()).hexdigest(),
                 sim_config=str(sim_path), planner_config=str(planner_path),
                 timeout_s=60.0, goal_radius=0.2, collision_radius=0.0,
                 startup_delay_s=3.0, reproduction_level="same_scene_different_execution")
    trial_path = inputs / "trial.json"
    trial_path.write_text(json.dumps(trial, indent=2) + "\n")
    return trial_path, scene


def evaluate(native_dir, scene, source_run):
    """Sample SUPER's measured odometry at 20 Hz and score original geometry."""
    with (native_dir / "states.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    stamps = np.array([float(x["t_s"]) for x in rows])
    points = np.array([[float(x[k]) for k in ("p_x", "p_y", "p_z")] for x in rows])
    order = np.argsort(stamps)
    stamps, points = stamps[order], points[order]
    keep = np.r_[True, np.diff(stamps) > 1e-8]
    stamps, points = stamps[keep], points[keep]
    sample_times = np.arange(0, max(0.0, stamps[-1]) + 1e-8, 0.05)
    if len(sample_times) == 0:
        sample_times = np.array([0.0])
    # The last terminal state carries a genuine odometry sample, so retain it.
    if stamps[-1] - sample_times[-1] > 1e-6:
        sample_times = np.r_[sample_times, stamps[-1]]
    sample_times = np.clip(sample_times, stamps[0], stamps[-1])
    sample_positions = np.column_stack([np.interp(sample_times, stamps, points[:, i]) for i in range(3)])
    sampled = native_dir / "trajectory.csv"
    with sampled.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sim_time_s", "x", "y", "z"])
        writer.writerows(np.column_stack((sample_times, sample_positions)))
    geometry = SceneGeometry(scene)
    clearance = geometry.clearance(sample_positions)
    collisions = np.flatnonzero(clearance <= 0)
    outside = np.flatnonzero(geometry.out_of_bounds(sample_positions))
    near = np.flatnonzero(np.linalg.norm(sample_positions - np.asarray(scene["goal"]), axis=1) < 0.2)
    native = json.loads((native_dir / "result.json").read_text())
    reached = len(near) > 0
    if reached:
        end = int(near[0]) + 1
        collisions = collisions[collisions < end]
        outside = outside[outside < end]
        sample_positions = sample_positions[:end]
        sample_times = sample_times[:end]
        clearance = clearance[:end]
    duration = float(sample_times[-1] - sample_times[0])
    distance = float(np.linalg.norm(np.diff(sample_positions, axis=0), axis=1).sum())
    success = bool(reached and len(collisions) == 0 and len(outside) == 0)
    summary = dict(run_id=source_run["run_id"], scene=source_run["scene"],
                   planner_seed=source_run["planner_seed"], reached_goal=bool(reached),
                   success=success, collision=bool(len(collisions)), out_of_bounds=bool(len(outside)),
                   termination_reason=native["termination_reason"],
                   flight_time_s=duration, mean_speed_mps=distance / duration if duration else None,
                   clearance_m=float(np.min(clearance)), path_length_m=distance,
                   state_samples=len(sample_times), super_odom_samples=native["state_samples"],
                   mean_compute_ms=native.get("mean_compute_ms"),
                   first_collision_time_s=float(sample_times[collisions[0]]) if len(collisions) else None,
                   first_out_of_bounds_time_s=float(sample_times[outside[0]]) if len(outside) else None)
    (native_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--run-id", action="append")
    parser.add_argument("--ros-domain-id", type=int, default=242)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    result_dir = args.results.resolve()
    result_dir.mkdir(parents=True, exist_ok=True)
    manifest = json.loads(MANIFEST.read_text())
    runs = [x for x in manifest["runs"] if args.run_id is None or x["run_id"] in args.run_id]
    if not runs:
        parser.error("No matching predeclared CORTO run IDs")
    binaries = [SUPER / "build/ros2/install" / package / "lib" / package / exe
                for package, exe in (("super_planner", "fsm_node"), ("perfect_drone_sim", "perfect_drone_node"))]
    for binary in binaries:
        if not binary.is_file():
            raise FileNotFoundError(binary)
    provenance = {"super_source": str(SUPER), "super_git_commit": subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=SUPER, text=True).strip(),
                  "super_planner_binary_sha256": hashlib.sha256(binaries[0].read_bytes()).hexdigest(),
                  "perfect_drone_binary_sha256": hashlib.sha256(binaries[1].read_bytes()).hexdigest(),
                  "manifest_sha256": hashlib.sha256(MANIFEST.read_bytes()).hexdigest(),
                  "execution_model": "SUPER original PerfectDrone ideal position tracking",
                  "scoring_model": "CORTO 20 Hz sampled center-point geometry"}
    (result_dir / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOCALHOST_ONLY="1")
    glfw = str(ROOT / ".deps/glfw/usr/lib/x86_64-linux-gnu")
    env["LD_LIBRARY_PATH"] = glfw + os.pathsep + env.get("LD_LIBRARY_PATH", "")
    msg_python = str(ROOT / "install/mars_quadrotor_msgs/lib/python3.12/site-packages")
    env["PYTHONPATH"] = msg_python + os.pathsep + env.get("PYTHONPATH", "")
    for run in runs:
        dest = result_dir / run["run_id"]
        if args.resume and (dest / "summary.json").is_file():
            print("Keeping", run["run_id"], flush=True)
            continue
        if dest.exists():
            raise FileExistsError(dest)
        trial, scene = prepare(run, result_dir)
        cmd = [sys.executable, str(ROOT / "scripts/super_native_recorder.py"), str(trial), str(dest)]
        print("Starting SUPER", run["run_id"], flush=True)
        with (result_dir / "inputs" / run["run_id"] / "runner.log").open("w") as output:
            proc = subprocess.run(cmd, cwd=SUPER, env=env, stdout=output, stderr=subprocess.STDOUT)
        if proc.returncode:
            print("SUPER runner failed", run["run_id"], "code", proc.returncode, flush=True)
            continue
        summary = evaluate(dest, scene, run)
        print(run["run_id"], "arrived", summary["reached_goal"], "safe", summary["success"],
              "time_s", round(summary["flight_time_s"], 2), flush=True)
    summaries = [json.loads(p.read_text()) for p in sorted(result_dir.glob("direct_*/summary.json"))]
    groups = []
    for scene in ("sparse", "medium", "dense"):
        selected = [x for x in summaries if x["scene"] == scene]
        groups.append(dict(scene=scene, completed=len(selected), arrival=sum(x["reached_goal"] for x in selected),
                           safe_success=sum(x["success"] for x in selected),
                           collision=sum(x["collision"] for x in selected),
                           out_of_bounds=sum(x["out_of_bounds"] for x in selected),
                           mean_safe_flight_s=(float(np.mean([x["flight_time_s"] for x in selected if x["success"]]))
                                               if any(x["success"] for x in selected) else None)))
    (result_dir / "report.json").write_text(json.dumps({"groups": groups, "provenance": provenance}, indent=2) + "\n")
    print(json.dumps(groups), flush=True)


if __name__ == "__main__":
    with compiled_super_path():
        main()
