#!/usr/bin/env python3
"""Run the predeclared trial list and retain failed trials and process logs."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from benchmark_report import report


ROOT = Path(__file__).resolve().parents[1]


def stop(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--experiments", type=Path, default=ROOT / "experiments")
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--run-id", action="append")
    parser.add_argument("--ros-domain-id", type=int, default=185)
    parser.add_argument("--resume", action="store_true", help="Skip every completed trial, including failures")
    args = parser.parse_args()
    experiments, results = args.experiments.resolve(), args.results.resolve()
    manifest = json.loads((experiments / "manifest.json").read_text())
    runs = [x for x in manifest["runs"] if args.run_id is None or x["run_id"] in args.run_id]
    if not runs:
        parser.error("No predeclared trials match the selection")
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.ros_domain_id))
    for run in runs:
        output = results / run["run_id"]
        if (output / "summary.json").exists() and args.resume:
            print(f"Retaining completed trial {run['run_id']}", flush=True)
            continue
        if (output / "trajectory.csv").exists():
            raise FileExistsError(f"Existing raw trial must be retained: {output}")
        output.mkdir(parents=True, exist_ok=True)
        command = ["ros2", "launch", str(ROOT / "launch/table3.launch.py"),
                   f"scene:={run['scene']}", f"planner_seed:={run['planner_seed']}",
                   f"output_dir:={output}", "rviz:=false"]
        (output / "launch_command.json").write_text(json.dumps({"argv": command, "ros_domain_id": args.ros_domain_id}, indent=2) + "\n")
        print(f"Starting {run['run_id']}", flush=True)
        with (output / "recorder.log").open("w") as recorder_log, (output / "launch.log").open("w") as launch_log:
            recorder = subprocess.Popen([sys.executable, str(ROOT / "scripts/benchmark_record.py"),
                                         "--experiments", str(experiments), "--run-id", run["run_id"], "--output", str(output)],
                                        cwd=ROOT, env=env, stdout=recorder_log, stderr=subprocess.STDOUT, start_new_session=True)
            launch = None
            natural_exit = None
            try:
                deadline = time.monotonic() + 15
                while not (output / "ready").exists() and recorder.poll() is None and time.monotonic() < deadline:
                    time.sleep(0.05)
                if not (output / "ready").exists():
                    raise RuntimeError(f"Recorder did not start; inspect {output / 'recorder.log'}")
                launch = subprocess.Popen(command, cwd=ROOT, env=env, stdout=launch_log,
                                          stderr=subprocess.STDOUT, start_new_session=True)
                while recorder.poll() is None:
                    if launch.poll() is not None:
                        # The recorder writes a terminal event and evaluates available raw data.
                        natural_exit = launch.returncode
                        (output / "process_exit.json").write_text(json.dumps({"launch_natural_exit": natural_exit}) + "\n")
                        stop(recorder)
                        break
                    time.sleep(0.1)
                if recorder.returncode:
                    raise RuntimeError(f"Recorder exited {recorder.returncode}; inspect {output / 'recorder.log'}")
            finally:
                if launch is not None:
                    stop(launch)
                stop(recorder)
            (output / "process_exit.json").write_text(json.dumps({"launch": launch.returncode if launch else None,
                                                                   "launch_natural_exit": natural_exit,
                                                                   "recorder": recorder.returncode}, indent=2) + "\n")
    report(experiments, results)


if __name__ == "__main__":
    main()
