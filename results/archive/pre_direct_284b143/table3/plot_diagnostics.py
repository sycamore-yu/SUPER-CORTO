#!/usr/bin/env python3
"""Render the frozen formal data; never alter trials or scoring outputs."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent
SCENES = ("sparse", "medium", "dense")
COLORS = {"nominal": "#2266a4", "so3": "#c74646"}
plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                     "axes.spines.right": False, "figure.dpi": 150})


def trial(mode, scene, seed):
    directory = ROOT / f"{mode}_{scene}_{seed:02d}"
    events = [json.loads(line) for line in (directory / "events.jsonl").read_text().splitlines()]
    index = next(i for i, event in enumerate(events)
                 if event.get("event") == "mission_end" and event.get("origin") == "collector")
    end = events[index]
    start = next(event["sim_time_s"] for event in events if event.get("event") == "mission_start")
    stop = end.get("terminal_time_s", end["sim_time_s"])
    accepted = [event for event in events[:index] if event.get("sim_time_s", stop) <= stop]
    with (directory / "trajectory.csv").open() as stream:
        rows = [row for row in csv.DictReader(stream)
                if start - 1e-9 <= float(row["sim_time_s"]) <= stop + 1e-9
                and float(row["wall_receive_s"]) <= end["wall_receive_s"]]
    return rows, accepted, start, stop, end


def main():
    figure, axes = plt.subplots(1, 3, figsize=(11.5, 3.8), sharex=True, sharey=True)
    max_altitude = 0.49
    for axis, scene in zip(axes, SCENES):
        for seed in range(10):
            rows, _, start, stop, end = trial("so3", scene, seed)
            times = [float(row["sim_time_s"]) - start for row in rows]
            altitude = [float(row["z"]) for row in rows]
            if "terminal_position" in end:
                times.append(stop - start)
                altitude.append(end["terminal_position"][2])
            max_altitude = max(max_altitude, max(altitude))
            axis.plot(times, altitude, color=COLORS["so3"], alpha=0.5, linewidth=1.3)
        axis.axhline(0.322, color="#333333", linestyle="--", linewidth=1.1,
                     label="Envelope lower boundary")
        axis.set_title(f"{scene.title()} | 10 predeclared trials")
        axis.set_xlabel("Flight simulation time (s)")
        axis.grid(alpha=0.15)
    axes[0].set_ylim(0.30, max_altitude + 0.01)
    axes[0].set_ylabel("Vehicle center altitude (m)")
    axes[0].legend(loc="lower left", fontsize=8)
    figure.suptitle("SO(3) altitude before the first terminal condition", fontsize=13)
    figure.tight_layout()
    for extension in ("png", "svg"):
        figure.savefig(ROOT / f"so3_altitude.{extension}", bbox_inches="tight")
    plt.close(figure)

    figure, axes = plt.subplots(1, 3, figsize=(11.5, 3.8), sharey=True)
    for axis, scene in zip(axes, SCENES):
        values = []
        for mode in ("nominal", "so3"):
            elapsed = []
            for seed in range(10):
                _, events, _, _, _ = trial(mode, scene, seed)
                elapsed.extend(event["elapsed_ms"] for event in events if event.get("event") == "nmpc_result")
            values.append(np.asarray(elapsed))
        boxes = axis.boxplot(values, labels=["Nominal", "SO(3)"], patch_artist=True,
                             medianprops={"color": "black"}, flierprops={"markersize": 3})
        for box, mode in zip(boxes["boxes"], ("nominal", "so3")):
            box.set_facecolor(COLORS[mode])
            box.set_alpha(0.55)
        axis.axhline(50, color="#a42e2e", linestyle="--", linewidth=1.1)
        axis.set_title(f"{scene.title()} | n={len(values[0])}/{len(values[1])}")
        axis.grid(axis="y", alpha=0.15)
    axes[0].set_ylabel("NMPC solve wall time (ms)")
    figure.suptitle("NMPC before termination | dashed line: 50 ms target period", fontsize=13)
    figure.tight_layout()
    for extension in ("png", "svg"):
        figure.savefig(ROOT / f"nmpc_runtime.{extension}", bbox_inches="tight")
    plt.close(figure)


if __name__ == "__main__":
    main()
