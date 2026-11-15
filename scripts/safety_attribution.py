#!/usr/bin/env python3
"""Count diagnostic events before and after the first measured safety violation."""
import argparse
from collections import Counter
import json
from pathlib import Path


def analyze(results: Path):
    scenes = {}
    for scene in ("sparse", "medium", "dense"):
        total = Counter()
        for summary_file in sorted(results.glob(f"direct_{scene}_*/summary.json")):
            summary = json.loads(summary_file.read_text())
            events = [json.loads(line) for line in (summary_file.parent / "events.jsonl").read_text().splitlines()]
            terminal = next((i for i, event in enumerate(events)
                             if event["event"] == "mission_end" and event.get("origin") == "collector"), len(events))
            end = events[terminal]["sim_time_s"] if terminal < len(events) else float("inf")
            first = min((value for value in (summary.get("first_collision_time_s"),
                                             summary.get("first_out_of_bounds_time_s")) if value is not None),
                        default=float("inf"))
            total["runs"] += 1
            total["safe_runs"] += bool(summary["success"])
            for event in events[:terminal]:
                if event.get("sim_time_s", 0) > end + 1e-9:
                    continue
                phase = "safe" if summary["success"] else (
                    "pre" if event.get("sim_time_s", 0) < first else "post")
                if event["event"] == "nmpc_result" and event.get("success"):
                    total[f"{phase}.nmpc_positive"] += 1
                    total[f"{phase}.corridor_exceeded"] += (
                        event.get("maximum_corridor_violation", 0) > 1e-3)
                    total[f"{phase}.floor_predicted"] += (
                        event.get("minimum_predicted_z_m", 1) < 0)
                elif event["event"] == "topo_query" and event.get("reason"):
                    total[f"{phase}.topo_{event['reason']}"] += 1
                elif event["event"] == "plan_handoff":
                    total[f"{phase}.handoff_rejected"] += not event["accepted"]
        scenes[scene] = dict(total)
    combined = Counter()
    for counts in scenes.values():
        combined.update(counts)
    return {"scenes": scenes, "total": dict(combined),
            "definition": "pre: before first sampled collision or out-of-bounds; post: after; "
                          "safe: whole episode without violation. Corridor threshold 0.001."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", type=Path, required=True)
    args = parser.parse_args()
    output = analyze(args.results.resolve())
    (args.results / "safety_attribution.json").write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n")
    print(json.dumps(output["total"], sort_keys=True))
