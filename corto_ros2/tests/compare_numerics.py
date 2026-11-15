#!/usr/bin/env python3
"""Run the unmodified upstream oracle and ROS 2 port with identical fixtures."""
import argparse
import json
import math
from pathlib import Path
import subprocess

def read_values(path):
    values = {}
    for line in path.read_text().splitlines():
        key, count, *items = line.split()
        assert int(count) == len(items), (key, count)
        values[key] = list(map(float, items))
    return values

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--upstream", required=True)
    p.add_argument("--ported", required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    upstream = args.output / "upstream_vectors.txt"
    ported = args.output / "ported_vectors.txt"
    for executable, output in [(args.upstream, upstream), (args.ported, ported)]:
        run = subprocess.run([executable, str(output)], capture_output=True, text=True)
        output.with_suffix(".log").write_text(run.stdout + run.stderr)
        if run.returncode:
            raise SystemExit(f"{executable} failed: {run.returncode}; see {output.with_suffix('.log')}")
    a, b = read_values(upstream), read_values(ported)
    assert a.keys() == b.keys(), "fixture output keys differ"
    results = {}
    for key in a:
        assert len(a[key]) == len(b[key]), key
        assert all(math.isfinite(v) for v in a[key] + b[key]), key
        err = max((abs(x-y) for x,y in zip(a[key],b[key])), default=0)
        scale = max((abs(x) for x in a[key]), default=0)
        tolerance = 1e-9 + 1e-8 * scale
        results[key] = {"count":len(a[key]),"max_abs_error":err,"tolerance":tolerance,"pass":err<=tolerance}
    summary = {
        "upstream_commit":"e05a2e60f78d9ae4425981c1343913c3c67d3bb3",
        "pass":all(r["pass"] for r in results.values()),
        "method":"Separate executables; vendor algorithms unmodified; same Eigen/PCL/NLopt/MOSEK; fixed PRM RNG and inputs.",
        "map_fixture":"Same voxel column occupancy. ROG adapter is covered by separate ROS integration tests.",
        "results":results,
    }
    (args.output/"regression.json").write_text(json.dumps(summary, indent=2)+"\n")
    print(json.dumps(summary, indent=2))
    return 0 if summary["pass"] else 1

if __name__ == "__main__":
    raise SystemExit(main())
