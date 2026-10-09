#!/usr/bin/env python3
"""Offline Phase 1 analysis. This program never accesses robot hardware."""

from __future__ import annotations

import csv
import json

import config
from core import ExecutionMetrics, process_execution


def analyze() -> tuple[list[ExecutionMetrics], list[tuple[str, float, str]]]:
    """Recompute every complete instance and rewrite both analysis CSV files."""
    with config.PROBLEMS_CSV.open(newline="", encoding="utf-8") as source:
        problems = list(csv.DictReader(source))
    output = config.PHASE1_ROOT / "analysis"
    output.mkdir(parents=True, exist_ok=True)
    executions = []
    instances = []
    for problem in problems:
        if problem["status"] != "ok":
            continue
        instance = problem["instance"]
        current = []
        for candidate in range(1, config.CANDIDATES_PER_INSTANCE + 1):
            run_id = f"{instance}_C{candidate:02d}"
            metric = process_execution(config.PHASE1_ROOT / "raw" / run_id,
                                       "phase1", config)
            executions.append(metric)
            current.append(metric.max_deviation_m)
        instances.append((instance, max(current), json.dumps(current, separators=(",", ":"))))
    with (output / "execution_metrics.csv").open("w", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["run_id", "motion_group", "max_deviation_m",
                         "commanded_distance_m", "actual_distance_m",
                         "commanded_actual_ratio", "initial_x_m", "initial_y_m",
                         "initial_theta_rad"])
        writer.writerows(item.__dict__.values() for item in executions)
    with (output / "instance_metrics.csv").open("w", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["instance", "max_candidate_deviation_m", "candidate_deviations_m"])
        writer.writerows(instances)
    return executions, instances


def main() -> None:
    executions, instances = analyze()
    print(f"Analyzed {len(executions)} executions across {len(instances)} complete instances")


if __name__ == "__main__":
    main()
