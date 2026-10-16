#!/usr/bin/env python3
"""Offline Phase 1 analysis over every saved trial_NNN. This program never accesses robot hardware."""

from __future__ import annotations

import csv
import json
from dataclasses import replace

import config
from core import ExecutionMetrics, process_execution
from main import trial_dirs


def analyze() -> tuple[list[tuple[ExecutionMetrics, float]], list[str]]:
    """Recompute E for every saved trial and rewrite analysis/execution_metrics.csv.

    Returns (metric, rho) for every usable trial, plus the reasons for the rest.
    """
    output = config.PHASE1_ROOT / "analysis"
    output.mkdir(parents=True, exist_ok=True)
    executions, failures = [], []
    for trial_dir in trial_dirs():
        try:
            with (trial_dir / "problem.json").open(encoding="utf-8") as source:
                rho = float(json.load(source)["rho_m"])
            metric = replace(process_execution(trial_dir / "raw", "phase1", config),
                             run_id=trial_dir.name)
        except (OSError, ValueError, KeyError) as error:
            failures.append(f"invalid:{trial_dir.name}:{error}")
            continue
        executions.append((metric, rho))
    with (output / "execution_metrics.csv").open("w", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["run_id", "motion_group", "max_deviation_m",
                         "commanded_distance_m", "actual_distance_m",
                         "commanded_actual_ratio", "initial_x_m", "initial_y_m",
                         "initial_theta_rad", "rho_m", "within_rho"])
        writer.writerows([*metric.__dict__.values(), rho, int(metric.max_deviation_m <= rho)]
                         for metric, rho in executions)
    return executions, failures


def main() -> None:
    executions, failures = analyze()
    for reason in failures:
        print(f"Skipped {reason}")
    for metric, rho in executions:
        print(f"  {metric.run_id}: E = {metric.max_deviation_m:.3f} m (rho = {rho:.3f} m)")
    within = sum(metric.max_deviation_m <= rho for metric, rho in executions)
    print(f"Analyzed {len(executions)} trial(s); {within} with E <= rho. "
          f"Written to {config.PHASE1_ROOT / 'analysis' / 'execution_metrics.csv'}")


if __name__ == "__main__":
    main()
