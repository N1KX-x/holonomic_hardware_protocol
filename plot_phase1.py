#!/usr/bin/env python3
"""Draw each Phase 1 candidate: planned RRT path, real trajectory, and obstacles.

Usage:
    python3 plot_phase1.py I047          # one instance
    python3 plot_phase1.py I047 I012     # several instances
    python3 plot_phase1.py all           # every instance that has plans

An instance that has not been collected yet is drawn with its planned paths
only. This program never accesses robot hardware.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import config
from core import ExecutionTrace, load_candidate_csv, nominal_step, trace_execution


SAMPLES_PER_SEGMENT = 20


def plot_dir() -> Path:
    return config.PHASE1_ROOT / "analysis" / "plots"


def _dense_path(commands, start):
    """Sample the exact curved nominal motion, plus the segment boundary points."""
    points = [start[:2]]
    boundaries = [start[:2]]
    state = start
    for command in commands:
        for index in range(1, SAMPLES_PER_SEGMENT + 1):
            x, y, _ = nominal_step(*state, command, config,
                                   command.dt*index/SAMPLES_PER_SEGMENT)
            points.append((x, y))
        state = nominal_step(*state, command, config)
        boundaries.append(state[:2])
    return points, boundaries


def _problem_row(instance_id: str) -> dict:
    with config.PROBLEMS_CSV.open(newline="", encoding="utf-8") as source:
        row = next((item for item in csv.DictReader(source)
                    if item["instance"] == instance_id), None)
    if row is None:
        raise ValueError(f"Unknown instance {instance_id}")
    return row


def _find_run(run_id: str) -> Path | None:
    raw = config.PHASE1_ROOT / "raw"
    for name in (run_id, f"{run_id}_discarded"):
        if (raw / name).is_dir():
            return raw / name
    return None


def plot_candidate(row: dict, plan_path: Path, run_dir: Path | None, output: Path) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    run_id = f"{row['instance']}_{plan_path.stem}"
    rho = float(row["rho_m"])
    robot_radius = float(row["robot_radius_m"])
    goal_radius = float(row["goal_radius_m"])
    planned_start = (float(row["start_x"]), float(row["start_y"]), float(row["start_theta"]))
    goal = (float(row["goal_x"]), float(row["goal_y"]))

    trace: ExecutionTrace | None = None
    note = "not collected yet"
    if run_dir is not None:
        try:
            trace = trace_execution(run_dir, config)
        except (OSError, ValueError) as error:
            note = f"no usable recording: {error}"

    figure, axis = plt.subplots(figsize=(9, 8))
    x_min, x_max = config.WORKSPACE_X_MIN_M, config.WORKSPACE_X_MAX_M
    y_min, y_max = config.WORKSPACE_Y_MIN_M, config.WORKSPACE_Y_MAX_M
    axis.add_patch(Rectangle((x_min, y_min), x_max-x_min, y_max-y_min, fill=False,
                             edgecolor="black", linewidth=1.2, label="Workspace"))
    for obstacle in json.loads(row["obstacles"]):
        center = (obstacle["x"], obstacle["y"])
        axis.add_patch(Circle(center, obstacle["radius"], color="dimgray", alpha=0.75,
                              label="Obstacle (virtual)"))
        # Inner ring: where the robot center would touch the obstacle.
        # Outer ring: the extra margin rho the planner also keeps clear.
        axis.add_patch(Circle(center, obstacle["radius"] + robot_radius, fill=False,
                              edgecolor="dimgray", linestyle="--", linewidth=1.2,
                              label="Obstacle + robot radius"))
        axis.add_patch(Circle(center, obstacle["radius"] + robot_radius + rho, fill=False,
                              edgecolor="darkorange", linestyle="--", linewidth=1.2,
                              label="Obstacle + robot radius + ρ (planner limit)"))
    axis.add_patch(Circle(goal, goal_radius, color="gold", alpha=0.25, label="Goal region"))
    axis.add_patch(Circle(goal, goal_radius - rho, fill=False, edgecolor="darkgoldenrod",
                          linestyle="--", linewidth=1.2, label="Goal shrunk by ρ"))
    axis.add_patch(Circle(planned_start[:2], config.START_REGION_RADIUS_M, color="green",
                          alpha=0.10, label="Start region"))

    planned, planned_ends = _dense_path(load_candidate_csv(plan_path), planned_start)
    axis.plot(*zip(*planned), "b--", linewidth=2, label="Planned RRT path")
    axis.plot(*zip(*planned_ends), "b.", markersize=6)
    axis.plot(*planned_start[:2], "o", markerfacecolor="none", markeredgecolor="green",
              markeredgewidth=2, markersize=10, label="Planned start")

    if trace is not None:
        nominal, _ = _dense_path(trace.commands, trace.initial_pose)
        axis.plot(*zip(*nominal), color="tab:purple", linestyle=":", linewidth=1.8,
                  label="Model from measured start")
        axis.plot(*zip(*trace.actual_path), color="red", linewidth=2,
                  label="Real trajectory (mocap)")
        errors = trace.errors
        worst = max(range(len(errors)), key=errors.__getitem__)
        nominal_point = trace.nominal_at_boundaries[worst][:2]
        actual_point = trace.actual_at_boundaries[worst]
        axis.plot(*zip(nominal_point, actual_point), color="black", linewidth=2.5,
                  label=f"Largest deviation E = {errors[worst]:.3f} m")
        axis.plot(*trace.initial_pose[:2], "o", color="green", markersize=10,
                  label="Measured start")
        axis.plot(*trace.actual_path[-1], "o", color="red", markersize=8, label="Real end")
        title = f"{run_id}: planned RRT path and real trajectory (E = {errors[worst]:.3f} m)"
    else:
        title = f"{run_id}: planned RRT path ({note})"

    axis.plot(*planned[-1], "o", color="gold", markeredgecolor="darkgoldenrod",
              markersize=10, label="Planned end")
    axis.set(xlabel="x (m)", ylabel="y (m)", title=title)
    axis.set_aspect("equal", adjustable="box")
    pad = 0.3
    axis.set_xlim(x_min - pad, x_max + pad)
    axis.set_ylim(y_min - pad, y_max + pad)
    axis.grid(alpha=0.3)
    handles, labels = axis.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    # Legend outside the map so it can never hide an obstacle or a path.
    axis.legend(unique.values(), unique.keys(), loc="upper left",
                bbox_to_anchor=(1.02, 1.0), borderaxespad=0, fontsize=9)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150, bbox_inches="tight")
    plt.close(figure)
    return output


def plot_instance(instance_id: str) -> list[Path]:
    """Draw every candidate of one instance into analysis/plots/<instance>_<candidate>.png."""
    row = _problem_row(instance_id)
    plans = sorted((config.PHASE1_ROOT / "plans" / instance_id).glob("C*.csv"))
    if not plans:
        raise ValueError(f"{instance_id} has no planned candidates (status {row['status']})")
    return [plot_candidate(row, plan, _find_run(f"{instance_id}_{plan.stem}"),
                           plot_dir() / f"{instance_id}_{plan.stem}.png")
            for plan in plans]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("instances", nargs="+", help="Instance IDs such as I047, or 'all'")
    args = parser.parse_args()
    instances = args.instances
    if instances == ["all"]:
        with config.PROBLEMS_CSV.open(newline="", encoding="utf-8") as source:
            instances = [row["instance"] for row in csv.DictReader(source)
                         if row["status"] != "empty"]
    for instance_id in instances:
        paths = plot_instance(instance_id)
        print(f"{instance_id}: saved {len(paths)} plot(s) in {plot_dir()}")


if __name__ == "__main__":
    main()
