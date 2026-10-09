#!/usr/bin/env python3
"""Hardware-free one-candidate demonstration using perfect synthetic mocap."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle as CirclePatch

import config
from core import nominal_step, process_execution
from planner import plan_candidate, sample_problem


OUTPUT = config.PROJECT_ROOT / "simulation_output" / "candidate_001"
RHO_M = 0.15
MOCAP_HZ = 120.0
T0 = 1000.0


def state_at(commands, initial, elapsed):
    state = initial
    remaining = max(0.0, elapsed)
    for command in commands:
        duration = min(command.dt, remaining)
        state = nominal_step(*state, command, config, duration)
        remaining -= duration
        if remaining <= 1e-12:
            break
    return state


def run() -> Path:
    if OUTPUT.exists():
        raise FileExistsError(f"Simulation output already exists: {OUTPUT}")
    raw = OUTPUT / "raw"
    raw.mkdir(parents=True)
    problem_seed = config.MASTER_RANDOM_SEED + 10000
    candidate_seed = problem_seed + 1
    problem = sample_problem(1, problem_seed, RHO_M)
    commands = plan_candidate(problem, candidate_seed, RHO_M)
    if commands is None:
        raise RuntimeError("Deterministic demonstration candidate was not found")
    initial = (problem.start_x, problem.start_y, problem.start_theta)
    total_time = sum(command.dt for command in commands)

    with (OUTPUT / "problem.json").open("x", encoding="utf-8") as target:
        json.dump({
            "problem_seed": problem_seed,
            "candidate_seed": candidate_seed,
            "rho_m": RHO_M,
            "robot_radius_m": config.ROBOT_RADIUS_M,
            "start": initial,
            "goal": {"x": problem.goal_x, "y": problem.goal_y,
                     "radius_m": config.GOAL_RADIUS_M,
                     "shrunk_radius_m": config.GOAL_RADIUS_M-RHO_M},
            "obstacles": [obstacle.__dict__ for obstacle in problem.obstacles],
        }, target, indent=2)

    with (OUTPUT / "candidate.csv").open("x", newline="", encoding="utf-8") as target, \
         (raw / "commands.csv").open("x", newline="", encoding="utf-8") as sent:
        candidate_writer = csv.writer(target)
        sent_writer = csv.writer(sent)
        candidate_writer.writerow(["k", "vx", "vy", "omega", "dt"])
        sent_writer.writerow(["k", "t_send", "vx", "vy", "omega", "dt"])
        for command in commands:
            candidate_writer.writerow([command.k, command.vx, command.vy,
                                       command.omega, command.dt])
            sent_writer.writerow([command.k, T0+command.k*command.dt,
                                  command.vx, command.vy, command.omega, command.dt])

    sample_start = T0-config.MOCAP_LEAD_TIME_S
    sample_end = T0+total_time+config.MOCAP_TRAIL_TIME_S
    sample_count = round((sample_end-sample_start)*MOCAP_HZ)+1
    mocap_rows = []
    for index in range(sample_count):
        timestamp = sample_start + index/MOCAP_HZ
        elapsed = min(total_time, max(0.0, timestamp-T0))
        x, y, theta = state_at(commands, initial, elapsed)
        mocap_rows.append((timestamp, x, y, theta, 1))
    with (raw / "mocap.csv").open("x", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["t", "x", "y", "theta_raw", "tracked"])
        writer.writerows(mocap_rows)
    with (raw / "publish_log.csv").open("x", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["t", "event", "k", "vx", "vy", "omega"])
        for command in commands:
            writer.writerow([T0+command.k*command.dt, "segment_start", command.k,
                             command.vx, command.vy, command.omega])

    metric = process_execution(raw, "simulation", config)
    with (OUTPUT / "summary.csv").open("x", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["run_id", "max_deviation_m", "commanded_distance_m",
                         "actual_distance_m", "initial_x_m", "initial_y_m",
                         "initial_theta_rad"])
        writer.writerow([metric.run_id, metric.max_deviation_m,
                         metric.commanded_distance_m, metric.actual_distance_m,
                         metric.initial_x_m, metric.initial_y_m,
                         metric.initial_theta_rad])

    dense = []
    for index in range(round(total_time*MOCAP_HZ)+1):
        elapsed = index/MOCAP_HZ
        dense.append((T0+elapsed, *state_at(commands, initial, elapsed)))
    with (OUTPUT / "nominal_path.csv").open("x", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["t", "x", "y", "theta"])
        writer.writerows(dense)

    figure, axis = plt.subplots(figsize=(8, 7))
    for obstacle in problem.obstacles:
        axis.add_patch(CirclePatch((obstacle.x, obstacle.y),
                                  obstacle.radius+config.ROBOT_RADIUS_M+RHO_M,
                                  color="tab:red", alpha=0.15))
        axis.add_patch(CirclePatch((obstacle.x, obstacle.y), obstacle.radius,
                                  color="tab:red", alpha=0.55, label="Virtual obstacle"))
    axis.add_patch(CirclePatch((problem.goal_x, problem.goal_y), config.GOAL_RADIUS_M,
                              color="gold", alpha=0.18, label="Goal region"))
    axis.add_patch(CirclePatch((problem.goal_x, problem.goal_y),
                              config.GOAL_RADIUS_M-RHO_M, fill=False,
                              edgecolor="darkgoldenrod", linewidth=2,
                              linestyle="--", label="Shrunk goal"))
    axis.plot([row[1] for row in dense], [row[2] for row in dense],
              color="tab:blue", linewidth=3, label="Nominal trajectory")
    shown = [row for index, row in enumerate(mocap_rows)
             if index % 24 == 0 and T0 <= row[0] <= T0+total_time]
    axis.scatter([row[1] for row in shown], [row[2] for row in shown],
                 s=15, facecolors="none", edgecolors="tab:orange",
                 label="Synthetic mocap")
    boundaries = [state_at(commands, initial, k*config.DT_S)
                  for k in range(len(commands)+1)]
    axis.scatter([state[0] for state in boundaries], [state[1] for state in boundaries],
                 s=28, color="tab:blue", zorder=5, label="Segment boundaries")
    axis.scatter([initial[0]], [initial[1]], marker="s", s=70,
                 color="black", zorder=6, label="Measured start")
    axis.set(xlabel="Mocap world x (m)", ylabel="Mocap world y (m)",
             title=f"One candidate: nominal and perfect synthetic mocap\nE = {metric.max_deviation_m:.3e} m")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlim(config.WORKSPACE_X_MIN_M, config.WORKSPACE_X_MAX_M)
    axis.set_ylim(config.WORKSPACE_Y_MIN_M, config.WORKSPACE_Y_MAX_M)
    axis.grid(alpha=0.25)
    handles, labels = axis.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    axis.legend(unique.values(), unique.keys(), loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(OUTPUT / "trajectory.png", dpi=180)
    plt.close(figure)
    return OUTPUT


if __name__ == "__main__":
    print(run())
