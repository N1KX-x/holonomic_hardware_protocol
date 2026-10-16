#!/usr/bin/env python3
"""Collect one Phase 1 trial per call: an RRT plan with the frozen rho, run from wherever the robot stands.

    python3 main.py              # preview a plan; no motion, nothing saved
    python3 main.py --execute    # read the robot pose, plan, run it, and save it
    python3 main.py --validate   # only check config.py against the Phase 0 result

Trials are numbered like Phase 0 pilots (trial_001, trial_002, ...). There is
no fixed number of trials and no reset: walk the robot back yourself.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import re
import shutil
from pathlib import Path

import config
from core import Command, nominal_rollout, trace_execution, validate_commands, write_candidate_csv
from planner import Problem, plan_candidate, sample_trial_problem


TRIAL_PATTERN = re.compile(r"^trial_(\d+)$")
# Left by the earlier Phase 1 (fixed pool of instances and candidates).
OLD_PHASE1_MARKER = "problems.csv"


def load_phase0() -> float:
    if not config.PHASE0_RESULTS_CSV.is_file():
        raise FileNotFoundError(
            f"Run phase0analyze.py first; missing {config.PHASE0_RESULTS_CSV}"
        )
    with config.PHASE0_RESULTS_CSV.open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    if len(rows) != 1 or rows[0]["status"] != "ready":
        raise RuntimeError("Phase 0 status is not ready")
    row = rows[0]
    if float(row["dt_s"]) != config.DT_S or int(row["t_segments"]) != config.T_SEGMENTS:
        raise RuntimeError("DT_S or T_SEGMENTS changed after Phase 0")
    if float(row["goal_radius_m"]) != config.GOAL_RADIUS_M:
        raise RuntimeError("GOAL_RADIUS_M changed after Phase 0")
    frozen = {
        "vx_max_mps": config.VX_MAX_MPS,
        "vy_max_mps": config.VY_MAX_MPS,
        "omega_max_radps": config.OMEGA_MAX_RADPS,
        "yaw_command_sign": config.POSITIVE_YAW_COMMAND_SIGN,
        "lateral_command_sign": config.POSITIVE_LATERAL_COMMAND_SIGN,
        "model_gain_vx_pos": config.MODEL_GAIN_VX_POS,
        "model_gain_vx_neg": config.MODEL_GAIN_VX_NEG,
        "model_gain_vy_pos": config.MODEL_GAIN_VY_POS,
        "model_gain_vy_neg": config.MODEL_GAIN_VY_NEG,
        "model_gain_omega_pos": config.MODEL_GAIN_OMEGA_POS,
        "model_gain_omega_neg": config.MODEL_GAIN_OMEGA_NEG,
        "model_offset_vx_pos_m": config.MODEL_OFFSET_VX_POS_M,
        "model_offset_vy_pos_m": config.MODEL_OFFSET_VY_POS_M,
        "model_offset_vy_neg_m": config.MODEL_OFFSET_VY_NEG_M,
        "model_offset_omega_pos_rad": config.MODEL_OFFSET_OMEGA_POS_RAD,
        "model_offset_omega_neg_rad": config.MODEL_OFFSET_OMEGA_NEG_RAD,
        "model_turn_lag_s": config.MODEL_TURN_LAG_S,
        "model_turn_lag_after_walk_s": config.MODEL_TURN_LAG_AFTER_WALK_S,
        "model_walk_yaw_rad_per_m": config.MODEL_WALK_YAW_RAD_PER_M,
        "robot_radius_m": config.ROBOT_RADIUS_M,
        "workspace_x_min_m": config.WORKSPACE_X_MIN_M,
        "workspace_x_max_m": config.WORKSPACE_X_MAX_M,
        "workspace_y_min_m": config.WORKSPACE_Y_MIN_M,
        "workspace_y_max_m": config.WORKSPACE_Y_MAX_M,
    }
    changed = [name for name, value in frozen.items()
               if name not in row or float(row[name]) != value]
    if changed:
        raise RuntimeError(f"Configuration changed after Phase 0: {', '.join(changed)}")
    calibration = (float(row["yaw_offset_rad"]), float(row["marker_to_body_x_m"]),
                   float(row["marker_to_body_y_m"]))
    current = (config.YAW_OFFSET_RAD, config.MARKER_TO_BODY_X_M, config.MARKER_TO_BODY_Y_M)
    if calibration != current:
        raise RuntimeError("Calibration in config.py differs from the Phase 0 result")
    rho = float(row["rho_m"])
    if not 0.0 < rho < config.GOAL_RADIUS_M:
        raise RuntimeError("Phase 0 rho is not viable")
    return rho


def trial_dirs(root: Path | None = None) -> list[Path]:
    """Saved trial folders in number order."""
    root = config.PHASE1_ROOT if root is None else root
    found = [path for path in (root.iterdir() if root.is_dir() else ())
             if path.is_dir() and TRIAL_PATTERN.fullmatch(path.name)]
    return sorted(found, key=lambda path: int(TRIAL_PATTERN.fullmatch(path.name).group(1)))


def next_trial_number(root: Path | None = None) -> int:
    """One more than the highest saved trial number."""
    saved = trial_dirs(root)
    return int(TRIAL_PATTERN.fullmatch(saved[-1].name).group(1)) + 1 if saved else 1


def reject_old_phase1(root: Path | None = None) -> None:
    root = config.PHASE1_ROOT if root is None else root
    if (root / OLD_PHASE1_MARKER).exists():
        raise RuntimeError(f"{root} holds runs from the earlier Phase 1. Rename that "
                           "folder (for example to phase1_old) and run again")


def plan_trial(number: int, pose, seed: int, rho: float) -> tuple[Problem, list[Command]]:
    """RRT plan from `pose` to the goal region shrunk by rho; raises if this pose has no plan."""
    x, y, _ = pose
    if not (config.WORKSPACE_X_MIN_M <= x <= config.WORKSPACE_X_MAX_M
            and config.WORKSPACE_Y_MIN_M <= y <= config.WORKSPACE_Y_MAX_M):
        raise RuntimeError(f"Robot at x={x:.2f}, y={y:.2f} is outside the boundary")
    distance = math.hypot(config.GOAL_X_M - x, config.GOAL_Y_M - y)
    if distance <= config.GOAL_RADIUS_M:
        raise RuntimeError(f"Robot is already {distance:.2f} m from the goal, inside its "
                           f"{config.GOAL_RADIUS_M:g} m radius; start further away")
    problem = sample_trial_problem(number, seed, pose, rho)
    commands = plan_candidate(problem, seed + 1, rho)
    if commands is None:
        raise RuntimeError("RRT found no path from this pose")
    return problem, validate_commands(commands, config)


def _problem_record(name: str, problem: Problem, rho: float) -> dict:
    return {
        "trial": name,
        "problem_seed": problem.problem_seed,
        "rrt_seed": problem.problem_seed + 1,
        "start": {"x": problem.start_x, "y": problem.start_y, "theta": problem.start_theta},
        "goal": {"x": problem.goal_x, "y": problem.goal_y,
                 "radius_m": config.GOAL_RADIUS_M},
        "obstacles": [{"x": o.x, "y": o.y, "radius": o.radius} for o in problem.obstacles],
        "robot_radius_m": config.PHASE1_ROBOT_RADIUS_M,
        "rho_m": rho,
        "workspace": {"x_min": config.WORKSPACE_X_MIN_M, "x_max": config.WORKSPACE_X_MAX_M,
                      "y_min": config.WORKSPACE_Y_MIN_M, "y_max": config.WORKSPACE_Y_MAX_M},
        "dt_s": config.DT_S,
        "t_segments": config.T_SEGMENTS,
    }


def plot_trial(trial_dir: Path) -> Path:
    """Draw one saved trial, with its rho rings, into its own folder."""
    from plot_phase1 import plot_candidate

    with (trial_dir / "problem.json").open(encoding="utf-8") as source:
        record = json.load(source)
    row = {
        "start_x": record["start"]["x"], "start_y": record["start"]["y"],
        "start_theta": record["start"]["theta"],
        "goal_x": record["goal"]["x"], "goal_y": record["goal"]["y"],
        "goal_radius_m": record["goal"]["radius_m"],
        "robot_radius_m": record["robot_radius_m"],
        "obstacles": json.dumps(record["obstacles"]),
        "rho_m": record["rho_m"],
    }
    return plot_candidate(row, trial_dir / "plan.csv", trial_dir / "raw",
                          trial_dir / "trajectory.png", run_id=trial_dir.name)


def _print_plan(name: str, problem: Problem, commands: list[Command], rho: float) -> None:
    moving = [c for c in commands if (c.vx, c.vy, c.omega) != (0.0, 0.0, 0.0)]
    print(f"{name}: from x={problem.start_x:.2f} m, y={problem.start_y:.2f} m, facing "
          f"{math.degrees(problem.start_theta):.0f} deg, to within "
          f"{config.GOAL_RADIUS_M - rho:.2f} m of ({problem.goal_x:.2f}, {problem.goal_y:.2f})")
    print(f"  {len(moving)} moving segments of {config.DT_S:g} s "
          f"(then standing still until {len(commands) * config.DT_S:g} s), "
          f"{len(problem.obstacles)} virtual obstacles inflated by "
          f"{config.PHASE1_ROBOT_RADIUS_M:g} + rho {rho:.3f} m")
    print("    k      vx      vy   omega")
    for command in moving:
        print(f"  {command.k:3d}  {command.vx:+.3f}  {command.vy:+.3f}  {command.omega:+.3f}")
    end = nominal_rollout(commands, (problem.start_x, problem.start_y, problem.start_theta),
                          config)[-1]
    print(f"  nominal end: x={end[0]:.2f} m, y={end[1]:.2f} m")


def collect(execute: bool, preview_pose=None) -> None:
    config.validate()
    rho = load_phase0()
    reject_old_phase1()
    number = next_trial_number()
    name = f"trial_{number:03d}"
    # A fresh seed every call, saved with the trial: a failed call must not
    # repeat the same obstacles and the same failed search.
    seed = random.SystemRandom().randrange(1, 2**31)
    if not execute:
        pose = preview_pose or (config.START_X_M, config.START_Y_M, 0.0)
        _print_plan(name, *plan_trial(number, pose, seed, rho), rho)
        print("Preview only. The real plan starts from the robot's measured pose. "
              "Add --execute to run it.")
        return
    if not config.ALLOW_HARDWARE_EXECUTION:
        raise RuntimeError("Hardware execution is disabled in config.py")
    if config.EDGE_STOP_MARGIN_M is not None:
        print(f"Note: EDGE_STOP_MARGIN_M = {config.EDGE_STOP_MARGIN_M} stops (and fails) "
              "any trial that comes this close to the boundary.")
    from hardware.collector import collect as collect_hardware
    from hardware.start_check import read_pose

    pose = read_pose()
    if pose is None:
        raise RuntimeError("Mocap is not tracking the robot")
    problem, commands = plan_trial(number, pose, seed, rho)
    _print_plan(name, problem, commands, rho)
    if config.REQUIRE_OPERATOR_CONFIRMATION:
        answer = input(f"Execute {name} on the robot? Type {name}: ").strip()
        if answer != name:
            raise RuntimeError("Operator confirmation did not match")
    trial_dir = config.PHASE1_ROOT / name
    trial_dir.mkdir(parents=True)
    try:
        write_candidate_csv(trial_dir / "plan.csv", commands)
        with (trial_dir / "problem.json").open("x", encoding="utf-8") as target:
            json.dump(_problem_record(name, problem, rho), target, indent=2)
        collect_hardware(trial_dir / "plan.csv", trial_dir / "raw")
        # A run the analysis could not use (tracking gap, mocap cut short) has failed.
        deviation = max(trace_execution(trial_dir / "raw", config).errors)
    except BaseException:
        shutil.rmtree(trial_dir, ignore_errors=True)
        raise
    print(f"{name} saved in {trial_dir}. E = {deviation:.3f} m (rho = {rho:.3f} m)")
    # The plot is optional: a missing matplotlib must not discard a good trial.
    try:
        print(f"Trajectory plot: {plot_trial(trial_dir)}")
    except Exception as error:
        print(f"Trajectory plot skipped: {error}")
    print(f"{number} trial(s) so far. Walk the robot back yourself before the next one. "
          "Run python3 analyze_phase1.py for the summary.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--execute", action="store_true",
                        help="run the trial on the robot (default: preview only)")
    parser.add_argument("--validate", action="store_true",
                        help="only check config.py against the Phase 0 result")
    parser.add_argument("--pose", type=float, nargs=3, metavar=("X", "Y", "THETA"),
                        help="start pose for the preview (default: START_X_M, START_Y_M, facing +x)")
    args = parser.parse_args()
    try:
        if args.validate:
            config.validate()
            print(f"Configuration and Phase 0 result are valid; rho={load_phase0():.6f} m")
            return
        collect(args.execute, tuple(args.pose) if args.pose else None)
    except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
        reason = str(error) or "Operator interrupted the run"
        parser.exit(1, f"Failed: {reason}. Nothing was saved; the next run uses the "
                       "same trial number.\n")


if __name__ == "__main__":
    main()
