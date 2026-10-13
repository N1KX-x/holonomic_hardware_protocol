#!/usr/bin/env python3
"""Collect one Phase 0 pilot per call: an RRT plan run from wherever the robot stands.

    python3 phase0.py              # preview a plan; no motion, nothing saved
    python3 phase0.py --execute    # read the robot pose, plan, run it, and save it
"""

from __future__ import annotations

import argparse
import json
import math
import random
import re
import shutil
from pathlib import Path

import config
from core import Command, nominal_step, trace_execution, validate_commands, write_candidate_csv
from planner import Problem, plan_pilot, sample_pilot_problem


PILOT_PATTERN = re.compile(r"^pilot_(\d+)$")
# Left by the earlier Phase 0 (fixed list of random-command pilots).
OLD_PHASE0_MARKER = "plan_manifest.csv"


def pilot_dirs(root: Path | None = None) -> list[Path]:
    """Saved pilot folders in number order."""
    root = config.PHASE0_ROOT if root is None else root
    found = [path for path in (root.iterdir() if root.is_dir() else ())
             if path.is_dir() and PILOT_PATTERN.fullmatch(path.name)]
    return sorted(found, key=lambda path: int(PILOT_PATTERN.fullmatch(path.name).group(1)))


def next_pilot_number(root: Path | None = None) -> int:
    """One more than the highest saved pilot number."""
    saved = pilot_dirs(root)
    return int(PILOT_PATTERN.fullmatch(saved[-1].name).group(1)) + 1 if saved else 1


def reject_old_phase0(root: Path | None = None) -> None:
    root = config.PHASE0_ROOT if root is None else root
    if (root / OLD_PHASE0_MARKER).exists():
        raise RuntimeError(f"{root} holds runs from the earlier Phase 0. Rename that "
                           "folder (for example to phase0_old) and run again")


def plan_pilot_run(number: int, pose, seed: int) -> tuple[Problem, list[Command]]:
    """RRT plan from `pose` to the goal region; raises if this pose has no plan."""
    x, y, _ = pose
    if not (config.WORKSPACE_X_MIN_M <= x <= config.WORKSPACE_X_MAX_M
            and config.WORKSPACE_Y_MIN_M <= y <= config.WORKSPACE_Y_MAX_M):
        raise RuntimeError(f"Robot at x={x:.2f}, y={y:.2f} is outside the boundary")
    distance = math.hypot(config.GOAL_X_M - x, config.GOAL_Y_M - y)
    if distance <= config.GOAL_RADIUS_M:
        raise RuntimeError(f"Robot is already {distance:.2f} m from the goal, inside its "
                           f"{config.GOAL_RADIUS_M:g} m radius; start further away")
    problem = sample_pilot_problem(number, seed, pose)
    commands = plan_pilot(problem, seed + 1)
    if commands is None:
        raise RuntimeError("RRT found no path from this pose")
    return problem, validate_commands(commands, config)


def _problem_record(name: str, problem: Problem) -> dict:
    return {
        "pilot": name,
        "problem_seed": problem.problem_seed,
        "rrt_seed": problem.problem_seed + 1,
        "start": {"x": problem.start_x, "y": problem.start_y, "theta": problem.start_theta},
        "goal": {"x": problem.goal_x, "y": problem.goal_y,
                 "radius_m": config.GOAL_RADIUS_M},
        "obstacles": [{"x": o.x, "y": o.y, "radius": o.radius} for o in problem.obstacles],
        "robot_radius_m": config.ROBOT_RADIUS_M,
        "workspace": {"x_min": config.WORKSPACE_X_MIN_M, "x_max": config.WORKSPACE_X_MAX_M,
                      "y_min": config.WORKSPACE_Y_MIN_M, "y_max": config.WORKSPACE_Y_MAX_M},
        "dt_s": config.DT_S,
        "t_segments": config.T_SEGMENTS,
    }


def plot_pilot(pilot_dir: Path) -> Path:
    """Draw one saved pilot like a Phase 1 candidate, into its own folder."""
    from plot_phase1 import plot_candidate

    with (pilot_dir / "problem.json").open(encoding="utf-8") as source:
        record = json.load(source)
    row = {
        "start_x": record["start"]["x"], "start_y": record["start"]["y"],
        "start_theta": record["start"]["theta"],
        "goal_x": record["goal"]["x"], "goal_y": record["goal"]["y"],
        "goal_radius_m": record["goal"]["radius_m"],
        "robot_radius_m": record["robot_radius_m"],
        "obstacles": json.dumps(record["obstacles"]),
        "rho_m": 0.0,
    }
    return plot_candidate(row, pilot_dir / "plan.csv", pilot_dir / "raw",
                          pilot_dir / "trajectory.png", run_id=pilot_dir.name)


def _print_plan(name: str, problem: Problem, commands: list[Command]) -> None:
    moving = [c for c in commands if (c.vx, c.vy, c.omega) != (0.0, 0.0, 0.0)]
    print(f"{name}: from x={problem.start_x:.2f} m, y={problem.start_y:.2f} m, facing "
          f"{math.degrees(problem.start_theta):.0f} deg, to within "
          f"{config.GOAL_RADIUS_M:g} m of ({problem.goal_x:.2f}, {problem.goal_y:.2f})")
    print(f"  {len(moving)} moving segments of {config.DT_S:g} s "
          f"(then standing still until {len(commands) * config.DT_S:g} s), "
          f"{len(problem.obstacles)} virtual obstacles")
    print("    k      vx      vy   omega")
    for command in moving:
        print(f"  {command.k:3d}  {command.vx:+.3f}  {command.vy:+.3f}  {command.omega:+.3f}")
    end = (problem.start_x, problem.start_y, problem.start_theta)
    for command in commands:
        end = nominal_step(*end, command, config)
    print(f"  nominal end: x={end[0]:.2f} m, y={end[1]:.2f} m")


def collect(execute: bool, preview_pose=None) -> None:
    config.validate()
    reject_old_phase0()
    number = next_pilot_number()
    name = f"pilot_{number:03d}"
    # A fresh seed every call, saved with the pilot: a failed call must not
    # repeat the same obstacles and the same failed search.
    seed = random.SystemRandom().randrange(1, 2**31)
    if not execute:
        pose = preview_pose or (config.START_X_M, config.START_Y_M, 0.0)
        if pose[0] is None:
            pose = ((config.WORKSPACE_X_MIN_M + config.WORKSPACE_X_MAX_M) / 2,
                    (config.WORKSPACE_Y_MIN_M + config.WORKSPACE_Y_MAX_M) / 2, 0.0)
        _print_plan(name, *plan_pilot_run(number, pose, seed))
        print("Preview only. The real plan starts from the robot's measured pose. "
              "Add --execute to run it.")
        return
    if not config.ALLOW_HARDWARE_EXECUTION:
        raise RuntimeError("Hardware execution is disabled in config.py")
    if config.EDGE_STOP_MARGIN_M is not None:
        print(f"Note: EDGE_STOP_MARGIN_M = {config.EDGE_STOP_MARGIN_M} stops (and fails) "
              "any pilot that comes this close to the boundary.")
    from hardware.collector import collect as collect_hardware
    from hardware.start_check import read_pose

    pose = read_pose()
    if pose is None:
        raise RuntimeError("Mocap is not tracking the robot")
    problem, commands = plan_pilot_run(number, pose, seed)
    _print_plan(name, problem, commands)
    if config.REQUIRE_OPERATOR_CONFIRMATION:
        answer = input(f"Execute {name} on the robot? Type {name}: ").strip()
        if answer != name:
            raise RuntimeError("Operator confirmation did not match")
    pilot_dir = config.PHASE0_ROOT / name
    pilot_dir.mkdir(parents=True)
    try:
        write_candidate_csv(pilot_dir / "plan.csv", commands)
        with (pilot_dir / "problem.json").open("x", encoding="utf-8") as target:
            json.dump(_problem_record(name, problem), target, indent=2)
        collect_hardware(pilot_dir / "plan.csv", pilot_dir / "raw")
        # A run the analysis could not use (tracking gap, mocap cut short) has failed.
        deviation = max(trace_execution(pilot_dir / "raw", config).errors)
    except BaseException:
        shutil.rmtree(pilot_dir, ignore_errors=True)
        raise
    print(f"{name} saved in {pilot_dir}. E with the current model: {deviation:.3f} m")
    # The plot is optional: a missing matplotlib must not discard a good pilot.
    try:
        print(f"Trajectory plot: {plot_pilot(pilot_dir)}")
    except Exception as error:
        print(f"Trajectory plot skipped: {error}")
    print(f"{number} pilot(s) so far. Run python3 phase0analyze.py when there are enough.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--execute", action="store_true",
                        help="run the pilot on the robot (default: preview only)")
    parser.add_argument("--pose", type=float, nargs=3, metavar=("X", "Y", "THETA"),
                        help="start pose for the preview (default: the Phase 1 start, facing +x)")
    args = parser.parse_args()
    try:
        collect(args.execute, tuple(args.pose) if args.pose else None)
    except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
        reason = str(error) or "Operator interrupted the run"
        parser.exit(1, f"Failed: {reason}. Nothing was saved; the next run uses the "
                       "same pilot number.\n")


if __name__ == "__main__":
    main()
