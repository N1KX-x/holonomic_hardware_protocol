#!/usr/bin/env python3
"""Collect random-command runs that train a learned dynamics model.

Each call collects one trial and saves it as the next free trial_NNN, so the
same command is repeated for every trial:

    python3 collect_training.py              # preview the next trial; no motion
    python3 collect_training.py --execute    # read the robot pose, plan, and run it

Every trial is TRAINING_SEGMENTS commands of DT_S seconds. Trials come in
blocks of TRAINING_MIXED_PER_BLOCK mixed runs plus one vx-only, one vy-only and
one omega-only run. Commands are drawn segment by segment from the measured
start pose so the nominal path stays TRAINING_PLAN_MARGIN_M inside the
workspace. A failed or interrupted trial keeps its files under
trial_NNN_discarded, and its number is never reused.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import re
from datetime import datetime
from pathlib import Path

import config
from core import Command, nominal_step, process_execution, validate_commands, write_candidate_csv


PLAN_DIR = config.TRAINING_ROOT / "plans"
RAW_DIR = config.TRAINING_ROOT / "raw"
TRIALS_CSV = config.TRAINING_ROOT / "trials.csv"
NOTES_FILE = config.TRAINING_ROOT / "notes.txt"
TRIAL_PATTERN = re.compile(r"^trial_(\d+)(?:_discarded)?$")
SINGLE_AXIS = ("vx_only", "vy_only", "omega_only")
SEGMENT_TRIES = 200
PLAN_TRIES = 50
PATH_CHECK_STEP_S = 0.1
TRIAL_FIELDS = ["trial", "motion_type", "status", "segments_sent", "max_deviation_m",
                "start_x_m", "start_y_m", "start_theta_rad", "finished_at", "note"]


def next_trial_number(raw_dir: Path | None = None, plan_dir: Path | None = None) -> int:
    """One more than the highest trial number in raw (discarded included) or plans."""
    raw_dir = RAW_DIR if raw_dir is None else raw_dir
    plan_dir = PLAN_DIR if plan_dir is None else plan_dir
    numbers = []
    for path in (*(raw_dir.iterdir() if raw_dir.is_dir() else ()),
                 *(plan_dir.glob("trial_*.csv") if plan_dir.is_dir() else ())):
        match = TRIAL_PATTERN.fullmatch(path.stem if path.suffix == ".csv" else path.name)
        if match:
            numbers.append(int(match.group(1)))
    return max(numbers, default=0) + 1


def motion_type(number: int) -> str:
    """Type of trial `number`, from a seeded shuffle of its block so every block has the same mix."""
    block = ["mixed"] * config.TRAINING_MIXED_PER_BLOCK + list(SINGLE_AXIS)
    random.Random(config.TRAINING_RANDOM_SEED * 1000 + (number - 1) // len(block)).shuffle(block)
    return block[(number - 1) % len(block)]


def _translation_limit(component_max: float) -> float:
    combined = config.TRANSLATIONAL_SPEED_MAX_MPS
    return component_max if combined is None else min(component_max, combined)


def _sample_command(kind: str, k: int, rng: random.Random) -> Command:
    vx = vy = omega = 0.0
    if kind == "vx_only":
        limit = _translation_limit(config.VX_MAX_MPS)
        vx = rng.uniform(-limit, limit)
    elif kind == "vy_only":
        limit = _translation_limit(config.VY_MAX_MPS)
        vy = rng.uniform(-limit, limit)
    elif kind == "omega_only":
        omega = rng.uniform(-config.OMEGA_MAX_RADPS, config.OMEGA_MAX_RADPS)
    else:
        combined = config.TRANSLATIONAL_SPEED_MAX_MPS
        for _ in range(1000):
            vx = rng.uniform(-config.VX_MAX_MPS, config.VX_MAX_MPS)
            vy = rng.uniform(-config.VY_MAX_MPS, config.VY_MAX_MPS)
            if combined is None or math.hypot(vx, vy) <= combined:
                break
        else:
            raise RuntimeError("Could not sample a mixed command within TRANSLATIONAL_SPEED_MAX_MPS")
        omega = rng.uniform(-config.OMEGA_MAX_RADPS, config.OMEGA_MAX_RADPS)
    return Command(k, vx, vy, omega, config.DT_S)


def _inside(x: float, y: float) -> bool:
    margin = config.TRAINING_PLAN_MARGIN_M
    return (config.WORKSPACE_X_MIN_M + margin <= x <= config.WORKSPACE_X_MAX_M - margin
            and config.WORKSPACE_Y_MIN_M + margin <= y <= config.WORKSPACE_Y_MAX_M - margin)


def _segment_stays_inside(pose, command: Command) -> bool:
    steps = max(1, math.ceil(command.dt / PATH_CHECK_STEP_S))
    return all(_inside(*nominal_step(*pose, command, config, duration=command.dt * i / steps)[:2])
               for i in range(1, steps + 1))


def plan_trial(kind: str, start_pose, rng: random.Random) -> list[Command]:
    """Random commands of one type whose nominal path stays inside the workspace margin."""
    if not _inside(*start_pose[:2]):
        raise ValueError(f"Robot at x={start_pose[0]:.2f}, y={start_pose[1]:.2f} is within "
                         f"{config.TRAINING_PLAN_MARGIN_M} m of the workspace edge; "
                         "move it toward the middle")
    for _ in range(PLAN_TRIES):
        pose, commands = start_pose, []
        for k in range(config.TRAINING_SEGMENTS):
            for _ in range(SEGMENT_TRIES):
                command = _sample_command(kind, k, rng)
                if _segment_stays_inside(pose, command):
                    break
            else:
                break  # dead end near an edge: draw the whole plan again
            commands.append(command)
            pose = nominal_step(*pose, command, config)
        if len(commands) == config.TRAINING_SEGMENTS:
            return validate_commands(commands, config, require_main_horizon=False)
    raise RuntimeError("Could not draw a plan that stays inside the workspace; "
                       "move the robot toward the middle")


def _segments_sent(run_dir: Path) -> int:
    try:
        with (run_dir / "commands.csv").open(newline="", encoding="utf-8") as source:
            return sum(1 for _ in csv.DictReader(source))
    except OSError:
        return 0


def _log_trial(name, kind, pose, status, segments_sent, deviation, note) -> None:
    exists = TRIALS_CSV.exists()
    with TRIALS_CSV.open("a", newline="", encoding="utf-8") as target:
        writer = csv.DictWriter(target, fieldnames=TRIAL_FIELDS)
        if not exists:
            writer.writeheader()
        writer.writerow({"trial": name, "motion_type": kind, "status": status,
                         "segments_sent": segments_sent, "max_deviation_m": deviation,
                         "start_x_m": pose[0], "start_y_m": pose[1], "start_theta_rad": pose[2],
                         "finished_at": datetime.now().isoformat(timespec="seconds"),
                         "note": note})


def _print_plan(name: str, kind: str, pose, commands: list[Command]) -> None:
    print(f"{name}: {kind}, {len(commands)} x {config.DT_S:g} s, from x={pose[0]:.2f} m, "
          f"y={pose[1]:.2f} m, facing {math.degrees(pose[2]):.0f} deg")
    print("    k      vx      vy   omega")
    for command in commands:
        print(f"  {command.k:3d}  {command.vx:+.3f}  {command.vy:+.3f}  {command.omega:+.3f}")
    end = pose
    for command in commands:
        end = nominal_step(*end, command, config)
    print(f"  nominal end: x={end[0]:.2f} m, y={end[1]:.2f} m")


def collect(execute: bool, preview_pose=None) -> None:
    config.validate()
    number = next_trial_number()
    name = f"trial_{number:03d}"
    kind = motion_type(number)
    rng = random.Random(config.TRAINING_RANDOM_SEED + number)
    if not execute:
        pose = preview_pose or ((config.WORKSPACE_X_MIN_M + config.WORKSPACE_X_MAX_M) / 2,
                                (config.WORKSPACE_Y_MIN_M + config.WORKSPACE_Y_MAX_M) / 2, 0.0)
        _print_plan(name, kind, pose, plan_trial(kind, pose, rng))
        print("Preview only. The real plan is drawn from the robot's measured pose. "
              "Add --execute to run it.")
        return
    if not config.ALLOW_HARDWARE_EXECUTION:
        raise RuntimeError("Hardware execution is disabled in config.py")
    if config.EDGE_STOP_MARGIN_M is None:
        print("Warning: EDGE_STOP_MARGIN_M is None, so nothing stops the robot at the "
              "workspace edge. Keep a hand on the e-stop.")
    if config.REQUIRE_OPERATOR_CONFIRMATION:
        answer = input(f"Execute {name} ({kind}) on the robot? Type {name}: ").strip()
        if answer != name:
            raise RuntimeError("Operator confirmation did not match")
    from hardware.collector import collect as collect_hardware
    from hardware.start_check import read_pose

    pose = read_pose()
    if pose is None:
        raise RuntimeError("Mocap is not tracking the robot")
    commands = plan_trial(kind, pose, rng)
    _print_plan(name, kind, pose, commands)
    plan_path = PLAN_DIR / f"{name}.csv"
    run_dir = RAW_DIR / name
    write_candidate_csv(plan_path, commands)
    try:
        collect_hardware(plan_path, run_dir)
    except BaseException as error:
        reason = str(error) or type(error).__name__
        discarded = run_dir.with_name(f"{name}_discarded")
        if run_dir.exists() and not discarded.exists():
            run_dir.rename(discarded)
        with NOTES_FILE.open("a", encoding="utf-8") as notes:
            notes.write(f"{name}: discarded: {reason}\n")
        _log_trial(name, kind, pose, "discarded", _segments_sent(discarded), "", reason)
        raise
    try:
        deviation = process_execution(run_dir, kind, config).max_deviation_m
        note = ""
        print(f"{name} saved. E with the current nominal model: {deviation:.3f} m")
    except (OSError, ValueError) as error:
        deviation, note = "", f"check failed: {error}"
        print(f"{name} saved, but its check failed: {error}")
    _log_trial(name, kind, pose, "ok", len(commands), deviation, note)
    print(f"Next: trial_{number + 1:03d} ({motion_type(number + 1)}). "
          "Bring the robot back near the middle if it ended close to an edge.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--execute", action="store_true",
                        help="run the next trial on the robot (default: preview only)")
    parser.add_argument("--pose", type=float, nargs=3, metavar=("X", "Y", "THETA"),
                        help="start pose for the preview (default: workspace middle, facing +x)")
    args = parser.parse_args()
    try:
        collect(args.execute, tuple(args.pose) if args.pose else None)
    except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
        reason = str(error) or "Operator interrupted the trial"
        parser.exit(1, f"Error: {reason}. Any collected raw data is kept.\n")


if __name__ == "__main__":
    main()
