#!/usr/bin/env python3
"""Drive the robot straight forward (vx only) and estimate YAW_OFFSET_RAD.

    python3 yaw_offset_calibration.py              # preview; no motion
    python3 yaw_offset_calibration.py --execute    # drive, record mocap, print the offset

The yaw offset is the angle from the rigid body's own heading in Motive to the
direction the robot really walks. It is estimated as the direction of the
travelled line minus the average raw marker heading. Each run is saved as the
next data/yaw_calibration/run_NNN; a failed run leaves nothing behind.
config.py is not changed: enter the printed value there yourself.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import shutil
import statistics
from pathlib import Path

import config
from core import Command, load_sent_commands, wrap_angle, write_candidate_csv


RUN_PATTERN = re.compile(r"^run_(\d+)$")


def next_run_number(root: Path | None = None) -> int:
    root = config.YAW_CALIBRATION_ROOT if root is None else root
    numbers = [int(match.group(1)) for path in (root.iterdir() if root.is_dir() else ())
               if (match := RUN_PATTERN.fullmatch(path.name))]
    return max(numbers, default=0) + 1


def estimate_yaw_offset(run_dir: Path) -> float:
    """Direction of the travelled line minus the mean raw marker heading."""
    commands = load_sent_commands(run_dir / "commands.csv")
    t0 = commands[0].t_send
    tf = t0 + sum(command.dt for command in commands)
    with (run_dir / "mocap.csv").open(newline="", encoding="utf-8") as source:
        rows = [row for row in csv.DictReader(source)
                if row["tracked"].strip().lower() in {"1", "true", "yes"}
                and t0 <= float(row["t"]) <= tf]
    if len(rows) < 2:
        raise ValueError("Not enough tracked samples for a yaw-offset estimate")
    line_heading = math.atan2(float(rows[-1]["y"])-float(rows[0]["y"]),
                              float(rows[-1]["x"])-float(rows[0]["x"]))
    sine = statistics.fmean(math.sin(float(row["theta_raw"])) for row in rows)
    cosine = statistics.fmean(math.cos(float(row["theta_raw"])) for row in rows)
    return wrap_angle(line_heading - math.atan2(sine, cosine))


def calibrate(execute: bool) -> None:
    config.validate()
    vx, duration = config.YAW_CALIBRATION_VX_MPS, config.YAW_CALIBRATION_DURATION_S
    name = f"run_{next_run_number():03d}"
    print(f"{name}: vx = {vx:g} m/s for {duration:g} s, vy = 0, omega = 0. "
          f"The robot needs about {vx * duration:.2f} m of free floor straight ahead.")
    if not execute:
        print("Preview only. Add --execute to run it.")
        return
    if not config.ALLOW_HARDWARE_EXECUTION:
        raise RuntimeError("Hardware execution is disabled in config.py")
    if config.REQUIRE_OPERATOR_CONFIRMATION:
        answer = input(f"Execute {name} on the robot? Type {name}: ").strip()
        if answer != name:
            raise RuntimeError("Operator confirmation did not match")
    from hardware.collector import collect as collect_hardware

    run_dir = config.YAW_CALIBRATION_ROOT / name
    run_dir.mkdir(parents=True)
    try:
        write_candidate_csv(run_dir / "plan.csv", [Command(0, vx, 0.0, 0.0, duration)])
        collect_hardware(run_dir / "plan.csv", run_dir / "raw")
        estimate = estimate_yaw_offset(run_dir / "raw")
    except BaseException:
        shutil.rmtree(run_dir, ignore_errors=True)
        raise
    change = wrap_angle(estimate - config.YAW_OFFSET_RAD)
    print(f"{name} saved in {run_dir}")
    print(f"Estimated yaw offset: {estimate:.4f} rad ({math.degrees(estimate):.1f} deg)")
    print(f"config.py now has:    {config.YAW_OFFSET_RAD:.4f} rad "
          f"(differs by {math.degrees(change):+.1f} deg)")
    print(f"To use it, set in config.py:  YAW_OFFSET_RAD = {estimate!r}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--execute", action="store_true",
                        help="drive the robot (default: preview only)")
    args = parser.parse_args()
    try:
        calibrate(args.execute)
    except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
        reason = str(error) or "Operator interrupted the run"
        parser.exit(1, f"Failed: {reason}. Nothing was saved.\n")


if __name__ == "__main__":
    main()
