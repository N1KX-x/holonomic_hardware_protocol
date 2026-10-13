"""Raw run files as a robot that follows a given model exactly would produce them."""

import csv
from pathlib import Path
from types import SimpleNamespace

from core import load_candidate_csv, nominal_step

T0 = 1000.0
MOCAP_HZ = 20.0
# Synthetic mocap has no marker offset, whatever the lab calibration is.
NO_CALIBRATION = dict(YAW_OFFSET_RAD=0.0, MARKER_TO_BODY_X_M=0.0, MARKER_TO_BODY_Y_M=0.0)
RAW_MODEL = dict(MODEL_GAIN_VX_POS=1.0, MODEL_GAIN_VX_NEG=1.0, MODEL_GAIN_VY_POS=1.0,
                 MODEL_GAIN_VY_NEG=1.0, MODEL_GAIN_OMEGA_POS=1.0, MODEL_GAIN_OMEGA_NEG=1.0,
                 MODEL_OFFSET_VX_POS_M=0.0, MODEL_OFFSET_VY_POS_M=0.0,
                 MODEL_OFFSET_VY_NEG_M=0.0, MODEL_OFFSET_OMEGA_POS_RAD=0.0,
                 MODEL_OFFSET_OMEGA_NEG_RAD=0.0)


def _state_at(commands, start, model, elapsed):
    state = start
    remaining = max(0.0, elapsed)
    for command in commands:
        duration = min(command.dt, remaining)
        state = nominal_step(*state, command, model, duration)
        remaining -= duration
        if remaining <= 0.0:
            break
    return state


def write_run(candidate: Path, run_dir: Path, start, model: dict = RAW_MODEL) -> None:
    """Write commands.csv and mocap.csv for the plan in `candidate`, run from `start`."""
    commands = load_candidate_csv(candidate)
    robot = SimpleNamespace(**model)
    run_dir.mkdir(parents=True)
    with (run_dir / "commands.csv").open("w", newline="") as target:
        writer = csv.writer(target)
        writer.writerow(["k", "t_send", "vx", "vy", "omega", "dt"])
        for command in commands:
            writer.writerow([command.k, T0 + command.k*command.dt, command.vx,
                             command.vy, command.omega, command.dt])
    total = sum(command.dt for command in commands)
    with (run_dir / "mocap.csv").open("w", newline="") as target:
        writer = csv.writer(target)
        writer.writerow(["t", "x", "y", "theta_raw", "tracked"])
        for index in range(round((total + 1.0)*MOCAP_HZ) + 1):
            timestamp = T0 - 0.5 + index/MOCAP_HZ
            writer.writerow([timestamp, *_state_at(commands, start, robot, timestamp - T0), 1])
