"""Mocap-verified autonomous return to an instance's first measured pose."""

from __future__ import annotations

import math
import time
from pathlib import Path

import config
from core import Command, latest_measured_pose, wrap_angle, write_candidate_csv
from hardware.collector import collect


def _errors(pose, target):
    return (math.hypot(target[0]-pose[0], target[1]-pose[1]),
            abs(wrap_angle(target[2]-pose[2])))


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def return_command(pose, target) -> Command:
    """Invert one exact nominal step, then clamp it to reset speed limits."""
    dt = config.RESET_COMMAND_DT_S
    heading_error = wrap_angle(target[2]-pose[2])
    omega = _clip(heading_error/dt, config.RESET_OMEGA_MAX_RADPS)
    world_dx, world_dy = target[0]-pose[0], target[1]-pose[1]
    body_dx = math.cos(pose[2])*world_dx + math.sin(pose[2])*world_dy
    body_dy = -math.sin(pose[2])*world_dx + math.cos(pose[2])*world_dy
    turn = omega*dt
    if abs(omega) < 1e-9 or abs(turn) < 1e-9:
        vx, vy = body_dx/dt, body_dy/dt
    else:
        a = math.sin(turn)/omega
        b = (math.cos(turn)-1.0)/omega
        determinant = a*a + b*b
        vx = (a*body_dx - b*body_dy)/determinant
        vy = (b*body_dx + a*body_dy)/determinant
    vx = _clip(vx, config.RESET_VX_MAX_MPS)
    vy = _clip(vy, config.RESET_VY_MAX_MPS)
    combined = config.TRANSLATIONAL_SPEED_MAX_MPS
    magnitude = math.hypot(vx, vy)
    if combined is not None and magnitude > combined:
        vx, vy = vx*combined/magnitude, vy*combined/magnitude
    return Command(0, vx, vy, omega, dt)


def autonomous_return(target_pose, current_run_dir: Path, reset_root: Path):
    if not config.AUTONOMOUS_RESET_ENABLED:
        raise RuntimeError("Autonomous reset is disabled in config.py")
    # Reset uses the latest post-stop pose. Candidate analysis separately ends
    # at nominal T*dt and therefore excludes this coasting interval.
    pose = latest_measured_pose(current_run_dir, config)
    reset_root.mkdir(parents=True, exist_ok=False)
    for attempt in range(1, config.RESET_MAX_ATTEMPTS + 1):
        position_error, heading_error = _errors(pose, target_pose)
        if (position_error <= config.RESET_POSITION_TOLERANCE_M
                and heading_error <= config.RESET_HEADING_TOLERANCE_RAD):
            return pose
        attempt_root = reset_root / f"attempt_{attempt:02d}"
        attempt_root.mkdir()
        plan = attempt_root / "reset_plan.csv"
        write_candidate_csv(plan, [return_command(pose, target_pose)])
        raw = attempt_root / "raw"
        collect(plan, raw)
        pose = latest_measured_pose(raw, config)
        time.sleep(config.RESET_SETTLE_TIME_S)
    position_error, heading_error = _errors(pose, target_pose)
    if (position_error <= config.RESET_POSITION_TOLERANCE_M
            and heading_error <= config.RESET_HEADING_TOLERANCE_RAD):
        return pose
    raise RuntimeError(
        "Autonomous reset did not converge: "
        f"position error={position_error:.3f} m, heading error={heading_error:.3f} rad"
    )
