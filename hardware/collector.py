"""Safe process orchestration for one candidate execution."""

from __future__ import annotations

import csv
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import config
from core import body_pose, load_candidate_csv, validate_commands


def latest_sample(path: Path):
    """Return (x, y, theta_raw, tracked) from the newest complete mocap row, or None."""
    try:
        with path.open("rb") as source:
            source.seek(0, os.SEEK_END)
            source.seek(max(0, source.tell() - 1024))
            lines = source.read().decode("utf-8", "replace").splitlines()
    except OSError:
        return None
    # The logger may be mid-write, so fall back to the previous line.
    for line in reversed(lines[-3:]):
        fields = line.split(",")
        try:
            x, y, theta = float(fields[1]), float(fields[2]), float(fields[3])
            tracked = fields[4].strip() == "1"
        except (ValueError, IndexError):
            continue
        return x, y, theta, tracked
    return None


def safety_problem(marker_x: float, marker_y: float, theta_raw: float) -> str | None:
    """Describe why the robot must stop now, or return None if it is safe."""
    margin = config.EDGE_STOP_MARGIN_M
    if margin is None:
        return None
    x, y, _ = body_pose(marker_x, marker_y, theta_raw, config)
    if not (config.WORKSPACE_X_MIN_M + margin <= x <= config.WORKSPACE_X_MAX_M - margin
            and config.WORKSPACE_Y_MIN_M + margin <= y <= config.WORKSPACE_Y_MAX_M - margin):
        return (f"robot at x={x:.2f}, y={y:.2f} is within {margin} m of the "
                "workspace edge")
    return None


def _wait_for_tracking(process: subprocess.Popen, path: Path) -> None:
    deadline = time.monotonic() + config.TRACKING_START_TIMEOUT_S
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("Mocap logger exited before tracking became valid")
        if path.exists():
            with path.open(newline="", encoding="utf-8") as source:
                for row in csv.DictReader(source):
                    if row.get("tracked", "").lower() in {"1", "true"}:
                        values = [float(row[name]) for name in ("t", "x", "y", "theta_raw")]
                        if all(map(math.isfinite, values)) and abs(time.time() - values[0]) < 0.5:
                            return
        time.sleep(0.02)
    raise RuntimeError("No recent valid mocap tracking before timeout")


def _stop_group(process: subprocess.Popen | None, sig=signal.SIGINT) -> None:
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, sig)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def collect(candidate: Path, run_dir: Path) -> None:
    """Collect one run. Caller is responsible for instance-level discard."""
    config.validate()
    if not config.ALLOW_HARDWARE_EXECUTION:
        raise RuntimeError("Hardware execution is disabled in config.py")
    if run_dir.exists():
        raise FileExistsError(run_dir)
    # Reject an invalid plan before creating the run folder, so a refused plan
    # never leaves a partial run behind that blocks a later collection.
    validate_commands(load_candidate_csv(candidate), config, require_main_horizon=False)
    run_dir.mkdir(parents=True)
    mocap_path = run_dir / "mocap.csv"
    mocap = subprocess.Popen(
        [sys.executable, "-m", "hardware.mocap_logger", str(mocap_path)],
        cwd=config.PROJECT_ROOT, start_new_session=True,
    )
    velocity = None
    try:
        _wait_for_tracking(mocap, mocap_path)
        time.sleep(config.MOCAP_LEAD_TIME_S)
        if not config.NOVA_ENV_FILE.is_file():
            raise FileNotFoundError(f"ROS environment file is missing: {config.NOVA_ENV_FILE}")
        shell_command = (
            'source "$1" && cd "$2" && exec python3 -m hardware.velocity_runner '
            '"$3" "$4" "$5"'
        )
        velocity = subprocess.Popen(
            ["bash", "-lc", shell_command, "collector",
             str(config.NOVA_ENV_FILE), str(config.PROJECT_ROOT), str(candidate),
             str(run_dir / "commands.csv"), str(run_dir / "publish_log.csv")],
            cwd=config.PROJECT_ROOT, start_new_session=True,
        )
        last_tracked = time.monotonic()
        while velocity.poll() is None:
            if mocap.poll() is not None:
                raise RuntimeError("Mocap logger exited during robot execution")
            # Raising here stops the robot: the finally block interrupts the
            # velocity runner, which always publishes its stop commands.
            sample = latest_sample(mocap_path)
            if sample is not None and sample[3]:
                last_tracked = time.monotonic()
                problem = safety_problem(*sample[:3])
                if problem:
                    raise RuntimeError(f"Safety stop: {problem}")
            elif time.monotonic() - last_tracked > config.LIVE_TRACKING_TIMEOUT_S:
                raise RuntimeError("Safety stop: mocap lost the robot for more than "
                                   f"{config.LIVE_TRACKING_TIMEOUT_S} s")
            time.sleep(0.02)
        if velocity.returncode != 0:
            raise RuntimeError(f"Velocity runner returned {velocity.returncode}")
        time.sleep(config.MOCAP_TRAIL_TIME_S)
    finally:
        _stop_group(velocity)
        _stop_group(mocap)
