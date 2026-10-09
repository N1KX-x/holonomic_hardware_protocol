"""Shared, hardware-independent protocol models and CSV processing."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class Command:
    k: int
    vx: float
    vy: float
    omega: float
    dt: float
    t_send: float | None = None


@dataclass(frozen=True)
class ExecutionMetrics:
    run_id: str
    motion_group: str
    max_deviation_m: float
    commanded_distance_m: float
    actual_distance_m: float
    commanded_actual_ratio: float
    initial_x_m: float
    initial_y_m: float
    initial_theta_rad: float


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_theta_raw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Project marker local +X into Motive's X/-Z ground plane."""
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(norm) or norm == 0.0:
        return math.nan
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    forward_x = 1.0 - 2.0 * (qy*qy + qz*qz)
    forward_minus_z = 2.0 * (qw*qy - qx*qz)
    if math.hypot(forward_x, forward_minus_z) < 1e-12:
        return math.nan
    return math.atan2(forward_minus_z, forward_x)


def step_holonomic(
    x: float, y: float, theta: float,
    vx: float, vy: float, omega: float, dt: float,
) -> tuple[float, float, float]:
    """Exact constant-command body-to-world model from the protocol."""
    if abs(omega) < 1e-10:
        return (
            x + (vx * math.cos(theta) - vy * math.sin(theta)) * dt,
            y + (vx * math.sin(theta) + vy * math.cos(theta)) * dt,
            theta,
        )
    next_theta = theta + omega * dt
    return (
        x + vx / omega * (math.sin(next_theta) - math.sin(theta))
          + vy / omega * (math.cos(next_theta) - math.cos(theta)),
        y - vx / omega * (math.cos(next_theta) - math.cos(theta))
          + vy / omega * (math.sin(next_theta) - math.sin(theta)),
        next_theta,
    )


def validate_commands(commands: Iterable[Command], cfg, require_main_horizon: bool = True) -> list[Command]:
    result = list(commands)
    if not result:
        raise ValueError("Control sequence is empty")
    if require_main_horizon and len(result) != cfg.T_SEGMENTS:
        raise ValueError(f"Expected {cfg.T_SEGMENTS} segments, got {len(result)}")
    for expected, command in enumerate(result):
        values = (command.vx, command.vy, command.omega, command.dt)
        if command.k != expected or not all(math.isfinite(v) for v in values):
            raise ValueError(f"Invalid command at segment {expected}")
        if command.dt <= 0:
            raise ValueError(f"Segment {expected} duration must be positive")
        if require_main_horizon and abs(command.dt - cfg.DT_S) > 1e-9:
            raise ValueError(f"Segment {expected} must use DT_S={cfg.DT_S}")
        if abs(command.vx) > cfg.VX_MAX_MPS or abs(command.vy) > cfg.VY_MAX_MPS:
            raise ValueError(f"Segment {expected} exceeds translation component limit")
        if abs(command.omega) > cfg.OMEGA_MAX_RADPS:
            raise ValueError(f"Segment {expected} exceeds yaw-rate limit")
        combined = cfg.TRANSLATIONAL_SPEED_MAX_MPS
        if combined is not None and math.hypot(command.vx, command.vy) > combined + 1e-12:
            raise ValueError(f"Segment {expected} exceeds combined translation limit")
    return result


def write_candidate_csv(path: Path, commands: Iterable[Command]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["k", "vx", "vy", "omega", "dt"])
        for command in commands:
            writer.writerow([command.k, command.vx, command.vy, command.omega, command.dt])


def load_candidate_csv(path: Path) -> list[Command]:
    with path.open(newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        required = {"k", "vx", "vy", "omega", "dt"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"{path} is missing candidate columns {sorted(required)}")
        return [Command(int(row["k"]), float(row["vx"]), float(row["vy"]),
                        float(row["omega"]), float(row["dt"])) for row in reader]


def load_sent_commands(path: Path) -> list[Command]:
    with path.open(newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        required = {"k", "t_send", "vx", "vy", "omega", "dt"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"{path} is missing sent-command columns {sorted(required)}")
        commands = [Command(int(row["k"]), float(row["vx"]), float(row["vy"]),
                            float(row["omega"]), float(row["dt"]), float(row["t_send"]))
                    for row in reader]
    if not commands or any(c.k != index for index, c in enumerate(commands)):
        raise ValueError(f"{path} has no commands or nonsequential k")
    return commands


def nominal_boundaries(commands: list[Command], initial_pose: tuple[float, float, float]):
    times = [commands[0].t_send]
    states = [initial_pose]
    state = initial_pose
    elapsed = 0.0
    for command in commands:
        state = step_holonomic(*state, command.vx, command.vy, command.omega, command.dt)
        elapsed += command.dt
        times.append(commands[0].t_send + elapsed)
        states.append(state)
    return times, states


def _load_mocap(path: Path, cfg):
    with path.open(newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        required = {"t", "x", "y", "theta_raw", "tracked"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"{path} is missing mocap columns {sorted(required)}")
        rows = [row for row in reader if row["tracked"].strip().lower() in {"1", "true", "yes"}]
    if len(rows) < 2:
        raise ValueError("Fewer than two valid mocap samples")
    data = sorted((float(r["t"]), float(r["x"]), float(r["y"]), float(r["theta_raw"]))
                  for r in rows)
    if not all(math.isfinite(value) for row in data for value in row):
        raise ValueError("Mocap contains a non-finite tracked value")
    unique = []
    for row in data:
        if not unique or row[0] != unique[-1][0]:
            unique.append(row)
    times = [row[0] for row in unique]
    marker_x = [row[1] for row in unique]
    marker_y = [row[2] for row in unique]
    theta = []
    previous = None
    for row in unique:
        value = row[3] + cfg.YAW_OFFSET_RAD
        if previous is not None:
            while value - previous > math.pi:
                value -= 2 * math.pi
            while value - previous < -math.pi:
                value += 2 * math.pi
        theta.append(value)
        previous = value
    body_x = [mx + math.cos(th)*cfg.MARKER_TO_BODY_X_M - math.sin(th)*cfg.MARKER_TO_BODY_Y_M
              for mx, th in zip(marker_x, theta)]
    body_y = [my + math.sin(th)*cfg.MARKER_TO_BODY_X_M + math.cos(th)*cfg.MARKER_TO_BODY_Y_M
              for my, th in zip(marker_y, theta)]
    return times, body_x, body_y, theta


def _interp(query: float, times: list[float], values: list[float]) -> float:
    if query < times[0] or query > times[-1]:
        raise ValueError("Interpolation requested outside mocap time span")
    import bisect
    right = bisect.bisect_left(times, query)
    if right == 0 or times[right] == query:
        return values[right]
    left = right - 1
    fraction = (query - times[left]) / (times[right] - times[left])
    return values[left] + fraction * (values[right] - values[left])


@dataclass(frozen=True)
class ExecutionTrace:
    commands: list[Command]
    initial_pose: tuple[float, float, float]
    nominal_at_boundaries: list[tuple[float, float, float]]
    actual_at_boundaries: list[tuple[float, float]]
    actual_path: list[tuple[float, float]]

    @property
    def errors(self) -> list[float]:
        return [math.hypot(ax-state[0], ay-state[1])
                for (ax, ay), state in zip(self.actual_at_boundaries,
                                           self.nominal_at_boundaries)]


def trace_execution(run_dir: Path, cfg) -> ExecutionTrace:
    """Validate one run and pair nominal and measured positions at segment boundaries."""
    commands = load_sent_commands(run_dir / "commands.csv")
    times, body_x, body_y, theta = _load_mocap(run_dir / "mocap.csv", cfg)
    t0 = commands[0].t_send
    tf = t0 + sum(command.dt for command in commands)
    if times[0] > t0 or times[-1] < tf:
        raise ValueError("Mocap does not cover the full nominal interval")
    for left, right in zip(times, times[1:]):
        if right > t0 and left < tf and right - left > cfg.MAX_TRACKING_GAP_S:
            raise ValueError("Tracking gap exceeds MAX_TRACKING_GAP_S")
    initial = (_interp(t0, times, body_x), _interp(t0, times, body_y),
               _interp(t0, times, theta))
    boundary_times, nominal = nominal_boundaries(commands, initial)
    actual = [(_interp(t, times, body_x), _interp(t, times, body_y)) for t in boundary_times]
    points = [actual[0]] + [(x, y) for t, x, y in zip(times, body_x, body_y) if t0 < t < tf] + [actual[-1]]
    return ExecutionTrace(commands, initial, nominal, actual, points)


def process_execution(run_dir: Path, motion_group: str, cfg) -> ExecutionMetrics:
    """Compute protocol E from raw commands.csv and mocap.csv."""
    trace = trace_execution(run_dir, cfg)
    initial = trace.initial_pose
    points = trace.actual_path
    actual_distance = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))
    commanded_distance = sum(math.hypot(c.vx, c.vy)*c.dt for c in trace.commands)
    return ExecutionMetrics(
        run_dir.name, motion_group, max(trace.errors), commanded_distance, actual_distance,
        commanded_distance / actual_distance if actual_distance > 0 else math.nan,
        initial[0], initial[1], wrap_angle(initial[2]),
    )


def execution_poses(run_dir: Path, cfg):
    """Return corrected/interpolated (initial_pose, final_pose) for one run."""
    commands = load_sent_commands(run_dir / "commands.csv")
    times, body_x, body_y, theta = _load_mocap(run_dir / "mocap.csv", cfg)
    t0 = commands[0].t_send
    tf = t0 + sum(command.dt for command in commands)
    if times[0] > t0 or times[-1] < tf:
        raise ValueError("Mocap does not cover the full nominal interval")
    initial = (_interp(t0, times, body_x), _interp(t0, times, body_y),
               wrap_angle(_interp(t0, times, theta)))
    final = (_interp(tf, times, body_x), _interp(tf, times, body_y),
             wrap_angle(_interp(tf, times, theta)))
    return initial, final


def latest_measured_pose(run_dir: Path, cfg):
    """Return the latest valid corrected mocap pose, including stopped/coasting time."""
    _, body_x, body_y, theta = _load_mocap(run_dir / "mocap.csv", cfg)
    return body_x[-1], body_y[-1], wrap_angle(theta[-1])
