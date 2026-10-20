#!/usr/bin/env python3
"""Fit the nominal model to the Phase 0 pilots, then report E, q and rho.

    python3 phase0analyze.py
"""

from __future__ import annotations

import csv
import math
import random
import statistics
from dataclasses import dataclass, replace
from datetime import datetime
from types import SimpleNamespace
from typing import Callable

import config
from core import (Command, ExecutionTrace, is_in_place_turn, motion_kind, nominal_rollout,
                  process_execution, trace_execution, validate_commands)
from phase0 import pilot_dirs, reject_old_phase0


@dataclass(frozen=True)
class Direction:
    label: str
    gain: str
    offset: str
    unit: str
    limit: str
    applies: Callable[[Command], bool]


# Every gain and offset applies only to segments of its own command kind
# (core.motion_kind), so it is fitted only through those segments: "+vx" is vx
# alone, and the vx of a diagonal walk or an arc has its own gain.
DIRECTIONS = (
    Direction("+vx", "MODEL_GAIN_VX_POS", "MODEL_OFFSET_VX_POS_M", "m",
              "VX_MAX_MPS", lambda c: motion_kind(c) == "vx"),
    Direction("+vy", "MODEL_GAIN_VY_POS", "MODEL_OFFSET_VY_POS_M", "m",
              "VY_MAX_MPS", lambda c: motion_kind(c) == "vy" and c.vy > 0),
    Direction("-vy", "MODEL_GAIN_VY_NEG", "MODEL_OFFSET_VY_NEG_M", "m",
              "VY_MAX_MPS", lambda c: motion_kind(c) == "vy" and c.vy < 0),
    Direction("+omega", "MODEL_GAIN_OMEGA_POS", "MODEL_OFFSET_OMEGA_POS_RAD", "rad",
              "OMEGA_MAX_RADPS", lambda c: motion_kind(c) == "omega" and c.omega > 0),
    Direction("-omega", "MODEL_GAIN_OMEGA_NEG", "MODEL_OFFSET_OMEGA_NEG_RAD", "rad",
              "OMEGA_MAX_RADPS", lambda c: motion_kind(c) == "omega" and c.omega < 0),
    Direction("vx+vy: vx", "MODEL_GAIN_DIAG_VX", "MODEL_OFFSET_DIAG_VX_M", "m",
              "DIAG_SPEED_SUM_MAX_MPS", lambda c: motion_kind(c) == "vx+vy"),
    Direction("vx+vy: +vy", "MODEL_GAIN_DIAG_VY_POS", "MODEL_OFFSET_DIAG_VY_POS_M", "m",
              "DIAG_SPEED_SUM_MAX_MPS", lambda c: motion_kind(c) == "vx+vy" and c.vy > 0),
    Direction("vx+vy: -vy", "MODEL_GAIN_DIAG_VY_NEG", "MODEL_OFFSET_DIAG_VY_NEG_M", "m",
              "DIAG_SPEED_SUM_MAX_MPS", lambda c: motion_kind(c) == "vx+vy" and c.vy < 0),
    Direction("vx+omega: vx", "MODEL_GAIN_ARC_VX", "MODEL_OFFSET_ARC_VX_M", "m",
              "ARC_VX_MAX_MPS", lambda c: motion_kind(c) == "vx+omega"),
    Direction("vx+omega: +omega", "MODEL_GAIN_ARC_OMEGA_POS", "MODEL_OFFSET_ARC_OMEGA_POS_RAD",
              "rad", "ARC_OMEGA_MAX_RADPS",
              lambda c: motion_kind(c) == "vx+omega" and c.omega > 0),
    Direction("vx+omega: -omega", "MODEL_GAIN_ARC_OMEGA_NEG", "MODEL_OFFSET_ARC_OMEGA_NEG_RAD",
              "rad", "ARC_OMEGA_MAX_RADPS",
              lambda c: motion_kind(c) == "vx+omega" and c.omega < 0),
)


@dataclass(frozen=True)
class Term:
    """A model parameter fitted on its own, kept within [lower, upper]."""
    name: str
    label: str
    unit: str
    scale: float
    lower: float
    upper: float
    applies: Callable[[Command], bool]


TERMS = (
    Term("MODEL_TURN_LAG_S", "turn lag", "s", config.DT_S, 0.0, 0.9*config.DT_S,
         is_in_place_turn),
    Term("MODEL_TURN_LAG_AFTER_WALK_S", "lag after walk", "s", config.DT_S, 0.0, 0.9*config.DT_S,
         is_in_place_turn),
    Term("MODEL_WALK_YAW_RAD_PER_M", "walk yaw", "rad/m", 0.2, -math.inf, math.inf,
         lambda c: motion_kind(c) == "vx"),
)
FITTED_NAMES = (tuple(name for d in DIRECTIONS for name in (d.gain, d.offset))
                + tuple(t.name for t in TERMS))
MODEL_NAMES = FITTED_NAMES + ("MODEL_GAIN_VX_NEG",)
METRICS_CSV = config.PHASE0_ROOT / "execution_metrics.csv"
REPORT_TXT = config.PHASE0_ROOT / "phase0_analysis.txt"
# Every random-search step draws its size between these two, as a fraction of
# each parameter's scale (1 for a gain, one segment at full speed for an offset).
# Mixing coarse and fine steps throughout keeps the search from stalling.
COARSE_STEP = 0.2
FINE_STEP = 0.001
MIN_GAIN = 0.05
DECIMALS = 4


def model_from_config() -> dict[str, float]:
    return {name: getattr(config, name) for name in MODEL_NAMES}


def boundary_errors(trace: ExecutionTrace, model) -> list[float]:
    """Distance between robot and model at every segment boundary after the start."""
    states = nominal_rollout(trace.commands, trace.initial_pose, model)
    return [math.hypot(actual_x - state[0], actual_y - state[1])
            for state, (actual_x, actual_y) in zip(states[1:], trace.actual_at_boundaries[1:])]


def max_deviation(trace: ExecutionTrace, model) -> float:
    """E of one pilot under `model`: largest boundary distance between robot and model."""
    return max(boundary_errors(trace, model))


def _quantile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    position = (len(ordered)-1)*probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (position-lower)*(ordered[upper]-ordered[lower])


def quantile_e(traces: list[ExecutionTrace], values: dict[str, float]) -> float:
    """q of the pilots under the model `values`: the PILOT_HIGH_QUANTILE quantile of E."""
    model = SimpleNamespace(**values)
    return _quantile([max_deviation(trace, model) for trace in traces],
                     config.PILOT_HIGH_QUANTILE)


def rms_error(traces: list[ExecutionTrace], values: dict[str, float],
              give_up_above: float = math.inf) -> float:
    """Root mean square of every boundary distance of every pilot under the model `values`.

    Returns inf as soon as it cannot be below `give_up_above`.
    """
    model = SimpleNamespace(**values)
    count = sum(len(trace.commands) for trace in traces)
    limit = give_up_above**2 * count
    total = 0.0
    for trace in traces:
        total += sum(error*error for error in boundary_errors(trace, model))
        if total >= limit:
            return math.inf
    return math.sqrt(total / count)


def segment_counts(traces: list[ExecutionTrace]) -> dict[str, int]:
    """Commanded segments per direction and term: the data each parameter is fitted on."""
    return {d.label: sum(1 for trace in traces for command in trace.commands
                         if d.applies(command))
            for d in (*DIRECTIONS, *TERMS)}


def fit(traces: list[ExecutionTrace], rng: random.Random, iterations: int) -> dict[str, float]:
    """Least squares by random search: the gains, offsets, turn lags and walk yaw
    with the lowest RMS of all boundary distances between robot and model.

    Every boundary of every pilot counts, so each parameter is pinned by all of
    its segments rather than by the few worst pilots that set q. Starts from
    the raw model (gain 1, offset 0, no lag, no walk yaw). Every step changes
    either one direction's gain and offset, or one term, or a random set of
    the parameters by random amounts, and keeps the change only if the RMS
    drops. A direction or term no pilot used keeps its config.py values.
    """
    counts = segment_counts(traces)
    values = model_from_config()
    pairs = []
    for d in DIRECTIONS:
        if counts[d.label]:
            values[d.gain], values[d.offset] = 1.0, 0.0
            pairs.append(((d.gain, 1.0), (d.offset, getattr(config, d.limit) * config.DT_S)))
    bounds = {}
    for t in TERMS:
        if counts[t.label]:
            values[t.name] = 0.0
            pairs.append(((t.name, t.scale),))
            bounds[t.name] = (t.lower, t.upper)
    if not pairs:
        return values
    free = [parameter for pair in pairs for parameter in pair]
    best = rms_error(traces, values)
    for _ in range(iterations):
        step = COARSE_STEP * (FINE_STEP / COARSE_STEP) ** rng.random()
        trial = dict(values)
        chosen = (rng.choice(pairs) if rng.random() < 0.5
                  else rng.sample(free, rng.randint(1, len(free))))
        for name, scale in chosen:
            trial[name] += rng.gauss(0.0, step * scale)
            if name.startswith("MODEL_GAIN"):
                trial[name] = max(MIN_GAIN, trial[name])
            if name in bounds:
                trial[name] = min(max(bounds[name][0], trial[name]), bounds[name][1])
        score = rms_error(traces, trial, best)
        if score < best:
            values, best = trial, score
    return {name: round(value, DECIMALS) if name in FITTED_NAMES else value
            for name, value in values.items()}


def load_pilots() -> tuple[dict[str, ExecutionTrace], list[str]]:
    """Every saved pilot that is valid under the current config.py, plus the reasons for the rest."""
    traces, failures = {}, []
    for pilot_dir in pilot_dirs():
        try:
            trace = trace_execution(pilot_dir / "raw", config)
            validate_commands(trace.commands, config)
        except (OSError, ValueError) as error:
            failures.append(f"invalid:{pilot_dir.name}:{error}")
            continue
        traces[pilot_dir.name] = trace
    return traces, failures


def analyze() -> None:
    config.validate()
    reject_old_phase0()
    traces, failures = load_pilots()
    if not traces:
        raise RuntimeError(f"No valid pilots in {config.PHASE0_ROOT}"
                           + (f" ({' | '.join(failures)})" if failures else ""))
    pilots = list(traces.values())
    rms_before = rms_error(pilots, model_from_config())
    q_before = quantile_e(pilots, model_from_config())
    fitted = fit(pilots, random.Random(config.MASTER_RANDOM_SEED), config.PHASE0_FIT_ITERATIONS)
    rms_fitted = rms_error(pilots, fitted)
    # Everything below uses the fitted model; config.py and its module stay as they are.
    settings = {name: value for name, value in vars(config).items() if name.isupper()}
    fitted_config = SimpleNamespace(**{**settings, **fitted})

    metrics = [replace(process_execution(config.PHASE0_ROOT / name / "raw", "pilot",
                                         fitted_config), run_id=name)
               for name in traces]
    with METRICS_CSV.open("w", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["run_id", "motion_group", "max_deviation_m",
                         "commanded_distance_m", "actual_distance_m",
                         "commanded_actual_ratio", "initial_x_m", "initial_y_m",
                         "initial_theta_rad"])
        writer.writerows(item.__dict__.values() for item in metrics)
    deviations = [item.max_deviation_m for item in metrics]
    q = _quantile(deviations, config.PILOT_HIGH_QUANTILE)
    rho = config.RHO_MULTIPLIER * q
    viable = q < rho < config.GOAL_RADIUS_M
    ready = viable and not failures
    with config.PHASE0_RESULTS_CSV.open("w", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["status", "pilot_count", "mean_e_m", "min_e_m", "median_e_m",
                         "max_e_m", "quantile_probability", "q_estimate_m", "rho_m",
                         "goal_radius_m", "dt_s", "t_segments",
                         "yaw_offset_rad", "marker_to_body_x_m", "marker_to_body_y_m",
                         *(name.lower() for name in MODEL_NAMES),
                         "vx_max_mps", "vy_max_mps", "omega_max_radps",
                         "diag_speed_sum_max_mps", "arc_vx_max_mps", "arc_omega_max_radps",
                         "yaw_command_sign", "lateral_command_sign",
                         "robot_radius_m",
                         "workspace_x_min_m", "workspace_x_max_m",
                         "workspace_y_min_m", "workspace_y_max_m",
                         "failures"])
        writer.writerow(["ready" if ready else "pending", len(deviations),
                         statistics.fmean(deviations), min(deviations),
                         statistics.median(deviations), max(deviations),
                         config.PILOT_HIGH_QUANTILE, q, rho,
                         config.GOAL_RADIUS_M, config.DT_S, config.T_SEGMENTS,
                         config.YAW_OFFSET_RAD, config.MARKER_TO_BODY_X_M,
                         config.MARKER_TO_BODY_Y_M,
                         *(fitted[name] for name in MODEL_NAMES),
                         config.VX_MAX_MPS, config.VY_MAX_MPS, config.OMEGA_MAX_RADPS,
                         config.DIAG_SPEED_SUM_MAX_MPS, config.ARC_VX_MAX_MPS,
                         config.ARC_OMEGA_MAX_RADPS,
                         config.POSITIVE_YAW_COMMAND_SIGN,
                         config.POSITIVE_LATERAL_COMMAND_SIGN,
                         config.ROBOT_RADIUS_M,
                         config.WORKSPACE_X_MIN_M, config.WORKSPACE_X_MAX_M,
                         config.WORKSPACE_Y_MIN_M, config.WORKSPACE_Y_MAX_M,
                         " | ".join(failures)])

    counts = segment_counts(pilots)
    quantile_name = f"{config.PILOT_HIGH_QUANTILE:.0%} quantile of E"
    lines = [
        f"Phase 0 analysis, {datetime.now():%Y-%m-%d %H:%M}, {len(pilots)} pilot(s)",
        "",
        f"Model per {config.DT_S:g} s segment.",
        f"Least squares by random search ({config.PHASE0_FIT_ITERATIONS} steps): minimizes",
        "the RMS of the robot-model distance at every segment boundary of every pilot.",
        f"  with the model now in config.py  RMS {rms_before:.3f} m, q {q_before:.3f} m",
        f"  with the fitted model            RMS {rms_fitted:.3f} m, q {q:.3f} m",
        f"  (q is the {quantile_name})",
        "",
        "  direction         segments    gain   offset per segment",
    ]
    for d in DIRECTIONS:
        note = "" if counts[d.label] else "   (no data: config.py value kept)"
        lines.append(f"  {d.label:<16}  {counts[d.label]:8d}  {fitted[d.gain]:6.4f}  "
                     f"{fitted[d.offset]:+7.4f} {d.unit}{note}")
    lines += ["", "  term              segments    value"]
    for t in TERMS:
        note = "" if counts[t.label] else "   (no data: config.py value kept)"
        lines.append(f"  {t.label:<16}  {counts[t.label]:8d}  {fitted[t.name]:+7.4f} {t.unit}{note}")
    lines += ["", "To use the fitted model, set these lines in config.py:"]
    lines += [f"{name} = {fitted[name]!r}" for name in FITTED_NAMES]
    lines += ["", "With the fitted model:"]
    lines += [f"  {name}: E = {deviation:.3f} m" for name, deviation in zip(traces, deviations)]
    lines += [
        f"  E: min {min(deviations):.3f}, median {statistics.median(deviations):.3f}, "
        f"max {max(deviations):.3f} m",
        f"  q ({quantile_name}) = {q:.3f} m",
        f"  rho = {config.RHO_MULTIPLIER:g} x q = {rho:.3f} m",
        "",
    ]
    lines += [f"Not used: {failure}" for failure in failures]
    if ready:
        lines.append("Status: ready. main.py accepts this result once config.py holds "
                     "the lines above.")
    elif not viable:
        lines.append(f"Status: pending. rho must be below the goal radius "
                     f"({config.GOAL_RADIUS_M:g} m).")
    else:
        lines.append("Status: pending. Remove or recollect the pilots listed as not used.")
    report = "\n".join(lines) + "\n"
    REPORT_TXT.write_text(report, encoding="utf-8")
    print(report, end="")
    print(f"Saved as {REPORT_TXT}")


def main() -> None:
    try:
        analyze()
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"Error: {error}")


if __name__ == "__main__":
    main()
