#!/usr/bin/env python3
"""Prepare, collect, and summarize pipeline checks and the deviation pilot.

Safe commands such as ``prepare`` and ``summarize`` never access hardware.
``collect`` additionally requires --execute and ALLOW_HARDWARE_EXECUTION=True.
The final phase0_results.csv is the only Phase 0 file main.py consumes.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import statistics
from pathlib import Path

import config
from core import (Command, load_sent_commands, process_execution, validate_commands,
                  wrap_angle, write_candidate_csv)


PLAN_DIR = config.PHASE0_ROOT / "plans"
RAW_DIR = config.PHASE0_ROOT / "raw"
METRICS_CSV = config.PHASE0_ROOT / "execution_metrics.csv"
REVIEW_CSV = config.PHASE0_ROOT / "pipeline_review.csv"
MANIFEST_CSV = config.PHASE0_ROOT / "plan_manifest.csv"


def _pipeline_plans():
    duration = config.PIPELINE_TEST_DURATION_S
    return {
        "pipeline_forward": ("pipeline_forward", [Command(0, config.PIPELINE_FORWARD_VX_MPS, 0, 0, duration)]),
        "pipeline_arc": ("pipeline_arc", [Command(0, config.PIPELINE_ARC_VX_MPS, 0,
                                                    config.PIPELINE_ARC_OMEGA_RADPS, duration)]),
        "pipeline_lateral": ("pipeline_lateral", [Command(0, 0, config.PIPELINE_LATERAL_VY_MPS, 0, duration)]),
    }


def _translation_limit(component_max: float) -> float:
    """Largest single-axis speed allowed by both the axis and combined limits."""
    combined = config.TRANSLATIONAL_SPEED_MAX_MPS
    return component_max if combined is None else min(component_max, combined)


def _pilot_commands(group: str, rng: random.Random) -> list[Command]:
    commands = []
    for k in range(config.T_SEGMENTS):
        if group == "vx_only":
            limit = _translation_limit(config.VX_MAX_MPS)
            vx, vy, omega = rng.uniform(-limit, limit), 0.0, 0.0
        elif group == "vy_only":
            limit = _translation_limit(config.VY_MAX_MPS)
            vx, vy, omega = 0.0, rng.uniform(-limit, limit), 0.0
        elif group == "omega_only":
            vx, vy, omega = 0.0, 0.0, rng.uniform(-config.OMEGA_MAX_RADPS, config.OMEGA_MAX_RADPS)
        else:
            limit = config.TRANSLATIONAL_SPEED_MAX_MPS
            for _ in range(1000):
                vx = rng.uniform(-config.VX_MAX_MPS, config.VX_MAX_MPS)
                vy = rng.uniform(-config.VY_MAX_MPS, config.VY_MAX_MPS)
                if limit is None or math.hypot(vx, vy) <= limit:
                    break
            else:
                raise RuntimeError("Could not sample a mixed command within TRANSLATIONAL_SPEED_MAX_MPS")
            omega = rng.uniform(-config.OMEGA_MAX_RADPS, config.OMEGA_MAX_RADPS)
        commands.append(Command(k, vx, vy, omega, config.DT_S))
    return commands


def _build_plans() -> dict[str, tuple[str, list[Command]]]:
    """Generate every Phase 0 plan and reject any the velocity runner would refuse."""
    plans = _pipeline_plans()
    rng = random.Random(config.MASTER_RANDOM_SEED)
    groups = (
        [("vx_only", config.PILOT_RUNS_VX_ONLY),
         ("vy_only", config.PILOT_RUNS_VY_ONLY),
         ("omega_only", config.PILOT_RUNS_OMEGA_ONLY),
         ("mixed", config.PILOT_RUNS_MIXED)]
    )
    for group, count in groups:
        for number in range(1, count + 1):
            run_id = f"pilot_{group}_{number:02d}"
            plans[run_id] = (group, _pilot_commands(group, rng))
    for run_id, (_, commands) in plans.items():
        try:
            validate_commands(commands, config,
                              require_main_horizon=run_id.startswith("pilot_"))
        except ValueError as error:
            raise ValueError(f"{run_id}: {error}") from error
    return plans


def prepare() -> None:
    config.validate()
    if MANIFEST_CSV.exists():
        raise FileExistsError(f"Phase 0 is already prepared: {MANIFEST_CSV}")
    # Every plan is validated before any file is written, so a rejected plan
    # never leaves a partial plans folder behind.
    plans = _build_plans()
    config.PHASE0_ROOT.mkdir(parents=True, exist_ok=True)
    PLAN_DIR.mkdir(exist_ok=True)
    with MANIFEST_CSV.open("x", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["run_id", "kind", "motion_group", "candidate_csv", "status"])
        for run_id, (group, commands) in plans.items():
            path = PLAN_DIR / f"{run_id}.csv"
            write_candidate_csv(path, commands)
            writer.writerow([run_id, "pipeline" if run_id.startswith("pipeline_") else "pilot",
                             group, path.relative_to(config.PROJECT_ROOT), "planned"])
    with REVIEW_CSV.open("x", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["test", "status", "operator_note"])
        writer.writerow(["pipeline_forward", "pending", "Expect mainly along-track error; no large sideways error"])
        writer.writerow(["pipeline_arc", "pending", "Nominal and actual arcs must turn the same direction"])
        writer.writerow(["pipeline_lateral", "pending", "Actual lateral direction must match nominal"])
        writer.writerow(["clock_sync", "pending", "Verify mocap and command timestamp synchronization"])
        writer.writerow(["yaw_sign", "pending", "Positive omega increases corrected yaw"])
        writer.writerow(["lateral_sign", "pending", "Positive vy moves robot left"])
    print(f"Prepared Phase 0 plans in {PLAN_DIR}")
    print(f"Before finalizing, change each status to pass in {REVIEW_CSV} after inspection.")


def _manifest():
    with MANIFEST_CSV.open(newline="", encoding="utf-8") as source:
        return {row["run_id"]: row for row in csv.DictReader(source)}


def collect(run_id: str, execute: bool) -> None:
    if not execute:
        raise RuntimeError("Collection requires the explicit --execute flag")
    rows = _manifest()
    if run_id not in rows:
        raise ValueError(f"Unknown Phase 0 run: {run_id}")
    if config.REQUIRE_OPERATOR_CONFIRMATION:
        answer = input(f"Execute {run_id} on the robot? Type the full run ID: ").strip()
        if answer != run_id:
            raise RuntimeError("Operator confirmation did not match")
    from hardware.collector import collect as collect_hardware
    candidate = config.PROJECT_ROOT / rows[run_id]["candidate_csv"]
    collect_hardware(candidate, RAW_DIR / run_id)


def _quantile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    position = (len(ordered)-1)*probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (position-lower)*(ordered[upper]-ordered[lower])


def _estimate_yaw_offset() -> float:
    """Estimate marker-to-forward yaw from the straight pipeline run."""
    run = RAW_DIR / "pipeline_forward"
    commands = load_sent_commands(run / "commands.csv")
    t0 = commands[0].t_send
    tf = t0 + sum(command.dt for command in commands)
    with (run / "mocap.csv").open(newline="", encoding="utf-8") as source:
        rows = [row for row in csv.DictReader(source)
                if row["tracked"].strip().lower() in {"1", "true", "yes"}
                and t0 <= float(row["t"]) <= tf]
    if len(rows) < 2:
        raise ValueError("Not enough tracked samples for yaw-offset estimate")
    line_heading = math.atan2(float(rows[-1]["y"])-float(rows[0]["y"]),
                              float(rows[-1]["x"])-float(rows[0]["x"]))
    sine = statistics.fmean(math.sin(float(row["theta_raw"])) for row in rows)
    cosine = statistics.fmean(math.cos(float(row["theta_raw"])) for row in rows)
    marker_heading = math.atan2(sine, cosine)
    return wrap_angle(line_heading-marker_heading)


def summarize() -> None:
    config.validate()
    rows = _manifest()
    metrics = []
    failures = []
    for run_id, row in rows.items():
        run_dir = RAW_DIR / run_id
        if not run_dir.is_dir():
            failures.append(f"missing:{run_id}")
            continue
        try:
            metrics.append(process_execution(run_dir, row["motion_group"], config))
        except (OSError, ValueError) as error:
            failures.append(f"invalid:{run_id}:{error}")
    with METRICS_CSV.open("w", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["run_id", "motion_group", "max_deviation_m",
                         "commanded_distance_m", "actual_distance_m",
                         "commanded_actual_ratio", "initial_x_m", "initial_y_m",
                         "initial_theta_rad"])
        for item in metrics:
            writer.writerow(item.__dict__.values())
    pilots = [item for item in metrics if item.run_id.startswith("pilot_")]
    expected_pilots = sum((config.PILOT_RUNS_VX_ONLY, config.PILOT_RUNS_VY_ONLY,
                           config.PILOT_RUNS_OMEGA_ONLY, config.PILOT_RUNS_MIXED))
    with REVIEW_CSV.open(newline="", encoding="utf-8") as source:
        reviews = list(csv.DictReader(source))
    review_passed = bool(reviews) and all(row["status"].strip().lower() == "pass" for row in reviews)
    try:
        yaw_estimate = _estimate_yaw_offset()
    except (OSError, ValueError):
        yaw_estimate = math.nan
    if pilots:
        deviations = [item.max_deviation_m for item in pilots]
        q = _quantile(deviations, config.PILOT_HIGH_QUANTILE)
        rho = config.RHO_MULTIPLIER*q
    else:
        deviations, q, rho = [], math.nan, math.nan
    viable = bool(deviations) and q < rho < config.GOAL_RADIUS_M
    ready = len(pilots) == expected_pilots and not failures and review_passed and viable
    config.PHASE0_RESULTS_CSV.parent.mkdir(parents=True, exist_ok=True)
    with config.PHASE0_RESULTS_CSV.open("w", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["status", "pilot_count", "expected_pilot_count", "min_e_m",
                         "median_e_m", "max_e_m", "quantile_probability", "q_estimate_m",
                         "rho_m", "goal_radius_m", "dt_s", "t_segments",
                         "yaw_offset_rad", "marker_to_body_x_m", "marker_to_body_y_m",
                         "yaw_offset_estimate_rad",
                         "model_gain_vx", "model_gain_vy", "model_gain_omega",
                         "vx_max_mps", "vy_max_mps", "omega_max_radps",
                         "yaw_command_sign", "lateral_command_sign",
                         "robot_radius_m",
                         "workspace_x_min_m", "workspace_x_max_m",
                         "workspace_y_min_m", "workspace_y_max_m",
                         "failures"])
        writer.writerow(["ready" if ready else "pending", len(pilots), expected_pilots,
                         min(deviations) if deviations else "",
                         statistics.median(deviations) if deviations else "",
                         max(deviations) if deviations else "", config.PILOT_HIGH_QUANTILE,
                         q if deviations else "", rho if deviations else "",
                         config.GOAL_RADIUS_M, config.DT_S, config.T_SEGMENTS,
                         config.YAW_OFFSET_RAD, config.MARKER_TO_BODY_X_M,
                         config.MARKER_TO_BODY_Y_M,
                         yaw_estimate if math.isfinite(yaw_estimate) else "",
                         config.MODEL_GAIN_VX, config.MODEL_GAIN_VY, config.MODEL_GAIN_OMEGA,
                         config.VX_MAX_MPS,
                         config.VY_MAX_MPS, config.OMEGA_MAX_RADPS,
                         config.POSITIVE_YAW_COMMAND_SIGN,
                         config.POSITIVE_LATERAL_COMMAND_SIGN,
                         config.ROBOT_RADIUS_M,
                         config.WORKSPACE_X_MIN_M, config.WORKSPACE_X_MAX_M,
                         config.WORKSPACE_Y_MIN_M, config.WORKSPACE_Y_MAX_M,
                         " | ".join(failures)])
    print(f"Wrote {config.PHASE0_RESULTS_CSV}; status={'ready' if ready else 'pending'}")


def main() -> None:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("prepare")
    collection = sub.add_parser("collect")
    collection.add_argument("run_id")
    collection.add_argument("--execute", action="store_true")
    sub.add_parser("summarize")
    args = parser.parse_args()
    if args.action == "prepare":
        prepare()
    elif args.action == "collect":
        collect(args.run_id, args.execute)
    else:
        summarize()


if __name__ == "__main__":
    main()
