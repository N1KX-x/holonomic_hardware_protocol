#!/usr/bin/env python3
"""Phase 1 planning and collection controller.

This program never starts a run by default. Planning is a separate ``prepare``
action. A single instance can only be executed with ``collect <ID> --execute``
and the independent safety switch in config.py.
"""

from __future__ import annotations

import argparse
import csv
import json
import random
from pathlib import Path

import config
from analyze_phase1 import analyze
from core import process_execution, validate_commands, write_candidate_csv
from planner import obstacles_json, plan_candidate, sample_problem


PLAN_ROOT = config.PHASE1_ROOT / "plans"
RAW_ROOT = config.PHASE1_ROOT / "raw"
ORDER_CSV = config.PHASE1_ROOT / "collection_order.csv"
VALIDATION_CSV = config.PHASE1_ROOT / "collection_validation.csv"
RESET_ROOT = config.PHASE1_ROOT / "resets"


def load_phase0() -> float:
    if not config.PHASE0_RESULTS_CSV.is_file():
        raise FileNotFoundError(
            f"Run phase0.py summarize first; missing {config.PHASE0_RESULTS_CSV}"
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
        "robot_radius_m": config.ROBOT_RADIUS_M,
        "workspace_x_min_m": config.WORKSPACE_X_MIN_M,
        "workspace_x_max_m": config.WORKSPACE_X_MAX_M,
        "workspace_y_min_m": config.WORKSPACE_Y_MIN_M,
        "workspace_y_max_m": config.WORKSPACE_Y_MAX_M,
    }
    changed = [name for name, value in frozen.items() if float(row[name]) != value]
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


def prepare() -> None:
    config.validate()
    rho = load_phase0()
    if config.PROBLEMS_CSV.exists() or PLAN_ROOT.exists() or ORDER_CSV.exists():
        raise FileExistsError("Phase 1 has already been prepared")
    PLAN_ROOT.mkdir(parents=True)
    rows = []
    for instance in range(1, config.N_POOL + 1):
        problem_seed = config.MASTER_RANDOM_SEED + instance*10000
        problem = sample_problem(instance, problem_seed, rho)
        rrt_seeds = [problem_seed + candidate for candidate in range(1, config.CANDIDATES_PER_INSTANCE + 1)]
        generated = []
        for seed in rrt_seeds:
            candidate = plan_candidate(problem, seed, rho)
            if candidate is None:
                generated = []
                break
            generated.append((seed, validate_commands(candidate, config)))
        shuffle_seed = problem_seed + 9999
        random.Random(shuffle_seed).shuffle(generated)
        status = "planned" if len(generated) == config.CANDIDATES_PER_INSTANCE else "empty"
        instance_dir = PLAN_ROOT / f"I{instance:03d}"
        if generated:
            instance_dir.mkdir()
            for order, (seed, commands) in enumerate(generated, start=1):
                write_candidate_csv(instance_dir / f"C{order:02d}.csv", commands)
        rows.append({
            "instance": f"I{instance:03d}",
            "start_x": problem.start_x, "start_y": problem.start_y,
            "start_theta": problem.start_theta,
            "goal_x": problem.goal_x, "goal_y": problem.goal_y,
            "goal_radius_m": config.GOAL_RADIUS_M,
            "robot_radius_m": config.ROBOT_RADIUS_M,
            "obstacles": obstacles_json(problem),
            "rrt_seeds": json.dumps(
                [seed for seed, _ in generated] if generated else rrt_seeds,
                separators=(",", ":"),
            ),
            "shuffle_seed": shuffle_seed, "rho_m": rho,
            "status": status,
            "note": "RRT failed for at least one required candidate" if status == "empty" else "",
        })
    fields = list(rows[0])
    config.PHASE1_ROOT.mkdir(parents=True, exist_ok=True)
    with config.PROBLEMS_CSV.open("x", newline="", encoding="utf-8") as target:
        writer = csv.DictWriter(target, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    planned = [row["instance"] for row in rows if row["status"] == "planned"]
    random.Random(config.MASTER_RANDOM_SEED + 777).shuffle(planned)
    with ORDER_CSV.open("x", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["order", "instance"])
        writer.writerows(enumerate(planned, start=1))
    with config.NOTES_FILE.open("x", encoding="utf-8") as target:
        target.write(f"Session: {config.SESSION_ID}\nOperator: {config.OPERATOR}\n")
        target.write(f"Surface: {config.SURFACE}\nMocap rate (Hz): {config.EXPECTED_MOCAP_RATE_HZ}\n")
        target.write(f"dt (s): {config.DT_S}\nT: {config.T_SEGMENTS}\nrho (m): {rho}\n")
        target.write(f"m: {config.CANDIDATES_PER_INSTANCE}\nN_pool: {config.N_POOL}\n\n")
        target.write("--- discarded instances ---\n")
    print(f"Prepared {len(rows)} instances: {len(planned)} planned, {len(rows)-len(planned)} empty")
    print("No robot command was issued.")


def _problem_rows():
    with config.PROBLEMS_CSV.open(newline="", encoding="utf-8") as source:
        return list(csv.DictReader(source))


def _replace_problem_rows(rows) -> None:
    temporary = config.PROBLEMS_CSV.with_suffix(".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as target:
        writer = csv.DictWriter(target, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(config.PROBLEMS_CSV)


def collect(instance_id: str, execute: bool) -> None:
    if not execute:
        raise RuntimeError("Collection requires the explicit --execute flag")
    load_phase0()
    rows = _problem_rows()
    row = next((item for item in rows if item["instance"] == instance_id), None)
    if row is None:
        raise ValueError(f"Unknown instance {instance_id}")
    if row["status"] != "planned":
        raise RuntimeError(f"{instance_id} status is {row['status']}; it cannot be executed")
    if config.CANDIDATES_PER_INSTANCE > 1 and not config.AUTONOMOUS_RESET_ENABLED:
        raise RuntimeError(
            "AUTONOMOUS_RESET_ENABLED is False. Review reset limits and enable it "
            "before multi-candidate collection."
        )
    if config.REQUIRE_OPERATOR_CONFIRMATION:
        answer = input(f"Execute all candidates in {instance_id}? Type the instance ID: ").strip()
        if answer != instance_id:
            raise RuntimeError("Operator confirmation did not match")
    from hardware.collector import collect as collect_hardware
    from hardware.reset_controller import autonomous_return
    row["status"] = "collecting"
    _replace_problem_rows(rows)
    try:
        home_pose = None
        for candidate_number in range(1, config.CANDIDATES_PER_INSTANCE + 1):
            candidate_id = f"C{candidate_number:02d}"
            run_dir = RAW_ROOT / f"{instance_id}_{candidate_id}"
            collect_hardware(PLAN_ROOT / instance_id / f"{candidate_id}.csv", run_dir)
            # Validate and process the candidate before any reset motion begins.
            metric = process_execution(run_dir, "phase1", config)
            if home_pose is None:
                home_pose = (metric.initial_x_m, metric.initial_y_m,
                             metric.initial_theta_rad)
            validation_exists = VALIDATION_CSV.exists()
            with VALIDATION_CSV.open("a", newline="", encoding="utf-8") as target:
                writer = csv.writer(target)
                if not validation_exists:
                    writer.writerow(["instance", "candidate", "max_deviation_m",
                                     "initial_x_m", "initial_y_m", "initial_theta_rad"])
                writer.writerow([instance_id, candidate_id, metric.max_deviation_m,
                                 metric.initial_x_m, metric.initial_y_m,
                                 metric.initial_theta_rad])
            if candidate_number < config.CANDIDATES_PER_INSTANCE:
                autonomous_return(
                    home_pose,
                    run_dir,
                    RESET_ROOT / f"{instance_id}_after_{candidate_id}",
                )
        row["status"] = "ok"
    except BaseException as error:
        row["status"] = "discarded"
        row["note"] = str(error) or type(error).__name__
        for run_dir in RAW_ROOT.glob(f"{instance_id}_C??"):
            discarded = run_dir.with_name(run_dir.name + "_discarded")
            if not discarded.exists():
                run_dir.rename(discarded)
        for reset_dir in RESET_ROOT.glob(f"{instance_id}_after_C??"):
            discarded = reset_dir.with_name(reset_dir.name + "_discarded")
            if not discarded.exists():
                reset_dir.rename(discarded)
        with config.NOTES_FILE.open("a", encoding="utf-8") as notes:
            notes.write(f"{instance_id}: {row['note']}\n")
        raise
    finally:
        _replace_problem_rows(rows)
    # Analysis runs only after the instance is saved as ok, so an analysis
    # error can never discard data that was collected cleanly.
    try:
        report_instance(instance_id)
    except Exception as error:
        print(f"{instance_id} is saved as ok, but analysis failed: {error}")
        print("Run python3 analyze_phase1.py to retry.")


def report_instance(instance_id: str) -> None:
    """Refresh the analysis files and print the deviations of one finished instance."""
    executions, instances = analyze()
    finished = [metric for metric in executions
                if metric.run_id.startswith(f"{instance_id}_C")]
    for metric in finished:
        print(f"  {metric.run_id}: E = {metric.max_deviation_m:.3f} m")
    largest = max(metric.max_deviation_m for metric in finished)
    print(f"{instance_id} done: largest E over {len(finished)} candidates = {largest:.3f} m")
    print(f"Analysis updated for {len(instances)} complete instance(s) in "
          f"{config.PHASE1_ROOT / 'analysis'}")


def status() -> None:
    rows = _problem_rows()
    counts = {}
    for row in rows:
        counts[row["status"]] = counts.get(row["status"], 0) + 1
    print(json.dumps(counts, indent=2, sort_keys=True))
    with ORDER_CSV.open(newline="", encoding="utf-8") as source:
        order = [row["instance"] for row in csv.DictReader(source)]
    remaining = {row["instance"] for row in rows if row["status"] == "planned"}
    next_instance = next((item for item in order if item in remaining), None)
    print(f"Next randomized instance: {next_instance or 'none'}")


def main() -> None:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("validate")
    sub.add_parser("prepare")
    sub.add_parser("status")
    collection = sub.add_parser("collect")
    collection.add_argument("instance", help="Instance ID such as I001")
    collection.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.action == "validate":
        config.validate()
        rho = load_phase0()
        print(f"Configuration and Phase 0 result are valid; rho={rho:.6f} m")
    elif args.action == "prepare":
        prepare()
    elif args.action == "status":
        status()
    else:
        collect(args.instance, args.execute)


if __name__ == "__main__":
    main()
