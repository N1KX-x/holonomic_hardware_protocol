#!/usr/bin/env python3
"""Publish one explicit candidate sequence and log what was actually issued."""

from __future__ import annotations

import argparse
import csv
import signal
import sys
import time
from pathlib import Path

import config
from core import load_candidate_csv, validate_commands


def _stamp() -> str:
    value = time.time_ns()
    return f"{value // 1_000_000_000}.{value % 1_000_000_000:09d}"


def run(candidate_path: Path, commands_log: Path, publish_log: Path) -> int:
    config.validate()
    if not config.ALLOW_HARDWARE_EXECUTION:
        raise RuntimeError("Hardware execution is disabled in config.py")
    commands = validate_commands(
        load_candidate_csv(candidate_path), config, require_main_horizon=False
    )

    import rclpy
    from geometry_msgs.msg import Twist
    from unitree_go.msg import WebRtcReq

    commands_log.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = rclpy.create_node("holonomic_protocol_velocity")
    velocity_pub = node.create_publisher(Twist, config.CMD_VEL_TOPIC, 10)
    stop_pub = node.create_publisher(WebRtcReq, config.STOP_TOPIC, 10)
    completed = False
    try:
        deadline = time.monotonic() + 10.0
        while not (velocity_pub.get_subscription_count() and stop_pub.get_subscription_count()):
            if time.monotonic() >= deadline:
                raise RuntimeError("Required ROS subscribers were not found")
            rclpy.spin_once(node, timeout_sec=0.1)

        with commands_log.open("x", newline="", encoding="utf-8") as sent_file, \
             publish_log.open("x", newline="", encoding="utf-8") as audit_file:
            sent = csv.writer(sent_file)
            audit = csv.writer(audit_file)
            sent.writerow(["k", "t_send", "vx", "vy", "omega", "dt"])
            audit.writerow(["t", "event", "k", "vx", "vy", "omega"])

            def publish(message, event: str, k: int | str, request_stop: bool = False):
                velocity_pub.publish(message)
                if request_stop:
                    request = WebRtcReq()
                    request.topic = "rt/api/sport/request"
                    request.api_id = 1003
                    request.parameter = ""
                    request.priority = 1
                    stop_pub.publish(request)
                now = _stamp()
                audit.writerow([now, event, k, message.linear.x,
                                message.linear.y, message.angular.z])
                audit_file.flush()
                return now

            def stop(event: str):
                for index in range(config.STOP_REPEATS):
                    publish(Twist(), event if index == 0 else f"{event}_retry", "", True)
                    if index + 1 < config.STOP_REPEATS:
                        time.sleep(config.STOP_INTERVAL_S)

            try:
                stop("pre_stop")
                period_ns = round(1_000_000_000 / config.COMMAND_HZ)
                for command in commands:
                    message = Twist()
                    message.linear.x = command.vx
                    message.linear.y = command.vy * config.POSITIVE_LATERAL_COMMAND_SIGN
                    message.angular.z = command.omega * config.POSITIVE_YAW_COMMAND_SIGN
                    first = publish(message, "segment_start", command.k)
                    # commands.csv stays in the protocol's standardized body
                    # convention. publish_log.csv retains the raw API values
                    # after any configured interface-sign correction.
                    sent.writerow([command.k, first, command.vx, command.vy,
                                   command.omega, command.dt])
                    sent_file.flush()
                    end_ns = time.monotonic_ns() + round(command.dt * 1_000_000_000)
                    next_ns = time.monotonic_ns() + period_ns
                    while time.monotonic_ns() < end_ns:
                        now_ns = time.monotonic_ns()
                        time.sleep(max(0, min(next_ns, end_ns) - now_ns) / 1e9)
                        if time.monotonic_ns() < end_ns:
                            publish(message, "command", command.k)
                        next_ns += period_ns
                completed = True
            finally:
                signal.signal(signal.SIGINT, signal.SIG_IGN)
                stop("stop")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if completed else 130


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("candidate", type=Path)
    parser.add_argument("commands_log", type=Path)
    parser.add_argument("publish_log", type=Path)
    args = parser.parse_args()
    return run(args.candidate.resolve(), args.commands_log.resolve(), args.publish_log.resolve())


if __name__ == "__main__":
    sys.exit(main())
