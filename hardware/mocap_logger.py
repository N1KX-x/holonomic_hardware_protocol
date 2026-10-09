#!/usr/bin/env python3
"""Log the configured Motive rigid body in protocol CSV form."""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import config
from core import quaternion_to_theta_raw


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if not config.ALLOW_HARDWARE_EXECUTION:
        raise RuntimeError("Hardware execution is disabled in config.py")
    vendor_dir = config.PROJECT_ROOT / "vendor"
    sys.path.insert(0, str(vendor_dir))
    try:
        from vendor.NatNetClient import NatNetClient
    except ImportError as error:
        raise RuntimeError(
            "NatNet SDK files are missing. Copy NatNetClient.py, DataDescriptions.py, "
            "and MoCapData.py into vendor/."
        ) from error

    client = NatNetClient()
    client.set_client_address(config.MOCAP_CLIENT_IP)
    client.set_server_address(config.MOCAP_SERVER_IP)
    client.set_use_multicast(config.MOCAP_USE_MULTICAST)
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(["t", "x", "y", "theta_raw", "tracked", "frame_number",
                         "motive_timestamp", "raw_x", "raw_y", "raw_z",
                         "qx", "qy", "qz", "qw"])
        target.flush()

        def log_frame(frame):
            timestamp = time.time()
            body_data = frame["mocap_data"].rigid_body_data
            bodies = body_data.rigid_body_list if body_data is not None else []
            body = next((item for item in bodies if item.id_num == config.RIGID_BODY_ID), None)
            if body is None:
                writer.writerow([timestamp, math.nan, math.nan, math.nan, 0,
                                 frame["frame_number"], frame["timestamp"],
                                 math.nan, math.nan, math.nan,
                                 math.nan, math.nan, math.nan, math.nan])
            else:
                raw_x, raw_y, raw_z = body.pos
                qx, qy, qz, qw = body.rot
                x = config.MOCAP_GROUND_X_SIGN * raw_x
                y = config.MOCAP_GROUND_Y_SIGN * raw_z
                theta = quaternion_to_theta_raw(qx, qy, qz, qw)
                writer.writerow([timestamp, x, y, theta, int(body.tracking_valid),
                                 frame["frame_number"], frame["timestamp"],
                                 raw_x, raw_y, raw_z, qx, qy, qz, qw])
            target.flush()

        client.new_frame_with_data_listener = log_frame
        started = False
        try:
            started = client.run("d")
            if not started:
                raise RuntimeError("Could not start NatNet listener")
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            if started:
                client.shutdown()


if __name__ == "__main__":
    main()
