"""Live mocap check that the robot stands at the planned start before a run."""

from __future__ import annotations

import math
import sys
import threading
import time

import config
from core import body_pose, quaternion_to_theta_raw, wrap_angle


def read_pose(duration_s: float = 0.5):
    """Return the latest corrected body pose of the robot, or None if untracked."""
    sys.path.insert(0, str(config.PROJECT_ROOT / "vendor"))
    from vendor.NatNetClient import NatNetClient

    latest = []
    lock = threading.Lock()

    def on_frame(frame):
        data = frame["mocap_data"].rigid_body_data
        for body in (data.rigid_body_list if data is not None else []):
            if body.id_num == config.RIGID_BODY_ID and body.tracking_valid:
                pose = body_pose(config.MOCAP_GROUND_X_SIGN * body.pos[0],
                                 config.MOCAP_GROUND_Y_SIGN * body.pos[2],
                                 quaternion_to_theta_raw(*body.rot), config)
                with lock:
                    latest[:] = [pose]

    client = NatNetClient()
    client.set_client_address(config.MOCAP_CLIENT_IP)
    client.set_server_address(config.MOCAP_SERVER_IP)
    client.set_use_multicast(config.MOCAP_USE_MULTICAST)
    client.new_frame_with_data_listener = on_frame
    if not client.run("d"):
        raise RuntimeError("Could not start NatNet listener")
    try:
        time.sleep(duration_s)
    finally:
        client.shutdown()
    with lock:
        return latest[0] if latest else None


def start_offset(pose, start):
    """Distance to the start, the move in the robot's own frame, and the turn needed."""
    dx, dy = start[0]-pose[0], start[1]-pose[1]
    forward = math.cos(pose[2])*dx + math.sin(pose[2])*dy
    left = -math.sin(pose[2])*dx + math.cos(pose[2])*dy
    return math.hypot(dx, dy), forward, left, wrap_angle(start[2]-pose[2])


def wait_for_start(start, label: str):
    """Block until the robot is within tolerance of the start pose; return its pose."""
    print(f"{label}: start at x={start[0]:.2f} m, y={start[1]:.2f} m, "
          f"facing {math.degrees(start[2]):.0f} deg")
    while True:
        pose = read_pose()
        if pose is None:
            print("  Mocap is not tracking the robot.")
        else:
            distance, forward, left, turn = start_offset(pose, start)
            if (distance <= config.START_POSITION_TOLERANCE_M
                    and abs(turn) <= config.START_HEADING_TOLERANCE_RAD):
                print(f"  OK: {distance:.2f} m from start, heading off by "
                      f"{math.degrees(turn):+.0f} deg.")
                return pose
            print(f"  Not at start: move {abs(forward):.2f} m "
                  f"{'forward' if forward >= 0 else 'backward'} and {abs(left):.2f} m "
                  f"{'left' if left >= 0 else 'right'}, then turn "
                  f"{'left' if turn >= 0 else 'right'} {abs(math.degrees(turn)):.0f} deg.")
        input("  Adjust the robot, then press Enter to check again (Ctrl+C to stop). ")
