#!/usr/bin/env python3
"""Read the robot's mocap coordinates, used to set up the boundary of the map.

    python3 robot_coordinate_getter.py

Stand the robot on a boundary point and press Enter to read it; type q to
finish. Each reading is the robot's body center in the experiment world frame,
the same coordinates the planner uses (marker shifted by MARKER_TO_BODY_*,
heading corrected by YAW_OFFSET_RAD). After exactly four corners the program
prints the WORKSPACE_* lines for config.py. Nothing is sent to the robot.
"""

from __future__ import annotations

import math


def inner_rectangle(corners) -> tuple[float, float, float, float]:
    """Largest axis-aligned rectangle inside four corners: (x_min, x_max, y_min, y_max)."""
    if len(corners) != 4:
        raise ValueError("Exactly four corners are required")
    xs = sorted(corner[0] for corner in corners)
    ys = sorted(corner[1] for corner in corners)
    return xs[1], xs[2], ys[1], ys[2]


def _print_summary(points) -> None:
    if not points:
        return
    print("\nRecorded points (x, y in m):")
    for number, (x, y, _) in enumerate(points, start=1):
        print(f"  {number}: ({x:.3f}, {y:.3f})")
    if len(points) != 4:
        print("Record exactly four corners to get the WORKSPACE_* lines.")
        return
    x_min, x_max, y_min, y_max = inner_rectangle(points)
    print("\nLargest axis-aligned rectangle inside these corners "
          f"({x_max - x_min:.2f} x {y_max - y_min:.2f} m). Paste into config.py:")
    print(f"WORKSPACE_X_MIN_M = {x_min:.3f}")
    print(f"WORKSPACE_X_MAX_M = {x_max:.3f}")
    print(f"WORKSPACE_Y_MIN_M = {y_min:.3f}")
    print(f"WORKSPACE_Y_MAX_M = {y_max:.3f}")


def main() -> None:
    from hardware.start_check import read_pose

    print("Stand the robot on a boundary point, then press Enter to read it. "
          "Type q then Enter to finish.")
    points = []
    while True:
        try:
            answer = input(f"Point {len(points) + 1}: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if answer == "q":
            break
        pose = read_pose()
        if pose is None:
            print("  Mocap is not tracking the robot.")
            continue
        points.append(pose)
        print(f"  x={pose[0]:.3f} m, y={pose[1]:.3f} m, "
              f"heading={math.degrees(pose[2]):.0f} deg")
    _print_summary(points)


if __name__ == "__main__":
    main()
