import csv
import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import config
from core import (Command, load_candidate_csv, process_execution, step_holonomic,
                  validate_commands, write_candidate_csv)


class CoreTests(unittest.TestCase):
    def test_straight_and_lateral_step(self):
        x, y, theta = step_holonomic(1.0, 2.0, math.pi/2, 0.4, 0.2, 0.0, 2.0)
        self.assertAlmostEqual(x, 0.6)
        self.assertAlmostEqual(y, 2.8)
        self.assertAlmostEqual(theta, math.pi/2)

    def test_exact_arc(self):
        x, y, theta = step_holonomic(0, 0, 0, 1, 0, math.pi/2, 1)
        self.assertAlmostEqual(x, 2/math.pi)
        self.assertAlmostEqual(y, 2/math.pi)
        self.assertAlmostEqual(theta, math.pi/2)

    def test_candidate_csv_round_trip(self):
        commands = [Command(k, 0.1, 0.0, 0.0, config.DT_S)
                    for k in range(config.T_SEGMENTS)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "candidate.csv"
            write_candidate_csv(path, commands)
            loaded = load_candidate_csv(path)
        self.assertEqual(commands, loaded)
        self.assertEqual(commands, validate_commands(loaded, config))

    def test_processing_uses_matched_times_and_measured_start(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / "run"
            run.mkdir()
            with (run / "commands.csv").open("w", newline="") as target:
                writer = csv.writer(target)
                writer.writerow(["k", "t_send", "vx", "vy", "omega", "dt"])
                writer.writerow([0, 10.0, 0.5, 0.0, 0.0, 2.0])
            with (run / "mocap.csv").open("w", newline="") as target:
                writer = csv.writer(target)
                writer.writerow(["t", "x", "y", "theta_raw", "tracked"])
                # Start is deliberately not (0, 0), and samples do not align
                # exactly with the nominal boundary times.
                writer.writerow([9.5, 4.75, 3.0, 0.0, 1])
                writer.writerow([10.5, 5.25, 3.0, 0.0, 1])
                writer.writerow([11.5, 5.75, 3.0, 0.0, 1])
                writer.writerow([12.5, 6.25, 3.0, 0.0, 1])
            # Synthetic data has no marker offset, whatever the lab calibration is.
            with mock.patch.multiple(config, MAX_TRACKING_GAP_S=2.0, YAW_OFFSET_RAD=0.0,
                                     MARKER_TO_BODY_X_M=0.0, MARKER_TO_BODY_Y_M=0.0):
                metric = process_execution(run, "test", config)
        self.assertAlmostEqual(metric.initial_x_m, 5.0)
        self.assertAlmostEqual(metric.initial_y_m, 3.0)
        self.assertAlmostEqual(metric.max_deviation_m, 0.0)


if __name__ == "__main__":
    unittest.main()
