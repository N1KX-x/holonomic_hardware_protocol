import csv
import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import config
from core import (Command, load_candidate_csv, nominal_step, process_execution,
                  step_holonomic, validate_commands, write_candidate_csv)


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

    def test_nominal_model_applies_calibrated_gains(self):
        gains = dict(MODEL_GAIN_VX_POS=0.9, MODEL_GAIN_VX_NEG=0.5, MODEL_GAIN_VY_POS=0.8,
                     MODEL_GAIN_VY_NEG=0.4, MODEL_GAIN_OMEGA_POS=0.7, MODEL_GAIN_OMEGA_NEG=0.3,
                     MODEL_OFFSET_VX_POS_M=0.0, MODEL_OFFSET_VY_POS_M=0.0,
                     MODEL_OFFSET_VY_NEG_M=0.0, MODEL_OFFSET_OMEGA_POS_RAD=0.0,
                     MODEL_OFFSET_OMEGA_NEG_RAD=0.0)
        # Each axis uses the gain for the direction of its own command.
        cases = [((0.5, 0.2, 0.4), (0.45, 0.16, 0.28)),
                 ((-0.5, -0.2, -0.4), (-0.25, -0.08, -0.12)),
                 ((0.5, -0.2, 0.4), (0.45, -0.08, 0.28))]
        with mock.patch.multiple(config, **gains):
            for (vx, vy, omega), scaled in cases:
                got = nominal_step(1.0, 2.0, 0.3, Command(0, vx, vy, omega, 2.0), config)
                expected = step_holonomic(1.0, 2.0, 0.3, *scaled, 2.0)
                for value, target in zip(got, expected):
                    self.assertAlmostEqual(value, target)

    def test_nominal_model_adds_offset_per_segment_but_keeps_zero_commands_still(self):
        model = dict(MODEL_GAIN_VX_POS=0.9, MODEL_GAIN_VX_NEG=0.5, MODEL_GAIN_VY_POS=0.8,
                     MODEL_GAIN_VY_NEG=0.4, MODEL_GAIN_OMEGA_POS=0.7, MODEL_GAIN_OMEGA_NEG=0.3,
                     MODEL_OFFSET_VX_POS_M=0.05, MODEL_OFFSET_VY_POS_M=0.02,
                     MODEL_OFFSET_VY_NEG_M=-0.03, MODEL_OFFSET_OMEGA_POS_RAD=0.01,
                     MODEL_OFFSET_OMEGA_NEG_RAD=-0.04)
        with mock.patch.multiple(config, **model):
            # Straight segments: motion along each axis is gain * command * dt + offset.
            x, y, theta = nominal_step(0.0, 0.0, 0.0, Command(0, 0.5, -0.2, 0.0, 2.0), config)
            self.assertAlmostEqual(x, 0.9*0.5*2.0 + 0.05)
            self.assertAlmostEqual(y, 0.4*-0.2*2.0 - 0.03)
            _, _, theta = nominal_step(0.0, 0.0, 0.0, Command(0, 0.0, 0.0, -0.4, 2.0), config)
            self.assertAlmostEqual(theta, 0.3*-0.4*2.0 - 0.04)
            # The offset belongs to the whole segment, so half of it applies at half time.
            x, _, _ = nominal_step(0.0, 0.0, 0.0, Command(0, 0.5, 0.0, 0.0, 2.0), config, 1.0)
            self.assertAlmostEqual(x, (0.9*0.5*2.0 + 0.05) / 2)
            # A zero command moves nothing, and backward vx has a gain only.
            self.assertEqual(nominal_step(1.0, 2.0, 0.3, Command(0, 0.0, 0.0, 0.0, 2.0), config),
                             (1.0, 2.0, 0.3))
            x, _, _ = nominal_step(0.0, 0.0, 0.0, Command(0, -0.5, 0.0, 0.0, 2.0), config)
            self.assertAlmostEqual(x, 0.5*-0.5*2.0)

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
                                     MARKER_TO_BODY_X_M=0.0, MARKER_TO_BODY_Y_M=0.0,
                                     MODEL_GAIN_VX_POS=1.0, MODEL_GAIN_VX_NEG=1.0, MODEL_GAIN_VY_POS=1.0,
                                     MODEL_GAIN_VY_NEG=1.0, MODEL_GAIN_OMEGA_POS=1.0,
                                     MODEL_GAIN_OMEGA_NEG=1.0,
                                     MODEL_OFFSET_VX_POS_M=0.0, MODEL_OFFSET_VY_POS_M=0.0,
                                     MODEL_OFFSET_VY_NEG_M=0.0, MODEL_OFFSET_OMEGA_POS_RAD=0.0,
                                     MODEL_OFFSET_OMEGA_NEG_RAD=0.0):
                metric = process_execution(run, "test", config)
        self.assertAlmostEqual(metric.initial_x_m, 5.0)
        self.assertAlmostEqual(metric.initial_y_m, 3.0)
        self.assertAlmostEqual(metric.max_deviation_m, 0.0)


if __name__ == "__main__":
    unittest.main()
