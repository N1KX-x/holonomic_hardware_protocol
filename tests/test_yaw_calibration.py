import csv
import tempfile
import unittest
from pathlib import Path

import yaw_offset_calibration


class YawOffsetTests(unittest.TestCase):
    def test_offset_is_travel_direction_minus_marker_heading(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            with (run / "commands.csv").open("w", newline="") as target:
                writer = csv.writer(target)
                writer.writerow(["k", "t_send", "vx", "vy", "omega", "dt"])
                writer.writerow([0, 10.0, 0.25, 0.0, 0.0, 4.0])
            with (run / "mocap.csv").open("w", newline="") as target:
                writer = csv.writer(target)
                writer.writerow(["t", "x", "y", "theta_raw", "tracked"])
                # The robot walks along +y (heading pi/2) while the marker reads 0.3 rad.
                for index in range(60):
                    t = 9.0 + index*0.1
                    writer.writerow([t, 1.0, 2.0 + 0.25*max(0.0, t - 10.0), 0.3, 1])
            self.assertAlmostEqual(yaw_offset_calibration.estimate_yaw_offset(run),
                                   3.141592653589793/2 - 0.3)

    def test_run_numbers_count_up(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(yaw_offset_calibration.next_run_number(root), 1)
            (root / "run_002").mkdir()
            self.assertEqual(yaw_offset_calibration.next_run_number(root), 3)


if __name__ == "__main__":
    unittest.main()
