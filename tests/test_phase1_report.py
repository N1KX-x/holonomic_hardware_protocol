import contextlib
import csv
import io
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import config
import main
import plot_phase1
from core import Command, load_candidate_csv, step_holonomic, write_candidate_csv

try:
    import matplotlib  # noqa: F401
    HAVE_MATPLOTLIB = True
except ImportError:
    HAVE_MATPLOTLIB = False


START = (0.5, -0.3, 0.4)
T0 = 1000.0
MOCAP_HZ = 50.0


def _state_at(commands, elapsed):
    state = START
    remaining = max(0.0, elapsed)
    for command in commands:
        duration = min(command.dt, remaining)
        state = step_holonomic(*state, command.vx, command.vy, command.omega, duration)
        remaining -= duration
        if remaining <= 0.0:
            break
    return state


def _fake_collect(candidate: Path, run_dir: Path) -> None:
    """Write the raw files a perfect robot and mocap would have produced."""
    commands = load_candidate_csv(candidate)
    run_dir.mkdir(parents=True)
    with (run_dir / "commands.csv").open("w", newline="") as target:
        writer = csv.writer(target)
        writer.writerow(["k", "t_send", "vx", "vy", "omega", "dt"])
        for command in commands:
            writer.writerow([command.k, T0 + command.k*command.dt, command.vx,
                             command.vy, command.omega, command.dt])
    total = sum(command.dt for command in commands)
    with (run_dir / "mocap.csv").open("w", newline="") as target:
        writer = csv.writer(target)
        writer.writerow(["t", "x", "y", "theta_raw", "tracked"])
        for index in range(round((total + 1.0)*MOCAP_HZ) + 1):
            timestamp = T0 - 0.5 + index/MOCAP_HZ
            writer.writerow([timestamp, *_state_at(commands, timestamp - T0), 1])


class Phase1ReportTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = Path(directory.name) / "phase1"
        self.root = root
        patches = [
            mock.patch.object(config, "PHASE1_ROOT", root),
            mock.patch.object(config, "PROBLEMS_CSV", root / "problems.csv"),
            mock.patch.object(config, "NOTES_FILE", root / "notes.txt"),
            mock.patch.object(config, "CANDIDATES_PER_INSTANCE", 2),
            mock.patch.object(config, "REQUIRE_OPERATOR_CONFIRMATION", False),
            mock.patch.object(config, "AUTONOMOUS_RESET_ENABLED", False),
            # Synthetic data has no marker offset, whatever the lab calibration is.
            mock.patch.multiple(config, YAW_OFFSET_RAD=0.0, MARKER_TO_BODY_X_M=0.0,
                                MARKER_TO_BODY_Y_M=0.0, MODEL_GAIN_VX=1.0,
                                MODEL_GAIN_VY=1.0, MODEL_GAIN_OMEGA=1.0),
            mock.patch.object(main, "PLAN_ROOT", root / "plans"),
            mock.patch.object(main, "RAW_ROOT", root / "raw"),
            mock.patch.object(main, "VALIDATION_CSV", root / "collection_validation.csv"),
            mock.patch.object(main, "RESET_ROOT", root / "resets"),
            mock.patch.object(main, "load_phase0", return_value=0.2),
            mock.patch("hardware.collector.collect", side_effect=_fake_collect),
            mock.patch("hardware.reset_controller.autonomous_return"),
            mock.patch("hardware.start_check.wait_for_start", return_value=START),
        ]
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)
        for number, vy in ((1, 0.1), (2, -0.1)):
            commands = [Command(k, 0.3, vy, 0.2, config.DT_S)
                        for k in range(config.T_SEGMENTS)]
            write_candidate_csv(root / "plans" / "I001" / f"C{number:02d}.csv", commands)
        obstacles = '[{"x":1.5,"y":1.0,"radius":0.2}]'
        with config.PROBLEMS_CSV.open("w", newline="") as target:
            writer = csv.writer(target)
            writer.writerow(["instance", "start_x", "start_y", "start_theta", "goal_x",
                             "goal_y", "goal_radius_m", "robot_radius_m", "obstacles",
                             "rho_m", "status", "note"])
            for instance in ("I001", "I002"):
                writer.writerow([instance, *START, 2.0, 2.0, 0.5, 0.3, obstacles,
                                 0.2, "planned", ""])

    def _collect(self, instance_id: str) -> str:
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            main.collect(instance_id, execute=True)
        return output.getvalue()

    def _status(self, instance_id: str) -> str:
        with config.PROBLEMS_CSV.open(newline="") as source:
            return next(row["status"] for row in csv.DictReader(source)
                        if row["instance"] == instance_id)

    def test_finished_instance_is_analyzed_immediately(self):
        output = self._collect("I001")
        self.assertEqual(self._status("I001"), "ok")
        self.assertIn("I001_C01: E = 0.000 m", output)
        self.assertIn("I001 done: largest E over 2 candidates", output)
        with (self.root / "analysis" / "instance_metrics.csv").open(newline="") as source:
            rows = list(csv.DictReader(source))
        self.assertEqual([row["instance"] for row in rows], ["I001"])
        self.assertLess(float(rows[0]["max_candidate_deviation_m"]), 1e-3)

    @unittest.skipUnless(HAVE_MATPLOTLIB, "matplotlib is not installed")
    def test_finished_instance_is_plotted(self):
        output = self._collect("I001")
        self.assertIn("Saved 2 trajectory plot(s)", output)
        for candidate in ("C01", "C02"):
            plot = self.root / "analysis" / "plots" / f"I001_{candidate}.png"
            self.assertGreater(plot.stat().st_size, 0)

    @unittest.skipUnless(HAVE_MATPLOTLIB, "matplotlib is not installed")
    def test_uncollected_instance_plots_planned_paths(self):
        write_candidate_csv(self.root / "plans" / "I002" / "C01.csv",
                            [Command(k, 0.3, 0.0, 0.0, config.DT_S)
                             for k in range(config.T_SEGMENTS)])
        paths = plot_phase1.plot_instance("I002")
        self.assertEqual([path.name for path in paths], ["I002_C01.png"])
        self.assertGreater(paths[0].stat().st_size, 0)

    def test_missing_matplotlib_keeps_instance_ok(self):
        with mock.patch.object(main, "plot_instance",
                               side_effect=ImportError("No module named 'matplotlib'")):
            output = self._collect("I001")
        self.assertEqual(self._status("I001"), "ok")
        self.assertIn("I001 done", output)
        self.assertIn("Trajectory plots skipped", output)

    def test_analysis_failure_keeps_instance_ok(self):
        with mock.patch.object(main, "analyze", side_effect=RuntimeError("disk full")):
            output = self._collect("I001")
        self.assertEqual(self._status("I001"), "ok")
        self.assertIn("analysis failed: disk full", output)
        self.assertTrue((self.root / "raw" / "I001_C01").is_dir())


if __name__ == "__main__":
    unittest.main()
