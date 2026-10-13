import contextlib
import io
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import config
import phase0
from core import Command, load_candidate_csv, nominal_step, write_candidate_csv
from fake_run import NO_CALIBRATION, RAW_MODEL, write_run
from hardware import collector
from planner import Problem


try:
    import matplotlib  # noqa: F401
    HAVE_MATPLOTLIB = True
except ImportError:
    HAVE_MATPLOTLIB = False

START = (0.5, -0.3, 0.4)
COMMANDS = [Command(k, 0.3, 0.1 if k % 2 else -0.1, 0.2, config.DT_S)
            for k in range(config.T_SEGMENTS)]


def _far_from_goal(x_fraction: float, y_fraction: float, turn: float):
    """A pose inside the workspace, mirrored if needed to be outside the goal region.

    It faces `turn` radians away from the goal. Facing straight away is avoided:
    forward-only plans from there need many turning segments and often fail.
    """
    def position(fx, fy):
        return (config.WORKSPACE_X_MIN_M + fx*(config.WORKSPACE_X_MAX_M - config.WORKSPACE_X_MIN_M),
                config.WORKSPACE_Y_MIN_M + fy*(config.WORKSPACE_Y_MAX_M - config.WORKSPACE_Y_MIN_M))
    def distance(p):
        return math.hypot(p[0] - config.GOAL_X_M, p[1] - config.GOAL_Y_M)
    x, y = max(position(x_fraction, y_fraction), position(1 - x_fraction, 1 - y_fraction),
               key=distance)
    return x, y, math.atan2(config.GOAL_Y_M - y, config.GOAL_X_M - x) + turn


class PilotNumberTests(unittest.TestCase):
    def test_next_number_is_one_more_than_the_highest_saved_pilot(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "phase0"
            self.assertEqual(phase0.next_pilot_number(root), 1)
            (root / "pilot_001").mkdir(parents=True)
            (root / "pilot_004").mkdir()
            (root / "phase0_results.csv").write_text("")
            self.assertEqual(phase0.next_pilot_number(root), 5)
            self.assertEqual([path.name for path in phase0.pilot_dirs(root)],
                             ["pilot_001", "pilot_004"])


class PilotPlanTests(unittest.TestCase):
    def _plan(self, start):
        for seed in range(1, 6):
            try:
                return phase0.plan_pilot_run(1, start, seed)
            except RuntimeError:
                continue
        self.fail("RRT found no path from this pose with any of five seeds")

    def test_plan_runs_from_the_given_pose_to_the_goal_region(self):
        for start in (_far_from_goal(0.2, 0.2, 1.2), _far_from_goal(0.5, 0.1, -0.8)):
            with self.subTest(start=start):
                problem, commands = self._plan(start)
                self.assertEqual((problem.start_x, problem.start_y, problem.start_theta), start)
                self.assertEqual((problem.goal_x, problem.goal_y),
                                 (config.GOAL_X_M, config.GOAL_Y_M))
                self.assertEqual(len(commands), config.T_SEGMENTS)
                self.assertTrue(all(command.vx >= 0.0 for command in commands))
                pose = start
                for command in commands:
                    pose = nominal_step(*pose, command, config)
                    self.assertTrue(config.WORKSPACE_X_MIN_M <= pose[0] <= config.WORKSPACE_X_MAX_M)
                    self.assertTrue(config.WORKSPACE_Y_MIN_M <= pose[1] <= config.WORKSPACE_Y_MAX_M)
                self.assertLessEqual(math.hypot(pose[0] - problem.goal_x, pose[1] - problem.goal_y),
                                     config.GOAL_RADIUS_M)

    def test_a_start_on_the_boundary_itself_is_allowed(self):
        corner = _far_from_goal(0.0, 0.0, 0.5)
        problem, _ = self._plan(corner)
        self.assertEqual((problem.start_x, problem.start_y), corner[:2])

    def test_start_outside_the_boundary_or_inside_the_goal_is_refused(self):
        with self.assertRaisesRegex(RuntimeError, "outside the boundary"):
            phase0.plan_pilot_run(1, (config.WORKSPACE_X_MAX_M + 0.01, 0.0, 0.0), 1)
        with self.assertRaisesRegex(RuntimeError, "already"):
            phase0.plan_pilot_run(1, (config.GOAL_X_M, config.GOAL_Y_M, 0.0), 1)


class PilotCollectTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name) / "phase0"
        self.hardware = mock.Mock(side_effect=lambda candidate, run_dir:
                                  write_run(candidate, run_dir, START))
        problem = Problem(1, *START, 2.0, 2.0, (), 7)
        self.plan = mock.Mock(return_value=(problem, COMMANDS))
        patches = [
            mock.patch.object(config, "PHASE0_ROOT", self.root),
            mock.patch.object(config, "ALLOW_HARDWARE_EXECUTION", True),
            mock.patch.object(config, "REQUIRE_OPERATOR_CONFIRMATION", False),
            mock.patch.multiple(config, **NO_CALIBRATION, **RAW_MODEL),
            mock.patch.object(phase0, "plan_pilot_run", self.plan),
            mock.patch("hardware.collector.collect", self.hardware),
            mock.patch("hardware.start_check.read_pose", return_value=START),
        ]
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)

    def _collect(self, execute: bool = True) -> str:
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            phase0.collect(execute)
        return output.getvalue()

    def _saved(self) -> list[str]:
        return [path.name for path in phase0.pilot_dirs(self.root)]

    def test_each_run_saves_the_next_pilot_with_its_commands_and_raw_data(self):
        self.assertIn("pilot_001 saved", self._collect())
        self.assertIn("pilot_002 saved", self._collect())
        self.assertEqual(self._saved(), ["pilot_001", "pilot_002"])
        pilot = self.root / "pilot_002"
        self.assertEqual(load_candidate_csv(pilot / "plan.csv"), COMMANDS)
        record = json.loads((pilot / "problem.json").read_text())
        self.assertEqual(record["start"], {"x": START[0], "y": START[1], "theta": START[2]})
        self.assertEqual(record["goal"]["radius_m"], config.GOAL_RADIUS_M)
        for name in ("commands.csv", "mocap.csv"):
            self.assertTrue((pilot / "raw" / name).is_file())

    @unittest.skipUnless(HAVE_MATPLOTLIB, "matplotlib is not installed")
    def test_each_run_saves_a_trajectory_plot(self):
        self.assertIn("Trajectory plot:", self._collect())
        self.assertGreater((self.root / "pilot_001" / "trajectory.png").stat().st_size, 0)

    def test_plan_starts_from_the_measured_pose(self):
        self._collect()
        self.assertEqual(self.plan.call_args.args[:2], (1, START))

    def test_failed_planning_saves_nothing_and_keeps_the_number(self):
        self.plan.side_effect = RuntimeError("RRT found no path from this pose")
        with self.assertRaises(RuntimeError):
            self._collect()
        self.assertFalse(self.root.exists())
        self.hardware.assert_not_called()
        self.plan.side_effect = None
        self.assertIn("pilot_001 saved", self._collect())

    def test_failed_run_is_discarded_and_its_number_reused(self):
        self._collect()

        def interrupted(candidate, run_dir):
            write_run(candidate, run_dir, START)
            raise RuntimeError("Safety stop: mocap lost the robot")
        self.hardware.side_effect = interrupted
        with self.assertRaises(RuntimeError):
            self._collect()
        self.assertEqual(self._saved(), ["pilot_001"])
        self.hardware.side_effect = KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            self._collect()
        self.assertEqual(self._saved(), ["pilot_001"])
        self.hardware.side_effect = lambda candidate, run_dir: write_run(candidate, run_dir, START)
        self.assertIn("pilot_002 saved", self._collect())

    def test_run_with_unusable_mocap_is_discarded(self):
        def cut_short(candidate, run_dir):
            write_run(candidate, run_dir, START)
            lines = (run_dir / "mocap.csv").read_text().splitlines(keepends=True)
            (run_dir / "mocap.csv").write_text("".join(lines[:len(lines) // 2]))
        self.hardware.side_effect = cut_short
        with self.assertRaisesRegex(ValueError, "Mocap does not cover"):
            self._collect()
        self.assertEqual(self._saved(), [])

    def test_preview_moves_nothing_and_saves_nothing(self):
        self.assertIn("Preview only", self._collect(execute=False))
        self.hardware.assert_not_called()
        self.assertFalse(self.root.exists())

    def test_folder_of_the_earlier_phase0_is_refused(self):
        self.root.mkdir(parents=True)
        (self.root / phase0.OLD_PHASE0_MARKER).write_text("run_id\n")
        with self.assertRaisesRegex(RuntimeError, "earlier Phase 0"):
            self._collect()
        self.assertEqual(self._saved(), [])


class CollectorValidationTests(unittest.TestCase):
    def test_invalid_plan_is_rejected_before_run_folder_exists(self):
        too_fast = config.VX_MAX_MPS + 1.0
        with tempfile.TemporaryDirectory() as directory:
            candidate = Path(directory) / "candidate.csv"
            write_candidate_csv(candidate, [Command(0, too_fast, 0.0, 0.0, config.DT_S)])
            run_dir = Path(directory) / "run"
            old_allow = config.ALLOW_HARDWARE_EXECUTION
            config.ALLOW_HARDWARE_EXECUTION = True
            try:
                with mock.patch.object(collector.subprocess, "Popen",
                                       side_effect=AssertionError("hardware started")):
                    with self.assertRaises(ValueError):
                        collector.collect(candidate, run_dir)
            finally:
                config.ALLOW_HARDWARE_EXECUTION = old_allow
            self.assertFalse(run_dir.exists())


if __name__ == "__main__":
    unittest.main()
