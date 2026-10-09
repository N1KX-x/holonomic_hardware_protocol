import math
import tempfile
import unittest
from pathlib import Path

import config
from hardware.collector import latest_sample, safety_problem
from hardware.start_check import start_offset
from planner import plan_candidate, sample_problem


class FixedCourseTests(unittest.TestCase):
    def test_problem_uses_fixed_start_and_goal_facing_goal(self):
        problem = sample_problem(1, 123, 0.2)
        self.assertEqual((problem.start_x, problem.start_y), (config.START_X_M, config.START_Y_M))
        self.assertEqual((problem.goal_x, problem.goal_y), (config.GOAL_X_M, config.GOAL_Y_M))
        expected = math.atan2(config.GOAL_Y_M - config.START_Y_M,
                              config.GOAL_X_M - config.START_X_M)
        self.assertAlmostEqual(problem.start_theta, expected)

    def test_only_obstacles_change_between_instances(self):
        first, second = sample_problem(1, 123, 0.2), sample_problem(2, 456, 0.2)
        self.assertEqual((first.start_x, first.goal_x), (second.start_x, second.goal_x))
        self.assertNotEqual(first.obstacles, second.obstacles)

    def test_planned_path_starts_from_fixed_start(self):
        problem = sample_problem(1, config.MASTER_RANDOM_SEED + 10000, 0.2)
        self.assertIsNotNone(plan_candidate(problem, config.MASTER_RANDOM_SEED + 10001, 0.2))


class SafetyStopTests(unittest.TestCase):
    def test_center_of_workspace_is_safe(self):
        x = (config.WORKSPACE_X_MIN_M + config.WORKSPACE_X_MAX_M) / 2
        y = (config.WORKSPACE_Y_MIN_M + config.WORKSPACE_Y_MAX_M) / 2
        self.assertIsNone(safety_problem(x, y, 0.0))

    def test_near_edge_stops(self):
        x = config.WORKSPACE_X_MAX_M - config.EDGE_STOP_MARGIN_M / 2
        self.assertIn("workspace edge", safety_problem(x, 0.0, 0.0))

    def test_latest_sample_skips_partial_last_line(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mocap.csv"
            path.write_text("t,x,y,theta_raw,tracked\n"
                            "1.0,0.5,0.6,0.1,1\n"
                            "1.1,0.7,0.8,nan,0\n"
                            "1.2,0.9")
            self.assertEqual(latest_sample(path)[3], False)
            path.write_text("t,x,y,theta_raw,tracked\n1.0,0.5,0.6,0.1,1\n1.2,0.9")
            self.assertEqual(latest_sample(path), (0.5, 0.6, 0.1, True))


class StartCheckTests(unittest.TestCase):
    def test_offset_is_given_in_robot_frame(self):
        # Robot faces +y; the start is 1 m further along +y and 0.5 m toward -x.
        distance, forward, left, turn = start_offset((0.0, 0.0, math.pi/2),
                                                     (-0.5, 1.0, math.pi/2 + 0.1))
        self.assertAlmostEqual(forward, 1.0)
        self.assertAlmostEqual(left, 0.5)
        self.assertAlmostEqual(turn, 0.1)
        self.assertAlmostEqual(distance, math.hypot(0.5, 1.0))


if __name__ == "__main__":
    unittest.main()
