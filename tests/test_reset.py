import unittest

import config
from core import step_holonomic
from hardware.reset_controller import return_command


class ResetControllerTests(unittest.TestCase):
    def test_small_pose_error_is_inverted_by_nominal_step(self):
        pose = (0.0, 0.0, 0.2)
        target = (0.10, -0.05, 0.30)
        command = return_command(pose, target)
        reached = step_holonomic(*pose, command.vx, command.vy,
                                 command.omega, command.dt)
        self.assertAlmostEqual(reached[0], target[0], places=8)
        self.assertAlmostEqual(reached[1], target[1], places=8)
        self.assertAlmostEqual(reached[2], target[2], places=8)

    def test_autonomous_reset_flag_is_boolean(self):
        self.assertIsInstance(config.AUTONOMOUS_RESET_ENABLED, bool)


if __name__ == "__main__":
    unittest.main()
