import tempfile
import unittest
from pathlib import Path
from unittest import mock

import config
import phase0
from core import Command, validate_commands, write_candidate_csv
from hardware import collector


class Phase0PlanTests(unittest.TestCase):
    def test_every_prepared_plan_passes_runner_validation(self):
        plans = phase0._build_plans()
        for run_id, (_, commands) in plans.items():
            with self.subTest(run_id=run_id):
                validate_commands(commands, config,
                                  require_main_horizon=run_id.startswith("pilot_"))

    def test_single_axis_pilots_respect_combined_translation_limit(self):
        old_limit = config.TRANSLATIONAL_SPEED_MAX_MPS
        config.TRANSLATIONAL_SPEED_MAX_MPS = 0.3
        try:
            plans = phase0._build_plans()
        finally:
            config.TRANSLATIONAL_SPEED_MAX_MPS = old_limit
        for run_id, (group, commands) in plans.items():
            if group in ("vx_only", "vy_only"):
                for command in commands:
                    self.assertLessEqual(abs(command.vx), 0.3, run_id)
                    self.assertLessEqual(abs(command.vy), 0.3, run_id)


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
