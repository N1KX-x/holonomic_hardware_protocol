import csv
import random
import tempfile
import unittest
from collections import Counter
from pathlib import Path
from unittest import mock

import collect_training
import config
from core import nominal_step

MIDDLE = ((config.WORKSPACE_X_MIN_M + config.WORKSPACE_X_MAX_M) / 2,
          (config.WORKSPACE_Y_MIN_M + config.WORKSPACE_Y_MAX_M) / 2, 0.3)


class TrialNumberTests(unittest.TestCase):
    def test_next_number_counts_discarded_trials_and_plans(self):
        with tempfile.TemporaryDirectory() as directory:
            raw, plans = Path(directory) / "raw", Path(directory) / "plans"
            self.assertEqual(collect_training.next_trial_number(raw, plans), 1)
            (raw / "trial_001").mkdir(parents=True)
            (raw / "trial_002_discarded").mkdir()
            self.assertEqual(collect_training.next_trial_number(raw, plans), 3)
            plans.mkdir()
            (plans / "trial_003.csv").write_text("k,vx,vy,omega,dt\n")
            self.assertEqual(collect_training.next_trial_number(raw, plans), 4)

    def test_every_block_has_the_configured_mix(self):
        size = config.TRAINING_MIXED_PER_BLOCK + 3
        for block in range(4):
            kinds = Counter(collect_training.motion_type(block * size + i) for i in range(1, size + 1))
            self.assertEqual(kinds["mixed"], config.TRAINING_MIXED_PER_BLOCK)
            for kind in collect_training.SINGLE_AXIS:
                self.assertEqual(kinds[kind], 1)


class PlanTests(unittest.TestCase):
    def test_plans_stay_inside_and_use_only_their_axes(self):
        for kind in ("mixed", *collect_training.SINGLE_AXIS):
            for seed in range(5):
                with self.subTest(kind=kind, seed=seed):
                    commands = collect_training.plan_trial(kind, MIDDLE, random.Random(seed))
                    self.assertEqual(len(commands), config.TRAINING_SEGMENTS)
                    pose = MIDDLE
                    for command in commands:
                        pose = nominal_step(*pose, command, config)
                        self.assertTrue(collect_training._inside(pose[0], pose[1]))
                    if kind == "vx_only":
                        self.assertTrue(all(c.vy == 0 and c.omega == 0 for c in commands))
                    elif kind == "vy_only":
                        self.assertTrue(all(c.vx == 0 and c.omega == 0 for c in commands))
                    elif kind == "omega_only":
                        self.assertTrue(all(c.vx == 0 and c.vy == 0 for c in commands))

    def test_start_near_the_edge_is_refused(self):
        edge = (config.WORKSPACE_X_MAX_M - 0.01, MIDDLE[1], 0.0)
        with self.assertRaises(ValueError):
            collect_training.plan_trial("mixed", edge, random.Random(0))


class CollectTests(unittest.TestCase):
    def test_failed_trial_is_discarded_and_its_number_is_not_reused(self):
        def failing_run(candidate, run_dir):
            run_dir.mkdir(parents=True)
            (run_dir / "commands.csv").write_text("k,t_send,vx,vy,omega,dt\n0,1.0,0.1,0,0,2.0\n")
            raise RuntimeError("Safety stop: test")

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            paths = {"PLAN_DIR": root / "plans", "RAW_DIR": root / "raw",
                     "TRIALS_CSV": root / "trials.csv", "NOTES_FILE": root / "notes.txt"}
            old_allow = config.ALLOW_HARDWARE_EXECUTION
            config.ALLOW_HARDWARE_EXECUTION = True
            try:
                with mock.patch.multiple(collect_training, **paths), \
                     mock.patch("hardware.start_check.read_pose", return_value=MIDDLE), \
                     mock.patch("hardware.collector.collect", side_effect=failing_run), \
                     mock.patch("builtins.input", return_value="trial_001"):
                    with self.assertRaises(RuntimeError):
                        collect_training.collect(execute=True)
                    self.assertEqual(collect_training.next_trial_number(), 2)
            finally:
                config.ALLOW_HARDWARE_EXECUTION = old_allow
            self.assertTrue((root / "raw" / "trial_001_discarded").is_dir())
            self.assertFalse((root / "raw" / "trial_001").exists())
            with (root / "trials.csv").open(newline="") as source:
                row = next(csv.DictReader(source))
            self.assertEqual(row["status"], "discarded")
            self.assertEqual(row["segments_sent"], "1")
            self.assertIn("Safety stop", (root / "notes.txt").read_text())


if __name__ == "__main__":
    unittest.main()
