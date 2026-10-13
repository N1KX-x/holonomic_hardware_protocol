import contextlib
import csv
import io
import random
import shutil
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import config
import phase0analyze
from core import Command, ExecutionTrace, nominal_step, write_candidate_csv
from fake_run import NO_CALIBRATION, RAW_MODEL, write_run


# The simulated robot: slower forward and to its right, with small offsets.
TRUE_MODEL = dict(RAW_MODEL,
                  MODEL_GAIN_VX_POS=0.85, MODEL_OFFSET_VX_POS_M=-0.02,
                  MODEL_GAIN_VY_POS=1.00, MODEL_OFFSET_VY_POS_M=0.01,
                  MODEL_GAIN_VY_NEG=0.70, MODEL_OFFSET_VY_NEG_M=0.00,
                  MODEL_GAIN_OMEGA_POS=0.80, MODEL_OFFSET_OMEGA_POS_RAD=-0.01)
START = (0.5, -0.3, 0.4)


def _commands(rng: random.Random, segments: int) -> list[Command]:
    """Planner-like commands: forward-only vx, no right turns, some zero components."""
    def draw(low, high):
        return 0.0 if rng.random() < 0.25 else rng.uniform(low, high)
    return [Command(k, draw(0.0, config.VX_MAX_MPS),
                    draw(-config.VY_MAX_MPS, config.VY_MAX_MPS),
                    draw(0.0, config.OMEGA_MAX_RADPS), config.DT_S)
            for k in range(segments)]


def _trace(commands: list[Command]) -> ExecutionTrace:
    robot = SimpleNamespace(**TRUE_MODEL)
    state, actual = START, [START[:2]]
    for command in commands:
        state = nominal_step(*state, command, robot)
        actual.append(state[:2])
    return ExecutionTrace(commands, START, [], actual, [])


class FitTests(unittest.TestCase):
    def setUp(self):
        rng = random.Random(3)
        self.traces = [_trace(_commands(rng, 12)) for _ in range(8)]

    def test_perfect_model_has_zero_q(self):
        self.assertAlmostEqual(phase0analyze.quantile_e(self.traces, TRUE_MODEL), 0.0)
        self.assertGreater(phase0analyze.quantile_e(self.traces, RAW_MODEL), 0.1)

    def test_q_is_the_high_quantile_of_the_pilot_deviations(self):
        model = SimpleNamespace(**RAW_MODEL)
        deviations = sorted(phase0analyze.max_deviation(trace, model) for trace in self.traces)
        with mock.patch.object(config, "PILOT_HIGH_QUANTILE", 0.5):
            self.assertAlmostEqual(phase0analyze.quantile_e(self.traces, RAW_MODEL),
                                   (deviations[3] + deviations[4]) / 2)

    def test_q_search_gives_up_only_when_the_limit_cannot_be_beaten(self):
        model = SimpleNamespace(**RAW_MODEL)
        smallest = min(phase0analyze.max_deviation(trace, model) for trace in self.traces)
        q = phase0analyze.quantile_e(self.traces, RAW_MODEL)
        # A limit q can beat never cuts the evaluation short.
        self.assertEqual(phase0analyze.quantile_e(self.traces, RAW_MODEL, q * 1.0001), q)
        # A limit it cannot beat gives q itself or inf, never something below the limit.
        self.assertGreaterEqual(phase0analyze.quantile_e(self.traces, RAW_MODEL, q), q)
        self.assertEqual(phase0analyze.quantile_e(self.traces, RAW_MODEL, smallest),
                         float("inf"))

    def test_random_search_finds_the_gains_and_offsets_that_minimize_q(self):
        with mock.patch.multiple(config, **RAW_MODEL):
            fitted = phase0analyze.fit(self.traces, random.Random(1), 6000)
        raw = phase0analyze.quantile_e(self.traces, RAW_MODEL)
        self.assertLess(phase0analyze.quantile_e(self.traces, fitted), 0.1 * raw)
        for name in ("MODEL_GAIN_VX_POS", "MODEL_GAIN_VY_NEG", "MODEL_GAIN_OMEGA_POS"):
            self.assertAlmostEqual(fitted[name], TRUE_MODEL[name], delta=0.08)

    def test_direction_without_data_keeps_its_config_values(self):
        untouched = dict(MODEL_GAIN_OMEGA_NEG=0.77, MODEL_OFFSET_OMEGA_NEG_RAD=0.003,
                         MODEL_GAIN_VX_NEG=0.91)
        self.assertEqual(phase0analyze.segment_counts(self.traces)["-omega"], 0)
        with mock.patch.multiple(config, **untouched):
            fitted = phase0analyze.fit(self.traces, random.Random(1), 200)
        for name, value in untouched.items():
            self.assertEqual(fitted[name], value)

    def test_same_seed_gives_the_same_fit(self):
        first = phase0analyze.fit(self.traces, random.Random(5), 300)
        second = phase0analyze.fit(self.traces, random.Random(5), 300)
        self.assertEqual(first, second)


class AnalyzeTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name) / "phase0"
        self.config_text = Path(config.__file__).read_text()
        patches = [
            mock.patch.object(config, "PHASE0_ROOT", self.root),
            mock.patch.object(config, "PHASE0_RESULTS_CSV", self.root / "phase0_results.csv"),
            mock.patch.object(config, "PHASE0_FIT_ITERATIONS", 3000),
            mock.patch.multiple(config, **NO_CALIBRATION, **RAW_MODEL),
            mock.patch.object(phase0analyze, "METRICS_CSV", self.root / "execution_metrics.csv"),
            mock.patch.object(phase0analyze, "REPORT_TXT", self.root / "phase0_analysis.txt"),
        ]
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)
        rng = random.Random(11)
        for number in range(1, 5):
            pilot = self.root / f"pilot_{number:03d}"
            write_candidate_csv(pilot / "plan.csv", _commands(rng, config.T_SEGMENTS))
            write_run(pilot / "plan.csv", pilot / "raw", START, TRUE_MODEL)

    def _analyze(self) -> tuple[str, dict]:
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            phase0analyze.analyze()
        with config.PHASE0_RESULTS_CSV.open(newline="") as source:
            return output.getvalue(), next(csv.DictReader(source))

    def test_e_q_and_rho_are_computed_with_the_fitted_model(self):
        raw_q = phase0analyze.quantile_e(list(phase0analyze.load_pilots()[0].values()), RAW_MODEL)
        output, row = self._analyze()
        self.assertLess(float(row["q_estimate_m"]), 0.2 * raw_q)
        self.assertEqual(int(row["pilot_count"]), 4)
        self.assertAlmostEqual(float(row["rho_m"]),
                               config.RHO_MULTIPLIER * float(row["q_estimate_m"]))
        self.assertLessEqual(float(row["q_estimate_m"]), float(row["max_e_m"]))
        with (self.root / "execution_metrics.csv").open(newline="") as source:
            metrics = list(csv.DictReader(source))
        self.assertEqual([item["run_id"] for item in metrics],
                         ["pilot_001", "pilot_002", "pilot_003", "pilot_004"])
        self.assertAlmostEqual(max(float(item["max_deviation_m"]) for item in metrics),
                               float(row["max_e_m"]))
        for expected in ("+vx", "-omega", "q (90% quantile of E)", "rho ="):
            self.assertIn(expected, output)

    def test_report_lists_the_config_lines_and_config_is_left_alone(self):
        output, row = self._analyze()
        report = (self.root / "phase0_analysis.txt").read_text()
        self.assertIn(report, output)
        for name in phase0analyze.FITTED_NAMES:
            # The text file and the results file hold the same numbers.
            self.assertIn(f"{name} = {float(row[name.lower()])!r}\n", report)
            self.assertEqual(getattr(config, name), RAW_MODEL[name])
        self.assertNotEqual(float(row["model_gain_vx_pos"]), RAW_MODEL["MODEL_GAIN_VX_POS"])
        self.assertEqual(Path(config.__file__).read_text(), self.config_text)

    def test_status_is_ready_only_when_rho_fits_inside_the_goal(self):
        self.assertEqual(self._analyze()[1]["status"], "ready")
        with mock.patch.object(config, "RHO_MULTIPLIER", 1e6):
            output, row = self._analyze()
        self.assertEqual(row["status"], "pending")
        self.assertIn("rho must be below", output)

    def test_unusable_pilot_is_reported_and_left_out(self):
        shutil.rmtree(self.root / "pilot_003" / "raw")
        output, row = self._analyze()
        self.assertEqual(int(row["pilot_count"]), 3)
        self.assertEqual(row["status"], "pending")
        self.assertIn("invalid:pilot_003", row["failures"])
        self.assertIn("Not used: invalid:pilot_003", output)

    def test_no_pilots_is_an_error(self):
        shutil.rmtree(self.root)
        with self.assertRaisesRegex(RuntimeError, "No valid pilots"):
            phase0analyze.analyze()
        self.assertFalse((self.root / "phase0_analysis.txt").exists())


if __name__ == "__main__":
    unittest.main()
