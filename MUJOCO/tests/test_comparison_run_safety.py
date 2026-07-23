from contextlib import redirect_stdout
import io
import unittest

from MUJOCO.utils.comparison_run_safety import (
    ANSI_BRIGHT_RED,
    ANSI_RESET,
    INCOMPLETE_SWEEP_MARKER,
    RUN_FAILURE_MARKER,
    print_run_failure,
    print_sweep_summary,
)


class ComparisonRunSafetyTests(unittest.TestCase):
    def test_failure_warning_is_red_and_requests_rerecord(self):
        output = io.StringIO()
        with redirect_stdout(output):
            print_run_failure("pose_4 / force", RuntimeError("retreat timeout"))

        message = output.getvalue()
        self.assertIn(ANSI_BRIGHT_RED, message)
        self.assertIn(ANSI_RESET, message)
        self.assertIn(RUN_FAILURE_MARKER, message)
        self.assertIn("pose_4 / force", message)
        self.assertIn("Please re-record this run", message)
        self.assertIn("continuing with the remaining runs", message)

    def test_incomplete_summary_lists_failed_runs_in_red(self):
        output = io.StringIO()
        with redirect_stdout(output):
            print_sweep_summary(
                24,
                ("pose_4 / force", "pose_5 / velocity"),
            )

        message = output.getvalue()
        self.assertIn(ANSI_BRIGHT_RED, message)
        self.assertIn(INCOMPLETE_SWEEP_MARKER, message)
        self.assertIn("2 of 24", message)
        self.assertIn("pose_4 / force", message)
        self.assertIn("pose_5 / velocity", message)


if __name__ == "__main__":
    unittest.main()
