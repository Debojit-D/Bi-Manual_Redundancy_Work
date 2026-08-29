import argparse
from contextlib import redirect_stderr, redirect_stdout
import io
import unittest

from bimanual_redundancy.simulation.cli import add_camera_view_arguments, run_cli


class SharedCliTests(unittest.TestCase):
    def test_camera_flags_are_mutually_exclusive(self):
        parser = argparse.ArgumentParser()
        add_camera_view_arguments(parser)

        front = parser.parse_args(["--front-view"])
        self.assertTrue(front.front_view)
        self.assertFalse(front.top_view)

        top = parser.parse_args(["--top-view"])
        self.assertTrue(top.top_view)
        self.assertFalse(top.front_view)

        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parser.parse_args(["--front-view", "--top-view"])

    def test_ctrl_c_runs_cleanup_and_exits_with_status_130(self):
        events = []

        def interrupted_main():
            try:
                raise KeyboardInterrupt
            finally:
                events.append("cleanup")

        output = io.StringIO()
        with redirect_stdout(output), self.assertRaises(SystemExit) as raised:
            run_cli(interrupted_main)

        self.assertEqual(events, ["cleanup"])
        self.assertEqual(raised.exception.code, 130)
        self.assertIn("shutdown complete", output.getvalue())


if __name__ == "__main__":
    unittest.main()
