"""Small command-line interface for reproduction and robot validation."""

from __future__ import annotations

import argparse
from pathlib import Path

from bimanual_redundancy.paper_reproduction import reproduce_paper, run_config


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="bimanual-redopt")
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run", help="run one TOML experiment campaign")
    run.add_argument("--config", type=Path, required=True)
    run.add_argument("--output-root", type=Path)
    run.add_argument("--smoke", action="store_true")
    reproduce = commands.add_parser(
        "reproduce-paper", help="run the complete configured paper campaign"
    )
    reproduce.add_argument("--output-root", type=Path)
    reproduce.add_argument("--smoke", action="store_true")
    return parser


def main(argv=None) -> int:
    arguments = build_parser().parse_args(argv)
    if arguments.command == "run":
        output = run_config(
            arguments.config,
            output_root=arguments.output_root,
            smoke=arguments.smoke,
        )
    else:
        output = reproduce_paper(
            output_root=arguments.output_root,
            smoke=arguments.smoke,
        )
    print(f"Paper reproduction output: {output}")
    if arguments.smoke:
        print("Smoke mode verified the pipeline; it does not reproduce publication statistics.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
