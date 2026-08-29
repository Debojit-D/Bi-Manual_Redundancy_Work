"""Small command-line interface for reproduction and robot validation."""

from __future__ import annotations

import argparse
from pathlib import Path

from bimanual_redundancy.paper_reproduction import reproduce_paper, run_config
from bimanual_redundancy.systems import (
    SYSTEM_SPECS,
    get_cooperative_system_spec,
    validate_cooperative_system_spec,
)


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
    validate_robot = commands.add_parser(
        "validate-robot", help="validate a registered cooperative robot"
    )
    validate_robot.add_argument(
        "--robot", required=True, choices=tuple(SYSTEM_SPECS)
    )
    return parser


def main(argv=None) -> int:
    arguments = build_parser().parse_args(argv)
    if arguments.command == "validate-robot":
        result = validate_cooperative_system_spec(
            get_cooperative_system_spec(arguments.robot)
        )
        print(
            f"Robot {result['identifier']} is valid: "
            f"{result['controlled_joint_count']} controlled joints; "
            f"qpos={result['qpos_indices']}; dofs={result['dof_indices']}"
        )
        return 0
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
