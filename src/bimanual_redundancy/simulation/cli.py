"""Shared command-line behavior for interactive MuJoCo scripts."""


def add_camera_view_arguments(parser, *, scope="run"):
    """Add mutually exclusive top/front camera switches to a parser."""
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--top-view",
        action="store_true",
        help=f"use a full overhead camera for the {scope}",
    )
    group.add_argument(
        "--front-view",
        action="store_true",
        help=f"use a straight-on front camera for the {scope}",
    )
    return group


def run_cli(main):
    """Run a CLI entry point and convert Ctrl+C into a clean exit status."""
    try:
        return main()
    except KeyboardInterrupt:
        print("\nInterrupted by Ctrl+C. Resources closed; shutdown complete.")
        raise SystemExit(130) from None
