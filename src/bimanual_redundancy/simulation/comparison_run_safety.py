"""Failure isolation and terminal warnings for comparison sweeps."""


ANSI_BRIGHT_RED = "\033[1;91m"
ANSI_RESET = "\033[0m"
RUN_FAILURE_MARKER = "COMPARISON_RUN_FAILED:"
INCOMPLETE_SWEEP_MARKER = "COMPARISON_SWEEP_INCOMPLETE:"


def print_run_failure(run_label, error):
    """Print one machine-detectable red warning for a failed run."""
    print(
        f"{ANSI_BRIGHT_RED}{RUN_FAILURE_MARKER} {run_label}: "
        f"{type(error).__name__}: {error}. "
        "Please re-record this run; continuing with the remaining runs."
        f"{ANSI_RESET}",
        flush=True,
    )


def print_sweep_summary(total_runs, failed_runs):
    """Report whether every attempted comparison run completed."""
    failed_runs = tuple(failed_runs)
    if not failed_runs:
        print(f"\nCompleted all {total_runs} comparison runs.")
        return
    labels = ", ".join(failed_runs)
    print(
        f"\n{ANSI_BRIGHT_RED}{INCOMPLETE_SWEEP_MARKER} "
        f"{len(failed_runs)} of {total_runs} run(s) need re-recording: "
        f"{labels}.{ANSI_RESET}",
        flush=True,
    )
