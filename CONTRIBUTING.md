# Contributing

This project accompanies a peer-reviewed manuscript. The priority is keeping
the paper's reported results reproducible while the framework grows beyond
the paper's original scope.

## Setup

Follow [Quick start](README.md#quick-start) in the README, then install the
test extra:

```bash
uv pip install -e ".[dev]"
pytest
```

See [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) if setup doesn't
work as documented.

## Tests

New code ships with tests. Tests exercising a manuscript equation reference
the equation number (see `tests/test_paper_equations.py`) and use a
tolerance no looser than the paper implies.

## Branch naming

`refactor/NN-short-description` for structural/documentation passes,
`feature/short-description` or `fix/short-description` for functionality
and bug fixes.

## Adding a robot

Cooperative robot embodiments are added through `CooperativeSystemSpec`. See
[`docs/ADDING_A_ROBOT.md`](docs/ADDING_A_ROBOT.md) for the interface, the
assumption-classification table, and the validation command
(`bimanual-redopt validate-robot --robot <name>`).

## Adding an objective

New manipulability/task objectives belong in `core/objectives.py`, following
the existing pattern of a raw cost, a scaled cost, and a null-space gradient.
New mathematics needs:

- unit tests exercising the objective directly, not only through an
  end-to-end experiment run;
- a citation to the paper equation, appendix, or external reference it
  implements, in a docstring or comment;
- an entry in [`docs/PAPER_CODE_MAP.md`](docs/PAPER_CODE_MAP.md) if it
  corresponds to a manuscript equation.

## Preserving paper configs

The TOML files under `configs/paper/` define what "reproduces the paper"
means for this repository (see
[`docs/REPRODUCING_THE_PAPER.md`](docs/REPRODUCING_THE_PAPER.md)). Don't
change their objectives, durations, gains, or robot bases as a side effect of
unrelated work. If a config genuinely needs to change, explain in the PR why
the manuscript's reported results are unaffected.

## Keeping the mathematical core generic

`core/` implements the manuscript's mathematics and stays free of
robot-specific, backend-specific (MuJoCo), or experiment-specific
assumptions; those live in `simulation/` and `experiments/` (see
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)). Mink is used as an external
IK dependency, not a required interface for the core math, and is not
vendored or forked here. Changes that couple `core/` to one robot, one
backend, or one IK library will generally be asked to move.

## Licensing and attribution

Original code is Apache-2.0 (see [`LICENSE`](LICENSE)); third-party
components are catalogued in
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md). Contributors must only
submit code or assets they have the right to submit, preserve existing
third-party attribution and license headers, identify any imported external
material and its license in the PR description (adding it to
`THIRD_PARTY_NOTICES.md`), and avoid introducing dependencies with licenses
incompatible with Apache-2.0 (for example strong-copyleft licenses) without
first discussing it in an issue or PR. No contributor license agreement is
required.
