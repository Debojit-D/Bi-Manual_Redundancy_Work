# Contributing

Thanks for your interest in contributing. This project accompanies a
peer-reviewed manuscript, so the priority is keeping the paper's reported
results reproducible while allowing the framework to grow beyond the paper's
original scope.

## Environment setup

Follow the [Quick start](README.md#quick-start) in the README:
`git clone --recurse-submodules`, then `uv venv --python 3.12 .venv`,
activate it, and `uv pip install -e .`. See
[`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) if anything doesn't work
as documented.

## Tests

Install the test dependencies and run the full suite from the repository
root:

```bash
uv pip install -e ".[dev]"
pytest
```

New code should ship with tests. Tests that exercise the manuscript's
equations should reference the equation number they cover (see
`tests/test_paper_equations.py` for the existing convention) and should not
be looser than the tolerance the paper implies.

## Branch naming

This repository uses `refactor/NN-short-description` for structural/
documentation passes (matching the existing history) and
`feature/short-description` / `fix/short-description` for new functionality
and bug fixes.

## Adding a robot

The framework is designed to support cooperative robot embodiments beyond
the reference dual-Franka-Panda setup through `CooperativeSystemSpec`. See
[`docs/ADDING_A_ROBOT.md`](docs/ADDING_A_ROBOT.md) for the extension
interface, the assumption-classification table (math vs. robot-specific vs.
backend-specific vs. experiment-specific vs. collision), and the validation
checklist (`bimanual-redopt validate-robot --robot <name>`).

## Adding an objective

New manipulability/task objectives belong in `core/objectives.py`, following
the existing pattern of a raw cost, a scaled cost, and a null-space gradient.
Any new mathematics must:

- ship with unit tests exercising the objective directly, not just through
  an end-to-end experiment run;
- cite the paper equation, appendix, or external reference it implements, in
  a docstring or code comment; and
- be added to [`docs/PAPER_CODE_MAP.md`](docs/PAPER_CODE_MAP.md)'s
  equation-to-code index if it corresponds to a manuscript equation.

## Preserving paper configs

The TOML files under `configs/paper/` define what "reproduces the paper"
means for this repository (see
[`docs/REPRODUCING_THE_PAPER.md`](docs/REPRODUCING_THE_PAPER.md)). Do not
change their objectives, durations, gains, or robot bases as a side effect
of unrelated work. If a paper config genuinely needs to change, say so
explicitly in the PR description and explain why the manuscript's reported
results are unaffected.

## Keeping the mathematical core generic

`core/` implements the manuscript's mathematics and should stay free of
robot-specific, backend-specific (MuJoCo), or experiment-specific assumptions
— those layers live in `simulation/` and `experiments/` respectively. See
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the intended dependency
direction. This separation is what makes `CooperativeSystemSpec`-based robot
extension possible; changes that couple `core/` to one robot or one backend
will generally be asked to move.

## Third-party attribution

If you contribute code or assets derived from another project (a robot
model, a numerical routine, a config format), identify the upstream source
and its license in the PR description, and preserve any existing
copyright/license header rather than replacing it. Only contribute material
you have the right to submit.

## Licensing

Original code in this repository is released under the Apache License 2.0
(see [`LICENSE`](LICENSE)); third-party components are catalogued in
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md). By contributing, you
agree to:

- only contribute code or assets you have the right to submit;
- preserve third-party attribution and license headers rather than
  overwriting them;
- clearly identify any imported external material (robot models, numerical
  routines, config formats) and its upstream license in your PR description,
  and add it to `THIRD_PARTY_NOTICES.md`; and
- avoid introducing dependencies with licenses incompatible with Apache-2.0
  (for example, strong-copyleft licenses) without first discussing it in an
  issue or PR.

This project does not require a contributor license agreement (CLA).
