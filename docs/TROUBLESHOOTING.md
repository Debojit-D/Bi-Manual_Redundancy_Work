# Troubleshooting

## Installation

### `python3.12` not found

Older distributions (Ubuntu 20.04 and earlier) don't ship Python 3.12.
Install a user-local one with [uv](https://docs.astral.sh/uv/), which does
not replace the system Python:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"
uv python install 3.12
uv venv --python 3.12 .venv
source .venv/bin/activate
uv pip install -e .
```

`uv venv` creates an environment without the `pip` module by default, so
`python -m pip` reports `No module named pip`; use `uv pip` as above, or
`uv venv --seed --python 3.12 .venv` to include `pip`.

Don't substitute an older system `python3`: pinned dependencies are tested
with 3.12, and on older interpreters pip may report it cannot find
`mink==1.1.1` (the release is hidden as incompatible with that interpreter,
not actually missing).

### Verifying the installation

```bash
python -c "import importlib.metadata as m, mujoco; from qpsolvers import available_solvers; print(mujoco.__version__, m.version('mink'), available_solvers)"
```

`daqp` must appear in the solver list.

### `ffmpeg` missing

Headless MP4 recording needs the `ffmpeg` executable (`ffmpeg -version` to
check; `sudo apt install ffmpeg` on Ubuntu).

## Runtime

### `SolverNotFound: osqp`

The active scripts use DAQP, not osqp. Confirm it's available:

```bash
python -c "from qpsolvers import available_solvers; print(available_solvers)"
```

### `ModuleNotFoundError: bimanual_redundancy`

The package isn't installed in the active environment:

```bash
source .venv/bin/activate
python -m pip install -e .
```

### Viewer does not open

The interactive viewer needs a working desktop/OpenGL display.
`--record-video` opens no visible window but still needs a working
OpenGL/display environment for its invisible GLFW context. For fully
headless environments (including CI), set `MUJOCO_GL=osmesa` (or `egl` with
a GPU/EGL driver) before running any command that constructs a MuJoCo scene.

### Publication figures use DejaVu Serif instead of Times New Roman

Times New Roman is never bundled or downloaded by this project; it must be
supplied locally (`outputs/.fonts/times-new-roman/`, or installed
system-wide) for strict publication typography.
