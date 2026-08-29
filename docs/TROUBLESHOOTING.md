# Troubleshooting

## Installation

### `python3.12` is not installed

The project is tested with Python 3.12. Ubuntu 20.04 and some other older
distributions do not provide it in their standard repositories. Install a
user-local Python with [uv](https://docs.astral.sh/uv/) instead — this does
not replace the operating system's Python:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"
uv python install 3.12
uv venv --python 3.12 .venv
source .venv/bin/activate
python --version  # Must report Python 3.12.x
uv pip install -e .
```

`uv venv` creates a minimal environment without the `pip` Python module by
default, so `python -m pip` will report `No module named pip` in this setup.
Use `uv pip` as shown above. Alternatively, `uv venv --seed --python 3.12
.venv` creates an environment containing `pip`.

If creating `.venv` fails, stop there: activation and installation cannot work
until the environment exists. Outside an activated environment, `python` may
refer to Python 2 on older Linux installations.

Do not substitute an older system `python3` when creating the environment.
The pinned dependencies are tested with Python 3.12; on older interpreters,
pip may misleadingly report that it cannot find `mink==1.1.1` — the release
is being hidden because it is not compatible with that interpreter.

Avoid launching experiment scripts with `/usr/bin/python3`, because that
bypasses the activated virtual environment.

### Verifying the installation

```bash
python -c "import importlib.metadata as m, mujoco; from qpsolvers import available_solvers; print(mujoco.__version__, m.version('mink'), available_solvers)"
```

`daqp` must appear in the solver list.

### `ffmpeg` is missing

Headless MP4 recording requires the `ffmpeg` executable:

```bash
ffmpeg -version
```

On Ubuntu, install it with `sudo apt install ffmpeg` if it is missing.

## Runtime

### `SolverNotFound: osqp`

The active scripts use DAQP. Confirm the local environment is active and DAQP
is available:

```bash
python -c "from qpsolvers import available_solvers; print(available_solvers)"
```

### `ModuleNotFoundError: bimanual_redundancy`

Install the repository editable from its root:

```bash
source .venv/bin/activate
python -m pip install -e .
```

### Viewer does not open

The interactive viewer requires a working desktop/OpenGL display. The
`--record-video` path opens no visible simulation window, but its invisible
GLFW rendering context still requires a working OpenGL/display environment on
the current machine. Confirm `ffmpeg -version` succeeds before recording.

For fully headless environments (including CI), set `MUJOCO_GL=osmesa` (or
`egl` if a GPU/EGL driver is available) before running any command that
constructs a MuJoCo scene.

### Publication figures use a fallback font instead of Times New Roman

Times New Roman must be supplied locally (project-relative
`outputs/.fonts/times-new-roman/`, or installed system-wide) for strict
publication typography — it is never bundled or downloaded by this project.
