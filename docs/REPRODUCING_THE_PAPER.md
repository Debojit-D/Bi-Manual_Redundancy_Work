# Reproducing the spatial paper experiments

The configuration-driven commands below execute the MuJoCo experiment logic
used for *Task-Specific Manipulability Metrics for Redundancy Optimization in
Cooperative Manipulation*. They reproduce experiment campaigns and raw data;
they do not currently promise pixel-identical final manuscript figures.

Install the project in a Python 3.11+ environment, including its pinned MuJoCo,
Mink, NumPy, and SciPy dependencies. From the repository root, run one campaign:

```bash
bimanual-redopt run --config configs/paper/static.toml
```

Run all configured spatial campaigns:

```bash
bimanual-redopt reproduce-paper
```

Run a single, very short representative static baseline through the same scene,
controller, no-render simulation, and CSV recording path:

```bash
bimanual-redopt reproduce-paper --smoke
```

Smoke mode verifies initialization and plumbing only. Its explicit short-duration
overlay is recorded in `resolved_config.toml`; it does not alter a paper TOML and
does not reproduce publication statistics.

## Campaigns and manuscript results

| Manuscript result | Experiment/raw-data command | Final artifact status |
|---|---|---|
| Figure 10 — static optimization | `bimanual-redopt run --config configs/paper/static.toml` | Reproduces the static raw campaign; use the existing static plotting scripts for a figure, without a pixel-identity claim. |
| Figure 11 — representative 6D trajectories | `bimanual-redopt run --config configs/paper/six_d.toml` | Reproduces all raw 6D trajectories, from which the representative paths are selected. |
| Figure 12 — static final configurations | static command above | Reproduces final states and recordings; snapshot extraction/figure assembly remains a separate step. |
| Figure 13 — translational pick-and-place | `bimanual-redopt run --config configs/paper/translational.toml` | Reproduces the four-mode raw campaign (baseline, velocity, force, direct directional-force). |
| Figure 14 — 6D pick-and-place | 6D command above | Reproduces the five-mode raw campaign, including direct and indirect directional-force. |
| Figure 15 — integrated actuator effort | translational and 6D commands above | CSVs contain actuator quantities used for integration; the command reproduces raw inputs, not an exact typeset table/figure. |
| Figure 16 — direct vs indirect configurations | `bimanual-redopt run --config configs/paper/directional_direct_vs_indirect.toml` | Reproduces the matched static spatial direct/indirect raw comparison and recordings. |
| Table I — tracking and computational latency | translational and 6D commands above | CSVs contain tracking and timing fields; `summarize_tracking_latency` produces the summary, not a pixel-identical manuscript table. |

The spatial formulation assignment is deliberate: Eq. (16), direct force-space,
is used in the primary static, translational, and 6D comparisons. Eq. (17),
indirect velocity-dual, is additionally evaluated only in the static and 6D
campaigns and in the matched direct-vs-indirect campaign.

## Outputs and provenance

Runs are never silently overwritten. Each invocation creates a new directory:

```text
outputs/paper_reproduction/<UTC-run-id>/
├── metadata/
├── static/
├── translational/
├── six_d/
└── directional_comparison/
```

Only directories relevant to the selected command are created. Every leaf run
directory contains `run_metadata.json` and `resolved_config.toml`, plus CSV and
video outputs selected by the configuration. Metadata records the commit, dirty
state, experiment, objective, initial identifier, dependency versions, platform,
timestamp, execution status, and the exact resolved configuration.

The TOMLs are the interface for paper runs. Existing Python constants remain as
backward-compatible defaults for direct legacy runner invocation; the
configuration pipeline injects the resolved values into those same runners and
restores their defaults after each run.
