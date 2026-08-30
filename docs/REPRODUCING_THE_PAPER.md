# Reproducing the spatial paper experiments

Configuration-driven commands that execute the MuJoCo experiment logic for
*Task-Specific Manipulability Metrics for Redundancy Optimization in
Cooperative Manipulation*. They reproduce experiment campaigns and raw data;
figure/table assembly from that data is a separate step, not automated here.

After the [Quick start](../README.md#quick-start) install, run one campaign
from the repository root:

```bash
bimanual-redopt run --config configs/paper/static.toml
```

Run every configured spatial campaign:

```bash
bimanual-redopt reproduce-paper
```

Run a short, representative smoke pass (same scene, controller, and CSV
recording path, at a tiny simulated duration) to check the pipeline without
reproducing publication statistics:

```bash
bimanual-redopt reproduce-paper --smoke
```

The smoke overlay is recorded in `resolved_config.toml`; it never alters a
paper TOML.

## Campaigns and manuscript results

| Manuscript result | Command | Output |
|---|---|---|
| Figure 10: static optimization | `bimanual-redopt run --config configs/paper/static.toml` | Static raw campaign; plot with the existing static plotting scripts. |
| Figure 11: representative 6D trajectories | `bimanual-redopt run --config configs/paper/six_d.toml` | All raw 6D trajectories, from which the representative paths are selected. |
| Figure 12: static final configurations | static command above | Final states and recordings; snapshot extraction is a separate step. |
| Figure 13: translational pick-and-place | `bimanual-redopt run --config configs/paper/translational.toml` | Four-mode raw campaign (baseline, velocity, force, direct directional-force). |
| Figure 14: 6D pick-and-place | 6D command above | Five-mode raw campaign, including direct and indirect directional-force. |
| Figure 15: integrated actuator effort | translational and 6D commands above | CSVs with the actuator quantities used for integration. |
| Figure 16: direct vs. indirect configurations | `bimanual-redopt run --config configs/paper/directional_direct_vs_indirect.toml` | Matched static spatial direct/indirect raw comparison and recordings. |
| Table I: tracking and computational latency | translational and 6D commands above | CSVs with tracking/timing fields; `summarize_tracking_latency` produces the summary. |

Eq. (16) (direct force-space) is used in the primary static, translational,
and 6D comparisons. Eq. (17) (indirect velocity-dual) is additionally
evaluated in the static and 6D campaigns and in the direct-vs-indirect
campaign.

## Outputs and provenance

Each invocation creates a new, timestamped directory; runs are never
overwritten:

```text
outputs/paper_reproduction/<UTC-run-id>/
├── metadata/
├── static/
├── translational/
├── six_d/
└── directional_comparison/
```

Only directories relevant to the selected command are created. Every leaf
run directory contains `run_metadata.json` (commit, dirty state, experiment,
objective, initial identifier, dependency versions, platform, timestamp,
execution status) and `resolved_config.toml`, plus the CSV and video outputs
selected by the configuration.

TOMLs are the interface for paper runs; the configuration pipeline injects
resolved values into the underlying experiment runners and restores their
defaults after each run.
