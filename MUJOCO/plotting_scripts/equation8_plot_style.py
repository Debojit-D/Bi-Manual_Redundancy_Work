"""Seaborn styling for Equation (8) comparison-batch figures."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import seaborn as sns


class Equation8PlotStyle:
    """Own the visual language and figure-export settings for batch plots."""

    BASELINE_COLOR = "#68737D"
    MODE_COLORS = {
        "velocity": "#0072B2",
        "force": "#D55E00",
        "directional_force": "#009E73",
    }
    SIX_PANEL_SIZE = (12.0, 6.4)

    def __init__(self, *, dpi=300):
        self.dpi = int(dpi)
        if self.dpi <= 0:
            raise ValueError("dpi must be greater than zero")

    def apply(self):
        """Apply the common Seaborn theme before figures are created."""
        sns.set_theme(
            context="paper",
            style="whitegrid",
            font="serif",
            font_scale=1.05,
            rc={
                "figure.dpi": 120,
                "savefig.bbox": "tight",
                "axes.axisbelow": True,
                "axes.edgecolor": "#343A40",
                "axes.labelcolor": "#212529",
                "axes.linewidth": 0.8,
                "axes.titleweight": "bold",
                "grid.alpha": 0.24,
                "grid.color": "#7A8288",
                "grid.linewidth": 0.55,
                "legend.frameon": False,
                "lines.solid_capstyle": "round",
                "pdf.fonttype": 42,
                "ps.fonttype": 42,
                "mathtext.fontset": "stix",
            },
        )
        plt.rcParams.update(
            {
                "font.family": "serif",
                "font.serif": [
                    "Times New Roman",
                    "Times",
                    "Nimbus Roman",
                    "Liberation Serif",
                    "DejaVu Serif",
                ],
                "axes.titlesize": 10,
                "axes.labelsize": 9.5,
                "xtick.labelsize": 8.5,
                "ytick.labelsize": 8.5,
                "legend.fontsize": 9,
                "figure.titlesize": 14,
            }
        )

    def mode_color(self, mode):
        try:
            return self.MODE_COLORS[mode]
        except KeyError as error:
            raise ValueError(f"No plot color configured for mode {mode!r}") from error

    def baseline_line_kwargs(self):
        """Return subdued styling so baseline traces remain in the background."""
        return {
            "color": self.BASELINE_COLOR,
            "linewidth": 2.3,
            "linestyle": (0, (4, 2)),
            "alpha": 0.72,
            "zorder": 1,
        }

    def optimized_line_kwargs(self, mode):
        return {
            "color": self.mode_color(mode),
            "linewidth": 1.9,
            "alpha": 0.98,
            "zorder": 2,
        }

    def legend_handles(self, mode, mode_label):
        return (
            Line2D(
                [0],
                [0],
                label="Baseline",
                **self.baseline_line_kwargs(),
            ),
            Line2D(
                [0],
                [0],
                label=mode_label,
                **self.optimized_line_kwargs(mode),
            ),
        )

    def all_mode_legend_handles(self, mode_labels):
        """Return handles for baseline and every optimization mode."""
        return (
            Line2D(
                [0],
                [0],
                label=mode_labels["baseline"],
                **self.baseline_line_kwargs(),
            ),
            *(
                Line2D(
                    [0],
                    [0],
                    label=mode_labels[mode],
                    **self.optimized_line_kwargs(mode),
                )
                for mode in self.MODE_COLORS
            ),
        )

    def _finish_six_panel_figure(
        self,
        figure,
        axes,
        *,
        handles,
        legend_columns,
    ):
        for axis in axes.flat:
            axis.margins(x=0)
            axis.ticklabel_format(axis="y", style="plain", useOffset=False)
        sns.despine(fig=figure, offset=2, trim=False)
        figure.legend(
            handles=handles,
            loc="upper center",
            bbox_to_anchor=(0.5, 0.935),
            ncol=legend_columns,
            handlelength=3.0,
        )
        figure.tight_layout(rect=(0.035, 0.045, 1.0, 0.88))

    def finish_six_panel_figure(self, figure, axes, *, mode, mode_label):
        """Finalize spacing, axis details, and one shared legend."""
        self._finish_six_panel_figure(
            figure,
            axes,
            handles=self.legend_handles(mode, mode_label),
            legend_columns=2,
        )

    def finish_all_modes_six_panel_figure(
        self,
        figure,
        axes,
        *,
        mode_labels,
    ):
        """Finalize a six-panel figure comparing all optimization modes."""
        self._finish_six_panel_figure(
            figure,
            axes,
            handles=self.all_mode_legend_handles(mode_labels),
            legend_columns=4,
        )

    def save(self, figure, output_dir, stem, output_format):
        """Save one figure as PNG, PDF, or both and return written paths."""
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        suffixes = (
            ("png", "pdf") if output_format == "both" else (output_format,)
        )
        written = []
        for suffix in suffixes:
            path = output_dir / f"{stem}.{suffix}"
            figure.savefig(
                path,
                dpi=self.dpi if suffix == "png" else None,
                bbox_inches="tight",
            )
            written.append(path)
        return tuple(written)
