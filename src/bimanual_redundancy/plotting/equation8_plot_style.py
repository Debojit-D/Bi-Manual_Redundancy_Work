"""Seaborn styling for Equation (8) comparison-batch figures."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
import seaborn as sns

from bimanual_redundancy import paths


class Equation8PlotStyle:
    """Own the visual language and figure-export settings for batch plots."""

    DOUBLE_COLUMN_WIDTH = 7.16
    SIX_PANEL_SIZE = (DOUBLE_COLUMN_WIDTH, 1.425)
    SIX_PANEL_V2_SIZE = (DOUBLE_COLUMN_WIDTH, 1.8)
    SIX_PANEL_GRID_SIZE = (DOUBLE_COLUMN_WIDTH, 2.7)
    THREE_BY_SIX_SIZE = (DOUBLE_COLUMN_WIDTH, 3.9)
    FOUR_BY_SIX_SIZE = (DOUBLE_COLUMN_WIDTH, 5.27)
    REDUCED_FOUR_BY_SIX_SIZE = (DOUBLE_COLUMN_WIDTH, 4.96)
    TIMES_NEW_ROMAN_DIR = paths.OUTPUT_FONTS_DIR / "times-new-roman"
    BASELINE_COLOR = "#68737D"
    COMMAND_COLOR = "#111111"
    MODE_COLORS = {
        "velocity": "#0072B2",
        "force": "#D55E00",
        "directional_force": "#009E73",
        "directional_force_indirect": "#CC79A7",
    }

    def __init__(self, *, dpi=300):
        self.dpi = int(dpi)
        if self.dpi <= 0:
            raise ValueError("dpi must be greater than zero")

    def apply(self):
        """Apply the common Seaborn theme before figures are created."""
        self._register_times_new_roman()
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
            },
        )
        plt.rcParams.update(
            {
                "font.family": "Times New Roman",
                "axes.titlesize": 8.5,
                "axes.labelsize": 9.5,
                "xtick.labelsize": 8,
                "ytick.labelsize": 8,
                "legend.fontsize": 8.5,
                "figure.labelsize": 9.5,
                "figure.titlesize": 10,
            }
        )

    def _register_times_new_roman(self):
        """Register the project-local Times files and reject substitution."""
        for path in sorted(self.TIMES_NEW_ROMAN_DIR.glob("*.[Tt][Tt][Ff]")):
            font_manager.fontManager.addfont(path)
        try:
            font_manager.findfont(
                "Times New Roman",
                fallback_to_default=False,
            )
        except ValueError as error:
            raise RuntimeError(
                "Times New Roman is unavailable. Expected font files in "
                f"{self.TIMES_NEW_ROMAN_DIR}"
            ) from error

    def mode_color(self, mode):
        try:
            return self.MODE_COLORS[mode]
        except KeyError as error:
            raise ValueError(f"No plot color configured for mode {mode!r}") from error

    def baseline_line_kwargs(self):
        """Return subdued styling so baseline traces remain in the background."""
        return {
            "color": self.BASELINE_COLOR,
            "linewidth": 1.35,
            "linestyle": (0, (4, 2)),
            "alpha": 0.72,
            "zorder": 1,
        }

    def command_line_kwargs(self):
        """Return styling for a commanded reference trajectory."""
        return {
            "color": self.COMMAND_COLOR,
            "linewidth": 1.25,
            "linestyle": (0, (1, 1.5)),
            "alpha": 0.9,
            "zorder": 3,
        }

    def optimized_line_kwargs(self, mode):
        return {
            "color": self.mode_color(mode),
            "linewidth": 1.15,
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
                for mode in mode_labels
                if mode != "baseline"
            ),
        )

    def tracking_legend_handles(self, mode_labels):
        """Return handles for one command and all measured mode trajectories."""
        return (
            Line2D(
                [0],
                [0],
                label="Commanded",
                **self.command_line_kwargs(),
            ),
            *(
                Line2D(
                    [0],
                    [0],
                    label=f"{handle.get_label()} tracked",
                    color=handle.get_color(),
                    linewidth=handle.get_linewidth(),
                    linestyle=handle.get_linestyle(),
                    alpha=handle.get_alpha(),
                )
                for handle in self.all_mode_legend_handles(mode_labels)
            ),
        )

    def _finish_six_panel_figure(
        self,
        figure,
        axes,
        *,
        handles,
        legend_columns,
        legend_y=0.97,
        layout_top=0.89,
    ):
        for axis in axes.flat:
            axis.margins(x=0)
            axis.ticklabel_format(axis="y", style="plain", useOffset=False)
        sns.despine(fig=figure, offset=2, trim=False)
        figure.tight_layout(
            rect=(0.015, 0.025, 1.0, layout_top),
            pad=0.35,
            h_pad=0.5,
            w_pad=0.45,
        )
        figure.legend(
            handles=handles,
            loc="upper center",
            bbox_to_anchor=(0.5, legend_y),
            ncol=legend_columns,
            handlelength=2.2,
            frameon=True,
            fancybox=False,
            framealpha=1.0,
            facecolor="white",
            edgecolor="#68737D",
            borderpad=0.4,
        )

    def finish_six_panel_figure(self, figure, axes, *, mode, mode_label):
        """Finalize spacing, axis details, and one shared legend."""
        self._finish_six_panel_figure(
            figure,
            axes,
            handles=self.legend_handles(mode, mode_label),
            legend_columns=2,
        )

    def finish_six_panel_row_figure(
        self,
        figure,
        axes,
        *,
        mode,
        mode_label,
    ):
        """Finalize a one-row figure comparing baseline and one mode."""
        self._finish_six_panel_figure(
            figure,
            axes,
            handles=self.legend_handles(mode, mode_label),
            legend_columns=2,
            legend_y=0.985,
            layout_top=0.87,
        )

    def finish_optimized_only_six_panel_row_figure(
        self,
        figure,
        axes,
        *,
        mode,
        mode_label,
    ):
        """Finalize a one-row figure containing one optimized mode."""
        handles = (
            Line2D(
                [0],
                [0],
                label=mode_label,
                **self.optimized_line_kwargs(mode),
            ),
        )
        self._finish_six_panel_figure(
            figure,
            axes,
            handles=handles,
            legend_columns=1,
            legend_y=0.98,
            layout_top=0.74,
        )

    def finish_all_modes_six_panel_figure(
        self,
        figure,
        axes,
        *,
        mode_labels,
        legend_columns=None,
        legend_handle_order=None,
        legend_y=0.97,
        layout_top=0.89,
    ):
        """Finalize a six-panel figure comparing all optimization modes."""
        handles = self.all_mode_legend_handles(mode_labels)
        if legend_handle_order is not None:
            handles = tuple(handles[index] for index in legend_handle_order)
        self._finish_six_panel_figure(
            figure,
            axes,
            handles=handles,
            legend_columns=legend_columns or len(mode_labels),
            legend_y=legend_y,
            layout_top=layout_top,
        )

    def finish_all_modes_six_panel_row_figure(
        self,
        figure,
        axes,
        *,
        mode_labels,
    ):
        """Finalize a one-row six-panel figure comparing all modes."""
        self._finish_six_panel_figure(
            figure,
            axes,
            handles=self.all_mode_legend_handles(mode_labels),
            legend_columns=len(mode_labels),
            legend_y=0.985,
            layout_top=0.87,
        )

    def finish_tracking_six_panel_figure(
        self,
        figure,
        axes,
        *,
        mode_labels,
    ):
        """Finalize a commanded-versus-measured six-panel figure."""
        self._finish_six_panel_figure(
            figure,
            axes,
            handles=self.tracking_legend_handles(mode_labels),
            legend_columns=5,
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
