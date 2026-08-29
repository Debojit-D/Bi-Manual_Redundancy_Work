"""Shared Seaborn/Matplotlib styling for publication figures."""

import matplotlib.pyplot as plt
import seaborn as sns


PAPER_WIDTH_IN = 7.1
COLORS = sns.color_palette("colorblind", 7)


def configure_publication_style():
    """Apply the common paper theme before creating any figure."""
    sns.set_theme(
        context="paper",
        style="whitegrid",
        palette="colorblind",
        font="Times New Roman",
        font_scale=1.15,
        rc={
            "figure.dpi": 120,
            "savefig.bbox": "tight",
            "axes.axisbelow": True,
            "axes.edgecolor": "#333333",
            "axes.labelcolor": "#222222",
            "axes.linewidth": 0.8,
            "grid.alpha": 0.22,
            "grid.color": "#7F7F7F",
            "grid.linewidth": 0.55,
            "legend.frameon": False,
            "lines.linewidth": 1.8,
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
            ],
            "axes.titleweight": "bold",
            "axes.titlesize": 10,
            "axes.labelsize": 9.5,
            "xtick.labelsize": 8.5,
            "ytick.labelsize": 8.5,
            "legend.fontsize": 8,
        }
    )


def finish_figure(fig):
    """Remove visual clutter and finalize subplot spacing."""
    sns.despine(fig=fig, offset=2, trim=False)
    fig.tight_layout()
