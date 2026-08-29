"""Tests for Equation8PlotStyle's Times New Roman fallback/strict behavior."""

import unittest
from unittest.mock import patch

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from bimanual_redundancy.plotting.equation8_plot_style import Equation8PlotStyle


def _raise_times_new_roman_unavailable(name, fallback_to_default=False):
    raise ValueError(f"Font family {name!r} not found.")


class Equation8PlotStyleFontFallbackTests(unittest.TestCase):
    """Times New Roman is never bundled/downloaded; CI runners won't have it."""

    def test_fallback_mode_uses_dejavu_serif_when_times_new_roman_unavailable(self):
        style = Equation8PlotStyle(dpi=72)
        self.assertFalse(style.strict)

        with patch(
            "bimanual_redundancy.plotting.equation8_plot_style"
            ".font_manager.findfont",
            side_effect=_raise_times_new_roman_unavailable,
        ):
            style.apply()

        self.assertIn(
            Equation8PlotStyle.FALLBACK_SERIF_FONT,
            plt.rcParams["font.family"],
        )

    def test_strict_mode_raises_when_times_new_roman_unavailable(self):
        style = Equation8PlotStyle(dpi=72, strict=True)

        with patch(
            "bimanual_redundancy.plotting.equation8_plot_style"
            ".font_manager.findfont",
            side_effect=_raise_times_new_roman_unavailable,
        ):
            with self.assertRaisesRegex(
                RuntimeError,
                "Times New Roman is unavailable",
            ):
                style.apply()


if __name__ == "__main__":
    unittest.main()
