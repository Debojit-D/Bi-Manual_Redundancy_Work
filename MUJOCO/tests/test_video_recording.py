import unittest

from MUJOCO.utils.video_recording import (
    ffmpeg_h264_encoder_arguments,
    video_encoder_sequence,
)


class VideoRecordingTests(unittest.TestCase):
    def test_nvenc_uses_high_quality_constant_quality_settings(self):
        arguments = ffmpeg_h264_encoder_arguments("nvenc")

        self.assertEqual(arguments[0:2], ("-c:v", "h264_nvenc"))
        self.assertIn("hq", arguments)
        self.assertIn("18", arguments)

    def test_x264_preserves_existing_software_settings(self):
        arguments = ffmpeg_h264_encoder_arguments("x264")

        self.assertEqual(arguments[0:2], ("-c:v", "libx264"))
        self.assertIn("medium", arguments)
        self.assertIn("18", arguments)

    def test_unknown_encoder_is_rejected_before_ffmpeg_starts(self):
        with self.assertRaisesRegex(ValueError, "video encoder"):
            ffmpeg_h264_encoder_arguments("unknown")

    def test_excess_nvenc_views_spill_to_x264(self):
        self.assertEqual(
            video_encoder_sequence("nvenc", 3, nvenc_view_limit=2),
            ("nvenc", "nvenc", "x264"),
        )

    def test_x264_selection_remains_software_only(self):
        self.assertEqual(
            video_encoder_sequence("x264", 3, nvenc_view_limit=2),
            ("x264", "x264", "x264"),
        )

    def test_view_limit_is_clamped_for_single_view_recording(self):
        self.assertEqual(
            video_encoder_sequence("nvenc", 1, nvenc_view_limit=3),
            ("nvenc",),
        )


if __name__ == "__main__":
    unittest.main()
