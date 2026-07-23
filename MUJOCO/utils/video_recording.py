"""Headless dual-camera MuJoCo recording with FFmpeg and tqdm progress."""

from pathlib import Path
import shutil
import subprocess

import mujoco
from tqdm.auto import tqdm


VIDEO_ENCODER_CHOICES = ("nvenc", "x264")
DEFAULT_VIDEO_ENCODER = "x264"


def ffmpeg_h264_encoder_arguments(encoder):
    """Return high-quality H.264 arguments for the selected encoder."""
    if encoder == "nvenc":
        return (
            "-c:v",
            "h264_nvenc",
            "-preset",
            "hq",
            "-rc",
            "vbr",
            "-cq",
            "18",
            "-b:v",
            "0",
        )
    if encoder == "x264":
        return (
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "18",
        )
    raise ValueError(
        f"video encoder must be one of {VIDEO_ENCODER_CHOICES}, got {encoder!r}"
    )


def video_encoder_sequence(encoder, view_count, nvenc_view_limit=None):
    """Select an encoder per view, spilling excess NVENC views to x264."""
    if encoder not in VIDEO_ENCODER_CHOICES:
        raise ValueError(
            f"video encoder must be one of {VIDEO_ENCODER_CHOICES}, "
            f"got {encoder!r}"
        )
    if view_count <= 0:
        raise ValueError("view_count must be greater than zero")
    if nvenc_view_limit is None:
        nvenc_view_limit = view_count
    if nvenc_view_limit < 0:
        raise ValueError("nvenc_view_limit cannot be negative")
    nvenc_view_limit = min(nvenc_view_limit, view_count)
    return tuple(
        "x264"
        if encoder == "nvenc" and index >= nvenc_view_limit
        else encoder
        for index in range(view_count)
    )


class FFmpegRGBWriter:
    """Stream fixed-size RGB frames to an H.264 MP4 file."""

    def __init__(
        self,
        path,
        *,
        width,
        height,
        fps,
        encoder=DEFAULT_VIDEO_ENCODER,
    ):
        executable = shutil.which("ffmpeg")
        if executable is None:
            raise RuntimeError("ffmpeg is required for --record-video")
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        command = (
            executable,
            "-loglevel",
            "error",
            "-y",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
            f"{width}x{height}",
            "-framerate",
            str(fps),
            "-i",
            "-",
            "-an",
            *ffmpeg_h264_encoder_arguments(encoder),
            "-pix_fmt",
            "yuv420p",
            self.path.as_posix(),
        )
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE)

    def write(self, frame):
        if self.process.stdin is None:
            raise RuntimeError("FFmpeg input pipe is closed")
        try:
            self.process.stdin.write(frame.tobytes())
        except BrokenPipeError as error:
            raise RuntimeError(f"FFmpeg failed while writing {self.path}") from error

    def close(self):
        if self.process.stdin is not None:
            self.process.stdin.close()
            self.process.stdin = None
        return_code = self.process.wait()
        if return_code != 0:
            raise RuntimeError(
                f"FFmpeg exited with status {return_code}: {self.path}"
            )


class HeadlessDualViewRecorder:
    """Viewer-compatible sink that records perspective and overhead frames."""

    def __init__(
        self,
        model,
        data,
        output_dir,
        *,
        perspective_lookat,
        top_lookat,
        front_lookat,
        perspective_azimuth,
        perspective_elevation,
        top_azimuth,
        top_elevation,
        front_azimuth,
        front_elevation,
        perspective_distance,
        top_distance,
        front_distance,
        control_hz,
        width=1280,
        height=720,
        fps=30,
        views=None,
        encoder=DEFAULT_VIDEO_ENCODER,
        nvenc_view_limit=None,
    ):
        if width <= 0 or height <= 0 or width % 2 or height % 2:
            raise ValueError("video width and height must be positive even values")
        if fps <= 0:
            raise ValueError("video fps must be positive")
        selected_views = (
            ("perspective", "top_view") if views is None else tuple(views)
        )
        available_views = {"perspective", "top_view", "front_view"}
        unknown_views = set(selected_views) - available_views
        if not selected_views or unknown_views:
            raise ValueError(
                "video views must contain perspective, top_view, and/or "
                "front_view"
            )
        view_encoders = video_encoder_sequence(
            encoder,
            len(selected_views),
            nvenc_view_limit,
        )
        self.model = model
        self.data = data
        self.output_dir = Path(output_dir)
        self.control_hz = float(control_hz)
        self.frame_interval = 1.0 / float(fps)
        self.elapsed_control_time = 0.0
        self.next_frame_time = 0.0
        self.running = True
        # MuJoCo defaults its offscreen framebuffer to 640x480. The renderer
        # rejects larger output sizes unless these model settings are raised
        # before MjrContext is created.
        self.model.vis.global_.offwidth = max(
            self.model.vis.global_.offwidth,
            width,
        )
        self.model.vis.global_.offheight = max(
            self.model.vis.global_.offheight,
            height,
        )
        # GLFW creates an invisible offscreen context. This uses the same
        # desktop OpenGL setup as the interactive MuJoCo viewer but opens no
        # visible simulation window.
        from mujoco.glfw import GLContext

        self.gl_context = GLContext(width, height)
        self.gl_context.make_current()
        camera_options = {
            "perspective": self._camera(
                perspective_lookat,
                perspective_azimuth,
                perspective_elevation,
                perspective_distance,
            ),
            "top_view": self._camera(
                top_lookat,
                top_azimuth,
                top_elevation,
                top_distance,
            ),
            "front_view": self._camera(
                front_lookat,
                front_azimuth,
                front_elevation,
                front_distance,
            ),
        }
        self.renderers = {
            name: mujoco.Renderer(model, height=height, width=width)
            for name in selected_views
        }
        self.cameras = {
            name: camera_options[name] for name in selected_views
        }
        self.writers = {}
        self.view_encoders = {}
        for name, view_encoder in zip(self.renderers, view_encoders):
            self.view_encoders[name] = view_encoder
            self.writers[name] = FFmpegRGBWriter(
                self.output_dir / f"{name}.mp4",
                width=width,
                height=height,
                fps=fps,
                encoder=view_encoder,
            )

    @staticmethod
    def _camera(lookat, azimuth, elevation, distance):
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = lookat
        camera.azimuth = azimuth
        camera.elevation = elevation
        camera.distance = distance
        return camera

    @property
    def user_scn(self):
        # Custom viewer overlays are intentionally unsupported in headless
        # mode; collision metrics remain recorded numerically in CSV output.
        return None

    def is_running(self):
        return self.running

    def sync(self):
        self.elapsed_control_time += 1.0 / self.control_hz
        if self.elapsed_control_time + 1e-12 < self.next_frame_time:
            return
        for name, renderer in self.renderers.items():
            self.gl_context.make_current()
            renderer.update_scene(self.data, camera=self.cameras[name])
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = 0
            self.writers[name].write(renderer.render())
        self.next_frame_time += self.frame_interval

    def close(self):
        if not self.running:
            return
        self.running = False
        errors = []
        for renderer in self.renderers.values():
            renderer.close()
        self.gl_context.free()
        for writer in self.writers.values():
            try:
                writer.close()
            except RuntimeError as error:
                errors.append(error)
        if errors:
            raise errors[0]

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


class TqdmSimulationRate:
    """RateLimiter-compatible progress counter that never sleeps."""

    def __init__(self, description):
        self.progress = tqdm(
            desc=description,
            unit="sim step",
            dynamic_ncols=True,
        )

    def sleep(self):
        self.progress.update(1)

    def close(self):
        self.progress.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
