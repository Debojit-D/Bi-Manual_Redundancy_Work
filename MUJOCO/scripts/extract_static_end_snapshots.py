"""Extract final front/top frames from static Equation (8) videos.

The output stores eight directly accessible images in each ``position_*``
directory beneath ``paper_figs/static_end_snapshots``. Filenames combine the
mode and view, for example ``baseline_front_view.png``. A CSV manifest records
the source video, final frame index, FPS, and corresponding video time.

Example::

    python3 -m MUJOCO.scripts.extract_static_end_snapshots \
        outputs/equation8_comparison_batches/<batch>/videos/static/<run>
"""

import argparse
import csv
import os
from pathlib import Path

try:
    import cv2
except ImportError as error:  # pragma: no cover - depends on local environment
    raise RuntimeError(
        "OpenCV is required to extract video frames. Run this script with a "
        "Python environment that provides the 'cv2' module."
    ) from error


MODES = ("baseline", "velocity", "force", "directional_force")
VIEWS = ("front_view", "top_view")
POSITION_PREFIX = "position_"


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "video_root",
        type=Path,
        help="static run directory containing position_*/<mode>/*.mp4",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help=(
            "snapshot directory (default: "
            "<batch>/paper_figs/static_end_snapshots)"
        ),
    )
    return parser.parse_args(argv)


def position_number(path):
    suffix = path.name.removeprefix(POSITION_PREFIX)
    if not suffix.isdigit():
        raise ValueError(f"Invalid static-position directory: {path}")
    return int(suffix)


def discover_positions(video_root):
    positions = tuple(
        sorted(
            (
                path
                for path in video_root.iterdir()
                if path.is_dir() and path.name.startswith(POSITION_PREFIX)
            ),
            key=position_number,
        )
    )
    if not positions:
        raise FileNotFoundError(
            f"No {POSITION_PREFIX}* directories found in: {video_root}"
        )
    return positions


def expected_videos(positions):
    return tuple(
        (position, mode, view, position / mode / f"{view}.mp4")
        for position in positions
        for mode in MODES
        for view in VIEWS
    )


def validate_videos(videos):
    missing = tuple(path for _, _, _, path in videos if not path.is_file())
    if missing:
        formatted = "\n".join(f"  {path}" for path in missing)
        raise FileNotFoundError(f"Missing expected videos:\n{formatted}")


def read_final_frame(path):
    """Return the final decodable frame and its video metadata."""
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"Could not open video: {path}")

    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    frame_index = max(frame_count - 1, 0)
    capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
    decoded, frame = capture.read()
    capture.release()

    if not decoded:
        # Some codecs do not support frame-accurate seeking. In that case,
        # decode sequentially and retain the final successful frame.
        capture = cv2.VideoCapture(str(path))
        frame = None
        frame_index = -1
        while True:
            decoded, candidate = capture.read()
            if not decoded:
                break
            frame_index += 1
            frame = candidate
        capture.release()
        if frame is None:
            raise RuntimeError(f"No decodable frames found in: {path}")

    video_time_s = frame_index / fps if fps > 0.0 else None
    return frame, frame_index, frame_count, fps, video_time_s


def write_png_atomic(path, frame):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_name(f".{path.stem}.tmp{path.suffix}")
    if not cv2.imwrite(str(temporary_path), frame):
        raise RuntimeError(f"Could not write snapshot: {temporary_path}")
    os.replace(temporary_path, path)


def default_output_directory(video_root):
    try:
        batch_dir = video_root.parents[2]
    except IndexError as error:
        raise ValueError(
            "Could not infer the batch directory from the video path; "
            "provide --output-dir explicitly."
        ) from error
    return batch_dir / "paper_figs" / "static_end_snapshots"


def extract_snapshots(video_root, output_dir):
    video_root = Path(video_root).expanduser().resolve()
    output_dir = Path(output_dir).expanduser().resolve()
    if not video_root.is_dir():
        raise FileNotFoundError(f"Video directory not found: {video_root}")

    positions = discover_positions(video_root)
    videos = expected_videos(positions)
    validate_videos(videos)

    rows = []
    for position, mode, view, input_path in videos:
        output_path = output_dir / position.name / f"{mode}_{view}.png"
        frame, frame_index, frame_count, fps, video_time_s = read_final_frame(
            input_path
        )
        write_png_atomic(output_path, frame)
        rows.append(
            {
                "position": position.name,
                "mode": mode,
                "view": view,
                "source_video": str(input_path.resolve()),
                "snapshot": str(output_path),
                "frame_index": frame_index,
                "reported_frame_count": frame_count,
                "fps": f"{fps:.6g}",
                "video_time_s": (
                    "" if video_time_s is None else f"{video_time_s:.6f}"
                ),
            }
        )
        print(f"Saved {output_path}")

    manifest_path = output_dir / "manifest.csv"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = tuple(rows[0])
    with manifest_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Saved {manifest_path}")
    return tuple(Path(row["snapshot"]) for row in rows), manifest_path


def main(argv=None):
    arguments = parse_arguments(argv)
    video_root = arguments.video_root.expanduser().resolve()
    output_dir = (
        arguments.output_dir.expanduser().resolve()
        if arguments.output_dir is not None
        else default_output_directory(video_root)
    )
    snapshots, manifest = extract_snapshots(video_root, output_dir)
    print(f"Extracted {len(snapshots)} end-frame snapshots to: {output_dir}")
    print(f"Manifest: {manifest}")
    return snapshots


if __name__ == "__main__":
    main()
