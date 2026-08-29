"""Universal camera distances for active dual-Franka MuJoCo views."""


PERSPECTIVE_CAMERA_DISTANCE = 2.0
TOP_CAMERA_DISTANCE = 1.7
FRONT_CAMERA_DISTANCE = 1.7

VIDEO_VIEW_CHOICES = ("all", "both", "perspective", "top", "front")


def video_views_for_choice(choice):
    """Return the recorder view identifiers for a public CLI choice."""
    mapping = {
        "all": ("perspective", "top_view", "front_view"),
        "both": ("perspective", "top_view"),
        "perspective": ("perspective",),
        "top": ("top_view",),
        "front": ("front_view",),
    }
    try:
        return mapping[choice]
    except KeyError as error:
        raise ValueError(
            f"video view must be one of {VIDEO_VIEW_CHOICES}"
        ) from error
