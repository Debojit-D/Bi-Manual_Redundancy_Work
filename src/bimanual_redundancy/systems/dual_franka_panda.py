"""Built-in reference embodiment used for every manuscript experiment."""

from __future__ import annotations

from bimanual_redundancy import paths
from .spec import ArmSpec, CollisionSpec, CooperativeSystemSpec, ObjectSpec


_PANDA_POSITION_LIMITS = (
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973),
)
_PANDA_VELOCITY_LIMITS = (
    (-2.175, 2.175),
    (-2.175, 2.175),
    (-2.175, 2.175),
    (-2.175, 2.175),
    (-2.61, 2.61),
    (-2.61, 2.61),
    (-2.61, 2.61),
)


DUAL_FRANKA_PANDA = CooperativeSystemSpec(
    identifier="dual_franka_panda",
    backend="mujoco",
    model_path=paths.ROBOT_MODELS_DIR / "franka_emika_panda" / "dual_panda_scene.xml",
    left_arm=ArmSpec(
        joint_names=tuple(f"joint{index}_l" for index in range(1, 8)),
        actuator_names=tuple(f"actuator{index}_l" for index in range(1, 8)),
        base_body="franka1",
        hand_site="attachment_site_left",
        target_body="target_left",
        gripper_actuator="actuator8_l",
        joint_position_limits=_PANDA_POSITION_LIMITS,
        joint_velocity_limits=_PANDA_VELOCITY_LIMITS,
    ),
    right_arm=ArmSpec(
        joint_names=tuple(f"joint{index}_r" for index in range(1, 8)),
        actuator_names=tuple(f"actuator{index}_r" for index in range(1, 8)),
        base_body="franka2",
        hand_site="attachment_site_right",
        target_body="target_right",
        gripper_actuator="actuator8_r",
        joint_position_limits=_PANDA_POSITION_LIMITS,
        joint_velocity_limits=_PANDA_VELOCITY_LIMITS,
    ),
    object=ObjectSpec(
        body="vention_table",
        joint="table_joint",
        reference_site="site_top_middle",
        contact_sites=("site_left", "site_right"),
    ),
    home_keyframe="home1",
    mounting_platform_geom="dual_arm_mounting_platform",
    collision=CollisionSpec(
        sphere_resource=(
            paths.ROBOT_MODELS_DIR
            / "franka_emika_panda"
            / "dual_panda_robots_only_spherefit.xml"
        ),
        sphere_geom_prefix="spherefit_",
        left_root_body="franka1",
        right_root_body="franka2",
        excluded_bodies=("franka1", "franka2", "link0_l", "link0_r"),
        left_inter_arm_bodies=(
            "link3_l", "link4_l", "link5_l", "link6_l", "link7_l", "hand_l"
        ),
        right_inter_arm_bodies=(
            "link3_r", "link4_r", "link5_r", "link6_r", "link7_r", "hand_r"
        ),
        left_table_bodies=(
            "link1_l", "link2", "link3_l", "link4_l", "link5_l", "link6_l", "link7_l"
        ),
        right_table_bodies=(
            "link1_r", "link2_r", "link3_r", "link4_r", "link5_r", "link6_r", "link7_r"
        ),
        left_terminal_bodies=("link7_l",),
        right_terminal_bodies=("link7_r",),
        left_gripper_bodies=("hand_l", "left_finger_l", "right_finger_l"),
        right_gripper_bodies=("hand_r", "left_finger_r", "right_finger_r"),
        table_root_body="vention_table",
        table_geom_names=("vention_table_top_collision", "table_top_collision"),
        table_top_half_size=(0.202, 0.134, 0.005),
    ),
)
