"""Generate collision spheres, then load and hold only the two Frankas.

This is the stripped-down scene counterpart to the Equation (8) baseline.
By default it uses cuRobo to replace each link's MuJoCo collision geometry with
fitted spheres, caches the resulting XML, and loads that sphere-based model.
There is no table, floor, shared object, or manipulation controller.

Examples
--------
Normal launch (generate once, then reuse the cached sphere XML)::

    python MUJOCO/scripts/load_dual_franka_robots_only.py

Refit with denser spheres or exact per-body counts::

    python MUJOCO/scripts/load_dual_franka_robots_only.py \
        --regenerate-spheres --sphere-density 1.5 --fit-iterations 300
    python MUJOCO/scripts/load_dual_franka_robots_only.py \
        --regenerate-spheres --num-spheres hand_l=12 --num-spheres hand_r=12

Generated spheres are visualization-only by default; original MuJoCo meshes
continue handling contact. ``--replace-collisions-with-spheres`` deliberately
uses the spheres for contact instead, while ``--no-spheres`` loads the original
model without generating or drawing spheres.

Use ``--sphere-view spheres-only`` when you want an unobstructed inspection of
the fitted spheres instead of the default transparent-robot overlay.
"""

import argparse
from pathlib import Path
import subprocess
import sys

import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter
from scipy.spatial.transform import Rotation


SOURCE_MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "robot_descriptions"
    / "franka_emika_panda"
    / "dual_panda_robots_only_scene.xml"
)
SPHERE_MODEL_PATH = SOURCE_MODEL_PATH.with_name(
    "dual_panda_robots_only_spherefit.xml"
)
SPHERE_GENERATOR_PATH = (
    Path(__file__).resolve().parents[2]
    / "mujoco_curobo_bridge"
    / "demos"
    / "02_fit_mujoco_spheres.py"
)

CONTROL_HZ = 50.0
ENABLE_ARM_BIAS_COMPENSATION = True

# World-frame base poses, matching the baseline script by default.
LEFT_ARM_SPAWN_POSITION = np.array([0.0, -0.2, 0.0])
RIGHT_ARM_SPAWN_POSITION = np.array([0.0, 0.2, 0.0])
LEFT_ARM_SPAWN_EULER_XYZ_DEGREES = np.array([0.0, 0.0, 0.0])
RIGHT_ARM_SPAWN_EULER_XYZ_DEGREES = np.array([0.0, 0.0, 0.0])

LEFT_ARM_JOINTS = tuple(f"joint{i}_l" for i in range(1, 8))
RIGHT_ARM_JOINTS = tuple(f"joint{i}_r" for i in range(1, 8))


def set_robot_base_pose(model, body_name, position, euler_xyz_degrees):
    """Apply one editable fixed-base pose before simulation starts."""
    body_id = model.body(body_name).id
    model.body_pos[body_id] = np.asarray(position, dtype=float)
    quaternion_xyzw = Rotation.from_euler(
        "xyz",
        euler_xyz_degrees,
        degrees=True,
    ).as_quat()
    model.body_quat[body_id] = np.roll(quaternion_xyzw, 1)


def joint_dof_indices(model, joint_names):
    return np.array(
        [model.jnt_dofadr[model.joint(name).id] for name in joint_names],
        dtype=int,
    )


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--regenerate-spheres",
        action="store_true",
        help="refit spheres even when the generated XML already exists",
    )
    parser.add_argument(
        "--no-spheres",
        action="store_true",
        help="load the original mesh-collision model without fitting spheres",
    )
    parser.add_argument("--sphere-density", type=float, default=1.0)
    parser.add_argument("--fit-iterations", type=int, default=200)
    parser.add_argument(
        "--fit-type",
        choices=("morphit", "voxel", "surface"),
        default="morphit",
    )
    parser.add_argument(
        "--num-spheres",
        action="append",
        default=[],
        metavar="BODY=COUNT",
        help="exact count for one body, e.g. hand_l=12; repeatable",
    )
    parser.add_argument(
        "--fit-body",
        action="append",
        default=[],
        metavar="BODY",
        help="fit only this body; repeatable",
    )
    parser.add_argument("--compute-fit-metrics", action="store_true")
    parser.add_argument(
        "--replace-collisions-with-spheres",
        action="store_true",
        help=(
            "use fitted spheres for MuJoCo contacts; default keeps the "
            "original collision meshes and draws spheres only"
        ),
    )
    parser.add_argument(
        "--sphere-rgba",
        default="0 1 1 0.35",
        help="quoted MuJoCo RGBA used to draw generated spheres",
    )
    parser.add_argument(
        "--sphere-view",
        choices=("overlay", "spheres-only"),
        default="overlay",
        help="show transparent robot meshes with spheres, or spheres alone",
    )
    parser.add_argument(
        "--robot-alpha",
        type=float,
        default=0.25,
        help="visual-mesh opacity in overlay mode (0 to 1)",
    )
    parser.add_argument("--control-hz", type=float, default=CONTROL_HZ)
    parser.add_argument(
        "--left-position",
        type=float,
        nargs=3,
        default=LEFT_ARM_SPAWN_POSITION,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--right-position",
        type=float,
        nargs=3,
        default=RIGHT_ARM_SPAWN_POSITION,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--left-euler-deg",
        type=float,
        nargs=3,
        default=LEFT_ARM_SPAWN_EULER_XYZ_DEGREES,
        metavar=("RX", "RY", "RZ"),
    )
    parser.add_argument(
        "--right-euler-deg",
        type=float,
        nargs=3,
        default=RIGHT_ARM_SPAWN_EULER_XYZ_DEGREES,
        metavar=("RX", "RY", "RZ"),
    )
    return parser.parse_args()


def generate_sphere_model(args):
    """Run the shared MJCF fitter with this dual-robot scene."""
    command = [
        sys.executable,
        str(SPHERE_GENERATOR_PATH),
        "--input-xml",
        str(SOURCE_MODEL_PATH),
        "--output-xml",
        str(SPHERE_MODEL_PATH),
        "--sphere-density",
        str(args.sphere_density),
        "--iterations",
        str(args.fit_iterations),
        "--fit-type",
        args.fit_type,
        "--rgba",
        args.sphere_rgba,
    ]
    for body in args.fit_body:
        command.extend(("--body", body))
    for body_count in args.num_spheres:
        command.extend(("--num-spheres", body_count))
    if args.compute_fit_metrics:
        command.append("--compute-metrics")
    if args.replace_collisions_with_spheres:
        print(
            "WARNING: sphere contact geoms can create robot self-contact; "
            "use this mode intentionally."
        )
    else:
        command.extend(("--keep-source-collisions", "--visualization-only"))

    print("Generating dual-Franka collision spheres with cuRobo...")
    subprocess.run(command, check=True)


def resolve_model_path(args):
    if args.no_spheres:
        return SOURCE_MODEL_PATH
    if (
        args.regenerate_spheres
        or args.replace_collisions_with_spheres
        or not SPHERE_MODEL_PATH.exists()
    ):
        generate_sphere_model(args)
    else:
        print(
            f"Reusing generated sphere model: {SPHERE_MODEL_PATH}\n"
            "Pass --regenerate-spheres to refit it."
        )
    return SPHERE_MODEL_PATH


def configure_sphere_visualization(model, args):
    """Put fitted spheres in viewer group 4 and reveal the spheres clearly."""
    if not 0.0 <= args.robot_alpha <= 1.0:
        raise ValueError("--robot-alpha must be between 0 and 1")

    sphere_geom_ids = [
        geom_id
        for geom_id in range(model.ngeom)
        if model.geom(geom_id).name.startswith("spherefit_")
    ]
    if not sphere_geom_ids:
        return []

    for geom_id in sphere_geom_ids:
        # Group 3 is conventionally hidden collision geometry. Group 4 is
        # reserved here for the fitted-sphere overlay.
        model.geom_group[geom_id] = 4
    if args.sphere_view == "overlay":
        for geom_id in range(model.ngeom):
            if int(model.geom_group[geom_id]) == 2:
                model.geom_rgba[geom_id, 3] = args.robot_alpha
    return sphere_geom_ids


def main():
    args = parse_args()
    model_path = resolve_model_path(args)
    model = mujoco.MjModel.from_xml_path(model_path.as_posix())
    sphere_geom_ids = configure_sphere_visualization(model, args)
    set_robot_base_pose(
        model,
        "franka1",
        args.left_position,
        args.left_euler_deg,
    )
    set_robot_base_pose(
        model,
        "franka2",
        args.right_position,
        args.right_euler_deg,
    )

    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home1").id)
    mujoco.mj_forward(model, data)

    arm_dofs = np.concatenate(
        (
            joint_dof_indices(model, LEFT_ARM_JOINTS),
            joint_dof_indices(model, RIGHT_ARM_JOINTS),
        )
    )
    substeps = max(1, round((1.0 / args.control_hz) / model.opt.timestep))
    rate = RateLimiter(frequency=args.control_hz, warn=False)

    sphere_count = len(sphere_geom_ids)
    print(f"Loaded robots-only model: {model_path}")
    print(f"Bodies: {model.nbody}, joints: {model.njnt}, actuators: {model.nu}")
    print(f"Generated collision spheres: {sphere_count}")
    print("Holding both robots at the home keyframe; close the viewer to exit.")

    with mujoco.viewer.launch_passive(
        model,
        data,
        show_left_ui=False,
        show_right_ui=False,
    ) as viewer:
        # Show the dedicated fitted-sphere group and hide ordinary collision
        # meshes. In spheres-only mode, hide visual meshes as well.
        viewer.opt.geomgroup[3] = 0
        viewer.opt.geomgroup[4] = 1
        viewer.opt.geomgroup[2] = int(args.sphere_view == "overlay")
        viewer.cam.lookat[:] = [0.1, 0.0, 0.35]
        viewer.cam.azimuth = 70
        viewer.cam.elevation = -20
        viewer.cam.distance = 2.5

        while viewer.is_running():
            for _ in range(substeps):
                if ENABLE_ARM_BIAS_COMPENSATION:
                    data.qfrc_applied[arm_dofs] = data.qfrc_bias[arm_dofs]
                mujoco.mj_step(model, data)
            viewer.sync()
            rate.sleep()


if __name__ == "__main__":
    main()
