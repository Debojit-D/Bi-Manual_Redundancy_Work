"""Inspect the MuJoCo Panda model and display its configured home pose.

This first demo provides a quick check that the robot scene is usable before
introducing cuRobo. It loads the scene selected in ``bridge.config``, resets
the simulation to the model's ``home`` keyframe, and prints the model counts,
joint-to-qpos/qvel mapping, and initial joint positions. It then opens the
passive MuJoCo viewer and advances the simulation while the window is open.

Run this file directly from any directory, or as ``python -m
demos.01_load_panda`` from the ``mujoco_curobo_bridge`` directory.
"""

if __package__:
    from demos import _bootstrap
else:
    import _bootstrap

import mujoco
import mujoco.viewer

from bridge.config import MUJOCO_SCENE


def print_model_info(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    print(f"Scene: {MUJOCO_SCENE}")
    print(f"Bodies: {model.nbody}")
    print(f"Joints: {model.njnt}")
    print(f"Geometries: {model.ngeom}")
    print(f"Actuators: {model.nu}")

    print("\nJoint mapping:")
    for joint_id in range(model.njnt):
        name = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            joint_id,
        )
        print(
            f"  {joint_id:2d}  {name or '<unnamed>':20s} "
            f"qpos={model.jnt_qposadr[joint_id]:2d}  "
            f"qvel={model.jnt_dofadr[joint_id]:2d}  "
            f"type={model.jnt_type[joint_id]}"
        )

    print(f"\nHome qpos: {data.qpos}")


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MUJOCO_SCENE))
    data = mujoco.MjData(model)

    home_key = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_KEY,
        "home",
    )
    if home_key == -1:
        raise RuntimeError(f"Scene has no 'home' keyframe: {MUJOCO_SCENE}")

    mujoco.mj_resetDataKeyframe(model, data, home_key)
    mujoco.mj_forward(model, data)
    print_model_info(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
