# MuJoCo project profiles

Sphere-fitting profiles for this project, distinct from the reusable
`mujoco_curobo_bridge` submodule itself.

Generate fitted spheres for both Panda robots from the repository root:

```bash
python -m mujoco_curobo_bridge.robot_spheres \
  --config configs/robots/dual_panda_full_arm.json
```

Or, with the bridge installed as an editable package:

```bash
mujoco-fit-spheres --config configs/robots/dual_panda_full_arm.json
```

The profile uses the robot-only MJCF; environment geometry such as tables
stays as analytic obstacles and is not sphere-fitted.

The runtime reference embodiment is `dual_franka_panda`, registered in
`bimanual_redundancy.systems`. Validate it with:

```bash
bimanual-redopt validate-robot --robot dual_franka_panda
```

See [`docs/ADDING_A_ROBOT.md`](../../docs/ADDING_A_ROBOT.md) for the
specification interface.
