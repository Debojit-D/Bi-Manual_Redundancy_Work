# MuJoCo project profiles

These profiles belong to this robot project rather than to the reusable
`mujoco_curobo_bridge` submodule.

Generate fitted spheres for both complete Panda robots from the repository
root:

```bash
python -m mujoco_curobo_bridge.robot_spheres \
  --config configs/robots/dual_panda_full_arm.json
```

When running the bridge as an editable package instead, the equivalent entry
point is:

```bash
mujoco-fit-spheres --config MUJOCO/configs/dual_panda_full_arm.json
```

The profile deliberately uses the robot-only MJCF. Environment geometry such
as tables should remain analytic obstacles and should not be sphere-fitted.

The runtime reference embodiment is registered as `dual_franka_panda` in
`bimanual_redundancy.systems`. Validate it with:

```bash
bimanual-redopt validate-robot --robot dual_franka_panda
```

See `docs/ADDING_A_ROBOT.md` for the minimal specification interface.
