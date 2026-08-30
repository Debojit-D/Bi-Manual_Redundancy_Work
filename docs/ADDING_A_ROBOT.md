# Adding a cooperative robot embodiment

The manipulability objectives are intended to remain independent of a
particular robot embodiment. Robot-specific kinematics, limits, and simulation
geometry are supplied through the cooperative-system specification.

`CooperativeSystemSpec` is deliberately small. It describes the model, two
controlled joint/actuator groups, two hand/contact frames, the manipulated
object, physical limits, required backend names, and optional collision
resources. It is not a universal robotics API, and the MuJoCo scene remains a
MuJoCo backend rather than being rewritten around Mink.

## Add an embodiment

1. Add the robot and cooperative-scene model files under `models/robots/`.
2. Define the left and right controlled joint and actuator groups with their
   position and velocity limits.
3. Define each hand/contact site used to form the hand Jacobian.
4. Define the manipulated object's body, free joint, reference site, and two
   rigid contact sites.
5. Add the remaining limits and required backend names, including the home
   keyframe, base bodies, target bodies, and gripper actuators.
6. Optionally define collision body groups, table geometry, and a fitted-sphere
   resource. Use `collision=None` when these are unavailable; objective math
   remains usable without collision penalties.
7. Register the `CooperativeSystemSpec` in
   `bimanual_redundancy.systems.SYSTEM_SPECS`.
8. Validate every name, mapping, limit, frame, and optional resource:

   ```bash
   bimanual-redopt validate-robot --robot dual_franka_panda
   ```

9. Run a static baseline first, with collision penalties disabled if the new
   embodiment has no collision specification.
10. Run velocity, force, direct directional-force, and indirect
    directional-force objectives in turn, checking Jacobian rank, limits,
    tracking, and collision diagnostics before creating a larger campaign.

The built-in `dual_franka_panda` is the reference specification. Its resolved
qpos/DoF maps, limits, sites, object frame, collision spheres, Jacobian sizes,
controller behavior, and paper objective values are regression tested.

## Assumption classification

| Assumption | Classification | Location |
|---|---|---|
| Eq. (8), (13), (14), (16), and (17), grasp matrix algebra, null-space projection | mathematical | `core/`; no embodiment names |
| Joint/actuator names, hand and object sites, qpos/DoF mapping, limits, base/gripper names | robot-specific | `CooperativeSystemSpec` and its registered embodiment |
| MuJoCo model loading, mocap targets, keyframes, rendering, actuator commands | simulation-backend-specific | `simulation/scene.py` and spec backend names |
| Initial object poses, trajectories, gains, desired wrench, stopping and recording | experiment-specific | `configs/paper/*.toml` |
| Fitted spheres, collision body groups, table geom candidates | robot- and backend-specific | optional `CollisionSpec`; collision backend code remains MuJoCo-specific |

Generic kinematic and objective utilities depend on resolved DoF arrays and
frame names supplied by the specification. This keeps them suitable for a
future adapter to Mink or another robotics library without importing paper
experiment code.

Contributions adding well-validated cooperative robot embodiments are welcome.
