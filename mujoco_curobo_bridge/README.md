# mujoco_curobo_bridge

A bridge between MuJoCo (simulation + viewer) and NVIDIA cuRobo (GPU-accelerated
kinematics, collision checking, and motion planning) for a Franka Panda arm.
A neutral `Obstacle` / `WorldManager` representation feeds both sides, so the
same obstacles are rendered in the MuJoCo viewer and checked against by cuRobo's
planner.

## Architecture

```
world/            Neutral obstacle representation (no MuJoCo or cuRobo import)
  obstacle.py       Obstacle dataclass: shape, position, size, rgba, enabled
  world_manager.py  Dict-backed collection of Obstacles

bridge/           Everything that talks to MuJoCo or cuRobo directly
  mujoco_loader.py    MjModel / MjData wrapper, viewer launch, home reset
  curobo_loader.py    CudaRobotModel wrapper, forward kinematics
  state_sync.py       MuJoCo qpos -> 7-dof Panda arm joint slice
  sphere_bridge.py    cuRobo FK -> collision sphere list (x, y, z, r)
  world_bridge.py     WorldManager -> curobo.geom.types.WorldConfig
  motion_gen_bridge.py  MotionGen wrapper: plan_to_pose()

viewer/           Draws into MuJoCo's passive-viewer overlay scene
  world_renderer.py   Draws Obstacles as MuJoCo geoms
  sphere_renderer.py  Draws cuRobo collision spheres

demos/            Runnable scripts, 01 (load model) through 13 (multi-waypoint
                   planning + playback), each adding one capability on top of
                   the last.
```

## Requirements

- NVIDIA GPU, Volta architecture or newer, 4GB+ VRAM
- Python 3.10 (cuRobo recommends 3.10; 3.8-3.10 supported)
- CUDA-capable PyTorch build matching your driver
- `git-lfs` (cuRobo's asset repo uses it)

## Setup

```bash
# 1. Clone this repo
git clone <your-repo-url> mujoco_curobo_bridge
cd mujoco_curobo_bridge

# 2. Virtual environment
python3.10 -m venv .venv
source .venv/bin/activate

# 3. PyTorch - install the build matching your CUDA driver first
#    (see https://pytorch.org/get-started/locally/), then:
pip install -r requirements.txt

# 4. cuRobo - build from source, PINNED to v0.7.8.
#    main/HEAD is cuRoboV2 (v0.8.0+, April 2026), a rewrite with a
#    DIFFERENT public API. This project uses the v1 API
#    (CudaRobotModel, RobotConfig.from_dict, MotionGenConfig.load_from_robot_config),
#    so installing main will break every import in bridge/.
sudo apt install git-lfs
git clone https://github.com/NVlabs/curobo.git
cd curobo && git checkout v0.7.8
pip install -e . --no-build-isolation   # ~20 minutes
cd ..

# 5. Robot assets - mujoco_menagerie (not bundled in this repo)
git clone https://github.com/google-deepmind/mujoco_menagerie.git

# 6. Point bridge/config.py at your local scene.xml
#    Edit MUJOCO_SCENE in bridge/config.py to the absolute path of
#    mujoco_menagerie/franka_emika_panda/scene.xml on your machine.
```

Verify the install:

```bash
python -m demos.02_inspect_model     # MuJoCo model loads, prints joint/body counts
python -m demos.04_curobo_joint      # cuRobo robot config loads
```

Run everything as a module from the repo root (`python -m demos.05_fk_bridge`,
not `python demos/05_fk_bridge.py`) - the demos import `bridge.*` and `world.*`
as top-level packages, which only resolves correctly when the repo root is on
`sys.path`.

## Running the demos

| # | Script | What it shows |
|---|--------|----------------|
| 01 | `01_load_panda.py` | Load the model, open the passive viewer |
| 02 | `02_inspect_model.py` | Print joint / body / actuator counts |
| 03 | `03_joint_mapping.py` | qpos/qvel index of every joint |
| 04 | `04_curobo_joint.py` | Load the Panda into cuRobo |
| 05-06 | `0{5,6}_fk_bridge.py` | Read MuJoCo qpos, run cuRobo forward kinematics |
| 07 | `07_collision_spheres.py` | Live collision-sphere overlay on the moving arm |
| 08 | `08_world_test.py` | Build a `WorldManager` with a few obstacles (no viewer) |
| 09 | `09_world_viewer.py` | Render those obstacles in the MuJoCo viewer |
| 10 | `10_world_config_test.py` | Convert `WorldManager` -> cuRobo `WorldConfig` |
| 11 | `11_motion_plan.py` | Plan a single collision-free trajectory (headless) |
| 12 | `12_planned_playback.py` | Plan + play back one trajectory in the viewer |
| 13 | `13_multi_motion.py` | Cycle through several waypoints indefinitely |

## Known issues / current state

- **Demo 12 mixes kinematic playback with physics.** It calls both
  `mj.set_qpos()` (teleports joints to the planned trajectory) and
  `mj.step()` (integrates real dynamics against the position actuators'
  `ctrl` targets, which are still set to the home keyframe) every frame.
  The two fight each other. Demo 13 has the correct pattern: `mj.forward()`
  only, never `mj.step()`, during kinematic playback.
- **Warm-up looks like a freeze.** `MotionGen.warmup()` JIT-compiles CUDA
  kernels for 30s-2min on first use. Both demo 12 and 13 launch the MuJoCo
  viewer *before* constructing `MotionGenBridge`, so the window can appear
  unresponsive during warmup. Harmless, just slow.
- **Demo 13's `WAYPOINTS[1] = (0.70, 0.20, ...)` sits at ~92% of the Panda's
  straight-line max reach** (~0.855m from the shoulder), with orientation
  fully locked to `home_quat`. This is the most likely single point of
  planning failure - watch `result.status` on that waypoint specifically.
- The four table **legs** and the **ground plane** in `scene.xml` are not
  represented in the cuRobo `WorldConfig` - only `table_top` is. Fine for
  above-table motion, not collision-checked if a goal or IK seed goes low.
- `WorldConfig` is built once at startup. Moving an `Obstacle` after that
  point updates the viewer but **not** the planner - use
  `motion_gen.update_world()` if the world becomes dynamic.

## Math behind it

**Forward kinematics** is the product of per-joint homogeneous transforms
along the kinematic chain:

$$T_{ee}(q) = T_{base} \prod_{i=1}^{7} T_{i-1,i}(q_i)$$

Orientation is tracked as a unit quaternion (cuRobo and MuJoCo both use
`w, x, y, z` order), with rotation angle $\theta$ about axis $\mathbf{n}$:

$$q = (w, x, y, z), \quad w = \cos(\theta/2), \quad (x,y,z) = \sin(\theta/2)\,\mathbf{n}$$

The robot is approximated as a set of spheres for cheap, closed-form
collision checking against `WorldConfig` cuboids (center $c$, half-extents
$h$):

$$d(x_s) = \lVert x_s - \mathrm{clamp}(x_s,\, c-h,\, c+h) \rVert - r$$

`MotionGen` finds a trajectory $q_{1:T}$ minimizing smoothness plus a
collision penalty that activates within a distance $\epsilon$ of any
obstacle, subject to reaching the goal pose:

$$\min_{q_{1:T}} \sum_t \lVert q_{t+1} - 2q_t + q_{t-1} \rVert^2 \;+\; w_c \sum_{t,s} \max(0,\, \epsilon - d_s(q_t))^2 \;+\; w_g \lVert \mathrm{Pose}(q_T) \ominus \mathrm{Pose}_{goal} \rVert^2$$

Orientation error between two quaternions is measured geodesically:

$$\theta_{err} = 2 \arccos\left(\left|\langle q_{goal}, q_{current} \rangle\right|\right)$$

And reachability is a simple radius check against the shoulder-centered
workspace sphere of radius $R_{max} \approx 0.855\text{m}$:

$$\lVert p_{goal} - p_{shoulder} \rVert \leq R_{max}$$

## Unit conventions

| Quantity | MuJoCo | cuRobo |
|---|---|---|
| Box / cylinder size | half-extents | full extents (`dims`, `height`) |
| Sphere radius | radius | radius (no conversion) |
| Quaternion order | `w, x, y, z` | `w, x, y, z` |
| Pose format | separate pos + quat | `[x, y, z, qw, qx, qy, qz]` |
