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
  curobo_loader.py    Kinematics wrapper and forward kinematics
  state_sync.py       MuJoCo qpos -> 7-dof Panda arm joint slice
  sphere_bridge.py    cuRobo FK -> collision sphere list (x, y, z, r)
  world_bridge.py     WorldManager -> cuRobo v0.8 Scene
  motion_planner_bridge.py  MotionPlanner wrapper: plan_to_pose()

viewer/           Draws into MuJoCo's passive-viewer overlay scene
  world_renderer.py   Draws Obstacles as MuJoCo geoms
  sphere_renderer.py  Draws cuRobo collision spheres

demos/            Runnable scripts, 01 (load model) through 13 (multi-waypoint
                   planning + playback), each adding one capability on top of
                   the last.
```

## Requirements

- NVIDIA GPU newer than Turing with at least 4GB VRAM
- NVIDIA driver 580.65.06 or newer
- Python 3.10-3.13 (Python 3.11 is cuRobo's documented default)
- `uv` and `git`

## Setup

```bash
# 1. Clone this repository and enter its root
git clone <your-repo-url>
cd <repository-directory>
REPO_ROOT="$PWD"

# 2. One shared environment for MuJoCo and cuRobo
python3.11 -m venv .venv
source .venv/bin/activate

# 3. MuJoCo bridge dependencies
python -m pip install -r mujoco_curobo_bridge/requirements.txt

# 4. cuRobo v0.8 with CUDA 13 and its matching PyTorch wheel
git clone https://github.com/NVlabs/curobo.git ~/curobo
cd ~/curobo
git checkout a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9
uv pip install --python "$REPO_ROOT/.venv/bin/python" '.[cu13-torch]'
cd "$REPO_ROOT"

# 5. Robot assets
#    The demos use the Panda scene bundled in this repository under
#    MUJOCO/robot_descriptions/franka_emika_panda by default.
```

To use a different scene without editing the source, set `MUJOCO_SCENE`:

```bash
MUJOCO_SCENE=/path/to/scene.xml python -m demos.01_load_panda
```

Verify the install:

```bash
python -m demos.01_load_panda       # Inspect the home pose and open the viewer
python -m demos.04_curobo_joint      # cuRobo robot config loads
```

The demos support both module execution from `mujoco_curobo_bridge`
(`python -m demos.05_fk_bridge`) and direct file execution from any working
directory (`python /path/to/mujoco_curobo_bridge/demos/05_fk_bridge.py`).

## Running the demos

| # | Script | What it shows |
|---|--------|----------------|
| 01 | `01_load_panda.py` | Load the `home` keyframe, print model/joint information, and open the viewer |
| 02 | `02_fit_mujoco_spheres.py` | Fit cuRobo spheres directly to MJCF collision geometry and generate a new XML |
| 04 | `04_curobo_joint.py` | Load the Panda into cuRobo |
| 05-06 | `0{5,6}_fk_bridge.py` | Read MuJoCo qpos, run cuRobo forward kinematics |
| 07 | `07_collision_spheres.py` | Live collision-sphere overlay on the moving arm |
| 08 | `08_world_test.py` | Build a `WorldManager` with a few obstacles (no viewer) |
| 09 | `09_world_viewer.py` | Render those obstacles in the MuJoCo viewer |
| 10 | `10_world_config_test.py` | Convert `WorldManager` -> cuRobo v0.8 `Scene` |
| 11 | `11_motion_plan.py` | Plan a single collision-free trajectory (headless) |
| 12 | `12_planned_playback.py` | Plan + play back one trajectory in the viewer |
| 13 | `13_multi_motion.py` | Cycle through several waypoints indefinitely |

### Fit collision spheres directly from MJCF

Demo 02 compiles the input MJCF first, so included files and inherited defaults
are resolved. It fits each body's group-3 collision geometry with cuRobo and
writes a complete `scene_spherefit.xml`. Original collision geoms are retained
for reference but disabled; the generated body-local spheres replace them.
The checked-in Panda output uses density `1.0` and contains 158 fitted spheres.

```bash
python mujoco_curobo_bridge/demos/02_fit_mujoco_spheres.py \
  --sphere-density 1.0 \
  --iterations 200 \
  --compute-metrics
```

Set exact counts for selected bodies when automatic density is not desired:

```bash
python mujoco_curobo_bridge/demos/02_fit_mujoco_spheres.py \
  --body link5 --num-spheres link5=20 \
  --body hand  --num-spheres hand=12
```

Preview the generated model with the regular viewer:

```bash
MUJOCO_SCENE=MUJOCO/robot_descriptions/franka_emika_panda/scene_spherefit.xml \
  python mujoco_curobo_bridge/demos/01_load_panda.py
```

## Known issues / current state

- **Demo 13's `WAYPOINTS[1] = (0.70, 0.20, ...)` sits at ~92% of the Panda's
  straight-line max reach** (~0.855m from the shoulder), with orientation
  fully locked to `home_quat`. This is the most likely single point of
  planning failure - watch `result.status` on that waypoint specifically.
- The four table **legs** and the **ground plane** in `scene.xml` are not
  represented in the cuRobo `Scene` - only `table_top` is. Fine for
  above-table motion, not collision-checked if a goal or IK seed goes low.
- The cuRobo `Scene` is built once at startup. Moving an `Obstacle` after that
  point updates the viewer but **not** the planner - use
  `motion_bridge.planner.update_world()` if the world becomes dynamic.

## Math behind it

**Forward kinematics** is the product of per-joint homogeneous transforms
along the kinematic chain:

$$T_{ee}(q) = T_{base} \prod_{i=1}^{7} T_{i-1,i}(q_i)$$

Orientation is tracked as a unit quaternion (cuRobo and MuJoCo both use
`w, x, y, z` order), with rotation angle $\theta$ about axis $\mathbf{n}$:

$$q = (w, x, y, z), \quad w = \cos(\theta/2), \quad (x,y,z) = \sin(\theta/2)\,\mathbf{n}$$

The robot is approximated as a set of spheres for cheap, closed-form
collision checking against `Scene` cuboids (center $c$, half-extents
$h$):

$$d(x_s) = \lVert x_s - \mathrm{clamp}(x_s,\, c-h,\, c+h) \rVert - r$$

`MotionPlanner` finds a trajectory $q_{1:T}$ minimizing smoothness plus a
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
