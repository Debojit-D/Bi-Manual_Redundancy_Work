# Third-party notices

Original code in this repository is released under the Apache License 2.0
(see [`LICENSE`](LICENSE)). This file records every third-party component,
dependency, submodule, and vendored asset identified in this repository, its
license, and its relationship to this project. It was produced by directly
inspecting this repository's files, embedded license/README text, installed
package metadata, and (where a component's own files did not state a
license) the corresponding upstream repository's declared license.

Nothing below was guessed. Where a license could not be established with
confidence, it is explicitly marked **"Requires manual licensing review"**
rather than assumed.

## Python package dependencies

These are unmodified upstream packages installed from PyPI (declared in
`pyproject.toml`), not vendored into this repository. License field is as
declared by each package's own installed metadata (or, where empty, its
upstream GitHub repository).

| Package | License | Upstream |
|---|---|---|
| `mink` | Apache-2.0 | https://github.com/kevinzakka/mink |
| `mujoco` | Apache-2.0 | https://github.com/google-deepmind/mujoco |
| `daqp` | MIT | https://github.com/darnstrom/daqp |
| `qpsolvers` | LGPL-3.0 | https://github.com/qpsolvers/qpsolvers |
| `loop-rate-limiters` | Apache-2.0 | https://github.com/stephane-caron/loop-rate-limiters |
| `numpy` | BSD-3-Clause | https://github.com/numpy/numpy |
| `scipy` | BSD-3-Clause | https://github.com/scipy/scipy |
| `matplotlib` | PSF-based (with embedded BSD-style notices for bundled colormap code) | https://matplotlib.org |
| `seaborn` | BSD-3-Clause | https://github.com/mwaskom/seaborn |
| `tqdm` | MPL-2.0 AND MIT (dual, per package's own metadata) | https://github.com/tqdm/tqdm |
| `pytest` (dev-only, `[project.optional-dependencies].dev`) | MIT | https://github.com/pytest-dev/pytest |

`qpsolvers` (LGPL-3.0) is used unmodified as an external library dependency,
not linked statically or redistributed as modified source, so its copyleft
terms attach to `qpsolvers` itself, not to this repository's Apache-2.0 code.

## Git submodules

| Name | Upstream | License | Location | Relationship |
|---|---|---|---|---|
| `mujoco_curobo_bridge` | https://github.com/Debojit-D/mujoco_curobo_bridge | Apache-2.0 (`LICENSE` + `NOTICE`, Copyright 2026 Debojit Das) | `mujoco_curobo_bridge/` (git submodule) | Submodule (same author, separate repository). **Under active development**: APIs, structure, and functionality may change. |

`mujoco_curobo_bridge` is the same author's own separate GitHub repository,
pinned here via `.gitmodules` as a git submodule. It is not vendored source
in this repository; it carries its own Apache-2.0 `LICENSE` and `NOTICE`
files, independent of (though compatible with) this repository's own
Apache-2.0 license (submodules are independent repositories with
independent licensing). Its README notes that the repository is under
active development, so pins to it should track a specific commit rather
than assume API stability.

That submodule's own `README.md` documents that it talks to **NVIDIA
cuRobo** (Apache-2.0, https://github.com/NVlabs/curobo) as an external
runtime dependency the user installs separately (`git clone
https://github.com/NVlabs/curobo.git`, outside this repository); cuRobo
itself is not vendored in this repository or in the submodule.

## Robot models (MJCF, vendored, not submodules)

### `models/robots/franka_emika_panda/`

- **Upstream**: derived from MuJoCo Menagerie's `franka_emika_panda/`
  (https://github.com/google-deepmind/mujoco_menagerie), itself derived from
  Franka's publicly available URDF description
  (https://github.com/frankaemika/franka_description, Apache-2.0, confirmed
  via GitHub's repository license metadata).
- **License**: Apache-2.0. Confirmed two ways: (1) the embedded
  `models/robots/franka_emika_panda/LICENSE` file in this repository (2)
  Menagerie's own repository LICENSE file has a `franka_emika_panda/`
  section that is the identical Apache-2.0 text.
- **Relationship**: copied asset, with local modifications (this
  repository adds `dual_panda_*.xml` scene compositions, MJX variants, and
  the sphere-fit collision scene layered on top of the vendored geometry and
  MJCF).
- **Action taken**: none. The embedded `LICENSE` file is preserved
  byte-for-byte, unchanged.

### `models/robots/franka_fr3/`

- **Upstream**: derived from MuJoCo Menagerie's `franka_fr3/`
  (https://github.com/google-deepmind/mujoco_menagerie), itself derived from
  `frankaemika/franka_description` (Apache-2.0).
- **License**: Apache-2.0. Confirmed the same way as the Panda model above
  (embedded `LICENSE` file + Menagerie's own aggregate LICENSE file has a
  matching `franka_fr3/` Apache-2.0 section).
- **Relationship**: copied asset.
- **Action taken**: none. The embedded `LICENSE` file is preserved
  byte-for-byte, unchanged.

### `models/robots/heal/`

- **Upstream**: unknown. No `LICENSE`, `README`, or any embedded
  copyright/source comment exists anywhere in `dual_heal.xml`,
  `dual_heal_reconfigured_home.xml`, or the `meshes_new/*.STL` mesh files.
  Git history for this path goes back only to this repository's earliest
  recorded commits, with no upstream reference in any commit message.
- **License**: **Requires manual licensing review.** This directory name
  and the mesh set are consistent with a commercial cooperative-robot
  manufacturer's CAD-derived model, but nothing in the repository confirms
  the manufacturer, license, or redistribution terms, and none should be
  assumed.
- **Relationship**: unknown (possibly copied/exported CAD asset).
- **Action taken**: none. Not covered by this repository's Apache-2.0
  license.

### `models/objects/furniture/ventionTable/` and `ventionTable.xml`

- **Upstream**: `vikashplus/furniture_sim`
  (https://github.com/vikashplus/furniture_sim), stated explicitly in an
  embedded comment header in both `ventionTable_asset.xml` and
  `ventionTable_body.xml`: `Model :: Vention table (MuJoCoV2.0)` /
  `Details :: https://github.com/vikashplus/furniture_sim`.
- **License**: Apache-2.0. Confirmed two ways: (1) the same embedded header
  quotes the Apache-2.0 grant text directly, and (2) GitHub's repository
  license metadata for `vikashplus/furniture_sim` independently reports
  Apache-2.0.
- **Relationship**: copied asset. The embedded attribution/license comment
  is preserved unchanged in both files.
- **Action taken**: none.

## MuJoCo Menagerie-derived files

Both vendored Franka models above (`franka_emika_panda/`, `franka_fr3/`) are
the Menagerie-derived components in this repository; there are no other
Menagerie-derived files. See those two entries.

## cuRobo-derived configs/resources

`configs/robots/dual_panda_full_arm.json` is a project-original
configuration file (paths, sphere-fitting parameters) consumed by the
`mujoco_curobo_bridge` submodule's sphere-generation tooling. It contains no
copied cuRobo source or NVIDIA-authored content; it is this project's own
input to that tool. cuRobo itself is never vendored in this repository (see
the submodule entry above): it is installed separately by anyone who wants
to run the sphere-fitting workflow.

## Legacy ROS 1 / Gazebo code (`legacy/`)

`legacy/` is explicitly historical/unmaintained (see `legacy/README.md`) and
not part of the active, publicly reproducible pipeline. It was still
inspected for third-party content:

| Path | License (as declared in its own file) | Notes |
|---|---|---|
| `legacy/gazebo/legacy_gazebo_stack/panda_multiple_arms_moveit_config/package.xml` | BSD (declared in its own `<license>` tag) | MoveIt Setup Assistant-generated package; BSD is the tool's own default for generated packages, not edited here. |
| `legacy/gazebo/legacy_gazebo_stack/panda_multiple_arms/package.xml` | **Not declared** (`<license>TODO</license>`, an unfilled catkin template placeholder left by the original author) | Project-original xacro composing two Franka arms; references the separately-installed `franka_description` ROS package (Apache-2.0) via `$(find franka_description)` but does not copy its files into this repository. Not third-party code; the stale `TODO` placeholder was left as-is rather than edited in an unmaintained legacy file. |
| `legacy/gazebo/legacy_gazebo_stack/gazebo_assets/package.xml` | **Not declared** (`<license>TODO</license>`) | Project-original table URDF/mesh for the legacy Gazebo stack; same treatment as above. |

No other copied third-party source was identified elsewhere in the
repository (`src/bimanual_redundancy/` contains no inherited copyright or
license headers).

## Summary

| Component | Relationship | License | Status |
|---|---|---|---|
| Python dependencies (11 packages, table above) | dependency | Apache-2.0 / MIT / BSD-3-Clause / LGPL-3.0 / PSF / MPL-2.0+MIT (per-package) | Resolved |
| `mujoco_curobo_bridge` | submodule | Apache-2.0 | Resolved; under active development (own repo, own copyright) |
| `models/robots/franka_emika_panda/` | copied asset (modified) | Apache-2.0 | Resolved, embedded LICENSE preserved |
| `models/robots/franka_fr3/` | copied asset | Apache-2.0 | Resolved, embedded LICENSE preserved |
| `models/robots/heal/` | unknown | unknown | **Requires manual licensing review** |
| `models/objects/furniture/ventionTable*` | copied asset | Apache-2.0 | Resolved, embedded attribution preserved |
| `legacy/gazebo/.../panda_multiple_arms_moveit_config` | generated derivative | BSD (self-declared) | Resolved |
| `legacy/gazebo/.../panda_multiple_arms`, `.../gazebo_assets` | project-original, unlicensed placeholder | none declared (`TODO`) | Not third-party; left as-is (unmaintained legacy code) |

All original code in `src/`, `configs/`, `tests/`, and `docs/` (i.e.
everything not listed above as a third-party component) is covered by the
repository's root [`LICENSE`](LICENSE) (Apache-2.0).
