"""Fit cuRobo collision spheres directly to a MuJoCo MJCF/XML model.

The input MJCF is compiled with MuJoCo, so includes and default classes are
resolved first. For every selected body, this utility combines the body's
collision geometry into one mesh, fits spheres with cuRobo, and writes those
spheres back into a complete, flattened MJCF file. The generated sphere
positions are body-local, so they follow the robot joints correctly.

By default, source collision geoms (group 3) are disabled in the output and
replaced by sphere collision geoms. Visual geoms remain unchanged.
"""

if __package__:
    from demos import _bootstrap
else:
    import _bootstrap

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh

from bridge.config import MUJOCO_SCENE
from curobo.sphere_fit import SphereFitType, fit_spheres_to_mesh


SPHERE_RGBA = "0 1 1 0.35"


def _format(values) -> str:
    return " ".join(f"{float(value):.8g}" for value in values)


def _pose_matrix(position, quaternion) -> np.ndarray:
    rotation = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rotation, np.asarray(quaternion, dtype=np.float64))
    transform = np.eye(4)
    transform[:3, :3] = rotation.reshape(3, 3)
    transform[:3, 3] = position
    return transform


def _mesh_geom(model: mujoco.MjModel, geom_id: int) -> trimesh.Trimesh:
    mesh_id = int(model.geom_dataid[geom_id])
    vertex_start = int(model.mesh_vertadr[mesh_id])
    vertex_count = int(model.mesh_vertnum[mesh_id])
    face_start = int(model.mesh_faceadr[mesh_id])
    face_count = int(model.mesh_facenum[mesh_id])
    vertices = model.mesh_vert[vertex_start : vertex_start + vertex_count].copy()
    faces = model.mesh_face[face_start : face_start + face_count].copy()
    return trimesh.Trimesh(vertices=vertices, faces=faces, process=False)


def _primitive_geom(model: mujoco.MjModel, geom_id: int) -> trimesh.Trimesh | None:
    geom_type = int(model.geom_type[geom_id])
    sx, sy, sz = model.geom_size[geom_id]

    if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
        return trimesh.creation.box(extents=[2 * sx, 2 * sy, 2 * sz])
    if geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
        return trimesh.creation.icosphere(subdivisions=2, radius=sx)
    if geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return trimesh.creation.cylinder(radius=sx, height=2 * sy, sections=24)
    if geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE:
        return trimesh.creation.capsule(radius=sx, height=2 * sy, count=[16, 16])
    if geom_type == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
        mesh = trimesh.creation.icosphere(subdivisions=2, radius=1.0)
        mesh.apply_scale([sx, sy, sz])
        return mesh
    return None


def body_collision_mesh(
    model: mujoco.MjModel,
    body_id: int,
    source_group: int,
) -> tuple[trimesh.Trimesh | None, list[int]]:
    """Combine one body's selected MuJoCo geoms in body-local coordinates."""
    parts = []
    selected_geom_ids = []
    start = int(model.body_geomadr[body_id])
    count = int(model.body_geomnum[body_id])

    for geom_id in range(start, start + count):
        if int(model.geom_group[geom_id]) != source_group:
            continue

        if int(model.geom_type[geom_id]) == mujoco.mjtGeom.mjGEOM_MESH:
            part = _mesh_geom(model, geom_id)
        else:
            part = _primitive_geom(model, geom_id)

        if part is None:
            continue

        part.apply_transform(
            _pose_matrix(model.geom_pos[geom_id], model.geom_quat[geom_id])
        )
        parts.append(part)
        selected_geom_ids.append(geom_id)

    if not parts:
        return None, []
    return trimesh.util.concatenate(parts), selected_geom_ids


def _body_elements(root: ET.Element) -> dict[str, ET.Element]:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("flattened MJCF has no <worldbody>")
    return {
        body.get("name"): body
        for body in worldbody.iter("body")
        if body.get("name") is not None
    }


def _disable_source_geoms(
    model: mujoco.MjModel,
    body_id: int,
    body_element: ET.Element,
    selected_geom_ids: list[int],
) -> None:
    """Match compiled geoms to direct body child geoms by their local order."""
    xml_geoms = body_element.findall("geom")
    start = int(model.body_geomadr[body_id])
    count = int(model.body_geomnum[body_id])
    model_geom_ids = list(range(start, start + count))
    if len(xml_geoms) != len(model_geom_ids):
        raise RuntimeError(
            f"cannot map source geoms for body {model.body(body_id).name}: "
            f"XML has {len(xml_geoms)}, model has {len(model_geom_ids)}"
        )
    selected = set(selected_geom_ids)
    for geom_id, geom_element in zip(model_geom_ids, xml_geoms):
        if geom_id in selected:
            geom_element.set("contype", "0")
            geom_element.set("conaffinity", "0")


def _parse_exact_counts(values: list[str]) -> dict[str, int]:
    result = {}
    for value in values:
        try:
            body, count = value.rsplit("=", 1)
            result[body] = int(count)
        except ValueError as error:
            raise argparse.ArgumentTypeError(
                f"expected BODY=COUNT, received {value!r}"
            ) from error
        if result[body] < 1:
            raise argparse.ArgumentTypeError("sphere count must be positive")
    return result


def fit_mjcf(args: argparse.Namespace) -> Path:
    input_xml = args.input_xml.expanduser().resolve()
    output_xml = args.output_xml.expanduser().resolve()
    output_xml.parent.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(input_xml))
    mujoco.mj_saveLastXML(str(output_xml), model)
    tree = ET.parse(output_xml)
    root = tree.getroot()
    body_elements = _body_elements(root)
    exact_counts = _parse_exact_counts(args.num_spheres)
    fit_type = SphereFitType(args.fit_type)
    selected_bodies = set(args.body) if args.body else None
    available_bodies = {model.body(i).name for i in range(1, model.nbody)}
    if selected_bodies is not None:
        unknown = selected_bodies - available_bodies
        if unknown:
            raise ValueError(
                f"unknown MuJoCo bodies {sorted(unknown)}; available bodies are "
                f"{sorted(available_bodies)}"
            )
    unknown_counts = set(exact_counts) - available_bodies
    if unknown_counts:
        raise ValueError(
            f"--num-spheres uses unknown bodies {sorted(unknown_counts)}; "
            f"available bodies are {sorted(available_bodies)}"
        )
    fitted_total = 0

    for body_id in range(1, model.nbody):
        body_name = model.body(body_id).name
        if selected_bodies is not None and body_name not in selected_bodies:
            continue

        mesh, selected_geom_ids = body_collision_mesh(
            model,
            body_id,
            args.source_group,
        )
        if mesh is None or body_name not in body_elements:
            continue

        print(f"Fitting {body_name} ({len(mesh.faces)} triangles)...")
        fit = fit_spheres_to_mesh(
            mesh,
            num_spheres=exact_counts.get(body_name),
            sphere_density=args.sphere_density,
            fit_type=fit_type,
            iterations=args.iterations,
            coverage_weight=args.coverage_weight,
            protrusion_weight=args.protrusion_weight,
            compute_metrics=args.compute_metrics,
        )
        centers = fit.centers.detach().cpu().numpy()
        radii = fit.radii.detach().cpu().numpy()

        body_element = body_elements[body_name]
        if not args.keep_source_collisions:
            _disable_source_geoms(
                model,
                body_id,
                body_element,
                selected_geom_ids,
            )

        for index, (center, radius) in enumerate(zip(centers, radii)):
            collision_bit = "0" if args.visualization_only else "1"
            ET.SubElement(
                body_element,
                "geom",
                {
                    "name": f"spherefit_{body_name}_{index:03d}",
                    "type": "sphere",
                    "pos": _format(center),
                    "size": f"{float(radius):.8g}",
                    "group": str(args.output_group),
                    "contype": collision_bit,
                    "conaffinity": collision_bit,
                    "density": "0",
                    "rgba": args.rgba,
                },
            )

        fitted_total += len(radii)
        summary = f"  -> {len(radii)} spheres"
        if fit.metrics is not None:
            summary += (
                f", coverage={fit.metrics.coverage * 100:.1f}%"
                f", protrusion={fit.metrics.protrusion * 100:.1f}%"
                f", mean_gap={fit.metrics.surface_gap_mean * 1000:.2f} mm"
            )
        print(summary)

    ET.indent(tree, space="  ")
    tree.write(output_xml, encoding="unicode", xml_declaration=False)

    if fitted_total == 0:
        raise RuntimeError(
            f"no group-{args.source_group} collision geometry was found to fit"
        )

    # Compilation is the acceptance check: invalid paths, XML, body placement,
    # or sphere parameters fail here before the generated file is handed off.
    fitted_model = mujoco.MjModel.from_xml_path(str(output_xml))
    print(f"Generated {fitted_total} spheres across {fitted_model.nbody - 1} bodies")
    print(f"Validated output: {output_xml}")
    return output_xml


def parse_args() -> argparse.Namespace:
    default_output = MUJOCO_SCENE.with_name("scene_spherefit.xml")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-xml", type=Path, default=MUJOCO_SCENE)
    parser.add_argument("--output-xml", type=Path, default=default_output)
    parser.add_argument("--source-group", type=int, default=3)
    parser.add_argument("--output-group", type=int, default=3)
    parser.add_argument("--sphere-density", type=float, default=1.0)
    parser.add_argument(
        "--num-spheres",
        action="append",
        default=[],
        metavar="BODY=COUNT",
        help="exact count for one body; repeat for multiple bodies",
    )
    parser.add_argument("--body", action="append", help="fit only this body; repeatable")
    parser.add_argument(
        "--fit-type",
        choices=[fit.value for fit in SphereFitType],
        default=SphereFitType.MORPHIT.value,
    )
    parser.add_argument("--iterations", type=int, default=200)
    parser.add_argument("--coverage-weight", type=float)
    parser.add_argument("--protrusion-weight", type=float)
    parser.add_argument("--compute-metrics", action="store_true")
    parser.add_argument("--keep-source-collisions", action="store_true")
    parser.add_argument(
        "--visualization-only",
        action="store_true",
        help="draw fitted spheres without using them for MuJoCo contacts",
    )
    parser.add_argument("--rgba", default=SPHERE_RGBA)
    return parser.parse_args()


if __name__ == "__main__":
    fit_mjcf(parse_args())
