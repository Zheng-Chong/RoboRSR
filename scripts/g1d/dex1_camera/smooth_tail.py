"""Remove only the V1 tail's two protruding rings into a new, independent model directory."""

import argparse
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

if __package__:
    from .geometry import export_inspection_assets, save_urdf, sweep_check, write_json, xml_value
else:
    from geometry import export_inspection_assets, save_urdf, sweep_check, write_json, xml_value


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def triangles_digest(mesh):
    """Ignore only face/vertex ordering, retaining the delivered STL's exact float32 coordinates."""
    points = np.ascontiguousarray(mesh.triangles, dtype="<f4").view("V12").reshape(-1, 3)
    faces = np.ascontiguousarray(np.sort(points, axis=1)).view("V36").ravel()
    return sha256(np.sort(faces).tobytes()).hexdigest()


def unique_match(parts, bounds, tolerance):
    matches = [i for i, part in enumerate(parts) if np.allclose(part.bounds, bounds, rtol=0, atol=tolerance)]
    if len(matches) != 1:
        raise ValueError(f"Expected one supported V1 component at {bounds}; found {matches}")
    return matches[0]


def native_components(path):
    """Find connectivity on a merged copy without repairing or moving the original STL faces."""
    raw = trimesh.load(path, force="mesh", process=False)
    topology = raw.copy()
    topology.merge_vertices()
    groups = trimesh.graph.connected_components(topology.face_adjacency, nodes=np.arange(len(raw.faces)))
    return [raw.submesh([group], append=True, repair=False) for group in groups]


def cylinder_from_cover(cover, profile):
    section = cover.section(plane_origin=[0, profile["native_cylinder_section_y_m"], 0], plane_normal=[0, 1, 0])
    if section is None:
        raise ValueError("Missing native cylindrical midsection")
    radius = float(np.linalg.norm(section.vertices[:, [0, 2]], axis=1).max())
    if not 0.024 < radius < 0.026:
        raise ValueError(f"Unexpected native tail radius: {radius}")
    # Round CAD tessellation noise to a micrometre; this is not a measured hardware dimension.
    radius = round(radius, 6)
    low, high = cover.bounds[:, 1]
    rotation = trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])
    rotation[:3, 3] = [0, (low + high) / 2, 0]
    cylinder = trimesh.creation.cylinder(
        radius=radius, height=high - low, sections=profile["cylinder_sections"], transform=rotation
    )
    if not cylinder.is_volume or not cylinder.is_convex:
        raise ValueError("Tail cylinder must be a closed convex solid")
    return cylinder, radius


def smooth_tail(asset_dir, out, profile_path):
    asset_dir, out, profile_path = (p.resolve() for p in (asset_dir, out, profile_path))
    if out.exists() or asset_dir == out or asset_dir in out.parents or out in asset_dir.parents:
        raise ValueError("Output must be a new independent directory")
    profile = json.loads(profile_path.read_text())
    tolerance = profile["component_bounds_tolerance_m"]
    before = {p.relative_to(asset_dir).as_posix(): digest(p) for p in asset_dir.rglob("*") if p.is_file()}
    folder = Path("meshes/camera_geometry")
    originals = {name: native_components(asset_dir / folder / f"{name}_visual.STL") for name in ("body", "support")}
    rear = unique_match(originals["body"], profile["rear_cover_bounds_m"], tolerance)
    lower = unique_match(originals["body"], profile["lower_ring_bounds_m"], tolerance)
    upper = unique_match(originals["support"], profile["upper_ring_bounds_m"], tolerance)
    cylinder, radius = cylinder_from_cover(originals["body"][rear], profile)
    kept = {
        "body": [p for i, p in enumerate(originals["body"]) if i not in (rear, lower)],
        "support": [p for i, p in enumerate(originals["support"]) if i != upper],
    }
    # Locate the three old convex proxies by their source bounds, not unstable component indices.
    proxy_files = sorted((asset_dir / folder).glob("*_collision_*.STL"))
    proxies = [trimesh.load(p, force="mesh") for p in proxy_files]
    rear_proxy = proxy_files[unique_match(proxies, profile["rear_cover_bounds_m"], tolerance)].relative_to(asset_dir)
    removed = {
        proxy_files[unique_match(proxies, profile[f"{name}_ring_bounds_m"], tolerance)]
        .relative_to(asset_dir)
        .as_posix()
        for name in ("lower", "upper")
    }
    shutil.copytree(asset_dir, out, ignore=shutil.ignore_patterns("usd", "__pycache__"))
    trimesh.util.concatenate([*kept["body"], cylinder]).export(out / folder / "body_visual.STL")
    trimesh.util.concatenate(kept["support"]).export(out / folder / "support_visual.STL")
    cylinder.export(out / rear_proxy)
    for filename in removed:
        (out / filename).unlink()
    for filename in ("g1d_dex1.urdf", "left_gripper.urdf"):
        root = ET.parse(asset_dir / filename).getroot()
        expected = deepcopy(root)
        count = 0
        for link in root.findall("link"):
            for node in list(link.findall("collision")):
                mesh = node.find("geometry/mesh")
                if mesh is not None and mesh.get("filename") in removed:
                    if link.get("name") not in ("left_dex1_model_base_link", "right_dex1_model_base_link"):
                        raise ValueError("Ring collision unexpectedly belongs to another link")
                    link.remove(node)
                    count += 1
        if count != (4 if filename == "g1d_dex1.urdf" else 2):
            raise ValueError("Unexpected ring collision reference count")
        for link in expected.findall("link"):
            for node in list(link.findall("collision")):
                mesh = node.find("geometry/mesh")
                if mesh is not None and mesh.get("filename") in removed:
                    link.remove(node)
        save_urdf(root, out / filename)
        assert xml_value(ET.parse(out / filename).getroot()) == xml_value(expected)
    # Check the exported STL, including preservation of every triangle outside the requested regions.
    updated_body = trimesh.load(out / folder / "body_visual.STL", force="mesh", process=False)
    assert triangles_digest(updated_body) == triangles_digest(trimesh.util.concatenate([*kept["body"], cylinder]))
    updated_support = trimesh.load(out / folder / "support_visual.STL", force="mesh", process=False)
    assert triangles_digest(updated_support) == triangles_digest(trimesh.util.concatenate(kept["support"]))
    gripper = ET.parse(out / "left_gripper.urdf").getroot()
    export_inspection_assets(gripper, out)
    collision = sweep_check(gripper, out, openings=(-0.02, 0.0, 0.0245))
    assert all(s["passed"] for s in collision["samples"])
    record = json.loads((out / "build_result.json").read_text())
    record["three_pose_collision_check"] = collision
    for name in ("body", "support"):
        visual = trimesh.load(out / folder / f"{name}_visual.STL", force="mesh")
        info = record["mesh_parts"][name]
        info.update(
            visual_bounds_m=visual.bounds.tolist(),
            visual_triangles=len(visual.faces),
            collision_pieces=len(list((out / folder).glob(f"{name}_collision_*.STL"))),
        )
    record["tail_adaptation"] = {
        "status": profile["status"],
        "method": (
            "Replace rear cover by native-midsection-radius cylinder; remove only upper/lower ring solids and proxies"
        ),
        "radius_m_from_native_CAD": radius,
        "axial_bounds_y_m": cylinder.bounds[:, 1].tolist(),
        "camera_housing_upright_and_pad_preserved": True,
        "unaffected_body_triangles_preserved": True,
        "urdf_changes_only_removed_ring_collision_nodes": True,
        "removed_collision_files": sorted(removed),
        "replacement_collision_file": rear_proxy.as_posix(),
        "source_model_file_hashes": before,
        "calibration_modified": False,
        "inertials_preserved_not_reestimated": True,
    }
    write_json(out / "tail_profile.json", profile)
    write_json(out / "build_result.json", record)
    allowed = {
        "g1d_dex1.urdf",
        "left_gripper.urdf",
        "left_gripper_visual.glb",
        "left_gripper_collision.glb",
        "collision_comparison.png",
        "build_result.json",
        (folder / "body_visual.STL").as_posix(),
        (folder / "support_visual.STL").as_posix(),
        rear_proxy.as_posix(),
        *removed,
    }
    changed = [name for name, value in before.items() if not (out / name).exists() or digest(out / name) != value]
    if not set(changed) <= allowed:
        raise ValueError(f"Unexpected modifications outside tail: {set(changed) - allowed}")
    assert all(digest(asset_dir / name) == value for name, value in before.items())
    write_json(
        out / "tail_check.json",
        {
            "passed": True,
            "changed_or_removed_files": changed,
            "all_other_source_files_byte_identical": True,
            "source_files_unchanged": True,
            "camera_housing_upright_and_pad_geometry_preserved": True,
            "unaffected_body_triangles_preserved": True,
            "radius_m": radius,
            "geometry_scope": "Approximate tail exterior only; not a hardware measurement or optical calibration",
        },
    )
    print(f"Built smooth-tail V1 in {out}; native CAD radius={radius * 1000:g} mm")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=Path(__file__).with_name("tail_profile.json"))
    args = parser.parse_args()
    smooth_tail(args.asset_dir, args.out, args.profile)
