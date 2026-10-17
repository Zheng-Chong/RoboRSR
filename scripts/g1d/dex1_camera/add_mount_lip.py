"""Add one visible mounting-pad return to a read-only smooth-tail V1, in a new directory."""

import argparse
from copy import deepcopy
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

if __package__:
    from .geometry import export_inspection_assets, save_urdf, sweep_check, write_json, xml_value
    from .smooth_tail import digest, native_components, unique_match
else:
    from geometry import export_inspection_assets, save_urdf, sweep_check, write_json, xml_value
    from smooth_tail import digest, native_components, unique_match


def make_lip(support, platform, profile):
    """Derive an exposed rear return from the existing pad and platform, without moving either."""
    base_z = float(support.bounds[0, 2])
    if not np.isclose(platform.bounds[1, 2], base_z, atol=profile["bounds_tolerance_m"]):
        raise ValueError("The existing pad and platform are not at the expected common seating height")
    section = support.section(plane_origin=[0, 0, base_z + profile["section_offset_m"]], plane_normal=[0, 0, 1])
    if section is None:
        raise ValueError("Missing original mounting pad section")
    bounds = section.bounds
    x0, x1 = bounds[:, 0]
    rear = float(bounds[0, 1])
    contact = float(platform.bounds[0, 1])
    depth, chamfer, overlap = (profile[k] for k in ("downward_depth_m", "outer_lower_chamfer_m", "pad_overlap_m"))
    if not 0 < chamfer < min(depth, contact - rear) < 0.01 or not 0 < overlap < 0.001:
        raise ValueError("Invalid approximate lip dimensions or existing pad overhang")
    # A convex five-sided section with a softened lower outer corner. Only the visible rear edge wraps down.
    yz = [
        [rear, base_z + overlap],
        [contact, base_z + overlap],
        [contact, base_z - depth],
        [rear + chamfer, base_z - depth],
        [rear, base_z - depth + chamfer],
    ]
    lip = trimesh.convex.convex_hull(np.array([[x, y, z] for x in (x0, x1) for y, z in yz]))
    if not lip.is_volume or not lip.is_convex:
        raise ValueError("Mount lip must be a closed convex solid")
    # No volume enters the unchanged platform or cylindrical tail; the lip joins the pad with a tiny overlap.
    assert np.isclose(lip.bounds[1, 1], contact)
    assert lip.bounds[1, 2] < base_z + 0.001
    return lip, {
        "pad_bounds_at_seat_m": bounds.tolist(),
        "platform_contact_y_m": contact,
        "seat_z_m": base_z,
        "lip_bounds_m": lip.bounds.tolist(),
        "visible_edge": "native -Y / platform rear side, toward the cylindrical tail",
    }


def add_mount_lip(asset_dir, out, profile_path):
    asset_dir, out, profile_path = (Path(p).resolve() for p in (asset_dir, out, profile_path))
    if out.exists() or out == asset_dir or out in asset_dir.parents or asset_dir in out.parents:
        raise ValueError("Output must be a new independent directory")
    before = {str(p.relative_to(asset_dir)): digest(p) for p in asset_dir.rglob("*") if p.is_file()}
    record = json.loads((asset_dir / "build_result.json").read_text())
    if not record.get("tail_adaptation") or record.get("mount_lip_adaptation"):
        raise ValueError("Require a smooth-tail V1 without an existing mounting lip")
    profile = json.loads(profile_path.read_text())
    folder = Path("meshes/camera_geometry")
    support = trimesh.load(asset_dir / folder / "support_visual.STL", force="mesh")
    bodies = native_components(asset_dir / folder / "body_visual.STL")
    platform = bodies[unique_match(bodies, profile["platform_bounds_m"], profile["bounds_tolerance_m"])]
    lip, dimensions = make_lip(support, platform, profile)
    overlap = trimesh.boolean.intersection([support, lip], engine="manifold")
    if not overlap.is_volume or overlap.volume < 1e-12:
        raise ValueError("Lip must connect to the unchanged mounting pad")
    # Existing CAD meshes remain byte-identical: the fixed-base assembly receives only two new mesh references.
    shutil.copytree(asset_dir, out, ignore=shutil.ignore_patterns("usd", "__pycache__"))
    visual = (folder / "mount_lip_visual.STL").as_posix()
    collision = (folder / "mount_lip_collision_00.STL").as_posix()
    lip.export(out / visual)
    lip.export(out / collision)
    affected = {"left_dex1_model_base_link", "right_dex1_model_base_link"}
    for name in ("g1d_dex1.urdf", "left_gripper.urdf"):
        root = ET.parse(asset_dir / name).getroot()
        original = deepcopy(root)
        added = 0
        for link in root.findall("link"):
            if link.get("name") not in affected:
                continue
            template = next(
                n
                for n in link.findall("visual")
                if n.find("geometry/mesh").get("filename").endswith("/support_visual.STL")
            )
            for kind, filename in (("visual", visual), ("collision", collision)):
                node = deepcopy(template)
                node.tag = kind
                node.set("name", f"mount_lip_{kind}_00")
                node.find("geometry/mesh").set("filename", filename)
                if kind == "collision" and node.find("material") is not None:
                    node.remove(node.find("material"))
                link.append(node)
                added += 1
        if added != (4 if name == "g1d_dex1.urdf" else 2):
            raise ValueError("Unexpected base-link count")
        save_urdf(root, out / name)
        stripped = ET.parse(out / name).getroot()
        for link in stripped.findall("link"):
            for node in list(link):
                if node.get("name", "").startswith("mount_lip_"):
                    link.remove(node)
        assert xml_value(stripped) == xml_value(original)
    gripper = ET.parse(out / "left_gripper.urdf").getroot()
    export_inspection_assets(gripper, out)
    check = sweep_check(gripper, out, openings=(-0.02, 0.0, 0.0245))
    assert all(s["passed"] for s in check["samples"])
    record["three_pose_collision_check"] = check
    record["mount_lip_adaptation"] = {
        "status": profile["status"],
        "dimensions": dimensions,
        "approximate_depth_m": profile["downward_depth_m"],
        "visual_mesh": visual,
        "collision_mesh": collision,
        "all_baseline_STL_files_byte_identical": True,
        "existing_camera_and_support_geometry_preserved": True,
        "platform_and_tail_geometry_preserved": True,
        "existing_urdf_elements_joints_inertials_and_transforms_preserved": True,
        "calibration_modified": False,
        "baseline_source_hashes": before,
    }
    write_json(out / "mount_lip_profile.json", profile)
    write_json(out / "build_result.json", record)
    allowed = {
        "g1d_dex1.urdf",
        "left_gripper.urdf",
        "left_gripper_visual.glb",
        "left_gripper_collision.glb",
        "collision_comparison.png",
        "build_result.json",
    }
    changed = [name for name, h in before.items() if not (out / name).exists() or digest(out / name) != h]
    assert set(changed) <= allowed, changed
    assert all(digest(asset_dir / name) == h for name, h in before.items())
    write_json(
        out / "mount_lip_check.json",
        {
            "passed": True,
            "changed_existing_files": changed,
            "all_baseline_STL_files_byte_identical": True,
            "source_files_unchanged": True,
            "old_urdf_elements_preserved": True,
            "new_meshes": [visual, collision],
            "scope": "One visible rear mounting return only; no hidden fasteners or precision hardware claim",
        },
    )
    print(f"Built independent mounting-lip variant: {out}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=Path(__file__).with_name("mount_lip_profile.json"))
    args = parser.parse_args()
    add_mount_lip(args.asset_dir, args.out, args.profile)
