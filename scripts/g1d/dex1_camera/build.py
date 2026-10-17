"""Generate approximate Advanced Camera V1 on a production G1-D base, without installing it."""

import argparse
from copy import deepcopy
from hashlib import sha256
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

if __package__:
    from .geometry import (
        BASE_MESH,
        export_geometry,
        export_inspection_assets,
        geometry_parts,
        mesh_path,
        mount_profile,
        replace_geometry,
        save_urdf,
        standalone_gripper,
        sweep_check,
        write_json,
        xml_value,
    )
else:
    from geometry import (
        BASE_MESH,
        export_geometry,
        export_inspection_assets,
        geometry_parts,
        mesh_path,
        mount_profile,
        replace_geometry,
        save_urdf,
        standalone_gripper,
        sweep_check,
        write_json,
        xml_value,
    )


def build(base_urdf, native_mesh, profile_path, out):
    base_urdf, native_mesh, out = base_urdf.resolve(), native_mesh.resolve(), out.resolve()
    parents = (base_urdf.parent, native_mesh.parent)
    if out.exists() or any(p == out or p in out.parents or out in p.parents for p in parents):
        raise ValueError("Output must be a new independent directory")
    root = ET.parse(base_urdf).getroot()
    initial = deepcopy(root)
    source_hashes = {str(p): sha256(p.read_bytes()).hexdigest() for p in (base_urdf, native_mesh, profile_path)}
    profile = mount_profile(profile_path)
    parts, _ = geometry_parts(native_mesh, profile)
    out.mkdir(parents=True)
    visuals, collisions, mesh_report = export_geometry(parts, out, profile["support_collision_band_height_m"])
    # Normalize already-split V1 inputs too, allowing reproduction from the delivered model itself.
    for link in root.findall("link"):
        if link.get("name") not in ("left_dex1_model_base_link", "right_dex1_model_base_link"):
            continue
        for kind in ("visual", "collision"):
            nodes = link.findall(kind)
            template = deepcopy(nodes[0])
            template.find("geometry/mesh").set("filename", BASE_MESH)
            for node in nodes:
                link.remove(node)
            link.append(template)
    changed = replace_geometry(root, visuals, collisions)
    for mesh in root.iter("mesh"):
        name = mesh.get("filename")
        target = out / name
        if target.exists():
            continue
        source = mesh_path(base_urdf.parent, name)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        source_hashes[str(source)] = sha256(source.read_bytes()).hexdigest()
    for name in ("physics_contract.json", "left_robot_config.yml", "right_robot_config.yml", "LICENSE.unitree"):
        source = base_urdf.parent / name
        if source.is_file():
            shutil.copy2(source, out / name)
            source_hashes[str(source)] = sha256(source.read_bytes()).hexdigest()
    gripper = standalone_gripper(root)
    save_urdf(root, out / "g1d_dex1.urdf")
    save_urdf(gripper, out / "left_gripper.urdf")
    export_inspection_assets(gripper, out)
    collision_check = sweep_check(gripper, out, openings=(-0.02, 0.0, 0.0245))
    assert all(s["passed"] for s in collision_check["samples"])
    for node in initial.findall("joint"):
        assert xml_value(node) == xml_value(root.find(f"joint[@name='{node.get('name')}']"))
    for link in initial.findall("link"):
        other = root.find(f"link[@name='{link.get('name')}']")
        assert xml_value(link.find("inertial")) == xml_value(other.find("inertial"))
        if link.get("name") not in changed:
            assert xml_value(link) == xml_value(other)
    assert root.find("link[@name='d435_link']") is not None
    assert all(j.get("type") == "fixed" for j in root.findall("joint") if j.get("name").endswith("Wheel_Joint"))
    assert all(sha256(Path(p).read_bytes()).hexdigest() == h for p, h in source_hashes.items())
    write_json(out / "geometry_mount.json", profile)
    write_json(
        out / "build_result.json",
        {
            "status": "approximate_geometry_v1_not_calibrated",
            "changed_links": changed,
            "all_joints_inertials_and_other_links_preserved": True,
            "head_frame_locked_wheels_and_production_finger_collision_meshes_preserved": True,
            "source_files_unchanged": True,
            "source_hashes": source_hashes,
            "mesh_parts": mesh_report,
            "three_pose_collision_check": collision_check,
            "calibration_modified": False,
        },
    )
    print(f"Built approximate Advanced Camera V1: {out}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-urdf", type=Path, required=True)
    parser.add_argument("--native-mesh", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=Path(__file__).with_name("profile.json"))
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    build(args.base_urdf, args.native_mesh, args.profile, args.out)
