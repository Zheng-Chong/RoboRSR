"""Geometry helpers for the final approximate Dex1-1 Advanced V1 only.

No measurement fitting, legacy model generation, optical calibration or asset installation.
"""

from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

BASE_MESH = "meshes/dex1_1/base_link.STL"
WRIST_CAM_BODY_TOP = 0.0335
DEFAULT_PROFILE = Path(__file__).with_name("profile.json")


def split_wrist_camera(stl, body_top=WRIST_CAM_BODY_TOP):
    """Identify the native mesh's body, tall support, housing and high accessories.

    This is a topology-dependent decomposition, not a measured installation or an optical calibration.
    Reject incompatible geometry instead of silently treating a standard gripper component as a camera.
    """
    import trimesh  # Isaac Sim env

    parts = trimesh.load(stl, process=True).split(only_watertight=False)
    bracket = max(parts, key=lambda p: p.bounds[1, 2])
    above = [p for p in parts if p.bounds[0, 2] > body_top]
    if not above or not bracket.bounds[0, 2] < body_top < bracket.bounds[1, 2]:
        raise ValueError("Mesh does not match the supported Dex1 camera/support topology")
    camera = max(above, key=lambda p: p.extents[2])
    body = [p for p in parts if p is not bracket and all(p is not a for a in above)]
    return body, bracket, camera, [p for p in above if p is not camera]


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def mesh_path(root, filename):
    """Only accept self-contained relative mesh paths, including for read-only inputs."""
    p = Path(filename)
    if p.is_absolute() or ".." in p.parts or ":" in filename:
        raise ValueError(f"Expected a relative mesh filename: {filename}")
    result = root / p
    if not result.is_file():
        raise ValueError(f"Missing mesh: {result}")
    return result


def mount_profile(path=None):
    path = DEFAULT_PROFILE if path is None else Path(path)
    profile = json.loads(path.read_text())
    fields = {
        "source_split_height_m",
        "seat_height_m",
        "forward_shift_m",
        "lateral_shift_m",
        "extra_rpy_deg",
        "support_collision_band_height_m",
        "status",
        "basis",
        "solid_camera",
    }
    if set(profile) != fields:
        raise ValueError("Final V1 profile must contain only the documented geometry fields")
    if not profile["solid_camera"]:
        raise ValueError("Final V1 requires the solid housing and upright dimensions")
    for key in ("source_split_height_m", "seat_height_m", "forward_shift_m", "lateral_shift_m"):
        if not np.isfinite(profile[key]) or abs(profile[key]) > 0.2:
            raise ValueError(f"Invalid geometry offset in metres: {key}")
    angles = np.asarray(profile["extra_rpy_deg"], dtype=float)
    if angles.shape != (3,) or not np.isfinite(angles).all():
        raise ValueError("extra_rpy_deg must contain three finite angles")
    if not profile["status"] or not profile["basis"]:
        raise ValueError("Geometry provenance/status must be explicit")
    band_height = profile["support_collision_band_height_m"]
    if not np.isfinite(band_height) or band_height < 0 or band_height > 0.02:
        raise ValueError("support_collision_band_height_m must be between 0 and 0.02 metres")
    solid = profile["solid_camera"]
    if solid is not None:
        fields = {
            "housing_back_depth_m",
            "housing_edge_inset_m",
            "corner_radius_m",
            "mount_pad_thickness_m",
            "mount_pad_margin_m",
        }
        if not isinstance(solid, dict) or set(solid) != fields:
            raise ValueError("solid_camera must contain only the documented geometry dimensions")
        if any(not np.isfinite(v) or not 0 < v < 0.02 for v in solid.values()):
            raise ValueError("Invalid approximate solid camera dimensions")
        if not np.allclose(profile["extra_rpy_deg"], 0):
            raise ValueError("Solid-camera V1 retains the source mechanical angle")
    return profile


def geometry_parts(source, profile):
    """Reuse native mechanical parts and assemble the final solid camera profile."""
    body, support, housing, accessories = split_wrist_camera(source, profile["source_split_height_m"])
    cut = support.slice_plane([0, 0, profile["source_split_height_m"]], [0, 0, -1], cap=True)
    if cut is None or len(cut.faces) == 0 or not cut.is_watertight:
        raise ValueError("Support clipping did not produce a closed solid")
    pivot = housing.bounds.mean(axis=0)
    rotation = Rotation.from_euler("xyz", profile["extra_rpy_deg"], degrees=True).as_matrix()
    housing.vertices = (housing.vertices - pivot) @ rotation.T + pivot
    translation = np.array(
        [profile["lateral_shift_m"], profile["forward_shift_m"], profile["seat_height_m"] - housing.bounds[0, 2]]
    )
    housing.apply_translation(translation)
    # Legacy seating discards the entire upper support, including the housing's mounting plate.
    # Reuse that native plate/post at the same transform as the housing, shortening its lower end.
    upper = support.copy()
    upper.vertices = (upper.vertices - pivot) @ rotation.T + pivot + translation
    gap = profile["seat_height_m"] - profile["source_split_height_m"]
    if gap < -1e-7:
        raise ValueError("Seat below the body top would bury the housing; provide a compatible model instead")
    if not upper.bounds[0, 2] < profile["source_split_height_m"] < upper.bounds[1, 2]:
        raise ValueError("Relocated native support cannot reach the body plane for this mount")
    upper = upper.slice_plane([0, 0, profile["source_split_height_m"]], [0, 0, 1], cap=True)
    if upper is None or not upper.is_watertight:
        raise ValueError("Relocated support clipping did not produce a closed solid")
    upper, housing = solid_camera_parts(upper, housing, profile["solid_camera"])
    return {"body": body, "support": [cut, upper], "housing": [housing]}, accessories


def solid_camera_parts(upper, housing, dimensions):
    """Add a rear electronics case and solid pedestal; leave native lens/front plate unscaled.

    The source front panel is about 1.6 mm thick. Its rear-facing plane and original X/T bounds
    locate an approximate case, while native support sections locate the upright and foot.
    The new local dimensions are engineering approximations, not physical measurements.
    """
    normal = np.array([0, np.cos(np.deg2rad(35)), -np.sin(np.deg2rad(35))])
    tangent = np.array([0, -normal[2], normal[1]])
    basis = np.array([[1, 0, 0], normal, tangent])
    local = housing.vertices @ basis.T
    low, high = local.min(axis=0), local.max(axis=0)
    inset, radius = dimensions["housing_edge_inset_m"], dimensions["corner_radius_m"]
    # Convex rounded rectangular case in the face's X/T plane, behind the original rear skin.
    x0, x1 = low[0] + inset, high[0] - inset
    z0, z1 = low[2] + inset, high[2] - inset
    if 2 * radius >= min(x1 - x0, z1 - z0):
        raise ValueError("Case corner radius exceeds the native front-panel envelope")
    outline = []
    for cx, cz, angle in (
        (x1 - radius, z1 - radius, 0),
        (x0 + radius, z1 - radius, 90),
        (x0 + radius, z0 + radius, 180),
        (x1 - radius, z0 + radius, 270),
    ):
        for a in np.deg2rad(np.linspace(angle, angle + 90, 9)):
            outline.append([cx + radius * np.cos(a), cz + radius * np.sin(a)])
    front = low[1] + 0.0002  # small overlap joins the case to the native panel; not a measured offset
    points = [[x, n, z] for n in (front - dimensions["housing_back_depth_m"], front) for x, z in outline]
    case = trimesh.convex.convex_hull(np.array(points) @ basis)
    housing = trimesh.boolean.union([housing, case], engine="manifold")
    base_z = upper.bounds[0, 2]
    section = upper.section(plane_origin=[0, 0, base_z + 1e-6], plane_normal=[0, 0, 1])
    bounds = section.bounds
    rear = bounds[0, 1]
    vertical = np.max(np.abs(upper.triangles[:, :, 1] - rear), axis=1) < 1e-7
    top_z = float(upper.triangles[vertical, :, 2].max())
    if top_z - base_z < 0.005:
        raise ValueError("V1 requires a visible upright; refusing the truncated H/D mount")
    low_post = np.array([bounds[0, 0] + 0.0008, rear + 0.0005, base_z])
    high_post = np.array([bounds[1, 0] - 0.0008, bounds[1, 1] - 0.0005, top_z + 0.0002])
    post = trimesh.creation.box(extents=high_post - low_post)
    post.apply_translation((low_post + high_post) / 2)
    margin = dimensions["mount_pad_margin_m"]
    pad_low = np.array([bounds[0, 0] - margin, rear - margin, base_z])
    pad_high = np.array([bounds[1, 0] + margin, bounds[1, 1] + margin, base_z + dimensions["mount_pad_thickness_m"]])
    pad = trimesh.creation.box(extents=pad_high - pad_low)
    pad.apply_translation((pad_low + pad_high) / 2)
    upper = trimesh.boolean.union([upper, post, pad], engine="manifold")
    if not upper.is_volume or not housing.is_volume:
        raise ValueError("V1 camera case/pedestal must be closed solids")
    return upper, housing


def export_geometry(parts, out, support_band_height=0.0):
    folder = out / "meshes/camera_geometry"
    folder.mkdir(parents=True)
    visuals, collisions, report = {}, {}, {}
    for name, meshes in parts.items():
        visual = trimesh.util.concatenate(meshes)
        rel = f"meshes/camera_geometry/{name}_visual.STL"
        visual.export(out / rel)
        visuals[name] = rel
        hulls, skipped = [], []
        for index, mesh in enumerate(meshes):
            # Two-sided sheets and numerical fragments have no 3D collision volume.
            if len(mesh.vertices) < 4 or np.linalg.matrix_rank(mesh.vertices - mesh.vertices.mean(axis=0)) < 3:
                skipped.append(index)
                continue
            pieces = [("", mesh)]
            if name == "support" and index == 1 and support_band_height > 0:
                # The photo shows a C-shaped mounting plate. A single convex hull fills its recess.
                # Use capped horizontal slices; each resulting hull stays within its own height band.
                levels = np.arange(mesh.bounds[0, 2], mesh.bounds[1, 2], support_band_height)
                pieces = []
                for band, lower in enumerate(levels):
                    # Boolean clipping handles the native plate's coplanar holes more reliably
                    # than sequential capped slice_plane calls on this tessellation.
                    slab = trimesh.creation.box(
                        extents=[mesh.extents[0] + 0.002, mesh.extents[1] + 0.002, support_band_height]
                    )
                    slab.apply_translation([*mesh.bounds.mean(axis=0)[:2], lower + support_band_height / 2])
                    piece = trimesh.boolean.intersection([mesh, slab], engine="manifold")
                    if piece is None or not piece.is_watertight:
                        raise ValueError(f"Open support collision band: {band}")
                    pieces.append((f"_band_{band:02d}", piece))
                if not np.isclose(sum(m.volume for _, m in pieces), mesh.volume, rtol=0.001, atol=1e-12):
                    raise ValueError("Support collision bands do not cover the source solid")
            for suffix, piece in pieces:
                hull = piece.convex_hull
                if abs(hull.volume) < 1e-14:
                    skipped.append(index)
                    continue
                if not hull.is_watertight or not hull.is_convex or not np.isfinite(hull.vertices).all():
                    raise ValueError(f"Invalid convex collision: {name}/{index}{suffix}")
                filename = f"meshes/camera_geometry/{name}_collision_{index:02d}{suffix}.STL"
                hull.export(out / filename)
                hulls.append(filename)
        if not hulls:
            raise ValueError(f"No solid collision geometry for {name}")
        collisions[name] = hulls
        report[name] = {
            "visual_bounds_m": visual.bounds.tolist(),
            "visual_triangles": len(visual.faces),
            "collision_pieces": len(hulls),
            "nonvolumetric_components_skipped": skipped,
            "method": "one convex hull per connected solid component; cavities within each component approximated",
            "upper_support_band_height_m": support_band_height if name == "support" else None,
        }
    return visuals, collisions, report


def replace_geometry(root, visuals, collisions):
    changed = []
    for link in root.findall("link"):
        originals = {kind: [] for kind in ("visual", "collision")}
        for kind in originals:
            originals[kind] = [
                el for el in link.findall(kind) if any(m.get("filename") == BASE_MESH for m in el.iter("mesh"))
            ]
        if not any(originals.values()):
            continue
        if any(len(originals[kind]) != 1 for kind in originals):
            raise ValueError(f"Unexpected base visual/collision count for {link.get('name')}")
        for kind, templates in originals.items():
            template = templates[0]
            link.remove(template)
            files = {name: [path] for name, path in visuals.items()} if kind == "visual" else collisions
            for name, paths in files.items():
                for index, filename in enumerate(paths):
                    node = deepcopy(template)
                    node.set("name", f"{name}_{kind}_{index:02d}")
                    node.find("geometry/mesh").set("filename", filename)
                    link.append(node)
        changed.append(link.get("name"))
    if len(changed) != 2:
        raise ValueError(f"Expected two v4 native base links, found {changed}")
    return changed


def standalone_gripper(root):
    selected = {"left_dex1_base_link"}
    joints = []
    for _ in range(len(root.findall("link"))):
        for joint in root.findall("joint"):
            child = joint.find("child").get("link")
            if joint.find("parent").get("link") in selected and child not in selected:
                selected.add(child)
                joints.append(joint)
    out = ET.Element("robot", name="dex1_camera_geometry_reference")
    out.extend(deepcopy(link) for link in root.findall("link") if link.get("name") in selected)
    out.extend(deepcopy(joint) for joint in joints)
    if len(selected) < 8:
        raise ValueError("Incomplete v4 gripper subtree")
    return out


def save_urdf(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def xml_value(node):
    """Compare URDF semantics without mistaking pretty-print whitespace for changes."""
    if node is None:
        return None
    return node.tag, sorted(node.attrib.items()), (node.text or "").strip(), [xml_value(child) for child in node]


def transforms(root, opening):
    names = {el.get("name") for el in root.findall("link")}
    children = {j.find("child").get("link") for j in root.findall("joint")}
    roots = names - children
    if len(roots) != 1:
        raise ValueError("Expected one URDF root")
    poses = {next(iter(roots)): np.eye(4)}
    for _ in range(len(names)):
        for joint in root.findall("joint"):
            parent, child = (joint.find(tag).get("link") for tag in ("parent", "child"))
            if parent not in poses or child in poses:
                continue
            pose = origin_transform(joint.find("origin"))
            if joint.get("type") == "prismatic":
                pose[:3, 3] += pose[:3, :3] @ (np.fromstring(joint.find("axis").get("xyz"), sep=" ") * opening)
            elif joint.get("type") not in ("fixed",):
                raise ValueError("Only a standalone prismatic gripper is supported by the geometry check")
            poses[child] = poses[parent] @ pose
    if set(poses) != names:
        raise ValueError("Disconnected gripper links")
    return poses


def origin_transform(origin):
    result = np.eye(4)
    if origin is not None:
        result[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        result[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
    return result


def sweep_check(root, folder, openings=None):
    """Exact intersections of convex proxies; not a dynamics or calibrated collision test."""
    results = []
    cache = {}
    for opening in np.linspace(-0.02, 0.0245, 21) if openings is None else openings:
        poses = transforms(root, opening)
        housing, fingers = [], []
        for link in root.findall("link"):
            for node in link.findall("collision"):
                filename = node.find("geometry/mesh").get("filename")
                is_housing = any(
                    marker in filename
                    for marker in ("/housing_collision_", "/support_collision_01", "/mount_lip_collision_")
                )
                is_finger = "finger" in link.get("name", "")
                if not is_housing and not is_finger:
                    continue
                if filename not in cache:
                    cache[filename] = trimesh.load(mesh_path(folder, filename), force="mesh").convex_hull
                mesh = cache[filename].copy()
                mesh.apply_transform(poses[link.get("name")] @ origin_transform(node.find("origin")))
                (housing if is_housing else fingers).append(mesh)
        if not housing or not fingers:
            raise ValueError("Missing housing or finger colliders")
        volume = 0.0
        for camera in housing:
            for finger in fingers:
                overlap = np.minimum(camera.bounds[1], finger.bounds[1]) - np.maximum(
                    camera.bounds[0], finger.bounds[0]
                )
                if np.all(overlap > 0):
                    intersection = trimesh.boolean.intersection([camera, finger], engine="manifold")
                    volume += abs(intersection.volume)
        results.append(
            {"joint_position_m": opening, "camera_assembly_finger_intersection_m3": volume, "passed": volume < 1e-12}
        )
    return {
        "scope": f"camera assembly (including optional mount lip) versus fingers at {len(results)} static openings",
        "samples": results,
    }


def compare_image(meshes, out):
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib.collections import PolyCollection
    import matplotlib.pyplot as plt

    views = [
        np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]]),
        Rotation.from_euler("xyz", [62, 0, -32], degrees=True).as_matrix(),
    ]
    figure, axes = plt.subplots(2, len(meshes), figsize=(5 * len(meshes), 8), squeeze=False)
    for row, basis in enumerate(views):
        projected = [mesh.triangles @ basis.T for mesh in meshes.values()]
        lo = np.min([p[:, :, :2].min(axis=(0, 1)) for p in projected], axis=0) * 1000 - 6
        hi = np.max([p[:, :, :2].max(axis=(0, 1)) for p in projected], axis=0) * 1000 + 6
        for col, ((name, mesh), triangles) in enumerate(zip(meshes.items(), projected)):
            axis = axes[row, col]
            order = triangles[:, :, 2].mean(axis=1).argsort()
            light = np.array([0.3, -0.4, 0.85])
            shades = 0.35 + 0.6 * np.abs(mesh.face_normals @ light)
            colors = np.column_stack([shades * 0.62, shades * 0.73, shades * 0.85, np.ones(len(shades))])
            axis.add_collection(PolyCollection(triangles[order, :, :2] * 1000, facecolors=colors[order], linewidths=0))
            axis.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), aspect="equal", title=name)
            axis.grid(alpha=0.2)
            axis.set_xlabel("mm (native mesh frame; standard axes aligned)")
    figure.suptitle("Approximate Advanced V1 visual/collision geometry - no optical calibration")
    figure.tight_layout()
    figure.savefig(out, dpi=140)
    plt.close(figure)


def export_inspection_assets(gripper, out):
    """Portable full-gripper previews in its control frame, with no sensor or physics settings."""
    poses = transforms(gripper, 0.0)
    for kind in ("visual", "collision"):
        scene = trimesh.Scene()
        scene.units = "m"
        for link in gripper.findall("link"):
            for index, node in enumerate(link.findall(kind)):
                filename = node.find("geometry/mesh").get("filename")
                mesh = trimesh.load(mesh_path(out, filename), force="mesh")
                mesh.apply_transform(poses[link.get("name")] @ origin_transform(node.find("origin")))
                scene.add_geometry(mesh, node_name=f"{link.get('name')}_{kind}_{index}")
        scene.export(str(out / f"left_gripper_{kind}.glb"))
    visual_names = ["body", "support", "housing"]
    if (out / "meshes/camera_geometry/mount_lip_visual.STL").is_file():
        visual_names.append("mount_lip")
    visual = trimesh.util.concatenate(
        [trimesh.load(out / f"meshes/camera_geometry/{name}_visual.STL") for name in visual_names]
    )
    colliders = trimesh.util.concatenate(
        [trimesh.load(path) for path in sorted((out / "meshes/camera_geometry").glob("*_collision_*.STL"))]
    )
    compare_image(
        {"Visual geometry": visual, "Separate convex collision pieces": colliders}, out / "collision_comparison.png"
    )
