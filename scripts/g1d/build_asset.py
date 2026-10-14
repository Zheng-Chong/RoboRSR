"""Build the Unitree G1-D + Dex1-1 robot asset for RoboDojo.

Takes the team's assembled G1-D + Dex1-1 (camera version) URDF, v4: official g1_d.urdf with the standalone
dex1_1.urdf mounted on each wrist (assembled_v4_coupled/: g1_d_full_dex1_v4.urdf, meshes/, physics_contract.json),
adds the G1 head D435 frame from unitree_ros g1_29dof_mode_15_with_dex1_1.urdf and locks the wheels (tabletop only).
Writes:
    Assets/Robots/g1d/                     g1d_dex1.urdf, meshes/, physics_contract.json (self-collision filters),
                                           {left,right}_robot_config.yml
    Assets/Eval_Layout/RoboDojo/g1d/       arx_x5 layouts minus the camera stand (it would sit inside the robot),
                                           plus layouts for the G1-D tasks in G1D_TASKS (bottle swapped for our
                                           pitcher of red beans, set in the real office scene Room/real_scene1)
    Assets/Material/Fluid/RedBean.mdl
    Assets/Room/real_scene1/               with --scene: real_scene1.usd (room + table + lights) over the copied usd/

Usage:
    python scripts/g1d/build_asset.py --v4 DIR              # fetch the unitree_ros G1 URDF from GitHub into --src
    python scripts/g1d/build_asset.py --v4 DIR --src DIR    # DIR mirrors unitree_ros (robots/...); reused if present
    python scripts/g1d/build_asset.py --v4 DIR --scene sim_scene_opus/usd   # also (re)install the real office scene
"""

import argparse
import json
from pathlib import Path
import shutil
import urllib.request
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
UNITREE_RAW = "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/"
V4_URDF = "g1_d_full_dex1_v4.urdf"
G1_DEX1_URDF = "robots/g1_description/g1_29dof_mode_15_with_dex1_1.urdf"
# Office scene reconstructed from a phone video (sim_scene_opus/usd): room, white lift table and lights in one Room USD,
# replacing the stock room, table cube, ground cube and HDR dome of the G1-D task layouts. Its robot-side table edge
# (scene y=-0.31) lands on the stock one (y=-0.60) so the G1-D stance is unchanged; the tabletop keeps its real height
# (0.68 m, build_params.json) and the objects move down onto it.
REAL_SCENE = "real_scene1"
REAL_SCENE_ROOM = {
    "default": REAL_SCENE,
    "default_pos": [0.0, -0.29, 0.0],
    "default_ori": [1.0, 0.0, 0.0, 0.0],
    "scale": [1.0, 1.0, 1.0],
    "collision": False,  # the USD carries its own static colliders (floor box included)
}
# ponytail: lights live in the per-env Room, so N parallel envs get N suns; move them to /World if num_envs > 1 matters.
REAL_SCENE_USD = """#usda 1.0
(
    defaultPrim = "Scene"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Scene"
{
    def "Background" (
        prepend references = @./usd/background.usda@
    )
    {
    }

    def "Table" (
        prepend references = @./usd/table.usda@
    )
    {
    }

    def "Lights" (
        prepend references = @./usd/lights.usda@
    )
    {
    }
}
"""
# Red (adzuki) beans: Water_Opaque.mdl (glossy) with a dark maroon base colour (linear RGB).
RED_BEAN_COLOR = "color(0.15, 0.008, 0.01)"
RED_BEAN_MDL = "$Robodojo_ASSETS/Material/Fluid/RedBean.mdl"
# Beans are PBD solid (granular) particles: 4 mm spheres of 0.06 g, smooth (friction 0.2), no cohesion. The 13 mm
# pitcher pour opening jams 5 mm beans, or 4 mm ones at friction 0.4 (check_pitcher_fluid.py --red-beans).
RED_BEAN_PHYSICS = {
    "granular": True,
    "particle_mass": 6e-5,
    "particle_system": {
        "particle_contact_offset": 0.0024,
        "contact_offset": 0.0024,
        "rest_offset": 0.002,
        "solid_rest_offset": 0.002,
        "fluid_rest_offset": 0.0012,
        "max_velocity": 2.5,
        "smoothing": False,
        "anisotropy": False,
        "isosurface": False,  # render the particles as beans, not a liquid surface
    },
    "particle_material": {
        "friction": 0.2,
        "particle_friction_scale": 1.0,
        "damping": 0.0,
        "adhesion": 0.0,
        "particle_adhesion_scale": 0.0,
        "cohesion": 0.0,
        "viscosity": 0.0,
        "surface_tension": 0.0,
        "drag": 0.0,
        "lift": 0.0,
    },
}
# Half-full pitcher: FluidObject fills the fluid mesh (a 60 x 60 x 70 mm ellipsoid, 130 ml) scaled by visual.scale with
# particles, then scales the particle set again by scale * visual.scale; scale = 1 / visual.scale keeps the spacing.
# 1.2 x 1.2 x 2.7 -> 72 x 72 x 189 mm (513 ml, about half of the 1.1 l jug), standing on the inside bottom.
PITCHER_FILL = (1.2, 1.2, 2.7)
PITCHER_FLUID_Z = -0.017  # fluid centre relative to the pitcher centre
PITCHER_AXIS_X = -0.0137  # body axis in the pitcher frame (metadata contact_circle): the bbox centre is off-axis
OBJECT_SECTIONS = ("Rigid", "Geometry", "Fluid", "Articulation", "Garment", "Clutter", "Dynamic")
# Dex1-1 camera module placement (m, dex1 link frame), fitted by eye to the real gripper: the body block top, and how
# far the module moves forward from the v4 post so it sits over the block.
WRIST_CAM_BODY_TOP = 0.0335
WRIST_CAM_FORWARD = 0.015
D435_INERTIAL = (
    '<inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.01"/>'
    '<inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>'
)


def mesh_files(root):
    return {m.get("filename") for m in root.iter("mesh")}


def fetch(src, rel):
    dst = src / rel
    if not dst.exists():
        dst.parent.mkdir(parents=True, exist_ok=True)
        print(f"fetch {rel}")
        urllib.request.urlretrieve(UNITREE_RAW + rel, dst)
    return dst


def build_urdf(v4, src, out):
    g1d = ET.parse(v4 / V4_URDF).getroot()
    g1 = ET.parse(fetch(src, G1_DEX1_URDF)).getroot()
    g1d.extend(el for el in g1 if el.tag in ("link", "joint") and el.get("name") in ("d435_joint", "d435_link"))

    for joint in g1d.findall("joint"):
        if joint.get("name").endswith("Wheel_Joint"):
            joint.set("type", "fixed")
    for link in g1d.findall("link"):
        if link.find("inertial") is None:  # d435_link is massless in the G1 URDF; PhysX wants mass
            link.append(ET.fromstring(D435_INERTIAL))

    movable = [j.get("name") for j in g1d.findall("joint") if j.get("type") != "fixed"]
    for side in ("left", "right"):
        assert sum(n.startswith(side) and "dex1" not in n for n in movable) == 7, f"{side} arm must have 7 joints"
        assert sum(n.startswith(f"{side}_dex1_finger") for n in movable) == 2, f"{side} Dex1 must have 2 fingers"

    for mesh in mesh_files(g1d):
        (out / mesh).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(v4 / mesh, out / mesh)
    seat_wrist_camera(out / "meshes/dex1_1/base_link.STL")
    recess_carrier_hulls(g1d, out)
    shutil.copy(v4 / "physics_contract.json", out)
    shutil.rmtree(out / "usd", ignore_errors=True)  # UrdfFileCfg hashes only the URDF path, not mesh files
    ET.ElementTree(g1d).write(out / "g1d_dex1.urdf", encoding="utf-8", xml_declaration=True)
    print(f"wrote {out / 'g1d_dex1.urdf'} ({len(movable)} movable joints)")


def seat_wrist_camera(stl):
    """The v4 Dex1-1 mesh holds its camera module up on a ~6 cm post behind the body; on the real gripper the module
    sits on the body. Cut the post off at the body top, drop the cable plug on it, and set the module (tilt kept) on
    the body block. Mesh frame: dex1 link x = mesh y, link z = mesh z."""
    import trimesh  # Isaac Sim env

    parts = trimesh.load(stl, process=True).split(only_watertight=False)
    bracket = max(parts, key=lambda p: p.bounds[1, 2])
    above = [p for p in parts if p.bounds[0, 2] > WRIST_CAM_BODY_TOP]  # camera module + cable plug on the post
    camera = max(above, key=lambda p: p.extents[2])
    body = [p for p in parts if p is not bracket and all(p is not a for a in above)]
    bracket = bracket.slice_plane([0, 0, WRIST_CAM_BODY_TOP], [0, 0, -1], cap=True)  # keep the back plate
    camera.apply_translation([0, WRIST_CAM_FORWARD, WRIST_CAM_BODY_TOP - camera.bounds[0, 2]])
    trimesh.util.concatenate([*body, bracket, camera]).export(stl)


def recess_carrier_hulls(g1d, out):
    """Link{1,2}_2 is L-shaped (slider lug + finger beam); its single convex hull bridges the L and sticks 3.4 mm
    past the frictionless carrier into the grip zone ahead of the Link{1,2}_3 pad. Trim the lug on the collision copy
    so the carrier hull stays 0.5 mm behind the pad face."""
    import numpy as np
    import trimesh

    for idx, clip in ((1, lambda x: np.maximum(x, -0.002)), (2, lambda x: np.minimum(x, 0.002))):
        rel = f"meshes/dex1_1/Link{idx}_2_col.STL"
        mesh = trimesh.load(out / f"meshes/dex1_1/Link{idx}_2.STL", process=False)
        mesh.vertices[:, 0] = clip(mesh.vertices[:, 0])
        mesh.export(out / rel)
        for el in g1d.findall(f".//link/collision/geometry/mesh[@filename='meshes/dex1_1/Link{idx}_2.STL']"):
            el.set("filename", rel)


def shift_objects(layout, offset):
    """Translate every task object (not the room/table fixtures) by `offset`, keeping their relative layout."""
    for section in OBJECT_SECTIONS:
        for entries in layout.get(section, {}).values():
            for entry in entries:
                entry["default_pos"] = [p + d for p, d in zip(entry["default_pos"], offset)]
    return layout


def squeeze(layout, scale):
    """Scale every task object's (x, y) about the stock table centre: X5 layouts span 0.9 x 0.3 m in front of its
    arms, G1-D standing at a 0.68 m table reaches about 0.5 x 0.15 m with its hands down."""
    for section in OBJECT_SECTIONS:
        for entries in layout.get(section, {}).values():
            for entry in entries:
                entry["default_pos"][0] *= scale[0]
                entry["default_pos"][1] *= scale[1]
    return layout


def move_into_real_scene(layout, xy, table_top):
    """Swap the stock room/table/ground/dome for the real scene; move the objects by `xy` and down onto its tabletop."""
    table = layout.pop("Table")
    shift_objects(layout, (*xy, table_top - (table["default_pos"][2] + table["scale"][2] / 2)))
    layout.pop("Ground", None)
    layout.pop("Background", None)
    layout["Room"] = dict(REAL_SCENE_ROOM, table_height=table_top)  # for the layout stability check
    return layout


def swap_bottle_for_pitcher(layout, assets, layout_index, table_top):
    """Replace the wuliangye bottle (and the fluid inside it) by our fridge jug, spout turned toward the cup."""
    index = layout_index % 3  # jug colour variant
    meta = json.loads((assets / "Object/RoboDojo/Rigid/pitcher" / f"{index:05d}/metadata.json").read_text())
    bottle = layout["Rigid"].pop("wuliangye")[0]
    pitcher = dict(bottle, category="pitcher", category_idx=index, physics=dict(meta["physics"], type="rigid"))
    height = meta["physics"]["size"][2]
    x, y, _ = bottle["default_pos"]  # the bottle's origin is not its bbox centre, the pitcher's is
    z = table_top + height / 2 + 1e-4
    pitcher["default_pos"] = [x, y, z]
    flip = x < 0
    pitcher["default_ori"] = [0, 0, 0, 1] if flip else [1, 0, 0, 0]  # spout is -X, handle +X
    layout["Rigid"]["pitcher"] = [pitcher]
    fluid = layout["Fluid"]["wuliangye"][0]
    fluid["zlim"] = [PITCHER_FLUID_Z] * 2
    fluid["default_pos"] = [x - PITCHER_AXIS_X if flip else x + PITCHER_AXIS_X, y, z + PITCHER_FLUID_Z]
    fluid["physics"] = dict(fluid["physics"], **RED_BEAN_PHYSICS)
    fluid["visual"]["visual_mdl_path"] = RED_BEAN_MDL
    fluid["visual"]["scale"] = list(PITCHER_FILL)
    fluid["scale"] = [1 / s for s in PITCHER_FILL]
    return layout


def build_red_bean_material(assets):
    fluid = assets / "Material" / "Fluid"
    mdl = (fluid / "Water_Opaque.mdl").read_text()
    bean = mdl.replace("Water_Opaque(", "RedBean(").replace("color(0.008, 0.02, 0.03)", RED_BEAN_COLOR)
    assert bean.count("RedBean(") == 1 and RED_BEAN_COLOR in bean, "Water_Opaque.mdl changed"
    (fluid / "RedBean.mdl").write_text(bean)


def build_room(scene, assets):
    """Install the real-scene USD package (sim_scene_opus/usd) as Room category REAL_SCENE."""
    room = assets / "Room" / REAL_SCENE
    shutil.copytree(scene, room / "usd", dirs_exist_ok=True)
    (room / f"{REAL_SCENE}.usd").write_text(REAL_SCENE_USD)
    print(f"wrote {room}")


def _obb(assets, kind, cat, entry):
    from scipy.spatial.transform import Rotation

    idx = entry.get("category_idx", 0)
    sub = "Clutter" if entry.get("type") == "cluttered" else kind
    meta = json.loads((assets / "Object/RoboDojo" / sub / cat / f"{idx:05d}/metadata.json").read_text())
    verts = meta["geometry"]["oriented_bbox"]["vertices"]
    scale, pos, q = entry.get("scale", [1, 1, 1]), entry["default_pos"], entry["default_ori"]
    rot = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    axes = rot.as_matrix()
    up = max(range(3), key=lambda a: abs(axes[2, a]))
    ext = [(max(v[a] for v in verts) - min(v[a] for v in verts)) * scale[a] for a in range(3)]
    wverts = [rot.apply([v[a] * scale[a] for a in range(3)]) + pos for v in verts]
    lo = [min(v[a] for v in wverts) for a in range(2)]
    hi = [max(v[a] for v in wverts) for a in range(2)]
    return min(ext[a] for a in range(3) if a != up), ext[up], lo, hi


def _gap_xy(a, b):
    return max(a[2][0] - b[3][0], b[2][0] - a[3][0], a[2][1] - b[3][1], b[2][1] - a[3][1])


def filter_pickup(layout, assets, layout_index, table_top):
    """Drop layouts whose target cannot be grasped by the 90 mm Dex1-1 jaw (narrow horizontal side > 80 mm, or flat
    < 15 mm), and drop clutter items that the 0.6x squeeze pushed into the target or into each other."""
    rigid = layout["Rigid"]
    target_key, target = next((k, e) for k, es in rigid.items() for e in es if e.get("label") == "target")
    info = _obb(assets, "Rigid", target.get("category") or target_key, target)
    if info[0] > 0.08 or info[1] < 0.015:
        return None
    kept = {target_key: [target]}
    boxes = [(True, info)]
    for k, entries in rigid.items():
        if k == target_key:
            continue
        out = []
        for e in entries:
            box = _obb(assets, "Rigid", e.get("category") or k, e)
            if all(_gap_xy(box, b) >= (0.02 if is_target else 0.005) for is_target, b in boxes):
                out.append(e)
                boxes.append((False, box))
        if out:
            kept[k] = out
    layout["Rigid"] = kept
    return layout


# G1-D tasks reuse a benchmark task's eval layouts with the objects moved into G1-D reach (the X5 arms sit on the
# table; G1-D stands behind it, its arms reach about 0.42 m in front of the shoulders at table height):
# task: (benchmark task, (x, y) offset, (x, y) squeeze, post-filter/swap or None)
G1D_TASKS = {
    "g1d_pour_water_into_cup": ("pour_liquid_into_cup", (0.0, -0.20), (1.0, 1.0), swap_bottle_for_pitcher),
    "g1d_general_pickup": ("general_pickup", (0.0, -0.33), (0.6, 0.6), filter_pickup),
    "g1d_stack_blocks": ("stack_blocks", (0.0, -0.30), (0.6, 0.6), None),
}


def _layout_sort_key(path):
    tail = path.stem.rpartition("_")[2]
    return str(path.parent), int(tail) if tail.isdigit() else -1, path.stem


def build_layouts(assets):
    table_top = json.loads((assets / "Room" / REAL_SCENE / "usd" / "build_params.json").read_text())["table"]["top_z"]
    src = assets / "Eval_Layout" / "RoboDojo" / "arx_x5"
    dst = assets / "Eval_Layout" / "RoboDojo" / "g1d"
    for old in dst.rglob("g1d_*_*.json"):
        old.unlink()
    files = sorted(src.rglob("*.json"), key=_layout_sort_key)
    counts = {}
    for f in files:
        layout = json.loads(f.read_text())
        geometry = layout.get("Geometry", {})
        geometry.pop("camera_stand", None)
        if "Geometry" in layout and not geometry:
            del layout["Geometry"]
        out = dst / f.relative_to(src)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(layout, indent=4))
        base, _, index = f.stem.rpartition("_")
        for task, (base_task, xy, scale, swap) in G1D_TASKS.items():
            if base == base_task and index.isdigit():
                shifted = squeeze(json.loads(out.read_text()), scale)  # fresh copy per task
                shifted = move_into_real_scene(shifted, xy, table_top)
                key = (out.parent, task)
                seq = counts.get(key, 0)
                if swap:
                    shifted = swap(shifted, assets, seq, table_top)
                if shifted is not None:
                    (out.parent / f"{task}_{seq}.json").write_text(json.dumps(shifted, indent=4))
                    counts[key] = seq + 1
    print(f"wrote {len(files)} layouts (+{sum(counts.values())} G1-D task layouts) to {dst}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--v4", type=Path, required=True, help="assembled_v4_coupled/ directory")
    parser.add_argument("--src", type=Path, default=ROOT / ".cache" / "unitree_ros")
    parser.add_argument("--assets", type=Path, default=ROOT / "Assets")
    parser.add_argument("--scene", type=Path, help=f"sim_scene_opus/usd/ to install as Room/{REAL_SCENE}")
    args = parser.parse_args()

    if args.scene:
        build_room(args.scene, args.assets)
    out = args.assets / "Robots" / "g1d"
    build_urdf(args.v4, args.src, out)
    for side in ("left", "right"):
        shutil.copy(HERE / f"{side}_robot_config.yml", out)
    build_red_bean_material(args.assets)
    build_layouts(args.assets)


if __name__ == "__main__":
    main()
