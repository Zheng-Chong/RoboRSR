"""Build the Unitree G1-D + Dex1-1 robot asset for RoboDojo.

Takes the team's assembled G1-D + Dex1-1 (camera version) URDF, v4: official g1_d.urdf with the standalone
dex1_1.urdf mounted on each wrist (real2sim/robot/g1d_v4/: g1_d_full_dex1_v4.urdf, meshes/, physics_contract.json),
rebuilds the Dex1-1 wrist camera module at its real mount (scripts/g1d/dex1_camera), adds the G1 head D435 frame from
unitree_ros g1_29dof_mode_15_with_dex1_1.urdf and locks the wheels (tabletop only).
Writes:
    Assets/Robots/g1d/                     g1d_dex1.urdf, meshes/, physics_contract.json (self-collision filters),
                                           {left,right}_robot_config.yml
    Assets/Room/real_scene{1,2,3}/          the real scenes in SCENES (office, counter, lab): <room>.usd + usd/
    Assets/Eval_Layout/RoboDojo/<config>/  per scene config in SCENES, layouts for the G1-D tasks in G1D_TASKS (bottle
                                           swapped for our pitcher of red beans or two cups, set in the real scene);
                                           g1d/ also gets the arx_x5 layouts minus the camera stand (it would sit
                                           inside the robot)
    Assets/Material/Fluid/RedBean.mdl

Usage:
    python scripts/g1d/build_asset.py              # fetch the unitree_ros G1 URDF from GitHub into --src
    python scripts/g1d/build_asset.py --src DIR    # DIR mirrors unitree_ros (robots/...); reused if present
"""

import argparse
import copy
import json
from pathlib import Path
import random
import shutil
import urllib.request
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
UNITREE_RAW = "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/"
V4_URDF = "g1_d_full_dex1_v4.urdf"
G1_DEX1_URDF = "robots/g1_description/g1_29dof_mode_15_with_dex1_1.urdf"
# Real scenes reconstructed from phone video / photos (real2sim/scenes/<dir>/: usd/ and the Room entry <room>.usd), one
# env config each, replacing the stock room, table cube, ground cube and HDR dome of the G1-D task layouts. Each room
# sits with its robot-side table edge on the stock one (y=-0.60); the tabletop keeps its real height
# (usd/build_params.json) and the objects move onto it. A taller table raises the robot's lift column by as much
# (env_cfg/robot/<config>.yml) and the objects move with the robot by `shift`, so its reach is the same everywhere.
SCENES_DIR = ROOT / "real2sim" / "scenes"
SCENES = {  # env config: (scene dir, Room category, Room position, object (x, y) shift)
    "g1d": ("real_scene1_office", "real_scene1", [0.0, -0.29, 0.0], (0.0, 0.0)),
    # floor cabinet: the chassis cannot go under it, so the robot stands 7 cm further back (root y=-0.80)
    "g1d_counter": ("real_scene2_counter", "real_scene2", [-0.35, -0.32, 0.0], (0.0, -0.07)),
    "g1d_lab": ("real_scene3_lab", "real_scene3", [0.0, -0.257, 0.0], (0.0, 0.0)),
}
# ponytail: lights live in the per-env Room, so N parallel envs get N suns; move them to /World if num_envs > 1 matters.
# Red (adzuki) beans: Water_Opaque.mdl (glossy) with a dark maroon base colour (linear RGB).
RED_BEAN_COLOR = "color(0.15, 0.008, 0.01)"
RED_BEAN_MDL = "$Robodojo_ASSETS/Material/Fluid/RedBean.mdl"
# Beans are PBD solid (granular) particles: 5.6 mm spheres of 0.1 g, the volume and mass of a real 7 x 5 x 5 mm bean,
# smooth (friction 0.2), no cohesion. The 13 mm pitcher pour opening jams beans above ~5 mm, so the pitcher task no
# longer pours (4 mm spheres did; check_pitcher_fluid.py --red-beans).
RED_BEAN_PHYSICS = {
    "granular": True,
    "particle_mass": 1e-4,
    "particle_system": {
        "particle_contact_offset": 0.0034,
        "contact_offset": 0.0034,
        "rest_offset": 0.0028,
        "solid_rest_offset": 0.0028,
        "fluid_rest_offset": 0.0017,
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
# The second arm picks the cup and holds it under the spout: it reaches the cup's rim only ~0.40 m in front of the
# robot root (scripts/g1d/scripted_demo.py), so the cup comes this much closer than the jug; layouts where it then
# overlaps the jug are dropped.
POUR_CUP_PULL = -0.09
POUR_MIN_GAP = 0.03  # m between the jug's and the cup's footprints
# Cup-to-cup pour (two plastic cups): each cup within the side grasp's reach of the arm on its side, |x| and y ranges
# (from |x| = 0.178 the right arm cannot lift its cup up to the pour, 0.162 it can); the source cup a sixth full: the
# 60 x 60 x 70 mm fluid ellipsoid scaled to 46 x 46 x 50 mm (~55 ml), its centre this far above the cup centre (inside
# bottom + half its height + 2.8 mm)
CUP_PAIR_X, CUP_PAIR_Y = (0.10, 0.16), (-0.39, -0.31)
CUP_FILL = (0.77, 0.77, 0.71)
CUP_FLUID_Z = -0.0167
# Measuring cups (scripts/objects/make_plastic_cups.py): 500 ml source, 400 ml target, as in the desk photos. Turned
# 45 deg about z so the pads, closing across the 45-deg-inward wrist, meet the square body's flats (72-75 mm) and not
# its corners (88 mm, nearly the 95 mm jaw). Fluid 54 x 54 x 63 mm (~95 ml) 0.8 mm above the 2 mm inside bottom of
# the 158 mm cup.
MEASURING_CUPS = ((1, "source_cup"), (0, "target_cup"))  # measuring_cup index, label
MEASURING_ORI = [0.92388, 0.0, 0.0, 0.38268]
MEASURING_FILL = (0.9, 0.9, 0.9)
MEASURING_FLUID_Z = -0.0427
OBJECT_SECTIONS = ("Rigid", "Geometry", "Fluid", "Articulation", "Garment", "Clutter", "Dynamic")
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
    seat_wrist_camera(g1d, v4, out)
    recess_carrier_hulls(g1d, out)
    shutil.copy(v4 / "physics_contract.json", out)
    shutil.rmtree(out / "usd", ignore_errors=True)  # UrdfFileCfg hashes only the URDF path, not mesh files
    ET.ElementTree(g1d).write(out / "g1d_dex1.urdf", encoding="utf-8", xml_declaration=True)
    print(f"wrote {out / 'g1d_dex1.urdf'} ({len(movable)} movable joints)")


def seat_wrist_camera(g1d, v4, out):
    """Dex1-1 camera version: the v4 base mesh (= official dex1_1_service base_link.STL) holds the camera module up on
    a ~6 cm post; scripts/g1d/dex1_camera rebuilds the module on its upright at the real height, smooths the tail by
    the wrist and adds the mount lip (approximate geometry, *profile.json), as body / support / housing / lip visual
    meshes and convex colliders. Its lens lies on the cam_wrist optical axis, 11 mm ahead of the robot-config sensor,
    inside the camera's 3 cm near plane. The three steps run as delivered (each checks its inputs and the finger
    clearance) in a scratch dir; only the two dex1 base links and their meshes come back."""
    import tempfile

    import dex1_camera
    from dex1_camera.add_mount_lip import add_mount_lip
    from dex1_camera.build import build
    from dex1_camera.geometry import BASE_MESH
    from dex1_camera.smooth_tail import smooth_tail

    profiles = Path(dex1_camera.__file__).parent
    links = ("left_dex1_model_base_link", "right_dex1_model_base_link")
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        (tmp / "base").mkdir()
        (tmp / "base/meshes").symlink_to(out / "meshes")
        ET.ElementTree(g1d).write(tmp / "base/g1d_dex1.urdf", encoding="utf-8", xml_declaration=True)
        build(tmp / "base/g1d_dex1.urdf", v4 / BASE_MESH, profiles / "profile.json", tmp / "camera")
        smooth_tail(tmp / "camera", tmp / "tail", profiles / "tail_profile.json")
        add_mount_lip(tmp / "tail", tmp / "lip", profiles / "mount_lip_profile.json")
        built = ET.parse(tmp / "lip/g1d_dex1.urdf").getroot()
        for name in links:
            link, new = (r.find(f"link[@name='{name}']") for r in (g1d, built))
            link[:] = list(new)
        shutil.rmtree(out / "meshes/camera_geometry", ignore_errors=True)
        shutil.copytree(tmp / "lip/meshes/camera_geometry", out / "meshes/camera_geometry")
    (out / BASE_MESH).unlink()


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


def move_into_real_scene(layout, xy, room, room_pos, table_top):
    """Swap the stock room/table/ground/dome for the real scene; move the objects by `xy` and down onto its tabletop."""
    table = layout.pop("Table")
    shift_objects(layout, (*xy, table_top - (table["default_pos"][2] + table["scale"][2] / 2)))
    layout.pop("Ground", None)
    layout.pop("Background", None)
    layout["Room"] = {
        "default": room,
        "default_pos": room_pos,
        "default_ori": [1.0, 0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0],
        "collision": False,  # the USD carries its own static colliders (floor box included)
        "table_height": table_top,  # for the layout stability check
    }
    return layout


def swap_bottle_for_pitcher(layout, assets, layout_index, table_top):
    """Replace the wuliangye bottle (and the fluid inside it) by our fridge jug, spout turned toward the cup, and the
    cup by our water cup, POUR_CUP_PULL closer to the robot; None if the cup then sits too close to the jug."""
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
    key, mug = next((k, e) for k, es in layout["Rigid"].items() for e in es if e.get("label") == "cup")
    layout["Rigid"][key].remove(mug)
    if not layout["Rigid"][key]:
        del layout["Rigid"][key]
    meta = json.loads((assets / "Object/RoboDojo/Rigid/water_cup/00000/metadata.json").read_text())
    x, y, _ = mug["default_pos"]
    cup = dict(mug, category="water_cup", category_idx=0, physics=dict(meta["physics"], type="rigid"))
    cup["default_pos"] = [x, y + POUR_CUP_PULL, table_top + meta["physics"]["size"][2] / 2 + 1e-4]
    layout["Rigid"]["water_cup"] = [cup]
    if _gap_xy(_obb(assets, "Rigid", "pitcher", pitcher), _obb(assets, "Rigid", cup["category"], cup)) < POUR_MIN_GAP:
        return None
    return layout


def two_cups(layout, assets, layout_index, table_top):
    """Replace the bottle and the mug by two plastic cups: the source (red beans) on the robot's left, the target on its
    right; positions drawn from the bottle's original one, so every seed folder and layout differs (indices repeat).
    The right hand does not pour: the cup turns between the pads (one Dex1 finger sits on its open stop, on the other
    side of the cup than in the left hand) and the beans stay in at 85 deg."""
    meta = json.loads((assets / "Object/RoboDojo/Rigid/plastic_cup/00000/metadata.json").read_text())
    z = table_top + meta["physics"]["size"][2] / 2 + 1e-4
    template = layout["Rigid"].pop("wuliangye")[0]
    rng = random.Random(json.dumps(template["default_pos"]))
    side = -1
    layout["Rigid"] = {}
    cups = []
    for label, sign in (("source_cup", side), ("target_cup", -side)):
        xy = [sign * rng.uniform(*CUP_PAIR_X), rng.uniform(*CUP_PAIR_Y)]
        cups.append(
            dict(
                template,
                category="plastic_cup",
                category_idx=0,
                label=label,
                default_pos=[*xy, z],
                default_ori=[1.0, 0.0, 0.0, 0.0],
                physics=dict(meta["physics"], type="rigid"),
            )
        )
    layout["Rigid"]["plastic_cup"] = cups
    beans = layout["Fluid"]["wuliangye"][0]
    beans["label"], beans["relative_plane"] = "beans", "source_cup"
    beans["zlim"] = [CUP_FLUID_Z] * 2
    beans["default_pos"] = [*cups[0]["default_pos"][:2], z + CUP_FLUID_Z]
    beans["physics"] = dict(beans["physics"], **RED_BEAN_PHYSICS)
    beans["visual"]["visual_mdl_path"] = RED_BEAN_MDL
    beans["visual"]["scale"] = list(CUP_FILL)
    beans["scale"] = [1 / s for s in CUP_FILL]
    return layout


def two_measuring_cups(layout, assets, layout_index, table_top):
    """two_cups with the measuring cups (MEASURING_CUPS): same positions as two_cups draws for this layout."""
    layout = two_cups(layout, assets, layout_index, table_top)
    cups = layout["Rigid"].pop("plastic_cup")
    for cup, (idx, label) in zip(cups, MEASURING_CUPS):
        assert cup["label"] == label
        meta = json.loads((assets / f"Object/RoboDojo/Rigid/measuring_cup/{idx:05d}/metadata.json").read_text())
        cup.update(category="measuring_cup", category_idx=idx, default_ori=MEASURING_ORI)
        cup["physics"] = dict(meta["physics"], type="rigid")
        cup["default_pos"][2] = table_top + meta["physics"]["size"][2] / 2 + 1e-4
    layout["Rigid"]["measuring_cup"] = cups
    beans = layout["Fluid"]["wuliangye"][0]
    beans["zlim"] = [MEASURING_FLUID_Z] * 2
    beans["default_pos"][2] = cups[0]["default_pos"][2] + MEASURING_FLUID_Z
    beans["visual"]["scale"] = list(MEASURING_FILL)
    beans["scale"] = [1 / s for s in MEASURING_FILL]
    return layout


def build_red_bean_material(assets):
    fluid = assets / "Material" / "Fluid"
    mdl = (fluid / "Water_Opaque.mdl").read_text()
    bean = mdl.replace("Water_Opaque(", "RedBean(").replace("color(0.008, 0.02, 0.03)", RED_BEAN_COLOR)
    assert bean.count("RedBean(") == 1 and RED_BEAN_COLOR in bean, "Water_Opaque.mdl changed"
    (fluid / "RedBean.mdl").write_text(bean)


def install_rooms(assets):
    """Install every real scene as its Room category (select_room loads the one .usd at the top of the folder)."""
    for scene, room, _, _ in SCENES.values():
        src, dst = SCENES_DIR / scene, assets / "Room" / room
        shutil.copytree(src / "usd", dst / "usd", dirs_exist_ok=True)
        shutil.copy(src / f"{room}.usd", dst)
        print(f"wrote {dst}")


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
    "g1d_pour_beans_cup_to_cup": ("pour_liquid_into_cup", (0.0, -0.20), (1.0, 1.0), two_cups),
    "g1d_pour_beans_measuring_cups": ("pour_liquid_into_cup", (0.0, -0.20), (1.0, 1.0), two_measuring_cups),
    "g1d_general_pickup": ("general_pickup", (0.0, -0.36), (0.6, 0.6), filter_pickup),
    "g1d_stack_blocks": ("stack_blocks", (0.0, -0.35), (0.6, 0.6), None),
}


def _layout_sort_key(path):
    tail = path.stem.rpartition("_")[2]
    return str(path.parent), int(tail) if tail.isdigit() else -1, path.stem


def build_layouts(assets):
    """Per scene config: the G1-D task layouts; the office one (g1d) also gets every arx_x5 layout."""
    src = assets / "Eval_Layout" / "RoboDojo" / "arx_x5"
    files = sorted(src.rglob("*.json"), key=_layout_sort_key)
    for config, (_, room, room_pos, shift) in SCENES.items():
        table_top = json.loads((assets / "Room" / room / "usd" / "build_params.json").read_text())["table"]["top_z"]
        dst = assets / "Eval_Layout" / "RoboDojo" / config
        for old in dst.rglob("g1d_*_*.json"):
            old.unlink()
        counts = {}
        for f in files:
            layout = json.loads(f.read_text())
            geometry = layout.get("Geometry", {})
            geometry.pop("camera_stand", None)
            if "Geometry" in layout and not geometry:
                del layout["Geometry"]
            out = dst / f.relative_to(src)
            out.parent.mkdir(parents=True, exist_ok=True)
            if config == "g1d":
                out.write_text(json.dumps(layout, indent=4))
            base, _, index = f.stem.rpartition("_")
            for task, (base_task, xy, scale, swap) in G1D_TASKS.items():
                if base == base_task and index.isdigit():
                    shifted = squeeze(copy.deepcopy(layout), scale)
                    shifted = move_into_real_scene(shifted, xy, room, room_pos, table_top)
                    key = (out.parent, task)
                    seq = counts.get(key, 0)
                    if swap:
                        shifted = swap(shifted, assets, seq, table_top)
                    if shifted is not None:
                        shift_objects(shifted, (*shift, 0.0))
                        (out.parent / f"{task}_{seq}.json").write_text(json.dumps(shifted, indent=4))
                        counts[key] = seq + 1
        print(f"wrote {sum(counts.values())} G1-D task layouts to {dst}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--v4", type=Path, default=ROOT / "real2sim" / "robot" / "g1d_v4", help="assembled v4 URDF directory"
    )
    parser.add_argument("--src", type=Path, default=ROOT / ".cache" / "unitree_ros")
    parser.add_argument("--assets", type=Path, default=ROOT / "Assets")
    args = parser.parse_args()

    install_rooms(args.assets)
    out = args.assets / "Robots" / "g1d"
    build_urdf(args.v4, args.src, out)
    for side in ("left", "right"):
        shutil.copy(HERE / f"{side}_robot_config.yml", out)
    build_red_bean_material(args.assets)
    build_layouts(args.assets)


if __name__ == "__main__":
    main()
