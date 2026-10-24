"""Generate layouts of an upstream RoboDojo (dual X5) task with the factors a randomization profile turns on.

The released engine only replays saved layouts (Eval_Layout/RoboDojo/<env cfg>/<seed>/<task>_<i>.json) and has no
generator. This one takes the task's official layouts as templates for the structure (which objects, labels, parents,
fixtures) and redraws, per layout:
- object positions inside each entry's own xlim / ylim and yaws inside its rotate_deg (footprints from metadata
  oriented_bbox, no overlap, inside the tabletop, outside the task's ProhibitedArea); objects resting on another
  object move with it; objects on the ground stay;
- object instances from the base task's config, same category and group, never an index only its _random variant
  uses (those are the benchmark's held-out instances);
- clutter from Object/RoboDojo/Clutter, placed on its stable faces like the benchmark's _random layouts;
- the Room, the tabletop and ground MDL materials, the HDR background and its intensity, and extra lights;
- head-camera orientation jitter, through a generated env config (the engine draws it once per parallel env at
  start-up).
Positions are never copied from the templates, which are the benchmark's eval layouts. The HDR dome is shared by the
whole stage: the engine takes it from the first parallel env's layout.

    python scripts/datagen/x5_layouts.py --task stack_bowls_random --seed 900 --count 100 --assets Assets
    python scripts/datagen/x5_layouts.py --task hang_mugs --seed 950 --count 30 --split val
Writes <assets>/Eval_Layout/RoboDojo/<env cfg>/<seed>/ with split.json (train layouts are refused by the eval client
without --allow_train_layouts) and report.json; with camera jitter also env_cfg/arx_x5_rsi_<task>.yml and
env_cfg/camera/rsi_<task>.yml (both git-ignored).
"""

import argparse
import colorsys
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import random
import re
import sys

import numpy as np
from shapely.geometry import MultiPolygon, Point, box
from shapely.ops import nearest_points
import transforms3d as t3d
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "scripts" / "datagen")]
import rsi_assets  # noqa: E402
import rsi_profile  # noqa: E402

from utils.cluttered_generator import ClutteredGenerator  # noqa: E402

CONFIG_DIR = ROOT / "task" / "RoboDojo" / "config"
OBJECT_SECTIONS = ("Rigid", "Geometry", "Articulation", "Garment", "Fluid", "Dynamic")
MOVABLE = ("Rigid", "Geometry", "Articulation", "Garment")  # sections whose table entries get a new pose
TABLE_INSET = 0.05  # m, as LayoutManager.cluttered_generator_init
CLUTTER_MARGIN, CLUTTER_ROTATE = 0.015, 30  # as the benchmark's _random task configs
LIGHT_BOX = ((-0.6, 0.6), (-0.7, 0.5), (1.6, 2.4))  # m, x / y / z of an extra light above the table
LIGHT_TILT = 20  # deg about x and y; 0 points the light down
LIGHT_SHAPE = {  # type -> {attribute range key: (low, high)}
    "Sphere": {"radius_range": (0.05, 0.2)},
    "Disk": {"radius_range": (0.05, 0.2)},
    "Rect": {"width_range": (0.3, 1.0), "height_range": (0.3, 1.0)},
    "Cylinder": {"radius_range": (0.03, 0.1), "length_range": (0.3, 1.0)},
    "Distant": {"angle_range": (0.5, 3.0)},
}
YAW_TOL = 1e-3
TILT_TOL = 0.1  # quaternion x / y of a settled tilt (about 11 deg) still folded into the base orientation
# base task -> {factor: categories} a profile's color / scale may touch. A colour replaces the object's own textures
# (digits, labels), and a scaled object can stop fitting what the task puts it in (nested bowls, dolls sorted by size,
# a toaster slot, plugs and holes), so everything else stays as the template has it.
RESTYLE = {
    "stack_bowls": {"color": {"bowl"}},
    "stack_blocks": {"color": {"block"}, "scale": {"block"}},
    "hang_mugs": {"color": {"mug"}, "scale": {"mug"}},
    "push_T": {"color": {"t"}},
    "sweep_blocks": {"color": {"small_cube"}, "scale": {"small_cube"}},
    "pour_liquid_into_cup": {"color": {"mug", "goblet"}, "scale": {"mug"}},
    "pack_objects_into_box": {"scale": {"car", "shoe", "hammer", "electric_toothbrush"}},
    "store_laptop_and_headphones": {"scale": {"headset"}},
}
COLOR_HSV = ((0.0, 1.0), (0.45, 0.9), (0.35, 0.9))  # hue, saturation, value of a random colour
SEED_RANGES = {"train": (1000, 4999), "exam": (5000, 9999)}  # layout seed folders per pool, rsi/AGENTS.md section 2


def load_yaml(path):
    return (yaml.safe_load(path.read_text(encoding="utf-8")) or {}) if path.is_file() else {}


def ranges(lim):
    """[a, b] or [[a, b], ...] -> [(a, b), ...]."""
    return [tuple(lim)] if isinstance(lim[0], int | float) else [tuple(r) for r in lim]


def region(xlim, ylim):
    boxes = [box(x0, y0, x1, y1) for x0, x1 in ranges(xlim) for y0, y1 in ranges(ylim)]
    return boxes[0] if len(boxes) == 1 else MultiPolygon(boxes)


def pools(config):
    """{label: [(category, group, indices)]} and {category: indices} of a task config's object blocks."""
    by_label, by_category = {}, {}
    for section in MOVABLE:
        for block in config.get(section) or []:
            cats = [(c["name"], c.get("group"), list(c.get("index") or [])) for c in block.get("category") or []]
            for name, _, indices in cats:
                by_category.setdefault(name, set()).update(indices)
            for label in (block.get("select_mode") or {}).get("label") or []:
                by_label[label] = cats
    return by_label, by_category


def held_out(base_config, variant_config):
    """{category: indices} that only the _random variant uses."""
    base, variant = pools(base_config)[1], pools(variant_config)[1] if variant_config else {}
    return {cat: idx - base.get(cat, set()) for cat, idx in variant.items() if idx - base.get(cat, set())}


class Assets:
    def __init__(self, root):
        self.root = Path(root)
        self.cache = {}

    def meta(self, section, category, index, clutter=False):
        key = ("Clutter" if clutter else section, category, index)
        if key not in self.cache:
            path = self.root / "Object" / "RoboDojo" / key[0] / category / f"{index:05d}" / "metadata.json"
            self.cache[key] = json.loads(path.read_text(encoding="utf-8")) if path.is_file() else None
        return self.cache[key]


def bbox(meta, scale):
    v = np.asarray(meta["geometry"]["oriented_bbox"]["vertices"], dtype=float).reshape(-1, 3)
    return v * np.asarray(scale or [1, 1, 1], dtype=float)


def bottom(meta, scale, quat):
    return float((bbox(meta, scale) @ t3d.quaternions.quat2mat(quat).T)[:, 2].min())


def yaw_base(entry, meta):
    """(base orientation, current yaw deg) with default_ori = Rz(yaw) * base, or None when no candidate fits."""
    src = np.asarray(entry["default_ori"], dtype=float)
    candidates = [entry.get("qpos") or [1, 0, 0, 0]]
    for data in ((meta or {}).get("active") or {}).get("place", {}).values():
        if center := (data.get("projection_circle") or {}).get("center"):
            candidates.append(t3d.quaternions.qinverse(np.asarray(center[3:], dtype=float)))
    for base in candidates:
        base = np.asarray(base, dtype=float)
        rel = t3d.quaternions.qmult(src, t3d.quaternions.qinverse(base))
        if abs(rel[1]) < YAW_TOL and abs(rel[2]) < YAW_TOL:
            return base, math.degrees(2 * math.atan2(rel[3], rel[0]))
    rel = t3d.quaternions.qmult(src, t3d.quaternions.qinverse(np.asarray(candidates[0], dtype=float)))
    if abs(rel[1]) < TILT_TOL and abs(rel[2]) < TILT_TOL:  # settled with a small tilt (a draped garment): fold it in
        twist = np.array([rel[0], 0, 0, rel[3]]) / math.hypot(rel[0], rel[3])
        return t3d.quaternions.qmult(t3d.quaternions.qinverse(twist), src), math.degrees(
            2 * math.atan2(twist[3], twist[0])
        )
    return None


def pose(entry):
    m = np.eye(4)
    m[:3, :3] = t3d.quaternions.quat2mat(entry["default_ori"])
    m[:3, 3] = entry["default_pos"]
    return m


def set_pose(entry, m):
    entry["default_pos"] = [float(v) for v in m[:3, 3]]
    entry["default_ori"] = [float(v) for v in t3d.quaternions.mat2quat(m[:3, :3])]


def entries(layout):
    """(section, category, entry) of every task object (not clutter)."""
    for section in OBJECT_SECTIONS:
        for category, items in (layout.get(section) or {}).items():
            for entry in items:
                if entry.get("type") != "cluttered":
                    yield section, category, entry


def parent_label(entry):
    plane = str(entry.get("relative_plane") or "Table")
    return None if plane in ("Table", "Ground") else plane.split("/")[0]


class Generator:
    def __init__(self, task, profile, assets, templates, tiers=None):
        self.task, self.profile, self.assets = task, profile, Assets(assets)
        base, variant = rsi_profile.holdout_split(task)
        self.base_config = load_yaml(CONFIG_DIR / f"{base}.yml")
        self.variant_config = load_yaml(CONFIG_DIR / f"{variant}.yml") if variant else {}
        self.labels = pools(self.base_config)[0]
        self.holdout = held_out(self.base_config, self.variant_config)
        self.templates = templates
        self.prohibited = [
            area for config in (self.base_config, self.variant_config) for area in config.get("ProhibitedArea") or []
        ]
        clutter_cfg = (self.variant_config.get("Clutter") or self.base_config.get("Clutter") or [None])[0]
        self.clutter_region = region(clutter_cfg["xlim"], clutter_cfg["ylim"]) if clutter_cfg else None
        self.opts = rsi_profile.options(assets, profile.get("pool", "train"), tiers)
        self.reserved = set(self.opts["reserved"])
        self.extra = {}  # (section, category) -> the pool's new instances
        for asset_id in self.opts["instances"]:
            section, _, name = asset_id.partition(":")
            category, index = name.split("/")
            self.extra.setdefault((section, category), []).append(int(index))

    def layout(self, index, seed, tries=20):
        """One layout and its record of what was drawn: the first draw in which every object found a free spot, else
        the draw that kept the fewest template poses."""
        best = None
        for attempt in range(tries):
            layout, record = self.draw(index, f"{seed}/{self.task}/{index}/{attempt}")
            record["attempt"] = attempt
            if best is None or len(record["kept_pose"]) < len(best[1]["kept_pose"]):
                best = layout, record
            if not record["kept_pose"]:
                break
        return best

    def draw(self, index, key):
        rng = random.Random(key)
        np.random.seed(rng.randrange(2**32))
        name, template = self.templates[index % len(self.templates)]
        layout, record = deepcopy(template), {"template": name, "kept_yaw": [], "kept_pose": []}
        for section in OBJECT_SECTIONS:  # a template's own clutter never carries over
            for category in list(layout.get(section) or {}):
                kept = [e for e in layout[section][category] if e.get("type") != "cluttered"]
                if kept:
                    layout[section][category] = kept
                else:
                    del layout[section][category]
        p = self.profile
        if p.get("instances"):
            record["instances"] = self.swap_instances(layout, rng)
        if p.get("color") or p.get("scale"):
            record["restyled"] = self.restyle(layout, rng)
        table = layout.get("Table")
        gen = self.table_generator(table) if table else None
        if gen and (p.get("position") or p.get("yaw")):
            self.place_objects(layout, gen, record)
        if gen and (c := p.get("clutter")):
            record["clutter"] = self.place_clutter(layout, gen, rng, c)
        self.scene(layout, rng, record)  # no record in the layout: SceneManager spawns every unknown top-level key
        return layout, record

    def swap_instances(self, layout, rng):
        swapped, used = [], {}
        for section, category, entry in entries(layout):
            if section not in MOVABLE:
                continue
            group = entry.get("group")
            pool = [
                i
                for cat, grp, indices in self.labels.get(entry.get("label"), [])
                if cat == category and grp == group
                for i in indices + (self.extra.get((section, category), []) if group is None else [])
                if i not in self.holdout.get(category, set()) and f"{section}:{category}/{i}" not in self.reserved
            ]
            taken = used.setdefault((category, group), set())
            choices = [i for i in pool if i not in taken]
            if not choices:
                continue
            old, new = entry["category_idx"], rng.choice(choices)
            taken.add(new)
            if new == old:
                continue
            old_meta, new_meta = self.assets.meta(section, category, old), self.assets.meta(section, category, new)
            if not (old_meta and new_meta):
                continue
            shift = bottom(old_meta, entry.get("scale"), entry["default_ori"])
            shift -= bottom(new_meta, entry.get("scale"), entry["default_ori"])
            entry["category_idx"] = new
            entry["default_pos"][2] += shift
            entry["physics"] = {**entry.get("physics", {}), **(new_meta.get("physics") or {})}
            swapped.append(f"{category} {old}->{new}")
        return swapped

    def restyle(self, layout, rng):
        """Random colour and uniform scale of the task objects RESTYLE allows; an object with something resting on it,
        or resting on another object, keeps its size."""
        allow, p, done = RESTYLE.get(rsi_profile.holdout_split(self.task)[0], {}), self.profile, []
        parents = {parent_label(e) for _, _, e in entries(layout)} - {None}
        for section, category, entry in entries(layout):
            if section != "Rigid":
                continue
            meta = self.assets.meta(section, category, entry["category_idx"])
            if p.get("scale") and category in allow.get("scale", ()) and meta and entry.get("label") not in parents:
                if not parent_label(entry):
                    s, old = rng.uniform(*p["scale"]), entry.get("scale") or [1.0, 1.0, 1.0]
                    new = [float(v * s) for v in old]
                    entry["default_pos"][2] += bottom(meta, old, entry["default_ori"]) - bottom(
                        meta, new, entry["default_ori"]
                    )
                    entry["scale"] = new
                    if mass := (entry.get("physics") or {}).get("mass") or (meta.get("physics") or {}).get("mass"):
                        entry["physics"] = {**entry.get("physics", {}), "mass": float(mass * s**3)}
                    done.append(f"{category} x{s:.2f}")
            if p.get("color") and category in allow.get("color", ()):
                rgb = colorsys.hsv_to_rgb(*(rng.uniform(*r) for r in COLOR_HSV))
                linear = [round((c / 12.92) if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4, 4) for c in rgb]
                visual = {**entry.get("visual", {}), "color": linear}
                if parts := ((meta or {}).get("visual") or {}).get("color_parts"):
                    visual["color_parts"] = parts
                entry["visual"] = visual
                done.append(f"{category} colour")
        return done

    def table_generator(self, table):
        sx, sy, sz = table["scale"]
        x, y, z = table["default_pos"]
        container = box(
            x - sx / 2 + TABLE_INSET, y - sy / 2 + TABLE_INSET, x + sx / 2 - TABLE_INSET, y + sy / 2 - TABLE_INSET
        )
        gen = ClutteredGenerator(
            global_container=container, frame=np.array([0, 0, z + sz / 2, 1, 0, 0, 0], dtype=float)
        )
        for a, b, c, d in self.prohibited:
            gen.add_box_prohibited_area(min(a, c), min(b, d), max(a, c), max(b, d))
        return gen

    def footprint(self, section, category, entry):
        """Object-frame bbox vertices: metadata oriented_bbox, else a box of the layout's physics size."""
        meta = self.assets.meta(section, category, entry["category_idx"])
        if meta and "oriented_bbox" in (meta.get("geometry") or {}):
            return meta, bbox(meta, entry.get("scale"))
        size = (entry.get("physics") or {}).get("size")
        if not size:
            return meta, None
        half = np.asarray(size, dtype=float) / 2
        return meta, np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half

    def place_objects(self, layout, gen, record):
        p, top = self.profile, gen.frame[2]
        on_table = [
            (s, c, e)
            for s, c, e in entries(layout)
            if not parent_label(e) and str(e.get("relative_plane") or "Table") == "Table"
        ]

        def slots(e):  # discrete positions: xlim / ylim lists of single values, e.g. [[-0.2, -0.2], [0, 0]]
            if e.get("xlim") is None or not all(lo == hi for lo, hi in ranges(e["xlim"]) + ranges(e["ylim"])):
                return None
            spots = [(x, y) for x, _ in ranges(e["xlim"]) for y, _ in ranges(e["ylim"])]
            return spots if len(spots) > 1 else None

        def movable(item):
            s, _, e = item
            if s not in MOVABLE or e.get("xlim") is None or slots(e):
                return False
            fixed_xy = all(lo == hi for lo, hi in ranges(e["xlim"]) + ranges(e["ylim"]))
            return (p.get("position") and not fixed_xy) or (p.get("yaw") and e.get("rotate_rand"))

        moved = {}  # id(entry) -> (old pose, new pose)
        if p.get("position"):  # objects sharing a slot list swap places
            groups = {}
            for s, _, e in on_table:
                if s in MOVABLE and (spots := slots(e)):
                    groups.setdefault(json.dumps(spots), (spots, []))[1].append(e)
            for spots, members in groups.values():
                if len(members) > len(spots):
                    continue
                new = random.Random(np.random.randint(2**31)).sample(spots, len(members))
                for e, (x, y) in zip(members, new):
                    old_spot = min(
                        spots, key=lambda q: (q[0] - e["default_pos"][0]) ** 2 + (q[1] - e["default_pos"][1]) ** 2
                    )
                    old = pose(e)
                    e["default_pos"] = [
                        e["default_pos"][0] + x - old_spot[0],
                        e["default_pos"][1] + y - old_spot[1],
                        e["default_pos"][2],
                    ]
                    moved[id(e)] = (old, pose(e))
        for item in sorted(on_table, key=movable):  # what stays is placed first, so the rest avoids it
            section, category, entry = item
            meta, verts = self.footprint(section, category, entry)
            if verts is None:
                continue
            z = entry["default_pos"][2] - top
            here = Point(*entry["default_pos"][:2])
            if not movable(item):
                gen.add_model_with_fixed_pose(verts, z, entry["default_ori"], allowed_region=here, check_mode="enforce")
                continue
            fixed_xy = all(lo == hi for lo, hi in ranges(entry["xlim"]) + ranges(entry["ylim"]))
            allowed = here if fixed_xy or not p.get("position") else region(entry["xlim"], entry["ylim"])
            # the ranges bound a placement point that can sit off the origin (a garment, an object on a place face):
            # sample that anchor, kept at the template's offset in the object frame
            anchor = np.zeros(3)
            if allowed is not here and not allowed.contains(here):
                near = nearest_points(allowed, here)[0]
                rot = t3d.quaternions.quat2mat(entry["default_ori"])
                anchor = rot.T @ np.array([near.x - here.x, near.y - here.y, 0.0])
                verts = verts - anchor
            base = yaw_base(entry, meta) if p.get("yaw") and entry.get("rotate_rand") else None
            if p.get("yaw") and entry.get("rotate_rand") and base is None:
                record["kept_yaw"].append(entry.get("label") or category)
            for margin in dict.fromkeys((entry.get("margin") or 0.0, 0.0)):  # the entry's margin, then none
                ok, world, _ = gen.add_model_with_fixed_pose(
                    origin_bbox_points=verts,
                    z=z,
                    qpos=base[0] if base else np.asarray(entry["default_ori"], dtype=float),
                    rotate_rand=base is not None,
                    rotate_deg=entry.get("rotate_deg"),
                    margin=margin,
                    max_attempts=200,
                    allowed_region=allowed,
                    check_mode=entry.get("check_mode") or "bbox",
                )
                if ok:
                    break
            if not ok:  # keep the template pose, still blocking the others
                record["kept_pose"].append(entry.get("label") or category)
                gen.add_model_with_fixed_pose(verts, z, entry["default_ori"], allowed_region=here, check_mode="enforce")
                continue
            old = pose(entry)
            origin = world[:3] - t3d.quaternions.quat2mat(world[3:]) @ anchor
            entry["default_pos"], entry["default_ori"] = [float(v) for v in origin], [float(v) for v in world[3:]]
            moved[id(entry)] = (old, pose(entry))
        by_label = {e["label"]: e for _, _, e in entries(layout) if e.get("label")}
        children = [e for _, _, e in entries(layout) if parent_label(e)]
        while todo := [e for e in children if id(e) not in moved and id(by_label.get(parent_label(e))) in moved]:
            for entry in todo:  # children follow their parent, grandchildren theirs
                old_parent, new_parent = moved[id(by_label[parent_label(entry)])]
                old = pose(entry)
                set_pose(entry, new_parent @ np.linalg.inv(old_parent) @ old)
                moved[id(entry)] = (old, pose(entry))

    def place_clutter(self, layout, gen, rng, cfg):
        """Distractors from the pool (rsi_assets: unused clutter and unused objects of other categories), one per
        category, never a category the task itself uses."""
        wanted = rng.randint(*cfg["count"])
        used = {c for _, c, _ in entries(layout)}
        keep = set(cfg.get("categories") or self.opts["clutter"])
        by_category = {}
        for asset_id in self.opts["distractors"]:
            kind, _, name = asset_id.partition(":")
            category, index = name.split("/")
            if category in keep and category not in used:
                by_category.setdefault(category, []).append((kind, int(index)))
        categories = sorted(by_category)
        rng.shuffle(categories)
        placed = []
        for category in categories:
            if len(placed) >= wanted:
                break
            kind, index = rng.choice(by_category[category])
            meta = self.assets.meta(kind, category, index, clutter=kind == "clutter")
            if not meta:
                continue
            ok, world, _ = gen.add_model_from_config(
                meta,
                rotate_rand=True,
                rotate_deg=CLUTTER_ROTATE,
                margin=CLUTTER_MARGIN,
                max_attempts=50,
                allowed_region=self.clutter_region,
                cluttered=True,
            )
            if not ok:
                continue
            physics = {
                "type": "rigid",
                **{k: v for k, v in (meta.get("physics") or {}).items() if k in ("mass", "friction")},
            }
            entry = {
                "category_idx": index,
                "physics": physics,
                "scale": [1.0, 1.0, 1.0],
                "relative_plane": "Table",
                "default_pos": [float(v) for v in world[:3]],
                "default_ori": [float(v) for v in world[3:]],
            }
            if kind == "clutter":  # LayoutManager loads a cluttered entry from Object/RoboDojo/Clutter
                entry |= {"type": "cluttered", "clutter_idx": len(placed), "yaml_path": "Clutter/clutter.yml"}
            else:  # an object of another category, loaded from its own section; no label, so no task check sees it
                entry |= {"category": category, "group": None}
            layout.setdefault(kind if kind != "clutter" else "Rigid", {}).setdefault(category, []).append(entry)
            placed.append(f"{kind}:{category}/{index}")
        return placed

    def scene(self, layout, rng, record):
        p = self.profile
        if (rooms := p.get("rooms")) and "Room" in layout:
            layout["Room"]["default"] = record["room"] = rng.choice(rooms)
        if (tables := p.get("tables")) and "Table" in layout:
            layout["Table"]["default"] = record["table"] = rng.choice(tables)
        if (grounds := p.get("grounds")) and "Ground" in layout:
            layout["Ground"].setdefault("materials", {"type": "mdl"})["default"] = record["ground"] = rng.choice(
                grounds
            )
        if bg := p.get("background"):
            back = layout.setdefault("Background", {"intensity": 1000})
            if bg.get("hdrs"):
                back["category_name"] = record["hdr"] = rng.choice(bg["hdrs"])
            if bg.get("intensity"):
                back["intensity"] = record["hdr_intensity"] = round(rng.uniform(*bg["intensity"]), 1)
        if lights := p.get("lights"):
            layout["Light"], record["lights"] = {}, []
            for n in range(rng.randint(*lights["count"])):
                kind = rng.choice(lights["types"])
                pos = [round(rng.uniform(*r), 3) for r in LIGHT_BOX]
                ori = [round(rng.uniform(-LIGHT_TILT, LIGHT_TILT), 1) for _ in range(2)] + [0.0]
                common = {"initial_pos_range": pos + pos, "initial_ori_range": ori + ori}
                if temp := lights.get("color_temperature"):
                    t = round(rng.uniform(*temp))
                    common["color_temperature_range"] = [t, t]
                scale = rng.uniform(*(lights.get("intensity") or [1, 1]))
                shape = {k: [v := round(rng.uniform(*r), 3), v] for k, r in LIGHT_SHAPE[kind].items()}
                intensity = round(rsi_profile.LIGHT_REF[kind] * scale, 1)
                layout["Light"][f"light_{n}"] = {
                    "types": kind,
                    "common": common,
                    kind: {"intensity_range": [intensity, intensity], **shape},
                }
                record["lights"].append({"type": kind, "pos": pos, "intensity": intensity})


def camera_configs(task, camera):
    """Write the env config and camera config that jitter the head camera; returns the env config name."""
    name = f"arx_x5_rsi_{task}"
    cam = yaml.safe_load((ROOT / "env_cfg" / "camera" / "camera_config.yml").read_text(encoding="utf-8"))
    o = camera["ori_deg"]
    cam["random"] = {"cam_head": {"enabled": True, "ori": {"min": [-o] * 3, "max": [o] * 3}}}
    (ROOT / "env_cfg" / "camera" / f"rsi_{task}.yml").write_text(yaml.safe_dump(cam, sort_keys=False), encoding="utf-8")
    env = yaml.safe_load((ROOT / "env_cfg" / "arx_x5.yml").read_text(encoding="utf-8"))
    env["config_name"], env["config"]["camera"] = name, f"rsi_{task}"
    (ROOT / "env_cfg" / f"{name}.yml").write_text(yaml.safe_dump(env, sort_keys=False), encoding="utf-8")
    return name


def templates_of(assets, task, env_cfg="arx_x5", seed="0"):
    base, variant = rsi_profile.holdout_split(task)
    folder = Path(assets) / "Eval_Layout" / "RoboDojo" / env_cfg / seed
    if variant and task == variant:
        base_labels, variant_labels = (set(pools(load_yaml(CONFIG_DIR / f"{t}.yml"))[0]) for t in (base, variant))
        if base_labels != variant_labels:
            raise SystemExit(f"{task} 的物体标签和 {base} 不同，训练侧只有 {base} 的实例；要先补训练侧资产")
    source = base if (folder / f"{base}_0.json").is_file() else task
    found = [f for f in folder.glob(f"{source}_*.json") if re.fullmatch(rf"{re.escape(source)}_\d+", f.stem)]
    found.sort(key=lambda f: int(f.stem.rsplit("_", 1)[-1]))
    if not found:
        raise SystemExit(f"{folder} 里没有 {source} 的布局，没有模板可用")
    return [(f.name, json.loads(f.read_text(encoding="utf-8"))) for f in found]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--task", required=True)
    ap.add_argument("--seed", required=True, type=int, help="output seed folder; never 0 (the benchmark's eval set)")
    ap.add_argument("--count", type=int, default=100)
    ap.add_argument("--assets", type=Path, default=ROOT / "Assets")
    ap.add_argument("--profile", type=Path, help="default real2sim/benchmark_tasks/<task>/randomization.json")
    ap.add_argument("--split", choices=("train", "val"), default="train")
    args = ap.parse_args(argv)
    if args.seed == 0:
        ap.error("seed 0 is the benchmark's eval set")
    path = args.profile or rsi_profile.PROFILES_DIR / args.task / "randomization.json"
    profile = json.loads(path.read_text(encoding="utf-8"))
    pool = profile.get("pool", "train")
    if error := rsi_profile.check(profile, rsi_profile.options(args.assets, pool)):
        raise SystemExit(f"{path}: {error}")
    low, high = SEED_RANGES[pool]
    if not low <= args.seed <= high:
        ap.error(f"a {pool} profile writes seeds {low}-{high} (rsi/AGENTS.md, section 2)")
    allowed = rsi_assets.allowed(rsi_assets.load(), pool)
    gen = Generator(args.task, profile, args.assets, templates_of(args.assets, args.task))
    env_cfg = camera_configs(args.task, profile["camera"]) if profile.get("camera") else "arx_x5"
    out = args.assets / "Eval_Layout" / "RoboDojo" / env_cfg / str(args.seed)
    if any(out.glob(f"{args.task}_*.json")):
        raise SystemExit(f"{out} already has {args.task} layouts")
    split_path = out / "split.json"
    previous = json.loads(split_path.read_text(encoding="utf-8")) if split_path.is_file() else {}
    if previous and previous.get("split") != args.split:
        raise SystemExit(f"{out} is already a {previous['split']} folder")
    out.mkdir(parents=True, exist_ok=True)
    records, dropped, draw = {}, 0, 0
    while len(records) < args.count and draw < 3 * args.count:  # a draw that left an object on its template pose
        layout, record = gen.layout(draw, args.seed)  # overlaps it: drop it
        draw += 1
        if record["kept_pose"]:
            dropped += 1
            continue
        if banned := sorted(rsi_assets.layout_assets(layout)[0] - allowed):
            raise SystemExit(f"layout {draw - 1} uses reserved or other-pool assets: {', '.join(banned)}")
        name = f"{args.task}_{len(records)}.json"
        (out / name).write_text(json.dumps(layout, ensure_ascii=False), encoding="utf-8")
        records[name] = record
    if len(records) < args.count:
        print(f"only {len(records)} of {args.count} layouts: the rest kept a template pose in every try")
    digest = hashlib.sha256(json.dumps(profile, sort_keys=True).encode()).hexdigest()[:12]
    split = {"split": args.split, "generator": "scripts/datagen/x5_layouts.py"}
    tasks = {**previous.get("tasks", {}), args.task: {"count": len(records), "profile_sha": digest}}
    split_path.write_text(
        json.dumps({**previous, **split, "tasks": tasks}, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    kept_yaw = sum(bool(r["kept_yaw"]) for r in records.values())
    report = {
        "env_cfg": env_cfg,
        "profile": profile,
        "count": len(records),
        "dropped": dropped,
        "layouts_with_kept_yaw": kept_yaw,
        "layouts": records,
    }
    (out / f"report_{args.task}.json").write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"{len(records)} layouts -> {out} (env cfg {env_cfg}); dropped {dropped}; {kept_yaw} kept some yaw")


if __name__ == "__main__":
    main()
