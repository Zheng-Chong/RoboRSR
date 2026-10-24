"""x5_layouts.py on a tiny synthetic asset tree and the real stack_bowls / stack_bowls_random configs (no Isaac)."""

import json
from pathlib import Path
import sys
import tempfile

import numpy as np
from shapely.geometry import MultiPoint
import transforms3d as t3d

sys.path.insert(0, str(Path(__file__).resolve().parent))
import x5_layouts as X  # noqa: E402

CUBE = [[x, y, z] for x in (-0.05, 0.05) for y in (-0.05, 0.05) for z in (-0.03, 0.03)]


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def assets():
    root = Path(tempfile.mkdtemp())
    for i in range(1, 16):  # bowls 1-10 train stack_bowls, 11-15 only stack_bowls_random uses
        size = 1 + i / 100  # a different height per instance, so a swap moves z
        meta = {"geometry": {"oriented_bbox": {"vertices": (np.array(CUBE) * size).tolist()}}, "physics": {"mass": i}}
        write(root / f"Object/RoboDojo/Rigid/bowl/{i:05d}/metadata.json", meta)
    meta = {"geometry": {"oriented_bbox": {"vertices": (np.array(CUBE) * 1.2).tolist()}}, "physics": {"mass": 16}}
    write(root / "Object/RoboDojo/Rigid/bowl/00016/metadata.json", meta)  # a bowl no official layout uses
    place = {"active": {"place": {"base": {"projection_circle": {"center": [0, 0, -0.03, 1, 0, 0, 0]}}}}}
    for section, cat in (("Clutter", "pen"), ("Clutter", "cup"), ("Clutter", "book"), ("Rigid", "cactus")):
        write(
            root / f"Object/RoboDojo/{section}/{cat}/00000/metadata.json",
            place | {"geometry": {"oriented_bbox": {"vertices": CUBE}}},
        )
    for d in ("Simple_Room_nolight", "Simple_Room_red", "Simple_Room_teal"):
        (root / "Room" / d).mkdir(parents=True)
    for d in ("material_0122", "material_0564", "material_0001", "material_0002", "material_0003"):
        (root / "Material" / d).mkdir(parents=True)
    (root / "Background").mkdir()
    for h in ("brown_photostudio_02_4k.hdr", "studio.hdr"):
        (root / "Background" / h).write_text("")
    return root


# what rsi_assets.scan would give for that tree: bowls 1-10 and the base scene are seen, 11-15 and the teal room are
# the benchmark's, the rest splits into a train and an exam pool
POOLS = {
    "seen": [
        "room:Simple_Room_nolight",
        "material:material_0122",
        "material:material_0564",
        "hdr:brown_photostudio_02_4k.hdr",
        *(f"Rigid:bowl/{i}" for i in range(1, 11)),
    ],
    "t0": ["room:Simple_Room_teal", "clutter:book/0", *(f"Rigid:bowl/{i}" for i in range(11, 16))],
    "pools": {
        "train": {
            "rooms": ["Simple_Room_red"],
            "materials": ["material_0001", "material_0002"],
            "hdrs": ["studio.hdr"],
            "distractors": ["clutter:pen/0", "Rigid:cactus/0"],
            "instances": ["Rigid:bowl/16"],
        },
        "exam": {
            "rooms": [],
            "materials": ["material_0003"],
            "hdrs": [],
            "distractors": ["clutter:cup/0"],
            "instances": [],
        },
    },
}


def template(index):
    bowls = [
        {
            "category": "bowl",
            "category_idx": idx,
            "group": None,
            "xlim": [-0.45, 0.45],
            "ylim": [-0.25, 0.07],
            "qpos": [1, 0, 0, 0],
            "rotate_deg": 0,
            "rotate_rand": False,
            "relative_plane": "Table",
            "margin": 0.01,
            "check_mode": "bbox",
            "label": f"bowl{k}",
            "default_pos": [-0.3 + 0.3 * k, -0.1, 0.765 + 0.03 * (1 + idx / 100)],
            "default_ori": [1, 0, 0, 0],
            "scale": [1, 1, 1],
            "physics": {"type": "rigid"},
        }
        for k, idx in enumerate((11, 12, 13) if index % 2 else (1, 2, 3))
    ]
    return {
        "Rigid": {"bowl": bowls},
        "Room": {"default": "Simple_Room_nolight"},
        "Table": {"default": "material_0122", "scale": [1.4, 1.1, 0.05], "default_pos": [0, -0.05, 0.74]},
        "Ground": {"materials": {"type": "mdl", "default": "material_0564"}},
        "Background": {"intensity": 1000, "category_name": "brown_photostudio_02_4k.hdr"},
    }


PROFILE = {
    "position": True,
    "yaw": True,
    "instances": True,
    "color": True,
    "scale": [0.8, 1.2],  # stack_bowls allows colour only (RESTYLE): the bowls keep their size
    "rooms": ["Simple_Room_red", "Simple_Room_nolight"],
    "tables": ["material_0001"],
    "grounds": ["material_0002"],
    "background": {"hdrs": ["studio.hdr"], "intensity": [600, 1400]},
    "lights": {
        "count": [1, 2],
        "types": ["Sphere", "Rect"],
        "intensity": [0.5, 1.5],
        "color_temperature": [3500, 7500],
    },
    "clutter": {"count": [2, 3], "categories": []},
}


def test_profile_check():
    root = assets()
    opts = X.rsi_profile.options(root, "train", POOLS)
    assert opts["clutter"] == ["cactus", "pen"] and opts["rooms"] == ["Simple_Room_nolight", "Simple_Room_red"]
    assert opts["hdrs"] == ["brown_photostudio_02_4k.hdr", "studio.hdr"] and opts["instances"] == ["Rigid:bowl/16"]
    assert opts["materials"] == ["material_0001", "material_0002", "material_0122", "material_0564"]
    assert X.rsi_profile.check(PROFILE, opts) is None
    assert "不能用" in X.rsi_profile.check({"tables": ["material_9999"]}, opts)
    assert "不能用" in X.rsi_profile.check({"rooms": ["Simple_Room_teal"]}, opts)  # the benchmark's room
    assert "不能用" in X.rsi_profile.check({"tables": ["material_0003"]}, opts)  # the exam pool's material
    assert "干扰物池" in X.rsi_profile.check({"clutter": {"count": [1, 2], "categories": ["cup"]}}, opts)
    assert "pool" in X.rsi_profile.check({"pool": "exam"}, opts)
    assert "scale" in X.rsi_profile.check({"scale": [0.2, 1.0]}, opts)
    assert X.rsi_profile.options(root, "exam", POOLS)["clutter"] == ["cup"]
    assert "灯光要写数量" in X.rsi_profile.check({"lights": {"types": ["Sphere"]}}, opts)
    assert "出图全黑" in X.rsi_profile.check({"camera": {"pos_m": 0.02, "ori_deg": 1}}, opts)
    assert "0 ≤ 角度 ≤ 15°" in X.rsi_profile.check({"camera": {"ori_deg": 30}}, opts)


def footprint(entry, meta):
    v = X.bbox(meta, entry.get("scale"))
    w = v @ t3d.quaternions.quat2mat(entry["default_ori"]).T + np.array(entry["default_pos"])
    return MultiPoint(w[:, :2]).convex_hull


def test_layouts():
    root = assets()
    gen = X.Generator("stack_bowls_random", PROFILE, root, [(f"t{i}.json", template(i)) for i in range(2)], POOLS)
    seen, drawn = set(), set()
    ok = X.rsi_assets.allowed(POOLS, "train")
    for i in range(12):
        layout, record = gen.layout(i, 900)
        assert not record["kept_pose"]
        assert X.rsi_assets.layout_assets(layout)[0] <= ok  # rule 1: nothing reserved, nothing of the exam pool
        bowls = layout["Rigid"]["bowl"]
        idx = [b["category_idx"] for b in bowls]
        drawn |= set(idx)
        assert len(set(idx)) == 3 and set(idx) <= {1, 2, 3, 5, 7, 8, 9, 10, 16}  # never the held-out 11-15
        assert all(b["scale"] == [1, 1, 1] and len(b["visual"]["color"]) == 3 for b in bowls)
        clutter = [("Clutter", c, e) for c, es in layout["Rigid"].items() for e in es if e.get("type") == "cluttered"]
        clutter += [("Rigid", "cactus", e) for e in layout["Rigid"].get("cactus", [])]
        assert len(clutter) == 2 and {c for _, c, _ in clutter} == {"pen", "cactus"}
        assert all("label" not in e for _, _, e in clutter)  # no task check can pick a distractor
        polys = [footprint(b, gen.assets.meta("Rigid", "bowl", b["category_idx"])) for b in bowls]
        polys += [footprint(e, gen.assets.meta(s, c, 0, clutter=s == "Clutter")) for s, c, e in clutter]
        assert all(polys[a].intersection(polys[b]).area < 1e-9 for a in range(len(polys)) for b in range(a))
        for b in bowls:
            assert -0.45 <= b["default_pos"][0] <= 0.45 and -0.25 <= b["default_pos"][1] <= 0.07
            low = X.bottom(gen.assets.meta("Rigid", "bowl", b["category_idx"]), b["scale"], b["default_ori"])
            assert abs(b["default_pos"][2] + low - 0.765) < 1e-6  # the new instance rests on the tabletop
            seen.add(round(b["default_pos"][0], 3))
        assert layout["Room"]["default"] in PROFILE["rooms"] and layout["Table"]["default"] == "material_0001"
        assert layout["Ground"]["materials"]["default"] == "material_0002"
        assert (
            layout["Background"]["category_name"] == "studio.hdr" and 600 <= layout["Background"]["intensity"] <= 1400
        )
        for light in layout["Light"].values():
            pos = light["common"]["initial_pos_range"]
            assert light["types"] in ("Sphere", "Rect") and pos[:3] == pos[3:] and 1.6 <= pos[2] <= 2.4
        assert "rsi" not in layout  # SceneManager would spawn an unknown top-level key as objects
    assert len(seen) > 20  # positions are redrawn, not copied from the templates
    assert 16 in drawn  # the pool's new bowl joins the base task's instances
    again, _ = gen.layout(3, 900)
    assert again == gen.layout(3, 900)[0]  # reproducible from the seed and the index


def test_restyle_scale():
    root = assets()
    real = X.RESTYLE["stack_bowls"]
    X.RESTYLE["stack_bowls"] = {"scale": {"bowl"}}
    try:
        gen = X.Generator("stack_bowls", {"scale": [0.8, 1.2]}, root, [("t.json", template(0))], POOLS)
        layout, record = gen.layout(0, 1000)
    finally:
        X.RESTYLE["stack_bowls"] = real
    for b in layout["Rigid"]["bowl"]:
        s = b["scale"][0]
        meta = gen.assets.meta("Rigid", "bowl", b["category_idx"])
        assert 0.8 <= s <= 1.2 and b["scale"] == [s, s, s] and "visual" not in b
        assert abs(b["default_pos"][2] + X.bottom(meta, b["scale"], b["default_ori"]) - 0.765) < 1e-6  # on the table
        assert abs(b["physics"]["mass"] - meta["physics"]["mass"] * s**3) < 1e-9
    assert len(record["restyled"]) == 3


def test_slots_and_children():
    def entry(label, x):
        return {
            "category": "bowl",
            "category_idx": 1,
            "xlim": [[-0.2, -0.2], [0, 0], [0.2, 0.2]],
            "ylim": [-0.1, -0.1],
            "rotate_rand": False,
            "relative_plane": "Table",
            "label": label,
            "default_pos": [x, -0.1, 0.795],
            "default_ori": [1, 0, 0, 0],
            "scale": [1, 1, 1],
        }

    layout = template(0)
    layout["Rigid"]["bowl"] = [entry("a", -0.2), entry("b", 0.0), entry("c", 0.2)]
    child = {
        "category": "bowl",
        "category_idx": 2,
        "relative_plane": "a",
        "label": "on_a",
        "default_pos": [-0.2, -0.1, 0.85],
        "default_ori": [1, 0, 0, 0],
        "scale": [1, 1, 1],
    }
    layout["Rigid"]["bowl"].append(child)
    gen = X.Generator("stack_bowls", {"position": True}, assets(), [("t.json", layout)], POOLS)
    orders = set()
    for i in range(10):
        out, _ = gen.layout(i, 1)
        xs = {b["label"]: round(b["default_pos"][0], 6) for b in out["Rigid"]["bowl"]}
        assert sorted(xs[k] for k in "abc") == [-0.2, 0.0, 0.2]  # the three slots, each taken once
        assert xs["on_a"] == xs["a"]  # the child rides on its parent
        orders.add(tuple(xs[k] for k in "abc"))
    assert len(orders) > 1


def test_camera_configs():
    root, real = Path(tempfile.mkdtemp()), X.ROOT
    for rel in ("env_cfg/arx_x5.yml", "env_cfg/camera/camera_config.yml"):
        (root / rel).parent.mkdir(parents=True, exist_ok=True)
        (root / rel).write_text((real / rel).read_text(encoding="utf-8"), encoding="utf-8")
    X.ROOT = root
    try:
        name = X.camera_configs("stack_bowls", {"ori_deg": 2})
    finally:
        X.ROOT = real
    env = X.load_yaml(root / "env_cfg" / f"{name}.yml")
    cam = X.load_yaml(root / "env_cfg" / "camera" / "rsi_stack_bowls.yml")["random"]["cam_head"]
    assert env["config"]["camera"] == "rsi_stack_bowls"
    assert cam == {"enabled": True, "ori": {"min": [-2, -2, -2], "max": [2, 2, 2]}}  # no position jitter
