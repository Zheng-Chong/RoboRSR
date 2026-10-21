"""Prepare bean-pour layouts (g1d_pour_beans_cup_or_mug): tall measuring cup -> short measuring cup or white mug,
100-300 red beans, 2-5 Clutter distractors, on any G1-D tabletop scene.

Pouring hand, both cups' positions and the tall cup's yaw come from prepare_office_beans_layouts.sample_layouts. On
top of that, per layout:
- target: the short measuring cup (any yaw, sampled there) or the mug, half and half shuffled. The mug's handle (+X of
  the asset) points along the holding hand's fingers, within MUG_HANDLE deg: the pads close across the body and the
  palm comes from the other side, so the handle is clear of both;
- beans: a stratified count in BEANS; the bean ellipsoid is scaled by (count / BEANS_REF)^(1/3) from the 0.9 fill
  (BEANS_REF beans) and lowered to keep its bottom gap. The simulator samples the particles, so the actual count is
  near the target; the collection plugin records it and rejects counts outside BEANS;
- distractors: 2-5 Clutter objects (distinct categories) standing upright at a random yaw in the table's back strip
  and outer sides, clear of each other and of the cups' and arms' working area (ZONES, CLEAR).

Usage (repo root; reads metadata only, no simulator):
    python scripts/g1d/prepare_bean_pour_layouts.py \
        --template Assets/Eval_Layout/RoboDojo/g1d_shenzhen_lab/0/g1d_pour_beans_measuring_cups_0.json \
        --output Assets/Eval_Layout/RoboDojo/g1d_shenzhen_lab/9 --count 1000
"""

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import random
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.g1d.prepare_office_beans_layouts import sample_layouts, yaw_wxyz  # noqa: E402

TASK = "g1d_pour_beans_cup_or_mug"
BEANS, BEANS_REF, FILL_REF = (100, 300), 672, 0.9  # beans; beans at the 0.9 fill of the tall cup (2026-10-19 runs)
BEAN_GAP, BEAN_HALF = 0.0048, 0.035  # bottom gap above the outside bottom, ellipsoid half height at scale 1 (m)
MUG_HANDLE = 40  # deg either side of the holding hand's finger direction
CUP_YAW = 45  # scripted_demo.CUP_YAW: the hand turns this far inward from fingers-forward
DISTRACTORS = (2, 5)
DISTRACTOR_MAX_R = 0.09  # m, largest footprint (projection circle) used
# (x range, y range) of distractor footprint centres, |x| mirrored for the sides: the back strip and both outer sides
ZONES = (((-0.50, 0.50), (-0.20, -0.06)), ((0.30, 0.52), (-0.52, -0.22)), ((-0.52, -0.30), (-0.52, -0.22)))
TABLE = ((-0.58, 0.58), (-0.58, -0.02))  # footprints stay inside
CLEAR = 0.10  # m between a distractor's footprint and a cup centre
GAP = 0.02  # m between distractor footprints


def bean_counts(count, rng):
    low, high = BEANS
    values = [round(low + (high - low) * (i + rng.random()) / count) for i in range(count)]
    rng.shuffle(values)
    return values


def fill_scale(beans):
    return FILL_REF * (beans / BEANS_REF) ** (1 / 3)


def place_pose(trans, xy, z, yaw):
    """Object origin pose [x, y, z, w, x, y, z] putting its placement frame `trans` (origin -> placement centre, in
    the object frame) at (xy, z) turned `yaw` deg about z."""
    q = np.asarray(trans[3:], float)
    t_rot = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    rot = Rotation.from_euler("z", yaw, degrees=True) * t_rot.inv()
    pos = np.r_[xy, z] - rot.apply(trans[:3])
    x, y, z_, w = rot.as_quat()
    return pos.tolist(), [float(w), float(x), float(y), float(z_)]


def clutter_library(clutter_dir):
    """[(category, index, metadata)] of Clutter objects with a model and an upright placement small enough."""
    out = []
    for folder in sorted(Path(clutter_dir).glob("*/0*")):
        if not any((folder / name).exists() for name in ("object.usdz", "object.usd")):
            continue
        meta = json.loads((folder / "metadata.json").read_text())
        up = ((meta.get("active") or {}).get("place") or {}).get("up")
        if up and up["projection_circle"]["radius"] <= DISTRACTOR_MAX_R:
            out.append((folder.parent.name, int(folder.name), meta))
    return out


def sample_distractors(rng, library, cups, table_z, template):
    """Distractor Rigid entries (type cluttered) clear of the cups and each other; cups = [(x, y)]."""
    by_category = {}
    for category, index, meta in library:
        by_category.setdefault(category, []).append((index, meta))
    chosen = rng.sample(sorted(by_category), rng.randint(*DISTRACTORS))
    placed, entries = [], []
    for k, category in enumerate(chosen):
        index, meta = rng.choice(by_category[category])
        circle = meta["active"]["place"]["up"]["projection_circle"]
        radius = circle["radius"]
        for _ in range(500):
            (x_range, y_range) = rng.choice(ZONES)
            x, y = rng.uniform(*x_range), rng.uniform(*y_range)
            inside = (
                TABLE[0][0] + radius <= x <= TABLE[0][1] - radius and TABLE[1][0] + radius <= y <= TABLE[1][1] - radius
            )
            clear = all(math.dist((x, y), c) >= radius + CLEAR for c in cups)
            apart = all(math.dist((x, y), p) >= radius + r + GAP for p, r in placed)
            if inside and clear and apart:
                break
        else:
            continue  # no room left for this one
        yaw = rng.uniform(0, 360)
        pos, ori = place_pose(circle["center"], (x, y), table_z, yaw)
        placed.append(((x, y), radius))
        physics = dict(meta.get("physics") or {}, type="rigid")
        entries.append(
            (category, dict(deepcopy(template), category=category, category_idx=index, type="cluttered",
                            label=f"distractor_{k}", default_pos=pos, default_ori=ori, physics=physics))
        )  # fmt: skip
    return entries


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--template", type=Path, required=True, help="a two-measuring-cups layout of the scene")
    parser.add_argument("--output", type=Path, required=True, help="new layout seed directory")
    parser.add_argument("--assets", type=Path, default=ROOT / "Assets")
    parser.add_argument("--count", type=int, default=1000)
    parser.add_argument("--random-seed", type=int, default=20261021)
    args = parser.parse_args()
    template = json.loads(args.template.read_text())
    cups = {c["label"]: c for c in template["Rigid"]["measuring_cup"]}
    if {(label, c["category_idx"]) for label, c in cups.items()} != {("source_cup", 1), ("target_cup", 0)}:
        parser.error("Template must pour tall measuring cup 1 into short measuring cup 0")
    objects = args.assets / "Object/RoboDojo"
    mug_meta = json.loads((objects / "Rigid/plastic_mug/00000/metadata.json").read_text())
    library = clutter_library(objects / "Clutter")
    if len({c for c, _, _ in library}) < DISTRACTORS[1]:
        parser.error(f"Only {len(library)} usable Clutter objects in {objects / 'Clutter'}")
    source_entry, target_entry = cups["source_cup"], cups["target_cup"]
    table_z = target_entry["default_pos"][2] - target_entry["physics"]["size"][2] / 2 - 1e-4
    tall_h = source_entry["physics"]["size"][2]

    rng = random.Random(args.random_seed)
    layouts = sample_layouts(args.count, args.random_seed)
    kinds = ["measuring_cup", "plastic_mug"] * (args.count // 2) + ["measuring_cup"] * (args.count % 2)
    rng.shuffle(kinds)
    beans = bean_counts(args.count, rng)
    args.output.mkdir(parents=True, exist_ok=False)
    records = []
    for i, ((side, sx, sy, tx, ty, ys, yt), kind, n) in enumerate(zip(layouts, kinds, beans)):
        layout = deepcopy(template)
        source = dict(deepcopy(source_entry), default_ori=yaw_wxyz(ys))
        source["default_pos"][:2] = [sx, sy]
        if kind == "measuring_cup":
            target = dict(deepcopy(target_entry), default_ori=yaw_wxyz(yt))
        else:
            fingers = 90 - side * CUP_YAW  # the target is held by the hand on side -side (scripted_demo.cup_grasp)
            yt = (fingers + rng.uniform(-MUG_HANDLE, MUG_HANDLE)) % 360
            size = mug_meta["physics"]["size"]
            target = dict(deepcopy(target_entry), category="plastic_mug", category_idx=0, default_ori=yaw_wxyz(yt))
            target["physics"] = dict(mug_meta["physics"], type="rigid")
            target["default_pos"][2] = table_z + size[2] / 2 + 1e-4
        target["default_pos"][:2] = [tx, ty]
        layout["Rigid"] = {"measuring_cup": [source]} if kind == "plastic_mug" else {"measuring_cup": [source, target]}
        if kind == "plastic_mug":
            layout["Rigid"]["plastic_mug"] = [target]
        scale = fill_scale(n)
        fluid = layout["Fluid"]["wuliangye"][0]
        rel_z = BEAN_GAP + BEAN_HALF * scale - tall_h / 2
        fluid.update(zlim=[rel_z, rel_z], visual=dict(fluid["visual"], scale=[scale] * 3), scale=[1 / scale] * 3)
        fluid["default_pos"] = [sx, sy, source["default_pos"][2] + rel_z]
        distractors = sample_distractors(rng, library, [(sx, sy), (tx, ty)], table_z, source_entry)
        for category, entry in distractors:
            layout["Rigid"].setdefault(category, []).append(entry)
        (args.output / f"{TASK}_{i}.json").write_text(json.dumps(layout, indent=2) + "\n")
        hand = "left" if side < 0 else "right"
        names = [c for c, _ in distractors]
        record = dict(hand=hand, target=kind, target_beans=n, fill_scale=scale, distractors=names)
        records.append(dict(record, source=[sx, sy, ys], target_pose=[tx, ty, yt]))
    (args.output / "sampling.json").write_text(
        json.dumps(
            dict(
                method="latin_hypercube_with_balanced_hand_and_target",
                seed=args.random_seed,
                count=args.count,
                beans=BEANS,
                mug_handle_deg=MUG_HANDLE,
                distractors=DISTRACTORS,
                distractor_zones=ZONES,
                clutter=sorted({c for c, _, _ in library}),
                layouts=records,
            ),
            indent=2,
        )
        + "\n"
    )
    mugs = kinds.count("plastic_mug")
    print(f"{args.count} layouts in {args.output}: {mugs} mug targets, beans {min(beans)}-{max(beans)}")


if __name__ == "__main__":
    main()
