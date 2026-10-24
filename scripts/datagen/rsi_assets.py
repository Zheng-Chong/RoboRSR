"""Asset tiers of rsi/AGENTS.md rule 1: what the benchmark keeps for itself, and how the rest splits between the data
agent (train) and the exam agent (exam).

    T0 (reserved)  in an official eval layout (Eval_Layout/RoboDojo/arx_x5/0), or in the config of a task without
                   official training data (the _random variants, the open tasks), but never in the official training
                   data: nobody generates with it
    seen           in the official training data (eval layouts and configs of the 34 tasks it covers): both
    train / exam   never in an official layout: split 7:3 by a hash of the asset id, so adding assets never moves one

Stdlib only (the platform server imports it). Asset ids: room:<dir>, material:<dir>, hdr:<file>,
clutter:<category>/<index>, <Section>:<category>/<index> for task-object sections.

    python scripts/datagen/rsi_assets.py scan --assets Assets          # writes rsi/assets/pools.json
    python scripts/datagen/rsi_assets.py audit --assets Assets --layouts <seed folder> --pool train
"""

import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
POOLS = ROOT / "rsi" / "assets" / "pools.json"
# tasks with official training data (HF RoboDojo-Benchmark/RoboDojo data/RoboDojo/, 2026-10-22)
OFFICIAL_TRAIN_TASKS = frozenset(
    "arrange_largest_number build_tower classify_objects cover_blocks deposit_coin fasten_screws fill_egg_holder "
    "fill_pen_holder fold_clothes hang_mugs imitate_sorting_sequence insert_key insert_tubes make_kong make_toast "
    "match_and_pick_from_conveyor organize_table pack_objects_into_box play_Xylophone play_stacking_toy "
    "play_tic_tac_toe plug_in_charger pour_balls_into_vase pour_liquid_into_cup press_by_number push_T "
    "put_bottles_into_dustbin sort_nesting_dolls_by_size stack_blocks stack_bowls store_laptop_and_headphones swap_T "
    "swap_blocks sweep_blocks".split()
)
SECTIONS = ("Rigid", "Geometry", "Articulation", "Garment")
TRAIN_SHARE = 0.7
MAX_DISTRACTOR_M = (
    0.4  # longest bounding-box side of a new distractor; a saw or a sickle does not fit the clutter strips
)


def layout_assets(layout):
    """Asset ids an eval layout uses, and the categories of its task objects."""
    ids, categories = set(), set()
    ids.add(f"room:{(layout.get('Room') or {}).get('default')}")
    ids.add(f"material:{(layout.get('Table') or {}).get('default')}")
    ids.add(f"material:{((layout.get('Ground') or {}).get('materials') or {}).get('default')}")
    ids.add(f"hdr:{(layout.get('Background') or {}).get('category_name')}")
    for section in SECTIONS:
        for category, entries in (layout.get(section) or {}).items():
            for e in entries:
                if e.get("type") == "cluttered":
                    ids.add(f"clutter:{category}/{e['category_idx']}")
                else:
                    ids.add(f"{section}:{category}/{e['category_idx']}")
                    categories.add(category)
    return {i for i in ids if not i.endswith(":None")}, categories


def library(assets):
    """Every asset id in an Assets/ tree, with the bounding-box extents of objects."""
    assets, found = Path(assets), {}
    for p in sorted((assets / "Room").glob("*")):
        if p.is_dir():
            found[f"room:{p.name}"] = None
    for p in sorted((assets / "Material").glob("material*")):
        if p.is_dir():
            found[f"material:{p.name}"] = None
    for p in sorted((assets / "Background").glob("*.hdr")):
        found[f"hdr:{p.name}"] = None
    objects = assets / "Object" / "RoboDojo"
    for section in ("Clutter", *SECTIONS):
        for meta in sorted(objects.glob(f"{section}/*/*/metadata.json")):
            if not meta.parent.name.isdigit():
                continue
            geometry = json.loads(meta.read_text(encoding="utf-8")).get("geometry") or {}
            extents = (geometry.get("aligned_bbox") or geometry.get("oriented_bbox") or {}).get("extents")
            prefix = "clutter" if section == "Clutter" else section
            found[f"{prefix}:{meta.parent.parent.name}/{int(meta.parent.name)}"] = extents
    return found


def side(asset_id, seed="rsi-v1"):
    digest = hashlib.sha256(f"{seed}:{asset_id}".encode()).digest()
    return "train" if int.from_bytes(digest[:8], "big") / 2**64 < TRAIN_SHARE else "exam"


def config_instances(task):
    """Asset ids a task config can draw (its category index lists): part of the official training distribution even
    when no eval layout happens to show them."""
    import yaml  # the scan runs in the sim env; the platform server only reads pools.json

    path = ROOT / "task" / "RoboDojo" / "config" / f"{task}.yml"
    config = yaml.safe_load(path.read_text(encoding="utf-8")) if path.is_file() else {}
    return {
        f"{section}:{c['name']}/{i}"
        for section in SECTIONS
        for block in (config or {}).get(section) or []
        for c in block.get("category") or []
        for i in c.get("index") or []
    }


def scan(assets):
    folder = Path(assets) / "Eval_Layout" / "RoboDojo" / "arx_x5" / "0"
    seen = set().union(*(config_instances(t) for t in OFFICIAL_TRAIN_TASKS))
    upstream = [p.stem for p in (ROOT / "task" / "RoboDojo" / "config").glob("*.yml")]
    upstream = [t for t in upstream if not t.startswith(("_", "g1d_")) and t not in OFFICIAL_TRAIN_TASKS]
    evaluated = set().union(*(config_instances(t) for t in upstream))  # what the benchmark-only configs can draw
    task_categories = set()
    for f in sorted(folder.glob("*.json")):
        task, _, index = f.stem.rpartition("_")
        if not index.isdigit():
            continue
        ids, categories = layout_assets(json.loads(f.read_text(encoding="utf-8")))
        evaluated |= ids
        task_categories |= categories
        if task in OFFICIAL_TRAIN_TASKS:
            seen |= ids
    t0 = evaluated - seen
    pools = {
        p: {"rooms": [], "materials": [], "hdrs": [], "distractors": [], "instances": []} for p in ("train", "exam")
    }
    dropped = []
    for asset_id, extents in library(assets).items():
        if asset_id in evaluated or asset_id in seen:
            continue
        kind, _, name = asset_id.partition(":")
        if kind in ("room", "material", "hdr"):
            pools[side(asset_id)][f"{kind}s"].append(name)
            continue
        category = name.split("/")[0]
        if kind != "clutter" and category in task_categories:
            pools[side(asset_id)]["instances"].append(asset_id)
        elif extents and max(extents) <= MAX_DISTRACTOR_M:
            pools[side(asset_id)]["distractors"].append(asset_id)
        else:
            dropped.append(asset_id)
    return {
        "source": str(folder.relative_to(Path(assets))),
        "official_train_tasks": len(OFFICIAL_TRAIN_TASKS),
        "seen": sorted(seen),
        "t0": sorted(t0),
        "pools": pools,
        "dropped": sorted(dropped),
    }


def load(path=POOLS):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def allowed(pools, pool):
    """Asset ids a party may generate with: the seen ones and its own pool."""
    own = pools["pools"][pool]
    ids = set(pools["seen"]) | set(own["distractors"]) | set(own["instances"])
    ids |= {f"room:{r}" for r in own["rooms"]} | {f"material:{m}" for m in own["materials"]}
    return ids | {f"hdr:{h}" for h in own["hdrs"]}


def audit(layout_dir, pools, pool):
    """{layout file: [asset ids it may not use]} of a generated seed folder."""
    ok, bad = allowed(pools, pool), {}
    for f in sorted(Path(layout_dir).glob("*.json")):
        task, _, index = f.stem.rpartition("_")
        if not index.isdigit():
            continue
        ids, _ = layout_assets(json.loads(f.read_text(encoding="utf-8")))
        if extra := sorted(ids - ok):
            bad[f.name] = extra
    return bad


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("scan")
    s.add_argument("--assets", type=Path, required=True)
    s.add_argument("--out", type=Path, default=POOLS)
    a = sub.add_parser("audit")
    a.add_argument("--assets", type=Path, required=True)
    a.add_argument("--layouts", type=Path, required=True)
    a.add_argument("--pool", choices=("train", "exam"), required=True)
    args = ap.parse_args(argv)
    if args.cmd == "scan":
        result = scan(args.assets)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
        counts = {p: {k: len(v) for k, v in d.items()} for p, d in result["pools"].items()}
        print(f"T0 {len(result['t0'])}, seen {len(result['seen'])}, dropped {len(result['dropped'])}, pools {counts}")
    else:
        bad = audit(args.layouts, load(), args.pool)
        for name, ids in bad.items():
            print(f"{name}: {', '.join(ids)}")
        raise SystemExit(1 if bad else 0)


if __name__ == "__main__":
    main()
