"""Put a scene twin's chosen surface materials and desk height into G1-D layouts.

Inputs, per scene (real2sim/scenes/<dir>/):
- usd/parts.json (scripts/scenes/scene_parts.py): the floor, wall and tabletop meshes, and how the desk lifts;
- appearance.json (set on the Real2Sim platform, scene drawer):
      {"fixed":  {"floor": <material id>, "wall": ..., "tabletop": ..., "table_top_z": 0.85},
       "random": {"floor": [<ids>], "wall": [...], "tabletop": [...], "table_top_z": [0.70, 0.95]}}
  Ids are real2sim/materials/library.json entries, or "original" for the scene's own material. Missing or null keys
  keep the twin as built. "fixed" applies to every layout; with --randomize, a role (or the height) that has a "random"
  pool is drawn per layout from it instead, reproducibly from --seed and the layout's file name.
Each layout's Room gets "appearance" and "table_lift" (applied by env/scene_manager/objects/room_overrides.py), the
record "appearance_choice", and the new "table_height"; the task objects move up or down with the tabletop. The robot
stays where its env config puts it (lift column included), so a taller desk is a different reach.

    python scripts/scenes/appearance.py real2sim/scenes/real_scene3_lab Assets/Eval_Layout/RoboDojo/g1d_lab/9
    python scripts/scenes/appearance.py real2sim/scenes/real_scene3_lab <layout dir> --randomize --seed 7
Rewrites the layouts in place and writes <layout dir>/appearance_choices.json (file -> choice).
"""

import argparse
import json
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[2]
LIBRARY = ROOT / "real2sim" / "materials" / "library.json"
ROLES = ("floor", "wall", "tabletop")
ORIGINAL = "original"


def load(scene_dir):
    scene_dir = Path(scene_dir)
    parts = json.loads((scene_dir / "usd" / "parts.json").read_text(encoding="utf-8"))
    path = scene_dir / "appearance.json"
    settings = json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}
    library = {m["id"]: m for m in json.loads(LIBRARY.read_text(encoding="utf-8"))} if LIBRARY.is_file() else {}
    return parts, settings, library


def choose(settings, rng=None):
    """{role: id | None, "table_top_z": z | None}: the fixed choice, roles with a random pool redrawn when rng."""
    fixed, pools = settings.get("fixed") or {}, settings.get("random") or {}
    choice = {key: fixed.get(key) for key in (*ROLES, "table_top_z")}
    if rng is not None:
        for role in ROLES:
            if pools.get(role):
                choice[role] = rng.choice(pools[role])
        if pools.get("table_top_z"):
            low, high = pools["table_top_z"]
            choice["table_top_z"] = round(rng.uniform(low, high), 3)
    return choice


def material_spec(entry):
    if entry["kind"] == "mdl":
        return {"kind": "mdl", "mdl": entry["mdl"], "tile_m": entry.get("tile_m") or 1.0}
    return {"kind": "preview"} | {k: entry.get(k) for k in ("color", "texture", "roughness", "metallic")}


def overrides(parts, choice, library):
    """Room entries for `choice`: ({"appearance": [...], "table_lift": {...}}, new table top z)."""
    groups = {}
    for role in ROLES:
        mid = choice.get(role)
        if not mid or mid == ORIGINAL:
            continue
        if mid not in library:
            raise KeyError(f"{role}: material {mid!r} is not in {LIBRARY.relative_to(ROOT)}")
        entry = library[mid]
        for part in parts["roles"][role]["parts"]:
            scale = None
            if entry["kind"] == "preview" and entry.get("texture") and part["uv_m"] and entry.get("tile_m"):
                scale = round(part["uv_m"] / entry["tile_m"], 4)  # keep the texture's real repeat size
            groups.setdefault((mid, scale), []).append(part["path"])
    room = {
        "appearance": [
            {"prims": prims, "material": material_spec(library[mid]), "uv_scale": scale}
            for (mid, scale), prims in groups.items()
        ]
    }
    top = parts["top_z"]
    z = choice.get("table_top_z")
    if z is not None and abs(z - top) > 1e-6:
        lift = parts.get("lift")
        if not lift:
            raise ValueError(f"{parts['scene']}: {parts.get('fixed_reason') or 'the desk does not lift'}")
        low, high = lift["range"]
        if not low <= z <= high:
            raise ValueError(f"{parts['scene']}: table_top_z {z} outside the desk's travel {low}-{high} m")
        room["table_lift"] = {"dz": round(z - top, 4), "move": lift["move"], "stretch": lift["stretch"]}
        top = z
    return room, top


def apply(layout, scene_dir, rng=None, loaded=None):
    """Set the Room overrides of `layout` for the scene's appearance settings; returns the choice (None if unset)."""
    sys.path.insert(0, str(ROOT / "scripts" / "g1d"))
    from build_asset import shift_objects

    parts, settings, library = loaded or load(scene_dir)
    if not settings:
        return None
    choice = choose(settings, rng)
    room_overrides, top = overrides(parts, choice, library)
    room = layout["Room"]
    for key in ("appearance", "table_lift"):
        room.pop(key, None)
    room.update({k: v for k, v in room_overrides.items() if v})
    shift_objects(layout, (0.0, 0.0, top - room.get("table_height", parts["top_z"])))
    room["table_height"] = top
    room["appearance_choice"] = choice
    return choice


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scene", type=Path, help="real2sim/scenes/<dir>")
    parser.add_argument("layouts", type=Path, help="directory of layout json files of that scene")
    parser.add_argument("--randomize", action="store_true", help="draw the roles that have a random pool per layout")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    loaded = load(args.scene)
    if not loaded[1]:
        parser.error(f"no {args.scene / 'appearance.json'}: set the appearance on the Real2Sim platform first")
    choices = {}
    for path in sorted(args.layouts.glob("*.json")):
        layout = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(layout, dict) or "Room" not in layout:
            continue
        rng = random.Random(f"{args.seed}/{path.name}") if args.randomize else None
        choices[path.name] = apply(layout, args.scene, rng, loaded)
        path.write_text(json.dumps(layout, indent=2) + "\n", encoding="utf-8")
    (args.layouts / "appearance_choices.json").write_text(json.dumps(choices, indent=1) + "\n", encoding="utf-8")
    print(f"{len(choices)} layouts in {args.layouts}")


if __name__ == "__main__":
    main()
