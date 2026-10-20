"""Acceptance of the real-scene twins against the README monthly targets, from real2sim/scenes/*/scene_spec.json:

- tape-measured sizes within 1 cm of the model (tabletop height, length, width; where the table edge sits relative to
  the robot);
- the head-camera overlay (scripts/scenes/overlay.py) under 5 px in at least 3 views;
- the lighting recorded on site.

Model values are read from the scene's usd/build_params.json and the env config's robot stand, never typed into the
spec. Usage: python scripts/scenes/acceptance.py [--json]; exits 1 when a spec is malformed, not when a target is open.
"""

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.g1d.build_asset import SCENES  # noqa: E402
from scripts.scenes.overlay import LIMIT_PX, load_yaml  # noqa: E402

TAPE_M, VIEWS = 0.01, 3
LIGHTING = ("estimated", "recorded", "matched")  # guessed / types, positions, brightness noted on site / + head frame
KEYS = {"scene", "env_cfg", "photos", "reconstruction", "scale_basis", "measurements", "overlay", "lighting"}


def model_values(env_cfg):
    """The twin's sizes as built, m. Robot root = midpoint between the two drive wheels' centres (AGV_link)."""
    config = load_yaml(ROOT / "env_cfg" / f"{env_cfg}.yml")
    scene, _, room_pos, _ = SCENES[config["config_name"]]
    table = json.loads((ROOT / "real2sim/scenes" / scene / "usd/build_params.json").read_text())["table"]
    root = load_yaml(ROOT / "env_cfg/robot" / f"{config['config']['robot']}.yml")["robots"][0]["default_root_pos"]
    front = room_pos[1] + table.get("front_y", -table["width"] / 2)
    return dict(
        table_top_z=table["top_z"],
        table_length=table["length"],
        table_width=table["width"],
        table_edge_from_robot_root=front - root[1],
        table_centre_x_from_robot_root=room_pos[0] - root[0],
    )


def evaluate(spec):
    if missing := KEYS - set(spec):
        raise ValueError(f"{spec.get('scene')}: missing {sorted(missing)}")
    if spec["lighting"]["status"] not in LIGHTING:
        raise ValueError(f"{spec['scene']}: lighting.status must be one of {LIGHTING}")
    model = model_values(spec["env_cfg"])
    if unknown := set(spec["measurements"]) - set(model):
        raise ValueError(f"{spec['scene']}: unknown measurements {sorted(unknown)}; known {sorted(model)}")
    tape = {}
    for name, value in model.items():
        measured = (spec["measurements"].get(name) or {}).get("value_m")
        diff = None if measured is None else round(abs(measured - value), 4)
        state = "pending" if diff is None else "pass" if diff < TAPE_M else "fail"
        tape[name] = dict(model_m=round(value, 4), measured_m=measured, diff_m=diff, status=state)
    views = spec["overlay"]
    worst = max((v["max_px"] for v in views), default=None)
    overlay = "pending" if len(views) < VIEWS else "pass" if worst < LIMIT_PX else "fail"
    lighting = "pass" if spec["lighting"]["status"] != "estimated" else "pending"
    statuses = [t["status"] for t in tape.values()] + [overlay, lighting]
    overall = "fail" if "fail" in statuses else "pending" if "pending" in statuses else "pass"
    return dict(
        scene=spec["scene"],
        tape=tape,
        overlay=dict(views=len(views), max_px=worst, status=overlay),
        lighting=lighting,
        status=overall,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    reports = [
        evaluate(json.loads(p.read_text())) for p in sorted((ROOT / "real2sim/scenes").glob("*/scene_spec.json"))
    ]
    if args.json:
        print(json.dumps(reports, indent=2))
        return
    for r in reports:
        tape = ", ".join(f"{k} {v['status']}" for k, v in r["tape"].items())
        print(f"{r['scene']}: {r['status']} | tape: {tape} | overlay {r['overlay']} | lighting {r['lighting']}")


if __name__ == "__main__":
    main()
