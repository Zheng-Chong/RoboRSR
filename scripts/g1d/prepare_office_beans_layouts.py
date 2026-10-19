"""Prepare distinct office cup layouts: pouring hand, both cups' positions and yaws.

The hand alternates over a shuffled half-and-half split (side -1: the tall cup on the robot's left, so the left hand
pours into the short cup on the right; +1 mirrored). Both cups' |x|, y and yaw come from one six-dimensional Latin
hypercube; yaw spans 0-90 deg because the cups' square section repeats every 90 deg (scripted_demo.flat_turn turns
the wrist onto a pair of flats).
"""

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import random

# m; |x| of the tall (pouring) and short cup, both cups' y; robot-side table edge at y = -0.60. Probed with random hand
# and yaws (64 pours, office): inside these 16/23 passed the strict check; past them cups drop out of the grip (cups
# beyond y -0.28 or a short cup past |x| 0.19) or the tall cup's grasp runs out of reach (y below -0.46)
TALL_X, SHORT_X, CUP_Y = (0.05, 0.22), (0.05, 0.18), (-0.46, -0.28)
YAW = (0.0, 90.0)  # deg
RANGES = (TALL_X, CUP_Y, SHORT_X, CUP_Y, YAW, YAW)


def sample_layouts(count, seed):
    """[(side, tall x, tall y, short x, short y, tall yaw, short yaw)]; x signed, yaw in degrees."""
    if count < 1:
        raise ValueError("count must be positive")
    rng = random.Random(seed)
    columns = []
    for low, high in RANGES:
        values = [low + (high - low) * (i + rng.random()) / count for i in range(count)]
        rng.shuffle(values)
        columns.append(values)
    sides = [-1, 1] * (count // 2) + [-1] * (count % 2)
    rng.shuffle(sides)
    rows = zip(sides, zip(*columns))
    return [(side, side * sx, sy, -side * tx, ty, ys, yt) for side, (sx, sy, tx, ty, ys, yt) in rows]


def yaw_wxyz(deg):
    return [math.cos(math.radians(deg) / 2), 0.0, 0.0, math.sin(math.radians(deg) / 2)]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--template", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New g1d layout seed directory")
    parser.add_argument("--count", type=int, default=400)
    parser.add_argument("--random-seed", type=int, default=20261019)
    args = parser.parse_args()
    template = json.loads(args.template.read_text())
    if template["Room"]["default"] != "real_scene1":
        parser.error("Template must use the office scene")
    cups = template["Rigid"]["measuring_cup"]
    if {(c["label"], c["category_idx"]) for c in cups} != {("source_cup", 1), ("target_cup", 0)}:
        parser.error("Template must pour tall cup 1 into short cup 0")
    layouts = sample_layouts(args.count, args.random_seed)
    args.output.mkdir(parents=True, exist_ok=False)
    for i, (_, sx, sy, tx, ty, ys, yt) in enumerate(layouts):
        layout = deepcopy(template)
        for cup in layout["Rigid"]["measuring_cup"]:
            src = cup["label"] == "source_cup"
            cup["default_pos"][:2] = (sx, sy) if src else (tx, ty)
            cup["default_ori"] = yaw_wxyz(ys if src else yt)
        layout["Fluid"]["wuliangye"][0]["default_pos"][:2] = (sx, sy)
        (args.output / f"g1d_pour_beans_measuring_cups_{i}.json").write_text(json.dumps(layout, indent=2) + "\n")
    (args.output / "sampling.json").write_text(
        json.dumps(
            dict(
                method="latin_hypercube_with_balanced_hand",
                seed=args.random_seed,
                count=args.count,
                ranges=dict(tall_abs_x=TALL_X, short_abs_x=SHORT_X, y=CUP_Y, yaw_deg=YAW),
                columns=["side", "source_x", "source_y", "target_x", "target_y", "source_yaw", "target_yaw"],
                layouts=layouts,
            ),
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
