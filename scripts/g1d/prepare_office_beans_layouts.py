"""Prepare distinct office cup positions by four-dimensional Latin hypercube sampling."""

import argparse
from copy import deepcopy
import json
from pathlib import Path
import random


def sample_positions(count, seed):
    if count < 1:
        raise ValueError("count must be positive")
    rng = random.Random(seed)
    columns = []
    for low, high in ((-0.16, -0.10), (-0.39, -0.31), (0.10, 0.16), (-0.39, -0.31)):
        values = [low + (high - low) * (i + rng.random()) / count for i in range(count)]
        rng.shuffle(values)
        columns.append(values)
    return list(zip(*columns))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
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
    positions = sample_positions(args.count, args.random_seed)
    args.output.mkdir(parents=True, exist_ok=False)
    for i, (sx, sy, tx, ty) in enumerate(positions):
        layout = deepcopy(template)
        for cup in layout["Rigid"]["measuring_cup"]:
            cup["default_pos"][:2] = (sx, sy) if cup["label"] == "source_cup" else (tx, ty)
        layout["Fluid"]["wuliangye"][0]["default_pos"][:2] = (sx, sy)
        (args.output / f"g1d_pour_beans_measuring_cups_{i}.json").write_text(json.dumps(layout, indent=2) + "\n")
    (args.output / "sampling.json").write_text(
        json.dumps(
            dict(method="latin_hypercube", seed=args.random_seed, count=args.count, positions=positions), indent=2
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
