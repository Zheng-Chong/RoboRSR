"""Render every preview camera in its own process and retry blank frames.

Open3D/Filament on this machine intermittently returns all-black frames, so each output is checked
and re-rendered (up to 4 attempts, alternating resolution for the 4:3 photo cameras).
"""
import subprocess
import sys
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
REAL = ["v0040", "v0100", "v0160", "v0260", "v0280", "v0300", "v0340", "v0400", "v0420", "v0480",
        "p0_25a883", "p1_2a4974", "p2_6dda6d", "p3_c68ebb", "p4_cf220a"]
FREE = ["overview_ne", "overview_sw", "top_down", "task_area", "robot_head_est"]


def ok(path):
    im = cv2.imread(str(path))
    return im is not None and im.mean() > 8.0


def run(kind, key, tag):
    out = HERE / "preview" / (f"render_{key}{tag}.jpg" if kind == "real" else f"view_{key}{tag}.jpg")
    widths = [800, 800, 640, 800] if key.startswith("p") else [960, 960, 800, 960]
    for attempt, w in enumerate(widths):
        if out.exists():
            out.unlink()
        subprocess.run([sys.executable, str(HERE / "render_preview.py"), f"--{kind}", key, "--tag", tag,
                        "--width", str(w)], cwd=HERE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if ok(out):
            return f"{key}: ok (attempt {attempt + 1}, width {w})"
    return f"{key}: STILL BLANK after {len(widths)} attempts"


if __name__ == "__main__":
    args = sys.argv[1:]
    tag = args[args.index("--tag") + 1] if "--tag" in args else ""
    only = {a for a in args if a != "--tag" and a != tag}
    for kind, keys in (("real", REAL), ("free", FREE)):
        for k in keys:
            if not only or k in only:
                print(run(kind, k, tag), flush=True)
