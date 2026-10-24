"""Fit the lounge next to the counter (three phone photos, reference/lounge{1,2,3}.jpg) to hand-picked pixels: the three
camera poses, the wallpaper alcove and the round table.

Lounge frame (this script only): Z up, metres; x = 0 on the left face of the counter's left column (the alcove's right
side), y = 0 on the wallpaper, +X toward the counter, +Y into the wall. The alcove is the wallpaper wall between that
column (front at y = -col_depth) and a wood column on its left (inner face at x = -width, front at y = -left_depth).
Wood wainscot below the wallpaper, a back-lit green slat band above it, then a wood soffit.

Absolute scale comes from the round cafe table only (top assumed 0.75 m above the floor); nothing was tape-measured.
The focal length is held near the counter photos' fit (fit_photo.json, same phone, same 1024 x 768 crop). "points"
are corners; "lines" are pixels on a model edge whose ends are hidden (the counter column hides the wallpaper's right
end from where lounge1/2 were taken). Table: the extremes of the top's rim ellipse and of the pedestal's base disc.
Chairs and bins are placed afterwards by back-projecting their floor contacts with the fitted cameras ("floor").
Prints the fit and per-pixel errors; writes fit_lounge.json, read by build_scene.py.
"""

import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
W, H = 1024, 768
TABLE_Z = 0.75  # assumed: the only scale prior
FOCAL = json.loads((HERE / "fit_photo.json").read_text())["focal_px"]

DIMS = {  # name: (prior, sigma); loose, they only hold unobserved directions
    "width": (2.9, 0.6),
    "wainscot_z": (0.85, 0.2),
    "paper_z": (1.95, 0.3),
    "band_z": (2.35, 0.3),
    "col_depth": (0.54, 0.3),
    "left_depth": (0.45, 0.3),
    "table_x": (-1.2, 0.8),
    "table_y": (-1.2, 0.6),
    "table_r": (0.35, 0.1),
    "base_r": (0.24, 0.08),
    "focal": (FOCAL, 30.0),
}
# Pixels picked on 2-3x zoomed, gridded crops.
PHOTOS = {
    "lounge1.jpg": {
        "points": {"paper_tl": (147, 321), "paper_bl": (118, 542), "band_tl": (148, 240)},
        "lines": {
            "paper_top": [(938, 270)],
            "paper_bottom": [(935, 718), (530, 630)],
            "band_top": [(940, 102)],
            "col_edge": [(939, 40), (936, 700)],
            "left_edge": [(47, 230), (12, 690)],
        },
    },
    "lounge2.jpg": {
        "points": {"paper_tl": (178, 128), "paper_bl": (190, 316), "band_tl": (172, 48)},
        "lines": {
            "paper_top": [(830, 78)],
            "paper_bottom": [(808, 358)],
            "col_edge": [(834, 10), (800, 530)],
            "col_floor": [(805, 550), (1000, 578)],
            "wall_floor": [(330, 438), (700, 491)],
            "left_edge": [(101, 40), (113, 380)],
        },
        "table": {"top": (232, 462, 433, 500), "base": (285, 432, 610, 690)},  # x min, x max, y min, y max
    },
    "lounge3.jpg": {
        "points": {"paper_br": (900, 207), "paper_bl": (116, 215)},
        "lines": {"wall_floor": [(330, 343), (860, 350)]},
        "table": {"top": (433, 700, 333, 418)},
    },
}


def model_points(d):
    w = d["width"]
    return {
        "paper_tl": (-w, 0, d["paper_z"]),
        "paper_bl": (-w, 0, d["wainscot_z"]),
        "paper_br": (0, 0, d["wainscot_z"]),
        "band_tl": (-w, 0, d["band_z"]),
    }


def model_lines(d):
    """name: (point on the line, direction)."""
    return {
        "paper_top": ((0, 0, d["paper_z"]), (1, 0, 0)),
        "paper_bottom": ((0, 0, d["wainscot_z"]), (1, 0, 0)),
        "band_top": ((0, 0, d["band_z"]), (1, 0, 0)),
        "wall_floor": ((0, 0, 0), (1, 0, 0)),
        "col_edge": ((0, -d["col_depth"], 0), (0, 0, 1)),
        "col_floor": ((0, -d["col_depth"], 0), (1, 0, 0)),
        "left_edge": ((-d["width"], -d["left_depth"], 0), (0, 0, 1)),
    }


def camera(d, pose):
    k = np.array([[d["focal"], 0, W / 2], [0, d["focal"], H / 2], [0, 0, 1]])
    return k, pose[:3], pose[3:]


def project(cam, pts):
    k, r, t = cam
    return cv2.projectPoints(np.asarray(pts, float).reshape(-1, 3), r, t, k, None)[0].reshape(-1, 2)


def circle(cx, cy, z, r, n=96):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[cx + r * np.cos(a), cy + r * np.sin(a), np.full(n, z)]


def line_dist(cam, p0, direction, px):
    """Pixel distance from `px` to the projected 3D line."""
    a, b = project(cam, [p0, np.add(p0, np.multiply(direction, 5.0))])
    n = np.array([b[1] - a[1], a[0] - b[0]]) / np.linalg.norm(b - a)
    return float(np.dot(np.subtract(px, a), n))


def extremes(cam, pts):
    uv = project(cam, pts)
    return np.r_[uv[:, 0].min(), uv[:, 0].max(), uv[:, 1].min(), uv[:, 1].max()]


def unpack(x):
    d = dict(zip(DIMS, x[: len(DIMS)]))
    poses = {name: x[len(DIMS) + 6 * i : len(DIMS) + 6 * i + 6] for i, name in enumerate(PHOTOS)}
    return d, poses


def residuals(x, report=None):
    d, poses = unpack(x)
    res = [(d[k] - v) / s for k, (v, s) in DIMS.items()]
    pts, lines = model_points(d), model_lines(d)
    for name, obs in PHOTOS.items():
        cam = camera(d, poses[name])
        for key, px in obs["points"].items():
            e = project(cam, [pts[key]])[0] - px
            res += list(e)
            if report is not None:
                report.append((name, key, float(np.hypot(*e))))
        for key, pxs in obs["lines"].items():
            for px in pxs:
                e = line_dist(cam, *lines[key], px)
                res.append(e)
                if report is not None:
                    report.append((name, key, abs(e)))
        for key, box in obs.get("table", {}).items():
            z, r = (TABLE_Z, d["table_r"]) if key == "top" else (0.0, d["base_r"])
            e = extremes(cam, circle(d["table_x"], d["table_y"], z, r)) - box
            res += list(e)
            if report is not None:
                report.append((name, f"table_{key}", float(np.abs(e).max())))
    return np.array(res)


def initial_pose(d, obs):
    """PnP on the wallpaper corners (the hidden right ones taken at the column edge) for a starting pose."""
    pts, pix = [], []
    m = model_points(d)
    for key, px in obs["points"].items():
        pts.append(m[key])
        pix.append(px)
    for key, z in (("paper_top", d["paper_z"]), ("paper_bottom", d["wainscot_z"])):
        if key in obs["lines"]:
            pts.append((0, 0, z))
            pix.append(obs["lines"][key][0])
    if len(pts) < 4:  # lounge3: the wall's floor line, its ends taken at the alcove's sides
        (a, b) = obs["lines"]["wall_floor"]
        pts += [(-d["width"] * 0.75, 0, 0), (-0.05, 0, 0)]
        pix += [a, b]
    k = np.array([[d["focal"], 0, W / 2], [0, d["focal"], H / 2], [0, 0, 1]])
    ok, r, t = cv2.solvePnP(np.array(pts, float), np.array(pix, float), k, None, flags=cv2.SOLVEPNP_SQPNP)
    assert ok
    return np.r_[r.ravel(), t.ravel()]


def floor_point(cam, px):
    """Back-project a pixel onto the floor (z = 0)."""
    k, r, t = cam
    rot = cv2.Rodrigues(np.asarray(r))[0]
    ray = rot.T @ np.linalg.solve(k, [px[0], px[1], 1.0])
    origin = -rot.T @ np.asarray(t)
    s = -origin[2] / ray[2]
    return origin + s * ray


def main():
    d0 = {k: v for k, (v, _) in DIMS.items()}
    x0 = np.r_[list(d0.values()), *[initial_pose(d0, obs) for obs in PHOTOS.values()]]
    fit = least_squares(residuals, x0, loss="soft_l1", f_scale=3.0)
    d, poses = unpack(fit.x)
    report = []
    residuals(fit.x, report)
    for name, key, e in report:
        print(f"  {name:12s} {key:14s} {e:5.1f} px")
    cams = {}
    for name in PHOTOS:
        k, r, t = camera(d, poses[name])
        rot = cv2.Rodrigues(r)[0]
        centre = -rot.T @ t
        cams[name] = {"rvec": [round(v, 5) for v in r], "tvec": [round(v, 4) for v in t], "centre": centre.round(3)}
        print(f"{name}: camera at {centre.round(2)}")
    for k, v in d.items():
        print(f"{k:12s} {v:7.3f}")
    out = {
        "focal_px": round(d["focal"], 1),
        "table_z": TABLE_Z,
        "dims": {k: round(float(v), 3) for k, v in d.items() if k != "focal"},
        "cameras": {n: {k: np.asarray(v).tolist() for k, v in c.items()} for n, c in cams.items()},
        "max_px": round(max(e for _, _, e in report), 1),
    }
    (HERE / "fit_lounge.json").write_text(json.dumps(out, indent=2) + "\n")


if __name__ == "__main__":
    main()
