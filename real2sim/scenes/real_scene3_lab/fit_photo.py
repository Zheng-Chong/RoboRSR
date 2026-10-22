"""Fit the four phone camera poses and the unmeasured table dimensions to hand-picked pixels in reference/p1-p4.jpg.

Table frame: Z up, meters, origin on the floor under the tabletop centre, +X along the long side, +Y towards the
modesty panel (the robot stands on the open -Y side). p1 is taken from the panel side (+Y), p3 from the robot side
(-Y), p2 from the +X end, p4 from the -X end; the grommet box sits near the +Y edge, mid-length.

Nothing in the photos is metric, so absolute scale comes from the tabletop height prior (0.75 m, the standard
folding training table). "points" are corners; "lines" are pixels on a model edge whose ends are hidden (the
vertical leg columns). All four photos share the focal length (same phone, same crop). Writes fit_photo.json for
build_scene.py.
"""

import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
W, H = 1024, 768

# name: (prior value, prior sigma); loose except top_z, which fixes the scale.
DIMS = {
    "length": (1.40, 0.40),
    "width": (0.70, 0.20),
    "top_z": (0.75, 0.015),
    "thick": (0.025, 0.02),
    "grom_x": (0.0, 0.2),
    "grom_y": (0.20, 0.15),
    "grom_l": (0.25, 0.10),
    "grom_w": (0.08, 0.05),
    "panel_x": (0.55, 0.20),  # half length
    "panel_y": (0.25, 0.10),
    "panel_z": (0.50, 0.15),  # bottom edge height
    "col_x": (0.55, 0.20),  # leg column axis; the columns are vertical (they only look splayed in perspective)
    "col_y": (0.0, 0.10),
    "caster_x": (0.55, 0.20),
    "caster_y": (0.28, 0.10),
}
# Pixels picked on 4-6x zoomed, gridded crops. Corner names: <part>_<sx><sy>, sx/sy in {p, m} = sign of x/y.
PHOTOS = {
    "p1.jpg": {
        "points": {
            "top_pm": (236, 241),
            "top_mm": (806, 246),
            "top_pp": (120, 407),
            "top_mp": (931, 412),
            "edge_pp": (122, 418),
            "edge_mp": (930, 420),
            "grom_pm": (457, 354),
            "grom_mm": (597, 354),
            "grom_mp": (599, 387),
            "grom_pp": (455, 387),
            "panel_p": (212, 521),
            "panel_m": (830, 520),
            "caster_pp": (250, 644),
            "caster_mp": (782, 648),
        },
        "lines": {},  # the slanted strips under the panel are the V feet, not the columns
    },
    "p2.jpg": {
        "points": {
            "top_mm": (423, 176),
            "top_mp": (629, 188),
            "top_pm": (330, 367),
            "top_pp": (715, 389),
            "edge_pm": (332, 380),
            "edge_pp": (713, 400),
            "grom_mm": (585, 240),
            "grom_mp": (627, 241),
            "grom_pp": (637.5, 275),
            "grom_pm": (590, 271),
            "panel_p": (655, 488),
            "caster_pm": (395, 665),
            "caster_pp": (602, 650),
        },
        "lines": {
            "col_p_hi": ((561, 396), "col_p_lo", "col_p_hi"),
            "col_p_lo": ((547, 600), "col_p_lo", "col_p_hi"),
        },
    },
    "p3.jpg": {
        "points": {
            "top_mp": (276, 227),
            "top_pp": (776, 230),
            "top_mm": (190, 320),
            "top_pm": (887, 321),
            "edge_mm": (190, 330),
            "edge_pm": (887, 334),
            "grom_mp": (480, 236),
            "grom_pp": (566, 236),
            "grom_pm": (566, 248),
            "grom_mm": (480, 248),
            "caster_mm": (283, 588),
            "caster_pm": (800, 588),
        },
        "lines": {  # column centre lines above the V feet
            "col_m_hi": ((285, 360), "col_m_lo", "col_m_hi"),
            "col_m_lo": ((280, 470), "col_m_lo", "col_m_hi"),
            "col_p_hi": ((768, 345), "col_p_lo", "col_p_hi"),
            "col_p_lo": ((764, 430), "col_p_lo", "col_p_hi"),
        },
    },
    "p4.jpg": {
        "points": {
            "top_pp": (400, 183),
            "top_pm": (600, 176),
            "top_mp": (322, 366),
            "top_mm": (689, 352),
            "edge_mp": (323, 376),
            "edge_mm": (688, 360),
            "grom_pp": (404, 233),
            "grom_pm": (444.7, 232),
            "grom_mm": (439, 261.7),
            "grom_mp": (394, 264.7),
            "panel_m": (377, 466),
            "caster_mp": (415, 625),
            "caster_mm": (608, 617),
        },
        "lines": {
            "col_m_hi": ((468, 385), "col_m_lo", "col_m_hi"),
            "col_m_lo": ((475, 575), "col_m_lo", "col_m_hi"),
        },
    },
}
SIGMA = {"caster": 8.0, "panel": 5.0}  # px; dark, blurry features. Everything else ~2 px.


def model_points(d):
    out = {}
    for sx, kx in ((1, "p"), (-1, "m")):
        for sy, ky in ((1, "p"), (-1, "m")):
            out[f"top_{kx}{ky}"] = (sx * d["length"] / 2, sy * d["width"] / 2, d["top_z"])
            out[f"edge_{kx}{ky}"] = (sx * d["length"] / 2, sy * d["width"] / 2, d["top_z"] - d["thick"])
            out[f"grom_{kx}{ky}"] = (d["grom_x"] + sx * d["grom_l"] / 2, d["grom_y"] + sy * d["grom_w"] / 2, d["top_z"])
            out[f"caster_{kx}{ky}"] = (sx * d["caster_x"], sy * d["caster_y"], 0.0)
        out[f"panel_{kx}"] = (sx * d["panel_x"], d["panel_y"], d["panel_z"])
        out[f"col_{kx}_lo"] = (sx * d["col_x"], d["col_y"], 0.0)
        out[f"col_{kx}_hi"] = (sx * d["col_x"], d["col_y"], d["top_z"])
    return out


N = len(DIMS)


def unpack(x):
    d = dict(zip(DIMS, x[:N]))
    cams = [(x[N + 6 * i : N + 6 * i + 3], x[N + 6 * i + 3 : N + 6 * i + 6]) for i in range(len(PHOTOS))]
    return d, cams, x[-1]


def project(d, cam, f, names):
    pts = model_points(d)
    k = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1]])
    img, _ = cv2.projectPoints(np.array([pts[n] for n in names], np.float64), *cam, k, None)
    return img.reshape(-1, 2)


def point_errors(d, cam, f, photo):
    names = list(photo["points"])
    return np.linalg.norm(project(d, cam, f, names) - np.array([photo["points"][n] for n in names]), axis=1)


def line_errors(d, cam, f, photo):
    """Signed pixel distance of each line pixel to its projected model edge."""
    out = []
    for px, a, b in photo["lines"].values():
        pa, pb = project(d, cam, f, [a, b])
        n = np.array([pa[1] - pb[1], pb[0] - pa[0]])
        out.append(float(np.dot(np.asarray(px) - pa, n / np.linalg.norm(n))))
    return np.array(out)


def residuals(x):
    d, cams, f = unpack(x)
    r = []
    for cam, photo in zip(cams, PHOTOS.values()):
        names = list(photo["points"])
        sig = np.array([SIGMA.get(n.split("_")[0], 2.0) for n in names])[:, None]
        r.append(((project(d, cam, f, names) - np.array([photo["points"][n] for n in names])) / sig).ravel())
        r.append(line_errors(d, cam, f, photo) / 3.0)
    prior = [(d[n] - m) / s for n, (m, s) in DIMS.items()]
    return np.concatenate([*r, prior, [(f - 800) / 300]])


def initial_pose(d0, photo, k):
    names = list(photo["points"])
    obj = np.array([model_points(d0)[n] for n in names], np.float64)
    img = np.array([photo["points"][n] for n in names], np.float64)
    _, rvec, tvec = cv2.solvePnP(obj, img, k, None, flags=cv2.SOLVEPNP_SQPNP)
    return [*rvec.ravel(), *tvec.ravel()]


def main():
    d0 = {n: m for n, (m, _) in DIMS.items()}
    f0 = 800.0
    k = np.array([[f0, 0, W / 2], [0, f0, H / 2], [0, 0, 1]])
    x0 = list(d0.values())
    for photo in PHOTOS.values():
        x0 += initial_pose(d0, photo, k)
    sol = least_squares(residuals, np.r_[x0, f0], loss="soft_l1", f_scale=2.0)
    d, cams, f = unpack(sol.x)
    out = dict(dims={n: round(float(v), 3) for n, v in d.items()}, focal_px=round(float(f), 1))
    out["hfov_deg"] = round(float(np.degrees(2 * np.arctan(W / 2 / f))), 1)
    for (rvec, tvec), (name, photo) in zip(cams, PHOTOS.items()):
        rmat = cv2.Rodrigues(rvec)[0]
        out[name] = dict(
            camera_eye=[round(float(v), 3) for v in -rmat.T @ tvec],
            camera_forward=[round(float(v), 3) for v in rmat.T @ [0, 0, 1]],
            rvec=[float(v) for v in rvec],
            tvec=[float(v) for v in tvec],
            point_px={n: round(float(e), 1) for n, e in zip(photo["points"], point_errors(d, (rvec, tvec), f, photo))},
            line_px={
                n: round(abs(float(e)), 1) for n, e in zip(photo["lines"], line_errors(d, (rvec, tvec), f, photo))
            },
        )
    (HERE / "fit_photo.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
