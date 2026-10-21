"""Fit the phone camera poses and the unmeasured sizes (leg columns, feet, cabinet position) to hand-picked pixels in
reference/IMG_49*.jpeg, with the tabletop fixed to the tape measurements (registration.json).

Table frame: Z up, meters, origin on the floor under the tabletop centre, +X along the long side (the robot's right),
+Y towards the wall (the robot stands on the -Y side). The table's back edge touches the wall (registered clearance
0 cm), the metal cabinet stands against the same wall on the +X side. IMG_4913 is taken from the front left,
IMG_4914 from the front, IMG_4916 close and square-on from the front, IMG_4917 from the -X end, IMG_4918 (portrait)
from the front right; IMG_4915 repeats IMG_4914 and is not used. The cabinet edges in IMG_4916 (at the frame edge)
and its bottom edge in IMG_4914 (behind the bag) were picked but left out: they disagreed by 8-100 px.

No ChArUco board was measured (the board lying on the table has no recorded square size or position), so scale comes
only from the tape: length, width, height and thickness of the tabletop and the cabinet size are fixed, not fitted.
Pixels are in the photo downscaled to 1024 px on the long side, EXIF orientation applied. "points" are corners;
"lines" are pixels on a model edge (rounded table corners, occluded ends). Writes fit_photo.json for build_scene.py.
"""

import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
REG = json.loads((HERE / "registration.json").read_text())
M = {k: v["value_m"] for k, v in REG["measurements"].items()}
FIXED = dict(
    length=M["table_length"],
    width=M["table_width"],
    top_z=M["table_top_z"],
    thick=REG["table"]["thickness_cm"] / 100,
    wall_y=M["table_width"] / 2,  # back clearance 0 cm
    cab_w=0.75,  # registered cabinet 75 x 120 x 45 cm (width, height, depth)
    cab_h=1.20,
    cab_d=0.45,
)
# name: (prior value, prior sigma)
DIMS = {
    "col_x": (0.45, 0.10),  # leg column axis, |x|
    "col_y": (0.0, 0.15),
    "foot_front": (-0.28, 0.10),  # y of the foot's front tip (on the floor)
    "foot_back": (0.27, 0.10),  # y of the foot's back end
    "cab_x0": (0.90, 0.40),  # cabinet's left (-X) side
}
FOCAL_PRIOR = (769.0, 40.0)  # px at 1024 px: iPhone Air main camera, 26 mm equivalent (EXIF), 4:3
SIZE = {"IMG_4918.jpeg": (768, 1024)}  # (w, h); the others are landscape 1024 x 768

L_ = "lines"
PHOTOS = {
    "IMG_4913.jpeg": {
        "points": {
            "foot_m_front": (345, 637),
            "cab_top_mp": (670, 242),
            "cab_top_mm": (750, 249),
            "cab_top_pm": (883, 245),
        },
        L_: {
            "back_1": ((300, 377), "top_mp", "top_pp"),
            "back_2": ((340, 371), "top_mp", "top_pp"),
            "left_1": ((275, 400), "top_mm", "top_mp"),
            "left_2": ((291, 420), "top_mm", "top_mp"),
            "front_1": ((565, 392), "top_mm", "top_pm"),
            "front_2": ((610, 386.5), "top_mm", "top_pm"),
            "right_1": ((580, 360), "top_pm", "top_pp"),
            "right_2": ((615, 375), "top_pm", "top_pp"),
            "floor_1": ((290, 566), "wall_m", "wall_p"),
            "floor_2": ((365, 549), "wall_m", "wall_p"),
            "colm_1": ((325, 530), "col_m_lo", "col_m_hi"),
            "colm_2": ((330, 585), "col_m_lo", "col_m_hi"),
        },
    },
    "IMG_4914.jpeg": {
        "points": {"foot_m_front": (330.4, 568), "foot_p_front": (588, 607), "cab_top_mm": (882, 250)},
        L_: {
            "back_1": ((560, 343.5), "top_mp", "top_pp"),
            "back_2": ((620, 349), "top_mp", "top_pp"),
            "left_1": ((290, 364), "top_mm", "top_mp"),
            "left_2": ((330, 347), "top_mm", "top_mp"),
            "front_1": ((300, 379), "top_mm", "top_pm"),
            "front_2": ((350, 383), "top_mm", "top_pm"),
            "front_3": ((560, 401.5), "top_mm", "top_pm"),
            "front_4": ((600, 403.5), "top_mm", "top_pm"),
            "right_1": ((650, 370), "top_pm", "top_pp"),
            "right_2": ((639, 390), "top_pm", "top_pp"),
            "colm_1": ((361, 435), "col_m_lo", "col_m_hi"),
            "colm_2": ((366.5, 515), "col_m_lo", "col_m_hi"),
            "colp_1": ((605, 450), "col_p_lo", "col_p_hi"),
            "colp_2": ((606, 560), "col_p_lo", "col_p_hi"),
            "cabfl_1": ((881, 262), "cab_top_mm", "cab_bot_mm"),
            "cabfl_2": ((877, 330), "cab_top_mm", "cab_bot_mm"),
        },
    },
    "IMG_4916.jpeg": {
        "points": {
            "foot_m_front": (317.8, 620.7),
            "foot_p_front": (660, 620.7),
        },
        L_: {
            "back_1": ((291, 288.9), "top_mp", "top_pp"),
            "back_2": ((330, 288.2), "top_mp", "top_pp"),
            "back_3": ((657.8, 284.3), "top_mp", "top_pp"),
            "back_4": ((689, 284.6), "top_mp", "top_pp"),
            "left_1": ((259.8, 320), "top_mm", "top_mp"),
            "left_2": ((228.6, 379.2), "top_mm", "top_mp"),
            "front_1": ((239, 425.4), "top_mm", "top_pm"),
            "front_2": ((330, 424.1), "top_mm", "top_pm"),
            "front_3": ((689, 419.5), "top_mm", "top_pm"),
            "front_4": ((728, 419.5), "top_mm", "top_pm"),
            "right_1": ((711, 294.7), "top_pm", "top_pp"),
            "right_2": ((728, 337.6), "top_pm", "top_pp"),
            "right_3": ((748.8, 384.4), "top_pm", "top_pp"),
            "floor_1": ((385, 486), "wall_m", "wall_p"),
            "floor_2": ((608, 486), "wall_m", "wall_p"),
            "colm_1": ((320, 444), "col_m_lo", "col_m_hi"),
            "colm_2": ((334.6, 541), "col_m_lo", "col_m_hi"),
            "colp_1": ((664.6, 444), "col_p_lo", "col_p_hi"),
            "colp_2": ((649.7, 541), "col_p_lo", "col_p_hi"),
        },
    },
    "IMG_4917.jpeg": {
        "points": {"foot_p_back": (510.4, 560.8)},
        L_: {
            "back_1": ((100, 470.4), "top_mp", "top_pp"),
            "back_2": ((190.4, 441.6), "top_mp", "top_pp"),
            "back_3": ((300, 401.2), "top_mp", "top_pp"),
            "back_4": ((389.6, 370), "top_mp", "top_pp"),
            "back_5": ((460, 347.6), "top_mp", "top_pp"),
            "left_1": ((60, 510.4), "top_mm", "top_mp"),
            "left_2": ((99, 552), "top_mm", "top_mp"),
            "front_1": ((259.6, 683.2), "top_mm", "top_pm"),
            "front_2": ((362, 620.8), "top_mm", "top_pm"),
            "colp_1": ((580, 516), "col_p_lo", "col_p_hi"),
            "colp_2": ((579.2, 596), "col_p_lo", "col_p_hi"),
        },
    },
    "IMG_4918.jpeg": {
        "points": {"foot_p_front": (372, 727), "foot_p_back": (471, 662), "cab_top_mm": (634.6, 363.5)},
        L_: {
            "back_1": ((436, 448.4), "top_mp", "top_pp"),
            "back_2": ((492, 459.6), "top_mp", "top_pp"),
            "right_1": ((443, 488.3), "top_pm", "top_pp"),
            "right_2": ((499, 477.1), "top_pm", "top_pp"),
            "front_1": ((380, 513), "top_mm", "top_pm"),
            "colp_1": ((423, 533), "col_p_lo", "col_p_hi"),
            "colp_2": ((421, 666), "col_p_lo", "col_p_hi"),
            "cabfl_1": ((633.5, 385), "cab_top_mm", "cab_bot_mm"),
            "cabfl_2": ((631, 415), "cab_top_mm", "cab_bot_mm"),
        },
    },
}
SIGMA = {"foot": 4.0, "cab": 4.0}  # px; caps and the cabinet lid are blurry. Everything else ~2 px.
LINE_SIGMA = {"floor": 4.0, "colm": 3.0, "colp": 3.0, "cabfl": 3.0}
# Rough phone positions to start from (eye, look-at), scene frame.
START = {
    "IMG_4913.jpeg": ((-1.3, -2.6, 1.5), (0.3, 0.0, 0.6)),
    "IMG_4914.jpeg": ((0.3, -2.8, 1.5), (0.2, 0.0, 0.6)),
    "IMG_4916.jpeg": ((0.0, -1.6, 1.7), (0.0, 0.0, 0.6)),
    "IMG_4917.jpeg": ((-1.2, -1.0, 1.5), (0.8, 0.2, 0.6)),
    "IMG_4918.jpeg": ((1.2, -1.8, 1.5), (-0.2, 0.2, 0.6)),
}


def model_points(d):
    d = {**FIXED, **d}
    hl, hw, z = d["length"] / 2, d["width"] / 2, d["top_z"]
    out = {}
    for sx, kx in ((1, "p"), (-1, "m")):
        for sy, ky in ((1, "p"), (-1, "m")):
            out[f"top_{kx}{ky}"] = (sx * hl, sy * hw, z)
        out[f"col_{kx}_lo"] = (sx * d["col_x"], d["col_y"], 0.0)
        out[f"col_{kx}_hi"] = (sx * d["col_x"], d["col_y"], z)
        out[f"foot_{kx}_front"] = (sx * d["col_x"], d["foot_front"], 0.0)
        out[f"foot_{kx}_back"] = (sx * d["col_x"], d["foot_back"], 0.035)  # top of the end cap
        out[f"wall_{kx}"] = (sx * 3.0, d["wall_y"], 0.0)
    cy0, cy1 = d["wall_y"] - d["cab_d"], d["wall_y"]
    for kx, x in (("m", d["cab_x0"]), ("p", d["cab_x0"] + d["cab_w"])):
        for ky, y in (("m", cy0), ("p", cy1)):
            out[f"cab_top_{kx}{ky}"] = (x, y, d["cab_h"])
            out[f"cab_bot_{kx}{ky}"] = (x, y, 0.0)
    return out


N = len(DIMS)


def unpack(x):
    d = dict(zip(DIMS, x[:N]))
    cams = [(x[N + 6 * i : N + 6 * i + 3], x[N + 6 * i + 3 : N + 6 * i + 6]) for i in range(len(PHOTOS))]
    return d, cams, x[-1]


def intrinsics(name, f):
    w, h = SIZE.get(name, (1024, 768))
    return np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]])


def project(d, cam, f, name, names):
    pts = model_points(d)
    img, _ = cv2.projectPoints(np.array([pts[n] for n in names], np.float64), *cam, intrinsics(name, f), None)
    return img.reshape(-1, 2)


def point_errors(d, cam, f, name):
    photo = PHOTOS[name]
    names = list(photo["points"])
    if not names:
        return np.zeros(0)
    return np.linalg.norm(project(d, cam, f, name, names) - np.array([photo["points"][n] for n in names]), axis=1)


def line_errors(d, cam, f, name):
    """Signed pixel distance of each line pixel to its projected model edge."""
    out = []
    for px, a, b in PHOTOS[name][L_].values():
        pa, pb = project(d, cam, f, name, [a, b])
        n = np.array([pa[1] - pb[1], pb[0] - pa[0]])
        out.append(float(np.dot(np.asarray(px) - pa, n / np.linalg.norm(n))))
    return np.array(out)


def residuals(x):
    d, cams, f = unpack(x)
    r = []
    for cam, (name, photo) in zip(cams, PHOTOS.items()):
        names = list(photo["points"])
        if names:
            sig = np.array([SIGMA.get(n.split("_")[0], 2.0) for n in names])[:, None]
            r.append(((project(d, cam, f, name, names) - np.array([photo["points"][n] for n in names])) / sig).ravel())
        lsig = np.array([LINE_SIGMA.get(n.split("_")[0], 2.0) for n in photo[L_]])
        r.append(line_errors(d, cam, f, name) / lsig)
    prior = [(d[n] - m) / s for n, (m, s) in DIMS.items()]
    return np.concatenate([*r, prior, [(f - FOCAL_PRIOR[0]) / FOCAL_PRIOR[1]]])


def start_pose(eye, target):
    eye, fwd = np.array(eye, float), np.subtract(target, eye)
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1])
    right /= np.linalg.norm(right)
    rmat = np.stack([right, np.cross(fwd, right), fwd])  # rows: camera x right, y down, z forward
    return [*cv2.Rodrigues(rmat)[0].ravel(), *(-rmat @ eye)]


def main():
    x0 = [m for m, _ in DIMS.values()]
    for name in PHOTOS:
        x0 += start_pose(*START[name])
    sol = least_squares(residuals, np.r_[x0, FOCAL_PRIOR[0]], loss="soft_l1", f_scale=2.0, max_nfev=20000)
    d, cams, f = unpack(sol.x)
    out = dict(
        fixed=FIXED,
        dims={n: round(float(v), 3) for n, v in d.items()},
        focal_px=round(float(f), 1),
        hfov_deg_landscape=round(float(np.degrees(2 * np.arctan(512 / f))), 1),
    )
    for (rvec, tvec), name in zip(cams, PHOTOS):
        rmat = cv2.Rodrigues(rvec)[0]
        out[name] = dict(
            size=list(SIZE.get(name, (1024, 768))),
            camera_eye=[round(float(v), 3) for v in -rmat.T @ tvec],
            camera_forward=[round(float(v), 3) for v in rmat.T @ [0, 0, 1]],
            rvec=[float(v) for v in rvec],
            tvec=[float(v) for v in tvec],
            point_px={
                n: round(float(e), 1) for n, e in zip(PHOTOS[name]["points"], point_errors(d, (rvec, tvec), f, name))
            },
            line_px={
                n: round(abs(float(e)), 1) for n, e in zip(PHOTOS[name][L_], line_errors(d, (rvec, tvec), f, name))
            },
        )
    (HERE / "fit_photo.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
