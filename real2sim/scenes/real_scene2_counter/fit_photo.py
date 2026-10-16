"""Fit the two phone camera poses and the unmeasured counter dimensions to hand-picked pixels in reference/photo.jpg
(from the front right) and reference/photo2.jpg (from the front left).

Absolute scale comes from the microwave (assumed 0.485 x 0.29 m front, the common Galanz 23 l size). photo.jpg alone
cannot see the counter's right end (the right column hides it), so its length was unobservable; photo2.jpg sees both
ends, the sink and the faucet from the other side. Both photos share the focal length (same phone, same crop).
The shelves run from the left column (both photos show them meeting its side face). "points" are corners; "lines"
are pixels on a model edge whose ends are hidden or not distinct (column edges, occluded shelf ends, poster sides).
Prints the fitted dimensions, both camera poses and per-pixel errors; writes fit_photo.json, read by build_scene.py.
"""

import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
W, H = 1024, 768
WALL_Y, MW_W, MW_H, MW_FEET = 0.30, 0.485, 0.29, 0.012

# name: (prior value, prior sigma). Wall at y = 0.30 and counter centre at x = 0 fix the frame; the priors are loose
# and only keep unobserved directions in place.
DIMS = {
    "length": (2.30, 0.50),
    "top_z": (0.85, 0.05),
    "depth": (0.60, 0.05),
    "slab": (0.04, 0.02),
    "mw_front": (-0.12, 0.10),
    "mw_x0": (0.60, 0.30),
    "mw_d": (0.40, 0.05),
    "shelf_x1": (1.00, 0.30),
    "shelf_depth": (0.25, 0.03),
    "shelf_z1": (1.48, 0.15),
    "shelf_z2": (1.76, 0.15),
    "left_front": (-0.28, 0.10),
    "right_front": (-0.32, 0.10),
    "sink_cx": (-0.10, 0.30),
    "sink_cy": (-0.03, 0.10),
    "sink_sx": (0.50, 0.10),
    "sink_sy": (0.38, 0.10),
    "faucet_x": (0.00, 0.40),
    "faucet_y": (0.20, 0.10),
    "poster_x0": (0.30, 0.40),
    "poster_x1": (1.00, 0.30),
    "poster_z0": (1.86, 0.20),
    "poster_z1": (2.60, 0.30),
    "alcove_top": (2.60, 0.30),
}
# Pixels picked on 3-4x zoomed, gridded crops.
PHOTOS = {
    "photo.jpg": {
        "points": {
            "counter_front_left_top": (181, 471),
            "counter_front_left_bottom": (183, 488),
            "counter_back_left": (318, 437),
            "mw_front_left_top": (613, 474),
            "mw_front_left_bottom": (618, 585),
            "sink_front_left": (310, 513),
            "sink_front_right": (412, 550),
            "sink_back_left": (400, 483),
            "sink_back_right": (502, 513),
            "faucet": (465, 512),
            "poster_bottom_right": (798, 100),
        },
        "lines": {
            "counter_front_top_right": ((803, 703), "counter_front_left_top", "counter_front_right_top"),
            "counter_front_bottom_right": ((803, 752), "counter_front_left_bottom", "counter_front_right_bottom"),
            "mw_front_top_right": ((800, 490), "mw_front_left_top", "mw_front_right_top"),
            "mw_front_bottom_right": ((810, 655), "mw_front_left_bottom", "mw_front_right_bottom"),
            "left_col_edge_high": ((150, 250), "left_col_edge_low", "left_col_edge_top"),
            "left_col_edge_low": ((182, 467), "left_col_edge_low", "left_col_edge_top"),
            "right_col_edge_high": ((812, 320), "right_col_edge_low", "right_col_edge_top"),
            "right_col_edge_low": ((811, 690), "right_col_edge_low", "right_col_edge_top"),
            "shelf1_left": ((277, 284), "shelf1_front_left", "shelf1_front_right"),
            "shelf1_right": ((750, 281), "shelf1_front_left", "shelf1_front_right"),
            "shelf2_left": ((265, 210), "shelf2_front_left", "shelf2_front_right"),
            "shelf2_right": ((750, 133), "shelf2_front_left", "shelf2_front_right"),
            "poster_bottom": ((600, 132), "poster_bottom_left", "poster_bottom_right"),
            "poster_right": ((799, 50), "poster_bottom_right", "poster_top_right"),
        },
    },
    "photo2.jpg": {
        "points": {
            "counter_front_left_top": (346, 642),
            "counter_front_left_bottom": (349, 663),
            "counter_front_right_top": (832.5, 477.5),
            "counter_front_right_bottom": (833, 495),
            "left_col_edge_counter": (339, 626),
            "mw_front_left_top": (722.7, 419.3),
            "mw_front_left_bottom": (722.7, 494),
            "mw_front_right_top": (787.7, 418.3),
            "mw_back_left_top": (660, 420.7),
            "mw_back_left_bottom": (660.7, 481.7),
            "shelf1_front_right": (731.7, 321),
            "shelf2_front_right": (735, 243),
            "sink_front_left": (540, 566.7),
            "sink_front_right": (641.7, 535),
            "sink_back_left": (461.7, 542.7),
            "sink_back_right": (563.3, 514),
            "faucet": (540, 520),
            "poster_bottom_left": (593.3, 233.3),
            "poster_bottom_right": (686.7, 246.7),
            "poster_top_right": (689, 22),
            "alcove_corner_top": (718, 29),
        },
        "lines": {
            "shelf1_left": ((334, 350), "shelf1_front_left", "shelf1_front_right"),
            "shelf2_left": ((331, 236), "shelf2_front_left", "shelf2_front_right"),
            "left_col_edge_high": ((331, 250), "left_col_edge_low", "left_col_edge_top"),
            "left_col_edge_low": ((337.5, 600), "left_col_edge_low", "left_col_edge_top"),
            "right_col_edge_high": ((837, 10), "right_col_edge_low", "right_col_edge_top"),
            "right_col_edge_low": ((832, 333), "right_col_edge_low", "right_col_edge_top"),
            "alcove_corner_low": ((715, 333), "alcove_corner_low", "alcove_corner_top"),
            "poster_left": ((594, 100), "poster_bottom_left", "poster_top_left"),
            "poster_right": ((688, 120), "poster_bottom_right", "poster_top_right"),
        },
    },
}


def model_points(d):
    hl, fy, sy, t = d["length"] / 2, WALL_Y - d["depth"], WALL_Y - d["shelf_depth"], d["top_z"]
    sx0, sx1 = d["sink_cx"] - d["sink_sx"] / 2, d["sink_cx"] + d["sink_sx"] / 2
    sf, sb = d["sink_cy"] - d["sink_sy"] / 2, d["sink_cy"] + d["sink_sy"] / 2
    mx0, mx1, mf, mb = d["mw_x0"], d["mw_x0"] + MW_W, d["mw_front"], d["mw_front"] + d["mw_d"]
    px0, px1, pz0, pz1 = d["poster_x0"], d["poster_x1"], d["poster_z0"], d["poster_z1"]
    return {
        "counter_front_left_top": (-hl, fy, t),
        "counter_front_left_bottom": (-hl, fy, t - d["slab"]),
        "counter_back_left": (-hl, WALL_Y, t),
        "counter_front_right_top": (hl, fy, t),
        "counter_front_right_bottom": (hl, fy, t - d["slab"]),
        "left_col_edge_counter": (-hl, d["left_front"], t),
        "left_col_edge_low": (-hl, d["left_front"], 0.0),
        "left_col_edge_top": (-hl, d["left_front"], 2.0),
        "right_col_edge_low": (hl, d["right_front"], 0.0),
        "right_col_edge_top": (hl, d["right_front"], 2.0),
        "alcove_corner_low": (hl, WALL_Y, 0.0),
        "alcove_corner_top": (hl, WALL_Y, d["alcove_top"]),
        "mw_front_left_top": (mx0, mf, t + MW_H),
        "mw_front_left_bottom": (mx0, mf, t + MW_FEET),
        "mw_front_right_top": (mx1, mf, t + MW_H),
        "mw_front_right_bottom": (mx1, mf, t + MW_FEET),
        "mw_back_left_top": (mx0, mb, t + MW_H),
        "mw_back_left_bottom": (mx0, mb, t + MW_FEET),
        "shelf1_front_left": (-hl, sy, d["shelf_z1"]),
        "shelf1_front_right": (d["shelf_x1"], sy, d["shelf_z1"]),
        "shelf2_front_left": (-hl, sy, d["shelf_z2"]),
        "shelf2_front_right": (d["shelf_x1"], sy, d["shelf_z2"]),
        "sink_front_left": (sx0, sf, t),
        "sink_front_right": (sx1, sf, t),
        "sink_back_left": (sx0, sb, t),
        "sink_back_right": (sx1, sb, t),
        "faucet": (d["faucet_x"], d["faucet_y"], t),
        "poster_bottom_left": (px0, WALL_Y, pz0),
        "poster_bottom_right": (px1, WALL_Y, pz0),
        "poster_top_left": (px0, WALL_Y, pz1),
        "poster_top_right": (px1, WALL_Y, pz1),
    }


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
        r.append((project(d, cam, f, names) - np.array([photo["points"][n] for n in names])).ravel() / 2.0)
        r.append(line_errors(d, cam, f, photo) / 2.0)  # ~2 px picking error
    prior = [(d[n] - m) / s for n, (m, s) in DIMS.items()]
    return np.concatenate([*r, prior, [(f - 800) / 200]])  # iPhone main cam: f ~ 800 px


def main():
    d0 = {n: m for n, (m, _) in DIMS.items()}
    f0 = 800.0
    k = np.array([[f0, 0, W / 2], [0, f0, H / 2], [0, 0, 1]])
    x0 = list(d0.values())
    for photo in PHOTOS.values():
        names = list(photo["points"])
        obj = np.array([model_points(d0)[n] for n in names], np.float64)
        img = np.array([photo["points"][n] for n in names], np.float64)
        _, rvec, tvec = cv2.solvePnP(obj, img, k, None, flags=cv2.SOLVEPNP_ITERATIVE)
        x0 += [*rvec.ravel(), *tvec.ravel()]
    sol = least_squares(residuals, np.r_[x0, f0])
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
