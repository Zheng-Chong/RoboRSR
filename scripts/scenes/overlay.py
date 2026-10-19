"""Overlay check of a real-scene twin against real robot camera frames (scene acceptance: < 5 px in >= 3 views).

A camera's pose comes from the robot URDF (real2sim/robot/g1d_v4) at the env config's stand (root pose, lift joints;
waist and head at zero; arm joints as recorded for a wrist view), its mount from scripts/g1d/<side>_robot_config.yml
and its intrinsics from env_cfg/camera/template.py: the numbers the simulator renders with. The scene features
(tabletop corners and edges) come from the scene's usd/build_params.json, placed where scripts/g1d/build_asset.py
SCENES puts the room. Label the same features in a real 640 x 480 frame (head: left eye) taken with the robot at its
real stand; the error then covers the scene geometry, the stand and the camera together. Head plus wrist frames at a
few arm poses give the 3 views.

Usage (numpy, scipy, pyyaml; opencv for --image):
    python scripts/scenes/overlay.py --env-cfg g1d_lab [--camera cam_left_wrist --joints q.json]   # labelling guide
    python scripts/scenes/overlay.py --env-cfg g1d_lab --labels view1.json [--image view1.jpg --out check1.jpg]
A labelled view: {"camera": "cam_head", "joints": {URDF joint: rad}, "points": {"table_back_left": [u, v], ...},
"edges": {"table_front_edge": [[u, v], ...], ...}}, pixels. A corner is where its two edges' lines meet (round
corners too); an edge takes any pixels on it, scored by their distance to the projected edge line, so it counts when
its corners are out of view (the front ones usually are from the head). Record each view's max_px in the scene's
scene_spec.json "overlay" list.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from env_cfg.camera import template  # noqa: E402
from real2sim.scene3_pour_pack.motion.kinematics import G1Kinematics  # noqa: E402
from scripts.g1d.build_asset import SCENES  # noqa: E402

URDF = ROOT / "real2sim/robot/g1d_v4/g1_d_full_dex1_v4.urdf"
MOUNTS = {  # camera: (robot config declaring it, its name there)
    "cam_head": ("left", "cam_head"),
    "cam_left_wrist": ("left", "cam_wrist"),
    "cam_right_wrist": ("right", "cam_wrist"),
}
USD_TO_CV = np.diag([1.0, -1.0, -1.0, 1.0])  # USD camera (-z forward, +y up) -> OpenCV (+z forward, +y down)
LIMIT_PX = 5.0
EDGES = {
    "table_front_edge": ("table_front_left", "table_front_right"),
    "table_back_edge": ("table_back_left", "table_back_right"),
    "table_left_edge": ("table_front_left", "table_back_left"),
    "table_right_edge": ("table_front_right", "table_back_right"),
}


def load_yaml(path):
    import yaml

    return yaml.safe_load(Path(path).read_text())


def pose(position, wxyz):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([*wxyz[1:], wxyz[0]]).as_matrix()
    T[:3, 3] = position
    return T


def intrinsics(spec):
    (w, h), f = spec["resolution"], spec["focal_length"]
    fx, fy = f / spec["horizontal_aperture"] * w, f / spec["vertical_aperture"] * h
    return np.array([[fx, 0, w / 2], [0, fy, h / 2], [0, 0, 1]]), (w, h)


def sim_camera(env_cfg, name="cam_head", joints=None):
    """(K, image size, world-from-camera 4x4 in OpenCV axes) of a simulated camera at this env config's stand;
    `joints` ({URDF joint name: position}, e.g. a recorded arm state) place the wrist cameras."""
    config = load_yaml(ROOT / "env_cfg" / f"{env_cfg}.yml")["config"]
    robot = load_yaml(ROOT / "env_cfg/robot" / f"{config['robot']}.yml")["robots"][0]
    q = dict(robot.get("init_joint_pos") or {}, **(joints or {}))
    links = G1Kinematics(URDF).fk(q, base=pose(robot["default_root_pos"], robot["default_root_rot"]))
    side, declared = MOUNTS[name]
    config_file = ROOT / "scripts/g1d" / f"{side}_robot_config.yml"
    mount = next(c for c in load_yaml(config_file)["camera"] if c["name"] == declared)
    K, size = intrinsics(getattr(template, mount["type"].upper()))
    return K, size, links[mount["link"]] @ pose(mount["pos"], mount["ori"]) @ USD_TO_CV


def features(env_cfg):
    """{name: world xyz} of the tabletop corners; left/right as the robot sees them (it faces +y)."""
    scene, _, room_pos, _ = SCENES[load_yaml(ROOT / "env_cfg" / f"{env_cfg}.yml")["config_name"]]
    table = json.loads((ROOT / "real2sim/scenes" / scene / "usd/build_params.json").read_text())["table"]
    half, front = table["length"] / 2, table.get("front_y", -table["width"] / 2)
    corners = dict(front_left=(-half, front), front_right=(half, front))
    corners |= dict(back_left=(-half, front + table["width"]), back_right=(half, front + table["width"]))
    return {f"table_{k}": np.array([x, y, table["top_z"]]) + room_pos for k, (x, y) in corners.items()}


def project(K, world_from_camera, points):
    """Pixels of world points, and whether each is in front of the camera."""
    camera = (np.linalg.inv(world_from_camera) @ np.c_[points, np.ones(len(points))].T)[:3]
    uv = K @ camera
    return (uv[:2] / uv[2]).T, camera[2] > 0


def line_distance(a, b, pixels):
    """Distances (px) of pixels from the line through a and b."""
    d = (b - a) / np.linalg.norm(b - a)
    rel = np.asarray(pixels, dtype=float) - a
    return np.abs(rel[:, 0] * d[1] - rel[:, 1] * d[0])


def check(env_cfg, labels):
    """Pixel errors of the labelled corners (distance) and edge pixels (distance to the projected edge line), in the
    view of labels["camera"] (default cam_head) with labels["joints"] (URDF names) for the arm."""
    K, _, world_from_camera = sim_camera(env_cfg, labels.get("camera", "cam_head"), labels.get("joints"))
    known = features(env_cfg)
    points, edges = labels.get("points", {}), labels.get("edges", {})
    if unknown := sorted(set(points) - set(known)) + sorted(set(edges) - set(EDGES)):
        raise ValueError(f"Unknown features {unknown}; known: {sorted(known) + sorted(EDGES)}")
    uv, front = project(K, world_from_camera, np.array(list(known.values())))
    visible = dict(zip(known, front))
    if behind := [n for n in [*points, *(c for e in edges for c in EDGES[e])] if not visible[n]]:
        raise ValueError(f"{sorted(set(behind))} behind the camera")
    sim = dict(zip(known, uv))
    report = {
        n: dict(sim=sim[n].round(1).tolist(), error_px=round(float(np.linalg.norm(sim[n] - p)), 2))
        for n, p in points.items()
    }
    for name, pixels in edges.items():
        a, b = (sim[c] for c in EDGES[name])
        report[name] = dict(
            sim=[a.round(1).tolist(), b.round(1).tolist()], error_px=round(float(line_distance(a, b, pixels).max()), 2)
        )
    worst = max((r["error_px"] for r in report.values()), default=None)
    return dict(env_cfg=env_cfg, features=report, max_px=worst, passed=worst is not None and worst < LIMIT_PX)


def draw(image_path, labels, report, out):
    import cv2

    def px(p):
        return tuple(int(round(v)) for v in p)

    image = cv2.imread(str(image_path))
    for name, real in labels.get("points", {}).items():
        cv2.circle(image, px(report["features"][name]["sim"]), 6, (0, 200, 0), 2)
        cv2.drawMarker(image, px(real), (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
    for name, pixels in labels.get("edges", {}).items():
        a, b = (np.array(p) for p in report["features"][name]["sim"])
        far = 4 * max(image.shape)  # extend the projected line across the image
        d = (b - a) / np.linalg.norm(b - a)
        cv2.line(image, px(a - far * d), px(a + far * d), (0, 200, 0), 1)
        for p in pixels:
            cv2.drawMarker(image, px(p), (0, 0, 255), cv2.MARKER_CROSS, 8, 1)
    for i, (name, r) in enumerate(report["features"].items()):
        cv2.putText(image, f"{name} {r['error_px']}px", (8, 16 + 14 * i), 0, 0.4, (255, 255, 255), 1)
    cv2.imwrite(str(out), image)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--env-cfg", required=True)
    parser.add_argument("--labels", type=Path, help="labelled view; holds its camera and joints")
    parser.add_argument("--camera", default="cam_head", choices=sorted(MOUNTS), help="guide only")
    parser.add_argument("--joints", type=Path, help="guide only: JSON {URDF joint: position}")
    parser.add_argument("--image", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    if args.labels is None:
        joints = json.loads(args.joints.read_text()) if args.joints else None
        K, size, world_from_camera = sim_camera(args.env_cfg, args.camera, joints)
        known = features(args.env_cfg)
        uv, front = project(K, world_from_camera, np.array(list(known.values())))
        guide = {n: p.round(1).tolist() if ok else None for n, p, ok in zip(known, uv, front)}
        print(json.dumps(dict(image_size=size, camera_xyz=world_from_camera[:3, 3].round(4).tolist(), features=guide)))
        return
    labels = json.loads(args.labels.read_text())
    report = check(args.env_cfg, labels)
    if args.image and args.out:
        draw(args.image, labels, report, args.out)
    print(json.dumps(report, indent=2))
    sys.exit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
