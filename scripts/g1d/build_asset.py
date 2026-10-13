"""Build the Unitree G1-D + Dex1-1 robot asset for RoboDojo.

Takes the official G1-D URDF (Dex3 hands) from unitree_ros, swaps the hands for the Dex1-1 gripper and adds the
G1 head D435 frame (both taken from g1_29dof_mode_15_with_dex1_1.urdf), and locks the wheels (tabletop only).
Writes:
    Assets/Robots/g1d/                     g1d_dex1.urdf, meshes/, {left,right}_robot_config.yml
    Assets/Eval_Layout/RoboDojo/g1d/       arx_x5 layouts minus the camera stand (it would sit inside the robot)

Usage:
    python scripts/g1d/build_asset.py                # fetch unitree_ros files from GitHub into --src
    python scripts/g1d/build_asset.py --src DIR      # DIR mirrors unitree_ros (robots/...); reused if present
"""

import argparse
import json
from pathlib import Path
import shutil
import urllib.request
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
UNITREE_RAW = "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/"
G1D_URDF = "robots/g1_d_description/g1_d.urdf"
G1_DEX1_URDF = "robots/g1_description/g1_29dof_mode_15_with_dex1_1.urdf"
D435_INERTIAL = (
    '<inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.01"/>'
    '<inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>'
)


def is_dex3(name):
    return "_hand_" in name


def is_dex1_or_camera(el):
    name = el.get("name", "")
    return "dex1" in name or name in ("left_base_joint", "right_base_joint", "d435_joint", "d435_link")


def mesh_files(root):
    return {m.get("filename") for m in root.iter("mesh")}


def fetch(src, rel):
    dst = src / rel
    if not dst.exists():
        dst.parent.mkdir(parents=True, exist_ok=True)
        print(f"fetch {rel}")
        urllib.request.urlretrieve(UNITREE_RAW + rel, dst)
    return dst


def build_urdf(src, out):
    g1d = ET.parse(fetch(src, G1D_URDF)).getroot()
    g1 = ET.parse(fetch(src, G1_DEX1_URDF)).getroot()

    for el in list(g1d):
        if (el.tag == "link" and is_dex3(el.get("name"))) or (
            el.tag == "joint" and is_dex3(el.find("child").get("link"))
        ):
            g1d.remove(el)
    dex1 = [el for el in g1 if el.tag in ("link", "joint") and is_dex1_or_camera(el)]
    g1d.extend(dex1)

    for joint in g1d.findall("joint"):
        if joint.get("name").endswith("Wheel_Joint"):
            joint.set("type", "fixed")
    for link in g1d.findall("link"):
        if link.find("inertial") is None:  # d435_link is massless in the G1 URDF; PhysX wants mass
            link.append(ET.fromstring(D435_INERTIAL))

    movable = [j.get("name") for j in g1d.findall("joint") if j.get("type") != "fixed"]
    for side in ("left", "right"):
        assert sum(n.startswith(side) and "dex1" not in n for n in movable) == 7, f"{side} arm must have 7 joints"
        assert sum(n.startswith(f"{side}_dex1_finger") for n in movable) == 2, f"{side} Dex1 must have 2 fingers"

    dex1_meshes = {m.get("filename") for el in dex1 for m in el.iter("mesh")}
    (out / "meshes").mkdir(parents=True, exist_ok=True)
    for mesh in mesh_files(g1d):
        urdf_rel = G1_DEX1_URDF if mesh in dex1_meshes else G1D_URDF
        shutil.copy(fetch(src, str(Path(urdf_rel).parent / mesh)), out / mesh)
    ET.ElementTree(g1d).write(out / "g1d_dex1.urdf", encoding="utf-8", xml_declaration=True)
    print(f"wrote {out / 'g1d_dex1.urdf'} ({len(movable)} movable joints)")


def build_layouts(assets):
    src = assets / "Eval_Layout" / "RoboDojo" / "arx_x5"
    dst = assets / "Eval_Layout" / "RoboDojo" / "g1d"
    files = sorted(src.rglob("*.json"))
    for f in files:
        layout = json.loads(f.read_text())
        geometry = layout.get("Geometry", {})
        geometry.pop("camera_stand", None)
        if "Geometry" in layout and not geometry:
            del layout["Geometry"]
        out = dst / f.relative_to(src)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(layout, indent=4))
    print(f"wrote {len(files)} layouts to {dst}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--src", type=Path, default=ROOT / ".cache" / "unitree_ros")
    parser.add_argument("--assets", type=Path, default=ROOT / "Assets")
    args = parser.parse_args()

    out = args.assets / "Robots" / "g1d"
    build_urdf(args.src, out)
    for side in ("left", "right"):
        shutil.copy(HERE / f"{side}_robot_config.yml", out)
    build_layouts(args.assets)


if __name__ == "__main__":
    main()
