"""CPU checks for the G1-D body (waist, lift, chassis): action keys <-> joints and the mobile URDF (no Isaac)."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from env.robot_manager.robot_class.g1d import BASE_LEAD, G1DLeft
from scripts.g1d.build_asset import V4_URDF, mobile_urdf

ROOT = Path(__file__).resolve().parents[2]
DT = 1 / 300


def robot(mobile):
    r = G1DLeft.__new__(G1DLeft)  # body_state / body_targets only read `mobile`
    r.mobile = mobile
    return r


class BodyTests(unittest.TestCase):
    def test_tabletop_waist_and_lift(self):
        command = {"waist_joint_state": [0.3, 0.5], "lift_joint_state": [0.2]}
        pos, vel = robot(False).body_targets(command, np.zeros(4), np.zeros(4), DT)
        np.testing.assert_allclose(pos, [0.3, 0.5, 0.1, 0.1])  # torso_Joint (yaw), Yaw_Joint (pitch), two stages
        np.testing.assert_allclose(vel, 0)
        state = robot(False).body_state(pos, vel)
        self.assertEqual(state, {"waist_joint_state": [0.3, 0.5], "lift_joint_state": [0.2]})

    def test_chassis_drives_along_its_heading(self):
        target = np.array([0, 0, 0.1, 0.1, 0.0, 0.0, np.pi / 2])  # facing +y of the start frame
        command = {"waist_joint_state": [0, 0], "lift_joint_state": [0.2], "base_velocity": [1.0, 0.3]}
        pos, vel = robot(True).body_targets(command, target, target, DT)
        np.testing.assert_allclose(vel[4:], [0, 1.0, 0.3], atol=1e-12)
        np.testing.assert_allclose(pos[4:], target[4:] + vel[4:] * DT)
        state = robot(True).body_state(pos, vel)
        np.testing.assert_allclose(state["base_velocity"], [1.0, 0.3], atol=1e-5)  # chassis frame, yaw one step on
        np.testing.assert_allclose(state["base_pose"], pos[4:])

    def test_chassis_clips_to_official_limits_and_stalls_when_blocked(self):
        target = np.zeros(7)
        command = {"waist_joint_state": [0, 0], "lift_joint_state": [0], "base_velocity": [5.0, -2.0]}
        _, vel = robot(True).body_targets(command, target, target, DT)
        np.testing.assert_allclose(vel[4:], [1.5, 0, -0.6])
        blocked = robot(True).body_targets(command, target + [0, 0, 0, 0, 0.5, 0, 0], target, 1.0)[0]
        np.testing.assert_allclose(blocked[4:], target[4:] + [BASE_LEAD[0], 0, -BASE_LEAD[2]])

    def test_mobile_urdf_puts_the_chassis_on_planar_joints(self):
        urdf = mobile_urdf(ET.parse(ROOT / "real2sim/robot/g1d_v4" / V4_URDF).getroot())
        joints = {j.get("name"): j for j in urdf.findall("joint")}
        chain = [
            (joints[n].find("parent").get("link"), joints[n].find("child").get("link"))
            for n in ("base_x_joint", "base_y_joint", "base_yaw_joint")
        ]
        self.assertEqual(
            chain, [("base_world", "base_x_link"), ("base_x_link", "base_y_link"), ("base_y_link", "AGV_link")]
        )
        children = {j.find("child").get("link") for j in urdf.findall("joint")}
        self.assertEqual({link.get("name") for link in urdf.findall("link")} - children, {"base_world"})
        for name in ("Left_Wheel_Link", "RIght_Wheel_Link"):
            self.assertIsNone(urdf.find(f"link[@name='{name}']/collision"))
        self.assertIsNotNone(urdf.find("link[@name='AGV_link']/collision"))


if __name__ == "__main__":
    unittest.main()
