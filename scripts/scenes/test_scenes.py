"""CPU checks for the scene tools: python -m pytest scripts/scenes"""

import json
from pathlib import Path
import struct
import unittest

import numpy as np

from scripts.scenes import acceptance, export_glb, overlay

try:
    import cv2

    from scripts.scenes import charuco
except ImportError:
    cv2 = None
ROOT = Path(__file__).resolve().parents[2]


class OverlayTests(unittest.TestCase):
    def test_head_camera_matches_the_unitree_contract(self):
        K, size, world_from_camera = overlay.sim_camera("g1d")
        self.assertEqual(size, (640, 480))
        self.assertAlmostEqual(K[0, 0], 279.5, delta=0.1)
        axis = world_from_camera[:3, 2]
        self.assertAlmostEqual(np.degrees(np.arcsin(-axis[2])), 47.6, delta=0.1)  # optical axis 47.6 deg down
        self.assertGreater(axis[1], 0)  # toward the table (+y)
        np.testing.assert_allclose(world_from_camera[:3, 3], [-0.0298, -0.6285, 1.281], atol=1e-3)  # left eye

    def test_points_and_edges_are_scored_in_pixels(self):
        K, _, world_from_camera = overlay.sim_camera("g1d_lab")
        known = overlay.features("g1d_lab")
        uv = dict(zip(known, overlay.project(K, world_from_camera, np.array(list(known.values())))[0]))
        a, b = uv["table_back_left"], uv["table_back_right"]
        normal = np.array([-(b - a)[1], (b - a)[0]]) / np.linalg.norm(b - a)
        for shift, passed in ((3.0, True), (6.0, False)):
            edge = [(a + t * (b - a) + shift * normal).tolist() for t in (0.2, 0.5, 0.8)]
            labels = dict(points={"table_back_left": (a + [0, 2]).tolist()}, edges={"table_back_edge": edge})
            report = overlay.check("g1d_lab", labels)
            self.assertAlmostEqual(report["features"]["table_back_edge"]["error_px"], shift, places=1)
            self.assertAlmostEqual(report["features"]["table_back_left"]["error_px"], 2.0, places=1)
            self.assertIs(report["passed"], passed)
        with self.assertRaisesRegex(ValueError, "Unknown"):
            overlay.check("g1d_lab", dict(points={"chair": [0, 0]}))

    def test_wrist_camera_follows_the_arm_joints(self):
        rest = overlay.sim_camera("g1d", "cam_left_wrist")[2]
        bent = overlay.sim_camera("g1d", "cam_left_wrist", {"left_elbow_joint": 0.8})[2]
        self.assertGreater(np.linalg.norm(rest[:3, 3] - bent[:3, 3]), 0.05)
        self.assertLess(rest[:3, 3][0], 0)  # the left hand is on the robot's left: world -x


class SpecTests(unittest.TestCase):
    def test_scene_specs_are_complete_and_read_the_model(self):
        expected = {  # edge ahead, centre x
            "g1d": (0.13, 0.0),
            "g1d_counter": (0.2, -0.35),
            "g1d_lab": (0.13, 0.0),
            "g1d_school": (0.19, 0.0828),
            "g1d_shenzhen_lab": (0.13, 0.0),
        }
        specs = sorted((ROOT / "real2sim/scenes").glob("*/scene_spec.json"))
        self.assertEqual(len(specs), 5)
        for path in specs:
            spec = json.loads(path.read_text())
            report = acceptance.evaluate(spec)
            tape = report["tape"]
            edge, centre = expected[spec["env_cfg"]]
            self.assertAlmostEqual(tape["table_edge_from_robot_root"]["model_m"], edge, delta=1e-3)
            self.assertAlmostEqual(tape["table_centre_x_from_robot_root"]["model_m"], centre, delta=1e-3)
            self.assertTrue(all(any(path.parent.glob(p)) for p in spec["photos"]), path)
            spec["measurements"]["table_top_z"] = dict(value_m=tape["table_top_z"]["model_m"] + 0.02)
            self.assertEqual(acceptance.evaluate(spec)["tape"]["table_top_z"]["status"], "fail")


@unittest.skipIf(cv2 is None, "opencv not installed")
class CharucoTests(unittest.TestCase):
    def render(self, board, K, camera_from_board, size=(1280, 960)):
        """The printed board as a camera at camera_from_board would see it (on a grey background)."""
        px = 100
        image = board.generateImage((charuco.SQUARES[0] * px, charuco.SQUARES[1] * px), marginSize=0, borderBits=1)
        metres_per_px = board.getSquareLength() / px
        H = K @ camera_from_board[:3][:, [0, 1, 3]] @ np.diag([metres_per_px, metres_per_px, 1])
        return cv2.warpPerspective(image, H, size, borderValue=160)

    def poses(self):
        for tilt, turn, distance in ((20, 0, 0.6), (35, 15, 0.7), (-25, -20, 0.55), (10, 30, 0.8), (40, -10, 0.65)):
            T = np.eye(4)
            T[:3, :3] = cv2.Rodrigues(np.radians([tilt, turn, 5.0]))[0]
            T[:3, 3] = [-0.12, -0.09, distance]
            yield T

    def test_pose_and_calibration_from_synthetic_photos(self):
        board = charuco.make_board()
        K = np.array([[900.0, 0, 640], [0, 900.0, 480], [0, 0, 1]])
        views = []
        for T in self.poses():
            obj, img = charuco.detect(self.render(board, K, T), board)
            views.append((obj, img))
            found, rms = charuco.board_pose(obj, img, K, np.zeros(5))
            self.assertLess(rms, 0.5)
            np.testing.assert_allclose(found[:3, 3], T[:3, 3], atol=0.002)  # 2 mm at 0.6-0.8 m
            angle = np.degrees(np.linalg.norm(cv2.Rodrigues(found[:3, :3] @ T[:3, :3].T)[0]))
            self.assertLess(angle, 0.3)
        K_fit, _, rms = charuco.calibrate(views, (1280, 960))
        self.assertLess(rms, 0.5)
        self.assertAlmostEqual(K_fit[0, 0] / 900.0, 1.0, delta=0.01)


class GlbTests(unittest.TestCase):
    def test_glb_layout(self):
        quad = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], float)
        tris = export_glb.fan([4])
        np.testing.assert_array_equal(tris, [0, 1, 2, 0, 2, 3])
        mesh = {"name": "/World/Table/Top", "positions": quad[tris], "uv": quad[tris, :2], "material": 0}
        material = {"pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}}
        data = export_glb.pack_glb([mesh], [material], [(b"\x89PNG", "image/png", 33648, 10497)])
        magic, version, length = struct.unpack("<III", data[:12])
        self.assertEqual((magic, version, length), (0x46546C67, 2, len(data)))
        size, kind = struct.unpack("<II", data[12:20])
        gltf = json.loads(data[20 : 20 + size])
        self.assertEqual(kind, 0x4E4F534A)
        self.assertEqual(gltf["nodes"][0]["name"], "/World/Table/Top")
        position = gltf["accessors"][gltf["meshes"][0]["primitives"][0]["attributes"]["POSITION"]]
        self.assertEqual((position["count"], position["max"]), (6, [1, 1, 0]))
        self.assertEqual(gltf["samplers"][0], {"wrapS": 33648, "wrapT": 10497})
        self.assertEqual(len(data) % 4, 0)
        bin_size = struct.unpack("<I", data[20 + size : 24 + size])[0]
        self.assertEqual(bin_size, gltf["buffers"][0]["byteLength"])

    def test_primvars_expand_per_corner(self):
        counts, indices = np.array([3, 3]), np.array([0, 1, 2, 0, 2, 3])
        uniform = export_glb.per_corner([[1, 0, 0], [0, 1, 0]], "uniform", indices, counts)
        np.testing.assert_array_equal(uniform[3:], [[0, 1, 0]] * 3)
        vertex = export_glb.per_corner(np.eye(4)[:, :2], "vertex", indices, counts)
        np.testing.assert_array_equal(vertex[4], [0, 0])


if __name__ == "__main__":
    unittest.main()
