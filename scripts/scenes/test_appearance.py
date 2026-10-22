"""CPU checks for scene parts (scene_parts.py output) and the layout appearance overrides (appearance.py)."""

import json
from pathlib import Path
import random
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts" / "scenes"))
import appearance  # noqa: E402

SCENES = ROOT / "real2sim" / "scenes"
LIBRARY = {
    "real_scene1_office/floor/carpet": {"id": "real_scene1_office/floor/carpet", "kind": "preview", "color": None}
    | {"texture": "real2sim/scenes/real_scene1_office/usd/textures/carpet_tile.png", "roughness": 0.95}
    | {"metallic": 0.0, "tile_m": 0.5},
    "mdl/material_0122": {"id": "mdl/material_0122", "kind": "mdl"}
    | {"mdl": "$Robodojo_ASSETS/Material/material_0122/Mahogany_Planks.mdl"},
}


class SceneParts(unittest.TestCase):
    def test_every_built_scene_has_its_surfaces(self):
        for path in sorted(SCENES.glob("*/usd/parts.json")):
            parts = json.loads(path.read_text())
            for role in appearance.ROLES:
                self.assertTrue(parts["roles"][role]["parts"], f"{parts['scene']}: no {role}")
            if parts["lift"]:
                lift = parts["lift"]
                self.assertTrue(lift["move"] and lift["stretch"], parts["scene"])
                self.assertTrue(all(base < lift["split_z"] < top + 1e-3 for _, base, top in lift["stretch"]))
        counter = json.loads((SCENES / "real_scene2_counter/usd/parts.json").read_text())
        self.assertIsNone(counter["lift"])  # a floor-standing counter, not a desk


class Appearance(unittest.TestCase):
    def setUp(self):
        self.parts = json.loads((SCENES / "real_scene3_lab/usd/parts.json").read_text())

    def test_overrides(self):
        choice = {"floor": "real_scene1_office/floor/carpet", "wall": None, "tabletop": "mdl/material_0122"}
        room, top = appearance.overrides(self.parts, choice | {"table_top_z": 0.85}, LIBRARY)
        floor, table = room["appearance"]
        self.assertEqual(floor["prims"], ["Background/Floor"])
        self.assertAlmostEqual(floor["uv_scale"], 0.6 / 0.5)  # lab floor UVs repeat every 0.6 m, office carpet 0.5 m
        self.assertEqual(table["material"]["kind"], "mdl")
        self.assertEqual(sorted(table["prims"]), ["Table/Top/slab", "Table/Top/surface"])
        self.assertAlmostEqual(room["table_lift"]["dz"], 0.85 - 0.749)
        self.assertEqual(top, 0.85)
        with self.assertRaises(ValueError):  # past the desk's travel
            appearance.overrides(self.parts, {"table_top_z": 1.5}, LIBRARY)
        counter = json.loads((SCENES / "real_scene2_counter/usd/parts.json").read_text())
        with self.assertRaises(ValueError):
            appearance.overrides(counter, {"table_top_z": 0.9}, LIBRARY)
        with self.assertRaises(KeyError):
            appearance.overrides(self.parts, {"floor": "nope"}, LIBRARY)

    def test_choose_and_apply(self):
        settings = {"fixed": {"wall": "original", "table_top_z": 0.8}, "random": {"floor": list(LIBRARY)}}
        self.assertEqual(appearance.choose(settings)["floor"], None)
        draws = {appearance.choose(settings, random.Random(f"0/{i}"))["floor"] for i in range(20)}
        self.assertEqual(draws, set(LIBRARY))
        self.assertEqual(appearance.choose(settings, random.Random(1)), appearance.choose(settings, random.Random(1)))
        layout = {
            "Room": {"default": "real_scene3", "table_height": 0.749},
            "Rigid": {"cup": [{"default_pos": [0, 0, 0.8]}]},
        }
        loaded = (self.parts, settings, LIBRARY)
        for _ in range(2):  # applying again keeps the objects on the tabletop
            appearance.apply(layout, None, random.Random(3), loaded)
            self.assertAlmostEqual(layout["Rigid"]["cup"][0]["default_pos"][2], 0.8 + 0.8 - 0.749)
        self.assertEqual(layout["Room"]["table_height"], 0.8)
        self.assertIn("table_lift", layout["Room"])

    def test_cli_needs_settings(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(appearance.load(SCENES / "real_scene3_lab")[0]["scene"], "real_scene3_lab")
            self.assertIsNone(appearance.apply({"Room": {}}, tmp, loaded=(self.parts, {}, LIBRARY)))


if __name__ == "__main__":
    unittest.main()
