"""CPU checks for the per-layout size and colour randomization in build_asset.py (no Isaac)."""

import copy
import json
from pathlib import Path
import tempfile
import unittest

from scripts.g1d.build_asset import RANDOMIZE, randomize_objects

TABLE = 0.75


def layout():
    cup = {"label": "target", "category_idx": 0, "default_pos": [0.1, -0.4, TABLE + 0.05], "physics": {"mass": 0.1}}
    other = {"label": "clutter", "category_idx": 0, "default_pos": [0.0, -0.3, TABLE + 0.02], "physics": {"mass": 0.2}}
    return {"Rigid": {"cup": [cup], "box": [other]}}


class RandomizeTests(unittest.TestCase):
    def setUp(self):
        self.assets = Path(tempfile.mkdtemp())
        for cat, meta in (("cup", {"visual": {"color_parts": ["model"]}}), ("box", {})):
            path = self.assets / f"Object/RoboDojo/Rigid/{cat}/00000/metadata.json"
            path.parent.mkdir(parents=True)
            path.write_text(json.dumps(meta))

    def test_scale_keeps_the_object_on_the_table(self):
        rules = {"target": {"scale": (1.2, 1.2)}}
        out = randomize_objects(layout(), rules, TABLE, self.assets, "seed")
        cup = out["Rigid"]["cup"][0]
        self.assertEqual(cup["scale"], [1.2, 1.2, 1.2])
        self.assertAlmostEqual(cup["default_pos"][2], TABLE + 0.06)  # 5 cm above the top -> 6 cm
        self.assertAlmostEqual(cup["physics"]["mass"], 0.1 * 1.2**3)
        self.assertEqual(out["Rigid"]["box"][0], layout()["Rigid"]["box"][0])  # other labels untouched

    def test_colour_is_reproducible_and_names_the_parts(self):
        rules = {"target": {"color": True}, "clutter": {"color": True}}
        a = randomize_objects(layout(), rules, TABLE, self.assets, "task/0/3")
        b = randomize_objects(layout(), rules, TABLE, self.assets, "task/0/3")
        self.assertEqual(a, b)
        visual = a["Rigid"]["cup"][0]["visual"]
        self.assertEqual(visual["color_parts"], ["model"])
        self.assertTrue(all(0 <= c <= 1 for c in visual["color"]) and len(visual["color"]) == 3)
        self.assertNotIn("color_parts", a["Rigid"]["box"][0]["visual"])  # no parts listed: the whole object
        c = randomize_objects(layout(), rules, TABLE, self.assets, "task/0/4")
        self.assertNotEqual(c["Rigid"]["cup"][0]["visual"], visual)
        self.assertNotIn("scale", a["Rigid"]["cup"][0])

    def test_off_by_default(self):
        self.assertEqual(RANDOMIZE, {})  # turning a task on changes its eval layouts; see the note in build_asset.py
        before = layout()
        self.assertEqual(randomize_objects(copy.deepcopy(before), {}, TABLE, self.assets, "s"), before)


if __name__ == "__main__":
    unittest.main()
