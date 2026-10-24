"""manitwin.py selection and metadata on synthetic annotations (no usd-core, no Isaac)."""

import json
from pathlib import Path
import sys
import tempfile

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import manitwin as M  # noqa: E402


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def test_select():
    root = Path(tempfile.mkdtemp())
    assets, ann = root / "Assets", root / "mt"
    write(
        assets / "Object/RoboDojo/Clutter/pen/00000/metadata.json",
        {"geometry": {"aligned_bbox": {"extents": [0.0135, 0.1436, 0.0114]}}},
    )
    write(
        assets / "Object/RoboDojo/Rigid/mug/00001/metadata.json",
        {"geometry": {"aligned_bbox": {"extents": [0.08, 0.12, 0.1]}}},
    )
    for src, dims in {
        "pen/000": [0.0114, 0.0135, 0.1436],  # RoboDojo's pen: a copy
        "coffee_mug/000": [0.088, 0.132, 0.11],  # a mug of the same proportions, rescaled 1.1x: a copy
        "kettle/000": [0.15, 0.2, 0.22],
        "kettle/001": [0.16, 0.2, 0.23],
        "kettle/002": [0.17, 0.2, 0.24],  # over --per-category
        "rug/000": [0.6, 0.9, 0.01],  # too large
        "coin/000": [0.002, 0.02, 0.02],  # too thin
    }.items():
        write(ann / src / "manipulation_annotations.json", {"bounding_box": {"dimensions": dims}})
    result = M.select(ann, assets, per_category=2)
    assert [(c["src"], c["index"]) for c in result["chosen"]] == [("kettle/000", 900), ("kettle/001", 901)]
    reasons = {d["src"]: d["reason"] for d in result["dropped"]}
    assert reasons["pen/000"] == "like Clutter/pen/00000" and reasons["coffee_mug/000"] == "like Rigid/mug/00001"
    assert reasons["rug/000"] == reasons["coin/000"] == "size"


def test_metadata():
    grid = np.array([[x, y, z] for x in (-0.05, 0.05) for y in (-0.02, 0.02) for z in (0.0, 0.1)])
    meta = M.metadata(grid, 12, 8, "box/000")
    assert np.allclose(meta["geometry"]["aligned_bbox"]["extents"], [0.1, 0.04, 0.1])
    place = meta["active"]["place"]["up"]
    assert place["projection_circle"]["center"][:3] == [0.0, 0.0, 0.0]
    assert abs(place["projection_circle"]["radius"] - np.hypot(0.05, 0.02)) < 1e-9
    assert M.MASS_KG[0] <= meta["physics"]["mass"] <= M.MASS_KG[1] and meta["source"]["license"] == "Apache-2.0"
