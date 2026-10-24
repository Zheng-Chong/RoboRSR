"""rsi_assets.py on a tiny synthetic asset tree with two official layouts (the real stack_bowls configs, no Isaac)."""

import json
from pathlib import Path
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rsi_assets as R  # noqa: E402


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def layout(room, table, bowl, clutter=None):
    rigid = {"bowl": [{"category": "bowl", "category_idx": bowl, "label": "bowl0"}]}
    if clutter:
        rigid[clutter[0]] = [{"category_idx": clutter[1], "type": "cluttered"}]
    return {
        "Rigid": rigid,
        "Room": {"default": room},
        "Table": {"default": table},
        "Ground": {"materials": {"default": "material_0564"}},
        "Background": {"category_name": "brown_photostudio_02_4k.hdr"},
    }


def tree():
    root = Path(tempfile.mkdtemp())
    for d in (
        "Room/Simple_Room_nolight",
        "Room/Simple_Room_red",
        "Room/Simple_Room_teal",
        "Material/material_0122",
        "Material/material_0564",
        "Material/material_0001",
        "Material/material_0003",
    ):
        (root / d).mkdir(parents=True)
    (root / "Background").mkdir()
    (root / "Background/brown_photostudio_02_4k.hdr").write_text("")
    for section, cat, idx, ext in (
        ("Rigid", "bowl", 1, 0.1),
        ("Rigid", "bowl", 11, 0.1),
        ("Rigid", "bowl", 16, 0.1),
        ("Rigid", "gizmo", 0, 0.17),
        ("Rigid", "long_rod", 0, 0.42),
        ("Clutter", "book", 0, 0.2),
        ("Clutter", "pen", 0, 0.14),
    ):
        meta = {"geometry": {"aligned_bbox": {"extents": [0.05, 0.05, ext]}}}
        write(root / f"Object/RoboDojo/{section}/{cat}/{idx:05d}/metadata.json", meta)
    folder = root / "Eval_Layout/RoboDojo/arx_x5/0"
    write(folder / "stack_bowls_0.json", layout("Simple_Room_nolight", "material_0122", 1))
    write(folder / "stack_bowls_random_0.json", layout("Simple_Room_teal", "material_0003", 11, ("book", 0)))
    return root


def test_scan_and_audit():
    root = tree()
    result = R.scan(root)
    for asset_id in (
        "Rigid:bowl/11",
        "Rigid:bowl/15",
        "room:Simple_Room_teal",
        "material:material_0003",
        "clutter:book/0",
    ):
        assert asset_id in result["t0"]  # the _random layout's, and what its config can draw
    for asset_id in ("Rigid:bowl/1", "Rigid:bowl/5", "room:Simple_Room_nolight", "hdr:brown_photostudio_02_4k.hdr"):
        assert asset_id in result["seen"]  # bowl 5 is in the stack_bowls config, though no layout shows it
    pools = result["pools"]
    both = {k: pools["train"][k] + pools["exam"][k] for k in pools["train"]}
    assert both["instances"] == ["Rigid:bowl/16"]  # a task category: a new instance, not a distractor
    assert sorted(both["distractors"]) == ["Rigid:gizmo/0", "clutter:pen/0"]
    assert both["rooms"] == ["Simple_Room_red"] and both["materials"] == ["material_0001"]
    assert result["dropped"] == ["Rigid:long_rod/0"]  # too long for the clutter strips
    assert R.side("Rigid:gizmo/0") == R.side("Rigid:gizmo/0")  # a stable split
    ok = R.allowed(result, "train")
    assert "room:Simple_Room_nolight" in ok and "Rigid:bowl/11" not in ok
    out = root / "generated"
    write(out / "stack_bowls_0.json", layout("Simple_Room_nolight", "material_0122", 1))
    write(out / "stack_bowls_1.json", layout("Simple_Room_teal", "material_0122", 11))
    assert R.audit(out, result, "train") == {"stack_bowls_1.json": ["Rigid:bowl/11", "room:Simple_Room_teal"]}


def test_link():
    base, extra, out = (Path(tempfile.mkdtemp()) for _ in range(3))
    write(base / "Object/RoboDojo/Clutter/pen/00000/metadata.json", {})
    write(base / "Room/Simple_Room/base0.usd", {})
    write(extra / "Object/RoboDojo/Clutter/pen/00900/metadata.json", {})
    write(extra / "Object/RoboDojo/Clutter/kettle/00900/metadata.json", {})
    write(extra / "Background/studio.hdr", {})
    (out / "Eval_Layout").mkdir()  # a run's own folder stays
    R.link([base, extra], out)
    pen = out / "Object/RoboDojo/Clutter/pen"
    assert pen.is_dir() and not pen.is_symlink()  # in both trees: merged one level down
    assert (pen / "00000").resolve() == (base / "Object/RoboDojo/Clutter/pen/00000").resolve()
    assert (pen / "00900").is_symlink() and (out / "Object/RoboDojo/Clutter/kettle").is_symlink()
    assert (out / "Room").is_symlink() and (out / "Background/studio.hdr").exists()
    assert not (out / "Eval_Layout").is_symlink()
