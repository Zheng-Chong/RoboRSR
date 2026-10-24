"""Import ManiTwin-100K objects (Apache-2.0, huggingface.co/datasets/ManiTwin/ManiTwin-100K) as RoboDojo rigid
objects, for the RSI distractor and instance pools (rsi/AGENTS.md). RoboDojo's own clutter library was built in a
ManiTwin workspace (its clutter.yml source_root), so a ManiTwin object the size of any RoboDojo object is dropped as a
likely copy.

    select   <annotations root> + Assets/ -> candidates.json: tabletop sized, not a near-duplicate, --per-category
    convert  candidates.json + the downloaded base_rescaled.usdz -> <out>/Object/RoboDojo/<section>/<category>/<index>/
             object.usdz (/World: the ManiTwin model referenced whole, its own dome light switched off, standing on
             z = 0, plus a decimated convex-decomposition collision mesh), metadata.json, description.json

Needs usd-core, numpy and fast-simplification (no Isaac):
    python scripts/objects/manitwin.py select --annotations manitwin/ManiTwin-100K --assets Assets --out candidates.json
    python scripts/objects/manitwin.py convert --candidates candidates.json --source manitwin/ManiTwin-100K --out extra
Then drop-test with scripts/objects/check_object.py, link the tree into a run's Assets (rsi_assets.py link) and rescan.
"""

import argparse
import json
from pathlib import Path
import re
import shutil
import tempfile
import uuid

import numpy as np

SIZE_M = (0.03, 0.30)  # longest side of a tabletop object
THIN_M = 0.005  # shortest side
DUP_ABS, DUP_REL = 0.003, 0.03  # sorted extents this close to a RoboDojo object: treated as the same asset
SHAPE_REL = 0.05  # aspect ratios this close, with a shared category word: treated as the same asset, rescaled
FIRST_INDEX = 900  # new instances sit above every official index
COLLISION_FACES = 2000
MASS_KG = (0.02, 0.5)
DENSITY = 250.0  # kg/m^3 of the bounding box, a hollow household object


def robodojo_objects(assets):
    """(id, category, sorted extents) of every object in a RoboDojo Assets tree."""
    found = []
    for meta in Path(assets).glob("Object/RoboDojo/*/*/*/metadata.json"):
        geometry = json.loads(meta.read_text(encoding="utf-8")).get("geometry") or {}
        extents = (geometry.get("aligned_bbox") or geometry.get("oriented_bbox") or {}).get("extents")
        if extents:
            section, category, index = meta.parts[-4:-1]
            found.append((f"{section}/{category}/{index}", category, sorted(extents)))
    return found


def duplicate_of(category, dims, known):
    """The RoboDojo object a ManiTwin object most likely is, or None."""
    words = set(category.split("_"))
    for oid, cat, ext in known:
        if all(abs(a - b) <= max(DUP_ABS, DUP_REL * b) for a, b in zip(dims, ext)):
            return oid
        if words & set(cat.split("_")):
            ra, rb = np.asarray(dims) / dims[-1], np.asarray(ext) / ext[-1]
            if np.all(np.abs(ra - rb) <= SHAPE_REL):
                return oid
    return None


def prim_name(category):
    """A ManiTwin category as a USD prim name (the loaders build prim paths from it): chef's_knife -> chef_s_knife."""
    name = re.sub(r"[^0-9A-Za-z_]", "_", category)
    return f"n{name}" if name[0].isdigit() else name


def select(annotations, assets, per_category):
    known, chosen, dropped = robodojo_objects(assets), [], []
    taken = {}
    for path in sorted(Path(annotations).glob("*/*/manipulation_annotations.json")):
        raw, obj = path.parts[-3], path.parts[-2]
        if raw.endswith("_tmp"):  # ManiTwin's working copies of another category
            continue
        category = prim_name(raw)
        box = json.loads(path.read_text(encoding="utf-8")).get("bounding_box") or {}
        dims = sorted(box.get("dimensions") or [0, 0, 0])
        src = f"{raw}/{obj}"
        if not (SIZE_M[0] <= dims[-1] <= SIZE_M[1] and dims[0] >= THIN_M):
            dropped.append({"src": src, "reason": "size", "dims": dims})
        elif dup := duplicate_of(category, dims, known):
            dropped.append({"src": src, "reason": f"like {dup}", "dims": dims})
        elif taken.get(category, 0) < per_category:
            index = FIRST_INDEX + taken.get(category, 0)
            taken[category] = taken.get(category, 0) + 1
            chosen.append({"src": src, "section": "Clutter", "category": category, "index": index})
    return {"source": "ManiTwin/ManiTwin-100K", "license": "Apache-2.0", "chosen": chosen, "dropped": dropped}


def mesh_arrays(stage):
    """World-space points and triangle faces of every mesh under /root except lights."""
    from pxr import Usd, UsdGeom

    points, faces, offset = [], [], 0
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        p = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        m = np.asarray(cache.GetLocalToWorldTransform(prim), dtype=float)  # row-vector convention
        points.append(p @ m[:3, :3] + m[3, :3])
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
        idx = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()) + offset
        start = np.concatenate([[0], np.cumsum(counts)[:-1]])
        for s, c in zip(start, counts):  # fan-triangulate
            faces.extend([idx[s], idx[s + k], idx[s + k + 1]] for k in range(1, c - 1))
        offset += len(p)
    return np.concatenate(points), np.asarray(faces, dtype=np.int64)


def metadata(points, faces_total, vertices_total, src):
    lo, hi = points.min(0), points.max(0)
    corners = [[float(x), float(y), float(z)] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    extents = [float(v) for v in hi - lo]
    center = (lo[:2] + hi[:2]) / 2
    radius = float(np.linalg.norm(points[:, :2] - center, axis=1).max())
    base = points[points[:, 2] <= lo[2] + 0.005]
    contact = float(np.linalg.norm(base[:, :2] - center, axis=1).max()) if len(base) else radius / 2
    circle = [float(center[0]), float(center[1]), 0.0, 1.0, 0.0, 0.0, 0.0]
    mass = float(np.clip(DENSITY * np.prod(extents), *MASS_KG))
    bbox = {"vertices": corners, "extents": extents}
    return {
        "uuid": str(uuid.uuid5(uuid.NAMESPACE_URL, f"manitwin:{src}")),
        "geometry": {
            "faces": faces_total,
            "vertices": vertices_total,
            "aligned_bbox": bbox,
            "oriented_bbox": bbox,
            "radius": radius,
        },
        "active": {
            "place": {
                "up": {
                    "orientation": [1.0, 0.0, 0.0, 0.0],
                    "projection_circle": {"center": circle, "radius": radius},
                    "contact_circle": {"center": circle, "radius": contact},
                }
            }
        },
        "physics": {"mass": round(mass, 4)},
        "source": {"dataset": "ManiTwin/ManiTwin-100K", "object": src, "license": "Apache-2.0"},
    }


def convert_one(item, source, out):
    import fast_simplification
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

    usdz = Path(source) / item["src"] / "base_rescaled.usdz"
    target = Path(out) / "Object" / "RoboDojo" / item["section"] / item["category"] / f"{item['index']:05d}"
    src_stage = Usd.Stage.Open(str(usdz))
    points, faces = mesh_arrays(src_stage)
    lo, hi = points.min(0), points.max(0)
    shift = np.array([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
    points = points + shift
    col_pts, col_faces = points, faces
    if len(faces) > COLLISION_FACES:
        col_pts, col_faces = fast_simplification.simplify(
            points, faces, target_reduction=1 - COLLISION_FACES / len(faces)
        )
    with tempfile.TemporaryDirectory() as tmp:
        shutil.copy(usdz, Path(tmp) / "model.usdz")
        stage = Usd.Stage.CreateNew(str(Path(tmp) / "object.usda"))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(world.GetPrim())
        visual = UsdGeom.Xform.Define(stage, "/World/visual")
        visual.GetPrim().GetReferences().AddReference("./model.usdz", Sdf.Path("/root"))
        visual.AddTranslateOp().Set(tuple(float(v) for v in shift))
        stage.OverridePrim("/World/visual/env_light").SetActive(False)  # every object would add a dome light
        col = UsdGeom.Mesh.Define(stage, "/World/collision")
        col.CreatePointsAttr([tuple(map(float, p)) for p in col_pts])
        col.CreateFaceVertexCountsAttr([3] * len(col_faces))
        col.CreateFaceVertexIndicesAttr([int(i) for i in np.asarray(col_faces).ravel()])
        UsdPhysics.CollisionAPI.Apply(col.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(col.GetPrim()).CreateApproximationAttr("convexDecomposition")
        col.MakeInvisible()
        stage.GetRootLayer().Save()
        target.mkdir(parents=True, exist_ok=True)
        if not UsdUtils.CreateNewUsdzPackage(
            Sdf.AssetPath(str(Path(tmp) / "object.usda")), str(target / "object.usdz")
        ):
            raise RuntimeError(f"packaging {usdz} failed")
    meta = metadata(points, int(len(faces)), int(len(points)), item["src"])
    (target / "metadata.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    caption = Path(source) / item["src"] / "caption.json"
    seen = json.loads(caption.read_text(encoding="utf-8")).get("seen", []) if caption.is_file() else []
    (target / "description.json").write_text(json.dumps({"description": seen}, ensure_ascii=False), encoding="utf-8")
    return target


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("select")
    s.add_argument("--annotations", type=Path, required=True)
    s.add_argument("--assets", type=Path, required=True)
    s.add_argument("--per-category", type=int, default=2)
    s.add_argument("--out", type=Path, required=True)
    c = sub.add_parser("convert")
    c.add_argument("--candidates", type=Path, required=True)
    c.add_argument("--source", type=Path, required=True)
    c.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    if args.cmd == "select":
        result = select(args.annotations, args.assets, args.per_category)
        args.out.write_text(json.dumps(result, indent=1), encoding="utf-8")
        reasons = {}
        for d in result["dropped"]:
            key = "size" if d["reason"] == "size" else "duplicate"
            reasons[key] = reasons.get(key, 0) + 1
        categories = len({c["category"] for c in result["chosen"]})
        print(f"chosen {len(result['chosen'])} from {categories} categories; dropped {reasons}")
    else:
        failed = []
        for item in json.loads(args.candidates.read_text(encoding="utf-8"))["chosen"]:
            done = args.out / "Object" / "RoboDojo" / item["section"] / item["category"] / f"{item['index']:05d}"
            if (done / "metadata.json").is_file():
                continue
            try:
                convert_one(item, args.source, args.out)
            except Exception as e:  # keep going; report at the end
                failed.append((item["src"], repr(e)[:200]))
        print(f"failed {len(failed)}: {failed[:10]}")


if __name__ == "__main__":
    main()
