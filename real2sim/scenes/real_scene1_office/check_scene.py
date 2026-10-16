"""Offline checks for the delivered USD package. Writes check_report.json; prints a summary.

Only geometric / authoring checks are performed here. PhysX settling, IK, grasping and RTX
rendering require Isaac Sim and are NOT covered.
"""
import json
import shutil
import tempfile
from pathlib import Path

import numpy as np
import trimesh
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

HERE = Path(__file__).resolve().parent
USD = HERE / "usd"
ROBOT_STAND = dict(x=0.0, y=-0.61, r=0.22)


def world_mesh(prim, cache):
    m = UsdGeom.Mesh(prim)
    pts = np.array(m.GetPointsAttr().Get(), dtype=np.float64)
    idx = np.array(m.GetFaceVertexIndicesAttr().Get(), dtype=np.int64).reshape(-1, 3)
    M = np.array(cache.GetLocalToWorldTransform(prim)).T
    return trimesh.Trimesh(pts @ M[:3, :3].T + M[:3, 3], idx, process=False)


def main():
    rep = {"checks": [], "not_covered": [
        "PhysX settling / stability (needs Isaac Sim)", "IK reachability and grasping",
        "RTX rendering, exposure and camera colour match", "real material constants (friction/mass are estimates)"]}

    def add(name, ok, detail):
        rep["checks"].append(dict(name=name, ok=bool(ok), detail=detail))
        print(("PASS " if ok else "FAIL ") + name + ": " + str(detail))

    layers = sorted(USD.rglob("*.usda"))
    for lp in layers:
        st = Usd.Stage.Open(str(lp))
        add(f"stage metadata {lp.relative_to(USD)}",
            UsdGeom.GetStageUpAxis(st) == UsdGeom.Tokens.z and UsdGeom.GetStageMetersPerUnit(st) == 1.0
            and st.GetDefaultPrim().IsValid(),
            dict(upAxis=UsdGeom.GetStageUpAxis(st), metersPerUnit=UsdGeom.GetStageMetersPerUnit(st),
                 defaultPrim=str(st.GetDefaultPrim().GetPath())))

    layers_dep, assets, unresolved = UsdUtils.ComputeAllDependencies(str(USD / "scene.usda"))
    add("dependencies resolve", len(unresolved) == 0,
        dict(layers=len(layers_dep), assets=[Path(a.resolvedPath if hasattr(a, "resolvedPath") else a).name for a in assets],
             unresolved=[str(u) for u in unresolved]))

    with tempfile.TemporaryDirectory() as td:
        dst = Path(td) / "copied_package"
        shutil.copytree(USD, dst)
        _, _, unres2 = UsdUtils.ComputeAllDependencies(str(dst / "scene.usda"))
        st2 = Usd.Stage.Open(str(dst / "scene.usda"))
        add("relocated copy opens with all assets", len(unres2) == 0 and st2.GetPrimAtPath("/World/Objects/paper_cup").IsValid(),
            dict(unresolved=[str(u) for u in unres2]))

    st = Usd.Stage.Open(str(USD / "scene.usda"))
    cache = UsdGeom.XformCache()
    bad_mesh, n_mesh, n_tri = [], 0, 0
    colliders = {}
    for prim in st.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        n_mesh += 1
        m = UsdGeom.Mesh(prim)
        pts = np.array(m.GetPointsAttr().Get())
        idx = np.array(m.GetFaceVertexIndicesAttr().Get())
        cnt = np.array(m.GetFaceVertexCountsAttr().Get())
        n_tri += len(cnt)
        nrm = m.GetNormalsAttr().Get()
        interp = m.GetNormalsInterpolation()
        exp = len(idx) if interp == UsdGeom.Tokens.faceVarying else len(pts)
        tm = world_mesh(prim, cache)
        degenerate = int((tm.area_faces < 1e-12).sum())
        if (not np.isfinite(pts).all() or idx.max() >= len(pts) or (cnt != 3).any() or
                (nrm is not None and len(nrm) != exp) or degenerate):
            bad_mesh.append(dict(path=str(prim.GetPath()), degenerate=degenerate))
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            colliders[str(prim.GetPath())] = tm
    add("mesh validity (finite points, triangle indices, normals count, no degenerate faces)", not bad_mesh,
        dict(meshes=n_mesh, triangles=n_tri, bad=bad_mesh[:10]))

    unbound = []
    for prim in st.Traverse():
        if prim.IsA(UsdGeom.Mesh) and UsdGeom.Imageable(prim).ComputePurpose() == UsdGeom.Tokens.default_:
            mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            if not mat:
                unbound.append(str(prim.GetPath()))
    add("every render mesh has a bound material (ComputeBoundMaterial)", not unbound, dict(unbound=unbound[:10]))

    params = json.loads((USD / "build_params.json").read_text(encoding="utf-8"))
    top_z = params["table"]["top_z"]
    L, W = params["table"]["length"], params["table"]["width"]
    obj_info = {}
    for name in params["objects"]:
        prim = st.GetPrimAtPath(f"/World/Objects/{name}")
        rb = prim.HasAPI(UsdPhysics.RigidBodyAPI)
        mass = UsdPhysics.MassAPI(prim).GetMassAttr().Get() if prim.HasAPI(UsdPhysics.MassAPI) else None
        parts = [tm for p, tm in colliders.items() if p.startswith(f"/World/Objects/{name}/")]
        allv = np.vstack([p.vertices for p in parts])
        obj_info[name] = dict(rigid_body=rb, mass_kg=mass, collider_parts=len(parts),
                              min_z=round(float(allv[:, 2].min()), 5),
                              xy_min=np.round(allv[:, :2].min(0), 3).tolist(), xy_max=np.round(allv[:, :2].max(0), 3).tolist())
        add(f"object {name}: rigid body + mass + colliders", rb and mass and mass > 0 and parts,
            dict(mass_kg=mass, collider_parts=len(parts)))
        inside = (allv[:, 0].min() > -L / 2 + 0.01 and allv[:, 0].max() < L / 2 - 0.01 and
                  allv[:, 1].min() > -W / 2 + 0.01 and allv[:, 1].max() < W / 2 - 0.01)
        add(f"object {name}: rests on table top (no gap/penetration) and inside the top", abs(allv[:, 2].min() - top_z) < 1e-4 and inside,
            dict(min_z=obj_info[name]["min_z"], table_top=top_z))

    mgr = trimesh.collision.CollisionManager()
    for name in params["objects"]:
        for p, tm in colliders.items():
            if p.startswith(f"/World/Objects/{name}/"):
                mgr.add_object(f"{name}|{p.split('/')[-1]}", tm)
    hit, pairs = mgr.in_collision_internal(return_names=True)
    cross = sorted({tuple(sorted((a.split("|")[0], b.split("|")[0]))) for a, b in pairs if a.split("|")[0] != b.split("|")[0]})
    dmin = mgr.min_distance_internal(return_names=True)
    add("no initial interpenetration between task objects", not cross, dict(colliding_pairs=cross))

    env = trimesh.collision.CollisionManager()
    for p, tm in colliders.items():
        if p.startswith("/World/Objects/") or p.endswith("/floor"):
            continue
        if p.startswith("/World/Table/Colliders/top"):
            continue
        env.add_object(p, tm)
    obj_mgr = trimesh.collision.CollisionManager()
    for p, tm in colliders.items():
        if p.startswith("/World/Objects/"):
            obj_mgr.add_object(p, tm)
    _, env_pairs = obj_mgr.in_collision_other(env, return_names=True)
    add("task objects do not intersect background / table frame colliders", not env_pairs, dict(pairs=sorted(env_pairs)[:10]))

    table_top = colliders["/World/Table/Colliders/top"]
    bg = trimesh.collision.CollisionManager()
    for p, tm in colliders.items():
        if p.startswith("/World/Background/") and not p.endswith("/floor"):
            bg.add_object(p, tm)
    tbl = trimesh.collision.CollisionManager()
    for p, tm in colliders.items():
        if p.startswith("/World/Table/"):
            tbl.add_object(p, tm)
    _, tb_pairs = tbl.in_collision_other(bg, return_names=True)
    d_tb, names_tb = tbl.min_distance_other(bg, return_names=True)
    add("task table does not intersect background colliders", not tb_pairs,
        dict(pairs=sorted(tb_pairs)[:10], min_clearance_m=round(float(d_tb), 3), nearest=list(names_tb)))

    cup_open = {}
    for name, spec in params["objects"].items():
        if spec["kind"] != "cup":
            continue
        parts = [tm for p, tm in colliders.items() if p.startswith(f"/World/Objects/{name}/")]
        x, y = spec["pos"]
        origin = np.array([[x, y, top_z + spec["height"] + 0.05]])
        hits = []
        for tm in parts:
            loc, _, _ = tm.ray.intersects_location(origin, np.array([[0, 0, -1.0]]))
            hits += [float(z) for z in np.asarray(loc).reshape(-1, 3)[:, 2]]
        first = max(hits) if hits else None
        cup_open[name] = None if first is None else round(first - top_z, 4)
        add(f"cup {name}: collision opening is not sealed (center ray first hits the bottom)",
            first is not None and first - top_z < 0.006, dict(first_hit_height_above_table_m=cup_open[name]))

    stand = trimesh.creation.cylinder(radius=ROBOT_STAND["r"], height=1.3,
                                      transform=trimesh.transformations.translation_matrix(
                                          [ROBOT_STAND["x"], ROBOT_STAND["y"], 0.70]))
    rob = trimesh.collision.CollisionManager()
    rob.add_object("robot_zone", stand)
    allc = trimesh.collision.CollisionManager()
    for p, tm in colliders.items():
        if not p.endswith("/floor"):
            allc.add_object(p, tm)
    d_r, names_r = rob.min_distance_other(allc, return_names=True)
    add("static clearance of an estimated robot standing zone (cylinder r=0.22 m at y=-0.61)", d_r > 0.0,
        dict(min_clearance_m=round(float(d_r), 3), nearest=list(names_r),
             table_edge_to_zone_center_m=round(abs(ROBOT_STAND["y"]) - W / 2, 3)))

    lt = Usd.Stage.Open(str(USD / "lights.usda"))
    lc = UsdGeom.XformCache()
    dirs = {}
    for n in ("Sun", "WindowFillNorth", "WindowFillEast"):
        m = lc.GetLocalToWorldTransform(lt.GetPrimAtPath(f"/Lights/{n}"))
        d = np.array(m.TransformDir(Gf.Vec3d(0, 0, -1)))
        to_center = -np.array(m.ExtractTranslation())
        to_center[2] = 0.0
        dirs[n] = dict(emits=np.round(d, 3).tolist(), into_room=bool(d @ to_center > 0) if n != "Sun" else None)
    sun_ok = np.allclose(-np.array(dirs["Sun"]["emits"]) / np.linalg.norm(dirs["Sun"]["emits"]),
                         np.array(params["sun_direction_to_sun"]) / np.linalg.norm(params["sun_direction_to_sun"]), atol=1e-3)
    add("lights: sun matches measured direction, window fills emit into the room",
        sun_ok and dirs["WindowFillNorth"]["into_room"] and dirs["WindowFillEast"]["into_room"], dirs)

    rep["objects"] = obj_info
    (HERE / "check_report.json").write_text(json.dumps(rep, indent=1), encoding="utf-8")
    n_ok = sum(c["ok"] for c in rep["checks"])
    print(f"\n{n_ok}/{len(rep['checks'])} checks passed")


if __name__ == "__main__":
    main()
