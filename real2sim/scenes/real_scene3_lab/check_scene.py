"""Offline checks for the lab operating-table USD package (real_scene3). Writes check_report.json; prints a summary.

Only authoring / geometry checks. PhysX settling, IK reach, grasping and RTX rendering need Isaac Sim and are NOT
covered.
"""

import json
from pathlib import Path
import shutil
import tempfile

import cv2
import numpy as np
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, UsdUtils

HERE = Path(__file__).resolve().parent
USD = HERE / "usd"
# G1-D footprint assumed as in real_scene1_office/check_scene.py (stand cylinder r = 0.22 m around the root, x = 0).
ROBOT_STAND_R = 0.22
# RoboDojo G1-D stance: root at y = -0.73 with the stock table edge at y = -0.60 (env_cfg/robot/g1d.yml).
G1D_ROOT_Y, STOCK_EDGE_Y = -0.73, -0.60


def bounds(prim, cache):
    pts = np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=np.float64)
    m = np.array(cache.GetLocalToWorldTransform(prim)).T
    w = pts @ m[:3, :3].T + m[:3, 3]
    return w.min(0), w.max(0)


def main():
    rep = {
        "checks": [],
        "not_covered": [
            "PhysX settling / stability (needs Isaac Sim)",
            "IK reach and grasping",
            "RTX rendering, exposure and colour match against the photos",
            "real sizes (scale from the 0.75 m height prior), friction and light intensities (estimates)",
        ],
    }

    def add(name, ok, detail):
        rep["checks"].append(dict(name=name, ok=bool(ok), detail=detail))
        print(("PASS " if ok else "FAIL ") + name + ": " + str(detail))

    for lp in sorted(USD.glob("*.usda")):
        st = Usd.Stage.Open(str(lp))
        add(
            f"stage metadata {lp.name}",
            UsdGeom.GetStageUpAxis(st) == UsdGeom.Tokens.z
            and UsdGeom.GetStageMetersPerUnit(st) == 1.0
            and st.GetDefaultPrim().IsValid(),
            dict(upAxis=UsdGeom.GetStageUpAxis(st), defaultPrim=str(st.GetDefaultPrim().GetPath())),
        )

    _, assets, unresolved = UsdUtils.ComputeAllDependencies(str(HERE / "real_scene3.usd"))
    add(
        "dependencies resolve (Room entry real_scene3.usd)",
        not unresolved,
        dict(assets=sorted(Path(str(a)).name for a in assets), unresolved=[str(u) for u in unresolved]),
    )
    with tempfile.TemporaryDirectory() as td:
        dst = Path(td) / "real_scene3"
        dst.mkdir()
        shutil.copytree(USD, dst / "usd")
        shutil.copy(HERE / "real_scene3.usd", dst)
        _, _, unres2 = UsdUtils.ComputeAllDependencies(str(dst / "real_scene3.usd"))
        add("relocated copy resolves", not unres2, dict(unresolved=[str(u) for u in unres2]))

    st = Usd.Stage.Open(str(USD / "scene.usda"))
    cache = UsdGeom.XformCache()
    params = json.loads((USD / "build_params.json").read_text())
    t = params["table"]

    meshes = [p for p in st.Traverse() if p.IsA(UsdGeom.Mesh)]
    render = [p for p in meshes if UsdGeom.Imageable(p).GetPurposeAttr().Get() != UsdGeom.Tokens.guide]
    colliders = [p for p in meshes if p.HasAPI(UsdPhysics.CollisionAPI)]
    unbound = [str(p.GetPath()) for p in render if not UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]]
    add("every render mesh has a bound material", not unbound, dict(render_meshes=len(render), unbound=unbound))
    tris = sum(len(UsdGeom.Mesh(p).GetFaceVertexIndicesAttr().Get()) // 3 for p in render)
    add("render triangle budget (< 100k)", tris < 100_000, dict(triangles=tris))
    bad = [str(p.GetPath()) for p in colliders if UsdGeom.Imageable(p).GetPurposeAttr().Get() != UsdGeom.Tokens.guide]
    add("colliders are guide purpose (not rendered)", not bad, dict(colliders=len(colliders), rendered=bad))
    rigid = [str(p.GetPath()) for p in st.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    add("scene is static (no RigidBodyAPI)", not rigid, dict(rigid=rigid))

    big = [str(p.GetPath()) for p in colliders if len(UsdGeom.Mesh(p).GetPointsAttr().Get()) > 64]
    add("convex colliders have at most 64 points (else particles fall through)", not big, dict(over=big))

    def item(p):  # "/World/Background/Furniture/desk_0/collider_1" -> "desk_0"
        s = str(p.GetPath())
        return s.split("/Furniture/")[1].split("/")[0] if "/Furniture/" in s else None

    cb = {str(p.GetPath()).rsplit("/", 1)[-1]: bounds(p, cache) for p in colliders if not item(p)}
    furniture = [(item(p), bounds(p, cache)) for p in colliders if item(p)]
    lo, hi = cb["top"]
    add(
        "tabletop collider: top at top_z, covers length x width",
        abs(hi[2] - t["top_z"]) < 1e-4
        and abs((hi[0] - lo[0]) - t["length"]) < 1e-3
        and abs((hi[1] - lo[1]) - t["width"]) < 1e-3,
        dict(top_z=t["top_z"], top=round(float(hi[2]), 4), size=[round(float(v), 3) for v in hi[:2] - lo[:2]]),
    )
    sm = UsdGeom.Mesh(st.GetPrimAtPath("/World/Table/Top/surface"))
    sz = np.array(sm.GetPointsAttr().Get())[:, 2]
    add("visible tabletop coincides with the collider", np.allclose(sz, t["top_z"], atol=1e-4), dict(z=float(sz.max())))
    above = [n for n, (a, b) in cb.items() if n != "top" and b[2] > t["top_z"] + 1e-4 and not n.startswith("wall_")]
    add("nothing of the table sticks out above the tabletop", not above, dict(above=above))
    casters = {n: round(float(b[0][2]), 4) for n, b in cb.items() if n.startswith("caster_")}
    add("four casters stand on the floor", len(casters) == 4 and all(abs(z) < 1e-4 for z in casters.values()), casters)
    pnl = cb["panel"]
    add(
        "modesty panel hangs under the tabletop, inside its footprint",
        pnl[1][2] <= lo[2] + 1e-4 and pnl[0][1] > lo[1] and pnl[1][1] < hi[1],
        dict(z=[round(float(pnl[0][2]), 3), round(float(pnl[1][2]), 3)], y=round(float(pnl[0][1]), 3)),
    )
    walls = {n: b for n, b in cb.items() if n.startswith("wall_")}
    table_parts = {n: b for n, b in cb.items() if not n.startswith("wall_") and n != "floor"}
    clash = [
        (a, b)
        for a, (wa, wb) in walls.items()
        for b, (ta, tb) in table_parts.items()
        if (wa < tb).all() and (ta < wb).all()
    ]
    add("table clears the walls", not clash, dict(clash=clash))

    # The G1-D chassis goes under the (legless) stock table. Robot side here is -Y, between the two leg columns.
    offset = STOCK_EDGE_Y - t["front_y"]
    root = np.array([0.0, G1D_ROOT_Y - offset])
    hits = []
    for n, (a, b) in table_parts.items():
        if b[2] < 0.005:  # floor-level parts of nothing but the floor
            continue
        nearest = np.clip(root, a[:2], b[:2])
        if np.linalg.norm(nearest - root) < ROBOT_STAND_R and a[2] < 0.70:
            hits.append(n)
    free_y = min(
        a[1]
        for n, (a, b) in table_parts.items()
        if a[0] < ROBOT_STAND_R and b[0] > -ROBOT_STAND_R and a[2] < t["top_z"] - t["thick"] - 0.05
    )
    add(
        "G1-D stock stance (root 0.13 m behind the edge) clears the legs and the panel",
        not hits,
        dict(
            room_offset_y=round(offset, 3),
            root_in_scene=root.round(3).tolist(),
            hits=hits,
            free_under_table_from_front=round(float(free_y - t["front_y"]), 3),
        ),
    )

    names = sorted({n for n, _ in furniture})
    floor_z = {}
    for p in render:
        if item(p):
            floor_z[item(p)] = min(floor_z.get(item(p), np.inf), float(bounds(p, cache)[0][2]))
    add(
        "room furniture stands on the floor",
        set(floor_z) == set(names) and all(abs(z) < 1e-4 for z in floor_z.values()),
        dict(items=len(names), off_floor={n: round(z, 4) for n, z in floor_z.items() if abs(z) >= 1e-4}),
    )

    def overlap(a, b):
        return (a[0] < b[1]).all() and (b[0] < a[1]).all()

    in_wall = sorted({n for n, fb in furniture for wb in walls.values() if overlap(fb, wb)})
    add("room furniture clears the walls", not in_wall, dict(in_wall=in_wall))
    tlo = np.min([a for a, _ in table_parts.values()], axis=0)
    thi = np.max([b for _, b in table_parts.values()], axis=0)
    near = sorted(
        {
            n
            for n, (a, b) in furniture
            if np.linalg.norm(np.clip(root, a[:2], b[:2]) - root) < 1.0
            or ((a[:2] < thi[:2] + 0.3).all() and (tlo[:2] - 0.3 < b[:2]).all())
        }
    )
    add("room furniture keeps 1 m from the G1-D root and 0.3 m from the table", not near, dict(near=near))
    clash = sorted({(m, n) for m, fa in furniture for n, fb in furniture if m < n and overlap(fa, fb)})
    add("furniture pieces do not overlap (collider boxes)", not clash, dict(clash=clash))

    black = {}
    for f in sorted((USD / "textures").glob("*.png")):
        img = cv2.imread(str(f))
        black[f.name] = round(float((img.max(2) < 8).mean()), 4)
    add("textures have no unfilled (black) areas (< 1%)", all(v < 0.01 for v in black.values()), black)

    lights = [p for p in st.Traverse() if p.HasAPI(UsdLux.LightAPI)]
    add("lights present", len(lights) >= 2, dict(lights=[str(p.GetPath()) for p in lights]))

    rep["passed"] = sum(c["ok"] for c in rep["checks"])
    rep["total"] = len(rep["checks"])
    (HERE / "check_report.json").write_text(json.dumps(rep, indent=1, default=str), encoding="utf-8")
    print(f"{rep['passed']}/{rep['total']} checks passed")


if __name__ == "__main__":
    main()
