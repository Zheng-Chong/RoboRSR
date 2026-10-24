"""Offline checks for the pantry-counter USD package (real_scene2). Writes check_report.json; prints a summary.

Only authoring / geometry checks. PhysX settling, IK reach, grasping and RTX rendering need Isaac Sim and are NOT
covered.
"""

import json
from pathlib import Path
import shutil
import tempfile

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
            "RTX rendering, exposure and colour match against the photo",
            "real sizes, friction and light intensities (estimates)",
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
            dict(
                upAxis=UsdGeom.GetStageUpAxis(st),
                metersPerUnit=UsdGeom.GetStageMetersPerUnit(st),
                defaultPrim=str(st.GetDefaultPrim().GetPath()),
            ),
        )

    _, assets, unresolved = UsdUtils.ComputeAllDependencies(str(USD / "scene.usda"))
    add(
        "dependencies resolve",
        not unresolved,
        dict(assets=sorted(Path(str(a)).name for a in assets), unresolved=[str(u) for u in unresolved]),
    )
    with tempfile.TemporaryDirectory() as td:
        dst = Path(td) / "copy"
        shutil.copytree(USD, dst)
        _, _, unres2 = UsdUtils.ComputeAllDependencies(str(dst / "scene.usda"))
        add("relocated copy resolves", not unres2, dict(unresolved=[str(u) for u in unres2]))

    st = Usd.Stage.Open(str(USD / "scene.usda"))
    cache = UsdGeom.XformCache()
    params = json.loads((USD / "build_params.json").read_text())
    top_z = params["table"]["top_z"]

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

    cb = {str(p.GetPath()).rsplit("/", 1)[-1]: bounds(p, cache) for p in colliders}
    tops = [b for n, b in cb.items() if n.startswith("top_")]
    add(
        "countertop collision surface at top_z",
        all(abs(b[1][2] - top_z) < 1e-4 for b in tops),
        dict(top_z=top_z, tops=[round(float(b[1][2]), 4) for b in tops]),
    )

    s = params["sink"]
    xy = np.array([s["cx"], s["cy"]])
    under = sorted((float(b[1][2]), n) for n, b in cb.items() if (b[0][:2] <= xy).all() and (xy <= b[1][:2]).all())
    first = under[-1] if under else None
    add(
        "sink opening is not sealed (first collider below the opening centre is the bowl bottom)",
        first is not None
        and first[1] == "sink_0"
        and abs(first[0] - (top_z - params["table"]["slab"] - s["depth"] + 0.003)) < 1e-3,
        dict(first_hit=first, bowl_bottom=round(top_z - params["table"]["slab"] - s["depth"] + 0.003, 4)),
    )

    mw = cb["microwave"]
    add("microwave stands on the countertop", abs(mw[0][2] - top_z) < 1e-4, dict(bottom=round(float(mw[0][2]), 4)))
    floating = []
    for level, z in zip(("lower", "upper"), params["shelf_top_z"]):
        for n, b in cb.items():
            if n.startswith(f"{level}_") and abs(b[0][2] - z) > 1e-4:
                floating.append(n)
    add("shelf items rest on their shelves", not floating, dict(floating=floating))

    front = min(b[0][1] for n, b in cb.items() if n.startswith("carcass"))
    add(
        "cabinet front below the countertop overhang",
        front > params["table"]["front_y"] - 1e-6,
        dict(cabinet_front_y=round(float(front), 4), counter_front_y=params["table"]["front_y"]),
    )
    # The G1-D chassis extends under the stock (legless) table; here the cabinets reach the floor.
    offset = STOCK_EDGE_Y - params["table"]["front_y"]
    root_min = front + offset - ROBOT_STAND_R
    add(
        "G1-D stock stance clears the cabinets (else move the robot back)",
        G1D_ROOT_Y <= root_min,
        dict(
            room_offset_y=round(offset, 3),
            cabinet_front_in_robodojo=round(float(front + offset), 3),
            stock_root_y=G1D_ROOT_Y,
            max_root_y=round(float(root_min), 3),
            counter_edge_to_root=round(float(STOCK_EDGE_Y - min(G1D_ROOT_Y, root_min)), 3),
        ),
    )

    lounge = params["lounge"]
    tops = {n: round(float(b[1][2]), 4) for n, b in cb.items() if n.startswith("Table_") and n.endswith("_top")}
    add(
        "lounge table tops at table_top_z",
        len(tops) == len(lounge["tables"]) and all(abs(z - lounge["table_top_z"]) < 1e-4 for z in tops.values()),
        dict(table_top_z=lounge["table_top_z"], tops=tops),
    )
    big = {}
    for p in colliders:
        if str(p.GetPath()).startswith("/World/Lounge"):
            n = len({tuple(v) for v in np.round(np.array(UsdGeom.Mesh(p).GetPointsAttr().Get()), 6)})
            if n > 64:
                big[str(p.GetPath())] = n
    add("lounge colliders within the PhysX GPU convex limit (<= 64 vertices)", not big, dict(over=big))

    lights = [p for p in st.Traverse() if p.HasAPI(UsdLux.LightAPI)]
    add("lights present", len(lights) >= 2, dict(lights=[str(p.GetPath()) for p in lights]))

    rep["passed"] = sum(c["ok"] for c in rep["checks"])
    rep["total"] = len(rep["checks"])
    (HERE / "check_report.json").write_text(json.dumps(rep, indent=1, default=str), encoding="utf-8")
    print(f"{rep['passed']}/{rep['total']} checks passed")


if __name__ == "__main__":
    main()
