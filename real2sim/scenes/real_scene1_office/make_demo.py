"""Export usd/scene.usda into demo/scene_data.js for the offline WebGL viewer (demo/index.html).

Geometry, normals, UVs and bound materials are read from the delivered USD (ComputeBoundMaterial);
collision meshes (guide purpose) are exported separately so the viewer can toggle them.
Textures are embedded as data URLs because browsers block WebGL textures loaded from file://.
"""
import base64
import json
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

HERE = Path(__file__).resolve().parent
USD = HERE / "usd"
DEMO = HERE / "demo"
REAL_KEYS = ["v0040", "v0100", "v0160", "v0260", "v0280", "v0300", "v0340", "v0400", "v0420", "v0480",
             "p0_25a883", "p1_2a4974", "p2_6dda6d", "p3_c68ebb", "p4_cf220a"]
REAL_LABELS = {"v0040": "视频 0040 · 起点，看笔记本桌", "v0100": "视频 0100 · 看沙发", "v0160": "视频 0160 · 正对沙发窗墙",
               "v0260": "视频 0260 · 沙发与绿植", "v0280": "视频 0280 · 绿植与墙角", "v0300": "视频 0300 · 沿桌长看女同事桌",
               "v0340": "视频 0340 · 女同事桌与柜子", "v0400": "视频 0400 · 隔桌看机器人与电视",
               "v0420": "视频 0420 · 机器人与笔记本桌", "v0480": "视频 0480 · 终点，看笔记本桌",
               "p0_25a883": "照片 P0 · 桌子 +X 端", "p1_2a4974": "照片 P1 · 桌子 -X 端", "p2_6dda6d": "照片 P2 · 沙发侧看机器人",
               "p3_c68ebb": "照片 P3 · 机器人", "p4_cf220a": "照片 P4 · 机器人侧看沙发"}


def b64(a):
    return base64.b64encode(np.ascontiguousarray(a, dtype=np.float32).tobytes()).decode()


def material_info(mat):
    out = dict(color=[0.5, 0.5, 0.5], opacity=1.0, roughness=0.5, metallic=0.0, texture=None, emissive=False)
    if not mat:
        return out
    surf = mat.ComputeSurfaceSource()[0]
    if not surf:
        return out
    for name in ("diffuseColor", "opacity", "roughness", "metallic", "emissiveColor"):
        inp = surf.GetInput(name)
        if not inp:
            continue
        if inp.HasConnectedSource():
            f = UsdShade.Shader(inp.GetConnectedSource()[0].GetPrim()).GetInput("file")
            if f and name == "diffuseColor":
                out["texture"] = Path(f.Get().path).name
            if name == "emissiveColor":
                out["emissive"] = True
        else:
            v = inp.Get()
            if name == "diffuseColor":
                out["color"] = [round(float(c), 4) for c in v]
            elif name in ("opacity", "roughness", "metallic"):
                out[name] = round(float(v), 4)
    return out


def owner_of(path):
    parts = path.strip("/").split("/")
    if parts[1] == "Objects":
        return "物体/" + parts[2]
    if parts[1] == "Table":
        return "任务桌"
    if parts[1] == "Background":
        return "背景/" + (parts[3] if parts[2] == "Colliders" else parts[2])
    return parts[1]


def main():
    stage = Usd.Stage.Open(str(USD / "scene.usda"))
    cache = UsdGeom.XformCache()
    groups, tris = [], 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        img = UsdGeom.Imageable(prim)
        collider = prim.HasAPI(UsdPhysics.CollisionAPI)
        if img.ComputePurpose() != UsdGeom.Tokens.default_ and not collider:
            continue
        m = UsdGeom.Mesh(prim)
        pts = np.array(m.GetPointsAttr().Get(), dtype=np.float64)
        idx = np.array(m.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
        M = np.array(cache.GetLocalToWorldTransform(prim)).T
        P = (pts @ M[:3, :3].T + M[:3, 3])[idx]
        g = dict(path=str(prim.GetPath()), owner=owner_of(str(prim.GetPath())), collider=collider,
                 position=b64(P), count=int(len(idx)))
        if not collider:
            nrm = m.GetNormalsAttr().Get()
            if nrm is not None and len(nrm):
                N = np.array(nrm, dtype=np.float64) @ M[:3, :3].T
                N = N[idx] if m.GetNormalsInterpolation() == UsdGeom.Tokens.vertex else N
                N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-9)
                g["normal"] = b64(N)
            pv = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")
            if pv and pv.HasValue():
                g["uv"] = b64(np.array(pv.Get(), dtype=np.float64))
            mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            g["material"] = material_info(mat)
            g["material_path"] = str(mat.GetPath()) if mat else ""
            g["no_shadow"] = bool(UsdGeom.PrimvarsAPI(prim).GetPrimvar("doNotCastShadows"))
            tris += len(idx) // 3
        groups.append(g)
    textures = {p.name: "data:image/png;base64," + base64.b64encode(p.read_bytes()).decode()
                for p in (USD / "textures").glob("*.png")}
    cams_all = json.loads((HERE / "reference" / "cameras_scene.json").read_text(encoding="utf-8"))
    cams = {k: dict(label=REAL_LABELS[k], width=cams_all[k]["width"], height=cams_all[k]["height"],
                    fx=cams_all[k]["fx"], fy=cams_all[k]["fy"], cx=cams_all[k]["cx"], cy=cams_all[k]["cy"],
                    R=cams_all[k]["R_cam_from_scene"], C=cams_all[k]["center_scene"]) for k in REAL_KEYS}
    params = json.loads((USD / "build_params.json").read_text(encoding="utf-8"))
    report = json.loads((HERE / "check_report.json").read_text(encoding="utf-8"))
    data = dict(name="real_scene1 · Opus 版", groups=groups, textures=textures, cameras=cams,
                sun_to=params["sun_direction_to_sun"], table=params["table"], objects=params["objects"],
                scale=dict(m_per_unit=params["meters_per_sfm_unit"], correction=params["scale_correction"]),
                checks=report["checks"], not_covered=report["not_covered"],
                meshCount=sum(1 for g in groups if not g["collider"]),
                colliderCount=sum(1 for g in groups if g["collider"]), triangleCount=tris)
    DEMO.mkdir(exist_ok=True)
    (DEMO / "scene_data.js").write_text("window.SCENE_DATA=" + json.dumps(data, ensure_ascii=False, separators=(",", ":")) + ";",
                                        encoding="utf-8")
    print(f"scene_data.js: {data['meshCount']} render meshes, {data['colliderCount']} colliders, {tris} triangles, "
          f"{(DEMO / 'scene_data.js').stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
