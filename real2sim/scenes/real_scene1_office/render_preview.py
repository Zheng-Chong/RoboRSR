"""Offline preview renderer for the delivered USD (Open3D/Filament rasterizer, NOT Isaac RTX).

Reads usd/scene.usda, resolves the bound material of every render-purpose mesh with
UsdShade.MaterialBindingAPI.ComputeBoundMaterial, and renders from:
  * the SfM-recovered real cameras (side-by-side with the undistorted real frame), or
  * named free cameras (overview / task area / estimated robot head view).
Lighting here is an approximation of lights.usda (sun + ambient IBL); RTX results will differ.
One Filament renderer per process: run real video frames, photos and free views separately.
"""
import argparse
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import open3d.visualization.rendering as rendering
from pxr import Usd, UsdGeom, UsdLux, UsdShade

HERE = Path(__file__).resolve().parent
REF = HERE / "reference"


def shader_inputs(mat):
    out = dict(color=(0.5, 0.5, 0.5), opacity=1.0, rough=0.5, metal=0.0, texture=None, emissive=False)
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
            src = inp.GetConnectedSource()[0]
            f = UsdShade.Shader(src.GetPrim()).GetInput("file")
            if f and name == "diffuseColor":
                out["texture"] = f.Get().resolvedPath or f.Get().path
            if name == "emissiveColor":
                out["emissive"] = True
        else:
            v = inp.Get()
            if name == "diffuseColor":
                out["color"] = tuple(v)
            elif name == "opacity":
                out["opacity"] = float(v)
            elif name == "roughness":
                out["rough"] = float(v)
            elif name == "metallic":
                out["metal"] = float(v)
    return out


def collect(usd_path, exclude=()):
    stage = Usd.Stage.Open(str(usd_path))
    cache = UsdGeom.XformCache()
    items, sun = [], None
    for prim in stage.Traverse():
        if prim.IsA(UsdLux.DistantLight):
            M = np.array(cache.GetLocalToWorldTransform(prim)).T
            sun = M[:3, 2] / np.linalg.norm(M[:3, 2])
        if not prim.IsA(UsdGeom.Mesh) or any(e in str(prim.GetPath()) for e in exclude):
            continue
        img = UsdGeom.Imageable(prim)
        if img.ComputePurpose() != UsdGeom.Tokens.default_ or img.ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        m = UsdGeom.Mesh(prim)
        pts = np.array(m.GetPointsAttr().Get(), dtype=np.float64)
        idx = np.array(m.GetFaceVertexIndicesAttr().Get(), dtype=np.int64).reshape(-1, 3)
        M = np.array(cache.GetLocalToWorldTransform(prim)).T
        P = pts @ M[:3, :3].T + M[:3, 3]
        fv = P[idx.reshape(-1)]
        nrm = m.GetNormalsAttr().Get()
        if nrm is not None and len(nrm):
            N = np.array(nrm, dtype=np.float64) @ M[:3, :3].T
            N = N[idx.reshape(-1)] if m.GetNormalsInterpolation() == UsdGeom.Tokens.vertex else N
        else:
            N = np.repeat(np.cross(P[idx[:, 1]] - P[idx[:, 0]], P[idx[:, 2]] - P[idx[:, 0]]), 3, axis=0)
        N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-9)
        uv = None
        pv = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")
        if pv and pv.HasValue():
            uv = np.array(pv.Get(), dtype=np.float64)
        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        items.append(dict(path=str(prim.GetPath()), fv=fv, N=N, uv=uv, mat=shader_inputs(mat),
                          mat_path=str(mat.GetPath()) if mat else ""))
    return items, sun


class Renderer:
    def __init__(self, items, sun, w, h, sun_lux=95000.0, ibl=22000.0):
        self.w, self.h = w, h
        self.r = rendering.OffscreenRenderer(w, h)
        sc = self.r.scene
        sc.set_background([0.0, 0.0, 0.0, 1.0])
        tex_cache = {}
        for k, it in enumerate(items):
            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector(it["fv"])
            mesh.triangles = o3d.utility.Vector3iVector(np.arange(len(it["fv"])).reshape(-1, 3))
            mesh.vertex_normals = o3d.utility.Vector3dVector(it["N"])
            mr = rendering.MaterialRecord()
            si = it["mat"]
            mr.base_color = list(si["color"]) + [si["opacity"]]
            mr.base_roughness = si["rough"]
            mr.base_metallic = si["metal"]
            mr.shader = "defaultLitTransparency" if si["opacity"] < 0.99 else "defaultLit"
            if si["texture"] and it["uv"] is not None:
                uv = it["uv"].copy()
                uv[:, 1] = 1.0 - uv[:, 1]
                mesh.triangle_uvs = o3d.utility.Vector2dVector(uv)
                tp = si["texture"]
                if tp not in tex_cache:
                    tex_cache[tp] = o3d.io.read_image(tp)
                mr.albedo_img = tex_cache[tp]
                mr.base_color = [1.0, 1.0, 1.0, 1.0]
                if si["emissive"]:
                    mr.shader = "defaultUnlit"
            sc.add_geometry(f"m{k}", mesh, mr)
        if sun is not None:
            sc.scene.set_sun_light((-sun).tolist(), [1.0, 0.95, 0.88], sun_lux)
            sc.scene.enable_sun_light(True)
        sc.scene.enable_indirect_light(True)
        sc.scene.set_indirect_light_intensity(ibl)
        sc.view.set_shadowing(True, rendering.View.ShadowType.PCF)

    def render(self, K, R, t):
        Kmat = np.array([[K[0], 0, K[2]], [0, K[1], K[3]], [0, 0, 1]])
        E = np.eye(4)
        E[:3, :3] = R
        E[:3, 3] = t
        self.r.setup_camera(Kmat, E, self.w, self.h)
        self.r.scene.camera.set_projection(Kmat, 0.03, 60.0, float(self.w), float(self.h))
        shots = [np.asarray(self.r.render_to_image()) for _ in range(3)]
        img = max(shots, key=lambda s: float(s.mean()))
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def look_at(eye, target, up=(0, 0, 1)):
    eye, target, up = (np.asarray(v, dtype=np.float64) for v in (eye, target, up))
    f = target - eye
    f /= np.linalg.norm(f)
    r = np.cross(f, up)
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    R = np.stack([r, -u, f])
    return R, -R @ eye


FREE_CAMS = {
    "overview_ne": ((2.0, -1.9, 2.6), (-0.3, 0.3, 0.4), 75),
    "overview_sw": ((-2.4, 1.6, 2.5), (0.3, -0.5, 0.5), 75),
    "top_down": ((0.0, 0.0, 7.5), (0.0, 0.001, 0.0), 50),
    "task_area": ((0.0, -0.95, 1.35), (0.0, 0.0, 0.70), 60),
    "robot_head_est": ((0.0, -0.61, 1.22), (0.0, 0.05, 0.68), 87),
}


def edges_overlay(real, img):
    e = cv2.Canny(cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 9), 20, 60)
    ov = real.copy()
    ov[cv2.dilate(e, np.ones((2, 2), np.uint8)) > 0] = (0, 0, 255)
    return ov


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", default=str(HERE / "usd" / "scene.usda"))
    ap.add_argument("--real", nargs="*", default=[])
    ap.add_argument("--free", nargs="*", default=[])
    ap.add_argument("--out", default=str(HERE / "preview"))
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--tag", default="")
    a = ap.parse_args()
    if a.real and a.free:
        sys.exit("render real and free cameras in separate runs (one Filament renderer per process)")
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    items, sun = collect(a.usd)
    print("meshes", len(items))
    cams = json.loads((REF / "cameras_scene.json").read_text(encoding="utf-8"))
    sizes = {int(round(cams[k]["height"] * a.width / cams[k]["width"])) for k in a.real}
    if len(sizes) > 1:
        sys.exit("all --real cameras in one run must share an aspect ratio (video frames vs photos)")
    if a.real:
        h = sizes.pop()
        rd = Renderer(items, sun, a.width, h)
        for key in a.real:
            c = cams[key]
            s = a.width / c["width"]
            img = rd.render((c["fx"] * s, c["fy"] * s, c["cx"] * s, c["cy"] * s),
                            np.array(c["R_cam_from_scene"]), np.array(c["t_cam_from_scene"]))
            real = cv2.resize(cv2.imread(str(REF / f"{key}.jpg")), (a.width, h))
            cv2.imwrite(str(out / f"cmp_{key}{a.tag}.jpg"), np.hstack([real, img, edges_overlay(real, img)]))
            cv2.imwrite(str(out / f"render_{key}{a.tag}.jpg"), img)
            cv2.imwrite(str(out / f"ov_{key}{a.tag}.jpg"), edges_overlay(real, img))
    if a.free:
        w, h = a.width, int(a.width * 9 / 16)
        rd = Renderer(items, sun, w, h)
        for key in a.free:
            eye, tgt, fov = FREE_CAMS[key]
            f = (w / 2) / np.tan(np.radians(fov) / 2)
            R, t = look_at(eye, tgt, up=(0, 1, 0) if key == "top_down" else (0, 0, 1))
            cv2.imwrite(str(out / f"view_{key}{a.tag}.jpg"), rd.render((f, f, w / 2, h / 2), R, t))
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
