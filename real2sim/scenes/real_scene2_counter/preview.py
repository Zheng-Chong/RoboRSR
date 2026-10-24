"""Rough preview renders of usd/scene.usda with a tiny numpy rasteriser (Lambert + ambient, textures, alpha).

For checking geometry, placement and textures against reference/photo.jpg only: it does NOT reproduce the USD lights
or RTX shading. Writes preview/compare_photo.jpg (photo | render from an estimated photo viewpoint) and
preview/robot_head.jpg (estimated G1-D head-camera view).
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np
from pxr import Usd, UsdGeom, UsdShade

HERE = Path(__file__).resolve().parent
USD = HERE / "usd"
NEAR = 0.02
LIGHT = np.array([0.25, -0.45, 1.0]) / np.linalg.norm([0.25, -0.45, 1.0])


def srgb(c):
    c = np.clip(c, 0, 1)
    return np.where(c <= 0.0031308, 12.92 * c, 1.055 * c ** (1 / 2.4) - 0.055)


def load_scene():
    st = Usd.Stage.Open(str(USD / "scene.usda"))
    cache = UsdGeom.XformCache()
    tex_cache, out = {}, []
    for p in st.Traverse():
        if not p.IsA(UsdGeom.Mesh) or UsdGeom.Imageable(p).GetPurposeAttr().Get() == UsdGeom.Tokens.guide:
            continue
        mesh = UsdGeom.Mesh(p)
        pts = np.array(mesh.GetPointsAttr().Get(), np.float64)
        m = np.array(cache.GetLocalToWorldTransform(p)).T
        pts = pts @ m[:3, :3].T + m[:3, 3]
        faces = np.array(mesh.GetFaceVertexIndicesAttr().Get(), np.int64).reshape(-1, 3)
        pv = UsdGeom.PrimvarsAPI(mesh).GetPrimvar("st")
        st_uv = np.array(pv.Get(), np.float64).reshape(-1, 3, 2) if pv and pv.HasValue() else None
        mat = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
        sh = UsdShade.Shader(st.GetPrimAtPath(mat.GetPath().AppendChild("Shader")))
        color, tex, opacity = np.array([0.7, 0.7, 0.7]), None, 1.0
        dc = sh.GetInput("diffuseColor")
        if dc.HasConnectedSource():
            tx = UsdShade.Shader(st.GetPrimAtPath(mat.GetPath().AppendChild("Texture")))
            f = str(tx.GetInput("file").Get().resolvedPath)
            if f not in tex_cache:
                t = cv2.imread(f, cv2.IMREAD_UNCHANGED)
                tex_cache[f] = cv2.cvtColor(t, cv2.COLOR_BGRA2RGBA if t.shape[2] == 4 else cv2.COLOR_BGR2RGB) / 255
            tex = tex_cache[f]
        else:
            color = srgb(np.array(dc.Get(), np.float64))
        op = sh.GetInput("opacity")
        if op and op.Get() is not None:
            opacity = float(op.Get())
        out.append(dict(tris=pts[faces], uv=st_uv, color=color, tex=tex, opacity=opacity))
    return out


def look_at(eye, target, up=(0, 0, 1)):
    f = np.asarray(target, float) - eye
    f /= np.linalg.norm(f)
    r = np.cross(f, up)
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return np.stack([r, -u, f])  # rows: camera x (right), y (down), z (forward)


def clip_near(cam, uv):
    """Clip a camera-space triangle against z > NEAR; returns a list of (cam, uv) triangles."""
    poly = list(zip(cam, uv))
    out = []
    for i in range(len(poly)):
        (a, ta), (b, tb) = poly[i], poly[(i + 1) % len(poly)]
        ina, inb = a[2] > NEAR, b[2] > NEAR
        if ina:
            out.append((a, ta))
        if ina != inb:
            t = (NEAR - a[2]) / (b[2] - a[2])
            out.append((a + t * (b - a), ta + t * (tb - ta)))
    return [
        ((out[0][0], out[k][0], out[k + 1][0]), (out[0][1], out[k][1], out[k + 1][1])) for k in range(1, len(out) - 1)
    ]


def render(scene, eye, target, hfov_deg, size=(800, 600), bg=(0.0, 0.0, 0.0), R=None):
    w, h = size
    fx = w / 2 / math.tan(math.radians(hfov_deg) / 2)
    if R is None:
        R = look_at(np.asarray(eye, float), target)
    img = np.empty((h, w, 3), np.float32)
    img[:] = bg
    depth = np.full((h, w), np.inf, np.float32)
    passes = [[o for o in scene if o["opacity"] >= 1], [o for o in scene if o["opacity"] < 1]]
    for transparent, objs in enumerate(passes):
        for o in objs:
            uvs = o["uv"] if o["uv"] is not None else np.zeros((len(o["tris"]), 3, 2))
            for tri, uv in zip(o["tris"], uvs):
                n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
                if np.linalg.norm(n) < 1e-12:
                    continue
                n /= np.linalg.norm(n)
                if np.dot(n, np.asarray(eye) - tri[0]) < 0:
                    n = -n
                shade = 0.55 + 0.45 * max(0.0, float(np.dot(n, LIGHT)))
                cam = (tri - eye) @ R.T
                for ctri, cuv in clip_near(cam, uv):
                    ctri, cuv = np.array(ctri), np.array(cuv)
                    px = np.c_[fx * ctri[:, 0] / ctri[:, 2] + w / 2, fx * ctri[:, 1] / ctri[:, 2] + h / 2]
                    x0, y0 = np.floor(px.min(0)).astype(int)
                    x1, y1 = np.ceil(px.max(0)).astype(int)
                    x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, w - 1), min(y1, h - 1)
                    if x0 > x1 or y0 > y1:
                        continue
                    ys, xs = np.mgrid[y0 : y1 + 1, x0 : x1 + 1]
                    (ax, ay), (bx, by), (cx, cy) = px
                    den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
                    if abs(den) < 1e-9:
                        continue
                    l0 = ((by - cy) * (xs + 0.5 - cx) + (cx - bx) * (ys + 0.5 - cy)) / den
                    l1 = ((cy - ay) * (xs + 0.5 - cx) + (ax - cx) * (ys + 0.5 - cy)) / den
                    l2 = 1 - l0 - l1
                    inside = (l0 >= 0) & (l1 >= 0) & (l2 >= 0)
                    if not inside.any():
                        continue
                    iz = l0 / ctri[0, 2] + l1 / ctri[1, 2] + l2 / ctri[2, 2]
                    z = 1 / iz
                    vis = inside & (z < depth[y0 : y1 + 1, x0 : x1 + 1])
                    if not vis.any():
                        continue
                    if o["tex"] is not None:
                        u = (
                            l0 * cuv[0, 0] / ctri[0, 2] + l1 * cuv[1, 0] / ctri[1, 2] + l2 * cuv[2, 0] / ctri[2, 2]
                        ) * z
                        v = (
                            l0 * cuv[0, 1] / ctri[0, 2] + l1 * cuv[1, 1] / ctri[1, 2] + l2 * cuv[2, 1] / ctri[2, 2]
                        ) * z
                        th, tw = o["tex"].shape[:2]
                        col = o["tex"][((1 - v[vis]) % 1 * (th - 1)).astype(int), (u[vis] % 1 * (tw - 1)).astype(int)]
                        if col.shape[1] == 4:  # cutout decal
                            keep = col[:, 3] > 0.5
                            vis[vis] = keep
                            col = col[keep, :3]
                    else:
                        col = o["color"][None]
                    region = img[y0 : y1 + 1, x0 : x1 + 1]
                    a = o["opacity"]
                    region[vis] = region[vis] * (1 - a) + col * shade * a
                    if not transparent:
                        depth[y0 : y1 + 1, x0 : x1 + 1][vis] = z[vis]
    return (np.clip(img, 0, 1) * 255).astype(np.uint8)


def main():
    (HERE / "preview").mkdir(exist_ok=True)
    scene = load_scene()
    fit = json.loads((HERE / "fit_photo.json").read_text())  # phone poses fitted by fit_photo.py
    dy = json.loads((USD / "build_params.json").read_text())["table"]["width"] / 2 - 0.30  # fit frame -> scene
    rows = []
    for name in ("photo.jpg", "photo2.jpg"):
        cam = fit[name]
        R = cv2.Rodrigues(np.array(cam["rvec"]))[0]
        eye = -R.T @ np.array(cam["tvec"]) + [0, dy, 0]
        view = render(scene, eye=eye, target=None, hfov_deg=fit["hfov_deg"], size=(1024, 768), R=R)
        rows.append(np.hstack([cv2.imread(str(HERE / "reference" / name)), cv2.cvtColor(view, cv2.COLOR_RGB2BGR)]))
    cv2.imwrite(str(HERE / "preview" / "compare_photo.jpg"), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 85])
    # lounge photos through the cameras of fit_lounge.py (its frame is the scene frame shifted to the lounge corner);
    # the chairs are layout objects, not in the scene USD
    lounge = json.loads((HERE / "fit_lounge.json").read_text())
    params = json.loads((USD / "build_params.json").read_text())
    origin = np.array([params["lounge"]["x_range"][1], params["lounge"]["wall_y"], 0.0])
    hfov = math.degrees(2 * math.atan(512 / lounge["focal_px"]))
    rows = []
    for name, cam in lounge["cameras"].items():
        R = cv2.Rodrigues(np.array(cam["rvec"]))[0]
        eye = -R.T @ np.array(cam["tvec"]) + origin
        view = render(scene, eye=eye, target=None, hfov_deg=hfov, size=(1024, 768), R=R)
        rows.append(np.hstack([cv2.imread(str(HERE / "reference" / name)), cv2.cvtColor(view, cv2.COLOR_RGB2BGR)]))
    cv2.imwrite(str(HERE / "preview" / "compare_lounge.jpg"), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 80])
    plan = render(scene, eye=(-3.0, -2.2, 9.0), target=(-3.0, -2.19, 0.0), hfov_deg=80, size=(1024, 768))
    cv2.imwrite(str(HERE / "preview" / "plan.jpg"), cv2.cvtColor(plan, cv2.COLOR_RGB2BGR))
    # Estimated G1-D head camera: root 0.20 m behind the counter edge, head 0.15 m ahead of the waist, ~1.25 m high,
    # pitched 47.6 deg down (scripts/g1d/left_robot_config.yml), wide-angle ~96 deg h-FOV as in the sim config.
    eye = np.array([0.0, -0.35, 1.25])
    pitch = math.radians(47.6)
    head = render(scene, eye=eye, target=eye + [0, math.cos(pitch), -math.sin(pitch)], hfov_deg=96, size=(640, 480))
    cv2.imwrite(str(HERE / "preview" / "robot_head.jpg"), cv2.cvtColor(head, cv2.COLOR_RGB2BGR))
    top = render(scene, eye=(0.0, -2.6, 2.4), target=(0.0, 0.0, 0.9), hfov_deg=70, size=(1024, 768))
    cv2.imwrite(str(HERE / "preview" / "overview.jpg"), cv2.cvtColor(top, cv2.COLOR_RGB2BGR))
    print("wrote", HERE / "preview")


if __name__ == "__main__":
    main()
