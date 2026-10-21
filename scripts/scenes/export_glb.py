"""Export a scene twin's usd/scene.usda to preview/scene.glb, for the 3D view on the Real2Sim platform.

Visible render meshes only (guide-purpose collision proxies are skipped), flattened to world space, Z-up turned to
glTF's Y-up. UsdPreviewSurface maps to glTF PBR: diffuse colour or texture, roughness, metallic, opacity (blend, or
mask with opacityThreshold), emissive. USD lights are not exported; the viewer lights the scene itself.
Needs only usd-core and numpy (no Isaac):

    uv run --no-project --with usd-core --with numpy python scripts/scenes/export_glb.py [real2sim/scenes/<dir> ...]

Without arguments, every scene under real2sim/scenes/ that has usd/scene.usda. preview/ is not in git; sync it to the
platform machine's checkout (same path) to show the model there.
"""

import argparse
import json
from pathlib import Path
import struct

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SCENES_DIR = ROOT / "real2sim" / "scenes"
WRAP = {"repeat": 10497, "mirror": 33648, "clamp": 33071, "black": 33071}
Z_UP_TO_Y_UP = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], np.float64)  # (x, y, z) -> (x, z, -y)


def pack_glb(meshes, materials, images):
    """GLB bytes. meshes: [{name, positions (N, 3), normals (N, 3) or None, uv (N, 2) or None, material}], unindexed
    triangle lists; materials: glTF material dicts whose textures index `images`, [(bytes, mime type, wrapS, wrapT)]."""
    blob, views, accessors = bytearray(), [], []

    def add(data, target=None):
        while len(blob) % 4:
            blob.append(0)
        views.append(
            {"buffer": 0, "byteOffset": len(blob), "byteLength": len(data), **({"target": target} if target else {})}
        )
        blob.extend(data)
        return len(views) - 1

    def accessor(array, kind):
        array = np.ascontiguousarray(array, np.float32)
        acc = {"bufferView": add(array.tobytes(), 34962), "componentType": 5126, "count": len(array), "type": kind}
        if kind == "VEC3":
            acc |= {"min": array.min(axis=0).tolist(), "max": array.max(axis=0).tolist()}
        accessors.append(acc)
        return len(accessors) - 1

    gltf_meshes, nodes = [], []
    for mesh in meshes:
        attributes = {"POSITION": accessor(mesh["positions"], "VEC3")}
        if mesh.get("normals") is not None:
            attributes["NORMAL"] = accessor(mesh["normals"], "VEC3")
        if mesh.get("uv") is not None:
            attributes["TEXCOORD_0"] = accessor(mesh["uv"], "VEC2")
        primitive = {"attributes": attributes, "mode": 4}
        if mesh.get("material") is not None:
            primitive["material"] = mesh["material"]
        gltf_meshes.append({"name": mesh["name"], "primitives": [primitive]})
        nodes.append({"name": mesh["name"], "mesh": len(gltf_meshes) - 1})
    samplers, textures, gltf_images = [], [], []
    for data, mime, wrap_s, wrap_t in images:
        gltf_images.append({"bufferView": add(data), "mimeType": mime})
        samplers.append({"wrapS": wrap_s, "wrapT": wrap_t})
        textures.append({"sampler": len(samplers) - 1, "source": len(gltf_images) - 1})
    gltf = {
        "asset": {"version": "2.0", "generator": "RoboDojo scripts/scenes/export_glb.py"},
        "scene": 0,
        "scenes": [{"nodes": list(range(len(nodes)))}],
        "nodes": nodes,
        "meshes": gltf_meshes,
        "materials": materials,
        "accessors": accessors,
        "bufferViews": views,
        "buffers": [{"byteLength": len(blob)}],
    }
    if gltf_images:
        gltf |= {"images": gltf_images, "samplers": samplers, "textures": textures}
    while len(blob) % 4:
        blob.append(0)
    text = json.dumps(gltf, separators=(",", ":")).encode()
    text += b" " * (-len(text) % 4)
    chunks = struct.pack("<II", len(text), 0x4E4F534A) + text + struct.pack("<II", len(blob), 0x004E4942) + bytes(blob)
    return struct.pack("<III", 0x46546C67, 2, 12 + len(chunks)) + chunks


def fan(counts):
    """Corner indices (into the face-varying list) of a fan triangulation of polygons with `counts` corners."""
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    return np.array([[s, s + k, s + k + 1] for s, n in zip(starts, counts) for k in range(1, n - 1)], np.int64).reshape(
        -1
    )


def per_corner(values, interpolation, vertex_indices, counts):
    """A primvar expanded to one value per face corner."""
    values = np.asarray(values, np.float64)
    if interpolation == "faceVarying":
        return values
    if interpolation == "vertex" or interpolation == "varying":
        return values[vertex_indices]
    if interpolation == "uniform":
        return np.repeat(values, counts, axis=0)
    return np.repeat(values[:1], len(vertex_indices), axis=0)  # constant


def export(scene_dir):
    from pxr import Gf, Usd, UsdGeom, UsdShade

    stage = Usd.Stage.Open(str(scene_dir / "usd" / "scene.usda"))
    cache = UsdGeom.XformCache()
    materials, images, by_material, by_file, meshes = [], [], {}, {}, []

    def texture(shader_input):
        """(glTF texture index, the source shader's output name) if the input is connected to a UsdUVTexture."""
        source = shader_input.GetConnectedSource() if shader_input else None
        if not source:
            return None, None
        shader = UsdShade.Shader(source[0].GetPrim())
        if shader.GetIdAttr().Get() != "UsdUVTexture":
            return None, None
        path = Path(shader.GetInput("file").Get().resolvedPath)
        wrap = [
            WRAP.get(shader.GetInput(k).Get() if shader.GetInput(k) else "repeat", 10497) for k in ("wrapS", "wrapT")
        ]
        key = (path, *wrap)
        if key not in by_file:
            mime = "image/png" if path.suffix.lower() == ".png" else "image/jpeg"
            images.append((path.read_bytes(), mime, *wrap))
            by_file[key] = len(images) - 1
        return by_file[key], source[1]

    def material(prim):
        bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        key = str(bound.GetPath()) if bound else None
        if key in by_material:
            return by_material[key]
        pbr, out = {"metallicFactor": 0.0, "roughnessFactor": 0.8}, {"name": key or "default"}
        shader = bound.ComputeSurfaceSource()[0] if bound else None
        if shader and shader.GetIdAttr().Get() == "UsdPreviewSurface":

            def value(name, default):
                inp = shader.GetInput(name)
                v = inp.Get() if inp and not inp.HasConnectedSource() else None
                return default if v is None else v

            color, alpha = list(value("diffuseColor", Gf.Vec3f(0.18))), float(value("opacity", 1.0))
            tex, _ = texture(shader.GetInput("diffuseColor"))
            if tex is not None:
                pbr["baseColorTexture"], color = {"index": tex}, [1.0, 1.0, 1.0]
            pbr |= {"metallicFactor": float(value("metallic", 0.0)), "roughnessFactor": float(value("roughness", 0.5))}
            alpha_tex, channel = texture(shader.GetInput("opacity"))
            if alpha_tex is not None and channel == "a":
                pbr["baseColorTexture"] = {"index": alpha_tex}
                out |= {"alphaMode": "MASK", "alphaCutoff": float(value("opacityThreshold", 0.5))}
            elif alpha < 1:
                out["alphaMode"] = "BLEND"
            pbr["baseColorFactor"] = [*map(float, color), alpha]
            emissive_tex, _ = texture(shader.GetInput("emissiveColor"))
            if emissive_tex is not None:
                out |= {"emissiveTexture": {"index": emissive_tex}, "emissiveFactor": [1.0, 1.0, 1.0]}
            elif any(value("emissiveColor", Gf.Vec3f(0))):
                out["emissiveFactor"] = list(map(float, value("emissiveColor", Gf.Vec3f(0))))
        else:
            color = UsdGeom.PrimvarsAPI(prim).GetPrimvar("displayColor")
            if color and color.HasValue():
                pbr["baseColorFactor"] = [*map(float, color.Get()[0]), 1.0]
        out["pbrMetallicRoughness"] = pbr
        out["doubleSided"] = bool(UsdGeom.Mesh(prim).GetDoubleSidedAttr().Get())
        materials.append(out)
        by_material[key] = len(materials) - 1
        return by_material[key]

    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        image = UsdGeom.Imageable(prim)
        if image.ComputeVisibility() == UsdGeom.Tokens.invisible or image.ComputePurpose() not in ("default", "render"):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = np.array(mesh.GetPointsAttr().Get() or [], np.float64)
        counts = np.array(mesh.GetFaceVertexCountsAttr().Get() or [], np.int64)
        indices = np.array(mesh.GetFaceVertexIndicesAttr().Get() or [], np.int64)
        if not len(points) or not len(counts):
            continue
        world = np.array(cache.GetLocalToWorldTransform(prim), np.float64).T  # column-vector convention
        tris = fan(counts)
        if mesh.GetOrientationAttr().Get() == UsdGeom.Tokens.leftHanded:
            tris = tris.reshape(-1, 3)[:, ::-1].reshape(-1)
        to_gltf = Z_UP_TO_Y_UP @ world[:3, :3]
        positions = (points[indices] @ to_gltf.T + Z_UP_TO_Y_UP @ world[:3, 3])[tris]
        normals = None
        if mesh.GetNormalsAttr().HasValue():
            n = per_corner(mesh.GetNormalsAttr().Get(), mesh.GetNormalsInterpolation(), indices, counts)
            n = n @ np.linalg.inv(to_gltf).T  # normals transform with the inverse transpose
            normals = (n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12))[tris]
        uv = None
        st = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")
        if st and st.HasValue():
            uv = per_corner(st.ComputeFlattened(), st.GetInterpolation(), indices, counts)[tris]
            uv[:, 1] = 1.0 - uv[:, 1]  # glTF's texture origin is the top-left corner
        meshes.append({"name": str(prim.GetPath()), "positions": positions, "normals": normals, "uv": uv,
                       "material": material(prim)})  # fmt: skip
    out = scene_dir / "preview" / "scene.glb"
    out.parent.mkdir(exist_ok=True)
    out.write_bytes(pack_glb(meshes, materials, images))
    return out, len(meshes)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("scenes", nargs="*", type=Path, help="scene directories (default: all under real2sim/scenes/)")
    args = parser.parse_args()
    dirs = args.scenes or sorted(d for d in SCENES_DIR.iterdir() if (d / "usd" / "scene.usda").is_file())
    for d in dirs:
        out, count = export(d.resolve())
        print(f"{out.relative_to(ROOT)}: {count} meshes, {out.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
