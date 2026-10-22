"""Per-layout overrides of a reconstructed room (real2sim/scenes/): rebound surface materials and the desk height.

A layout's Room entry may carry, as written by scripts/scenes/appearance.py from the scene's usd/parts.json:
    "appearance": [{"prims": ["Background/Floor", ...], "material": {...}, "uv_scale": 0.6}, ...]
        material {"kind": "preview", "color": [r, g, b] | null, "texture": "<repo-relative path>" | null,
                  "roughness": float, "metallic": float}
              or {"kind": "mdl", "mdl": "$Robodojo_ASSETS/Material/<dir>/<name>.mdl", "tile_m": float}
        uv_scale multiplies the mesh's texture coordinates (preview textures keep their real repeat size); MDL
        materials are projected in world space, one texture repeat per tile_m metres (default 1).
    "table_lift": {"dz": 0.12, "move": [paths], "stretch": [[path, base_z, top_z], ...]}
        the desk top and everything above its telescoping joint rise by dz; legs stretch from their base.
Paths are below the Room prim; z values are in the room's frame. Applied right after the Room is referenced, before
PhysX parses its colliders.
"""

from __future__ import annotations

import os

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from env.global_configs import ROOT_DIR


def apply_room_overrides(stage, room_path: str, config) -> None:
    room = stage.GetPrimAtPath(room_path)
    for i, group in enumerate(config.get("appearance") or []):
        material = _material(stage, f"{room_path}/OverrideLooks/m{i}", group["material"], group.get("uv_scale"))
        for rel in group["prims"]:
            prim = stage.GetPrimAtPath(f"{room_path}/{rel}")
            if prim.IsValid():
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.strongerThanDescendants)
    lift = config.get("table_lift")
    if lift and abs(lift.get("dz", 0.0)) > 1e-6:
        dz = float(lift["dz"])
        for rel in lift.get("move", []):
            _outer_op(stage, room, f"{room_path}/{rel}", Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, dz)))
        for rel, base, top in lift.get("stretch", []):
            s = (top - base + dz) / (top - base)
            m = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, -base))
            m *= Gf.Matrix4d().SetScale(Gf.Vec3d(1, 1, s))
            m *= Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, base))
            _outer_op(stage, room, f"{room_path}/{rel}", m)


def _outer_op(stage, room, path, room_matrix):
    """Apply `room_matrix` (row-vector, in the room's frame) to a prim as its outermost transform op."""
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return
    time = Usd.TimeCode.Default()
    room_world = UsdGeom.Xformable(room).ComputeLocalToWorldTransform(time)
    parent_world = UsdGeom.Xformable(prim.GetParent()).ComputeLocalToWorldTransform(time)
    parent_in_room = parent_world * room_world.GetInverse()
    outer = parent_in_room * room_matrix * parent_in_room.GetInverse()
    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()
    op = xform.AddTransformOp(UsdGeom.XformOp.PrecisionDouble, "tableLift")
    op.Set(outer)
    xform.SetXformOpOrder([op, *ops])


def _material(stage, path, spec, uv_scale):
    if spec["kind"] == "mdl":
        from isaacsim.replicator.behavior.utils.scene_utils import create_mdl_material

        from utils.path import resolve_path

        mdl = resolve_path(spec["mdl"])
        create_mdl_material(mdl, os.path.splitext(os.path.basename(mdl))[0], path)
        material = UsdShade.Material(stage.GetPrimAtPath(path))
        shader = material.ComputeSurfaceSource("mdl")[0]
        if shader:  # OmniPBR world-space projection: independent of the mesh's UVs
            shader.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
            shader.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(True)
            shader.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set(
                Gf.Vec2f(*(2 * [1.0 / float(spec.get("tile_m") or 1.0)]))
            )
        return material
    material = UsdShade.Material.Define(stage, path)
    surface = UsdShade.Shader.Define(stage, f"{path}/Surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(spec.get("roughness") or 0.8))
    surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(spec.get("metallic") or 0.0))
    if spec.get("texture"):
        reader = UsdShade.Shader.Define(stage, f"{path}/StReader")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        st = reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
        if uv_scale and abs(uv_scale - 1.0) > 1e-6:
            scale = UsdShade.Shader.Define(stage, f"{path}/UvScale")
            scale.CreateIdAttr("UsdTransform2d")
            scale.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(st)
            scale.CreateInput("scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(uv_scale, uv_scale))
            st = scale.CreateOutput("result", Sdf.ValueTypeNames.Float2)
        texture = UsdShade.Shader.Define(stage, f"{path}/Texture")
        texture.CreateIdAttr("UsdUVTexture")
        texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(os.path.join(ROOT_DIR, spec["texture"]))
        texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st)
        for wrap in ("wrapS", "wrapT"):
            texture.CreateInput(wrap, Sdf.ValueTypeNames.Token).Set("repeat")
        rgb = texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    else:
        color = spec.get("color") or [0.7, 0.7, 0.7]
        surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    return material
