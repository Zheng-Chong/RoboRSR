"""Find the floor, walls and tabletop of a built scene twin, their materials, and how its desk lifts.

Writes real2sim/scenes/<dir>/usd/parts.json from the Room entry (<dir>/<room>.usd), in the room's own frame:
- roles: per role (floor, wall, tabletop) the visual meshes (prim paths below the Room's default prim) with their
  bound material and how many metres one UV unit spans on that mesh (`uv_m`), and the role's materials
  (UsdPreviewSurface colour or texture, roughness; `tile_m` = metres per texture repeat on the original mesh). Found
  by shape: floor = horizontal mesh with its top at z~0; wall = vertical mesh taller than 1.5 m along the room's
  outline, not glass, doors or trim (NOT_WALL); tabletop = the visual mesh under the table group whose top is the
  table's top_z (usd/build_params.json) and no thicker than the slab;
- lift: a height-adjustable desk. Every mesh of the table group, and every mesh resting on its top (scene objects on
  the desk), entirely above `split_z` moves with the top (`move`); meshes crossing it (legs, columns), or ending at it
  (the outer tube), stretch from their base (`stretch`: path, base z, top z). split_z is build_params legs.split_z
  (the telescoping joint) when the scene records one, else half the table height. `range` is the desk's travel in
  metres (LIFT_RANGE, a common dual-motor desk); scenes in FIXED (a floor-standing counter) do not lift.
Layout generation (scripts/scenes/appearance.py) turns a choice of materials and a table height into the Room
overrides the simulator applies (env/scene_manager/objects/room.py).

    uv run --no-project --python 3.11 --with usd-core python scripts/scenes/scene_parts.py real2sim/scenes/real_scene*
    # material library from those parts.json and Assets/Material MDLs (needs Pillow, not usd-core; thumbnails under
    # real2sim/materials/preview/)
    python scripts/scenes/scene_parts.py --library --assets Assets real2sim/scenes/real_scene*
"""

import argparse
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
MATERIALS_DIR = ROOT / "real2sim" / "materials"  # library.json (git) and preview/<id>.jpg (not in git)
LIFT_RANGE = [0.62, 1.27]  # m, top height travel of a common dual-motor standing desk
FIXED = {"real_scene2_counter": "落地吧台柜，不是升降桌"}
TABLE_GROUPS = ("Table", "Counter")
NOT_WALL = r"glass|door|frame|post|mullion|sill|window|frost|skirting|trim"  # in a wall's outline, not its surface
ROLES = ("floor", "wall", "tabletop")
# MDL names -> coarse category for the platform's filter, first match wins
CATEGORIES = [
    (
        "wood",
        r"wood|plank|oak|walnut|maple|cherry|mahogany|bamboo|birch|pine|ash|beech|parquet|veneer|teak|mdf|timber"
        r"|cork|beadboard",
    ),
    ("tile", r"tile|ceramic|porcelain|mosaic|terrazzo"),
    (
        "stone",
        r"marble|granite|stone|concrete|slate|rock|travertine|limestone|cement|asphalt|brick|terracotta|quartz"
        r"|gravel|sand|clay|gypsum|mortar|shingle|retaining|aggregate|hard_court",
    ),
    (
        "fabric",
        r"carpet|fabric|textile|cloth|linen|cotton|wool|felt|leather|velvet|denim|rug|suede|polyester|tweed"
        r"|netting",
    ),
    (
        "metal",
        r"metal|steel|alumin|iron|copper|brass|chrom|zinc|gold|silver|bronze|titanium|nickel|platinum|tungsten"
        r"|mercury|diamond_plate|punched|solder",
    ),
    ("plastic", r"plastic|rubber|vinyl|acrylic|laminate|formica|resin|polymer|caoutchouc|carbon_fiber|tire"),
    ("paint", r"paint|plaster|stucco|wall|drywall|wallpaper|lacquer|enamel"),
    ("paper", r"paper|cardboard"),
    ("outdoor", r"grass|dirt|leaves|mulch|ground"),
    ("glass", r"glass|mirror"),
]


def parent_mesh_paths(stage, root):
    from pxr import Usd, UsdGeom

    return [p for p in Usd.PrimRange(root) if p.IsA(UsdGeom.Gprim)]


def material_info(material, scene_dir):
    """{name, color, texture, roughness} of a UsdPreviewSurface material; texture relative to the repo."""
    from pxr import UsdShade

    info = {"name": material.GetPrim().GetName(), "color": None, "texture": None, "roughness": None, "metallic": None}
    surface = material.ComputeSurfaceSource()[0]
    if not surface:
        return info
    for name in ("roughness", "metallic"):
        value = surface.GetInput(name)
        if value and value.Get() is not None:
            info[name] = round(float(value.Get()), 3)
    diffuse = surface.GetInput("diffuseColor")
    if diffuse:
        if diffuse.HasConnectedSource():
            reader = UsdShade.Shader(diffuse.GetConnectedSource()[0].GetPrim())
            file = reader.GetInput("file")
            path = file.Get().resolvedPath if file and file.Get() else ""
            if path:
                info["texture"] = str(Path(path).resolve().relative_to(ROOT))
        elif diffuse.Get() is not None:
            info["color"] = [round(float(c), 4) for c in diffuse.Get()]
    return info


def uv_metres(mesh):
    """Metres per UV unit on a mesh: the median over its faces' first edge, None without st."""
    from pxr import UsdGeom

    points = mesh.GetPointsAttr().Get()
    st = UsdGeom.PrimvarsAPI(mesh).GetPrimvar("st")
    if not points or not st or not st.Get():
        return None
    uv, interp = st.Get(), st.GetInterpolation()
    counts, indices = mesh.GetFaceVertexCountsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get()
    ratios, corner = [], 0
    for n in counts:
        a, b = indices[corner], indices[corner + 1]
        ua, ub = (uv[corner], uv[corner + 1]) if interp == "faceVarying" else (uv[a], uv[b])
        du = ((ua[0] - ub[0]) ** 2 + (ua[1] - ub[1]) ** 2) ** 0.5
        dp = sum((points[a][i] - points[b][i]) ** 2 for i in range(3)) ** 0.5
        if du > 1e-6 and dp > 1e-6:
            ratios.append(dp / du)
        corner += n
    return round(sorted(ratios)[len(ratios) // 2], 4) if ratios else None


def extract(scene_dir):
    from pxr import Usd, UsdGeom, UsdShade

    scene_dir = Path(scene_dir).resolve()
    entry = next(p for p in scene_dir.glob("real_scene*.usd"))
    stage = Usd.Stage.Open(str(entry))
    root = stage.GetDefaultPrim()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])
    params = json.loads((scene_dir / "usd" / "build_params.json").read_text())
    table_cfg = params.get("table", {})
    rel = lambda p: str(p.GetPath().MakeRelativePath(root.GetPath()))  # noqa: E731

    def box(prim):
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        return [list(r.GetMin()), list(r.GetMax())]

    def binding(prim):
        return UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]

    group = next(root.GetChild(n) for n in TABLE_GROUPS if root.GetChild(n))
    background = root.GetChild("Background")
    floor_box = None
    meshes = {"floor": [], "wall": [], "tabletop": []}
    for prim in parent_mesh_paths(stage, background):
        if not UsdShade.MaterialBindingAPI(prim).GetDirectBinding().GetMaterialPath():
            continue  # collider only
        (x0, y0, z0), (x1, y1, z1) = box(prim)
        if abs(z1) < 0.02 and z1 - z0 < 0.3 and (x1 - x0) * (y1 - y0) > 2:
            meshes["floor"].append(prim)
            floor_box = [x0, y0, x1, y1] if floor_box is None else floor_box
    fx0, fy0, fx1, fy1 = floor_box
    for prim in parent_mesh_paths(stage, background):
        if prim in meshes["floor"] or not UsdShade.MaterialBindingAPI(prim).GetDirectBinding().GetMaterialPath():
            continue
        (x0, y0, z0), (x1, y1, z1) = box(prim)
        thin_x, thin_y = x1 - x0 < 0.12, y1 - y0 < 0.12
        near = lambda v, a, b: min(abs(v - a), abs(v - b)) < 0.3  # noqa: E731
        on_outline = (thin_x and near((x0 + x1) / 2, fx0, fx1)) or (thin_y and near((y0 + y1) / 2, fy0, fy1))
        material = binding(prim)
        names = f"{prim.GetName()} {material.GetPrim().GetName() if material else ''}".lower()
        if z1 - z0 > 1.5 and z0 < 0.3 and on_outline and not re.search(NOT_WALL, names):
            meshes["wall"].append(prim)
    top_z = table_cfg.get("top_z")
    thick = table_cfg.get("thick", table_cfg.get("thickness", 0.06))
    for prim in parent_mesh_paths(stage, group):
        if not UsdShade.MaterialBindingAPI(prim).GetDirectBinding().GetMaterialPath():
            continue
        (x0, y0, z0), (x1, y1, z1) = box(prim)
        if abs(z1 - top_z) < 0.012 and z1 - z0 <= thick + 0.02 and (x1 - x0) * (y1 - y0) > 0.2:
            meshes["tabletop"].append(prim)

    roles = {}
    for role, prims in meshes.items():
        materials, parts = {}, []
        for prim in prims:
            material = binding(prim)
            info = material_info(material, scene_dir) if material else {"name": None}
            uv_m = uv_metres(UsdGeom.Mesh(prim)) if prim.IsA(UsdGeom.Mesh) else None
            parts.append({"path": rel(prim), "material": info["name"], "uv_m": uv_m})
            if info["name"] and info["name"] not in materials:
                materials[info["name"]] = dict(info, tile_m=uv_m)
        roles[role] = {"parts": parts, "materials": list(materials.values())}

    lift = None
    if scene_dir.name not in FIXED:
        split = params.get("legs", {}).get("split_z") or round(top_z / 2, 3)
        tb = box(group)
        move, stretch = [], []
        for prim in parent_mesh_paths(stage, group):
            (_, _, z0), (_, _, z1) = box(prim)
            if z0 >= split - 1e-3:
                move.append(rel(prim))
            elif z1 >= split - 1e-3:  # crossing the joint, or the outer tube ending at it
                stretch.append([rel(prim), round(z0, 4), round(z1, 4)])
        for prim in parent_mesh_paths(stage, root):  # desk objects that are part of the scene
            if prim.GetPath().HasPrefix(group.GetPath()):
                continue
            (x0, y0, z0), (x1, y1, z1) = box(prim)
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            if top_z - 0.005 < z0 < top_z + 0.03 and tb[0][0] < cx < tb[1][0] and tb[0][1] < cy < tb[1][1]:
                move.append(rel(prim))
        lift = {"top_z": top_z, "split_z": split, "range": LIFT_RANGE, "move": move, "stretch": stretch}
    return {
        "scene": scene_dir.name,
        "room": entry.stem,
        "top_z": top_z,
        "roles": roles,
        "lift": lift,
        "fixed_reason": FIXED.get(scene_dir.name),
    }


def category(name):
    low = name.lower()
    return next((c for c, pattern in CATEGORIES if re.search(pattern, low)), "other")


def thumbnail(src, dst, color=None):
    from PIL import Image

    dst.parent.mkdir(parents=True, exist_ok=True)
    if src:
        image = Image.open(src).convert("RGB")
        image.thumbnail((256, 256))
    else:
        image = Image.new("RGB", (64, 64), tuple(int(255 * min(1, c) ** (1 / 2.2)) for c in color))
    image.save(dst, quality=85)


def library(scene_dirs, assets):
    """Scene materials of every role, then each Assets/Material/material_* MDL."""
    entries = []
    for d in scene_dirs:
        parts = json.loads((Path(d) / "usd" / "parts.json").read_text())
        for role, info in parts["roles"].items():
            for m in info["materials"]:
                mid = f"{parts['scene']}/{role}/{m['name']}"
                if any(e["id"] == mid for e in entries):
                    continue
                entry = {"id": mid, "kind": "preview", "name": m["name"], "source": parts["scene"], "role": role}
                entry |= {k: m[k] for k in ("color", "texture", "roughness", "metallic", "tile_m")}
                entry["category"] = "scene"
                entries.append(entry)
                thumb = MATERIALS_DIR / "preview" / f"{mid.replace('/', '__')}.jpg"
                if m["texture"] or m["color"]:
                    thumbnail(ROOT / m["texture"] if m["texture"] else None, thumb, m["color"])
    for d in sorted((Path(assets) / "Material").glob("material_*")):
        mdl = next(d.glob("*.mdl"), None)
        if mdl is None:
            continue
        mid = f"mdl/{d.name}"
        entries.append(
            {"id": mid, "kind": "mdl", "name": mdl.stem, "source": "Assets/Material", "category": category(mdl.stem)}
            | {"mdl": f"$Robodojo_ASSETS/Material/{d.name}/{mdl.name}"}
        )
        images = sorted(d.glob("*[Bb]ase[Cc]olor*")) or sorted(d.glob("*[Aa]lbedo*")) or sorted(d.glob("*diff*"))
        images = [p for p in images if "multi" not in p.name.lower()]  # channel-packed maps (R ao, G grunge, B diff)
        if images:
            thumbnail(images[0], MATERIALS_DIR / "preview" / f"{mid.replace('/', '__')}.jpg")
    return entries


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenes", nargs="+", type=Path, help="real2sim/scenes/<dir>")
    parser.add_argument("--library", action="store_true", help="rebuild real2sim/materials/library.json instead")
    parser.add_argument("--assets", type=Path, default=ROOT / "Assets", help="with --library: for Material/")
    args = parser.parse_args()
    for d in [] if args.library else args.scenes:
        parts = extract(d)
        (d / "usd" / "parts.json").write_text(json.dumps(parts, indent=1, ensure_ascii=False) + "\n")
        counts = {r: len(v["parts"]) for r, v in parts["roles"].items()}
        lift = parts["lift"]
        print(d.name, counts, f"lift move {len(lift['move'])} stretch {len(lift['stretch'])}" if lift else "no lift")
    if args.library:
        entries = library(args.scenes, args.assets)
        MATERIALS_DIR.mkdir(parents=True, exist_ok=True)
        out = MATERIALS_DIR / "library.json"
        out.write_text(json.dumps(entries, indent=1, ensure_ascii=False) + "\n")
        print(f"{len(entries)} materials -> {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
