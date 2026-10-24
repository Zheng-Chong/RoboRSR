"""Randomization profile of an upstream RoboDojo task: which of the engine's random factors a generated layout varies.

Written by the Real2Sim platform (task drawer, "泛化随机方案") to real2sim/benchmark_tasks/<task>/randomization.json
and read by scripts/datagen/x5_layouts.py. Stdlib only, so the platform server can import it.

    {"pool": "train",                                           # train (data agent) | exam (exam agent), rsi/AGENTS.md
     "position": true, "yaw": true, "instances": true,          # resampled within the task's own ranges / the pool
     "color": true, "scale": [0.9, 1.1],                        # task objects, only where x5_layouts.RESTYLE allows
     "rooms": [...], "tables": [...], "grounds": [...],         # Assets/Room, Assets/Material dirs; [] keeps the task's
     "background": {"hdrs": [...], "intensity": [lo, hi]},      # Assets/Background files
     "lights": {"count": [lo, hi], "types": ["Sphere"], "intensity": [lo, hi], "color_temperature": [lo, hi]},
     "clutter": {"count": [lo, hi], "categories": []},          # distractor categories of the pool, [] = all
     "camera": {"ori_deg": 2.0}}                                # head camera, per parallel env at start-up
A key that is missing or null leaves that factor as the template layout has it. Light intensity is a multiple of the
type's LIGHT_REF. Every pickable asset comes from rsi/assets/pools.json (scripts/datagen/rsi_assets.py): the ones the
official training data already shows, plus the profile's own pool; the benchmark's reserved assets never appear.
"""

from pathlib import Path

import rsi_assets

ROOT = Path(__file__).resolve().parents[2]
PROFILES_DIR = ROOT / "real2sim" / "benchmark_tasks"  # <task>/randomization.json
LIGHT_TYPES = ("Sphere", "Disk", "Rect", "Cylinder", "Distant")
LIGHT_REF = {"Sphere": 3e4, "Disk": 3e4, "Rect": 3e4, "Cylinder": 3e4, "Distant": 1.5e3}
FLAGS = ("position", "yaw", "instances", "color")
POOLS = ("rooms", "tables", "grounds")
LIMITS = {  # (low, high) any range value must stay inside
    "background.intensity": (0, 1e5),
    "lights.count": (0, 8),
    "lights.intensity": (0.05, 20),
    "lights.color_temperature": (1500, 15000),
    "clutter.count": (0, 40),
    "scale": (0.5, 1.5),
}


def object_dir(assets, asset_id):
    """Object/RoboDojo/<Section or Clutter>/<category>/<index> of an object asset id."""
    kind, _, name = asset_id.partition(":")
    category, index = name.split("/")
    section = "Clutter" if kind == "clutter" else kind
    return Path(assets) / "Object" / "RoboDojo" / section / category / f"{int(index):05d}"


def options(assets, pool="train", pools=None):
    """What a profile of `pool` may pick from in `assets` (the Assets/ root)."""
    assets, pools = Path(assets), pools or rsi_assets.load()
    ok = rsi_assets.allowed(pools, pool)

    def names(kind, found):
        return sorted(n for n in found if f"{kind}:{n}" in ok)

    def objects(ids):
        return sorted(i for i in ids if (object_dir(assets, i) / "metadata.json").is_file())

    distractors = objects(pools["pools"][pool]["distractors"])
    return {
        "pool": pool,
        "rooms": names("room", (p.name for p in (assets / "Room").glob("*") if p.is_dir())),
        "materials": names("material", (p.name for p in (assets / "Material").glob("material*") if p.is_dir())),
        "hdrs": names("hdr", (p.name for p in (assets / "Background").glob("*.hdr"))),
        "distractors": distractors,
        "clutter": sorted({i.partition(":")[2].split("/")[0] for i in distractors}),
        "instances": objects(pools["pools"][pool]["instances"]),
        "reserved": sorted(pools["t0"]),
        "light_types": list(LIGHT_TYPES),
    }


def _range(value, key):
    low, high = LIMITS[key]
    if not (isinstance(value, list) and len(value) == 2 and all(isinstance(v, int | float) for v in value)):
        return f"{key} 要写成 [最小, 最大]"
    if not low <= value[0] <= value[1] <= high:
        return f"{key} 超出范围：要求 {low} ≤ 最小 ≤ 最大 ≤ {high}"
    return None


def check(profile, opts):
    """Error message for an unusable profile (checked against options()), or None."""
    if not isinstance(profile, dict):
        return "随机方案要是一个对象"
    unknown = set(profile) - {*FLAGS, *POOLS, "pool", "scale", "background", "lights", "clutter", "camera"}
    if unknown:
        return f"不认识的字段：{', '.join(sorted(unknown))}"
    if profile.get("pool", "train") != opts["pool"]:
        return f"随机方案的 pool 是 {profile.get('pool', 'train')}，可选项却是按 {opts['pool']} 给的"
    for flag in FLAGS:
        if not isinstance(profile.get(flag, False), bool):
            return f"{flag} 只能是 true / false"
    if profile.get("scale") is not None and (error := _range(profile["scale"], "scale")):
        return error
    pick = {"rooms": opts["rooms"], "tables": opts["materials"], "grounds": opts["materials"]}
    for key, allowed in pick.items():
        values = profile.get(key) or []
        if not isinstance(values, list) or (missing := [v for v in values if v not in allowed]):
            shown = ", ".join(map(str, missing if isinstance(values, list) else [values]))
            return f"{key} 里有不能用的项（资产库没有，或是榜单保留 / 对方池子的资产）：{shown}"
    if bg := profile.get("background"):
        if missing := [h for h in bg.get("hdrs") or [] if h not in opts["hdrs"]]:
            return f"背景里有不能用的 HDR：{', '.join(missing)}"
        if bg.get("intensity") is not None and (error := _range(bg["intensity"], "background.intensity")):
            return error
    if lights := profile.get("lights"):
        types = lights.get("types") or []
        if not types or [t for t in types if t not in LIGHT_TYPES]:
            return f"灯光类型要从 {', '.join(LIGHT_TYPES)} 里选至少一种"
        for key in ("count", "intensity", "color_temperature"):
            if lights.get(key) is not None and (error := _range(lights[key], f"lights.{key}")):
                return error
        if lights.get("count") is None:
            return "灯光要写数量范围"
    if clutter := profile.get("clutter"):
        if clutter.get("count") is None or (error := _range(clutter["count"], "clutter.count")):
            return error or "干扰物要写数量范围"
        if missing := [c for c in clutter.get("categories") or [] if c not in opts["clutter"]]:
            return f"干扰物池没有这些类别：{', '.join(missing)}"
    if camera := profile.get("camera"):
        # a head-camera shift of 2 cm renders 3 of 10 parallel envs black on the X5 stand (5090-2, 2026-10-23)
        if set(camera) != {"ori_deg"}:
            return "相机扰动只支持角度（ori_deg）：平移头部相机会让部分环境出图全黑"
        ori = camera["ori_deg"]
        if not isinstance(ori, int | float) or not 0 <= ori <= 15:
            return "相机扰动要求 0 ≤ 角度 ≤ 15°"
    return None


def holdout_split(task):
    """(base task, its _random variant or None) of an upstream task name; the variant's own instances are held out."""
    base = task.removesuffix("_random")
    variant = f"{base}_random"
    config = ROOT / "task" / "RoboDojo" / "config"
    return base, variant if (config / f"{variant}.yml").is_file() else None
