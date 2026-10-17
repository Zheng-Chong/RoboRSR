"""Package only the final V1 assets, source and selected evidence into a new directory."""

import argparse
from hashlib import sha256
import json
from pathlib import Path
import shutil
import tarfile

CODE_FILES = (
    "__init__.py",
    "build.py",
    "geometry.py",
    "profile.json",
    "smooth_tail.py",
    "tail_profile.json",
    "add_mount_lip.py",
    "mount_lip_profile.json",
    "validate_isaac.py",
    "package.py",
    "requirements.txt",
    "README.md",
    "GITHUB_FILES.txt",
    "SOURCES.json",
    "runtime/g1d.py",
    "runtime/launch.py",
    "tests/test_geometry.py",
)
VALIDATION_FILES = (
    "side.png",
    "front.png",
    "top.png",
    "oblique.png",
    "open.png",
    "middle.png",
    "closed.png",
    "open_middle_closed.png",
    "result.json",
)


def copy(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)


def package(asset_dir, evidence_dir, native_mesh, out, comparison_dir=None, rebuild_report=None):
    asset_dir, evidence_dir, native_mesh, out = (Path(p).resolve() for p in (asset_dir, evidence_dir, native_mesh, out))
    inputs = (asset_dir, evidence_dir, native_mesh.parent, Path(__file__).resolve().parent)
    if out.exists() or any(out == p or out in p.parents or p in out.parents for p in inputs):
        raise ValueError("Output must be a new directory independent of all inputs")
    result = json.loads((evidence_dir / "validation/result.json").read_text())
    if not result.get("passed"):
        raise ValueError("Require a successful final V1 Isaac validation result")
    model_record = json.loads((asset_dir / "build_result.json").read_text())
    if model_record.get("status") != "approximate_geometry_v1_not_calibrated":
        raise ValueError("Only a final V1 build may be packaged")
    out.mkdir(parents=True)
    # The source asset directory must be one final build, never a directory of experimental runs.
    urdfs = {p.name for p in asset_dir.glob("*.urdf")}
    if urdfs != {"g1d_dex1.urdf", "left_gripper.urdf"}:
        raise ValueError(f"Unexpected final URDF set: {urdfs}")
    for p in sorted(asset_dir.rglob("*")):
        if not p.is_file():
            continue
        rel = p.relative_to(asset_dir)
        if "usd" in rel.parts or "__pycache__" in rel.parts:
            continue
        if p.suffix.lower() not in {".urdf", ".stl", ".glb", ".yml", ".json", ".png", ".unitree"}:
            raise ValueError(f"Unexpected asset file: {rel}")
        copy(p, out / "model" / rel)
    for rel in CODE_FILES:
        copy(Path(__file__).parent / rel, out / "code" / rel)
    copy(native_mesh, out / "source_inputs/native_base.STL")
    copy(Path(__file__).with_name("SOURCES.json"), out / "source_inputs/SOURCES.json")
    for rel in VALIDATION_FILES:
        copy(evidence_dir / "validation" / rel, out / "validation" / rel)
    for name in ("official_vs_model.png", "real_vs_model_side.png", "four_views.png"):
        copy(evidence_dir / name, out / "preview" / name)
    for name in ("official.png", "important.png", "side_view_1.jpg"):
        copy(evidence_dir / "references" / name, out / "references" / name)
    if (evidence_dir / "references/advanced_product.png").is_file():
        copy(evidence_dir / "references/advanced_product.png", out / "references/advanced_product.png")
    if comparison_dir:
        for source, target in (
            ("dex1_initial_vs_delivery_v1.png", "initial_vs_final.png"),
            ("dex1_initial_versions_vs_delivery_v1.png", "initial_versions_vs_final.png"),
        ):
            copy(Path(comparison_dir) / source, out / "preview" / target)
    if rebuild_report:
        copy(Path(rebuild_report), out / "validation/rebuild_check.json")
    for name in ("tail_before_after.png", "mount_before_after.png", "mount_detail_comparison.png"):
        if (evidence_dir / name).is_file():
            copy(evidence_dir / name, out / "preview" / name)
    # Preserve source hashes while omitting machine-specific absolute source paths from the handoff.
    record = json.loads((out / "model/build_result.json").read_text())
    record["source_hashes"] = {
        f"source_{i:02d}/{Path(name).name}": digest for i, (name, digest) in enumerate(record["source_hashes"].items())
    }
    (out / "model/build_result.json").write_text(json.dumps(record, indent=2) + "\n")
    copy(Path(__file__).with_name("profile.json"), out / "model/geometry_mount.json")
    readme = (
        "# Dex1-1 Advanced Camera Model V1\n\n"
        "经过初步仿真验证的近似几何模型，不是精确标定版本。\n\n"
        "先看 `preview/official_vs_model.png`、`preview/real_vs_model_side.png`；"
        "四视图及开合截图在 `validation/`。\n\n"
        "- `model/`：同一最终 V1 的完整机器人和独立左夹爪 URDF、Visual/Collision STL、GLB。\n"
        "- `code/`：最终生成、验证、打包代码；相机参数 `profile.json` 与尾部参数 `tail_profile.json`。\n"
        "- `source_inputs/`：重新生成所需的原始 CAD 输入，不是另一个交付模型。\n"
        "- `references/`：官方产品图、尺寸图及实物侧视照片。\n"
        "- `MANIFEST.json`：包内文件 SHA256；旧模型及旧拟合入口未打包。\n\n"
        "## 接入 RoboRSR\n\n"
        "将本目录放在仓库的 `output/dex1_camera_delivery_v1/`，进入已有 Isaac Lab 环境，"
        "从 RoboRSR 仓库根目录使用临时启动器：\n\n"
        "```bash\nPYTHONPATH=.:XPolicyLab EVAL_NUM=1 python "
        "output/dex1_camera_delivery_v1/code/runtime/launch.py "
        "scripts/g1d/scripted_demo.py --env_cfg_type g1d --task_name g1d_stack_blocks --seed 0\n```\n\n"
        "启动器只对本进程选择独立模型，不覆盖共享资产或项目加载器。示例任务仍需现有场景资产，"
        "不表示叠块成功率已验收。完整重建及验证命令见 `code/README.md`。\n\n"
        "## 验证与限制\n\n"
        "完整 G1-D 导入、夹爪开/中/闭驱动和三开度几何碰撞检查已通过。"
        "完整结果见 `validation/result.json` 与 `model/build_result.json`。"
        "未改相机内外参、控制参数或共享资产。新增后壳深度 10 mm、圆角及安装脚厚度 1.5 mm "
        "是工程近似；惯量沿用原值，线缆和小紧固件简化。未验收精密尺寸、完整抓取或真机迁移。\n"
    ).replace("dex1_camera_delivery_v1/", f"{out.name}/")
    if record.get("mount_lip_adaptation"):
        readme += (
            "\n本包为可选的浅包边版本，基于 v1_smooth，仅在原安装脚后沿增加一段向下短折边。"
            "见 `preview/mount_before_after.png` 和 `preview/mount_detail_comparison.png`。"
            "新增下折深度约 4 mm、底部倒角约 0.8 mm 为外观近似；宽度、平台接触面和上沿来自原 CAD。"
            "原相机位置、倾角、支架、平台、光滑尾部及全部已有 STL 文件保持；仅增加两个新网格"
            "及其 URDF 引用。没有补造隐藏卡槽或紧固件。原 v1_smooth 交付目录及压缩包完整保留。"
            "重建和接入见 `code/README.md` 的可选折边版本一节。\n"
        )
    elif record.get("tail_adaptation"):
        readme += (
            "\n本版仅将上一交付的尾部双环外形改为光滑圆柱，见 `preview/tail_before_after.png`。"
            "半径 25 mm 来自原 CAD 中间截面，不是新增实测。相机外壳、竖直支架、安装脚、"
            "手指几何、关节和控制/标定参数保持；仅同步替换尾部碰撞体，惯量仍沿用原值。"
            "重建需依次执行 `build.py` 和 `smooth_tail.py`，见 `code/README.md`。\n"
        )
    (out / "README.md").write_text(readme)
    manifest = {
        str(p.relative_to(out)): sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob("*")) if p.is_file()
    }
    (out / "MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
    archive = out / "deliverable.tar.gz"
    with tarfile.open(archive, "w:gz") as tar:
        for p in sorted(out.rglob("*")):
            if p.is_file() and p != archive:
                tar.add(p, arcname=str(Path(out.name) / p.relative_to(out)))
    digest = sha256(archive.read_bytes()).hexdigest()
    (out / "deliverable.tar.gz.sha256").write_text(f"{digest}  deliverable.tar.gz\n")
    print(f"Packaged {len(manifest)} final files: {archive}\nSHA256 {digest}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-dir", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--native-mesh", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--comparison-dir", type=Path)
    parser.add_argument("--rebuild-report", type=Path)
    args = parser.parse_args()
    package(**vars(args))
