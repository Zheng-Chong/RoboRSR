# Dex1-1 Advanced Camera Model V1

对应项目 README 的“夹爪（含相机）建模”：当前交付是**经过基本仿真验证的近似几何 V1**，不表示精确数字孪生、相机标定或真机验收完成。

此目录只维护最终方案。保留官方 CAD 镜头、前板、倾斜背壳和主体机械结构，增加后部电子仓、实体竖直支撑及安装脚。不进行 H/D 拟合，不提供旧拟合模型生成入口，不修改相机内外参、机器人控制逻辑或共享资产。

当前交付为 **V1 光滑尾部更新**：仅去除尾部两个突出圆环，以原 CAD 中间段半径（约 25 mm）生成连续圆柱，保留原轴向范围。相机外壳、竖直支架、安装脚和手指保持不变。旧交付目录仅保留备份，新包位于 `output/dex1_camera_delivery_v1_smooth/`。

## 文件与参数

| 文件 | 用途 |
| --- | --- |
| `build.py`、`geometry.py` | 在新的独立目录生成最终模型；拒绝覆盖已有目录 |
| `profile.json` | 相机几何配置；长度单位米，附加旋转单位度，使用 CAD 原始网格坐标系 |
| `smooth_tail.py`、`tail_profile.json` | 在相机 V1 上仅简化尾部双环，拒绝覆盖输入；参数来自 CAD 而非新增实测 |
| `add_mount_lip.py`、`mount_lip_profile.json` | 可选变体：在照片可见的平台后侧增加短折边；不覆盖 v1_smooth |
| `validate_isaac.py` | 完整机器人导入、夹爪开中闭驱动及四视图渲染 |
| `runtime/launch.py`、`runtime/g1d.py` | 仅当前进程使用独立资产，保留原驱动设置 |
| `package.py` | 通过明确文件清单打包最终模型、代码和精选证据 |
| `tests/test_geometry.py` | 最终配置边界、源 CAD 保持和镜头外形保护检查 |

`profile.json` 中 44 mm 壳体最低位置、沿原始网格 +Y 的 19 mm 位移是工程安装估计，不是直接套用 H/D。原 CAD 倾角保留。新增后壳深度 10 mm、圆角 1.5 mm、安装脚厚度 1.5 mm 也是工程近似。惯量沿用原值，线缆及小紧固件简化。

## 在仓库里的用法

`scripts/g1d/build_asset.py` 默认生成带相机的夹爪：以 `real2sim/robot/g1d_v4/meshes/dex1_1/base_link.STL`（即 `SOURCES.json` 记录的官方 CAD，SHA256 一致）为输入，在临时目录里依次运行本目录的 `build.build`（按 `profile.json` 重组相机模组）、`smooth_tail.smooth_tail`（按 `tail_profile.json` 把手腕侧尾部的双环换成光滑圆柱）和 `add_mount_lip.add_mount_lip`（按 `mount_lip_profile.json` 加安装脚后沿折边）。每一步都保留作者的输入检查、逐字节保护和开/中/闭三开度外壳与手指碰撞检查。最后只把两个 `*_dex1_model_base_link` 的视觉/碰撞节点和 `meshes/camera_geometry/` 搬进 `Assets/Robots/g1d/`。改几何只需改对应的 profile 后重跑 `build_asset.py`。

腕部相机传感器（`scripts/g1d/*_robot_config.yml` 的 `cam_wrist`）用宇树相机合同位姿 `(0.0627, 0, 0.0555)`、俯 30°，不随本模型变化；模型镜头前端中心在 `(72.4, 0, 49.5)` mm，正好在该光轴上、传感器前 11 mm。外壳在传感器 3 cm 近裁剪面之内，不进腕部画面；头部画面里能看到夹爪上方的相机模组，和真机一致。

几何检查（需要 numpy/scipy/trimesh/manifold3d/matplotlib，仓库根目录执行）：

```bash
python -m pytest scripts/g1d/dex1_camera/tests -q
```

`build.py`、`smooth_tail.py`、`add_mount_lip.py` 也可以单独运行，在新目录生成完整机器人 + 单独左夹爪 URDF、GLB 预览和检查记录（`--out` 必须是新目录；`build.py` 的 `--base-urdf` 需要仍引用 `meshes/dex1_1/base_link.STL` 的 G1-D URDF）。`validate_isaac.py`、`runtime/`、`package.py` 是交付时用的独立验证、单进程换资产启动器和打包工具，常规流程不需要。

## 限制

近似几何，未做精密尺寸测量、完整抓取验收、动力学辨识或真机迁移；惯量沿用原值。相机模组在倾倒类动作里会碰到另一只手或容器（真机也有这块模组），倒豆子脚本需要按它调整，见仓库 `HANDOFF.md`。源 CAD 来自 Unitree `dex1_1_service`（BSD-3-Clause）。
