# real_scene5：深圳实验室办公桌 建模说明

v1（2026-10-21），依据平台登记 `registration.json`（卷尺实测）和 6 张手机照片 `reference/IMG_4913–4918.jpeg`（iPhone Air 主摄，26 mm 等效）。场景变更时同步更新本文件。

## 文件

| 文件 | 作用 |
| --- | --- |
| `registration.json` | 平台登记（卷尺实测、桌子、环境、灯光），原样保留 |
| `fit_photo.py` | 5 张照片上手标的桌沿、立柱、桌脚和柜角，桌面尺寸固定为卷尺值，联合反求相机位姿、共享焦距、桌腿和柜子位置，写 `fit_photo.json` |
| `build_scene.py` | 读 `fit_photo.json`，生成 `usd/`（`background.usda`、`table.usda`、`cabinet.usda`、`lights.usda`、`scene.usda`、地毯贴图）和 Room 入口 `real_scene5.usd` |
| `check_scene.py` | 离线检查，写 `check_report.json`（22/22 通过） |
| `preview.py` | numpy 光栅化预览（`preview/`，不入库）。`compare_photo.jpg` 每行：照片、拟合相机下的渲染、桌子和柜子的模型边线叠在照片上；只核对几何 |

运行（本机无 Isaac 也可）：

```bash
uv run --no-project --python 3.11 --with usd-core --with trimesh --with numpy --with shapely \
    --with opencv-python-headless --with scipy --with mapbox-earcut --with manifold3d \
    sh -c "python fit_photo.py && python build_scene.py && python check_scene.py && python preview.py"
```

## 坐标系

与 real_scene1–3 一致：Z 向上，米制；原点在桌面中心正下方的地面；+X 沿桌子长边（机器人右手方向），+Y 指向桌后的墙。机器人站在 −Y 一侧、面向 +Y。桌面前沿 y = −0.30，后沿 y = +0.30 贴墙。

照片机位（拟合结果）：4913 左前、4914 正前偏右、4916 正前近处、4917 −X 端、4918（竖拍）右前。4915 和 4914 几乎同一机位，没有参与拟合。

## 尺寸与依据

| 项 | 取值 | 依据 |
| --- | --- | --- |
| 桌面高 × 长 × 宽 × 厚 | 0.75 × 1.20 × 0.60 m，2 cm | **卷尺**（2026-10-21），拟合时固定 |
| 桌面圆角 | R 5 cm | 照片目测 |
| 桌后沿到墙 | 0 | 登记 |
| 桌腿 | 两条 T 形升降腿：立柱 x = ±0.472、y = +0.005；脚 y ∈ [−0.243, 0.251] | 拟合（立柱边线、脚端帽） |
| 立柱截面、两节分界高 0.45 m、侧梁、横梁 | 5–6 cm 等 | 目测 |
| 储物柜 | 0.75 × 0.45 × 1.20 m（宽 × 深 × 高），x ∈ [1.227, 1.977]，背贴墙 | 尺寸登记；位置拟合（柜顶角、前左立边） |
| 踢脚线 | 高 0.08 m，黑色 | 目测 |
| 房间 | x ∈ [−2.5, 2.5]，y ∈ [−3.0, 0.30]，高 2.8 m | 后墙登记；侧墙、前墙、层高没拍到，放在视野外 |

拟合残差：桌沿 ≤ 3.7 px，桌脚和柜角 ≤ 8 px（端帽、柜顶盖较模糊，sigma 取 4 px）。焦距拟合 773.6 px（1024 px 宽），与 EXIF 26 mm 等效换算的 769 px 一致，说明卷尺尺度和照片自洽。4916 里柜子的手标点和 4914 底边（被袋子挡住）误差 8–100 px，是标错了，已剔除；剔除后模型的柜子在 4916 上也对得上（见 `preview/compare_photo.jpg`）。

## 和登记不一致的地方

- 登记写“四条腿”，照片是两条 T 形升降腿（立柱 + 落地横脚），按照片建。
- 登记写“灰色条纹瓷砖”，照片是蓝灰条纹方块地毯，用 4913 里一块 0.3 m 的干净区域按地面单应性矫正、镜像平铺（亮度乘 0.67，同 real_scene3）。
- 桌上有一块 ChArUco 标定板，但没登记格子边长和位置，没有用。下次采集按 `real2sim/scenes/SKILL.md` 量格子边长，可以用 `charuco.py solve` 复核相机。

## 灯光（`lighting.status: recorded`）

登记：室内顶部顶条，面光源，颜色和照片对齐；没有色温、照度和位置。建模为 4 条 1.2 × 0.15 m 的 RectLight 贴天花板（桌子正上方、机器人上方、左右各一条），6000 K，强度 54000，加 Dome 150。Isaac 相机没有白平衡，墙的乳白、桌子和柜子的颜色写在材质里（从 4916 取 sRGB）。

颜色按 Isaac RTX 渲染和照片对齐：把 `cam_third` 放到 4916 的拟合位姿，比墙、桌面、地毯三块区域的 sRGB 中值（照片 / 仿真）：

| 轮次 | 设置 | 墙 | 桌面 | 地毯 |
| --- | --- | --- | --- | --- |
| 照片 | | 206, 203, 196 | 228, 228, 220 | 96, 100, 108 |
| 1 | 6500 K、60000、墙 222、地毯增益 0.67 | 225, 224, 223（偏冷） | 238, 238, 238 | 77, 79, 87 |
| 2 | 6000 K、54000、地毯 0.9 | 225, 222, 216 | 238, 237, 236 | 105, 105, 110 |
| 3（采用） | 墙 205、地毯 0.85 | 218, 212, 208 | 238, 236, 235 | 97, 97, 102 |

照片有自动曝光和白平衡，绝对亮度只能对到这个程度。没和头部相机画面对比，所以是 recorded，不是 matched。

## 接入 RoboDojo

- `scripts/g1d/build_asset.py` 的 `SCENES`：`g1d_shenzhen_lab` → Room `real_scene5`，位置 (0, −0.30, 0)，物体平移 (0, 0)。桌面前沿落在 y = −0.60，和办公室、实验室一样。
- 机器人站位**未实测**：`env_cfg/robot/g1d_shenzhen_lab.yml` 根节点 y = −0.73（桌沿在根节点前 0.13 m）、在桌子中线上，这样桌面物体落在 y ≈ [−0.55, −0.25]。桌面比办公室（0.68 m）高 0.07 m，升降柱两节各 0.035。真机站位量到后改这个文件和 `SCENES`，并在 `scene_spec.json` 填 `table_edge_from_robot_root`、`table_centre_x_from_robot_root`。
- 底盘按站立圆柱 r = 0.22 m 检查，不碰桌腿和桌脚（立柱间距 0.94 m）。
- 碰撞体全是盒体（8 顶点、四边形面），满足凸包 ≤ 64 顶点、单面 ≤ 32 顶点。

## 验证记录

- **2026-10-21，5090-2 隔离目录 `/data/robodojo-runs/shenzhen-lab-20261021-125246`**（GPU 3；资产从学校场景运行目录复制，加平台镜像里的 `arx_x5` 布局和物体元数据；共享库没动）：`build_asset.py --assets Assets` 退出 0，`g1d_shenzhen_lab` 生成 739 个 G1-D 任务布局。`scripted_demo.py --env_cfg_type g1d_shenzhen_lab --task_name g1d_pour_beans_measuring_cups --seed 0`：`_result.json` success_rate 1.0、eval_time 1；头部、左右腕、第三视角四路视频都正常（桌子、后墙、踢脚线、地毯、柜子、天花板都在，没有棕色 HDR 漏光）。
- 再把 `cam_third` 放到 4916、4913 的拟合位姿各跑一次（只改运行目录里的 `g1d_camera.yml`，跑完还原），用于上面的颜色对齐，两次都 success_rate 1.0。
- `preview/compare_isaac.jpg`：照片和 Isaac 渲染（Gemini 96° 画面按照片 67° 视场中心裁剪放大）并排；`preview/robot_cameras_isaac.jpg`：最终灯光下四路相机的首、中、末帧（这次 `cam_third` 在 4916 位姿）。

## 未覆盖

- 机器人站位卷尺、头部 / 腕部相机叠图（需要真机画面和关节角）。
- 灯光位置和亮度没记录；未和头部相机对比。
- 柜子右侧的墙体凸出、地上的纸箱和杂物没建；柜门细节是示意。
- 摩擦系数是估计值。
