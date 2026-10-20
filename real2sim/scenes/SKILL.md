---
name: real2sim-scene
description: 给真实工位建或更新 RoboDojo 数字孪生场景：现场拍摄（带 ChArUco 标定板）和卷尺测量规范、照片拟合与建模、写回实测、头部/腕部相机叠图检查、按月度目标验收、接入 build_asset.py 和 env 配置。用于：新建场景、按实测修正尺寸、做叠图验收、检查场景验收状态。
---

# 真实场景孪生

验收标准来自 README 月度目标：卷尺实测误差 < 1 cm；用标定过的机器人相机叠图，偏差 < 5 px，且至少 3 个视角；灯光按现场记录建模。每个场景当前的验收状态记在它的 `scene_spec.json` 里，用 `python scripts/scenes/acceptance.py` 汇总。

## 各层放在哪

| 层 | 位置 | 入库 |
| :-- | :-- | :-- |
| 场景源文件 | `real2sim/scenes/<场景>/`：`registration.json`（平台登记）、`reference/` 照片、`fit_photo.py`、`build_scene.py`、`check_scene.py`、`preview.py`、`usd/`、`PLAN.md`、`scene_spec.json` | 是（二进制走 LFS） |
| 共享工具 | `scripts/scenes/`：`charuco.py`（标定板）、`overlay.py`（叠图）、`acceptance.py`（验收汇总） | 是 |
| 接入 | `scripts/g1d/build_asset.py` 的 `SCENES`、`env_cfg/g1d_<场景>.yml`、`env_cfg/robot/g1d_<场景>.yml` | 是 |
| 安装结果 | `Assets/Room/<room>/`、`Assets/Eval_Layout/RoboDojo/<配置>/` | 否，由 `build_asset.py` 生成 |

`scene_spec.json` 只记录人工输入：照片、重建方法、尺度依据、卷尺实测、叠图结果、灯光记录。模型的数值由 `acceptance.py` 从 `usd/build_params.json` 和机器人站位配置里读出来，不要手抄进 spec。

## 平台登记

新场景先在数采平台「场景重建 → 登记场景」登记，三步：① 名称、目录名（`real_scene<N>_<后缀>`，N 自动取下一个）、类型、地点、用途（孪生 / 训练泛化 / L4 保留），至少 5 张不同角度的照片；② 卷尺实测桌面离地高、长、宽（必填，cm），可选桌面厚度、机器人站位（桌沿到根节点、横向偏移、偏角、升降柱读数）、ChArUco 标定板（格子实测边长、板角到桌左沿和前沿的距离）、桌子支撑和桌面外观；③ 周围环境（地面、墙面、后沿到墙、要建模的固定物）、灯光（光源、色温、照度、自然光）、是否已拍机器人相机画面、计划任务、建模要求。

登记写到 `real2sim/scenes/<场景>/registration.json`，照片在同目录的 `reference/`。卷尺值已换成米，按 `scene_spec.json` 的 `measurements` 格式写好，建模时原样抄进 `scene_spec.json`。场景抽屉里会生成给 coding agent 的「建模任务」文本；抽屉里改登记字段会重写 `registration.json`。

## 现场采集规范

一次带齐，免得返工。平台登记的字段就是下面这些内容。

1. **标定板**：先 `python scripts/scenes/charuco.py board --out board.png`，按 100% 比例打印在 A4 上，再用卡尺量一个格子的边长（打印机会缩放）。把板平放在桌面上，记下板的左上外角离桌面左沿、前沿各多远。
2. **手机照片**：围着桌子拍 5–8 张（平台要求至少 5 张：正前方即机器人站位、左前、右前、左侧、右侧、斜上俯视），每张都要拍到整块标定板和桌面的四条边；拍摄时关掉自动微距，不要变焦，所有照片分辨率保持一致。之后用 `charuco.py solve photos/*.jpg --square-mm <实测> --out cameras.json` 得到每张照片的相机位姿，单位是米。尺度不再依赖“桌高 0.75 m”这类假设。
3. **卷尺**（写进 `scene_spec.json` 的 `measurements`，格式 `{"value_m": ..., "tool": "卷尺", "date": "..."}`）：
   - `table_top_z`、`table_length`、`table_width`：桌面离地高度、长、宽；
   - `table_edge_from_robot_root`：两个驱动轮轮心连线的中点（即仿真里的机器人根节点 `AGV_link`）到桌沿的垂直距离；
   - `table_centre_x_from_robot_root`：这个中点到桌面中线的横向偏移，桌面中心在机器人右侧为正。
4. **灯光**：记下灯的类型、数量、相对桌子的位置，以及色温和亮度（手机测光 App 即可）。记好后把 `lighting.status` 改成 `recorded`；再和机器人头部相机的画面对比过，改成 `matched`。
5. **机器人相机画面**：机器人停在实际工作位置，用头部左眼拍 1 张，再用腕部相机在 2 个以上不同臂姿下各拍 1 张，同时记下每张对应的关节角（URDF 关节名）。这些画面用于叠图检查。

## 建模

- 从最接近的已有场景复制目录开始。`real_scene3_lab` 是最新的写法：在照片上手标角点和边线，联合拟合相机和尺寸。本机不需要 Isaac 也能跑，命令见该目录的 `PLAN.md`。
- 有标定板时，把 `cameras.json` 里的相机位姿和焦距作为强先验（sigma 取小值）喂给 `fit_photo.py`；只有一个尺寸先验时，要在 `scale_basis` 里写明。
- 量到实测值之后：整体比例不对，用 `build_scene.py --scale-correction` 缩放；单个尺寸不对，就在 `fit_photo.py` 里把对应先验固定成实测值，重新拟合。改完重跑 `check_scene.py` 和 `preview.py`。
- 碰撞体：每个凸包最多 64 个顶点、单个面最多 32 个顶点，否则不能和颗粒碰撞（吧台场景就有 6–7 个超限，豆子会穿过去）。
- 灯光放在 Room 里。房间要有天花板，否则 RoboDojo 默认的棕色 HDR 会从上方漏进来，画面偏黄；Isaac 相机没有白平衡，色温要按照片调。

## 叠图检查

```bash
python scripts/scenes/overlay.py --env-cfg <配置> [--camera cam_left_wrist --joints q.json]   # 仿真里桌角和桌沿投影在哪
python scripts/scenes/overlay.py --env-cfg <配置> --labels view1.json --image view1.jpg --out check1.jpg
```

标注文件格式见 `overlay.py` 的文档字符串：`camera`、`joints`、桌角 `points`、桌沿 `edges`（在桌沿上点几个像素即可）。从头部相机看，桌面前沿的两个角通常在画面外，这时就标桌沿。每个视角的 `max_px` 记进 `scene_spec.json` 的 `overlay` 列表：`{"image": ..., "labels": ..., "max_px": ..., "date": ...}`。误差把场景几何、机器人站位和相机标定的误差都算在一起，超过 5 px 时，先用 `acceptance.py` 看卷尺项是不是已经通过，再去查站位。

## 接入和登记

1. 在 `build_asset.py` 的 `SCENES` 里加一行（场景目录、Room 名、Room 位置、物体平移），新增 `env_cfg/g1d_<场景>.yml` 和 `env_cfg/robot/g1d_<场景>.yml`。桌子比办公室的高多少，升降柱就抬高多少，物体跟着平移，这样机器人的可达范围在各场景里保持一致。
2. 在 GPU 服务器的隔离目录里跑 `build_asset.py`，再跑一次 `scripted_demo`，检查四路相机画面。装进共享资产库要按 README 的五步流程走，先征得同意。注意：`build_asset.py` 会先删掉旧布局；如果共享库里缺任务要用的物体，它会中途退出，留下一个空的布局目录（10-16 出过这个事故）。
3. 更新 `real2sim/ASSETS.md` 的场景行、`scene_spec.json`、`PLAN.md` 和 `HANDOFF.md`，一起提交。
