# real_scene1 验证报告

## 结论

- 交付的是模块化 USD：任务桌 1 张、桌面物体 5 个，另有一个粗背景房间。
- 桌面区域和其上 5 个物体的位置，与视频、照片都一致。
- 背景的位置、轮廓、颜色和遮挡关系，在 8 个真实相机视角下与原图对齐。
- 离线检查 31 项全部通过（`check_report.json`）。
- 未在 Isaac 中运行。

## 1. 观测一致性：照片与视频不冲突

把视频 99 帧和 5 张照片放在一起跑 SfM（pycolmap），104 张全部注册到同一坐标系，平均重投影误差 0.78 px。

| 物体 | 视频帧反投影（v0460 / v0480） | 照片反投影（P1 / P2 / P4） | 顶部三角化残差 |
| --- | --- | --- | --- |
| 蓝杯 A | (0.036, 0.020) | (0.025–0.067, 0.026–0.079) | 1.4 cm |
| 蓝杯 B | (−0.153, −0.183) | (−0.182 – −0.130, −0.189 – −0.143) | 1.0 cm |
| 纸杯 | (0.172, −0.179) | (0.160–0.198, −0.193 – −0.141) | 0.8 cm |
| 纸巾包 | (−0.384, −0.070) | (−0.409 – −0.377, −0.081 – −0.066) | 1.8 cm |
| 玩偶 | (−0.238, 0.113) | (−0.266 – −0.234, 0.077–0.144) | 1.1 cm |

单位为米，场景坐标系。照片与视频之间的差异在底面拾取误差范围内（≤ 4 cm），所以没有丢弃任何一张照片；最终摆位以视频为准。

## 2. 尺度

- 采用 0.151 m / SfM 单位，由三个相互独立的标准尺寸推得，三者一致：
  - 桌长 1.20 m；
  - 办公桌高 0.75 m；
  - 地毯拼缝间距：在 3 个视角中实测 0.495 / 0.500 / 0.505 m。
- 尚无直接实测值。拿到实测尺寸后运行 `python build_scene.py --scale-correction k`，即可整体修正。

## 3. 离线检查（`python check_scene.py`）

| 检查 | 结果 |
| --- | --- |
| 9 个 layer：Z-up、metersPerUnit = 1、defaultPrim 有效 | 通过 |
| 依赖解析：9 个 layer、2 张贴图，无未解析项 | 通过 |
| 整包拷贝到临时目录后仍能完整打开 | 通过 |
| 网格有效性：160 个 mesh、20,138 个三角形；无 NaN，无退化面，法线数量匹配 | 通过 |
| 每个渲染 mesh 都有生效材质（`ComputeBoundMaterial`） | 通过 |
| 5 个物体都有 RigidBodyAPI、MassAPI 和碰撞体 | 通过 |
| 5 个物体底面 z = 0.680，与桌面零间隙、无穿插，且都在桌面范围内 | 通过 |
| 物体之间无初始穿插（FCL） | 通过 |
| 物体与桌架、背景碰撞体不相交 | 通过 |
| 任务桌与背景碰撞体不相交（最近的是墙角椅，间距 3.2 cm） | 通过 |
| 3 个杯子的碰撞体没有封住杯口（从杯口中心向下的射线，首次命中在杯底，高 4 mm） | 通过 |
| 估计的机器人站立区（y = −0.61、r = 0.22 m 的圆柱）静态间距 8 cm | 通过 |
| 太阳朝向与测得方向一致；两盏窗户补光都照向室内（东窗补光原本朝外，已修正） | 通过 |

## 4. 视觉对照

- `preview/comparison_sheet.jpg`：8 个真实相机视角，每行依次为原图、USD 渲染、渲染边缘叠加在原图上。
- `preview/rework_v0040_before_after.jpg`：同一相机在椅子位置返工前后的对比。改动来自重新生成的 `background.usda`。
- `preview/free_views_sheet.jpg`：两个俯瞰视角、正上方俯视、任务区特写、估计的机器人头部视角。

这些预览由 Open3D（Filament）光栅化渲染：

- 用的是"太阳 + 环境 IBL"的近似光照，不等于 `lights.usda` 里的灯光；
- 半透明蓝杯在预览中颜色被冲淡；
- 预览不代表 Isaac RTX 的运行时效果。

## 5. 已知偏差

- 背景只是粗模，误差估计如下：
  - 沙发、植物、橙色桌：约 ±5–10 cm；
  - 墙、门、电视、远处椅子：约 ±10–30 cm；
  - 东墙位置：约 ±0.3 m；
  - 电视宽度为估计值。
- 1.5 m 以上几乎没有重建点：相机始终在 1.4–1.6 m 高度向下拍，上方又多是玻璃和纯色墙面。窗高和层高（2.8 m）都是估计值。
- 玩偶、纸巾包用参数化形状近似，纸巾包没有印花贴图。杯子的口径和底径是估计值，杯高经过测量。
- 质量、摩擦、灯光强度都没有标定，是估计值。
- 未建模：真实机器人、人物、桌面杂物、线缆。

## 6. 未覆盖（待 Isaac 验收）

- PhysX 沉降与稳定性
- G1 的 IK 可达性和抓取
- RTX 渲染效果：曝光、色彩、半透明、阳光投影；需要用机器人相机渲染一帧，与 `reference/` 中的真实帧对比
- 真实材料参数

## 7. 墙面、柜子、椅子返工（2026-10-24）

改了什么、依据是什么见 `PLAN.md`「墙面、柜子、椅子」。验证：

- `check_scene.py` 31/31 通过；任务桌离最近的背景碰撞体 5.5 cm（东侧橙色桌，原来是墙角椅 3.2 cm）。
- 15 个真实相机都重新渲染、和原图叠加核对过（`render_all.py --tag`，`preview/` 不入库）；`preview/rework_furniture_sheet.jpg` 是六个视角的"照片 / 改前 / 改后"。柜子的玻璃门下沿、门缝、拉手，电视墙的腰线、白板边、墙板缝，窗台和窗槛墙，五把椅子的轮廓，在 v0300、v0400、v0480、p3、v0160 的叠图里目视重合；没有逐点量像素误差（门边椅在 v0160 里还偏约 30 px，墙角椅在 v0260 里偏右一点）。
- 本机没有 Isaac：RTX 下的木纹、半透明网背、光面白板的效果没有看过。

## 8. 复现

```bash
# 本机环境（macOS 上 Open3D 还要 arm64 的 libusb，放进 DYLD_FALLBACK_LIBRARY_PATH）：
# uv run --no-project --python 3.11 --with usd-core --with trimesh --with numpy --with shapely --with scipy \
#   --with opencv-python-headless --with open3d --with python-fcl --with rtree --with mapbox-earcut --with networkx python ...
python build_scene.py [--scale-correction k]   # 生成 usd/；之后重跑 scripts/scenes/scene_parts.py 生成 usd/parts.json
python check_scene.py                           # 离线检查 -> check_report.json
python make_manifest.py                         # manifest.json（含 sha256）
python render_all.py                            # 预览（每个相机一个进程，自动重试空帧）
python make_demo.py                             # demo/scene_data.js（离线网页 demo/index.html 的数据）
```

观测分析脚本（SfM、平面拟合、拾取与三角化）放在 `../work/`。`reference/cameras_scene.json` 里是全部 104 个相机在场景坐标系下的内外参。
