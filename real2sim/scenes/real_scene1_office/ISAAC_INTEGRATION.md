# 接入 Isaac Sim / Isaac Lab（unitree_sim_isaaclab）

以下内容均**未在 Isaac 中运行过**，所有步骤都处于待验收状态。本机预览用的是 Open3D 光栅化渲染，它与 RTX 渲染**不等价**。

## 1. 放到服务器上

把整个 `sim_scene/` 目录拷贝过去。USD 文件之间、USD 与贴图之间都是相对路径引用，已离线验证过：整包拷到其他目录后仍能完整打开。

```bash
export REAL_SCENE1_USD_DIR=/path/to/sim_scene/usd     # 可选；默认取 isaac/ 的上一级 usd/
cp sim_scene/isaac/real_scene1_scene_cfg.py unitree_sim_isaaclab/tasks/common_scene/
```

在任务的 `*_env_cfg.py` 中，把 `ObjectTableSceneCfg(TableCylinderSceneCfg)` 改为继承 `RealScene1SceneCfg`。机器人配置继续沿用任务原有的那份，本包不包含机器人。

## 2. 坐标与摆放

- 场景坐标系：Z 向上，单位米；原点在任务桌中心正下方的地面；+Y 从机器人一侧指向沙发。
- `SCENE_OFFSET = (-0.15, 0.61, 0.0)`：把桌子中心对到 G1 默认的 x = −0.15，桌子靠机器人一侧的边缘落在机器人前方 0.30 m。
  - 这只是初始值，请按实际站位调整。真实场景里是轮式底盘的机器人，距桌边约 0.64 m。
- 桌面高 0.68 m，与 unitree 自带 PackingTable 的桌面高度（约 0.665 m）接近。
- 物体的初始位姿在 `OBJECT_POSES` 中，由 cfg 从 `usd/build_params.json` 自动读取。
  - `TARGET_OBJECT` 指定的物体以 `object` 这个名字加载，供任务里的奖励和终止条件引用。

## 3. Prim 分层

| 文件 / 根 prim | 内容 | 物理 |
| --- | --- | --- |
| `background.usda` `/Background` | Floor（地毯纹理）、WallN/E/S/W、NECornerBlock、Exterior（窗外背景板）、Sofa、Plant、DeskEast/DeskWest/DeskWest2、Chair_*、Cabinets、Bench、TV | `/Background/Colliders/*`：guide purpose 的静态碰撞体，用盒体或凸包近似 |
| `table.usda` `/Table` | Top（圆角桌面）、Frame（升降腿） | `/Table/Colliders/*`：静态碰撞体 |
| `objects/<name>.usda` `/<name>` | Visual/*（渲染用） | 根 prim 带 RigidBodyAPI 和 MassAPI；碰撞体在 Collision/*；摩擦系数用 physics purpose 绑定的材质给出 |
| `lights.usda` `/Lights` | Sun、WindowFillNorth、WindowFillEast、CeilingPanel_0/1、AmbientDome | — |

材质统一使用 UsdPreviewSurface。检查绑定时请用 `UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()` 取最终生效的材质，不要只看 prim 上直接写的 binding 关系。

## 4. 渲染接入要点（批量采集前必须完成）

1. **确定渲染模式并记录**：RTX Real-Time 还是 Path Tracing。蓝色杯子用了 `opacity = 0.55` 来表现半透明，RTX Real-Time 下需要开启半透明（fractional cutout opacity / translucency）相关设置，否则杯子会显示成不透明或镂空。
2. **开启相机图像输出**：确认 `CameraCfg.data_types` 里有 `rgb`（需要时加上 `depth`），并在 headless 模式下开启相机渲染。
3. **相机标定**：
   - 头部相机：unitree 默认的 `front_cam` 参数是 focal_length = 7.6、horizontal_aperture = 20，水平视场约 105°。请换成真机 D435 的标定内参，或者明确标注为估计值。
   - 第三人称相机：`world_camera` 的位姿是估计值。
4. **处理任务默认灯光**：`RealScene1SceneCfg` 已经替换掉原来的 `DomeLight(intensity=3000)` 和 warehouse 房间。如果子类里还继承或保留着它们，光照会叠加两次。
5. **避免重复加载**：本包不含机器人，也没有 warehouse；确认场景里只有一个机器人实例、一个房间。
6. **固定曝光、色彩和采样**：关闭自动曝光，固定 tonemapper、曝光值和每像素采样数。灯光强度（太阳 3000、北窗补光 400、东窗补光 250、Dome 150）都是**未标定的起始值**。
7. **单帧对比**：先用目标机器人相机渲染一帧，和真实照片对比（`reference/` 下有去畸变后的原始帧），确认轮廓、接触阴影、阳光投影方向和长度、暗部亮度都合理之后，再做批量采集或光照随机化。
   - 太阳方向：方位角 103°、仰角 41°，由杯子的投影测得。
   - 窗外背景板设置了 `primvars:doNotCastShadows = true`，不会挡住阳光；请在 RTX 中确认这个属性生效。
8. **窗户没有玻璃**：窗洞是空的，阳光直接射入；需要玻璃反射时再另外添加。

## 5. 物理接入要点

- **杯子的碰撞体**由 12 段凸壁加一块杯底组成，杯口是敞开的（已离线验证：从杯口中心向下的射线第一次碰到的是杯底）。PhysX 会把每一段当作一个凸包处理。
- **质量和摩擦系数都是估计值**，并非实测。
  - 质量：蓝杯 22 g、纸杯 14 g、纸巾包 90 g、玩偶 35 g。
  - 摩擦：静/动摩擦系数在 0.40–0.85 之间。
- **桌子和背景**都是静态碰撞体，没有 RigidBodyAPI，不需要在 cfg 里设置 kinematic。
- **地面**：`/Background/Colliders/floor` 是一个比房间大 1 m 的盒体，可以作为地面使用。原 cfg 里的 GroundPlane 是注释掉的，保持注释即可。
- **待验收**：
  - PhysX 沉降：物体初始姿态下的底面与桌面零间隙，离线检查未发现穿插；
  - G1 的 IK 可达性；
  - 抓取稳定性。

## 6. 尺度修正

整包尺度来自三个"标准尺寸"假设（见 `PLAN.md`）。量到一个真实尺寸后（建议量桌长），按下面的方式修正：

```bash
python build_scene.py --scale-correction <实测桌长 / 1.20>
python check_scene.py && python make_manifest.py
```

`real_scene1_scene_cfg.py` 会从 `usd/build_params.json` 读取桌面高度和物体位姿，重建后会自动保持一致；只有 `SCENE_OFFSET` 需要人工确认。
