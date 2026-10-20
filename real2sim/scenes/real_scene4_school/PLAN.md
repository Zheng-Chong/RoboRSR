# real_scene4 学校实验室绿幕桌

学校实验室的工位：一张木色桌板，后面挂绿幕，桌上两只纸杯（白、黑，各是两只套在一起）和三个纸盘，原任务是食物分拣。env 配置 `g1d_school`。

## 来源

同事在 sys01 上做的 MuJoCo 孪生 `g1d_food_sorting`（`/home/sys01/Documents/Codex/2026-10-13/real2sim-real2sim/outputs/g1d-real2sim/exports/simulation/g1d_food_sorting`），10-20 整包拷到了 `5090-2:/root/g1d_sim_assets_20261020`（`SHA256SUMS` 校验通过）。本目录只取了其中的几何：

| 文件 | 内容 |
| :-- | :-- |
| `bundle/environment_manifest.json` | 桌面四角（初始躯干坐标系）、桌面高度、板厚、标称地面高度；来源路径和哈希 |
| `bundle/cup_shapes.json` | 两只纸杯的拟合轮廓（`scripts/objects/make_school_tableware.py` 读它） |
| `reference/bundle_*.png` | 源孪生的渲染（Isaac 总览、MuJoCo 头部相机），不是实拍 |

另外两个数取自包里，写在 `build_scene.py` 的常量里：

- 绿幕：`scenes/food_episode_0000.xml` 的 `backdrop` 盒子（中心、半尺寸），源里只是可视体。
- 躯干偏航：`demonstrations/episode_0000/reference.npz` 的 `raw_motor_qpos[0, 12]` = 0.067 rad（3.8°）。按宇树 SDK，12 号电机是腰部偏航，也就是 G1-D 的 `torso_Joint`；符号按 URDF 同向理解，没有在真机上核对。

## 做法

1. 源坐标系是第一帧的 `torso_link`（X 前、Y 左、Z 上）。按 g1d_v4 URDF，零升降、零腰俯仰时躯干原点在 `AGV_link` 正上方 0.7335 m、前 0.0534 m，`AGV_link` 离地 0.1115 m，合计 0.845 m，和包里的标称地面（-0.8448）一致。
2. 桌面四角绕躯干原点转 +3.8°，换到底盘坐标系。
3. 四角拟合一个矩形：长边方向取前后两条边的平均，长宽取投影平均。
4. 场景坐标系沿用 real_scene1-3：原点在桌面中心正下方的地面上，+X 沿桌长（机器人右手方向），+Y 远离机器人。
5. 补上没观测到的部分：四条方腿、灰色房间和天花板（挡住 RoboDojo 默认的棕色 HDR）、四块吸顶灯（照搬 real_scene3）。

运行：`uv run --no-project --with usd-core --with numpy python real2sim/scenes/real_scene4_school/build_scene.py`（本机即可，不需要 Isaac）。输出 `usd/`、`real_scene4.usd`，数值见 `usd/build_params.json`。

## 结果

| 项 | 值 | 置信度 |
| :-- | :-- | :-- |
| 桌面 | 1.065 × 0.596 m，板厚 15.7 mm | 拟合；四角离矩形最远 15 mm（源多边形本身不是严格矩形，右前角是外推的） |
| 桌面高度 | 离地 0.763 m（躯干下 0.082 m） | 躯干下 0.082 m 是拟合值；离地高度取决于零升降的假设 |
| 机器人站位 | 根节点在桌沿后 0.190 m、桌面中心左侧 0.083 m；底盘相对桌面法向左转 1.63° | 拟合 + 偏航假设 |
| 绿幕 | 桌子远侧边后约 5 cm，相对桌长方向转 5.5°，离地 0.29–2.29 m | 源里的盒子，尺寸看起来是给定值 |
| 桌腿、房间、灯光 | — | 未观测，随手定的 |

## 和其他场景的差别

- **升降柱保持 0**。办公室、吧台、实验室三个场景都是"桌子高多少，升降柱抬多少"，让躯干到桌面的高差一样（0.165 m）。学校这张桌子按拟合结果离躯干只有 0.082 m，这里保留真实高差，不抬升降柱。所以办公室调出来的抓取高度放到这里可能不合适。
- **机器人有偏航**（1.63°），其他场景都是正对桌面。
- 物体随机器人平移 (0, -0.06)，和吧台一样。

## 还没做

- 卷尺：桌长、桌宽、桌高、桌沿到底盘中心的距离（`real2sim/scenes/SKILL.md`）。
- 叠图：真机头部、腕部画面在 sys01 上，没拷过来；源孪生的相机候选也标着"未合格"。
- 灯光记录和对色。
- 源孪生的花材（40 簇）没有接入。
