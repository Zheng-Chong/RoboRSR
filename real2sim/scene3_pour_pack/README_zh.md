# 场景 3：G1 桌面倒水

本包包含根据四张照片制作的 USD 场景、宇树官方 G1 + Dex1-1 关节模型，以及在服务器 Isaac Lab 环境中执行倒水的控制脚本。

**按本次要求，没有运行 Isaac Sim 物理验证，也没有提供成功率或“已完成倒水”的测试结论。** 浏览器动画只是动作几何意图；实际抓取、液体流动和结果由服务器运行时计算。

## 1. 快速使用

将整个 `scene3_pour_pack` 文件夹复制到服务器。使用你运行 `unitree_sim_isaaclab` 时的同一个 Python / Conda 环境：

```bash
conda activate <你的IsaacLab环境名>
cd /path/to/scene3_pour_pack
python run_pour.py --device cpu --water-ml 50
```

如果通过 Isaac Lab 启动脚本使用 Python：

```bash
/path/to/IsaacLab/isaaclab.sh -p /path/to/scene3_pour_pack/run_pour.py --device cpu --water-ml 50
```

这里的 `--device cpu` 指 **CPU 张量管线**。脚本另外启用 **NVIDIA GPU 上的 PhysX 粒子动力学**，用于取得液体粒子状态并统计接水量。水的物理模拟仍需要服务器 NVIDIA GPU。不能简单改成 `--device cuda:0`：Isaac Sim 部分版本的直接 GPU 管线不把粒子状态回写到 USD。

当前接口面向 Isaac Sim 4.5 / 5.x 与对应 Isaac Lab 2.x；服务器版本未提供，因此具体安装版本的兼容性尚未运行确认。

仅查看场景，不自动执行动作：

```bash
python run_pour.py --device cpu --water-ml 50 --scene-only
```

执行空杯抓取、倾倒动作：

```bash
python run_pour.py --device cpu --water-ml 0
```

脚本直接使用 Isaac Lab 的 Articulation、RigidObject 和 DifferentialIKController，不启动 DDS，不需要修改项目的 `sim_main.py` 或已有任务。文件夹可以放在项目目录内，也可以单独放置。

## 2. 打开哪些 USD

| 文件 | 内容 |
|---|---|
| `assets/scene3_g1.usda` | 推荐打开：完整桌面场景 + 可驱动的 G1，已经摆好任务准备姿态 |
| `assets/scene3_g1_standalone.usdc` | 同一场景的单文件版本，无外部 USD 引用，方便单独传递 |
| `assets/scene3.usda` | 只有桌面和六个物体，不含机器人 |
| `assets/objects/source_cup.usdc` | 右手拿起的源杯，空杯 |
| `assets/objects/receiver_cup.usdc` | 留在桌上的接水杯，空杯 |
| `assets/objects/toy.usdc` | 棕色玩偶与绿色小龟装饰 |
| `assets/objects/paper.usdc` | 纸团 |
| `assets/objects/basket_left.usdc`、`basket_right.usdc` | 两个篮子 |
| `robot/g1_dex1.usdc` | 官方 G1 29DoF mode 15 + Dex1-1，经本包转换成 USD 的关节资产 |

USD 使用米、千克、Z 向上。各物体为独立资产。直接在 Isaac Sim 中打开完整 USD 可以查看和运行场景物理；**自动倒水与注水需要启动 `run_pour.py`，USD 本身不包含自动执行 Python 逻辑。**

## 3. 桌面布局与机器人站位

桌面尺寸沿用实测：**长 1.20 m，宽 0.60 m，桌面离地 0.75 m**。

世界原点为桌子中心下方的地面。X 沿桌子短边，从机器人朝沙发；Y 沿长边，朝机器人的左侧；Z 向上。机器人朝 +X，使用的是双足 **G1**。

| 物体 | 位置 x, y, z（m） | 尺寸或用途 |
|---|---|---|
| 源杯 | −0.160, −0.105, 0.751 | 高 118 mm，口外径 73 mm，底外径 55 mm |
| 接水杯 | −0.160, +0.100, 0.751 | 与源杯相同 |
| 右篮子 | +0.015, −0.415, 0.751 | 395 × 270 × 100 mm |
| 左篮子 | +0.015, +0.415, 0.751 | 395 × 270 × 100 mm |
| 玩偶 | +0.125, −0.085, 0.751 | 后侧障碍物 |
| 纸团 | +0.125, +0.130, 0.751 | 后侧障碍物 |
| G1 骨盆根节点 | −0.470, −0.045, 约 0.7929 | 朝 +X；固定骨盆，脚底接近地面 |

骨盆与桌子近侧边的水平距离为 17 cm；这不是机器人外壳到桌子的净空距离。根节点高度由官方模型零位下的脚部碰撞几何推导。

四张照片用于判断物体的相对位置、形状和材质。**除桌子外，其余尺寸、质量、材质和布局是估计值；没有进行相机标定或精密多视角重建。** 机器人站位是为此倒水任务布置的，没有沿用照片中 G1-D 的底盘位置。

## 4. 建模细节

- 两个杯子使用高分段旋转曲面，白色不透明外壁、金属内壁、卷口和底圈；杯口保持开口。
- 每个杯子由 **48 段凸杯壁 + 独立杯底** 构成碰撞体，内部没有用实心凸包封堵。杯壁厚 3 mm、杯底厚 6 mm，为模拟采用的近似值。
- 篮子具有贯穿的可视孔洞、外扩侧壁、顶部边框和握持处。碰撞模型使用连续底板和侧壁，以减少粒子和机器人接触的复杂度。
- 玩偶有身体、头、耳朵、脚、短纤维表面、绿色小龟及背带。它是刚体近似，不模拟布料变形。
- 纸团使用不规则表面和折叠片表现褶皱，碰撞采用一个凸包。
- 完整任务中，篮子、玩偶和纸团设为运动学静态障碍物；两个杯子是动态刚体。单独物体 USD 保留其质量与动态刚体配置，便于复用。
- G1 的网格、关节变换、限位、质量与惯性来自官方 URDF。身体碰撞经本包转换为简化凸体；夹爪使用官方独立碰撞网格。额外的黑色面罩背板只是填补原 CAD 的面部开口，不参与质量、碰撞或运动学。

## 5. 动作如何生成

本包没有提供一串手工填写的关节角作为执行动作。

1. 启动时读取官方 URDF，通过有界数值逆运动学求解双手准备姿态。
2. 运行时读取 PhysX 的机器人关节状态、末端位姿和雅可比矩阵。
3. 抓取点定义在源杯中部，夹爪 TCP 定义在 Dex1 底座前方 112 mm，位于两指接触区域之间。
4. 每个控制周期用 Isaac Lab 的阻尼最小二乘逆运动学更新关节目标，同时施加关节限位、目标速度与加速度约束，并加入重力补偿。
5. 阶段间使用五次平滑插值。末端跟踪落后时，暂停阶段进度，等待机器人追上目标。

执行顺序为：

**等待稳定 → 预抓取 → 接近 → 闭合夹爪 → 抬高 25 mm 确认抓取 → 抬杯 → 对准接水杯 → 倾倒 → 等待水流 → 回正 → 返回 → 落杯 → 松开 → 撤离。**

夹爪通过物理接触夹住杯子。控制器不创建手杯固定约束，也不会每帧修改杯子位姿。若手指开口不符合杯子直径、杯子没有随小幅抬手移动、搬运时滑落、阶段超时或接近桌面／篮子，脚本停止并记录原因。

倾倒时围绕源杯的出水侧杯沿安排杯子位姿，让最低杯沿保持在接水杯中心上方。默认杯沿高度比接水杯口高 15 cm，最大倾斜 105°。右手操作，左手保持停靠姿态；骨盆固定，不包含行走、全身平衡或自主站位搜索。

这是**基于任务几何和实时逆运动学的控制器**，读取仿真真值，不包含 RGB 识别模型或训练好的策略。接近障碍物时使用运行时停止保护，不是通用的全局无碰撞规划器。

## 6. 水与成功条件

照片中的杯子没有水，因此交付的杯子资产本身为空杯。`run_pour.py` 启动时向源杯加入 50 mL PhysX PBD 粒子流体，粒子间距默认 3 mm，密度采用 1000 kg/m³；参数可以修改。

水用浅蓝色便于观察，支持时启用液体等值面显示。显示表面不会决定接水量；控制器统计的是仿真粒子实际位置。

当前成功条件：

- 接水杯内粒子对应水量达到初始水量的 75%（50 mL 时为 37.5 mL）；
- 落到桌面或地面区域的洒水量不超过 20%；
- 源杯回到初始位置 25 mm 范围内，并保持直立。

液体在飞行中与已经洒落的水分别统计。质量参数只包含空杯，水质量由粒子系统承担。

## 7. 服务器运行输出

每次运行创建 `runs/<时间>/`：

- `requested_spec.json`：本次场景参数；
- `trace.jsonl`：运行时阶段、末端跟踪误差、两个杯子的实际位姿、水量与停止原因；
- `result.json`：实际完成、失败、异常或仅空杯动作完成。

交付包不包含预填的成功日志。浏览器中的 `motion/preview_trajectory.json` 仅用于显示动作意图，不作为服务器的执行轨迹。

## 8. 修改参数

统一配置位于 `scene_spec.json`。只修改站位和摆放位置时，服务器脚本会读取新配置并重新求解初始 IK。

**修改杯子尺寸、杯壁、篮子尺寸或几何形状后，必须重新生成 USD。** 仅改 JSON 不会改变已有网格。资产生成环境需要 `requirements-authoring.txt` 中的包：

```bash
python scripts/build_assets.py
python scripts/build_robot.py
python scripts/prepare_motion.py
python scripts/build_preview.py
python scripts/flatten_scene.py
```

运行服务器控制脚本不需要安装 `trimesh` 或独立 `usd-core`。不要在 Isaac Sim 自带环境内用资产作者依赖覆盖它自带的 USD 库。

## 9. 资料与第三方来源

- [宇树官方 G1 模型目录](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description)
- [本包使用的 G1 + Dex1-1 URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_mode_15_with_dex1_1.urdf)
- [Unitree Isaac Lab 项目](https://github.com/unitreerobotics/unitree_sim_isaaclab)
- [Isaac Lab 逆运动学示例](https://github.com/isaac-sim/IsaacLab/blob/main/scripts/tutorials/05_controllers/run_diff_ik.py)
- [NVIDIA PhysX 粒子流体](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/particles/particles.html)
- [NVIDIA 关于粒子 GPU 管线与 USD 回写的说明](https://forums.developer.nvidia.com/t/particle-system-issues-with-fabric-enabled/309169/14)
- [PhysX 重力补偿接口](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.0/extensions/runtime/source/omni.physics.tensors/docs/api/python.html)

官方 G1 源文件保存在 `robot/source/`，附带原 BSD-3-Clause 许可证与 SHA-256 来源记录。浏览器 Three.js 依赖及许可证位于 `preview/vendor/`。
