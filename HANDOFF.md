# Project Handoff

更新时间：2026-10-14
当前分支：chongzheng/env-g1d-dex1
当前目标：在 RoboDojo 上接入 Unitree G1-D（Dex1-1 夹爪）做桌面操作仿真，后续加入自有物体和任务

## 项目状态

- 本仓库是 RoboDojo（Isaac Sim 5.1 / IsaacLab 2.3）的私有副本：`origin` = Zheng-Chong/RoboDojo（私有），`upstream` = RoboDojo-Benchmark/RoboDojo。
- 已接入机器人 `g1d`：一个 articulation、两条 7 自由度手臂（coupled 模式）+ Dex1 两指夹爪；轮子锁死、升降柱和腰部关节保持初始值（只做桌面操作）。
- 运行：`bash scripts/robodojo.sh eval ... --env-cfg g1d --action-type joint`。动作维度 `_robot_info.json`：arm [7, 7]，ee [1, 1]。
- 资产不在 git 里：`python scripts/g1d/build_asset.py` 从 unitree_ros 取 G1-D URDF，嫁接 Dex1 和 G1 头部 D435 坐标系，写入 `Assets/Robots/g1d/`，并从 arx_x5 布局生成 `Assets/Eval_Layout/RoboDojo/g1d/`（去掉相机支架）。
- 下载官方资产后必须运行一次 `python utils/update_embodiment_config_path.py` 生成 `Assets/Robots/*/curobo.yml`，否则 X5 任务启动即报错。
- 服务器环境（Host 别名 `21196`）：仿真环境 `RoboDojo`、策略环境 `demo-policy`（XPolicyLab 需 websockets>=14，Isaac Sim 需 websockets==12，所以分开）。激活方式见本地 `.agent/remote-validation.local.toml`（不入库）。

## 最近任务

### 目标
- 在服务器搭好 RoboDojo 环境并跑通基线；接入 G1-D + Dex1 并验证。

### 已完成
- 环境：Isaac Sim 5.1、IsaacLab、cuRobo、41 GB 资产安装完成；`robodojo.sh doctor` 14 项通过。
- G1-D 接入：`env/robot_manager/robot_config/g1d.py`（直接加载 URDF，`merge_fixed_joints=False` 保留相机/末端坐标系）、`robot_class/g1d.py`（G1DLeft/G1DRight）、注册表、`env_cfg/g1d.yml`、`env_cfg/robot/g1d.yml`、`scripts/g1d/`。
- `robot_class/x5.py` 增加可选类属性 `config_key`，子类可读 `<key>_robot_config.yml`；X5 行为不变。
- 相机（按真机录像目测对齐）：头部相机在 `d435_link` 原位（头壳内，头壳 `head_link/visuals` 设为不渲染），在 D435 姿态上再俯 2.4°（共约 50°），用广角类型 `Gemini_345Lg`（约 96°×71°），画面上方见墙、底部露一点胸口，与真机一致；腕部相机在 `*_dex1_base_link` 的 (0.055, 0, 0.095)、沿手指 43° 俯视，几何拟合真机画面：指尖约在画面一半高度，手指根部与凹槽可见，夹爪外壳在画外，零位时画面上方不露墙。
- 外观（按产品图 + 真机头部/腕部画面逐段核对）：URDF 颜色只有漫反射，加载后按连杆名绑定材质（`robot_config/g1d.py` 的 `LINK_MATERIALS`）。黑色：Dex1 整体、腰部 `Yaw_Link`/`Pitching_Link`、头、颈部 `logo_link`、轮子，以及 `*_elbow_link` 网格末端 x>0.075 m 的小臂圆环（紧挨弯曲腕部件，用 GeomSubset 按面绑定，见 `_paint_ring`；已用分段着色图与用户逐段确认）；其余（躯干、肩/大臂、肘关节支架与棱纹段、`wrist_*`、升降柱、底盘）为中灰哑光金属（diffuse 0.62、metallic 1.0、roughness 0.5）。绑定前须关闭 `visuals` 的 instanceable，否则 IsaacLab 会跳过绑定。
- URDF 转换结果缓存在 `Assets/Robots/g1d/usd`（按 URDF/配置哈希复用），不再每次往 `/tmp/IsaacLab` 写新目录。

### 关键决策
- 用 IsaacLab `UrdfFileCfg` 直接加载 URDF，省去单独 USD 转换步骤；USD 缓存在 TMPDIR。
- `need_planner: False`：目前只支持关节角动作；末端位姿动作需要 G1-D 的 cuRobo 配置（每臂一份）。
- 机器人根位置 y=-0.73：腰部立柱在桌沿之后，底盘伸入桌下（桌子是无腿桌板）。
- 复用 arx_x5 的评测布局，但删掉 (0, -0.47) 的相机支架，否则会插进机器人胸前。

### 涉及范围
- `env/robot_manager/`：G1-D 配置、类、注册表；x5.py 一行兼容改动。
- `env_cfg/`：g1d 运行配置与动作维度。
- `scripts/g1d/`：资产构建脚本与左右臂配置。

### 验证
- 服务器（profile 标签 gpu-4090-isaacsim51，custom 环境）隔离目录：ruff check/format、task_inventory、build_asset、`stack_bowls` + demo_policy（零动作）分别跑 arx_x5 与 g1d，各 1 轮，退出码 0，`_result.json` 中 `eval_time >= 1`；成功率 0 为零动作策略的预期结果。
- 人工检查渲染帧：侧视图中 G1-D 站在桌沿，手臂在桌面上方；头部相机看到桌面、双夹爪和碗；腕部相机看到手指和碗；配色与真机头部画面对比一致（小臂圆环黑、其余哑光金属、Dex1 全黑）；头部/双腕三视角与真机训练录像对比一致；无物理不稳定。

## 冷水壶资产接入（2026-10-13）

- 新增 `scripts/objects/make_pitcher.py`：生成 `pitcher/00000` 烟灰、`00001` 琥珀、`00002` 透明三款竖纹方形玻璃冷水壶；输出 `object.usdz`、`metadata.json`、`description.json`，完整尺寸 0.1325 × 0.089 × 0.245 m，质量 0.5 kg。
- 玻璃壶身 `/root/visual/model` 与塑料盖/嘴/把手 `/root/visual/lid` 为独立 Mesh，直接绑定 OmniGlass/OmniPBR；碰撞使用分段凸包，保留壶内空腔及出水口。几何自检检查绕序、容量和碰撞不侵入空腔。
- 兼容性：壶身仍是加载器遍历到的第一个 Mesh；布局优先使用 metadata 的完整 oriented_bbox，因此包含把手/壶嘴。`get_bbox()` 回退仍仅覆盖第一个网格；`sample_mesh_vertices()` 的奖励调用用于衣物顶点标记，冷水壶的 `passive.functional` 为空。
- 渲染根因补充：拆分网格之后，RTX 默认近距离折射重投影仍会把玻璃错误绘制在不透明壶盖/把手前方。实际对照确认 `/rtx/translucency/worldEps = 0.0` 恢复正确遮挡；已加入 `BaseEnv` 的渲染初始化与检查脚本；先向 Carb 注册该设置，再初始化 SimulationContext（否则 IsaacLab 2.3 会因未注册设置报错），用户显式 carb_settings 覆盖仍然生效。全局采用实际折射可能增加渲染开销，尚未做性能基准。
- 新增 `scripts/objects/check_object.py`：通过任务使用的 `RigidObject` 加载，检查落地高度、倾斜和速度，可保存三款壶的侧视截图；启用透射、反射和全局光照，与主环境保持一致。物理步不渲染，只在最终截图时渲染；`--device` 同时用于物理仿真。结尾刷新输出后 `os._exit`，避免相机渲染产品造成关闭卡住/退出码被 Kit 吞掉。
- 验证：服务器隔离目录执行对象脚本 ruff check/format、task_inventory --check、三款生成自检及落地/截图检查。三款均 z=0.1225 m、倾斜 0°、速度约 0.0015–0.0016 m/s；截图确认玻璃和塑料正确显示，进程退出码 0。本机只做语法与 diff 检查，不运行 GPU 仿真。
- 资产安装到服务器共享资产库 `Assets/Object/RoboDojo/Rigid/pitcher/0000{0,1,2}`，登记对应 `uuid_map.json`；资产不入 Git，可由生成脚本重建。
- 复现：`python scripts/objects/make_pitcher.py`；然后 `PYTHONPATH=. python scripts/objects/check_object.py Assets/Object/RoboDojo/Rigid/pitcher/00000 Assets/Object/RoboDojo/Rigid/pitcher/00001 Assets/Object/RoboDojo/Rigid/pitcher/00002 --device cuda:1 --image pitcher_check.png`。
- 尚未将冷水壶替换进倒水任务；流体对照已完成（见下节），未验证策略操控。工作区原有 G1-D 倒水任务相关改动（build_asset、task_inventory、任务注册/配置/类）保留未提交，本次提交仅包含对象脚本、渲染设置和本交接记录。

## 冷水壶流体交互验证（2026-10-13）

- 新增 `scripts/objects/check_pitcher_fluid.py`：用实际 `RigidObject` / `FluidObject` 加载器对比开口壶与额外封住出水口的同款壶；使用 `pitcher/00002`（三款壶几何相同）及原有 `Fluid/wuliangye/00000`，保留原粒子系统/材质参数。每组 4,872 粒子、间距 3 mm；静置 2 s、4 s 平滑倾斜至壶嘴侧 130°、保持 3 s，dt=0.004 s。壶以运动学刚体按预设姿态运动，不涉及机器人策略。
- 原 GPU 状态管线下 USD 粒子坐标不更新：增加回读、显式 simulationOwner、关闭 Fabric、完整渲染步进均不能在该管线获得有效位移。GPU 状态管线结果判为无效，不能据此宣称不漏水。相关上游问题：[IsaacLab #1927](https://github.com/isaac-sim/IsaacLab/issues/1927)。
- 有效路径：`device=cpu`、`use_fabric=False` 使用 CPU 状态管线，同时显式启用场景 GPU dynamics 和 `overwrite_gpu_setting(1)`，PBD 求解仍在 GPU。关闭 suppressReadback，并在纯物理步后显式 `update_transformations(False, True)`，保证读取真实粒子状态。脚本限制 `--device cpu`，避免误用已知无效路径；未修改主环境的设备默认值或 FluidObject。
- 成功判据：两组静置保留率 >=99%、平均下落 >3 mm；封口对照倾倒后保留 >=99%；开口壶倾倒后保留 <=50%，至少50%粒子穿过壶顶平面时位于壶嘴附近的实际开口范围；限制 X/Y 边界，排除倾斜坐标平面扫过远处散落粒子的误计。检查粒子数不变、坐标有限和初始流体完全位于壶内。
- 最终结果：开口壶静置保留 99.9589%，平均下落 36.827 mm，倾倒后保留 0.5131%；4,845 / 4,872 粒子（99.4458%）从实际开口范围越过壶顶，总离壶并落地 4,847 粒子（含静置时散落的 2 粒子）。封口壶静置及倾倒后保留 99.9795%（1 粒子静置散落，穿过实际开口范围计数为 0）。五项判据全部通过，退出码 0。完整应用步进的独立有效对照也通过，开口组流出 99.4253%，封口组保留 99.9795%。
- 输出 `metrics.json`、`particles.npz` 与 `upright.png` / `pouring.png` / `final.png`；截图确认开口壶形成水流，红色封口对照保持盛水。指标与截图保存在本机 `output/pitcher/fluid/` 和服务器运行项目的同名输出目录，不入 Git。
- 验证环境：gpu-4090-isaacsim51 / custom；ruff check、ruff format、静态语法、几何/变换断言和实际 GPU PBD 检查通过。本机不运行 GPU 仿真。服务器保留失败诊断目录用于对照。
- 复现：`PYTHONPATH=. python scripts/objects/check_pitcher_fluid.py --device cpu --out output/pitcher/fluid`。需要选卡时用 `CUDA_VISIBLE_DEVICES` 选择 PBD GPU，并通过 `--kit_args=--/renderer/activeGpu=N` 选择对应 Vulkan 渲染卡（编号取决于机器）。
- 边界：只验证现有小体积流体与刚体碰撞、盛装及倾倒；未验证满壶、真实水参数标定、接杯、机器人抓取或策略成功率。GPU 状态管线的粒子回读问题仍未修复；实际倒水任务接入时必须沿用有效状态管线或另行解决回读。

## 倒水任务改为倒红豆（2026-10-14）

- `g1d_pour_water_into_cup` 壶内的咖啡换成红小豆。任务名、标签（`wine`/`bottle`/`cup`）、布局结构不变，指令改为 "Pour some red beans from the pitcher into the cup."。
- `FluidObject` 新增可选 `physics.granular`（默认 False，原流体任务不受影响）：True 时粒子按 PBD solid 粒子创建（`fluid=False`），填充间距与显示球半径改用 `solid_rest_offset`。
- `build_asset.py` 的 `RED_BEAN_PHYSICS`：4 mm 颗粒（solid/rigid rest offset 2 mm、contact offset 2.4 mm）、单粒 0.06 g、摩擦 0.2、无内聚/粘度/表面张力，关闭 isosurface/smoothing/anisotropy 直接渲染颗粒；整体覆盖原 `wuliangye` 条目的 `particle_system`/`particle_material`，其余键保留。材质 `RedBean.mdl` 由 `Water_Opaque.mdl` 改色生成（替代 Coffee.mdl）。奖励 `scatter_connect_radius` 0.0065 → 0.009（约两倍颗粒间距）。
- `check_pitcher_fluid.py` 增加 `--red-beans`。壶嘴开口 13 mm 下的参数扫描（静置 2 s、4 s 倾到 130°、保持 3 s）：5 mm/摩擦 0.4 倒出 7.7%，5 mm/0.2 倒出 25%，4 mm/0.4 倒出 13%，均卡住；4 mm/0.2 倒出 58.7%（1,216/2,070 粒经开口），五项判据通过，封口对照保留 100%。
- 修复 `scripted_demo.py` 误判：EvalEnv 每回合以 success=True 开始，脚本动作在 step_lim 前放完且奖励未触发时会被记为成功。现在脚本结束仍未 end 的回合记为失败。
- 修复后服务器结果（gpu-4090-isaacsim51，各 1 轮，退出码 0，eval_time=1）：`g1d_pour_water_into_cup` 失败（倾到约 110°、保持 48 步后仅约 6% 豆子离壶，判据要求 ≥20%）；`g1d_general_pickup` 失败（夹爪闭合后物体未离开桌面）；`g1d_stack_blocks` 失败。后两者与红豆无关，此前记录的成功是上述误判。远程验证 profile 中要求这两个任务 success_rate=1.0 的检查因此不通过。
- 渲染输出：服务器 `/data2/tmp/robodojo/bean_render/`，本机 `output/bean_render/`（壶对照截图、三个任务的五路视频），不入 Git。
- 远程运行备注：服务器负载高时 SSH 长连接约 15 分钟断开（退出码 255）会杀掉仿真；改为在隔离目录用 `nohup setsid` 后台跑。验证脚本会拒绝子模块条目，本次上传时跳过 gitlink（服务器上替换为共享软链接）。

## `g1d_general_pickup` 与 `g1d_stack_blocks` 抓取与叠块修复（2026-10-14）

- **碰撞体与布局根因（已入 `build_asset.py`）**：
  1. Dex1-1 滑块连杆 `Link1_2.STL` / `Link2_2.STL` 为 L 形（后方滑块凸耳 + 前方指梁），PhysX 单个凸包将内角连成斜面，在俯仰 `-60°` 时比指垫 `Link1_3` / `Link2_3` 内表面突出 `1.6 mm`，先于指垫无摩擦顶走物体或互相卡死。`recess_carrier_hulls` 生成修剪后的 `Link{1,2}_2_col.STL`（`x_mesh` 截断至 `±0.002 m`）并清除 `Assets/Robots/g1d/usd` 缓存强制重转 USD。
  2. `g1d_general_pickup`（偏移 `y = -0.36 m`）过滤掉窄边 `> 80 mm` 或高度 `< 15 mm` 的目标物体，并剔除 `0.6x` 缩放后与目标间距 `< 20 mm` 或彼此 `< 5 mm` 的干扰物；`g1d_stack_blocks` 偏移设为 `y = -0.35 m`。
- **运动学（IK）与夹持定心根因（`scripts/g1d/scripted_demo.py`）**：
  1. **TCP 雅可比与关节限位投影**：`Arm.ik()` 将雅可比从 `dex1_base_link` 原点平移到指尖抓取点 `pos + rot.apply([PAD_REACH, 0, 0])`（`jac[:3] - np.cross(r, jac[3:].T).T`），姿态权重设为 `0.3`、阻尼 `0.02`，并将关节限位内缩 `0.025 rad` 后按预测步 `q + dq` 投影零化越界关节列。解决了叠三层方块时 `right_wrist_pitch_joint` 在 `1.595 rad` 被 PhysX 限位挡住却未触发列零化、导致指尖在 `y` 方向偏差 `22 mm` 并在下降时甩落方块的问题。
  2. **双指力饱和失稳与线性 PD 定心**：两指独立驱动且 `effort_limit_sim = 20 N`、`stiffness = 10000 N/m` 时，若对 `35 mm` 方块（接触位 `q ≈ 0.0076 m`）直接指令 `u = 0`（`q_target = 0.0245 m`），两指同时饱和在 `20 N`，差模定心刚度为 0，离桌后单指会将方块推偏 `16 mm`。现改为先在桌面合拢让两指停在物体表面，再对方块取实测均值加 `1.5 mm` 压入量（`close_cmd = clip((0.023 - fingers.mean()) / 0.0445, 0, 1)`，约 `15 N` 处于线性 PD 区，具备 `10,000 N/m` 自动定心恢复力，抬起偏移 `< 0.2 mm`）；对通用抓取（锥面/曲面重物如 `0.45 kg` 小车）保持全闭 `close=True` 持续施加满额 `20 N`。
  3. **侧向抓取手腕朝向与接近高度**：`|yaw| > 60°` 时将手腕朝向肩部内侧（右臂 `yaw < 0`、左臂 `yaw > 0`），使 `dex1_base_link` 落在肩正前方可达区（`x ≈ 0.148 m`），并以 `grip + [0, 0, lift]` 高度接近避免旋转时扫碰桌面物体。
- **验证（`gpu-4090-isaacsim51`，运行 ID `20261014-201423-9ccaf38f`）**：
  - `ruff check`、`ruff format --check`、`task_inventory.py --check`、`build_asset.py` 全部通过。
  - `g1d_general_pickup`（seed 0，`car/00011`）：第 119 步将小车从 `z = 0.698 m` 稳稳抓起至 `z = 0.801 m` 触发成功退出，`_result.json` 中 `eval_time = 1`、`success_rate = 1.0`。
  - `g1d_stack_blocks`（seed 0）：依次将 `block_2`（`z = 0.732 m`）与 `block_1`（`z = 0.767 m`）叠放到 `block_0`（`[0.147, -0.486, 0.697]`）上，水平对齐误差 `≤ 2 mm`，双臂回零后在第 605 步触发成功退出，`_result.json` 中 `eval_time = 1`、`success_rate = 1.0`。

## 双臂倒水脚本 + 吧台场景（2026-10-14）

- 吧台 real2sim 场景 `Assets/sim_scene_counter/`（只在本机，gitignored）：两张照片联合拟合，`build_scene.py` 生成 `real_scene2.usd`；水龙头鹅颈改为连续扫掠管（`swept_tube`）。服务器隔离目录 `/data2/tmp/robodojo/agent-remote-runs/counter-pour-20261014-163053` 以 Room=real_scene2 跑倒水（机器人根 y=-0.80、升降柱各 0.075，物体整体 y-0.07、z+0.15），共享资产库未改。
- `scripted_demo.py` 倒水改为双臂：离壶近的手臂抓壶把手，另一臂用 TOP_ROT 斜向下（手腕内转 60°）夹杯身、抬起 4 cm；倾倒时抓壶手按“实际壶嘴 vs 实际杯口”闭环修正（`SPOUT_GAIN`），补偿壶在手里打滑；倾倒保持不再固定 48 步，用满 step_lim 余量；先竖直提壶再横移、限速 `JUG_SPEED`（斜着拉会把壶拽倒）；0–80° 快倾。
- 可达性探测（隔离目录 `scripts/g1d/probe.py`，不入库）：原布局杯子 y≈-0.28（吧台坐标）持杯手任何姿态都够不到（最小误差 5.7 cm）；y≈-0.40、杯口上方附近可达。因此 `build_asset.py` 增加 `POUR_CUP_PULL=-0.09`（杯子比壶近 9 cm）和 `POUR_MIN_GAP=0.03`（与壶间隙不足的布局丢弃），150 → 140 个倒水布局；`g1d_pour_water_into_cup.yml` 杯子 ylim 说明同步为 [-0.39, -0.29]。本机/共享资产库的布局尚未重建（需运行 `build_asset.py`）。
- 验证（隔离目录，各 1 轮，退出码 0，eval_time=1）：seed 1（壶左、右手持杯）抓壶、持杯、倾倒全程成功，第 450 步出壶 20%、在杯 17%，因 step_lim=480 用尽判失败；seed 0（壶右）右手插把手时把壶推偏 2 cm 没抓住，左手也没抬起杯子，失败。视频 `output/counter_pour_bimanual/seed1_grid.mp4`。
- ruff check/format 通过；未跑 task_inventory / doctor（未改任务注册）。
- 倾杯迎流（`CUP_TILT`，杯口随壶倾角同步朝壶倾斜，绕夹持点转并平移补偿手掌、杯口高度取倾斜后的最高点）：seed 1 实测 25° 手腕够不到并撞掉壶；15° 能倾到位，但上方抓杯的手掌/杯子进入倾斜壶身下方，两者相撞脱手（SPOUT_GAIN 积分在碰撞时继续把壶往杯上推）。默认设为 0（行为等同未倾杯版本）。

## 实验室操作台 real2sim 场景 real_scene3（2026-10-14）

- `Assets/sim_scene_lab/`（只在本机，gitignored）：用户提供四张围着实验室白色折叠培训桌拍的照片（`reference/p1–p4.jpg`）。流程照搬 real_scene2：`fit_photo.py` 联合拟合四台相机和尺寸，`build_scene.py` 生成 `usd/` 和 Room 入口 `real_scene3.usd`，`check_scene.py`、`preview.py` 做离线检查和预览。详细尺寸、置信度和接入方法见该目录下的 `PLAN.md`。
- 结果：桌面 1.369 × 0.686 m，高 0.749 m，板厚 21 mm。尺度只来自“桌高 0.75 m”这一个先验，长宽与标准 1.4 × 0.7 m 相差约 2%，可以作为交叉验证。挡板在 +Y（底边离地 0.456 m）；两根斜立柱向外倾约 15°，偏向挡板一侧 8.6 cm；走线盒是桌面贴图，没有几何体。桌面贴图取四张照片矫正后的逐像素中值；墙是照片背景板（墙前的工位等被压平到墙上）；地毯从照片按真实尺度矫正。
- 约定：机器人站在没有挡板的 −Y 一侧（这是假设，真机若站在另一侧就把 Room 转 180°）。Room `default_pos` 取 (0, −0.257, 0)，使桌沿落在 y=−0.60；G1-D 默认站位不会碰到桌腿（桌下净深 0.63 m）；桌面比 real_scene1 高 0.069 m，需要用升降柱补偿、物体同步抬高。
- 验证：只做了本机离线检查。`check_scene.py` 19/19 通过；`preview/compare_photo.jpg` 中，桌子模型边缘在四个拟合视角下都与照片贴合；拟合残差大多在 0.3–4 px（脚轮位于暗处，最大约 20 px）。未上服务器，未跑 Isaac（RTX 渲染、PhysX 静置、机器人可达性都没有验证）；未改代码，也没有安装到 `Assets/Room/`。

## 剩余事项与风险

- real_scene3：尺寸没有实测（量一下桌长或桌高，再用 `--scale-correction` 修正）；灯光强度没有标定；需要拷到服务器 `Assets/Room/real_scene3/`，并在 Isaac 里用头部相机渲染一帧和真机画面对比。

- 双臂倒水：step_lim=480 不够（颗粒流速约 0.1%/步，需 ≥20% 出壶），需提高 `g1d_pour_water_into_cup` 的 step_lim 或提高流速；右臂抓壶（壶在右、y≈-0.28）插把手会推壶，需调接近路径；持杯手在可达边缘，杯子 y 更远的布局（-0.46）未测。
- G1 手臂在桌面高度的可达范围约 0.45 m；RoboDojo 现有任务（为桌面上的 X5 设计）有物体距肩部 0.4–0.7 m，部分不可达。自有任务的物体应放在 y≈[-0.55, -0.25]，或利用腰部前倾（`Yaw_Joint`）。
- 相机内参是借用的现有类型（头部 Gemini_345Lg、腕部 d435），位姿为目测/几何拟合；拿到真机相机型号和内参后应替换为实测值。头壳被隐藏，第三视角里机器人没有头部外壳。
- 末端位姿（ee）动作未支持：需写 G1-D cuRobo 配置；且 `RobotManager` 的 planner 以 `robot_name` 为键，coupled 左右臂会共用同一个 planner，需要改成按手臂区分。
- `_task.yml` 中指定了 `robot_config`/`scene_config` 的任务（如 x5+franka 竞赛任务、传送带任务）会覆盖 g1d 机器人或场景。
- 棱纹环按 `elbow_link` 网格的 x 区间切分（换 URDF/网格后需重新核对 `RING_X`）；材质参数为目测，仿真房间光照比真机亮，金属会显得偏亮。
- 服务器上 `/usr/local/bin/ffmpeg` 不带 libx264，已在仿真环境 bin 中链接系统 ffmpeg；新建环境需同样处理，否则录视频时 BrokenPipe。
- 自建资产入库：`.gitignore` 改为 `/Assets/*` 并放行 `Assets/scene3_pour_pack/`（排除 `preview/`、`*.bak`）；其中 `*.usdc`/`*.STL` 走 Git LFS（`.gitattributes`），clone 后需 `git lfs install && git lfs pull`。本机 git-lfs 为官方二进制，装在 `~/.local/bin`（Homebrew 在此机会从源码编译整条依赖链，不要用）。`Assets/sim_scene_counter`、`Assets/sim_scene_opus`、`Assets/sim_scene_lab` 仍只在本机，未入库。

## 下一会话

1. 先看 `git log` 和本文件；服务器环境已可用，直接用 demo_policy 跑 `--env-cfg g1d` 复现。
2. 下一步建议：加入自有物体（`Assets/Object/RoboDojo/<类别>/<编号>/object.usd`）并写第一个 G1-D 专用任务及其评测布局。

## 最近历史

- 2026-10-14：用四张照片重建实验室操作台 real_scene3（`Assets/sim_scene_lab/`，本机，gitignored），离线检查 19/19 通过，未在 Isaac 中验证。
- 2026-10-14：修复 `g1d_general_pickup` 与 `g1d_stack_blocks` 脚本抓取与叠块（TCP 雅可比 + 关节限位内缩投影、方块线性 PD 自动定心、侧向抓取手腕朝向肩部），远端验证两任务均拿到 `success_rate = 1.0`；倒水脚本改为双臂协同，倒水布局杯子拉近 9 cm。

- 2026-10-14：`README.md` 按 Notion「Real2Sim2Real」页面改写为中文项目说明（目标、现状、快速开始、TODO、月度目标、零样本上真机路线），保留上游致谢/引用/许可证。Notion 写"scripted_demo 已成功倒过一次水"，与本文件验证记录冲突，README 按验证记录写为三个演示均未成功。
- 2026-10-14：`Assets/scene3_pour_pack`（代码、场景/物体 USD、G1+Dex1 机器人 USD 与网格，约 102 MB，170 个 LFS 文件）入库，commit `9934da5`。
- 2026-10-14：`CLAUDE.md` 增加 Session Handoff 规则，新建 `AGENTS.md`（同一规则，并指向 `CLAUDE.md`）：编辑前读本文件，任务完成后更新并随改动一起提交。
- 2026-10-14：倒水任务改为倒 4 mm 红豆（颗粒粒子）；修复脚本演示误判后三个 G1-D 演示均为失败。
- 2026-10-13：按产品图与真机录像调整全身配色（小臂圆环/夹爪/腰部黑，其余哑光金属）和头部/腕部相机；URDF 转换结果缓存到资产目录。
- 2026-10-13：搭建服务器环境；接入 G1-D + Dex1（关节角控制），X5 与 G1-D 冒烟评测通过。
