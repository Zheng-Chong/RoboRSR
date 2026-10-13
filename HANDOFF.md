# Project Handoff

更新时间：2026-10-13
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

## 剩余事项与风险

- G1 手臂在桌面高度的可达范围约 0.45 m；RoboDojo 现有任务（为桌面上的 X5 设计）有物体距肩部 0.4–0.7 m，部分不可达。自有任务的物体应放在 y≈[-0.55, -0.25]，或利用腰部前倾（`Yaw_Joint`）。
- 相机内参是借用的现有类型（头部 Gemini_345Lg、腕部 d435），位姿为目测/几何拟合；拿到真机相机型号和内参后应替换为实测值。头壳被隐藏，第三视角里机器人没有头部外壳。
- 末端位姿（ee）动作未支持：需写 G1-D cuRobo 配置；且 `RobotManager` 的 planner 以 `robot_name` 为键，coupled 左右臂会共用同一个 planner，需要改成按手臂区分。
- `_task.yml` 中指定了 `robot_config`/`scene_config` 的任务（如 x5+franka 竞赛任务、传送带任务）会覆盖 g1d 机器人或场景。
- 棱纹环按 `elbow_link` 网格的 x 区间切分（换 URDF/网格后需重新核对 `RING_X`）；材质参数为目测，仿真房间光照比真机亮，金属会显得偏亮。
- 服务器上 `/usr/local/bin/ffmpeg` 不带 libx264，已在仿真环境 bin 中链接系统 ffmpeg；新建环境需同样处理，否则录视频时 BrokenPipe。

## 下一会话

1. 先看 `git log` 和本文件；服务器环境已可用，直接用 demo_policy 跑 `--env-cfg g1d` 复现。
2. 下一步建议：加入自有物体（`Assets/Object/RoboDojo/<类别>/<编号>/object.usd`）并写第一个 G1-D 专用任务及其评测布局。

## 最近历史

- 2026-10-13：按产品图与真机录像调整全身配色（小臂圆环/夹爪/腰部黑，其余哑光金属）和头部/腕部相机；URDF 转换结果缓存到资产目录。
- 2026-10-13：搭建服务器环境；接入 G1-D + Dex1（关节角控制），X5 与 G1-D 冒烟评测通过。
