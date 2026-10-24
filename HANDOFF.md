# Project Handoff

更新时间：2026-10-24
当前分支：`main`。G1-D 桌面版 / 移动版（[PR #17](https://github.com/Zheng-Chong/RoboRSR/pull/17)）已于 2026-10-21 合并进 `main`。深圳实验室场景（[PR #22](https://github.com/Zheng-Chong/RoboRSR/pull/22)）已于 2026-10-21 合并（`2975b4a`）并部署到数采平台。更早：流水线规范化（[PR #1](https://github.com/Zheng-Chong/RoboRSR/pull/1)）和学校场景接入（[PR #2](https://github.com/Zheng-Chong/RoboRSR/pull/2)）都已于 2026-10-20 合并，短分支已删除。
当前目标：
- **榜单冲分（2026-10-22 起，20 天）**：用本平台做多轮 Coding Agent 仿真数据 → 评测 → 对抗式（找失败、补难例和恢复数据）迭代，提高 OpenWAM、pi0.5 在上游 RoboDojo 榜单（54 个 dual X5 任务）上的成功率。用户确认：榜单允许用自生成数据训练；训练在 dsw-4（一轮 ≤ 1 天）。见「最近任务」第一节。
- 把 G1-D 的三条流水线规范化：Real2Sim 采集（物体、场景）、仿真数据生成，以及本仓库内的仿真评估。训练在仓库外。

本文件只记录当前状态。逐次实验的细节（相机拟合、抓取调参、倒豆参数扫描、各次服务器验证）见 `git show 4fb9290:HANDOFF.md`。数采平台的运维细节（端口、账号、隧道、重启方法）已移到本机 `.agent/ops.local.md`，不入库，因为仓库是公开的。

## 项目状态

- **机器人** G1-D + Dex1-1（带相机版夹爪）：两条 7 自由度手臂加两指夹爪，关节角控制，30 Hz。分两个版本（对应官方标准版 / 旗舰版）：桌面版 `g1d` 底盘固定，腰（`waist_joint_state`）和升降（`lift_joint_state`）是可选动作键，不发就保持；移动版 `g1d_mobile` 再加差速底盘 `base_velocity` [vx, vyaw]，不发就停车。已有数据和策略都只用 16 维手臂 + 夹爪，不受影响。细节见 README「机器人本体」和 CONTRACT「观测与动作」。
  - 头部用双目的左眼 `cam_head`（棋盘格标定 fx≈279.5，宇树合同外参），两个腕部相机 fx≈314；投影到真机画面核对过（误差 0.4–3.6 px）。
  - 关节驱动参数（刚度 4400、阻尼 40）和夹爪力是拍定的，没做系统辨识。同事把"相机+动作标定工具包"作为 `g1d_calib_toolkit_20261019.zip` 传到了仓库根目录，还没接入。
  - 构建：`python scripts/g1d/build_asset.py`（读 `real2sim/robot/g1d_v4`）。下载官方资产后先跑一次 `utils/update_embodiment_config_path.py`。
- **场景**（`real2sim/scenes/`）：env 配置 `g1d` 是办公室，`g1d_counter` 是吧台（移动版 `g1d_counter_mobile` 加了左侧客区，见「最近任务」第一节），`g1d_lab` 是公司实验室，`g1d_school` 是学校实验室绿幕桌，`g1d_shenzhen_lab` 是深圳实验室办公桌。前三个从照片拟合，尺度靠先验；学校的来自同事用真机相机画面拟合的 MuJoCo 孪生；深圳实验室是第一个按平台登记、用卷尺定尺度的场景（卷尺三项通过，站位和叠图待做）。其余都没有卷尺实测，都没有叠图验收。状态汇总：`python scripts/scenes/acceptance.py`（目前全部 pending）。新场景先在数采平台登记（≥ 5 张照片 + 卷尺实测，写 `registration.json`），再按 `real2sim/scenes/SKILL.md` 交给 coding agent 建模。新场景先在数采平台「登记场景」（至少 5 张照片 + 卷尺实测，写 `real2sim/scenes/<场景>/registration.json`），再按 `real2sim/scenes/SKILL.md` 建模。
- **物体**：每件资产的来源和匹配程度看 `real2sim/ASSETS.md`。按数采平台的登记（`real2sim/objects/<id>/spec.json` + 照片）给实物建模，交给 coding agent 时按 `scripts/objects/SKILL.md`；平台物体页生成的「建模任务」文本第一行就指向它。
  - 量杯、塑料马克杯、红豆有实测尺寸和质量。
  - 共享资产库缺 `measuring_cup`、`plastic_mug`，以及量杯任务的布局。
  - 学校的纸杯 `paper_cup/0000{0,1}`、纸盘 `paper_plate/0000{0,1,2}` 是孪生的拟合几何，没在平台登记，没有实测，未装共享库。平台上能看到它们（10-20 部署，见下面「数采平台」）。
  - 耳机充电盒 `airpods/00000`：第一件按 `scripts/objects/SKILL.md` 从平台登记建的物体，实测尺寸和质量，已装共享库。
  - 自建物体写的 `friction` 不生效：运行时只读 `physics.static_friction` 和 `dynamic_friction`。
- **任务和脚本专家**（`scripts/g1d/scripted_demo.py`）：
  - 拾取、叠块：三个场景 seed 0 都成功。
  - 杯对杯倒豆、量杯对倒：部分 seed 成功。
  - 壶倒杯：5.6 mm 的豆子会堵住 13 mm 的壶嘴。
  - 移动送杯 `g1d_deliver_cup_to_lounge`（`g1d_counter_mobile`）：seed 0 前 4 个布局 4/4。
- **数据**：所有数据集登记在 `scripts/datagen/registry.json`。目前有 2 份办公室量杯全量倒豆数据，各 100 条，LeRobot v3.0，都是按旧的 legacy 格式导出的：
  - 固定朝向的一份，已上传 ModelScope 私有仓库；
  - 随机左右手、随机朝向、大范围位置的一份，也已上传 ModelScope 私有仓库（单独的仓库）。
- **数采平台** `src/real2sim_platform/`：部署在 `5090-2`，经阿里云反向隧道对外访问。场景卡片封面是 RTX 渲染图，抽屉里有 3D 预览（见「最近历史」10-21）。
- **服务器**：只用 `5090-2`（8×5090）。
  - 共享资产库：`/data/robodojo-assets/Assets`（10-20 整体迁来，39 GB）。改它要先征得同意，备份放 `/data/robodojo-assets/backups/`。运行目录的 `Assets/` 覆盖目录里，不改的部分直接从它软链接或拷贝。数采平台现在读的还是手动刷新的元数据镜像 `/root/RoboDojo_assets/Assets`，不是这个库。
  - 仿真环境 `/root/miniconda3/envs/RoboDojo`；导出环境 `robodojo-lerobot`（lerobot 0.4.4，10-19 补装了 h5py 3.16，`pip -i` 用清华镜像）；上传环境 `/data/robodojo-runs/ms-venv`；运行目录在 `/data/robodojo-runs/`。

## 最近任务

### 吧台客区 + 第一个移动操作任务（2026-10-23/24，`chongzheng/scene-counter-lounge-mobile`）

- **用户要求**：把吧台旁边的客区（3 张照片：墙纸凹龛、圆桌、蓝色扶手椅）加进吧台场景，做成支持移动操作的场景。用户选定：从移动版分支开（已在 `main` 里）、加一个跨区任务并带脚本专家、椅子做成可推动的刚体、暂时没有卷尺，尺度按先验。
- **场景**（`real2sim/scenes/real_scene2_counter/`，细节见其 `PLAN.md`「左侧客区」）：新增 `fit_lounge.py`，三张照片 `reference/lounge{1,2,3}.jpg` 联合拟合三台相机、凹龛和圆桌（31 个像素约束，最大 12.7 px），尺度只来自「圆桌高 0.75 m」。凹龛宽 5.07 m，在吧台左木柱左侧；吧台左木柱宽从假设的 0.45 m 改为 1.0 m（两组照片的下限）。新增 `usd/lounge.usda`（墙纸和灯带贴图从照片透视校正、两张圆桌、垃圾桶、静态碰撞体），房间扩到 x = −9.84、y = −5.5，客区上方加两块顶灯（未标定）。椅子不在场景 USD 里，摆位写在 `usd/build_params.json` 的 `lounge.chairs`。
- **椅子** `Rigid/lounge_chair/00000`（蓝）、`00001`（白，只换颜色）：`scripts/objects/make_lounge_chair.py`，尺寸按照片估计，7 kg。`RigidObject` 新增布局键 `physics.mass_cap`（默认仍截到 0.5 kg，只有椅子放开），否则椅子 0.5 kg 一碰就飞。
- **env 配置** `g1d_counter_mobile`（`env_cfg/g1d_counter_mobile.yml`、`env_cfg/robot/g1d_counter_mobile.yml`）：移动版 G1-D，起点和升降柱同 `g1d_counter`，`body_states` 打开。`build_asset.py` 的 `SCENES` 加了它，`LOUNGE_TASKS` 给它生成客区任务布局（seed 0–2 各 10 个，按 `任务/seed/序号` 定种子）。
- **任务** `g1d_deliver_cup_to_lounge`：从台面拿塑料杯（`plastic_cup/00000`，\(|x|\) 0.06–0.16 m 随机左右、y −0.46 ~ −0.38），送到客区圆桌放下。判据（`delivered_checks()`）：杯心在桌心半径 −4 cm 内、高度没掉到地上、杯口朝上 15° 内、两个夹爪都张开。1500 步。
- **脚本专家**（`scripted_demo.deliver_cup_actions`）：侧抓（同倒豆的 `cup_grasp`；俯抓会把 30 g 塑料杯夹滑，第一次 GPU 运行就是空手开过去的），抬 8 cm，后退 0.7 m，挑椅子之间最朝向出发点的空档，开过去时把升降柱降到 0.07 m，转向圆桌，按到桌后实测的手–杯偏移放下，张开后沿手指退出再抬手。`Robot.drive` 是 `base_velocity` 的转向 + 直行控制器。
- **框架修复**：`func_parser.is_all_gripper_open` 归一化时没看 `gripper_move.sign`，G1-D 的 Dex1-1（sign −1，正向闭合）全开时算成 0，永远判不过；改成和 `obs_manager._to_gripper_scale` 一致。X5、Franka 的 sign 都是 1，结果不变。之前没有 G1-D 任务用到它。
- **验证**：
  - 本机：`check.sh`（ruff 0.11.7、41 passed 1 skipped、任务清单）；`build_scene.py` + `check_scene.py` 19/20（唯一 FAIL 是吧台站位的固定提醒）；`scene_parts.py` 重新生成 `parts.json`。
  - `5090-2` 隔离目录 `/data/robodojo-runs/counter-lounge-20261023-163001`（1 号卡，和 `owam-server` 共用；其余 7 张卡被另一位同事的 `gpu_hold.py` 占着）：两把椅子 `check_object.py` OK；脚本专家 `--env_cfg_type g1d_counter_mobile --seed 0` 前 4 个布局 4/4 成功（左手 1 个、右手 3 个），`_result.json` `success_rate` 1.0、`eval_time` 4。头部相机截图在本机 `output/deliver_head.jpg`（不入库）。
  - 没跑：完整 `build_asset.py`（服务器缺 `g1d_v4`/`unitree_ros`，布局用 `build_lounge_layouts` 单独生成），seed 1、2，`body_probe.py`，椅子被推开的专门测试。
- **共享资产库没动**：`lounge_chair`、新的 `Room/real_scene2`（带客区）、`Eval_Layout/RoboDojo/g1d_counter_mobile/` 都只在隔离目录里。装库要先征得用户同意；新 `real_scene2` 也会改变 `g1d_counter` 的画面左侧（原来是一面刷漆墙加挂画），吧台本身几何不变。
### RSI 协议初版 `rsi/AGENTS.md`（2026-10-24，直接推 `main`，只有文档）

- **用户要求**：① RoboDojo 评测用到的随机资产，生产数据时必须保留（第一规则；用户原话里的"拼车"按语音误识别理解为"评测"）；② 给 coding agent 能做泛化的操作（探索边际），避免押题、抄答案；③ 多 agent：一个改进数据、一个改进评测，二者对抗、不能互相通信；④ 写成类似 AGENT.md 的东西，给定模型、训练资源、评测平台就能跑 RSI。
- **内容**：三条硬规则；资产分三层（T0 官方保留、T1 考场池、T2 训练池，剩余资产 7:3 分 T2 / T1）；数据方、评测方、裁判（确定性脚本 + 人工）的权限和 seed 段；泛化算子表；成绩单只给分档的科目成绩（信息流唯一通道）；每轮流程、封存探针（画"轮数 → 泛化"曲线）、固定训练配方；裁判审计清单；`rsi/rsi.example.yaml` 输入配置；v0.1 缺口清单。
- **规则 1 的实测依据**（`5090-2` 上统计官方 `arx_x5/0` 全部 54 个任务）：房间 14 个全被评测用掉（基础任务 2 个，`_random` 12 个）；材质 504 种里评测用了 374 种，没用过的 130 种；杂物 193 个实例里评测用了 192 个（`_random` 191，`general_pickup` 145），官方训练数据里没有杂物；`_random` 独有物体实例 95 个；HDR 只有 1 张（基础任务也用）。所以房间和干扰物只能靠新资产做随机。
- **和现有代码的冲突**：10-23 提交的 `x5_layouts.py` 和平台随机方案能选到 T0 资产，违反规则 1，要先做裁判的 T0 统计和池子过滤（协议第八节 P0）。
- **验证**：只有文档；`check.sh`。
- **v0.2（同日，用户觉得 80% / 20% 不合适）**：看了官方仿真榜单（2026-10-24 版，61 个模型）后，难度调整改为用 Score（0–100 部分完成分）和相对本任务 L0 的保留率 ρ：ρ 的置信下界 ≥ 0.7 升级、上界 < 0.3 降级，每个科目、每个级别至少 30 回合，15% 回合留作 L0 锚点；L0 Score < 10 的任务算能力缺口，不调难度。依据（Gen-Std → Gen-Rand，按 Score）：OpenWAM-α 33.16 → 8.26（ρ 0.25，成功率 25.56% → 4.11%），Pi-05 20.93 → 5.82（0.28，14.89% → 1.44%），VPP2 0.40，Simate-beta 0.73，WorldScape-R²SI 0.84，HexaShell-Preview 0.81。总分：OpenWAM-α 17.18 / 11.92%（第 14），Pi-05 11.44 / 6.93%（第 22）。

### 上海实验室 `real_scene3_lab`：墙面去掉照片贴图，角落放一台 G1-D（2026-10-23，`chongzheng/scene-lab-walls`）

- **用户要求**：墙面不要用贴图；角落里那台 G1-D 直接放一个 G1-D 模型过去。
- **墙面**（`build_scene.py` 的 `WALL_SPANS`、`MULLIONS`、`DOOR`、`BOARDS`、`TV`）：四张墙面照片贴图和抠图补色的代码都删了。墙改成纯色饰面，颜色取照片中值：南墙是玻璃隔断（不透明绿灰色，带竖框）、白墙和一扇白门；北墙是白墙，挂一块黑边白板和一块黑屏；东西两面是深灰吸音板加踢脚线，挂黑边白板；西墙 y > 2.5 是白墙。位置由照片像素反投影得到，表格见 `PLAN.md`「墙面（v3）」。墙面网格带米制 UV，平台换材质可以用；材质库里实验室的 4 个照片墙条目换成 `paint`、`acoustic` 两个（`library.json` 只动了这几条，`test_server.py` 跟着改）。
- **G1-D**：用 `real2sim/robot/g1d_v4/g1_d_full_dex1_v4.urdf` 做正运动学（两肘 1.2 rad，手臂下垂和 p3 一致；其余关节为 0），材质和仿真里的 G1-D 一致（机身、手臂、底盘、升降柱哑光金属，夹爪、头、腰、轮子和前臂一圈黑色，规则抄自 `env/robot_manager/robot_config/g1d.py`；URDF 里的蓝白色是 CAD 默认色，仿真不用），每个网格单独抽稀，共约 5.5 万面，写成二进制层 `usd/g1d.usdc`（1.2 MB，走 LFS），由 `/Background/Furniture/g1d_0` 引用，位置 (−2.22, 2.92)、面朝 −Y。碰撞体是底盘、上身两个盒子；它不是 articulation，不会动。位置取 p3（p2 偏 0.4–0.7 m）。构建要多装 `fast-simplification`（`PLAN.md` 里的命令已更新）。
- **验证**：`check_scene.py` 24/24；`preview.py` 的四张照片对比里墙面分段和照片对得上，p3 视角下 G1-D 和照片里的机器人重合；`parts.json` 的墙面识别为 6 块（玻璃、门、竖框按名字排除）；本机 `check.sh` 46 passed、1 skipped。10-24 在 `5090-2` 隔离目录 `/data/robodojo-runs/lab-walls-20261024-1314/`（GPU 0；`Assets/` 拷自 `lab-furniture-install-20261022-085858`，只换了 `Room/real_scene3/`）验证：`render_scenes.py` 正常；`g1d_lab` seed 0 的拾取、叠块、量杯倒豆都是 1.0、`eval_time` 1。之后（用户指出夹爪不是黑的）改了 G1-D 的材质，重跑 `render_scenes.py` 和拾取 seed 0（1.0），头部相机里角落那台和工作的那台外观一致。10-24 经用户同意装进共享库 `5090-2:/data/robodojo-assets/Assets/Room/real_scene3/`（和隔离目录 `diff -r` 一致，布局没动、也没有布局引用旧墙面路径），旧版备份 `/data/robodojo-assets/backups/backup-shared-20261024-135419.tgz`；平台部署机只替换了这个场景 `preview/` 下的五个展示文件（旧的在 `/root/RoboDojo/.cache/lab-preview-before-walls-20261024-135438/`），部署机检出没快进（上面有别的会话没提交的改动），所以平台抽屉里的检查数和材质库条目还是旧的。
- **坑**：其他卡满载时，只给 `--device cuda:N` 不够，RTX 会用所有卡渲染，Vulkan 报显存不足后进程卡死（要 `kill -9`）。要加 Kit 参数 `--/renderer/multiGpu/enabled=false --/renderer/multiGpu/maxGpuCount=1 --/renderer/activeGpu=N`；`scripted_demo` 通过 `main(extra_kit_args=...)` 传。

### 数采平台：上游榜单任务的泛化随机方案 + X5 布局生成器（2026-10-23，直接推 `main`）

- **用户要求**：把上游引擎已经支持的随机项都在数采平台上实现（RSI 的"课程"要覆盖 RoboDojo `_random` 测的泛化）。实现是另一个会话（不在 Cursor 里，可能是 Codex / Claude 桌面端）在 `../RoboDojo-wt-leaderboard` 里写好、没提交的；用户让本会话接手，本会话审查、修了相机扰动、补测试和 GPU 验证后提交。
- **RoboDojo `_random` 测的**（统计官方 `arx_x5/0` 全部 12 对布局）：没见过的实例（编号和训练分开）、7–20 个干扰物、11–12 种房间、每 30 个布局 22–30 种桌面 / 地面材质；所有任务都随机位置，部分随机朝向。官方训练数据（34 个基础任务）全是同一房间、同一桌面、无干扰物，所以只用官方数据就是 clean-to-random。榜单不测的：灯光、HDR、相机扰动、指令改写、尺寸、颜色。
- **代码**：
  - `scripts/datagen/rsi_profile.py`：方案格式和校验（只用标准库，平台服务端直接导入）。方案写 `real2sim/benchmark_tasks/<任务>/randomization.json`，入库。
  - `scripts/datagen/x5_layouts.py`：上游引擎只回放布局、没有生成器，这个脚本以官方评测布局为结构模板重抽：位置 / 朝向只在任务配置自己的范围里（配置写死位置的物体不动，从不照抄评测布局）、实例只抽基础任务的训练侧编号（`_random` 独有编号不抽）、干扰物（杂物库、稳定面、±30°）、房间、桌面 / 地面 MDL 材质、HDR 和亮度、额外灯光（类型、数量、亮度、色温）、头部相机角度扰动（生成 git 忽略的 `env_cfg/arx_x5_rsi_<任务>.yml`、`env_cfg/camera/rsi_<任务>.yml`）。写 `split.json`（默认 train）和 `report_<任务>.json`；seed 0 拒绝。
  - 平台：任务抽屉新增「泛化随机方案」（只对上游任务显示）：物体（位置、朝向、实例，列出训练侧和留出编号）、干扰物、房间与表面（房间、MDL 材质搜索选择）、背景 HDR、灯光、相机扰动；`PUT /api/items?kind=task` 的 `randomization` 字段经校验后写文件；`GET /api/rsi-options` 给选项。`datagen/SKILL.md` 新增「上游榜单任务（dual X5）的随机布局」。CI / `check.sh` 加 `shapely rtree transforms3d`。
- **本会话改的**：相机扰动去掉平移，只留角度；方案里写 `pos_m` 会被校验拒绝。原因：`stack_bowls_random` 10 个布局开 ±2 cm / ±2° 时第 2、4、6 号并行环境头部相机最后一帧 45–98% 全黑（两次都是这三个）；同样布局不开相机扰动、或只开角度，10/10 正常。没查清平移撞到了什么（相机在 z = 1.308，`camera_stand` 顶部约同高，"只往上抬"也照样黑）。另外：ruff 格式、`.gitignore` 两条、`test_camera_configs`、页面文案。
- **验证**：本机 `check.sh` 46 passed 1 skipped（含 `test_x5_layouts.py` 4 项、`test_randomization_profile`），页面脚本 `node --check`。`5090-2` 隔离目录 `/data/robodojo-runs/rsi-layouts-20261023/`（另一会话 13:55 建的，本会话沿用；`Assets` 软链接到 `leaderboard-baseline-20261022` 的官方资产，`Eval_Layout` 单独）：
  - 另一会话已在 seed 901 给 54 个任务各生成 5 个布局；本会话用它的 `rsi_check.py` 复查：留出实例 0 次，干扰物 8–15 个；"和评测布局同位置"的都是配置里写死位置的物体（相机支架、棋盘棋子、麻将槽位等），"超出范围"都是锚点不在原点的槽位物体（检查脚本判得粗，官方布局本身也报）。`play_stacking_toy` 有 2 处足迹重叠没细查。
  - 端到端：全部随机项（14 个房间中的 Simple_Room、20 种桌面 + 20 种地面材质、HDR 亮度 600–1400、1–3 盏灯含平行光、8–15 个干扰物、相机 ±2°）→ `x5_layouts.py --seed 905 --count 10 --split val` → 保持姿态的策略（`rsi_hold.py`，不带 `--allow_train_layouts`）跑 10 个布局：退出码 0、`eval_time` 10、10/10 无黑块，画面各不相同。
- **没做**：页面没在浏览器里点过（只有服务端单测和语法检查）；没部署到 `5090-2:/root/RoboDojo`。

### 真机 Motus 倒豆模型在上海实验室的仿真评测（2026-10-23，`chongzheng/eval-motus-lab-beans`）

- **用户要求**：测 ModelScope `Vincia/WanVlmMask-G1DPourBeansGenEps222-AbsQpos16-30000` 在上海实验室（`g1d_lab`）仿真里倒豆子的成功率，只用量杯，不加干扰物。推理代码在 `rtx5090-180`（端口 34134，内网 `192.168.0.180`）的 `/home/work/wenjj/InferenceMotusV2`。
- **模型**：Motus（Wan2.2-TI2V-5B + Qwen3-VL-2B），从 380 条的 checkpoint 出发，用 222 条**真机**"倒豆子"数据后训练 3 万步；16 维绝对关节位置 qpos（内部顺序 [L7, LG, R7, RG]），每块 16 步、30 Hz；头部左眼加两个腕部相机拼成 T 形 384×320；指令只有"倒豆子"（带 T5 缓存）。权重已在服务器 `/home/work/wenjj/ckpt/` 下，预训练模型在 `/home/work/cache/wx1469573/pretrained_models/`；eps222 训练集在服务器上已经不存在。`/home/work` 盘 100% 满，自己的文件放根分区。
- **做法**（不改 InferenceMotusV2）：
  - 服务端 `rtx5090-180:/root/motus-sim-eval-20261023/serve_motus_sim.py`（本机副本在 `output/motus_lab_eval_20261023/`）：包装 `MotusV2PolicyAdapter`，按真机 `inference_motusv2_PourBeans.sh` 的串行路径调用（5 步去噪、插值 1、不开 RTC），协议同 OpenWAM 的 `serve_g1d.py`；Python 用 `/home/work/wenjj/envs/wallx`，要加 `PYTHONNOUSERSITE=1`（否则 `/root/.local` 的 torchvision 冲突）。每个实例 20 GB，加载约 2 分钟，单次推理约 0.63 s。`run_server.sh <gpu> <port>`，tmux `motus-sim-<port>`，已全部停掉。
  - 仿真端 `5090-2:/data/robodojo-runs/motus-lab-beans-20261023/`：入库 `scripts/g1d/remote_policy.py`（原来只在 OpenWAM 运行目录，原样收入）和 `env_cfg/g1d_lab_beans_collect.yml`（实验室 + 1/600 s）；`Assets/` 是 `lab-furniture-install-20261022-085858` 的硬链接副本（那份机器人 USD 按当前代码重建、在 `g1d_lab` 用脚本专家验证过；共享库的 `Robots/g1d` 和它不同），`Eval_Layout` 单独复制。两台机器内网直连。
  - 核对过的对齐：真机图像是 RGB（`process_images_and_observations` 做了 BGR→RGB），仿真也是；真机夹爪 0 闭合到 5.4 全开，夹住量杯读约 4.1–4.5，仿真夹住量杯实测开度约 0.75（≈4.05），线性换算一致；真机数据里腕部相机没有左右互换。没对齐的：真机起始姿态是双臂下垂（例如肘部约 −0.3），仿真默认肘部 −1.0、夹爪贴近头部相机。
- **结果**（seed 0 的 20 个布局各 1 次；`build_asset.py` 的标准布局：左手倒、两杯 45° 偏航、672 颗豆子、无干扰物；步数上限 1200）：任务判据 0/20，严格判据 0/20，矮杯里 0 颗。7 个回合碰洒了豆子（共 178 颗，最多 98 颗，是高杯被碰倒）。失败模式每个回合都一样：左手伸向高杯后夹爪闭合在空处（开度 < 0.5 持续 537–1098 帧），从来没有夹住过杯子；右手从不闭合。`_result.json` 三份，`eval_time` 合计 20。逐回合 JSON、轨迹 npz 和截图在本机 `output/motus_lab_eval_20261023/`。
- **验证**：本机 `check.sh` 41 passed 1 skipped（`test_server.py` 的 `g1d_lab` 现在对应两个 env 配置）；单回合探针后再跑全量，三路并行（服务端 GPU 1/4/5，仿真 5090-2 的 GPU 2 两路、GPU 7 一路），约 85 分钟。
- **没做**：每个布局只跑了 1 次（SKILL 要求 3 次；结果是 0 颗入杯，重复意义不大）；没试真机起始姿态、10 步去噪、夹爪阈值等变体；没有和脚本专家在同一批 20 个布局上对照（10-22 只验证过布局 0）。

### 场景中文名（2026-10-23，直接推 `main`）

- 用户要求所有场景都显示中文名。五个重建场景的 `scene_spec.json` 加 `name`：办公室升降桌、茶水间吧台、上海实验室（10-23 由公司实验室改名）、学校实验室绿幕桌、深圳实验室（同登记名）；平台扫描读它，卡片标题用中文名、下面一行小字是目录名。显示优先级：登记名 > `scene_spec.json` / 导入场景 `platform.json` / RoboDojo 自带场景的名字 > 目录名。给 coding agent 的建模任务文本和 SKILL 都要求新场景在 `scene_spec.json` 写 `name`。验证：`check.sh` 42 passed（`test_repo_scan` 断言五个场景都有名字）。

### 研究资产入库 + 界面按 UI UX Pro Max 优化（2026-10-22，直接推 `main`）

- **用户要求**：用 UI UX Pro Max skill 优化「研究资产」页；用户选定先把这块入库再改（它原来只在部署机上，是杨开元会话未提交的改动，见下一节）。
- **入库**：部署机工作区的平台改动（`prototype` 类型和扫描、研究资产页，以及同批的"导入的仿真场景"卡片和任务 `implementation` 进展显示，数据不存在时不显示）、`real2sim/prototypes/` 的元数据 / 文档 / 校验 JSON、`scripts/prototypes/`；`preview/`（图、视频、15 MB GLB）照 `.gitignore` 不入库，部署机上原样保留。
- **界面**（skill 推荐的模式：画面优先 + 按类别筛选，避免"层级弱、无筛选"；它推荐的衬线字体和学术配色没用，保持平台统一风格）：顶部搜索 + 分类分段按钮（带数量）；按「官方外形参考 / 柔性数值原型 / 待建模实物」分段，每段一句说明这类是什么、缺什么；卡片统一封面底色，封面角标 3D / 视频数，状态用图标 + 短标签（不只靠颜色）再附完整状态句，摘要限三行，底部「已核对 N · 待完成 N · 记录 N」；没图的卡片封面压成一条"还没有模型或图片"。抽屉：顶部状态、摘要、3D、预览图、视频网格（带文件名）、能力核对拆成「已核对」和「未完成 / 待核对」两组（按结论里有没有"未 / 待 / 无 / 候选 / 不同 / 非实测"分）、接下来要做、来源、检查记录按钮。导航图标改成和物体资产不同的 `layers`。
- **部署**（21:57）：入库时发现部署机上 5 个研究资产文件 21:51 又被那边的会话更新（软块 Isaac FEM 检查等），另补提交 `e02c2e6` 收进最新版再部署。部署机快进到 `e02c2e6`：已入库且内容逐字节一致的未跟踪文件先删再快进，`preview/` 不动；平台两文件不再是未提交改动；HANDOFF 合并时保留了部署机新增的两节绿桌任务笔记（刘小凡），删掉了重复的研究资产原文。备份 `.cache/deploy-ra-20261022-215710/`（含未提交改动 patch 和研究资产目录打包）。部署机上还有别的会话未入库的 `real2sim/scenes/g1d_green_table*`、`real2sim/tasks/g1d_green_table_five_stage`、`real2sim/tasks/g1d_behavior_pick_place_20261022`，没动。重启后部署机 `test_server.py` 8 项通过。
- **验证**：本机 `check.sh` 42 passed（新增 `test_prototypes`：七项都在、都不进任务、检查记录不能越出资产目录）；`scripts/prototypes/test_assets.py` 3 passed 2 skipped；本机浏览器看了列表、筛选、搜索、无结果、抽屉。

### 极光 Lite、通用法兰与柔性研究资产同步（2026-10-22，杨开元会话；原文取自部署机 HANDOFF，10-22 晚入库）

- 用户授权先核对官方模型、传感器及法兰，再同步到平台。定向部署本机独立分支提交 `ac31ff5`：平台新增 `prototype` / "研究资产"页，7项：极光Lite、通用法兰、布片、软块、长条拉链布袋、塑料包装袋、线缆；不进入任务对象/机器人候选。最近操作署名"杨开元（agent 代更新）"。
- 当前官方Aurora STEP与本地原件逐字节同源；USD/MJCF为静态整体几何，无运动/碰撞指垫或视触觉。CAD内G1-WS与当前单独WS模型版本名不同；无实机序列/SDK/逐设备标定。小面缺失和CAD包围盒/官网标称差异保留待核。
- 用户kaiyuan2提供的通用法兰与官网字节不同，但BRep双向差体积均为0，外径68、厚22.5mm与图纸一致。未取得G1-D腕端接口图/孔位与固定变换，没有套用旧Dex1安装或替换机器人。
- 布片和软块为已跑过独立数值测试的原型；袋/线缆仅reference登记、无原图文件及几何模型，明确待建模。厂商原件/派生USD、STL与MJCF放 `/data/robodojo-assets/research/yangkaiyuan-20261022/`，25文件约141MB，逐文件hash核对，目录只供root读取。没有安装共享Assets或修改默认夹爪/训练。
- 批次 `/data/robodojo-runs/research-assets-sync-20261022/`：`deployment.json`、`server_runtime_check.json`、`private_manifest.json`；备份 `backup-20261022-122100/`。
- 下一步：取得G1-D腕端安装接口和实机传感器序列/SDK标定，验证安装、运动、指垫接触/触觉后再考虑任务接入；三个实物参考仍需原图与尺寸/材料采集。

### 场景换材质 + 升降桌（2026-10-22，直接推 `main`）

- **用户要求**：场景建好后提取地板、墙面、桌面材质，每个场景可选别的材质替换；桌子都是升降桌，可调高度。用户选定：平台固定组合 + 数据生成时随机两层都做；材质库 = 场景提取 + 资产库 504 个 MDL；**调桌高时机器人不动**（站位和升降柱不变，测高度泛化）。
- **提取** `scripts/scenes/scene_parts.py`（usd-core，本机跑）→ 各场景 `usd/parts.json`：按形状找地板（顶面 z≈0 的水平网格）、墙面（沿房间轮廓高于 1.5 m，排除玻璃 / 门 / 框）、桌面（桌子分组里顶面 = top_z 的薄网格），记材质和每个网格的 UV 尺度；升降：伸缩接缝（build_params `legs.split_z`，没有就取半高）以上的网格上移，跨接缝或止于接缝的拉伸；桌上的场景物体跟着上移。五个场景都提取到三类表面；吧台 `real_scene2_counter` 是落地柜，标为不升降。办公室墙面只剩一面实墙（其余是玻璃隔断和窗）。
- **材质库** `real2sim/materials/library.json`（入库）：523 个 = 19 个场景材质 + 504 个 MDL，按名字分 12 类；按表面过滤（用户 18:53 指出地板和墙面能选到同样的材质）：`appearance.ROLE_CATEGORIES` 规定地板（木材 / 瓷砖 / 石材 / 织物 / 塑料）、墙面（涂料 / 瓷砖 / 石材 / 木材 / 纸 / 织物 / 塑料）、桌面（木材 / 石材 / 塑料 / 金属 / 玻璃 / 瓷砖）可用的类别，场景提取的材质只给原来的表面，室外和"其他"（轮胎、反光片、屏幕、水）都不给；`/api/materials` 每项带 `roles`，选择器按它过滤，保存和 `appearance.py` 都按同一规则拒绝；可选数量地板 242、墙面 279、桌面 354；缩略图 `real2sim/materials/preview/`（375 张，不入库，部署机要同步）。在 5090-2 用共享库 `Material/` 生成（读图要 4–5 分钟）。
- **运行时** `env/scene_manager/objects/room_overrides.py`，`Room` 加载后调用（只有布局 Room 带 `appearance` / `table_lift` 时）：建 UsdPreviewSurface（贴图按原真实重复尺寸缩放 UV）或 MDL（OmniPBR 世界坐标投影，每 1 m 重复）并换绑；桌子网格加最外层变换（上移或按底部拉伸），在 PhysX 解析前完成。不改 Room 资产，所以不用动共享库。
- **布局** `scripts/scenes/appearance.py`：读 `parts.json` + 平台 `appearance.json` + 材质库，把选择写进布局 Room（`appearance`、`table_lift`、`appearance_choice`、`table_height`），物体随桌面平移（按当前 `table_height` 算差，重复应用不会累加）；`--randomize --seed` 时有随机池的部分每个布局按文件名可复现地抽。`build_asset.build_layouts` 在场景有 `appearance.json` 时自动套固定设置（没有就和以前完全一样）。
- **平台**：`/api/materials`；场景扫描带 `parts`、`appearance`；建好的场景抽屉加「外观与桌高」：每类表面一个固定替换（单选）和随机池（多选，可含原材质），复用任务向导的搜索选择器（缩略图、悬停预览、按类别中文名搜）；固定桌高、随机桌高范围（限行程 0.62–1.27 m）。保存时服务端校验材质 id、行程、不升降的场景，再写 `appearance.json`（全空则删除）。
- **3D 预览**（用户随后要求：Web 3D 预览加换地板、墙面、灯光、桌板）：抽屉 3D 预览下方加控制面板：地板 / 墙面 / 桌面各一个材质选择器（同一个搜索选择器），桌高滑块（按 `parts.json` 上移、拉伸同一批网格；预览模型里桌上的摆件按位置识别后一起上移，仿真 Room 里没有这些摆件），灯光亮度、色温、主光方向。MDL 用底色缩略图近似（1 m 重复），没有 UV 的网格（办公室桌面）现场按法线投影生成米制 UV。「设为固定设置」把预览选择填进「外观与桌高」（再保存才写 `appearance.json`）；「载入固定设置」反向。灯光只影响预览，没写进场景（仿真灯光在 USD 里）。
- **验证**：本机 `check.sh` 40 passed（`test_appearance.py` 4 项、`test_server.py` 新增外观设置校验）。5090-2 隔离目录 `/data/robodojo-runs/scene-appearance-20261022-1807/`（资产覆盖层：只有 `Eval_Layout/RoboDojo/g1d_lab` 是新目录，其余软链共享库，未改共享库），`g1d_lab` 布局目录 7（地板 MDL 橡木、墙深圳墙色、桌面 MDL 大理石、桌高 0.90）和 8（地板吧台水磨石贴图、墙实验室北墙照片、桌面 MDL 桃花心木、桌高 0.70），拾取、叠块各 2 回合：画面里材质都换上、桌子升降对、物体停在新桌面上、无 PhysX 报错；成功率 0.90：叠块 0/2、拾取 0/2；0.70：叠块 1/2、拾取 0/2；原布局对照：叠块 2/2、拾取 1/2（机器人不随桌高变，脚本专家按原桌高调）。第一次在 GPU 4 上跑报 PhysX GPU 内核启动失败，GPU 4 当时被别人占到 30/32 GB，换 GPU 5 后同样布局无报错。平台本机起服务用浏览器走了一遍：选材质、保存、刷新后保留、3D 预览换材质 / 灯光 / 桌高（实验室、办公室）。
- **没做 / 风险**：材质缩略图有 172 个 MDL 没有（只有打包通道图或没有底色图），选择器里显示图标、预览里用灰色；场景提取的贴图材质 UV 尺度换到别的网格上是按"每米 UV"换算的，照片墙贴到别的墙上会被拉伸重复，只适合做测试（实验室四面墙 10-23 起已是纯色）；`appearance.json` 的随机池还没接到 `run_batch.py`，采集前要手动对布局目录跑 `appearance.py --randomize`；共享库评测布局没有重新生成（各场景都还没有 `appearance.json`）。
- **部署**（18:47）：`5090-2:/root/RoboDojo` 快进到 `ea2ea54`，做法同下面「删除场景」（另一会话未提交的平台改动和 HANDOFF 三方合并无冲突，合并前备份 `.cache/deploy-overlap-20261022-184728/`），缩略图拷进 `real2sim/materials/preview/`，重启后部署机 `test_server.py` 7 项通过。注意部署机上学校场景有另一会话未提交的几何改动（U 形绿幕），而 `parts.json` 是按仓库版本提取的，那边入库后要重跑 `scene_parts.py`。
- **文档**：`real2sim/scenes/SKILL.md`「换材质和升降桌」+ 接入步骤 4；CONTRACT「仿真配置」加一条：外观和桌高随布局记录，评估要用同样设置的布局。

### 数采平台：RoboDojo 自带场景（2026-10-22，直接推 `main`）

- **用户要求**：把 RoboDojo 自身的场景也放进场景重建页。
- **做法**：`scan_benchmark_scenes` 读 `env_cfg/scene/*.yml`（标准库逐行解析），记为 `robodojo_default`、`robodojo_conveyor`（`auto.source = "robodojo"`）：房间、桌子尺寸 / 高度 / 材质、HDR 背景，以及按 `_task.yml` 的 `scene_config` 选用它的上游任务（default 52 个，conveyor 2 个）。场景页在孪生卡片下面单列「RoboDojo 基准场景」，抽屉只读（配置、任务链接、重渲染命令），没有验收和删除（服务端 409）。总览的验收计数和 G1-D 任务向导的场景选择器不含它们。
- **封面**：`render_scenes.py` 现在也接受 `env_cfg/scene/<name>.yml`，用环境自己的 `Room` / `Table` / `Background` 类从 `Assets/` 生成，输出 `real2sim/benchmark_scenes/<name>/preview/render/sim.jpg`（`preview/` 不入库）。`5090-2` 隔离目录 `/data/robodojo-runs/bench-scene-render-20261022-1359/`（GPU 6，资产软链共享库只读）两个都退出码 0；default 是悬空桌板（上游桌子本来就只有桌面），conveyor 是空房间（传送带由任务生成）。
- **验证**：`test_server.py` 新增扫描断言（桌高 0.765、`random: False` 解析成布尔、conveyor 两个任务、无 G1-D 任务）和删除 409；本机 `check.sh` 35 passed；本机起服务用浏览器看了场景页和抽屉。
- **部署**（14:05）：`5090-2:/root/RoboDojo` 快进到 `ed49a76`，做法同下面「删除场景」：另一会话未提交的平台改动和 HANDOFF 三方合并无冲突（合并前备份 `.cache/deploy-overlap-20261022-140459/`）；两张封面拷进 `real2sim/benchmark_scenes/`；重启后扫描 8 个场景（含两个基准场景）、研究资产 7 项，部署机 `test_server.py` 6 项通过。

### 榜单基线：OpenWAM、pi0.5 官方 checkpoint 全量评测（2026-10-22，进行中，没改仓库代码）

- **20 天计划**：① 基线 + 失败归类；② 把 `scripts/datagen` 推广到 dual X5（CONTRACT 补 X5 语义），Coding Agent 按"差距 × 好写程度"给任务写专家（特权状态 + cuRobo，验收严于 `run_reward`）；③ 第 1 轮数据 → dsw-4 微调 → 内部验证集评测；④ 两轮对抗：训练分布内大范围采样找失败、聚类、在失败附近生成难例，加 DAgger 式恢复数据；⑤ 官方 seed 终评。官方评测布局（`Eval_Layout/RoboDojo/arx_x5/0`）只在基线和终评各用一次，迭代看内部留出集。
- **关键事实**：
  - 官方训练数据 = HF 数据集 `RoboDojo-Benchmark/RoboDojo` 的 `data/RoboDojo/<任务>/arx_x5/data/`，34 个基础任务各 100 条（单任务 28–46 GB，共约 1.3 TB）；dsw-4 已有完整副本 `/mnt/data/datasets/RoboDojo`（另有 `RoboDojo_ee_lerobot`）。
  - **54 个榜单任务里 20 个没有官方数据**：12 个 `_random` 变体，加 `align_blocks`、`classify_objects_by_language`、`general_pickup`、`pick_from_conveyor_by_image`、`pour_by_language`、`solve_equation`、`stack_blocks_by_language`、`store_tools_in_toolbox`。这是自生成数据最大的抓手。
  - 官方 checkpoint 也在同一 HF 数据集的 `ckpt/RoboDojo/<模型>/`：pi0.5 是 `Pi_05/RoboDojo-sim-arx_x5-joint-{0,1,2}/59999`（joint）；OpenWAM 是模型仓库 `OpenWAM/OpenWAM-Alpha-Sim-RoboDojo`（`checkpoint_step_60000`，24.8 GB，ee）。
  - 仓库里没有任何榜单任务的脚本专家，只有 `env/planner_manager/curobo_planner.py`。
- **运行目录** `5090-2:/data/robodojo-runs/leaderboard-baseline-20261022/`：代码是本机 `a694da5` 的 `git archive`（`REVISION`）加本机 XPolicyLab `53c5da5`（rsync），`third_party` 软链接到 `office-beans-random-20261019`。`Assets/` 是 HF 官方 Assets 的干净副本（15365 个文件，经 hf-mirror 逐文件下载，最后 521 个从 dsw-4 经本机中转），不是 `/data/robodojo-assets` 共享库（那份有 G1-D 改动）；下载后跑过 `utils/update_embodiment_config_path.py`（不跑会缺 `Robots/x5/curobo.yml`，而 `robodojo.sh eval` 照样返回 0）。checkpoint 在 `ckpt/`，软链接进两个策略的 `checkpoints/`。
- **环境（5090-2 上新增）**：`openwam-bench`（克隆 `openwam`，再装 vendored OpenWAM；原 `openwam` 指向 G1-D 分叉，给真机服务用，没动）；pi0.5 的 uv 环境在运行目录 `XPolicyLab/policy/Pi_05/openpi/.venv`（JAX 0.5.3，uv 在 `.uvtool/`，依赖经本机 `ssh -R` SOCKS 隧道装，隧道已关）；tokenizer 预置在 `~/.cache/openpi/big_vision/`；conda base 装了 `pyyaml`（Pi_05 服务端脚本用 base python 读 yaml）。Isaac 要 `OMNI_KIT_ACCEPT_EULA=YES`（`smoke_one.sh`、`bench.sh` 已设）。
- **下载坑**：5090-2 连不上 huggingface.co、GitHub、PyPI。hf-mirror 要 `HF_HUB_DISABLE_XET=1`；`snapshot_download` 和超过 1000 个文件的 `list_repo_tree` 会卡住（分页链接指回 huggingface.co）；git LFS 经镜像全部失败。可行的是按文件列表 `hf_hub_download`（`dl_prefix.py --list`）。
- **验证**：两个模型 `stack_bowls` 各 1 回合：OpenWAM 成功（314 s），pi0.5 成功（193 s），`_result.json` `eval_time` 1。
- **全量评测（13:40 启动）**：`bench.sh` → `robodojo.sh benchmark --eval-num native --tasks-file leaderboard_tasks.txt`（54 个任务，去掉 `g1d_*`）。OpenWAM run id `baseline_openwam_20261022`，策略 GPU 0/2/3，仿真 GPU 4/5/6（策略 25 GB，不能和 Isaac 同卡），预计每组约 16 h；pi0.5 run id `baseline_pi05_20261022`，GPU 7 单卡串行，实测 `align_blocks` 50 回合约 25 min，全量约 22 h。汇总在 `smoke_results/<run id>.md`，日志 `logs/bench_*.log`。GPU 1 是 `owam-server`（另一会话）。
- **16:35 按用户要求停止**（两个 `bench.sh` 会话组和 4 个残留 Isaac 客户端都已结束，GPU 0、2–7 释放，`owam-server` 未动）。停止时完整跑完的任务（成功率 / 回合）：OpenWAM `imitate_sorting_sequence` 0/50、`pour_by_language` 0/50；pi0.5 `align_blocks` 0/50、`arrange_largest_number` 0/25、`arrange_largest_number_random` 0/25、`build_tower` 0.48/50。中途停下的（`_result.json` 只有部分回合，不能当基线）：OpenWAM `play_tic_tac_toe` 0.1/40、`classify_objects_by_language` 0/30、`play_stacking_toy` 0/20；pi0.5 `classify_objects` 0.1/10。续跑：pi0.5 是串行模式，每个任务写一次 `smoke_results/baseline_pi05_20261022.json`（已有 4 个 PASS），同一 `bench.sh` 加 `--resume` 即可跳过；OpenWAM 是并行模式，summary 只在全部结束时写，停止时没有生成，`--resume` 不起作用，要从 `leaderboard_tasks.txt` 去掉已完成的 2 个任务另做任务列表。中途停下的任务都会整任务重跑。

### 数采平台：管理员删除场景（2026-10-22，直接推 `main`）

- **用户要求**：场景重建里加删除，只有管理员能删。
- **做法**：`DELETE /api/items?kind=scene` 非管理员回 403；未建模的场景整目录删除（`registration.json`、照片、预览一起删，原来只删 `registration.json`，照片目录留着，下次扫描又冒出来）；已建模的场景（有 `*.usd` / `usd/`，或在 `build_asset.SCENES` 里）回 409，提示在仓库里删。场景抽屉的「删除场景」按钮只对管理员、且对未建模场景显示（已登记未建模和只有照片的都有）；原来人人可见的「删除登记」去掉了。
- **验证**：`test_server.py` 新增成员 403、已建模 409、整目录删除三条；本机 `check.sh`（ruff 0.11.7 经 `uvx`）35 passed；页面脚本 `node --check`。
- **部署**（13:22）：`5090-2:/root/RoboDojo` 从 `4f8cccf` 快进到 `fafb475` 并重启服务。部署机上有另一会话（杨开元）未提交的「研究资产 / prototype」平台改动、学校场景改动和 HANDOFF、ASSETS 笔记：平台两文件三方合并无冲突；`HANDOFF.md` 两边段落都留，`ASSETS.md` 实验室、深圳行取主线、学校行取部署机版本；仍是未提交状态，等那边入库。合并前的四个文件备份在 `.cache/deploy-overlap-20261022-132146/`，全部未提交改动的 patch 在 `.cache/deploy-dirty-20261022-132106.patch`。没用 `git stash`：部署机跳过 LFS smudge，stash 回来会把 `real_scene4.usd` 写成指针。新提交里 7 张实验室贴图（LFS）从本机 rsync 了实体。部署机上用系统 python 跑 `test_server.py` 6 项通过，研究资产 7 项仍在。
- **注意**：部署机上删除的是工作区文件；已入库的场景（如 `real_scene5_shenzhen_lab` 的登记和照片）被删后部署机检出会变脏，需要入库同步。

### 深圳实验室倒豆 500 条（2026-10-22，`chongzheng/datagen-shenzhen-beans` 快进进 `main`，已上传 ModelScope）

- **用户要求**：深圳实验室场景；高量杯倒进矮量杯或白色塑料马克杯；2–5 个无关干扰物；500 条；多卡并行。看过首批视频后改为：红豆 300–600（原 100–300）、抓取要稳、两杯不能碰。
- **数据集** `g1d_shenzhen_pour_beans_cup_or_mug_500`：500 回合、199697 帧、30 Hz，XPolicyLab 官方转换器出的 LeRobot v3.0；目标矮量杯 257、马克杯 243；豆数 320–594；干扰物 2/3/4/5 个各 140/123/119/118 回合。私有仓库 `ChongZheng/g1d_shenzhen_pour_beans_cup_or_mug_500`。registry 已登记。
- **代码**：任务 `g1d_pour_beans_cup_or_mug`（指令按目标说 “…into the short measuring cup / the white mug.”）；插件 `scripts/datagen/plugins/g1d_pour_beans_cup_or_mug.py`，验收 `all_beans_retained_any_target_v2`：结尾 30 帧全部豆子在目标杯腔内、两杯 ≤ 15°；整回合两杯杯身不接触（圆柱采样 + 马克杯把手）、目标杯 ≤ 15°、干扰物位移 ≤ 2 cm，违规即结束回合；`scripts/objects/plastic_mug_shape.py`（马克杯尺寸、原点偏移、杯腔）；`scripted_demo.cup_to_cup_actions` 按杯种取夹持高度、下倾角、杯腔，源杯先在原地升到倒豆高度再横移（旧版 43% 回合在搬运时擦碰）；布局 `scripts/g1d/prepare_bean_pour_layouts.py`（复用办公室采样器；马克杯把手朝持杯手指方向 ±40°；豆数目标 350–630，因颗粒网格采样实际 320–594；干扰物从 19 类 Clutter 直立放在桌后部和两侧，离杯 ≥ 0.16 m）；env 配置 `g1d_shenzhen_lab_beans_collect`；`run_batch.py --per-gpu N`，以及进程位多于分片时空队列 pop 崩溃的修复。
- **运行** `5090-2:/data/robodojo-runs/shenzhen-beans-20261021`，`output/datagen/shenzhen-beans-v2`，代码 `1d39c9d`（rebase 前的提交号，内容同 `main` 上的对应提交）。GPU 0/6/7 每卡 3 进程，6.3 小时，751 回合 505 合格（67%；矮量杯 262/372、马克杯 243/379；左手 246/364、右手 259/387）。不合格：结尾条件 143、目标杯倾斜 87、两杯接触 15、干扰物被碰 1。100–300 豆的 v1 批次（199 回合 156 合格）已弃用。
- **验证**：导出读回 500/199697；`audit.py` 500/500 重算验收通过，布局互不相同（`final_validation.json`、`sha256.json`、`coverage.png`）；开环回放 seed500、seed509 各 4 条：手臂状态偏差 ≤ 2e-3 rad，6/8 复现合格，1 条豆子落点不同（190 颗留在源杯，颗粒物理开环发散），1 条稳定帧 29/30；上传后 `upload.py verify` 逐文件核对 sha256。本机 `check.sh`。
- **没做**：55 N 夹爪 A/B（另一会话 `c575c8e`）合格率同为 16/28，没合入；指令只有一种英文说法。

### 实验室场景 `real_scene3_lab` 的家具建成立体模型（2026-10-22）

- **用户要求**：实验室场景要把桌子、椅子、柜子也重建出来，而不是只贴图。原来只有操作台是几何体，房间里的家具都压平在四面墙的照片上。
- **做法**：相机沿用 `fit_photo.json`，在照片上手标落地点（桌脚、椅脚、柜底边），反投影到地面；p1、p4 都拍到的铁皮柜做三角化。尺寸取常见规格，再用照片核对。`build_scene.py` 新增 `FURNITURE`（15 件：北排 3 张工位桌 + 4 把网椅，西侧 2 张工位桌 + 2 把网椅，活动柜带纸箱，两门文件柜，木面边桌，圆咖啡桌）和六种参数化模型，放在 `background.usda` 的 `/Background/Furniture/`。这样 Room 入口、`render_scenes.py` 的取景（只对准 Background、Lights 以外的部分）和平台的「隐藏房间」都不用改。碰撞体都是盒体或 ≤ 50 点的凸包。墙面贴图把家具的投影区域膨胀 21 px 后抠掉，按行中值补色；操作台那块原来用 Telea 修补，也改成按行补色。
- **照片之间的矛盾**（写进了 `PLAN.md`）：p2 的相机在画面边缘外推不准，按它算西墙转了约 13°，远处高度也偏低约 5 cm，所以西侧家具按与墙平行摆放。北排最右那张桌子按 p3 摆在 y≈3.05，p4 却要求 y≈2.45，相差 0.6 m；p4 左缘这一块用 `HAND_MASK` 手标多边形抠掉。圆桌只在 p4 出现，高度按 0.75 m 假设。
- **验证**：`check_scene.py` 24/24。新增 5 项：碰撞体 ≤ 64 点、家具落地、不进墙、离 G1-D 根节点 1 m 以上且离操作台 0.3 m 以上、家具之间不重叠；原来的站位等检查改成只看操作台。`preview.py` 的叠图用绿色画家具边线：p1、p3、p4 误差约 10 px，p2 约 20 px。本机 `check.sh` 29 passed 1 skipped；`export_glb.py` 输出 205 个网格、2.9 MB。`5090-2` 隔离目录 `/data/robodojo-runs/lab-furniture-20261022-003901/`（GPU 4）只载场景跑 `render_scenes.py`：退出码 0，没有报错，家具正常渲染、有投影。带机器人的脚本演示见下面「装共享库」。
- **平台**：部署机 `/root/RoboDojo` 只替换了这个场景 `preview/` 下的五个展示文件（`compare_photo.jpg`、`overview.jpg`、`robot_head.jpg`、`scene.glb`、`render/sim.jpg`，都不入库），旧文件备份在 `.cache/lab-preview-before-legs-20261022-112855/`。部署机的检出仍停在 `4f8cccf`，没有快进：上面有别的会话没提交的 `HANDOFF.md`、`airpods` 的 `spec.json`、`real2sim/scenes/real_scene4_school/` 的 9 个文件，以及未跟踪的 `real2sim/scenes/g1d_green_table/`，而且部署机这次连不上 GitHub。所以平台抽屉里这个场景的检查数、尺寸、拟合残差还是旧的，等下次快进再更新。浏览器缓存的旧图要刷新两次（`/file` 先显示缓存，再在后台校验）。
- **桌腿改直**（10-22 用户指出：中间桌子的腿是直的，不是斜的）：v1 在 p1 上标的四条立柱线其实落在 V 形底脚上，把立柱拟合成了外倾 15°。现在 `fit_photo.py` 只用一个 `col_x`（竖直），立柱线改在 p3 的两根立柱中线上取，结果 `col_x = 0.638`（桌端往里约 5 cm），相机变化 ≤ 3 cm，焦距 786.5 → 790.6 px。试过再加一个 Y 方向倾斜，拟合出来只有 1.5 cm，所以不加。p3 立柱线残差 4–15 px（暗处取点），其他照片的点残差不变。重新验证：`check_scene.py` 24/24；隔离目录重跑 `build_asset.py`，`g1d_lab` 的叠块、拾取 seed 0 都是 1.0（两个演示同时启动时，有一个读到同时生成的机器人 USD 半成品而报错，单独重跑就好了）；RTX 渲染重出。共享库的 `Room/real_scene3` 再次替换，旧版备份为 `/data/robodojo-assets/backups/backup-shared-20261022-112705.tgz`。
- **装共享库**（用户 10-22 同意：先验证再替换）：
  - 隔离目录 `5090-2:/data/robodojo-runs/lab-furniture-install-20261022-085858/`：资产复制自 `shenzhen-lab-20261021-125246`，缺的积木、小车和三类杂物另外补进来。在里面跑 `build_asset.py`（退出码 0），再用 `g1d_lab` 跑 `scripted_demo` 的 seed 0：拾取、叠块、量杯倒豆都是 `success_rate` 1.0、`eval_time` 1，五路相机画面正常。
  - 踩坑：起初拾取、叠块的 34 个 seed 全部判为不稳定。用旧房间、办公室场景对照，结果一样，说明和家具无关：复制来的资产里 `block`、`car` 只有元数据，没有 `object.usdz`。**5090-2 上的运行目录资产不全，跑别的任务前先确认 `object.usdz` 在。**
  - 只替换了共享库 `5090-2:/data/robodojo-assets/Assets` 的 `Room/real_scene3/`（`diff -r` 和验证产物一致），布局没动。旧版备份在 `/data/robodojo-assets/backups/backup-shared-20261022-101545.tgz`。
- **没做**：家具没有卷尺实测。（墙上的照片贴图和压平在上面的杂物 10-23 去掉了，见本节上方的 v3。）

### G1-D 桌面版 / 移动版（2026-10-21，[PR #17](https://github.com/Zheng-Chong/RoboRSR/pull/17) 合并进 `main`）

- **用户要求**：仿真支持腰部、升降和底盘移动，分成桌面操作版和移动操作版；底盘和升降按官方方案。
- **官方方案**（unitree_sdk2 `example/g1/g1d`、`g1_agv_client.hpp`，宇树 G1D 开发者文档）：标准版推拉底盘无电驱，17 自由度（两臂 14、腰 2、升降 1）；旗舰版两轮差速，19 自由度。腰是底层 12 WaistYaw、14 WaistPitch 位置控制；升降是 `AgvClient.HeightAdjust` 速度指令（最大 76.5 mm/s，高度读 `rt/hispeed_state`）；底盘是 `AgvClient.Move(vx, 0, vyaw)`（±1.5 m/s、±0.6 rad/s，`vy` 忽略）。官方 `g1_d.urdf` 只有两个驱动轮、没有万向轮，也没有任何仿真配置（`unitree_sim_isaaclab` 不支持 G1-D）。官方 URDF 里腰部关节名就和转轴对不上（`torso_Joint` 绕 z 是偏航，`Yaw_Joint` 绕 y 是俯仰），不是 v4 改的，CONTRACT 已更正。
- **决策**（用户选定桌面版身体自由度为可选键）：
  - 动作键：`waist_joint_state` [偏航, 俯仰] rad，`lift_joint_state` [高度] m（0–0.42，两级各一半），`base_velocity` [vx, vyaw]（仅移动版）。可选，缺省保持（底盘缺省为 0），旧 16 维策略不用改。
  - 移动版底盘用虚拟平面关节（`base_x/y/yaw_joint`，在起点坐标系里），按差速运动学把 (vx, vyaw) 积分成位置目标并下发速度前馈；目标最多领先实测 1 cm / 0.02 rad，撞上东西会停而不是蓄力。轮子锁死、去掉碰撞体（两个轮子没有万向轮，靠轮子驱动会翻；轮子碰撞会拖地），底盘外壳（离地 3.4 cm）照常碰撞。
  - 升降和腰按官方限速（0.0765 m/s 分到两级、5 rad/s）；复位时直接把身体关节写回默认值（升降柱 1 s 内开不回来）。
  - 观测要 env 配置 `observation.robot.body_states: true` 才加（只有 `g1d_mobile` 打开），所以桌面版六个配置的观测和以前完全一样。
- **改动**：`env/robot_manager/robot_class/g1d.py`（身体键 ↔ 关节的映射）、`robot_manager.py`（关节索引、复位、初始保持值、控制下发、`get_body_state`，注册 `g1d_mobile`）、`robot_config/g1d.py`（`mobile` 参数；执行器拆成 lift、waist、base）、`robot_config/g1d_mobile.py`、`x5.py`（`asset_name`，移动版读 `Assets/Robots/g1d/`）、`obs_manager.py`、`eval_env.py`（校验、身体键转控制、腰和升降在动作内插值）、`build_asset.py`（多写 `g1d_mobile_dex1.urdf`，`SCENES` 加 `g1d_mobile` 生成办公室布局）、`env_cfg/g1d_mobile.yml`、`env_cfg/robot/g1d_mobile.yml`、`_robot_info.json`、新测试 `scripts/g1d/test_g1d_body.py`（进 `check.sh`）、GPU 自检 `scripts/g1d/body_probe.py`；README、CONTRACT 同步。`test_server.py` 的期望改为办公室场景对应 `g1d` 和 `g1d_mobile` 两个配置。
- **验证**：
  - 本机：ruff 0.11.7（改动文件）、`git diff --check`、pytest 26 passed 1 skipped（XPolicyLab 空）、任务清单检查。
  - `5090-2` 隔离目录 `/data/robodojo-runs/g1d-mobile-20261021/`（资产从 `school-scene-20261020-114517` 复制，`g1d_mobile_dex1.urdf` 由 `mobile_urdf()` 从其中的 `g1d_dex1.urdf` 生成，`Eval_Layout/RoboDojo/g1d_mobile` 软链接到 `g1d`；没跑 `build_asset.py`，那边没有 `unitree_ros`）。用量杯任务 seed 100（运行目录只有量杯布局）：
    - `body_probe.py --env_cfg_type g1d_mobile`，2 回合都通过：升降 +0.1 m、腰 [0.3, 0.2]、后退 2 s 到 -0.6 m、原地转 2 s 到 0.6 rad，误差都 < 1e-5；躯干世界位姿跟着底盘走；不发底盘键 0.5 s 后速度 4e-5 m/s；第二回合开始时底盘和升降回到默认；`_result.json` success_rate 1.0、eval_time 2。截图在本机 `output/g1d_body_probe_20261021/`。
    - `body_probe.py --env_cfg_type g1d`，2 回合都通过，桌面版拒绝 `base_velocity`。
    - 回归：`scripted_demo.py --env_cfg_type g1d --task_name g1d_pour_beans_measuring_cups --seed 0` success_rate 1.0、eval_time 1。
  - 踩坑：运行目录的资产只有量杯布局，拾取任务 seed 0 没有布局时 `main.py` 不报错直接退出；Kit 退出时会吞掉缓冲的 stdout，要 `PYTHONUNBUFFERED=1` 才看得到 `[main]` 输出。

### 深圳实验室场景 `real_scene5_shenzhen_lab`（2026-10-21，[PR #22](https://github.com/Zheng-Chong/RoboRSR/pull/22) 合并进 `main`，已部署）

- **输入**：平台登记 `registration.json`（卷尺：桌面 0.75 m 高、1.20 × 0.60 m、厚 2 cm；储物柜 75 × 120 × 45 cm；后沿贴墙；顶条面光源）和 6 张照片，从 `5090-2:/root/RoboDojo/real2sim/scenes/real_scene5_shenzhen_lab/` 同步，入库。
- **建模**（从 `real_scene3_lab` 的写法复制）：`fit_photo.py` 在 5 张照片上手标桌沿、立柱、桌脚、柜角（4915 和 4914 同机位，不用），桌面尺寸、厚度、柜子尺寸、贴墙都固定成登记值，只拟合相机、共享焦距、桌腿和柜子位置。桌沿残差 ≤ 3.7 px；焦距 773.6 px，与 EXIF 26 mm 等效的 769 px 一致。`build_scene.py` 出 `usd/`（背景、桌子、柜子、灯光四层）和 `real_scene5.usd`；碰撞体全是盒体。`check_scene.py` 22/22。`acceptance.py`：桌高、长、宽误差 0。
- **和登记不一致**（按照片建，写进了 `PLAN.md` 和 `scene_spec.json`）：“四条腿”实为两条 T 形升降腿；“条纹瓷砖”实为方块地毯；桌上有 ChArUco 板但没量格子，没用。
- **站位未实测**：`env_cfg/robot/g1d_shenzhen_lab.yml` 根节点 y=-0.73（桌沿在前 0.13 m），桌子中线；`SCENES` Room 位置 (0, -0.30, 0)，物体不平移；升降柱各 0.035（桌高比办公室高 0.07 m）。
- **灯光**：4 条 1.2 × 0.15 m RectLight，6000 K，强度 54000；`lighting.status: recorded`。在 Isaac 里把 `cam_third` 放到 4916 的拟合位姿对了三轮颜色（数值表在 `PLAN.md`），最终墙 / 桌 / 地毯和照片差 12 / 10 / 1 sRGB 级。
- **顺手修的 bug**：`build_asset.py` 的 `seat_wrist_camera` 在临时目录里建 meshes 软链接时用了 `out` 的原样路径，`--assets` 传相对路径时链接断开，报 `Missing mesh: .../AGV_link.STL`；改为 `.resolve()`。默认参数（绝对路径）不受影响。
- **验证**：
  - 本机：`scripts/check.sh` 通过（22 passed，1 skipped）；新场景脚本单独跑过 ruff 0.11.7；`test_scenes.py` 加了 `g1d_shenzhen_lab` 的期望站位。
  - `5090-2` 隔离目录 `/data/robodojo-runs/shenzhen-lab-20261021-125246`（GPU 3）：资产 = 学校场景运行目录的 `Assets` + 平台镜像的 `arx_x5` 布局和物体元数据；`unitree_ros` 里唯一需要的 URDF 在本机下载后放进 `.cache/unitree_ros`（5090-2 上不了 GitHub）。`build_asset.py --assets Assets` 退出 0，`g1d_shenzhen_lab` 739 个布局。`scripted_demo --env_cfg_type g1d_shenzhen_lab --task_name g1d_pour_beans_measuring_cups --seed 0`：success_rate 1.0、eval_time 1，四路相机正常；`cam_third` 换到 4916 / 4913 位姿又各跑一次（也都 1.0），运行目录的相机配置已还原。
  - 对比图在场景目录 `preview/`（不入库）：`compare_photo.jpg`（几何叠线）、`compare_isaac.jpg`（照片 vs Isaac）、`robot_cameras_isaac.jpg`（四路相机）；已拷到 `5090-2:/root/RoboDojo/real2sim/scenes/real_scene5_shenzhen_lab/preview/` 给平台显示。
- **共享库**（用户同意后，2026-10-21 13:5x）：`Room/real_scene5`（8 个文件）和 `Eval_Layout/RoboDojo/g1d_shenzhen_lab`（574 个布局，0/1/2 三组）装进的是迁移前的旧资产库，**`5090-2:/data/robodojo-assets/Assets` 里没有**，要从上面隔离目录的验证产物补装（见「剩余事项」）。
  - **去掉了 `g1d_pour_beans_measuring_cups_*`（165 个）**：共享库没有 `measuring_cup`，其他配置也没有这个任务的布局。
  - 生成输入核对：同一次运行生成的 `g1d_lab` 和共享库的 `g1d_lab` 比，拾取、叠块逐字节相同；倒水、杯对杯只有豆子参数不同，因为**共享库里 `g1d`、`g1d_counter`、`g1d_lab` 的这两个任务布局还是旧的 4 mm 豆子参数**，当前代码是 5.6 mm。`g1d_shenzhen_lab` 按当前代码生成，所以和旧配置在这两个任务上不一致（见「剩余事项」）。
  - 平台：`5090-2` 的元数据镜像补了同样的 574 个布局 json（整体重刷镜像时会从共享库带上）。
- **部署**（14:00）：`5090-2` 从 `d5db8aa` 快进到 `2975b4a`（推临时分支再 `--ff-only`）；部署机 `HANDOFF.md` 里另一会话的绿幕桌记录照旧先存为 `.cache/handoff-local-20261021-135957.patch`，快进后重新应用；airpods `spec.json` 的平台时间戳改动保留。`real_scene5.usd` 和地毯贴图检出后只是 LFS 指针，从本机 rsync 真实文件后 `git add` 刷新 stat。重启服务后未登录 `/api/info` 401；部署机上直接调用 `scan_scenes()`：已建模、配置 `g1d_shenzhen_lab`、离线检查 22/22、拟合 5 视角（最大 8.1 px）、5 张渲染图。网页里点「同步项目数据」即可看到。
- **没做**：站位卷尺和叠图（要真机头部、腕部画面和关节角）；灯光没和头部相机比。

### 数采平台任务登记（2026-10-20，[PR #11](https://github.com/Zheng-Chong/RoboRSR/pull/11) 合并进 `main`，已部署）

- **用户要求**：任务采集也加新建 / 登记按钮，主要能选泛化等级，并选定所用物体范围、干扰物、可用场景，以及其他泛化。
- **页面**：「任务采集」新增「登记任务」四步向导，沿用场景和物体登记的向导组件：
  1. 任务定义：新建（ID `g1d_<后缀>`，不能和已有任务重名），或给已有 G1-D 任务登记泛化方案（物体角色从任务配置的 label 预填）；中英文名、语言指令（英文，原样发给策略）、成功判据（必填，用于写 `run_reward`）、用手、预计时长、合格回合目标、负责人；
  2. 物体角色：每个角色一个 label、每回合数量、候选物体（从物体资产里选，可多选）、可选 x/y 范围和朝向随机；干扰物：数量范围、指定物体、是否也从 Clutter 库取；
  3. 场景：每个场景设为训练、L4 保留或不用，未建模的场景也能选；至少一个训练场景；
  4. 泛化等级 L0–L4：勾选等级，范围和目标布局数从原 `LEVELS` 预填，可改；L3 选保留实例；选了 L4 必须有保留场景；其他泛化（预设：桌高、站位、初始姿态、指令改写、控制延迟）；脚本专家思路、其他要求、备注。
- **落盘**：服务端 `write_task_registration` 写 `real2sim/tasks/<任务>/registration.json`，场景附上加载它的 env 配置（取自 `build_asset.SCENES`），空字段省略（`compact` 现在也递归处理列表）。`scan_tasks` 把只有登记、还没实现的任务也列出来（`runnable: False`、`module: None`），流程里「任务配置」显示为待办，而不是不通过。
- **抽屉**：登记过的任务顶部是给 coding agent 的实现任务（新任务指向 CLAUDE.md 的任务约定和 `scripts/datagen/SKILL.md`，已有任务只补布局和采集设置），下面是登记方案摘要和可直接编辑的文字字段；分级评测集表格改为只列登记的等级，显示登记的范围和目标数。「删除登记」：未实现的任务删记录，已实现的只清掉登记，保留负责人、目标和采集记录。
- **验证**：`test_server.py`（登记 JSON、场景映射 env 配置、列表内空字段省略、未实现任务出现在扫描里、坏 id 400、删除）；`check.sh` 的 ruff、diff、任务清单；pytest 22 passed、1 skipped。浏览器里走完向导：缺指令、角色没有 label、角色没选物体、干扰物最少大于最多、没有训练场景、选了 L4 却没有保留场景都被拦住；提交后文件内容正确，列表显示「待实现」，抽屉文字字段写回文件，「已有任务」模式预填了量杯任务的 3 个角色。测试中发现最后一步的校验没在提交前跑，已修复。测试数据已删。
- **部署**：`5090-2` 从 `4e9567b` 快进到 `73b2c89`，部署机 `HANDOFF.md` 的另一会话记录照旧先存补丁、快进后重新应用；重启后本机和公网未登录 `/api/info` 401，页面带「登记任务」。线上登记的 `real2sim/tasks/*/registration.json` 要拷回入库。

### 数采平台场景登记（2026-10-20，[PR #10](https://github.com/Zheng-Chong/RoboRSR/pull/10) 合并进 `main`，已部署）

- **用户要求**：场景重建也按物体资产那样登记，每个场景不少于 5 张不同角度的照片，再加需要人工提供的信息（字段由 agent 设计）。
- **页面**：「场景重建」新增「登记场景」三步向导，和物体向导共用步骤条、校验和拖放组件（抽成 `wzFld`/`wzPane`/`wzNav`/`wzFoot`/`photoDrop`，`wizGo` 按当前打开的向导取步骤和校验函数）。
  1. 中英文名称，目录名 `real_scene<N>_<后缀>`（N 取已有最大号 + 1，后缀由英文名生成，可改，不能和已有场景重名），类型、地点、用途（孪生 / 训练泛化 / L4 保留），照片至少 5 张，不够不能下一步；
  2. 卷尺：桌面离地高、长、宽必填（cm），可选厚度、测量日期、支撑形式、桌面外观、桌沿形状；机器人站位（桌沿到根节点、横向偏移、偏角、升降柱读数）；ChArUco 标定板（勾上后格子实测边长必填，加板角到桌左沿、前沿的距离）；
  3. 地面、墙面、后沿到墙距离、要建模的固定物；光源、色温、照度、自然光；是否已拍机器人相机画面；计划任务（G1-D 自建任务多选）、建模要求、备注。
- **落盘**：服务端 `write_registration` 写 `real2sim/scenes/<场景>/registration.json`。卷尺值换成米，按 `scene_spec.json` 的 `measurements` 键和格式写（`table_top_z` 等，`tool: 卷尺`、`date`），空字段省略（0 保留）。照片上传 `/api/photo?kind=scene` 进 `reference/`，删除照片放行 `real2sim/scenes/`。场景 id 校验 `real_scene\d+_[a-z0-9_]+`。不新建 `scene_spec.json`：`acceptance.py` 要求场景已接入 `build_asset.py`，未建模的场景放进去会报错。
- **部署**（23:2x）：`5090-2` 从 `daf4ea0` 快进到 `4e9567b`（推临时分支再 `--ff-only`）。快进前删掉了和 `main` 内容一致的未跟踪 airpods 文件（`spec.json` 和 3 张照片，LFS oid 核对过），检出后照片只剩 LFS 指针，又从本机 rsync 回真实图片；部署机 `HANDOFF.md` 里另一会话（绿幕桌相机标定）未提交的一段先存为 `.cache/handoff-local-*.patch`，快进后重新应用。重启服务后扫描到 5 个场景（含未入库的 `g1d_green_table/`），本机和公网未登录 `/api/info` 都是 401，页面带「登记场景」。
- **显示**：「照片」阶段改为至少 5 张才算通过，旧场景里吧台（2 张）、实验室（4 张）、学校（2 张渲染）因此变成待办。「尺寸实测」阶段没有手填桌高时，用登记的桌高。已登记但还没建模的场景，抽屉顶部是给 coding agent 的建模任务文本（指向 `real2sim/scenes/SKILL.md`），下面是照片上传/删除、验收进度和可编辑的登记信息；建好后登记信息仍在抽屉里。`real2sim/scenes/SKILL.md` 加了「平台登记」一节。
- **验证**：`test_server.py`（新增：登记写出的 JSON 全量比对、坏 id 返回 400、场景照片上传/删除、扫描识别已登记、删除登记会删文件）；`check.sh` 里的 ruff 0.11.7、`git diff --check`、任务清单；pytest 22 passed、1 skipped（本机没有 pytest，用 `uv run --with pytest ...` 补）。本机起服务在浏览器里走完向导：3 张照片被拦、6 张通过、缺桌高被拦、勾标定板不填格子被拦；提交后 `registration.json` 和 6 张照片落盘，抽屉显示建模任务；抽屉里改墙面会写回文件。测试场景目录已删。没有在手机宽度下截图检查。

### 耳机充电盒 `airpods/00000` 实物孪生（2026-10-20，PR #7 合并进 `main`）

- **输入**：平台登记 `real2sim/objects/Rigid/airpods/00000/`（`spec.json` + 3 张照片，从 `5090-2:/root/RoboDojo` 同步）：PP 塑料、白色、38 g，`尺寸（高 × 宽 × 厚）：46.40 × 54.40 × 21.38 毫米`。
- **脚本** `scripts/objects/make_airpods.py`：平放的圆角盒，沿"高"放样，截面是"宽-厚"平面里的圆角矩形；俯视圆角 R15、边缘圆角 R8、盖深 16 mm 是照片估计。可见部件：壳 `model`、盖缝 `seam`（壳上 0.3 mm 深的槽加槽底灰带，不超出外形）、USB-C 口 `port`。碰撞体是一个 64 顶点凸包，比外观最多小 1.2 mm。不写摩擦（用默认）。`measurements_mm` 写 `高`/`宽`/`厚`，从壳的网格量。
- **踩坑**：第一版凸包是贪心选点，落地后歪 2.4°、下沉 0.7 mm（GPU 和 CPU 物理都一样，延长到 6 s 也一样，改成 60 顶点也一样）。原因是 PhysX 烹饪的平面容差 0.7 mm：离平底面不到 0.7 mm 的点被合并，底面就不平了。现在凸包的点要么正好在平面上，要么离开 ≥ 1.5 mm；已记进 SKILL「常见坑」。
- **验证**：
  - 本机：`scripts/check.sh` 通过（用 `uv run --with ruff==0.11.7 ...` 补依赖；22 passed，1 skipped）；pxr 打桩跑几何和 `self_check`。
  - `5090-2` 隔离目录 `/data/robodojo-runs/airpods-20261020-194222`（GPU 3）：生成到 `ovl/Rigid`，`check_object.py` OK：z=0.0107（期望 0.0107）、倾角 0.00°、速度 0。尺寸：高 46.4 / 宽 54.4 / 厚 21.4 mm，对实测误差 0 / 0 / 0.02 mm；质量 0.038 kg。
  - 渲染图 `preview/sim.png` 和 3 张实拍对比过：轮廓比例、盖缝位置、USB-C 口一致，仿真高光比实拍弱。已放到本机和部署机的同一路径。生成产物也拷到了本机 worktree 的 `Assets/`（不入库）。
- **共享库**（用户同意后，20:22）：新目录 `Object/RoboDojo/Rigid/airpods/00000`，没有覆盖任何东西，所以没做备份；从隔离目录 `ovl/Rigid/airpods/00000` 经本机中转拷入，三个文件的 sha256 和验证产物一致。
- **平台**：没有整体重刷镜像（会删掉手动放的纸杯纸盘元数据），只把这个 id 的 `metadata.json`、`description.json` 拷进镜像（和共享库一致，以后整体重刷也会保留），再重启服务重新扫描。数据库里显示：已入库、质量 0.038、`measurements` 高 46.4 / 宽 54.4 / 厚 21.4、有渲染图；「生成脚本」为空，因为部署机代码还没更新到含 `make_airpods.py` 的 `main`。
- **尺寸文本**：登记原文「尺寸（高 × 宽 × 厚）：46.40 × 54.40 × 21.38 毫米」旧版 `parseDims` 拆成乱码；PR #9 的新解析拆出 `高`/`宽`/`厚`（本机用 node 跑过），和 `measurements_mm` 对得上，登记不用改。

### 学校实验室场景和纸杯纸盘接入（2026-10-20，PR #2，已合并进 `main`）

- **来源**：同事在 sys01 上做的 MuJoCo 孪生 `g1d_food_sorting`，10-20 整包拷到了 `5090-2:/root/g1d_sim_assets_20261020`（`SHA256SUMS` 通过）。用户选的范围：场景，加上杯盘（用包里的拟合几何）；花材（40 簇）不接。
- **场景** `real2sim/scenes/real_scene4_school/`（Room `real_scene4`，env 配置 `g1d_school`）：
  - `build_scene.py` 读 `bundle/environment_manifest.json` 里的桌面四角（初始躯干坐标系），按 `reference.npz` 里记录的躯干偏航（12 号电机 0.067 rad）换到底盘坐标系，拟合成矩形：1.065 × 0.596 m，离地 0.763 m，四角残差 15 mm。绿幕取自源场景的 `backdrop`。桌腿、房间、灯光没观测到，是随手定的。只需要 pxr 和 numpy：`uv run --no-project --with usd-core --with numpy python real2sim/scenes/real_scene4_school/build_scene.py`。
  - 站位：桌沿在根节点前 0.19 m，桌面中心在机器人右侧 0.083 m，底盘左偏 1.63°（`env_cfg/robot/g1d_school.yml` 根节点 y=-0.79）。`build_asset.py` 里 `SCENES` 新增一行：Room 位置 (0.0828, -0.302)，物体平移 (0, -0.06)。
  - **和前三个场景不同**：升降柱保持 0，桌面相对躯干的高差按拟合值保留（0.082 m），不统一成办公室的 0.165 m。偏航来自"12 号电机就是 `torso_Joint`、方向与 URDF 相同"这个假设，没在真机上核对。细节见该目录的 `PLAN.md`。
- **物体** `scripts/objects/make_school_tableware.py`：复用 `make_plastic_cups.py` 的放样、杯壁分片碰撞体和 `save()`。
  - 纸杯：读 `bundle/cup_shapes.json` 的拟合轮廓（每只是两只纸杯套在一起）。
  - 纸盘：平底加锥形边，尺寸取自孪生的网格。
  - 质量取自孪生的密度假设，颜色是孪生的 sRGB 纯色换算成线性值（第一次没换算，黑杯渲染成灰色）。
- **验证**：
  - 本机：`scripts/check.sh` 通过（22 passed，1 skipped：worktree 里 XPolicyLab 是空的）；`test_scenes.py` 的期望值加上了 `g1d_school`。新文件已手动跑过 ruff（check.sh 只看已跟踪的文件）。
  - `5090-2` 隔离目录 `/data/robodojo-runs/school-scene-20261020-114517`：
    - `Assets/` 复制自 `office-beans-random-20261019` 的覆盖目录，再手动装上 `Room/real_scene4`。
    - `g1d_school` 的布局由运行目录里的 `school_layouts.py` 从该目录的 `g1d` 布局换算得到：Room 换掉，物体抬高 0.0828 m、平移 -0.06 m。**没有跑 `build_asset.py`**，因为 5090-2 没有 `g1d_v4`、`unitree_ros` 和完整资产库。
    - `check_object.py`：5 件物体全部 OK（倾角 0°，落地高度和半高相差 ≤ 0.1 mm）。
    - `scripted_demo.py --env_cfg_type g1d_school --task_name g1d_pour_beans_measuring_cups --seed 0`：成功，`_result.json` 的 success_rate 1.0、eval_time 1。头部画面和源孪生的渲染一致（深绿幕、木色桌板）。注意这个配置的步长是 1/300 s，颗粒任务正式评估仍然要用 `g1d_beans_collect`。
- **没做**：卷尺、叠图（真机画面在 sys01 上，这次 SSH 连不上）、灯光；杯盘没有任务和布局；没有装共享库。

### 采集、评估流水线规范化（2026-10-19，PR #1，2026-10-20 合并进 `main`）

- **起因**：用户要把"基于 Coding Agent 的 Real2Sim 采集 + 数据生成 + 仿真评估"做规范。在建议清单里，用户选了四项：数据契约、通用采集流水线、Agent 文档和 CI、场景标准化。
- **数据契约**（`scripts/datagen/CONTRACT.md`）：
  - 发现的问题：评估时给策略的夹爪 state 是上一步的指令值（`prev_control`），而旧数据集里存的是实测开度。
  - 对策：新增 env 配置项 `observation.robot.ee_state: measured`（`env/observation_manager/obs_manager.py`），G1-D 的四个配置都打开；不写这个键时行为不变，X5 等任务不受影响。
  - 录制直接写 XPolicyLab 标准轨迹 HDF5：state 取评估时的 obs，指令取评估时给策略的那句。LeRobot 只通过官方转换器生成。
  - 物理配置写进每个回合的 `additional_info/env_cfg` 和质检单。颗粒任务不要在 1/300 s 下评估。
  - 回放证据：
    - 两条录制回合在 `g1d_beans_collect` 下重放：state 偏差 ≤ 1e-4 rad；一条严格验收通过，另一条差 1 颗豆子；
    - 同样的动作放到 `g1d`（1/300 s）下：各洒出 57–59 颗，约 9%，连任务自身的判据都不通过。
- **评估入口**：
  - `src/eval_client/main.py` 新增三个参数：`--policy_module`（在进程内跑策略，不起服务器）、`--layout_ids`、`--allow_train_layouts`。
  - `env/seed_manager/seed_manager.py`：seed 目录里有 `split.json` 且标为 train 时，评估直接拒绝运行。
  - `EvalEnv` 新增 `end_on_success`（默认 True，任务判据达标即结束回合）和 `task_success`（任务判据是否触发过），取代采集时 monkey patch `get_reward` 的做法。
  - `scripted_demo.main()` 改用 `--policy_module`，不再往 `sys.modules` 里塞假的策略包。`cup_to_cup_actions` 新增参数 `pour_share`（不能叫 `pour`，会和函数内部的 `pour()` 重名；第一次 GPU 运行就是因此报的 TypeError）。旧采集器通过改模块常量 `C2C_POUR` 的方式仍然有效。
- **通用流水线** `scripts/datagen/`：
  - `collect.py`：录制器和 worker；
  - `run_batch.py`：分片（写 `split.json`）、多卡调度、续跑，记录源码和资产的 sha256；
  - `export.py`：调用官方转换器，再用 LeRobot 加载器逐回合核对长度、指令和首尾帧，生成数据集卡片；
  - `audit.py`：从原始数据独立复算验收，统计布局覆盖，写 `final_validation.json` 和 `sha256.json`；
  - `replay.py`：开环回放，检查数据环境和评估环境是否一致；
  - `upload.py`：上传 ModelScope；
  - `plugins/g1d_pour_beans_measuring_cups.py`：量杯任务插件，复用 `collect_office_beans` 里的计数和判据，验收版本 `all_beans_retained_v2`；
  - `registry.json`：已有的两批数据；
  - `SKILL.md`：操作手册，采集相关的常见坑已从本文件移到这里。
- **场景**：
  - `scripts/scenes/charuco.py`：打印标定板、检测、相机标定、求位姿；
  - `scripts/scenes/overlay.py`：用 URDF 正运动学加相机配置得到头部或腕部相机的位姿，把桌角和桌沿投影出来，和真实画面上的标注比较像素误差；
  - `scripts/scenes/acceptance.py`：按月度目标汇总三个场景的 `scene_spec.json`；
  - `real2sim/scenes/SKILL.md`：现场拍摄、卷尺测量和灯光记录规范。
- **Agent 文档和 CI**：
  - `CLAUDE.md` 新增本项目流水线的导航和规则（一个会话一个 worktree、提交前跑检查）；
  - 本文件瘦身；
  - README 加了数据生成入口、仓库结构和协作规则；
  - `scripts/check.sh` 和 `.github/workflows/checks.yml`：只对改动过的文件跑 ruff（旧文件还没格式化过），再跑单测和任务清单检查。为了让检查通过，顺手修了 `eval_env.py` 的一处 UP038 和 `main.py` 的一处格式问题。
- **验证**：
  - **本机**：`bash scripts/check.sh` 通过，共 23 个单测（采集 7、场景 5、量杯 6、平台 5）、ruff 0.11.7、任务清单检查和 `git diff --check`。
  - **CPU 单测上服务器**：在 `5090-2` 的仿真环境里跑了 17 个单测，用的是 OpenCV 4.11，包括 ChArUco 检测和"XPolicyLab 读取器读回我们写的 HDF5"。
  - **GPU 验证**：在隔离目录 `datagen-20261019-225252` 里跑（资产是 100 条那一轮的同一份，共享库没动）：
    - 脚本专家走新的 `--policy_module` 入口，`g1d` 下量杯 seed 0：`success_rate` 1.0，`eval_time` 1；
    - `run_batch.py` 用 2 张卡、分片 300 和 301 各 2 个布局：4 次尝试 4 次合格，每条 672 颗豆子全部进矮杯；夹持时夹爪 state（实测）是 0.75，action（指令）是 0；
    - 在 seed 300 上不加 `--allow_train_layouts` 跑评估，被拒绝；
    - 开环回放见上面"数据契约"一节和 CONTRACT（采集配置下偏差 ≤ 1e-4 rad，1/300 s 下洒出约 9%）。
  - **导出和审计**：在 `5090-2` 的隔离目录 `datagen-pipeline-20261019-223558` 里跑：官方转换器转了 2/2 个文件，每臂 8 维，AV1；`export.py` 逐回合核对通过（810 帧）；`audit.py` 输出 validated。
  - 原始数据已删除。本机留档在 `output/datagen_validation_20261019/`（报告、日志、质检单、数据集元数据、两帧图像）。
  - **合并 `a1b6db5` 之后复跑**：
    - 脚本专家跑量杯 seed 0：`success_rate` 1.0；
    - 采集探针（候选布局 7）：录制正常，但这一回合撒了 1 颗（671/672），没通过严格验收。同一个布局在 100 条那一轮里 392 帧就合格了；`a1b6db5` 改了倒豆循环的计数，会把杯角的豆子也倒完，这次倒了 452 帧。
  - **验证中发现并已修复**：`pour` 参数和函数内部的 `pour()` 重名；官方转换器在软链接目录下把根目录解析错了；只设 `CUDA_VISIBLE_DEVICES=1` 时渲染和计算不在同一张卡上，取相机数据失败（改成 `--device cuda:1 --device_id 1`）。
- **并行会话**：
  - 另一会话期间在 `main` 上的三个提交（`a1b6db5`、`353730a`、`4562fb4`，见下一节）都已合进来，每次都只有 `HANDOFF.md` 冲突，手工合并。
  - 物体建模 SKILL：`scripts/objects/SKILL.md`，以及 `CLAUDE.md`、`README.md`、平台 `index.html`、`export_office_beans.py` 的线程参数，仍在主工作区里，还没提交。

### 已完成：随机倒豆手 + 随机偏航 + 大范围位置的 100 条办公室数据（2026-10-19，`a1b6db5` 和 `353730a`，另一个会话）

- **用户要求**：左右手随机，两只杯子的偏航随机（用户说"很重要"），位置在能夹到的范围内尽量大；仍是办公室场景、100 条、严格全量判据、LeRobot 格式。
- **代码**：
  - `scripted_demo.flat_turn` 读取杯子的偏航，在 `C2C_FLAT_TURN` 窗口内转动手腕（相对原来的 45°，向外最多 35°、向内最多 15°），让指垫正好夹在一对平面上；超出窗口时取模 90° 的最近边界。56% 的朝向能正对，其余最多偏 20°。杯子在 45° 时转角为 0，旧行为不变。
  - `inside()` 对量杯改用 `measuring_cup_shape.in_cavity`。原来用的是杯口内切圆柱，角落里的豆子被算成已经倒出，于是只剩角落时杯子就回正，导致右手倒时有残留或撒豆。
  - `prepare_office_beans_layouts` 改为倒豆手一半对一半打乱，再加六维拉丁超立方采样：高杯 |x| 0.05–0.22 m，矮杯 |x| 0.05–0.18 m，y 从 −0.46 到 −0.28 m，偏航 0–90°。
- **探测**（运行目录 `5090-2:/data/robodojo-runs/office-beans-random-20261019` 的 `output/office_beans/exp{1,2,3}`，汇总脚本在本机 `.agent/remote-runs/office-beans-random/`）：
  - exp1（中心位置、极端偏航）：接杯手向内多转 25° 以上会把杯子碰倒，向外 44° 抓不起来；右手倒常剩下或撒出 1–5 颗。
  - exp2（修复后，全随机偏航）：18/24 合格，左手 11/12、右手 7/12，没有翻杯，也没有残留。
  - exp3（大范围，64 回合）：26/64 合格。失败主要是杯子离机器人太远（y > −0.29）或矮杯太靠外（|x| > 0.19）时掉杯，以及高杯太近（y < −0.46）够不到；落在选定范围内的是 16/23。
- **正式批次**：
  - 代码 `a1b6db5`，运行目录里关键文件的哈希和提交一致。
  - 400 个布局（左右手各 200），8 张卡，用旧采集器跑 collection_v3：176 个回合里 104 个合格（左 53、右 51），8 个进程退出码都是 0。
  - 协调脚本按手计数，左右各取最早的 50 条，硬链接到 `collection_v3/selected/` 后导出。
  - 数据集在 `5090-2:/data/robodojo-runs/office-beans-random-20261019/output/office_beans/collection_v3/lerobot`：517 MB，LeRobot v3.0，100 条，39827 帧，约 22.1 分钟。已登记进 `registry.json`（legacy 格式）。
- **验证**：
  - 导出器回读 100 条、39827 帧。
  - 原始审计逐条重算最后 30 帧：67200 颗全部转移，残留 0，散落 0；左右手各 50 条。
  - 覆盖：四个位置坐标按 20 档分箱全部覆盖，两只杯子的偏航（模 90°）18 档全部覆盖。
  - 最后 1 秒杯子的最大倾角 29.88°（阈值 30°）。
  - LeRobot 官方读取器解码了 600 张首尾帧，时间戳也通过。
- **ModelScope**（`4562fb4`）：经用户确认，上传为单独的私有数据集 `ChongZheng/g1d_office_pour_beans_random_hand_yaw_100`，和上一批分开。
  - 沿用服务器上的 `ms-venv` 和已保存的登录。服务器上的 `ms_upload.py`、`ms_verify.py` 改成由 `MS_DATA`、`MS_NAME`、`MS_MESSAGE` 三个环境变量指定目录和仓库，默认仍指向上一批；仓库里通用的上传脚本是 `scripts/datagen/upload.py`。
  - 核对结果：远端 17 个数据文件的路径、大小、SHA-256 全部一致；`private=True`，未登录访问返回 404；SDK 默认许可证 Apache-2.0；上传缓存已删除。

### 办公室量杯全量倒豆 100 条（2026-10-19，已发布）

- 实测量杯加 5.6 mm 红豆，600 Hz，118 次尝试 108 次合格，导出 100 条。共 40279 帧，3 路 640×480 视频。
- 独立复算：每条最后 30 帧里，672 颗豆子全部在目标杯内。官方加载器逐条解码了首尾帧。
- 已上传 ModelScope 私有仓库，逐文件核对 sha256 一致。
- 位置、格式和差异（legacy 格式：头部图像键是 `cam_head`，指令是自定义的一句话）见 `registry.json`。
- 用的令牌曾在聊天里出现过，已建议作废重建。

## 剩余事项与风险

- **X5 随机布局**：
  - 官方资产只有 1 张 HDR，"背景随机"实际只随机亮度；
  - 灯光亮度倍数 0.5–1.5 加 HDR 1400 加平行光时有几帧明显过曝（桌面发白），默认范围要调低或按画面亮度筛；
  - 写死位置的任务（`play_tic_tac_toe`、`cover_blocks`、`press_by_number`、`swap_blocks`、两个传送带任务等）位置随机不起作用，只能换实例、外观和干扰物；
  - 生成的布局还没有专家能解（仓库里没有 X5 脚本专家），现在只能当评测集用；
  - 页面没在浏览器里验证，平台没部署。
- **上海实验室 v3**：已装共享库、平台预览已换；部署机检出快进后，平台的场景抽屉和材质库才会显示 v3 的检查结果和墙面材质。
- **真机模型的仿真评测**：Motus eps222 在上海实验室 0/20，原因还没拆开（起始姿态、仿真画面与真机差异、杯位分布都可能）。要继续，先试双臂下垂的真机起始姿态（`remote_policy.py` 目前不支持设起始姿态），再在同一批布局上跑脚本专家做对照。服务端重启见「最近任务」第一节。

- **榜单基线**：10-22 16:35 停止，54 个任务只完成 6 个（见「最近任务」）；是否续跑等用户决定。续跑时 pi0.5 策略 + Isaac 同卡约 25 GB，一卡一组；OpenWAM 策略卡和仿真卡要分开。`/data` 剩约 192 GB（91%），另一会话的 `g1d_green_table/` 占 1.4 TB。官方榜单页面是 JS 渲染，没抓到公开分数，要和自测基线对照需手动查 robodojo-benchmark.com/LeaderBoard。dsw-4 根分区 100% 满（19 GB），训练数据和产出要放 `/mnt/data`。

- **G1-D 移动版**：
  - 共享资产库还没有 `g1d_mobile_dex1.urdf` 和 `Eval_Layout/RoboDojo/g1d_mobile/`，要在隔离目录跑完整的 `build_asset.py` 验证后，经用户同意再装；
  - 录制（`collect.py` 的 `state_keys`）、LeRobot 导出和 XPolicyLab 的 `_robot_info.json` 都只认 16 维，要采带身体自由度的数据，先定契约再改录制器和转换器；
  - 第一个用到底盘的任务是 `g1d_deliver_cup_to_lounge`（见「最近任务」第一节）；底盘撞桌、撞椅只靠领先量限幅，没有专门测；轮子只是外观，不转；
  - 客区尺度只来自「圆桌高 0.75 m」，吧台左木柱宽 1.0 m 是下限估计，椅子尺寸和 7 kg 都是估计；下次去现场按 `real_scene2_counter/PLAN.md`「需要实测的尺寸」第 5 项量；
  - 脚本专家的绕行只挑椅子空档，不做路径规划：路线上有别的障碍时会撞上；
  - 真机升降是速度接口，部署端要按官方 PD 示例把 `lift_joint_state` 换成 `HeightAdjust`；真机 `rt/hispeed_state` 高度的零点和仿真 0（最低）是否一致没核对；底盘加减速、腰和升降的动力学都没做辨识。

- **主工作区里未提交的改动**（10-20 晚核对）：`CLAUDE.md`、`HANDOFF.md`、`README.md`、`real2sim/scenes/SKILL.md`、`scripts/g1d/export_office_beans.py`，主要是「服务器切到 5090-2」那次的文档改动；主工作区的本地 `main` 还停在 `a694da5`，落后 `origin/main`。物体 SKILL 已随 PR #5 合并。
- **共享库缺 10-20 之后装的东西**：`/data/robodojo-assets/Assets` 是 10-20 的快照，之后装的内容没进来，缺 `Room/real_scene4`、`Room/real_scene5`，以及 `Eval_Layout/RoboDojo/` 下的 `g1d_school`、`g1d_shenzhen_lab`、`g1d_mobile`。要用这几个场景，先在隔离目录跑 `build_asset.py` 验证，经同意后补装。worktree `.claude/worktrees/object-wizard` 里还有一份没提交的同类文档改动，已被这次提交取代，不要再合。
- **采集配置下的严格判据有方差**：同一组动作回放，2 条里有 1 条最后差 1 颗豆子。评估策略时，严格成功率要在同一布局上重复多次才有意义。
- **物体运行时随机还没有任务打开**：要用就在 `build_asset.py` 的 `RANDOMIZE` 加任务和标签，在隔离目录重新生成布局、跑脚本专家，先问用户（会改那个任务的评测布局）。打开前确认任务和脚本专家只从缩放后的包围盒取尺寸；`cup_grasp` 的 `contact_circle`、量杯腔体计数都没按缩放修正。
- **XPolicyLab 注册**：`XPolicyLab/utils/robot/_robot_info.json` 里还没有 `g1d*`，用 XPolicyLab 的 `train.sh` 会拿不到动作维度。要给 XPolicyLab 提 PR（子模块不在本仓库里改）。
- **legacy 数据集**：和新契约在头部图像键、指令两处不同；用它训练的策略，评估前要先对齐。
- **新流水线还没跑过整批**：collection_v3 是用旧采集器采的，下一批数据应该改用 `scripts/datagen/run_batch.py`。
- **学校场景**：桌面高度和底盘偏航都依赖假设（零升降、12 号电机即躯干偏航），要用卷尺和叠图核对；从 sys01 拷真机头部和腕部画面（`outputs/g1d-real2sim`）做叠图。杯盘要在数采平台登记实测后，按 `scripts/objects/SKILL.md` 重建。
- **共享库的倒豆布局过期**：`g1d`、`g1d_counter`、`g1d_lab` 的倒水、杯对杯布局是旧豆子参数（4 mm，`particle_mass` 6e-5），和当前 `build_asset.py`（5.6 mm）不一致；`g1d_shenzhen_lab` 已是新参数。要统一，就在隔离目录重跑 `build_asset.py`，经同意后替换（先备份）。
- **深圳实验室**：已合并、部署；5090-2 的共享库里还没有；站位卷尺（桌沿到根节点、横向偏移）和三视角叠图后，`acceptance.py` 才能通过。
- **场景**：除深圳实验室外没有卷尺实测，都没有叠图；`real_scene1` 和 09-21 那次真机运行时的摆放不一样，那批画面不能用来叠图。下次去现场按 `real2sim/scenes/SKILL.md` 一次采齐。
- **真机对齐**：
  - 动力学没做系统辨识；末端位姿动作还不支持（需要 G1-D 的 cuRobo 配置，并且左右臂各用一个 planner）；
  - 真机开环回放的关卡没有工具；
  - 真机夹爪读数 0–5.4 和指尖行程是否成线性没有标定；
  - 真机腕部相机的流名左右互换（部署端 `SWAP_WRISTS=True`）。
- **可达性**：杯对杯 seed 1、壶倒杯 seed 0 都是够不到的老问题。G1 手臂在桌面高度的可达距离约 0.45 m，自有任务的物体应放在 y≈[-0.55, -0.25]。
- **数采平台**：
  - 线上登记的场景（`registration.json` 和照片）和任务（`real2sim/tasks/*/registration.json`）要像物体登记那样拷回入库；
  - 权限只有两级；HTTP 明文传密码；
  - `5090-2` 上的资产库是手动刷新的元数据镜像；学校纸杯、纸盘的 `metadata.json`/`description.json` 是 10-20 从隔离目录 `school-scene-20261020-114517` 的 `ovl/Rigid/` 手动放进镜像的（共享库里没有），平台因此显示为「已入库」；下次整体重刷镜像会被删掉，要么先装共享库，要么重刷后再放一次；
  - 看不到 `scripts/datagen` 的产出（`quality.json`、数据集登记表）；
  - 场景的 `preview/scene.glb` 和 `preview/render/sim.jpg` 不入库，是手动同步到部署机的，场景重建后要重跑 `export_glb.py` / `render_scenes.py` 再同步；3D 预览要浏览器能访问 `registry.npmmirror.com`（three.js），访问不了时抽屉里显示加载失败，卡片和其他内容不受影响；
  - 公开仓库的 git 历史里仍然有运维细节，建议把仓库改为私有。
- **零碎**：
  - 旧批次 `office-beans-100-20261019` 的 final-audit 进程还在等一个不会完成的导出，需要手动结束；
  - 服务器的 `/usr/local/bin/ffmpeg` 不带 libx264，新建环境要在仿真环境的 `bin` 里链接系统的 `ffmpeg`；
  - 指定了 `robot_config`/`scene_config` 的 `_task.yml` 会覆盖 g1d 的机器人或场景。

## 下一会话

0. RSI：先读 `rsi/AGENTS.md`。按第八节 P0 依次做：裁判 `rsi/referee/assets.py`（统计 T0、切分 T1 / T2、审计），`x5_layouts.py` 和平台随机方案只列出所在池子的资产；12 个 `_random` 任务基础版的 X5 脚本专家；新房间变体和新杂物。随机方案平台部署后在浏览器里走一遍；调低灯光 / HDR 默认范围避免过曝。
0. 榜单：基线已停（16:35，用户要求），先问用户是否续跑；续跑后收两份基线（`smoke_results/baseline_*_20261022.md`，核对每个任务 `eval_time >= 1`），按任务对比 OpenWAM / pi0.5 / 有无官方数据；看失败视频做归类；然后开始把 `collect.py` 和插件接口推广到 dual X5，按优先级给无官方数据的任务写专家。
0. 实现平台登记的 `g1d_pour_beans_any_cups`：在平台任务抽屉复制「实现任务」文本交给 coding agent（输入 `real2sim/tasks/g1d_pour_beans_any_cups/registration.json`）。
0. 深圳实验室（已合并、部署，共享库待补装）：去现场量站位、拍头部 / 腕部画面做叠图（`overlay.py --env-cfg g1d_shenzhen_lab`）。
1. 先看 `git log`、本文件和 `scripts/datagen/CONTRACT.md`。主工作区里还有物体建模 SKILL 会话没提交的改动（见"最近任务"里的"并行会话"）；拉取新的 `main` 前，先让那个会话提交，或者先 stash。
2. 下一批数据用新流水线采集。先跑单回合探针，确认 `joint_names`、夹爪 state 和图像都正常，再按 `scripts/datagen/SKILL.md` 批量采集、导出、审计、回放，最后登记。
3. 去现场按 `real2sim/scenes/SKILL.md` 采集（ChArUco 标定板照片、卷尺、灯光、机器人相机画面），在平台「登记场景」录入，回来跑 `overlay.py` 和 `acceptance.py`。
4. 给 XPolicyLab 提 PR，注册 `g1d*` 的动作维度；然后用一个 LeRobot 策略在 `--env-cfg g1d_beans_collect` 下走通"外部训练 → 本仓库评估"的整条链。
5. 学校场景：用户重建 sys01 连接后拷真机画面做叠图（`overlay.py --env-cfg g1d_school`）；要用学校场景做任务时，再写杯盘布局和任务。
6. G1-D 移动版：在隔离目录跑完整 `build_asset.py` 和 `body_probe.py`（两个版本），征得同意再装共享库（连同 `lounge_chair`、带客区的 `real_scene2`、`g1d_counter_mobile` 布局）；送杯任务再跑 seed 1、2；要采身体自由度数据时，先改 CONTRACT 和录制器（`base_velocity`、`lift_joint_state` 还不进 16 维录制）。

## 最近历史

- 2026-10-24：吧台场景加左侧客区（3 张照片拟合，墙纸凹龛、两张圆桌、可推动的扶手椅刚体）；新增移动版配置 `g1d_counter_mobile` 和第一个移动操作任务 `g1d_deliver_cup_to_lounge`（吧台拿杯送到客区圆桌），脚本专家 seed 0 前 4 个布局 4/4；修复 `is_all_gripper_open` 对 G1-D 夹爪方向判反。
- 2026-10-24：RSI 协议初版 `rsi/AGENTS.md` + `rsi/rsi.example.yaml`：官方评测资产保留、只许泛化不许押题、数据方和评测方对抗且只经裁判交换分档成绩；统计出房间和杂物几乎全是官方保留资产。
- 2026-10-23：上海实验室 `real_scene3_lab` 的墙面不再贴照片，改成纯色饰面加门、白板、电视；角落放了一台由 URDF 烘焙的静态 G1-D（`usd/g1d.usdc`）。离线检查通过；10-24 在 `5090-2` 跑 Isaac 渲染和三个脚本演示，全部通过，经用户同意装进共享库并更新平台预览。
- 2026-10-23：数采平台任务抽屉加「泛化随机方案」，新增 X5 布局生成器 `scripts/datagen/x5_layouts.py`（接手另一会话未提交的实现；相机扰动改为只转角度，平移会让部分环境出图全黑）；`5090-2` 端到端验证 10/10 正常。
- 2026-10-23：真机 Motus 倒豆模型（ModelScope `Vincia/WanVlmMask-G1DPourBeansGenEps222-AbsQpos16-30000`）在上海实验室仿真量杯倒豆 0/20（夹爪总是夹空）；新增 `g1d_lab_beans_collect`，收入 `scripts/g1d/remote_policy.py`。
- 2026-10-22：启动榜单冲分（20 天）：5090-2 建基线运行目录，拉齐官方 Assets、OpenWAM / pi0.5 官方 checkpoint 和两套推理环境，冒烟通过后启动两个模型 54 个任务的全量基线评测；确认 20 个榜单任务没有官方训练数据。
- 2026-10-22：深圳实验室倒豆 500 条（矮量杯或马克杯、300–600 豆、2–5 干扰物、严格整回合验收）采完、审计、上传 ModelScope 私有仓库 `ChongZheng/g1d_shenzhen_pour_beans_cup_or_mug_500`。
- 2026-10-22：实验室场景 `real_scene3_lab` 的工位桌、网椅、活动柜、文件柜、边桌、圆桌建成立体模型（`/Background/Furniture`），从墙面照片里抠掉；隔离目录里三个脚本演示通过后，经用户同意换进共享库（见「最近任务」）。
- 2026-10-21：物体的颜色、尺寸泛化改为仿真里按布局随机，不建新资产；形状、材质等仍建变体资产，平台上收进本体卡片（用户：泛化放到一个卡片下；能不能在仿真里随机改尺寸和颜色而不是新建资产；用户选定两个都做、随机值写进布局）。
  - 仿真：`RigidObject` 读布局条目的 `visual.color`（线性 RGB）绑一个 PreviewSurface，覆盖资产自带材质；有 `visual.color_parts` 时只换这些网格（保留标签、透明件）。`scale` 本来就从布局读、每次复位重设；新改的是 `LayoutManager.get_instance_bbox_vertices` 对刚体按 `scale` 缩放 metadata 包围盒（判据和脚本专家都用它，原来恒为未缩放值），`RigidObject.get_bbox` 同样处理。现有布局里刚体全是 scale 1（在 5090-2 元数据镜像里全量查过，只有流体豆子不是 1，流体不受影响），所以现有任务行为不变。
  - 布局：`build_asset.py` 新增 `RANDOMIZE`（按任务、标签打开 `{"scale": (lo, hi), "color": True}`）和 `randomize_objects`：按 `任务/seed 目录/序号` 定种子，等比缩放并保持立在桌面上，质量按体积换算，颜色在 HSV 里取（饱和度 0.45–0.9、明度 0.35–0.9）再转线性。**现有任务都没开**：开了会改它们的评测布局；倒豆任务按量杯固定腔体数豆子，不能缩放；`cup_grasp` 读的 metadata `contact_circle` 也没按缩放修正。这些值随 `additional_info/layout` 进每个回合（CONTRACT 已写）。
  - 平台：物体扫描新增 `variant_of`（`metadata.json`，或装库前的 `preview/check.json`，`check_object.py` 会透传）；卡片视图里变体不单列，本体卡片带「+N 变体」角标，抽屉列出变体（点开是变体自己的抽屉，顶部注明本体）；列表视图仍列全部并注明本体。泛化设置加「运行时换色」「运行时缩放 ±%」（`gen.rt_color`、`gen.rt_scale`，写进 `spec.json` 的 `generalization`），卡片显示「随机 颜色 · 尺寸 ±15%」；登记向导第 3 步分成「运行时随机」和「另建变体资产」两组，快捷要求改成材质、把手、不等比这类必须建资产的；建模任务文本相应分两段（运行时随机怎么在 `RANDOMIZE` 打开，变体要写 `variant_of`）。`scripts/objects/SKILL.md`、CONTRACT 同步。
  - 验证：本机 `check.sh` 32 passed 1 skipped（新增 `scripts/g1d/test_randomize.py` 进 `check.sh`：缩放后贴桌、质量、换色可复现、`color_parts`、默认关闭；`test_server.py` 加 `variant_of` 两种来源和 `rt_*` 写进 spec）。`5090-2` 隔离目录 `/data/robodojo-runs/obj-random-20261021-225540`（GPU 5，资产软链接自 `shenzhen-lab-20261021-125246`）：拾取布局把目标换成量杯，分别为 1.0 原色 / 0.85 + 随机色 / 1.05 + 随机色，`scripted_demo` 3 回合画面里颜色、大小都对，杯子全程立在桌上；3 回合都没抓起来，原尺寸基线也一样（拾取脚本抓不了这只量杯，与缩放无关）。在仿真环境里给 isaacsim 打桩直接调用 `get_instance_bbox_vertices`：刚体 ×1.2、流体不变。回归：原始量杯倒豆布局 `g1d_shenzhen_lab` seed 0 `success_rate` 1.0。本机浏览器：造一个 `variant_of` 指向 `measuring_cup/00001` 的测试变体（已删），卡片归组、角标、两个抽屉、建模任务文本、向导第 3 步都正常。
  - 顺带修了上一条提交里 `server.py` 第 4 行超长（检查后才改的文档字符串，已进 `main`）和 `layout_manager.py` 一处老的 UP038。
  - `4f8cccf` 快进推 `main` 并部署：`5090-2` 从 `e4552a2` 快进（部署机 `HANDOFF.md` 的另一会话改动先存 `.cache/handoff-local-20261021-235903.patch` 再放回），重启后扫描正常，未登录 `/api/info` 401。线上还没有登记过 `variant_of` 变体或运行时随机，所以页面暂时看不出变化。
- 2026-10-21：场景卡片改用 Isaac RTX 渲染图做封面，场景抽屉顶部加可拖拽旋转的 3D 预览（用户：卡片放渲染的场景图，网页端 3D 预览）。新脚本 `scripts/scenes/export_glb.py`（只要 usd-core + numpy，本机可跑）把 `usd/scene.usda` 的可见网格（跳过 guide 碰撞体）连同 UsdPreviewSurface 材质和贴图转成 `preview/scene.glb`（Y 轴向上，节点名是 USD 路径）；`scripts/scenes/render_scenes.py`（GPU）只载场景、用场景自带灯光，从机器人一侧斜上方对准 `/World/Background`、`/Lights` 以外的部分（桌子和桌上物体），相机限制在地面范围内、天花板以下，写 `preview/render/sim.jpg`。平台扫描 `renders`、`model`，`/file` 放行 `.glb`；卡片封面优先渲染图，有模型的卡片带「3D」标记；抽屉用 three.js 0.169（浏览器按需从 `registry.npmmirror.com` 加载，importmap，不进仓库）+ OrbitControls：左键旋转、右键平移、滚轮缩放，不能转到地板以下，「隐藏房间」（墙、天花板、背景家具，地面保留）、「重置视角」、「全屏」；墙和天花板单面朝内，从房间外看自动剖开。抽屉每次重绘时把活的查看器节点放回原位，关抽屉或切走时释放 WebGL。`5090-2` 隔离目录 `/data/robodojo-runs/scene-render-20261021-222151`（GPU 5，只拷了场景目录和脚本）渲出 5 个场景，逐张看过；GLB 0.03–2.7 MB。验证：`check.sh`（29 passed 1 skipped，新增 `GlbTests` 和场景 `renders`/`model` 扫描、`.glb` 下载）、新脚本 ruff；本机浏览器 5 个场景都加载、贴图和透明材质正常、模拟拖拽能转、隐藏房间、切换场景后只剩一个画布、关抽屉后画布释放。`real2sim/scenes/SKILL.md`「接入和登记」和平台建模任务文本加了这两步。`e4552a2` 直接快进推 `main` 并部署：`5090-2` 从 `01e50fe` 快进（推临时分支再 `--ff-only`，部署机 `HANDOFF.md` 的另一会话改动先存 `.cache/handoff-local-20261021-222905.patch` 再放回），5 个场景的 `scene.glb` 和 `render/sim.jpg` 拷到部署机同一路径，重启后扫描到模型和渲染图，未登录 `/api/info` 401。没有登录线上页面看（没有账号密码）。
- 2026-10-21：登记向导（物体、场景、任务、本体、评测）误关不再丢内容（用户：点到左边区域填的都没了）。抽屉关闭或切走时（点遮罩、Esc、关闭按钮、点别的行、换页面），如果表单相对刚打开时有改动、选了照片或物体 / 场景 / 任务，就把抽屉节点和向导状态（当前步骤、照片、各选择框的列表）暂存在页面内存里，并提示已暂存；再点同一个登记按钮原样恢复，顶部提示「已恢复上次没提交的内容」和「清空重填」。提交成功后清掉暂存。只存在当前页面，刷新浏览器就没了。验证：ruff；本机浏览器：物体向导填两步后点遮罩、重开恢复（步骤、名称、质量、类别），换页面后仍能恢复，清空重填得到空表；没改动的表单关闭不暂存；任务向导选了物体后按 Esc、重开恢复，提交后暂存清空。
- 2026-10-21：数采平台新增「模型评测」选项卡（用户：提交评测任务，给定模型、部署代码和要测的任务，负责 Real2Sim 的同学交给 coding agent 部署和评测）。提交向导三步：① 模型名称、版本、框架、checkpoint 位置、部署代码（必填）、训练数据，接口（本体、动作空间、图像输入、指令语言）、推理环境、显存、动作块；提示仓库公开、不要填凭据；② 任务（带预览的搜索框，至少一个）、可选限定场景、泛化等级（默认 L0、L1）、每个任务 × env 配置的布局数（默认 50）和重复次数（默认 3）、是否加脚本专家对照；③ 截止、优先级、评测负责人（可空，等认领）。编号 = 模型名 slug + 日期，同时是 eval_result 里的策略名。写 `real2sim/evals/<编号>/request.json`；提交人由服务端按登录账号写，后续更新不能改，新申请状态为「待受理」。抽屉：部署与评测任务文本（隔离目录、凭据不落盘、XPolicyLab 服务或 `--policy_module` 适配器、按 CONTRACT 对齐、冒烟后全量、颗粒任务用 `g1d_beans_collect`、同步 `eval_result/`、写 `result.json` 的格式、更新状态），受理（状态、评测负责人、评测记录），结果（`result.json` 的总成功率、按任务 × env 配置 × 等级的成功率 / 区间 / 专家对照、问题；以及 eval_result 里同名策略的回合，可跳质检），申请内容。datagen SKILL 补了这条流程。验证：`test_server.py`（申请写入、提交人不可伪造、状态和负责人更新、`result.json` 扫描、坏 id、删除）、ruff；本机浏览器：成员账号提交（校验、编号、任务搜索和预览）、模拟 `result.json` 后结果表和列表成功率、改状态为已完成。
- 2026-10-21：本体卡片加仿真渲染图。新脚本 `scripts/embodiments/render_embodiments.py`：按 `env/robot_manager/robot_config/<robot_name>.py` 把每个本体单独放在地面上（最低网格点落地，固定底座所以和臂姿无关），初始化并保持默认关节角后，按连杆实际位置加边距取景（`set_world_pose`，初始化前的 USD 包围盒和稳定后的姿态对不上，Franka 第一版被裁掉），写 `real2sim/embodiments/<id>/preview/sim.png`（不入库）。平台扫描 `preview/`：卡片封面优先用渲染图，抽屉分「仿真渲染」「实拍照片」。`5090-2` 隔离目录 `/data/robodojo-runs/embodiment-render-20261021-144517`（GPU 4；G1-D 资产复制自 `g1d-mobile-20261021`，X5 / Franka 从共享库拷贝，共享库未改；要设 `OMNI_KIT_ACCEPT_EULA=YES`）渲出 g1d、g1d_mobile、x5、franka 四张，拷到部署机同一路径。两版 G1-D 外观相同（移动版只多底盘自由度），靠「变体」区分。验证：`test_server.py`（renders 扫描）、ruff；四张图逐张看过。
- 2026-10-21：经用户同意，本会话把 PR #17（G1-D 桌面版 / 移动版，另一会话）合并进 `main`（`33c7157`）：在 `1d110fb` 上合并当时的 `main`，冲突在 `HANDOFF.md`、`README.md`、`_robot_info.json`、`CONTRACT.md`，都是两边各加内容，全部保留（CONTRACT 夹爪那条改为七个 G1-D 配置，HANDOFF 里桌面版配置数改为六个）；`check.sh` 27 passed 1 skipped（含 `test_g1d_body.py`）。直接推 `main`，PR #17 自动标为已合并。部署到 `5090-2`：平台服务用系统 Python 导入新 `build_asset.py` 正常，本体页出现 `g1d_mobile`（共 4 个本体）；部署机上和 `main` 一致的 `real2sim/embodiments/g1d/registration.json` 先挪开、快进后核对一致。另一会话的 worktree `../RoboDojo-g1d-mobile` 还停在 `1d110fb`。
- 2026-10-21：G1-D 分为桌面版 `g1d`（腰、升降可选动作键）和移动版 `g1d_mobile`（加差速底盘 `base_velocity`，虚拟平面关节），接口按官方 unitree_sdk2；`5090-2` 上两版 `body_probe.py` 和量杯回归通过（分支 `chongzheng/robot-g1d-tabletop-mobile`）。
- 2026-10-21：在线上平台代填 G1-D（`g1d`）的本体信息，内容全部取自本文件和配置：Unitree G1-D 桌面版、双臂 7 + 夹爪 1、Dex1-1 带相机夹爪、头部左眼 + 两腕部相机、30 Hz、`real2sim/robot/g1d_v4`；标定只有相机标为已完成（fx≈279.5 / 314，误差 0.4–3.6 px），关节零位无记录，夹爪和动力学未做，记录里写明原因和标定工具包未接入。署名「种政（agent 代填）」，负责人 chongzheng；`real2sim/embodiments/g1d/registration.json` 拷回入库。从这次起，按用户要求不再开 PR：仍在 worktree 短分支上改、本机跑检查、变基到最新 `origin/main` 后快进推送（不强推）。
- 2026-10-21：数采平台新增「本体管理」选项卡（用户：G1-D 至少有桌面版和移动版两个本体了）。本体 = `env/robot_manager/robot_config/` 里的每个 robot_name（现在 g1d、x5、franka；PR #17 合并后 g1d_mobile 自动出现），加平台登记的本体；同一本体在各场景的站位配置（`env_cfg/robot/*.yml`）算一个，列出站位、臂 / 末端维度（`_robot_info.json`）、根节点位置、加载它的 env 配置和场景、有布局的任务和采集回合。抽屉可传照片，填名称、型号、变体、负责人（账号）、自由度、末端执行器、相机、控制频率、URDF 来源、真机、备注，以及相机 / 关节零位 / 夹爪 / 动力学四项标定状态和记录；流程 = 仿真接入、站位配置加四项标定。录入写 `real2sim/embodiments/<id>/registration.json`。「登记本体」两步向导给还没接入仿真的本体用，生成接入任务文本（URDF 转 USD、robot_config / robot_class / env_cfg/robot / `_robot_info.json`、对齐 CONTRACT、5090-2 关节探针）。顺带修复：抽屉里连续改两个字段时，第二次保存基于旧数据覆盖了第一次（`onFieldChange` 先在本地记录生效再保存）。验证：`test_server.py`（新增 `test_embodiments`、本体登记 / 照片 / 删除）、ruff；本机浏览器：三张本体卡片、G1-D 抽屉的 5 个站位、连续改三个字段都保存、登记新本体生成接入任务。PR #23 合并（`31ef96a`）并部署。
- 2026-10-21：按平台登记新建深圳实验室场景 `real_scene5_shenzhen_lab`（env 配置 `g1d_shenzhen_lab`），卷尺定尺度，5090-2 上 `build_asset.py` 和脚本演示通过，经用户同意装库（不含量杯任务布局；装进的是迁移前的旧库，5090-2 共享库待补）；修 `build_asset.py` 相对 `--assets` 时的软链接问题。PR #22 合并（`2975b4a`）并部署到 `5090-2`。
- 2026-10-21：登记任务的评测保留物体改为跟着角色走（用户：每个角色都要有候选和保留，干扰物也要有）：第 2 步每个角色分「训练候选」「评测保留」两个选择框，干扰物分「训练用」「评测保留」，去掉单独的「评测保留实例」栏；已是训练候选的物体不能再选为保留，反之亦然（保留的必须是训练里没见过的）。L3 只在选了保留物体时开启（多候选、干扰物只算训练多样性）。「目标布局数」改名「评测布局数」并注明只用于评测、训练数据量看合格回合目标。`registration.json`：角色和 `distractors` 各加 `holdout`，去掉 `holdout_objects`。顺带修复 PR #15 把 PR #14 的两处改回旧版的问题：实现任务文本里的杂物类别、抽屉里的「含杂物库：类别」（登记文件的 `clutter_categories` 一直正确）。验证：`test_server.py`、ruff；本机浏览器：两栏选择、双向互斥提示、L3 依据、评测布局数、实现任务文本和抽屉摘要。PR #21 合并（`d5db8aa`）并部署。
- 2026-10-21：PR #19 合并（`2944698`）并部署；部署前删掉了部署机上和 `main` 内容一致的未跟踪 `real2sim/tasks/g1d_pour_beans_any_cups/registration.json`（sha256 核对过）。线上第一个场景登记 `real_scene5_shenzhen_lab`（深圳实验室，B5-2L，种政 10-21 12:08 登记：6 张照片、卷尺桌面 75 × 120 × 60 cm、厚 2 cm、四条腿白色圆角桌、储物柜 75×120×45 cm 金属、顶部条形面光源）拷回入库，照片走 LFS。还没建模，没量机器人站位，也没有标定板；部署 PR #21 时已核对一致、挪开后快进并放回真实照片，部署机上现在是跟踪文件。
- 2026-10-21：登记任务的泛化等级不再手动勾选，按前面的选择自动开启（用户：干扰物、换物体、选场景本身就是泛化，最后一步重复）：L0 始终开；L1 = 第 4 步勾了位置或朝向；L2 = 第 3 步新加的「外观随机」（灯光、纹理、相机扰动）勾了任一项；L3 = 某角色候选物体多于 1 个、有干扰物或选了保留实例（保留实例挪到第 2 步）；L4 = 选了保留场景。第 4 步只显示各级开没开和依据，开启的级可改范围和目标布局数。`registration.json` 新增 `appearance`。另外三个登记向导对管理员不再强制「填完才能下一步」：跳过的项在底栏列出；提交前仍检查 ID（物体类别和编号、场景目录名、任务 ID），对管理员也生效；普通成员不变。验证：`test_server.py`、ruff；本机浏览器：管理员空表跳到最后一步、空后缀提交被拦、等级随选择开关、取消位置和朝向后 L1 关闭、L4 目标改 80 写进登记；场景向导管理员可跳；普通成员三个向导都被拦在第 1 步。
- 2026-10-21：按用户口述在线上平台代登记任务 `g1d_pour_beans_any_cups`（任意杯子倒红豆）：源杯 / 目标杯各从 8 个杯子资产里选（measuring_cup 0/1、plastic_cup 0、plastic_mug 0、paper_cup 0/1、water_cup 0、cup 7；不能同一实例），红豆 100–300 颗随机，倒完两只杯子直立放回桌面、两臂回初始姿态且夹爪张开。agent 补的部分：成功判据阈值（目标杯 ≥ 95%、杯子倾角 < 10°、关节误差 < 0.05 rad）、35 s、训练场景为 4 个已建模场景、L0 + L1（位置和朝向随机）、中英文各 5 条指令。直接在 `5090-2` 用 `server.Store` + `write_task_registration` 写入（署名「种政（agent 代登记）」，负责人 chongzheng），`registration.json` 拷回入库。还没实现；两只量杯和 plastic_mug 不在共享库，实现时要先补。部署机上这个文件是未跟踪的，下次快进部署前先删掉它（内容和入库的一致）。
- 2026-10-21：登记任务的位置随机移到第 4 步 L1（用户：位置泛化应属于泛化等级），第 2 步只选物体；L1 下勾「位置 / 朝向」，训练布局和 L1 评测集都按它们随机，范围仍由 coding agent 定，都不勾即固定摆放。`registration.json` 的随机字段从每个角色的 `randomize` 移到任务级 `randomize`。第 3 步场景不再每个场景一个下拉框（场景多了不好选），改为「训练场景」「L4 保留场景」两个搜索选择框：和物体选择共用组件（`PICK` 按键名区分物体和场景），带缩略图和悬停预览（桌面尺寸、env 配置、照片数），同一场景只能在一边，选进另一边自动移走；「加入全部已建模场景」一键加入。验证：`test_server.py`、ruff；本机浏览器：加入全部已建模场景、搜索、悬停预览、回车把实验室移到 L4 保留、L1 取消朝向后提交，登记和实现任务文本正确。PR #16 合并（`f717246`）并部署。
- 2026-10-21：登记任务的物体角色不再手填 x/y 范围和朝向角度（用户：不要人为定这么细），改为「每回合随机」两个勾选项：位置、朝向（默认都勾）。`registration.json` 的角色字段由 `xlim/ylim/yaw_deg` 换成 `randomize: ["position", "yaw"]`；实现任务文本让 coding agent 按可达范围（桌面物体 y≈[-0.55, -0.25]）、物体不重叠且留抓取空间、按抓取方式定朝向来设范围，L0 孪生仍固定摆放，范围写进任务配置和交付说明。线上没有登记过任务，不用迁移。验证：`test_server.py`、ruff；本机浏览器里勾选、取消、提交，看了实现任务文本和抽屉摘要。PR #15 合并（`e1b347c`）并部署。
- 2026-10-21：登记任务里的物体选择改为带预览的搜索列表（用户反馈：选的时候看不出是哪个）：输入名称、类别或 id 过滤，每行有缩略图，鼠标悬停（或方向键）在旁边浮出大图卡片（名称、id、尺寸、质量、入库状态；图取渲染图，没有就取实拍），回车或点击加入；已选标签带缩略图，悬停也能预览。干扰物的杂物库说明清楚了：它是共享资产库 `Object/RoboDojo/Clutter/`（95 类 193 个，`clutter.yml` 索引），任务配置的 `Clutter` 段引用它；勾选后列出各类别和数量，可只选部分类别（不选即全库），写进 `distractors.clutter_categories`。新接口 `GET /api/clutter` 从资产库镜像统计类别。杂物没有渲染图，只显示类别名。验证：`test_server.py`（`/api/clutter`）、ruff；本机用部署机的元数据镜像和主工作区的渲染图在浏览器里试了搜索、悬停预览、方向键 + 回车、标签预览、杂物类别写入登记。PR #14 合并（`99cf0b6`）并部署到 `5090-2`，线上镜像统计出杂物库 95 类 193 个。
- 2026-10-21：任务登记不再让用户写语言指令：第 1 步改为中文「任务定义」（必填）、指令转写数量 N（1–50，默认 5）和指令语言（中文 / 英文，可多选，每种语言 N 条）。`registration.json` 的 `instruction` 换成 `definition` 和 `instructions: {count, languages}`。实现任务文本要求 coding agent 从定义总结指令并转写（不加定义外的信息，第一条为规范说法），写进 `real2sim/tasks/<任务>/instructions.json`，由 `gen_instruction` 返回（`desc_manager` 每回合随机选一句），用户审过再批量采集。平台扫描该文件：任务流程新增「语言指令」阶段（条数够了才通过），抽屉显示已写的转写和条数。去掉「其他泛化」里的「语言指令改写」预设。线上还没有登记过任务，不用迁移。验证：`test_server.py`（新字段、扫描 instructions.json）、ruff；本机浏览器里校验（缺定义、N=0、不选语言都被拦）、提交、放一份 2 条的 instructions.json 后阶段显示不足、抽屉列出转写。PR #13 合并（`518ec9c`）并部署到 `5090-2`。
- 2026-10-21：任务负责人改为从平台账号里选（登记向导和任务抽屉都是下拉框，默认当前账号；存用户名、显示显示名）。新增所有登录用户可读的 `GET /api/members`（只有用户名和显示名，`/api/users` 仍只给管理员）；任务登记 JSON 的 `owner` 改为 `{username, name}`，取自任务记录顶层的 `owner`，不再放在 `reg` 里。线上还没有人填过负责人，不需要迁移。验证：`test_server.py`（成员接口对普通成员 200 且不含权限字段、登记 JSON 的负责人）、ruff；本机用非管理员账号在浏览器里看了两处下拉并保存。PR #12 合并（`fc0a742`）并部署到 `5090-2`，重启后未登录 `/api/members` 401、公网页面 200。
- 2026-10-20：数采平台新增「登记任务」四步向导（新建或给已有任务登记泛化方案：物体角色和候选物体、干扰物、训练 / L4 保留场景、L0–L4 等级和范围、其他泛化），写 `real2sim/tasks/<任务>/registration.json`（PR #11，已部署到 `5090-2`）。
- 2026-10-20：数采平台新增「登记场景」三步向导（≥ 5 张照片 + 卷尺实测 + 站位、标定板、环境、灯光等），写 `real2sim/scenes/<场景>/registration.json`，抽屉生成建模任务文本（PR #10，已部署到 `5090-2`）。
- 2026-10-20：按平台登记新建耳机充电盒 `airpods/00000`（`make_airpods.py`，PR #7），实测尺寸和质量，`5090-2` 落地检查通过；经用户同意装进共享库，平台镜像已补上。
- 2026-10-20：物体流程在建好之后能自动推进了（用户反馈 airpods 建好后流程不动、物理检验不应全靠人工、「训练 / 保留划分」没用）。原因：平台只看资产库镜像，agent 在隔离目录建好、未装共享库时平台看不到；`check_object.py` 只打印结果；尺寸写成「（高 × 宽 × 厚）：a × b × c」时解析成乱码，和 `measurements_mm` 对不上。改动：`check_object.py --image-dir` 在 `preview/` 里和 `sim.png` 一起写 `check.json`（落地结果、质量、尺寸、`measurements_mm`、资产路径），agent 可补 `checks`/`notes`（碰撞、抓取、盛液、渲染）；平台扫描它，资产库没有时用它的质量和尺寸做核对，「建模」与「入共享库」拆开，物理检验取 `check.json`、人工结论优先；尺寸解析支持括号名 + × 分隔值；删掉物体的训练 / 保留划分（没有代码读它）。建模任务文本和 `scripts/objects/SKILL.md` 同步。验证：`check.sh`、`test_server.py`（新增 check.json 回退）、解析 4 种写法、本机浏览器用 airpods 的真实 metadata 造 `check.json` 看流程 3/5 和物理检验显示；`check_object.py` 只做了编译检查，没在 GPU 上跑。airpods 要在合并部署后重跑一次 `check_object.py --image-dir` 并同步 `preview/`，流程才会在线上推进。
- 2026-10-20：数采平台「登记物体」改成三步向导（分支 `chongzheng/platform-object-wizard`）：① 中英文名称、类型（类别由英文名自动生成，可改）、编号，多角度照片可拖放、可不传；② 实物测量（材质、颜色、质量、静摩擦、尺寸、外观、建模要求，可留空，但没照片时必须填尺寸）；③ 泛化：变体数量、允许变化的属性（颜色/材质/尺寸/形状）、泛化要求（可只写数量和要求，让 coding agent 设计参数）。`spec.json` 新增 `name_en`、`generalization`；物体抽屉可改英文名和泛化，「建模任务」文本按泛化设置要求在同类别后续空号建变体（不实测、标为泛化变体）。界面按 UI UX Pro Max skill（装在 `~/.claude/skills/ui-ux-pro-max`）的表单规范做了第二版：圆点连线步骤条、上标签两列网格、带单位输入、资产 ID 标签、大拖放区和方形缩略图、属性可选标签、数量步进器、泛化要求快捷示例、吸底操作栏；校验错误标在对应字段（`aria-invalid` 并聚焦），换步骤时聚焦首个输入，提交中按钮显示进度。验证：`test_server.py`（spec 新字段）、ruff；本机起服务在浏览器里走完三步（含 1280 宽和手机宽截图核对；校验、英文名转类别、拖放过滤非图片、提交、抽屉里改泛化写回 spec.json）。PR #5 合并（`8da8d30`），19:13 部署到 `5090-2`：推到临时分支再快进（线上未入库的登记改动不受影响），重启服务后页面和未登录 401 正常，隧道在。线上登记的 `measuring_cup/00001`（删 `IMG_4871.jpg`、新增 3 张照片和 `spec.json`，其中 `tempImageK59SQD.jpg` 与 `tempImageKLstnv.jpg` 内容相同）和 `plastic_mug/00000/spec.json` 拷回入库（分支 `chongzheng/platform-registrations-1020`）；合并后部署机要先删掉这些未跟踪的同名文件再快进。部署机上还有不明来源的未入库目录 `real2sim/scenes/g1d_green_table/`，没动。
- 2026-10-20：新增物体建模 skill `scripts/objects/SKILL.md`（按平台登记建实物孪生刚体资产：各层放什么、生成脚本和 `metadata.json`/`measurements_mm` 约定、`5090-2` 隔离目录验证、登记与提交），`CLAUDE.md`、README 和平台「建模任务」文本指向它。用只读子 agent 按它规划重建 `plastic_mug/00000` 做了两轮演练，按发现的问题修正（现有 `metadata()` 还不写 `measurements_mm`、外径要排除把手、一个脚本生成整族物体、老脚本参数是类别目录等）；里面的服务器说明并入了另一会话「服务器切到 5090-2」对这份文件的改动。没有在 GPU 上按它实际建过物体。
- 2026-10-20：数采平台图片和视频可以被浏览器缓存了（用户反馈每次打开都要重新加载，很慢）。原因：`/file` 不带 `Cache-Control`、`ETag`、`Last-Modified`，浏览器每次都整张重下；经隧道实测约 400 KB/s，物体页 63 张渲染图约 9.8 MB。现在 `/file` 带 `ETag`（mtime + 大小）和 `Last-Modified`，`Cache-Control: private, max-age=60, stale-while-revalidate=604800`，`If-None-Match` 命中回 304；页面本身 `no-cache`，部署后立即生效。服务端改为 HTTP/1.1 长连接（空闲 60 s 断开；在读请求体之前就拒绝的回复带 `Connection: close`，416 带 `Content-Length: 0`，视频拖动断开时关连接）。验证：`test_server.py`（新增 ETag/304、页面 no-cache、同一连接连续 416/200/304、未登录上传关连接）、ruff 0.11.7；浏览器里第二次打开物体页，68 个文件全部来自缓存，传输 0 字节（第一次 9.8 MB）。未做：缩略图（服务器系统 Python 没有 Pillow，标准库解不了图片），首次打开仍要下原图。
- 2026-10-20：学校场景和杯盘上了数采平台：部署仓库已在 `aea8d4a`（另一个会话部署）；补了场景的 LFS 文件，场景预览图（两张源孪生渲染转 jpg、一张 Isaac 头部画面）放在部署机的 `real2sim/scenes/real_scene4_school/preview/`（不入库），物体渲染图放在 `real2sim/objects/Rigid/paper_*/*/preview/`，元数据放进资产库镜像；重启服务后用平台的扫描函数核对：场景关联 `g1d_school`、已构建、3 张预览；5 件物体都有生成脚本、质量、尺寸和渲染图。页面要登录，没有在浏览器里看。
- 2026-10-20：接入学校实验室绿幕桌（`real_scene4` / `g1d_school`）和纸杯、纸盘资产，来自同事的 MuJoCo 孪生（PR #2）。
- 2026-10-20：采集、评估流水线规范化合并进 `main`（PR #1）：数据契约、通用采集流水线、场景工具、Agent 文档和 CI。
- 2026-10-20：随机朝向那批 100 条数据私有上传 ModelScope（`4562fb4`）。
- 2026-10-19：随机倒豆手、杯子偏航和大范围杯位（`a1b6db5`），修复量杯在倒豆循环里的豆子计数；按这套设置采完 100 条，左右手各 50（`353730a`）。
- 2026-10-19：100 条量杯数据上传 ModelScope 私有仓库；平台的物体登记简化（取消角色字段，尺寸改为自由文本）。
- 2026-10-19：用实测量杯重采 100 条；夹持点从杯口下 32 mm 改为 45 mm（高杯在夹爪里会转动）。
- 2026-10-19：量杯、塑料马克杯、红豆按实测重建；红豆改用等体积的 5.6 mm PBD 颗粒，0.1 g。
- 2026-10-19：新增数采平台（场景、物体、任务、质检、Sim↔Real 指标），加了账号、管理员和物体照片，部署到 `5090-2`。
- 2026-10-19：旧量杯加 4 mm 豆子的 100 条批次被取代（导出停在 8 条）。
- 2026-10-17：新任务 `g1d_pour_beans_measuring_cups`；接入带相机的 Dex1-1 夹爪；新增三个塑料杯资产。
- 2026-10-16：三个场景各自成为 env 配置并装入共享库；自建资产移到 `real2sim/`；相机按棋盘格和真机画面重新标定；新任务 `g1d_pour_beans_cup_to_cup`。
- 2026-10-14：倒水任务改为倒红豆；修复拾取和叠块（TCP 雅可比、线性 PD 定心）；重建实验室场景。
- 2026-10-13：搭建服务器环境，接入 G1-D + Dex1；冷水壶资产，以及流体检查只能走 CPU 状态管线。
