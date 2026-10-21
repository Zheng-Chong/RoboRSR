# Project Handoff

更新时间：2026-10-21
当前分支：`main`。深圳实验室场景（[PR #22](https://github.com/Zheng-Chong/RoboRSR/pull/22)）已于 2026-10-21 合并（`2975b4a`）并部署到数采平台。更早：流水线规范化（[PR #1](https://github.com/Zheng-Chong/RoboRSR/pull/1)）和学校场景接入（[PR #2](https://github.com/Zheng-Chong/RoboRSR/pull/2)）都已于 2026-10-20 合并，短分支已删除。
当前目标：把 G1-D 的三条流水线规范化：Real2Sim 采集（物体、场景）、仿真数据生成，以及本仓库内的仿真评估。训练在仓库外。

本文件只记录当前状态。逐次实验的细节（相机拟合、抓取调参、倒豆参数扫描、各次服务器验证）见 `git show 4fb9290:HANDOFF.md`。数采平台的运维细节（端口、账号、隧道、重启方法）已移到本机 `.agent/ops.local.md`，不入库，因为仓库是公开的。

## 项目状态

- **机器人** G1-D + Dex1-1（带相机版夹爪）：两条 7 自由度手臂加两指夹爪，关节角控制，30 Hz；轮子锁死，升降柱和腰部保持初始值。
  - 头部用双目的左眼 `cam_head`（棋盘格标定 fx≈279.5，宇树合同外参），两个腕部相机 fx≈314；投影到真机画面核对过（误差 0.4–3.6 px）。
  - 关节驱动参数（刚度 4400、阻尼 40）和夹爪力是拍定的，没做系统辨识。夹爪上限是每指 20 N；官方规格（零售商转载）写的是夹持力 5–120 N 可调，是否提高上限待用户决定（见「剩余事项」）。仿真里能读到夹持力：`obs["gripper_force"]`（指垫 ContactSensor，单位 N，不进数据集）。同事把"相机+动作标定工具包"作为 `g1d_calib_toolkit_20261019.zip` 传到了仓库根目录，还没接入。
  - 构建：`python scripts/g1d/build_asset.py`（读 `real2sim/robot/g1d_v4`）。下载官方资产后先跑一次 `utils/update_embodiment_config_path.py`。
- **场景**（`real2sim/scenes/`）：env 配置 `g1d` 是办公室，`g1d_counter` 是吧台，`g1d_lab` 是公司实验室，`g1d_school` 是学校实验室绿幕桌，`g1d_shenzhen_lab` 是深圳实验室办公桌。前三个从照片拟合，尺度靠先验；学校的来自同事用真机相机画面拟合的 MuJoCo 孪生；深圳实验室是第一个按平台登记、用卷尺定尺度的场景（卷尺三项通过，站位和叠图待做）。其余都没有卷尺实测，都没有叠图验收。状态汇总：`python scripts/scenes/acceptance.py`（目前全部 pending）。新场景先在数采平台登记（≥ 5 张照片 + 卷尺实测，写 `registration.json`），再按 `real2sim/scenes/SKILL.md` 交给 coding agent 建模。新场景先在数采平台「登记场景」（至少 5 张照片 + 卷尺实测，写 `real2sim/scenes/<场景>/registration.json`），再按 `real2sim/scenes/SKILL.md` 建模。
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
- **数据**：所有数据集登记在 `scripts/datagen/registry.json`。目前有 2 份办公室量杯全量倒豆数据，各 100 条，LeRobot v3.0，都是按旧的 legacy 格式导出的：
  - 固定朝向的一份，已上传 ModelScope 私有仓库；
  - 随机左右手、随机朝向、大范围位置的一份，也已上传 ModelScope 私有仓库（单独的仓库）。
- **数采平台** `src/real2sim_platform/`：部署在 `5090-2`，经阿里云反向隧道对外访问。
- **服务器**：
  - `21196`（4090）：共享资产库 `/data2/zhongzheng/Project/RoboDojo_assets/Assets`；环境用 `source /data2/zhongzheng/Project/robodojo_env.sh`；隔离目录放在 `/data2/tmp/robodojo/agent-remote-runs/`。
  - `5090-2`（8×5090）：仿真环境 `/root/miniconda3/envs/RoboDojo`；导出环境 `robodojo-lerobot`（lerobot 0.4.4，10-19 补装了 h5py 3.16，`pip -i` 用清华镜像）；上传环境 `/data/robodojo-runs/ms-venv`；运行目录在 `/data/robodojo-runs/`。

## 最近任务

### G1-D 夹爪力反馈（2026-10-21，分支 `chongzheng/robot-g1d-gripper-force`）

- **用户要求**：仿真里要能知道夹爪用了多大力，在三个方案里选了"指垫接触传感器"。
- **实现**：`robot_config/g1d.py` 新增 `FINGER_PADS`（四个手指关节各自对应的指垫 `*_dex1_finger_link_{1,2}_tip`，即 Link{1,2}_3），并打开 `activate_contact_sensors`。`RobotManager._get_SceneCfg` 发现 `FINGER_PADS` 时，给场景加 `robot{idx}_pads` 这个 ContactSensor。`get_gripper_force()` 返回每个指垫的接触合力大小（N，顺序同 `gripper_joints_name`），`ObsManager` 写进 `obs["gripper_force"][gripper_name]`。没写 `FINGER_PADS` 的机器人（X5、Franka）不受影响。录制器只取 `state_keys`，所以 16 维 state 和数据集格式不变（`CONTRACT.md` 已注明）。
- **验证**：本机 `scripts/check.sh` 通过（22 passed，1 skipped；契约单测加了 `gripper_force` 断言）。`5090-2` 隔离目录 `/data/robodojo-runs/gripper-force-20261021-142346`（GPU 2），`g1d_beans_collect` 量杯 seed 0：`success_rate` 1.0、`eval_time` 1；张开时力为 0，夹住时每个指垫 15–22 N（最大 22.6）。同一回合把上限改为 120 N、刚度改为 6e4（目录 `…-120n`，GPU 3，只改了服务器上的副本，没提交）：也成功，夹持中位约 24 N、最大 30 N。原因是脚本专家只把夹爪关到指令 0.3，力由刚度乘以剩余行程决定，到不了上限。`g1d_general_pickup` 没测：这份资产副本里没有它的布局。
- **没做**：上限仍是 20 N；力没有进数据集，也没给策略当输入；真机 Dex1-1 能否读出对应的量没查。


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
- **共享库**（用户同意后，2026-10-21 13:5x，按 README「改服务器共享资产库」五步）：新目录 `Room/real_scene5`（8 个文件）和 `Eval_Layout/RoboDojo/g1d_shenzhen_lab`（574 个布局，0/1/2 三组），没有覆盖任何东西，所以没做备份。
  - 来源是上面隔离目录里验证过的产物，经本机中转拷到共享库临时目录再改名到位；Room 和提交的文件 sha256 一致；拷入后 582 个文件和验证产物的 sha256 逐个一致；目录权限对齐为组可写（同 `real_scene3`）。
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
  - **GPU 验证**：在 `21196` 的隔离目录 `datagen-20261019-225252` 里跑（资产是 100 条那一轮的同一份，共享库没动）：
    - 脚本专家走新的 `--policy_module` 入口，`g1d` 下量杯 seed 0：`success_rate` 1.0，`eval_time` 1；
    - `run_batch.py` 用 2 张卡、分片 300 和 301 各 2 个布局：4 次尝试 4 次合格，每条 672 颗豆子全部进矮杯；夹持时夹爪 state（实测）是 0.75，action（指令）是 0；
    - 在 seed 300 上不加 `--allow_train_layouts` 跑评估，被拒绝；
    - 开环回放见上面"数据契约"一节和 CONTRACT（采集配置下偏差 ≤ 1e-4 rad，1/300 s 下洒出约 9%）。
  - **导出和审计**：在 `5090-2` 的隔离目录 `datagen-pipeline-20261019-223558` 里跑：官方转换器转了 2/2 个文件，每臂 8 维，AV1；`export.py` 逐回合核对通过（810 帧）；`audit.py` 输出 validated。
  - 两个隔离目录都保留着，`21196` 上的原始数据已删除。本机留档在 `output/datagen_validation_20261019/`（报告、日志、质检单、数据集元数据、两帧图像）。
  - **合并 `a1b6db5` 之后在 `21196` 复跑**：
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

- **主工作区里未提交的改动**（10-20 晚核对）：`CLAUDE.md`、`HANDOFF.md`、`README.md`、`real2sim/scenes/SKILL.md`、`scripts/g1d/export_office_beans.py`，主要是「服务器切到 5090-2」那次的文档改动；主工作区的本地 `main` 还停在 `a694da5`，落后 `origin/main`。物体 SKILL 已随 PR #5 合并。
- **采集配置下的严格判据有方差**：同一组动作回放，2 条里有 1 条最后差 1 颗豆子。评估策略时，严格成功率要在同一布局上重复多次才有意义。
- **XPolicyLab 注册**：`XPolicyLab/utils/robot/_robot_info.json` 里还没有 `g1d*`，用 XPolicyLab 的 `train.sh` 会拿不到动作维度。要给 XPolicyLab 提 PR（子模块不在本仓库里改）。
- **legacy 数据集**：和新契约在头部图像键、指令两处不同；用它训练的策略，评估前要先对齐。
- **新流水线还没跑过整批**：collection_v3 是用旧采集器采的，下一批数据应该改用 `scripts/datagen/run_batch.py`。
- **学校场景**：桌面高度和底盘偏航都依赖假设（零升降、12 号电机即躯干偏航），要用卷尺和叠图核对；从 sys01 拷真机头部和腕部画面（`outputs/g1d-real2sim`）做叠图。杯盘要在数采平台登记实测后，按 `scripts/objects/SKILL.md` 重建。
- **共享库的倒豆布局过期**：`g1d`、`g1d_counter`、`g1d_lab` 的倒水、杯对杯布局是旧豆子参数（4 mm，`particle_mass` 6e-5），和当前 `build_asset.py`（5.6 mm）不一致；`g1d_shenzhen_lab` 已是新参数。要统一，就在隔离目录重跑 `build_asset.py`，经同意后替换（先备份）。
- **深圳实验室**：已合并、部署、装共享库；站位卷尺（桌沿到根节点、横向偏移）和三视角叠图后，`acceptance.py` 才能通过。
- **场景**：除深圳实验室外没有卷尺实测，都没有叠图；`real_scene1` 和 09-21 那次真机运行时的摆放不一样，那批画面不能用来叠图。下次去现场按 `real2sim/scenes/SKILL.md` 一次采齐。
- **夹爪上限**：仿真每指 20 N，官方写 5–120 N 可调。改上限会改变物理，和已采数据的 env 不再一致（`replay.py` 能看出来），所以要先确认真机实际设的是多少，再决定是否改、是否重采。
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
  - 公开仓库的 git 历史里仍然有运维细节，建议把仓库改为私有。
- **零碎**：
  - 旧批次 `office-beans-100-20261019` 的 final-audit 进程还在等一个不会完成的导出，需要手动结束；
  - 服务器的 `/usr/local/bin/ffmpeg` 不带 libx264，新建环境要在仿真环境的 `bin` 里链接系统的 `ffmpeg`；
  - 指定了 `robot_config`/`scene_config` 的 `_task.yml` 会覆盖 g1d 的机器人或场景；
  - `21196` 的 `/data2` 已用 96%，只剩约 317 GB。

## 下一会话

0. 实现平台登记的 `g1d_pour_beans_any_cups`：在平台任务抽屉复制「实现任务」文本交给 coding agent（输入 `real2sim/tasks/g1d_pour_beans_any_cups/registration.json`）。
0. 深圳实验室（已合并、部署、装共享库）：去现场量站位、拍头部 / 腕部画面做叠图（`overlay.py --env-cfg g1d_shenzhen_lab`）。
1. 先看 `git log`、本文件和 `scripts/datagen/CONTRACT.md`。主工作区里还有物体建模 SKILL 会话没提交的改动（见"最近任务"里的"并行会话"）；拉取新的 `main` 前，先让那个会话提交，或者先 stash。
2. 下一批数据用新流水线采集。先跑单回合探针，确认 `joint_names`、夹爪 state 和图像都正常，再按 `scripts/datagen/SKILL.md` 批量采集、导出、审计、回放，最后登记。
3. 去现场按 `real2sim/scenes/SKILL.md` 采集（ChArUco 标定板照片、卷尺、灯光、机器人相机画面），在平台「登记场景」录入，回来跑 `overlay.py` 和 `acceptance.py`。
4. 给 XPolicyLab 提 PR，注册 `g1d*` 的动作维度；然后用一个 LeRobot 策略在 `--env-cfg g1d_beans_collect` 下走通"外部训练 → 本仓库评估"的整条链。
5. 学校场景：用户重建 sys01 连接后拷真机画面做叠图（`overlay.py --env-cfg g1d_school`）；要用学校场景做任务时，再写杯盘布局和任务。

## 最近历史

- 2026-10-21：G1-D 夹爪力反馈：指垫 ContactSensor，输出 `obs["gripper_force"]`（N），不进数据集；5090-2 上量杯 seed 0 通过，夹持时每个指垫 15–22 N。
- 2026-10-21：按平台登记新建深圳实验室场景 `real_scene5_shenzhen_lab`（env 配置 `g1d_shenzhen_lab`），卷尺定尺度，5090-2 上 `build_asset.py` 和脚本演示通过，经用户同意装进共享库（不含量杯任务布局）；修 `build_asset.py` 相对 `--assets` 时的软链接问题。PR #22 合并（`2975b4a`）并部署到 `5090-2`。
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
