# Project Handoff

更新时间：2026-10-19
当前分支：`main`。流水线规范化的改动已通过 [PR #1](https://github.com/Zheng-Chong/RoboRSR/pull/1) 合并（2026-10-20），短分支 `chongzheng/datagen-real2sim-pipeline` 已删除。
当前目标：把 G1-D 的三条流水线规范化：Real2Sim 采集（物体、场景）、仿真数据生成，以及本仓库内的仿真评估。训练在仓库外。

本文件只记录当前状态。逐次实验的细节（相机拟合、抓取调参、倒豆参数扫描、各次服务器验证）见 `git show 4fb9290:HANDOFF.md`。数采平台的运维细节（端口、账号、隧道、重启方法）已移到本机 `.agent/ops.local.md`，不入库，因为仓库是公开的。

## 项目状态

- **机器人** G1-D + Dex1-1（带相机版夹爪）：两条 7 自由度手臂加两指夹爪，关节角控制，30 Hz；轮子锁死，升降柱和腰部保持初始值。
  - 头部用双目的左眼 `cam_head`（棋盘格标定 fx≈279.5，宇树合同外参），两个腕部相机 fx≈314；投影到真机画面核对过（误差 0.4–3.6 px）。
  - 关节驱动参数（刚度 4400、阻尼 40）和夹爪力是拍定的，没做系统辨识。同事把"相机+动作标定工具包"作为 `g1d_calib_toolkit_20261019.zip` 传到了仓库根目录，还没接入。
  - 构建：`python scripts/g1d/build_asset.py`（读 `real2sim/robot/g1d_v4`）。下载官方资产后先跑一次 `utils/update_embodiment_config_path.py`。
- **场景**（`real2sim/scenes/`）：env 配置 `g1d` 是办公室，`g1d_counter` 是吧台，`g1d_lab` 是实验室。三个都是从照片拟合的，尺度靠先验；没有卷尺实测，也没有叠图验收。状态汇总：`python scripts/scenes/acceptance.py`（目前全部 pending）。
- **物体**：每件资产的来源和匹配程度看 `real2sim/ASSETS.md`。
  - 量杯、塑料马克杯、红豆有实测尺寸和质量。
  - 共享资产库缺 `measuring_cup`、`plastic_mug`，以及量杯任务的布局。
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

- **主工作区里未提交的改动**：物体 SKILL 会话的改动（`CLAUDE.md`、`README.md`、`export_office_beans.py`、平台 `index.html`、`scripts/objects/SKILL.md`）还在主工作区，没有提交。和新 `main` 模拟合并过，不冲突。它们提交时，往 `HANDOFF.md` 加的记录要压缩后放进本文件的结构里。
- **采集配置下的严格判据有方差**：同一组动作回放，2 条里有 1 条最后差 1 颗豆子。评估策略时，严格成功率要在同一布局上重复多次才有意义。
- **XPolicyLab 注册**：`XPolicyLab/utils/robot/_robot_info.json` 里还没有 `g1d*`，用 XPolicyLab 的 `train.sh` 会拿不到动作维度。要给 XPolicyLab 提 PR（子模块不在本仓库里改）。
- **legacy 数据集**：和新契约在头部图像键、指令两处不同；用它训练的策略，评估前要先对齐。
- **新流水线还没跑过整批**：collection_v3 是用旧采集器采的，下一批数据应该改用 `scripts/datagen/run_batch.py`。
- **场景**：没有卷尺实测，没有叠图；`real_scene1` 和 09-21 那次真机运行时的摆放不一样，那批画面不能用来叠图。下次去现场按 `real2sim/scenes/SKILL.md` 一次采齐。
- **真机对齐**：
  - 动力学没做系统辨识；末端位姿动作还不支持（需要 G1-D 的 cuRobo 配置，并且左右臂各用一个 planner）；
  - 真机开环回放的关卡没有工具；
  - 真机夹爪读数 0–5.4 和指尖行程是否成线性没有标定；
  - 真机腕部相机的流名左右互换（部署端 `SWAP_WRISTS=True`）。
- **可达性**：杯对杯 seed 1、壶倒杯 seed 0 都是够不到的老问题。G1 手臂在桌面高度的可达距离约 0.45 m，自有任务的物体应放在 y≈[-0.55, -0.25]。
- **数采平台**：
  - 权限只有两级；HTTP 明文传密码；
  - `5090-2` 上的资产库是手动刷新的元数据镜像；
  - 看不到 `scripts/datagen` 的产出（`quality.json`、数据集登记表）；
  - 公开仓库的 git 历史里仍然有运维细节，建议把仓库改为私有。
- **零碎**：
  - 旧批次 `office-beans-100-20261019` 的 final-audit 进程还在等一个不会完成的导出，需要手动结束；
  - 服务器的 `/usr/local/bin/ffmpeg` 不带 libx264，新建环境要在仿真环境的 `bin` 里链接系统的 `ffmpeg`；
  - 指定了 `robot_config`/`scene_config` 的 `_task.yml` 会覆盖 g1d 的机器人或场景；
  - `21196` 的 `/data2` 已用 96%，只剩约 317 GB。

## 下一会话

1. 先看 `git log`、本文件和 `scripts/datagen/CONTRACT.md`。主工作区里还有物体建模 SKILL 会话没提交的改动（见"最近任务"里的"并行会话"）；拉取新的 `main` 前，先让那个会话提交，或者先 stash。
2. 下一批数据用新流水线采集。先跑单回合探针，确认 `joint_names`、夹爪 state 和图像都正常，再按 `scripts/datagen/SKILL.md` 批量采集、导出、审计、回放，最后登记。
3. 去现场按 `real2sim/scenes/SKILL.md` 采集（ChArUco 标定板照片、卷尺、灯光、机器人相机画面），回来跑 `overlay.py` 和 `acceptance.py`。
4. 给 XPolicyLab 提 PR，注册 `g1d*` 的动作维度；然后用一个 LeRobot 策略在 `--env-cfg g1d_beans_collect` 下走通"外部训练 → 本仓库评估"的整条链。

## 最近历史

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
