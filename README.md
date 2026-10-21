<h2 align="center">基于 Multi-Agent 的 Real2Sim2Real 闭环自进化</h2>

<p align="center">Unitree G1-D（Dex1 夹爪）· Isaac Sim 5.1 / IsaacLab 2.3 · 基于 <a href="https://github.com/RoboDojo-Benchmark/RoboDojo">RoboDojo</a></p>

通过视频和多视角图像生成高保真 3D 资产与场景，在 Isaac Lab 中构建真实工位的仿真环境，利用仿真数据增强与 RL/IL 优化策略，再回到真实世界持续自进化。

本仓库是 RoboDojo 的分支，在其仿真与评测框架上接入了 G1-D 机器人、自建资产和 G1-D 专用任务。

**要端到端证明的一件事**：用仿真生成的轨迹训练策略，能在真机上零样本（zero-shot）成功。

## 现阶段已实现

**机器人本体**

- 两条 7 自由度手臂（coupled 模式）加 Dex1 两指夹爪，关节角控制，动作频率 30 Hz（`dt = 1/300 s`，每个动作 10 个物理步）。
- 头部双目相机加两个腕部相机，位姿按真机画面对齐；全身配色按产品图和真机画面核对。
- 两个版本，对应官方的标准版和旗舰版，接口按 unitree_sdk2 的 G1-D 示例：
  - **桌面版**（机器人 `g1d`，env 配置 `g1d` / `g1d_counter` / `g1d_lab` / `g1d_school` / `g1d_beans_collect`）：底盘固定。手臂和夹爪之外，可选动作键 `waist_joint_state`（[偏航, 俯仰] rad，对应官方 12、14 号电机）和 `lift_joint_state`（[升降柱高度] m，0–0.42，按官方 76.5 mm/s 限速）；不发这两个键就保持当前值，原来的 16 维策略和数据不受影响。
  - **移动版**（机器人 `g1d_mobile`，env 配置 `g1d_mobile`）：桌面版加差速底盘，可选动作键 `base_velocity`（[vx m/s, vyaw rad/s]，底盘坐标系，限幅 ±1.5、±0.6，同 `AgvClient.Move`）；不发就停车。官方 URDF 只有两个驱动轮、没有万向轮，所以底盘用三个虚拟平面关节（x、y、偏航）驱动，轮子不转、没有碰撞体，底盘外壳照常碰撞。
  - env 配置写 `observation.robot.body_states: true` 时，观测里加上 `waist_joint_state`、`lift_joint_state`，移动版再加 `base_velocity`（实测）和 `base_pose`（相对回合起点的 [x, y, yaw]）；只有 `g1d_mobile` 默认打开。
  - GPU 自检：`scripts/g1d/body_probe.py`（用法见文件头）。

**资产和物理**

- 程序化生成 3 款玻璃冷水壶（`scripts/objects/make_pitcher.py`），碰撞体保留壶内空腔和出水口。
- 流体倒水物理已验证：开口壶倒出 99.4%，封口对照保留 99.98%（`scripts/objects/check_pitcher_fluid.py`）。
- 倒水任务改为倒红豆（PBD 颗粒，现为与实测等体积的 5.6 mm；4 mm 时开口壶可倒出 58.7%，封口对照保留 100%，5.6 mm 未复测，预计会堵 13 mm 壶嘴）。
- 程序化生成水杯 / 塑料杯（`scripts/objects/make_water_cup.py`）。
- 3 个按实拍重建的数字孪生场景（办公室、台面、实验室）、倒水场景包和 G1-D v4 机器人 URDF 都在 `real2sim/` 下，二进制文件走 Git LFS。

**任务和脚本专家**

- 五个 G1-D 任务，默认放在重建的办公室场景 `real_scene1` 中：`g1d_pour_water_into_cup`（壶倒杯）、`g1d_pour_beans_cup_to_cup`（两杯空中对倒）、`g1d_pour_beans_measuring_cups`（两只量杯空中对倒，500 ml 倒进 400 ml）、`g1d_general_pickup`、`g1d_stack_blocks`。
- `scripts/g1d/scripted_demo.py`：用仿真中的物体真值位姿和雅可比逆运动学写出抓取、搬运、倾倒路点，不需要策略服务器。拾取和叠块 seed 0 成功；杯对杯倒豆 seed 0/2 成功、seed 1 失败；量杯对倒吧台 seed 0/1/2、办公室和实验室 seed 0 成功；壶倒杯 seed 1 成功、seed 0 失败（详见 `HANDOFF.md`）。

## 快速开始

资产分两部分，顺序是**先拉代码，再下载资产，最后在资产目录里生成自建内容**：

- `real2sim/`（随仓库，Git LFS）：自建的场景、场景包和 G1-D v4 URDF，这些不能重新生成。
- `Assets/`（不入库）：由 `scripts/init_assets.sh` 从 Hugging Face 下载 RoboDojo 官方资产（约 41 GB），`Assets` 是指向 `.cache/robodojo_assets_repo/Assets` 的软链接。G1-D 机器人、评测布局和自建物体都由下面的脚本生成到这里。

需要 Linux + NVIDIA GPU（Isaac Sim 5.1），并提前装好 `git-lfs`。

```bash
# 1. 拉代码（含 LFS 文件）
git lfs install
git clone https://github.com/Zheng-Chong/RoboRSR.git RoboDojo && cd RoboDojo
git lfs pull                                   # clone 前没装 git-lfs 时补拉

# 2. 安装环境：conda 环境 RoboDojo（Python 3.11）、子模块、Isaac Sim、IsaacLab、cuRobo
bash scripts/install.sh -i
conda activate RoboDojo

# 3. 下载 RoboDojo 官方资产到 Assets/，并生成 cuRobo 配置（下载后必须运行一次）
bash scripts/init_assets.sh
python utils/update_embodiment_config_path.py

# 4. 生成自建物体（写入 Assets/Object/RoboDojo/Rigid/），然后刷新 uuid_map.json
python scripts/objects/make_pitcher.py            # 冷水壶 pitcher
python scripts/objects/make_water_cup.py          # 水杯 water_cup
python scripts/objects/make_water_cup.py --plastic   # 塑料杯 plastic_cup
python scripts/objects/make_school_tableware.py   # 学校纸杯 paper_cup、纸盘 paper_plate
python scripts/objects/make_airpods.py            # 耳机充电盒 airpods
python Assets/Object/RoboDojo/Rigid/format.py

# 5. 构建 G1-D 机器人（读 real2sim/robot/g1d_v4，写 Assets/Robots/g1d/）、红豆材质，
#    安装 4 个场景到 Assets/Room/real_scene{1,2,3,4}/，并为每个场景生成 G1-D 任务的评测布局。
#    会从 GitHub 拉 unitree_ros 的 G1 URDF；离线时用 --src 指向本地 unitree_ros 镜像。
python scripts/g1d/build_asset.py
```

第 4 步必须在第 5 步之前：`build_asset.py` 生成布局时要读壶和杯子的 `metadata.json`。

**场景切换**：每个场景对应一个 env 配置，评测和脚本演示时用 `--env-cfg`（`scripted_demo.py` 用 `--env_cfg_type`）选择。机器人站位和升降柱高度写在各自的 `env_cfg/robot/<配置>.yml` 里，物体跟着机器人平移，同一编号的布局在各场景里物体相对机器人的位置一致（例外：杯对杯倒豆的两个杯子按布局随机抽取，各场景抽到的位置不同）。

| env 配置 | 场景 | 桌面高度 | 机器人 |
| :-- | :-- | :-- | :-- |
| `g1d` | `real_scene1` 办公室升降桌 | 0.68 m | 根节点 y=-0.73，升降柱 0 |
| `g1d_counter` | `real_scene2` 吧台（落地柜） | 0.833 m | 根节点 y=-0.80（底盘伸不进柜下），升降柱各 0.0765 |
| `g1d_lab` | `real_scene3` 实验室培训桌 | 0.749 m | 根节点 y=-0.73，升降柱各 0.0345 |
| `g1d_school` | `real_scene4` 学校实验室绿幕桌 | 0.763 m | 根节点 y=-0.79、左偏 1.63°，升降柱 0（桌面相对躯干的高差按拟合值保留） |
| `g1d_shenzhen_lab` | `real_scene5` 深圳实验室办公桌（卷尺实测） | 0.75 m | 根节点 y=-0.73（站位未实测），升降柱各 0.035 |
| `g1d_mobile` | `real_scene1` 办公室升降桌（移动版） | 0.68 m | 起点同 `g1d`，底盘可移动 |

布局写在 `Assets/Eval_Layout/RoboDojo/<配置>/`，评测结果也按配置分目录。

可选：`bash scripts/RoboDojo/download_data.sh`、`bash scripts/RoboDojo/download_ckpt.sh` 下载 RoboDojo 官方数据和模型权重。官方安装细节见 [RoboDojo 文档](https://robodojo-benchmark.com/doc/usage/install-and-download/)。

**脚本专家演示**（无策略服务器，视频和 `_result.json` 写入 `eval_result/`）：

```bash
CUDA_VISIBLE_DEVICES=1 PYTHONPATH=.:XPolicyLab EVAL_NUM=1 python scripts/g1d/scripted_demo.py \
    --env_cfg_type g1d --task_name g1d_stack_blocks --seed 0
```

**策略评测**：

```bash
bash scripts/robodojo.sh eval \
    --policy-dir XPolicyLab/policy/<POLICY_NAME> \
    --task g1d_stack_blocks \
    --ckpt <CKPT_NAME> \
    --policy-env <POLICY_ENV> \
    --env-cfg g1d --action-type joint --eval-num 1
```

**资产物理检验**：

```bash
PYTHONPATH=. python scripts/objects/check_object.py Assets/Object/RoboDojo/Rigid/pitcher/00000 --image pitcher_check.png
PYTHONPATH=. python scripts/objects/check_pitcher_fluid.py --device cpu --out output/pitcher/fluid   # 加 --red-beans 测红豆
```

流体检查必须用 `--device cpu`（CPU 状态管线，PBD 求解仍在 GPU）；GPU 状态管线下粒子坐标不会回读，结果无效。

**仿真数据生成**：脚本专家逐回合录成 XPolicyLab 标准轨迹（HDF5），通过任务插件的严格验收后，用 XPolicyLab 官方转换器导出为 LeRobot v3.0。训练在仓库外，数据格式和语义的约定见 [`scripts/datagen/CONTRACT.md`](scripts/datagen/CONTRACT.md)，完整流程见 [`scripts/datagen/SKILL.md`](scripts/datagen/SKILL.md)，已有数据集登记在 `scripts/datagen/registry.json`。

```bash
python scripts/datagen/run_batch.py --task g1d_pour_beans_measuring_cups --env-cfg g1d_beans_collect \
    --candidates 9 --shards 100-107 --gpus 0-7 --target 100 --output output/datagen/<批次>   # 多卡采集
python scripts/datagen/export.py --run output/datagen/<批次> --episodes 100 --name <名称>    # LeRobot 环境
python scripts/datagen/audit.py --dataset output/datagen/<批次>/lerobot/<名称>             # 独立审计
PYTHONPATH=.:XPolicyLab python scripts/datagen/replay.py --raw output/datagen/<批次>/raw/seed100 \
    --report output/datagen/<批次>/replay.json [--env-cfg g1d]                              # 开环回放一致性
```

**Real2Sim 数采管理平台**（Web，只用 Python 标准库，本机无 Isaac 也能跑）：

```bash
python3 src/real2sim_platform/server.py --add-user admin --name 管理员 --admin   # 只需一次：建第一个管理员（提示输入密码，至少 8 位）
python3 src/real2sim_platform/server.py          # 打开 http://127.0.0.1:8765 登录，在「成员管理」给其他人建账号
# 服务器上给团队用：指向共享资产库和各隔离目录的评测结果
python3 src/real2sim_platform/server.py --host 0.0.0.0 \
    --assets /data2/zhongzheng/Project/RoboDojo_assets/Assets \
    --eval-root eval_result --eval-root /data2/tmp/robodojo/agent-remote-runs
```

每个人用自己的账号登录，录入和质检结论自动以账号的显示名署名（录入时没法改署名）。账号存在平台数据库里（密码 PBKDF2 加盐哈希），分管理员和成员：管理员在侧栏「成员管理」页新建账号（默认填好随机初始密码，建好后显示一次可复制的登录信息）、改显示名、设为管理员、重置密码（生成新随机密码，踢掉该账号已有的登录）、删除账号；不能删除自己或取消自己的管理员，所以至少总有一个管理员。命令行只用来建第一个管理员或在忘记所有管理员密码时救急：`--add-user 用户名 [--name 显示名] [--admin]` 新建或重置密码，`--remove-user 用户名`，`--list-users`；脚本里可以把密码放在标准输入第一行。登录后在侧栏底部可以改自己的密码、退出登录。会话 14 天有效；同一用户名连续输错 5 次锁定 5 分钟。平台走 HTTP，密码在网络上是明文传输，经公网访问时有被窃听的风险。

启动时和点「从项目同步」时扫描仓库：`real2sim/scenes/`（照片、相机拟合残差、USD、`check_report.json`）、任务清单和 G1-D 任务用到的物体、资产库里的 `metadata.json` 和评测布局数、`--eval-root` 下的 `_result.json` 与各相机视频。页面分五块：

| 页面 | 自动得到 | 人工录入（按 Notion 月度目标的验收标准判定） |
| :-- | :-- | :-- |
| 场景重建 | 照片、拟合、构建、离线检查 | 卷尺实测（< 1 cm）、头部相机叠图（< 5 px，≥ 3 视角）、灯光、Isaac 渲染、入共享库 |
| 物体资产 | 用于哪些任务和标签、生成脚本、仿真尺寸/质量/摩擦、metadata 里按名称的模型尺寸、实拍照片与仿真渲染对照 | 登记实物（材质、质量、尺寸文本「名称 数值」、外观与建模要求、照片），生成给 coding agent 的建模任务；尺寸核对（误差 < 2 mm 或 2%、≤ 5 g）、5 项物理检验、7:3 划分 |
| 任务采集 | 布局数、回合数、`scripted_demo` 专家成功率（≥ 80%） | 合格回合目标（默认 500）、L0–L4 分级评测集进度、负责人 |
| 数据质检 | 结果写入、三路相机齐全、视频标签与结果一致 | 通过/拒绝（快捷键 J/K/P/R/U）、问题标签、真机对照结果；导出质检通过清单 |
| 总览 | 各环节完成度、资产泛化目标、拒绝原因 | Sim↔Real 结果一致率 AA、Cohen's κ、P(真机成功 \| 仿真成功) |

扫描结果和人工录入分开存（SQLite，默认 `.cache/real2sim_platform.sqlite3`，`--db` 可改），重新扫描不会覆盖录入内容。登记为实物孪生的物体，登记信息同时写到 `real2sim/objects/<id>/spec.json`（和 `reference/` 实拍照片同一目录，作为建模输入）；建好的资产在 `metadata.json` 顶层写 `measurements_mm`（按尺寸文本里的名称写模型实际值），平台从尺寸文本里解析出每个「名称 数值」，按名称自动核对。平台里的实测值是工作记录，定稿后按 `real2sim/ASSETS.md` 的维护规则写回生成脚本和登记表。自检：`python3 src/real2sim_platform/test_server.py`。

物体图片按 id（`<类型>/<类别>/<编号>`，与 `Assets/Object/RoboDojo/` 下相同）放在 `real2sim/objects/<id>/`：`reference/` 是实物照片（随仓库，走 LFS；网页上传也存到这里），`preview/` 是仿真渲染（可重新生成，不入库）。渲染在 GPU 服务器上每个物体单独出图：

```bash
PYTHONPATH=. python scripts/objects/check_object.py Assets/Object/RoboDojo/Rigid/measuring_cup/0000{0,1} --image-dir real2sim/objects
```

## 仓库结构

在 RoboDojo 原有结构上，与本项目相关的部分：

```text
env/robot_manager/robot_config/g1d.py   G1-D 加载（直接读 URDF）、材质绑定
env/robot_manager/robot_class/g1d.py    G1DLeft / G1DRight 手臂类
env_cfg/g1d{,_counter,_lab}.yml         G1-D 运行配置（30 Hz），每个场景一个
env_cfg/robot/g1d{,_counter,_lab}.yml   各场景的机器人站位、升降柱高度
env_cfg/sim/g1d_sim_config.yml          仿真步长 1/300 s
env_cfg/camera/g1d_camera.yml           头部双目 + 双腕相机
scripts/g1d/                            资产构建、脚本专家演示
scripts/objects/                        壶/杯生成、刚体/流体物理检验；SKILL.md：按平台登记给实物建模的流程
src/real2sim_platform/                  Real2Sim 数采管理平台（场景重建、物体资产、任务采集、数据质检）
task/RoboDojo/{config,tasks}/g1d_*      G1-D 任务
scripts/datagen/                        仿真数据生成与发布；CONTRACT.md 数据契约，SKILL.md 流程，registry.json 数据集登记
scripts/scenes/                         场景工具：ChArUco 标定板、相机叠图检查、场景验收汇总；流程见 real2sim/scenes/SKILL.md
scripts/check.sh                        提交前的 CPU 检查（CI 也运行它）
real2sim/                               自建资产（随仓库，二进制走 Git LFS）
  ASSETS.md                             自建资产登记表：真实对应物、匹配程度、共享库状态、待实测清单
  objects/<类型>/<类别>/<编号>/          物体实拍照片 reference/（入库）和仿真渲染 preview/（不入库）
  robot/g1d_v4/                         G1-D + Dex1-1 v4 URDF、网格、physics_contract（build_asset.py 的输入）
  scenes/real_scene1_office/            办公室孪生场景 → Room/real_scene1
  scenes/real_scene2_counter/           台面孪生场景 → Room/real_scene2
  scenes/real_scene3_lab/               实验室孪生场景 → Room/real_scene3
  scenes/real_scene4_school/            学校实验室绿幕桌孪生场景 → Room/real_scene4
  scenes/real_scene5_shenzhen_lab/      深圳实验室办公桌孪生场景 → Room/real_scene5
  scene3_pour_pack/                     倒水场景包（场景、物体、运动学与规划脚本）
Assets/                                 下载的官方资产 + 生成的 G1-D 资产（不入库，见快速开始）
HANDOFF.md                              跨会话交接记录（当前状态、决策、验证、遗留问题）
```

`real2sim/` 下各目录的 `preview/`、`demo/` 是本地生成的预览，不入库。

## 协作开发

### 资产放在哪里

同一份资产可能出现在好几个地方，改之前先弄清楚改的是哪一份：

| 位置 | 放什么 | 是否入库 | 谁能改 |
| :-- | :-- | :-- | :-- |
| `real2sim/` | 不能重新生成的自建资产：重建场景、场景包、G1-D v4 URDF | GitHub，二进制走 LFS | 所有人，随代码提交 |
| `scripts/objects/`、`scripts/g1d/build_asset.py` | 能重新生成的资产（壶、杯、G1-D 机器人、评测布局）的生成脚本 | GitHub | 所有人；只提交脚本，不提交生成结果 |
| 本机 `Assets/` | 官方资产 + 上面脚本的生成结果 | 不入库 | 自己；随时可以删掉重新生成 |
| 服务器共享资产库 `/data2/zhongzheng/Project/RoboDojo_assets/Assets` | 大家共用的 `Assets/` | 不入库 | **先征得同意再改**，见下文 |
| 服务器隔离目录 `/data2/tmp/robodojo/agent-remote-runs/<名称>-<时间>/` | 个人实验用的代码副本和资产覆盖目录 | 不入库 | 自己 |
| Hugging Face `RoboDojo-Benchmark/RoboDojo` | RoboDojo 官方资产 | — | 只读，不是我们的数据集 |
| `output/`、`eval_result/` | 预览图、视频、评测结果 | 不入库 | 自己 |

几条规则：

- **不要往 `Assets/` 里放要入库的文件。** `init_assets.sh` 发现 `Assets/` 不完整时，会把整个目录挪成 `Assets.partial.<时间戳>`，里面的东西也会一起被挪走。
- **以仓库为准。** 共享资产库或本机 `Assets/` 和仓库对不上时，以仓库里的 `real2sim/` 和脚本为准，重新运行 `build_asset.py` 生成。
- **共享资产库可能落后于代码。** 每个自建资产装没装进共享库、和真实的匹配程度，记在 [`real2sim/ASSETS.md`](real2sim/ASSETS.md)。

### 新增或修改资产

1. **能用脚本生成的**（参数化物体、布局）：只提交脚本，并在「快速开始」第 4 步或第 5 步加上生成命令。按数采平台的登记给实物建模，照 [`scripts/objects/SKILL.md`](scripts/objects/SKILL.md) 的流程做。
2. **不能生成的**（照片重建的场景、实测的 URDF 和网格）：放进 `real2sim/`。`.usd`、`.usdc`、`.stl`/`.STL`、`.png`、`.jpg`/`.jpeg` 会自动走 LFS；其他二进制格式（如 `.usdz`、`.glb`、`.obj`、`.mp4`）要先加到 `.gitattributes`，否则会以普通文件进入 git 历史，之后只能改写历史才能清掉。
3. **安装到 `Assets/`** 统一交给 `build_asset.py`，不要手动拷贝：新场景加到 `SCENES`（同时新增 `env_cfg/g1d_<场景>.yml` 和 `env_cfg/robot/g1d_<场景>.yml`），新任务的布局加到 `G1D_TASKS`。
4. **登记**：在 `real2sim/ASSETS.md` 加一行或更新对应行（真实对应物、各属性的匹配等级和依据、共享库状态）。

### 改服务器共享资产库

共享资产库会影响所有人的评测结果，按这个顺序来：

1. 在隔离目录里验证：建一个 `Assets/` 覆盖目录，要改的部分用自己的副本，其余软链接到共享资产库，在这里生成资产、跑评测。
2. 征得负责人同意。
3. 备份要被覆盖的部分：`tar czf /data2/tmp/robodojo/backup-shared-<时间>.tgz <目录>`。
4. 把隔离目录里**验证过的产物**拷进共享资产库（不要在共享库上重新生成），用 `diff -r` 确认和验证产物一致。
5. 在 `HANDOFF.md` 里记下改了什么、备份在哪。

### 协作规则

- **开始前**：`git pull --ff-only origin main`，然后读 `HANDOFF.md`，了解当前状态、决策和遗留问题。
- **分支**：仓库只保留 `main`。小改动直接提交到 `main`；大改动开短期分支（`<名字>/<范围>-<简述>`，如 `zixuan/task-hang-mugs`），合并回 `main` 后删除。
- **完成后**：更新 `HANDOFF.md`（日期、做了什么、关键决策、实际跑过的验证、遗留问题），和代码放在同一个提交里。
- **提交信息**：`[Scope] type: 简述`，Scope 取 `Task`、`Env`、`Config`、`scripts`、`utils`、`fix`、`refactor`，type 取 `feat`、`fix`、`update`、`refactor`、`docs`、`chore`。
- **多人同时改**：每个人（每个 agent 会话）用自己的 `git worktree` 和短分支，不在同一个工作区里并行改；共用一个工作区时，曾经出现过把别人暂存的文件提交进自己 commit 的情况。只提交自己改的文件，`git add` 时逐个列出文件，不要用 `git add -A`。
- **远程仓库**：只推送到 `origin`（Zheng-Chong/RoboRSR）；`upstream` 是 RoboDojo 官方仓库，不要往上推。子模块 `XPolicyLab/`、`third_party/` 不在本仓库里改。
- **提交前验证**：`bash scripts/check.sh`，它会对改动过的 Python 文件跑 ruff，再跑单元测试、`task_inventory.py --check` 和 `git diff --check`；GitHub Actions 对每次推送和 PR 跑同样的检查。改了任务、环境或资产的，还要在服务器上跑一次脚本演示或评测，确认退出码为 0，并且 `_result.json` 里 `eval_time >= 1`；只看退出码不够。
- **git-lfs**：macOS 上装官方二进制（[git-lfs releases](https://github.com/git-lfs/git-lfs/releases)），不要用 Homebrew 装，它会从源码编译一整条依赖链。

## TODO

**本体标定**

- [x] 相机内参：棋盘格标定（头部 fx≈279.5，腕部 fx≈314）；外参用宇树合同值，已用真机画面投影核对。畸变（< 2%）和腕部主点偏移还没建模。
- 关节执行器的刚度、阻尼是拍定的，需要做系统辨识。夹爪按真机 kp 估算为每指 3.6e4 N/m、上限 55 N（假设 q 和行程成线性，没实测）。
- 人工标定：给关节施加阶跃和正弦指令并记录响应；测夹爪开合时间和夹持力。
- 做一个"同一条轨迹在真机和仿真里各跑一遍再对比"的工具。
- 夹爪（含相机）建模：宇树没能提供与实际使用一致的 URDF。
- 待解决的问题：定制本体如果拿不到 URDF、关节参数和精确标定信息，后续迁移怎么做。

**环境匹配**

- 杯子、壶等资产的实物测量（称重、量尺寸）和 Agent 3D 建模。现状和待测清单见 `real2sim/ASSETS.md`。
- 资产在仿真中的物理检验（碰撞、形变、光照）。
- 资产泛化（尺寸、颜色、材质）。
- 制作资产清单（来源可以是开源物体库或 Agent 生成）。

**泛化场景**

- 域随机化：灯光、桌面、地面、背景、相机扰动、干扰物（`utils/pipeline_utils.py`）。
- 建立分级泛化评测集。

**遥操模式**

- [x] 采样频率对齐：仿真动作频率改为 30 Hz，与真机相机 30 fps 一致（`dt = 1/300 s`、`collect_freq: 30`、`render_interval: 10`）。
- [ ] 仿真轨迹录制器：产出与真机遥操数据同格式的数据。

## 月度目标

### 一、环境建模

**场景**：做 3 个数字孪生。验收标准：

- 桌高、桌沿相对机器人的位置用卷尺实测，误差 < 1 cm；
- 用标定好的头部相机把仿真渲染和真机照片叠图，桌沿和物体轮廓偏差 < 5 px，每个场景至少 3 个视角；
- 灯的类型、位置和大致亮度按现场记录建模。

**实物测量**：选 15–20 件真实物体（壶、杯、瓶、碗，加几个常见干扰物），每件要有：卡尺测的关键尺寸（±1 mm）、称重（±5 g）、多角度照片、斜面滑动法粗测的摩擦系数。

**建模**：

- 规则物体（杯、壶、瓶这类旋转体或方体）由 Agent 写参数化脚本生成，参照 `make_pitcher.py`，改参数就能批量出变体，直接服务泛化。
- 不规则物体（玩偶、纸巾包）用摄影测量或 3D 生成模型，再人工修碰撞体。
- 验收：尺寸误差 < 2 mm 或 2%，质量直接用实测值。

**物理检验**：把 `check_object.py` 扩展成标准检验流程，每个资产都要通过：

| 检验项 | 判据 |
| :-- | :-- |
| 落地静置 | 1 s 内稳定，倾斜 < 1°，速度 < 5 mm/s |
| 碰撞体 | 与桌面、其他物体初始无穿插；容器口没被碰撞体封住（参照 `real2sim/scenes/real_scene1_office/check_scene.py` 的射线检查） |
| 抓取 | Dex1 用脚本抓起，保持 3 s 不滑落 |
| 盛液（仅容器） | 静置时液体保留率 ≥ 99%（参照 `check_pitcher_fluid.py`） |
| 光照渲染 | 3 种灯光预设下截图，无穿透、黑面、折射错误；人工复核 |

形变现阶段只做刚体；纸杯这类软体下一轮规划再接 deformable 模拟。

### 二、资产泛化

以倒水任务为例，物体按角色分 3 类：

| 类别 | 实例数 | 来源 | 泛化维度 |
| :-- | :-- | :-- | :-- |
| 源容器（壶、瓶） | ≥ 12 | ≥ 3 件实物孪生 + 参数化变体 | 尺寸 ±20%（夹爪要能抓住），颜色 ≥ 4 种，材质 ≥ 3 种（玻璃、塑料、陶瓷、金属） |
| 接收容器（杯） | ≥ 15 | ≥ 5 件实物孪生 + 参数化变体 + RoboDojo 现有 mug、goblet | 同上，另加口径和高度变化 |
| 干扰物 | ≥ 30 | 复用 RoboDojo `Clutter` 库 + 实物孪生 | 只要求能稳定放置 |

每个类别按 7:3 划分训练实例和保留实例，保留实例不进训练，专用于泛化评测。

### 三、分级泛化评测集

每一级只改变一个因素，其余与 L0 完全相同，掉点可以明确归因。评测布局预先生成为固定 JSON（与现有 `Eval_Layout` 一致），不在运行时随机，保证可复现。

| 级别 | 变化因素 | 布局数（拟定） | 范围 |
| :-- | :-- | :-- | :-- |
| L0 孪生 | 无，与真机评测一一对应 | 25 | 真实工位、真实物体、打印摆放垫上的位置 |
| L1 位置 | 物体位置和朝向 | 50 | 可达范围内（y≈[-0.55, -0.25]），旋转 ±30° |
| L2 外观 | 灯光、纹理、相机 | 50 | 亮度 ×0.5–2，色温 3000–6500 K，灯位置；桌面/地面/背景纹理；头部相机 ±1 cm / ±2°，腕部相机 ±0.5 cm / ±2° |
| L3 物体 | 保留实例和干扰物 | 50 | 没见过的尺寸、颜色、材质，加 3–8 个干扰物 |
| L4 场景 | 没见过的场景 | 50 | 2 个不进训练的场景（如厨房台面、窄桌），桌高 ±5 cm |

每个任务约 225 个布局。场景总数目标 7 个：3 个孪生 + 2 个训练用泛化场景 + 2 个 L4 保留场景；后 4 个用 RoboDojo 现有房间加程序化生成的不同高度、材质的桌子，不需要实拍。

**可行性过滤**：每个布局先用脚本专家跑一遍，专家做不成的布局（够不到、被遮挡、物体不稳）剔除或修正。目标是 L0–L3 剩余布局上专家成功率 ≥ 80%，否则评测集测的是布局问题而不是策略泛化能力。

## 路线：仿真生成轨迹，零样本上真机

**1. 差距在哪里**

`scripted_demo.py` 本质上就是 coding agent 写的运动轨迹。和目标的差距不在"能不能生成"，而在三点：

- **规模和多样性**：目标 500 条，覆盖不同布局、物体、外观。
- **运动合理性**：现在的路点靠手工调参，没有质量指标。
- **控制对齐**：生成的轨迹能否在真机上原样执行，还没有验证。

**2. 控制对齐是关键**

仿真执行每个动作时会自己插值：10 个物理步里前 80% 线性插值到目标，其余保持不动，30 Hz 动作因此被平滑成阶梯式运动。真机控制器（宇树关节 PD 加通信延迟）几乎不可能是同样的行为，不对齐的话仿真里成功的轨迹到真机上会变形。需要对齐：

- 动作语义：目标关节角，30 Hz（已完成）。
- 底层跟踪：仿真 PD 参数按真机阶跃响应拟合，并加入真机指令延迟。
- 限幅：关节速度、加速度上限按真机设置；夹爪开合速度和夹持力按实测。
- 中间插值：真机怎么处理就在仿真里怎么复现，或者两边都去掉。

**训练前的关卡**：把仿真生成的轨迹直接在真机上开环回放。物体按同一布局摆好（用打印的摆放垫），把仿真录下的关节轨迹原样发给真机。开环回放能成功，说明控制已经对齐，再训练才有意义。

**3. 把"运动合理性"变成 agent 能读懂的反馈**

coding agent 不能只看仿真成功率，否则会钻仿真的空子（利用穿模、以真机做不到的速度甩动）。agent 每改一版生成代码，就自动跑一批布局，返回以下指标和视频：

| 类别 | 指标 |
| :-- | :-- |
| 可执行性 | 关节速度、加速度、加加速度是否超过真机限值；是否接近关节极限、奇异位形 |
| 安全 | 与桌面、自身的最小间距；接触力峰值；物体是否滑脱 |
| 自然度 | 与真机遥操轨迹统计的差距：速度分布、任务时长、接近物体的方向 |
| 结果 | 仿真成功率；按失败类型统计 |

agent 照着反馈迭代，优化目标就是"真机可执行"而不只是仿真成功率。这也是 RSI（自我迭代改进）的一个可落地版本：被迭代改进的是轨迹生成器本身。

**4. 对"仿真完全替代真机"保持可证伪**

设一组对照：只用仿真数据（500 条）zero-shot 上真机，对比仿真数据（500 条）加少量真机遥操数据（如 50 条）。两组差距很小，才能说"可以替代"。

第一个验证任务用 `g1d_general_pickup` 或 `g1d_stack_blocks` 这类刚体任务，不用倒水：流体只能走 CPU 状态管线，生成 500 条很慢；而且真实流体动力学会额外引入差距，失败时分不清是控制没对齐、视觉差距还是流体不准。

## 已知问题

- 倒豆脚本演示只在部分 seed 成功：壶在右侧时右手倒壶够不到杯口，或壶身撞上持杯手；杯对杯任务目标杯偏远时右手够不到。同一配置多次运行结果也会不同（仿真有随机性）。
- G1 手臂在桌面高度的可达范围约 0.45 m，RoboDojo 原有任务中部分物体够不到；自有任务的物体应放在 y≈[-0.55, -0.25]。
- 末端位姿（ee）动作还不支持，需要 G1-D 的 cuRobo 配置，并让 coupled 左右臂各用一个 planner。

更完整的状态、决策和验证记录见 [`HANDOFF.md`](HANDOFF.md)。

## 致谢

本项目基于 [RoboDojo](https://github.com/RoboDojo-Benchmark/RoboDojo)（[论文](https://arxiv.org/abs/2607.04434) | [文档](https://robodojo-benchmark.com/doc/)）和 [XPolicyLab](https://github.com/XPolicyLab/XPolicyLab)，并使用了 [Isaac Sim](https://developer.nvidia.com/isaac/sim)、[IsaacLab](https://github.com/isaac-sim/IsaacLab)、[cuRobo](https://github.com/NVlabs/curobo)。

```bibtex
@article{chen2026robodojo,
  title={{RoboDojo}: A Unified Sim-and-Real Benchmark for Comprehensive Evaluation of Generalist Robot Manipulation Policies},
  author={Chen, Tianxing and Chen, Yue and Li, Zixuan and Tang, Junyuan and Su, Kailun and Wan, Weijie and Chen, Baijun and Lu, Haoran and Yan, Haowen and Su, Honghao and others},
  journal={arXiv preprint arXiv:2607.04434},
  year={2026}
}

@article{community2026xpolicylab,
  title={{XPolicyLab}: A Unified Standard and Open Ecosystem for Robot Policy Evaluation and Deployment},
  author={Community, XPolicyLab and Chen, Tianxing and Chen, Yue and Nian, Tian and Cai, Zijian and Chen, Guangyu and Lin, Wenwei and Liang, Qiwei and Xiang, Peicheng and Su, Kailun and others},
  journal={arXiv preprint arXiv:2608.09892},
  year={2026}
}
```

## 许可证

沿用 RoboDojo 的 [MIT License](LICENSE)。
