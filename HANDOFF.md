# Project Handoff

更新时间：2026-10-19
当前分支：main（唯一分支）
当前目标：在 RoboDojo 上接入 Unitree G1-D（Dex1-1 夹爪）做桌面操作仿真，后续加入自有物体和任务

## 项目状态

- 本仓库基于 RoboDojo（Isaac Sim 5.1 / IsaacLab 2.3）：`origin` = Zheng-Chong/RoboRSR（原 Zheng-Chong/RoboDojo，已改名；GitHub 上当前为公开仓库），`upstream` = RoboDojo-Benchmark/RoboDojo。2026-10-16 起只保留 `main`（`chongzheng/env-g1d-dex1` 已快进合并后删除，GitHub 默认分支改为 `main`）。
- 已接入机器人 `g1d`：一个 articulation、两条 7 自由度手臂（coupled 模式）+ Dex1 两指夹爪；轮子锁死、升降柱和腰部关节保持初始值（只做桌面操作）。
- 运行：`bash scripts/robodojo.sh eval ... --env-cfg g1d --action-type joint`。动作维度 `_robot_info.json`：arm [7, 7]，ee [1, 1]。
- 资产分两处：`real2sim/`（入库，二进制走 Git LFS）放不能重新生成的自建内容（`robot/g1d_v4` URDF、`scenes/real_scene{1_office,2_counter,3_lab}`、`scene3_pour_pack`）；`Assets/`（不入库，`scripts/init_assets.sh` 下载，是指向 `.cache/robodojo_assets_repo/Assets` 的软链接）放官方资产和生成物。`python scripts/g1d/build_asset.py`（`--v4` 默认 `real2sim/robot/g1d_v4`）嫁接 G1 头部 D435 坐标系，写入 `Assets/Robots/g1d/`；安装 3 个场景到 `Assets/Room/real_scene{1,2,3}/`，并按 `SCENES` 为每个场景配置（`g1d` 办公室、`g1d_counter` 吧台、`g1d_lab` 实验室）生成 `Assets/Eval_Layout/RoboDojo/<配置>/` 的 G1-D 任务布局（`g1d/` 另含去掉相机支架的 arx_x5 布局）。完整顺序见 README「快速开始」。
- 下载官方资产后必须运行一次 `python utils/update_embodiment_config_path.py` 生成 `Assets/Robots/*/curobo.yml`，否则 X5 任务启动即报错。
- 服务器环境（Host 别名 `21196`）：仿真环境 `RoboDojo`、策略环境 `demo-policy`（XPolicyLab 需 websockets>=14，Isaac Sim 需 websockets==12，所以分开）。激活方式见本地 `.agent/remote-validation.local.toml`（不入库）。

## 最近任务

### 量杯按实测尺寸重建（2026-10-19）

- 用户实测（两只量杯同截面）：壁厚 2 mm，杯口 75 mm、杯底 63 mm（对边宽），高杯 170 mm / 60 g，矮杯 140 mm / 40 g；俯视照片：杯底为倒角方形（圆角八边形），杯口为大圆角方形，杯口有卡盖带和外凸翻边。
- 新增 `scripts/objects/measuring_cup_shape.py`（只依赖 numpy）：尺寸、截面、内外壁环、`origin_shift`、`in_cavity`。`make_plastic_cups.py` 的量杯改为从它放样：杯身为杯底八边形到翻边下圆角方形的直纹面（对边 63→68.4 mm），翻边 75 mm × 2 mm，卡盖带 70 mm，翻边下沿到杯口 12.5 mm；挂片移到翻边上（仍在 +X 面，照片里在角上，未改，免得影响 45° 侧抓）。75 mm 理解为含翻边的最大外宽；杯身中部那圈细台阶未建模。
- 采集判据 `collect_office_beans.in_cavity` 改为调用共享模块，不再手抄超椭圆尺寸；`check_cup_beans.py` 填充区改用杯底八边形内壁；`build_asset.py` 的 `MEASURING_FLUID_Z` -0.0427→-0.0487（豆堆离杯底高度不变，杯高 158→170 mm）及注释；`scripted_demo.py` 注释；`real2sim/ASSETS.md` 量杯行。
- 结果（本机 numpy/scipy，pxr 打桩）：外形 82 × 75 × 140 / 170 mm（含挂片），满口 512 / 622 ml，400 ml 刻度在 112 mm（80%）、500 ml 在 140 mm（82%）；每只 65 个碰撞凸包、实际顶点最多 24（GPU 上限 64）；`self_check` 不侵入内腔，bbox 与 `origin_shift` 一致；截面图 `output/measuring_cup/measuring_cup_check.png`（不入库）。`test_office_beans` 6 项通过（新增杯底倒角处在外、杯口同位置在内的用例），ruff check/format、task_inventory --check、git diff --check 通过。
- 服务器验证（21196，GPU 4，隔离目录 `/data2/tmp/robodojo/agent-remote-runs/mcups-measured-20261019`，同步的是当前工作区，含其他会话的马克杯/红豆改动；资产生成到该目录 `gen/Rigid`，共享资产库未改动）：`make_plastic_cups.py` 退出码 0，metadata 尺寸 0.082 × 0.075 × 0.14 / 0.17 m，质量 0.04 / 0.06 kg。`check_object.py` 两只量杯都是 OK：z 0.0700 / 0.0850（等于预期），倾斜 0°，速度 0.0007 / 0.0014 m/s。`check_cup_beans.py --device cpu`：量杯 1350 / 1620 颗 5.6 mm 豆子，100% 在杯内，沉降 0.62 / 0.93 mm（>0.5，判据通过）；截图确认八边形杯底、翻边、挂片，豆子不穿壁。脚本退出码为 1，原因只在马克杯：沉降 0.33 mm，低于 0.5 mm 阈值（100% 在杯内）。这是其他会话的马克杯/5.6 mm 豆改动导致的，不是量杯问题，尚未处理。截图在本机 `output/measuring_cup/server/`（不入库）。
- 未做：未跑任务或 `scripted_demo`。风险：杯更高、翻边更宽可能影响 `scripted_demo` 侧抓与倒豆可达性。正在运行的 100 条采集用的是旧量杯，需重建资产（`make_plastic_cups.py` → `format.py` → `build_asset.py`）后重跑。
- 本次只提交量杯部分：`make_plastic_cups.py` 中的马克杯实测（`MUG`、描述文字）、`ASSETS.md` 马克杯行、`export_office_beans.py` 线程参数是其他会话的未提交改动，留在工作区未纳入；服务器 check_cup_beans 中马克杯沉降 0.33 mm 未过阈值，也留给那边处理。

### 塑料马克杯按实测尺寸重建（2026-10-19）

- 用户给出 `plastic_mug/00000` 实测：重 89 g、杯壁 2 mm、高 100 mm、无斜度直筒、内径 80 mm；把手高 70 mm、厚 5 mm、宽 35 mm。改 `scripts/objects/make_plastic_cups.py` 的 `MUG`：R0=R1=42 mm、T_WALL 2 mm、H 100 mm、MASS 0.089；把手高/宽按外沿算（中心线 z 17.5–82.5 mm，外伸 32.5 mm + 半厚），"宽 3.5 cm" 理解为把手离杯壁外伸距离，把手条宽（沿 Y）仍为估计的 13 mm，杯底厚仍估计 4 mm。描述文字同步改为直筒、约 10 cm 高；`real2sim/ASSETS.md` 对应行改为实测。
- 验证：本机 numpy/scipy（stub 掉 pxr）构建几何：外形 119 × 84 × 100 mm，把手外伸 35.0、高 70.0、条宽 13.0 mm，容积约 482 ml，44 个碰撞凸包，`self_check` 不侵入内腔。5090-2 隔离目录 `/data/robodojo-runs/mug-render-20261019-171418`（Assets 软链到采集运行目录、只读）生成 USDZ 到 `ovl/Rigid/`（metadata：0.089 kg、0.119 × 0.084 × 0.1 m），GPU 7 跑 `check_object.py`：z=0.0500（符合预期）、倾斜 0°、速度 0.0002 m/s；渲染图在本机 `output/plastic_mug/`（不入库）。未跑 `check_cup_beans.py`，共享资产库（21196）未安装该杯。
- 教训：首次提交 `b077d1d` 与另一会话共用 git 暂存区发生竞态，实际提交进去的是红豆改动（后被 amend 为 `9bfdd78`），马克杯改动当时没有入库，第一次服务器渲染因此用了旧模型。并行会话同一工作区提交时用临时索引（`GIT_INDEX_FILE` + `commit-tree` + `update-ref` 带旧值校验）。
- 数采平台（5090-2）：`real2sim/objects/Rigid/plastic_mug/00000/preview/sim.png` 换成新渲染；资产元数据镜像 `/root/RoboDojo_assets/Assets/Object/RoboDojo/Rigid/plastic_mug/00000/{metadata,description}.json` 手动放入新生成的文件（21196 共享库没有这只杯，下次整体重跑镜像命令会把它删掉，需重新放或先装进共享库）。
- 平台孪生记录（2026-10-19，用户指出没标）：在 5090-2 平台数据库里给 `Rigid/plastic_mug/00000` 写入实测尺寸 119 × 84 × 100 mm（长 = 外径 84 + 把手外伸 35）、89 g、建模方式“参数化脚本”和备注，`_by` 为 Claude；写前备份 `.cache/real2sim_platform.sqlite3.bak-mug-20261019-172609`。仿真尺寸/质量与实测误差为 0。两只量杯在平台上的孪生勾选和实测值还没填。

### 红小豆实测尺寸与质量（2026-10-19）

- 用户给出实测值：长宽高 7 × 5 × 5 mm，单粒约 0.1 g。`make_red_bean.py` 网格改为 7 × 5 × 5 mm（numpy 采样包围盒核对为 7.0/5.0/5.0）、质量 1e-4 kg；`build_asset.py` 的 `RED_BEAN_PHYSICS.particle_mass` 由 6e-5 改为 1e-4；`check_cup_beans.py` 绘制缩放 0.85→0.7，画出的豆子大小不变；`real2sim/ASSETS.md` 同步。
- 用户随后确认 PBD 颗粒也改为等体积 5.6 mm 球：`RED_BEAN_PHYSICS` 的 rest/solid_rest_offset 0.002→0.0028，contact 偏移 0.0024→0.0034（保持 1.2 倍），fluid_rest_offset 0.0012→0.0017；`CUP_FLUID_Z`、`MEASURING_FLUID_Z` 各上移 0.8 mm，最低一层颗粒与杯底的间隙和 4 mm 时一致；两个任务的 `scatter_connect_radius` 0.009→0.012（约 2 个颗粒间距）；`check_cup_beans.py` 绘制缩放 0.8；600 Hz 步长的注释核对过（每步 4.17 mm，小于 5.6 + 2 mm）。
- 预期影响（均未经仿真验证）：量杯约 95 ml 豆量从 1840 颗降到约 670 颗；13 mm 壶嘴之前实测会被 5 mm 颗粒卡住，`g1d_pour_water_into_cup` 预计倒不出来。`collect_office_beans.py` 的杯内判据只看颗粒中心，不受影响。正在运行的 100 条采集使用旧的 4 mm 资产，与新参数不一致；需重新运行 `build_asset.py` 后再复跑。
- 只跑了 ruff check/format、py_compile、task_inventory --check、`git diff --check`；本机没有 scipy，`test_office_beans` 未运行。同一工作区里另一会话的量杯/马克杯实测改动（`make_plastic_cups.py`、`measuring_cup_shape.py` 等，以及 `build_asset.py` 中量杯注释和 `MEASURING_FLUID_Z` 的 -0.0487）未纳入本次提交；本提交中 `MEASURING_FLUID_Z` 为旧 158 mm 量杯的 -0.0427。
- 装库（用户要求）：21196 隔离目录 `agent-remote-runs/redbean-20261019-171859`（9bfdd78 的代码，不含 `real2sim/`）在 GPU 4 上运行 `make_red_bean.py ovl/Rigid`，metadata 为 size [0.007, 0.005, 0.005]、mass 1e-4；原样复制到共享库 `Object/RoboDojo/Rigid/red_bean/00000`（原先不存在，没有覆盖任何文件；未跑 `format.py`，运行时不读 `uuid_map.json`），`diff -r` 一致。`check_object.py --image-dir` 渲染出新的 `preview/sim.png`，已同步到本机和 `5090-2` 的 `real2sim/objects/`。落地检查报 FAIL（z 2.1 mm，期望 2.5 mm；倾角 47.8°）：5 × 5 mm 截面是圆的，豆子会绕长轴滚动，种脐转到上方，所以倾角判据对它不适用；另外中心比期望低 0.4 mm，没有再查。之后重新镜像元数据到 `5090-2`，并在平台同步。

### 进行中：办公室量杯全量倒红豆 100 条 LeRobot 数据（2026-10-19）

- 用户要求在 `5090-2` 的办公室场景用两个量杯，每次把所有豆子从高杯倒入矮杯，生成 100 条位置泛化数据；交付格式已明确为 LeRobot。
- 起点 `ff92a92a0a4e8422fc91475c452cbdd33945ba13`，本机工作区原干净，`git pull --ff-only` 无更新。服务器只有数采平台和元数据资产镜像，没有仿真环境；已在独立运行目录开始安装环境，并复制此前验证过的必要资产，不改平台或共享资产库。
- 新采集入口 `scripts/g1d/collect_office_beans.py`：复用原双臂脚本，将离杯目标设为 100%、上限 1200 步；保存 30 Hz 三路 RGB、动作前实测关节状态、实际指令和逐帧豆数/豆位/杯位姿。用真实锥形超椭圆内腔检查粒子中心，零残留/零散落/总数守恒，回正（沿用原任务 30° 定义）后连续保持 30 帧才合格。采集接管旧奖励提前结束逻辑；导出复核轨迹末 30 帧计数。原任务宽松奖励不改，也不用于本批验收。
- `prepare_office_beans_layouts.py`：四维 Latin hypercube，在已有可达范围内生成不同杯位。`export_office_beans.py`：只导出足数且位置不同的合格回合，检查视频帧数，用官方 LeRobot v3 API 写入并回读。说明在 `scripts/g1d/OFFICE_BEANS.md`。
- 已运行：本机和服务器 CPU 判据测试 6 项、ruff/语法和 task_inventory；doctor（跳过 Isaac 导入）13 pass / 0 fail、量杯 smoke dry-run；必要资产与来源 SHA-256 一致；400 个候选布局已生成；CUDA、Vulkan 和真实仿真启动通过。独立合成夹具的 LeRobot 0.4.4 / TorchCodec 0.4.0 / H.264 写入及三视角回读通过（不计入机器人数据）。
- 物理对照：原 300 Hz，1840 颗全部离开高杯，1838 入矮杯、2 颗在杯底正下方桌面；采集专用 `g1d_beans_collect` / `g1d_beans_collect_sim` 改为 600 Hz，1840/1840 入矮杯，零散落持续 2 秒。额外的 0.02 m/s 最大粒子速度筛选会因 PBD 杯内微振动拒绝成功转移，已取消；原额外 5° 回正筛选改为任务既有的 30°。用户要求的 100% 转移条件不变。两个对照回合保留为诊断，尚未计入正式数据。
- 正式首条严格合格：1840/1840 入杯，468 帧，最后 30 帧从逐颗位置/杯位姿独立重算通过；真实回合的 LeRobot 导出及三视角回读通过。完整 doctor 14 pass / 0 fail。
- 正式批量正在运行：400 个候选拆为 seed 100–107，每组 50；先单进程，首条严格合格后最多双进程；全局足 100 条后以 stop-file 完成当前回合并停止，再自动导出和回读。协调脚本、日志与运行位置只记在本机忽略目录 `.agent/remote-runs/office-beans/state.json`。本次提交包含通过验证的采集实现与交接记录，最终数据数量待批量结果确认。
- 多 GPU 扩展：发现 AppLauncher 添加 `--device` 前的预解析会把它误当作 `--device_id`，`main.py` 禁用参数缩写后恢复显式选卡；采集入口使用单卡渲染，普通 scripted_demo 默认不变。GPU 1 上实际渲染/物理及 1840/1840 入杯验收通过。准备把剩余 seed 分配到独立 GPU，保留已完成回合；采集实现首个提交 `4271c057601e414c5331524233ad6feb9641d855` 已推送。
- 实验记录已创建并回读：[办公室量杯全量倒豆](https://app.notion.com/p/3fec96063117819e914ff0a4cea9e54f)。下一步看批量 `status.json` 与回合 `quality.json`，核对位置覆盖和逐颗豆数，等待足数后的 LeRobot 导出/回读，再提交、推送并完成实验记录；不得降低全量转移阈值。

### 目标
- 在服务器搭好 RoboDojo 环境并跑通基线；接入 G1-D + Dex1 并验证。

### 已完成
- 环境：Isaac Sim 5.1、IsaacLab、cuRobo、41 GB 资产安装完成；`robodojo.sh doctor` 14 项通过。
- G1-D 接入：`env/robot_manager/robot_config/g1d.py`（直接加载 URDF，`merge_fixed_joints=False` 保留相机/末端坐标系）、`robot_class/g1d.py`（G1DLeft/G1DRight）、注册表、`env_cfg/g1d.yml`、`env_cfg/robot/g1d.yml`、`scripts/g1d/`。
- `robot_class/x5.py` 增加可选类属性 `config_key`，子类可读 `<key>_robot_config.yml`；X5 行为不变。
- 相机：头部双目挂在 `head_link` 上，宇树合同位姿 (0.0442, ±0.0298, 0.4900)、俯 47.6°（镜头在头壳内，头壳照常渲染，靠 10 cm 近裁剪面隐去），类型 `g1d_head`（棋盘格 fx 279.5，约 98°）；腕部在 `*_dex1_base_link` 的 (0.0627, 0, 0.0555)、沿手指俯 30°（宇树合同值），类型 `dex1_rgb`（棋盘格 fx 314，约 91°，近裁剪 3 cm）。依据见 2026-10-16 相机一节。
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

## 双臂倒水：吧台场景 + 自建水杯（2026-10-15，2026-10-16 随杯对杯任务一起提交）

- 吧台 real2sim 场景 `real2sim/scenes/real_scene2_counter/`：两张照片联合拟合，`build_scene.py` 生成 `real_scene2.usd`；水龙头鹅颈为连续扫掠管（`swept_tube`）。服务器隔离目录 `/data2/tmp/robodojo/agent-remote-runs/counter-pour-20261014-163053` 以 Room=real_scene2 跑倒水（机器人根 y=-0.80、升降柱各 0.075，物体整体 y-0.07、z+0.15）；隔离目录的 `Assets/Object` 为叠加目录（`prep_water_cup.py`），共享资产库未改。
- 新增 `scripts/objects/make_water_cup.py`：按 `real2sim/scene3_pour_pack/scene_spec.json` 的接水杯生成 `Rigid/water_cup/00000`（白色搪瓷锥形杯，无把手，高 0.118 m、口径 73 mm、底径 55 mm、0.165 kg、约 300 ml；杯底 + 32 片杯壁凸包，中空）。`check_object.py` 通过（落地 z=0.059、倾角 0）。
- `build_asset.py` 的 `swap_bottle_for_pitcher` 把马克杯换成 `water_cup`（仍前移 `POUR_CUP_PULL=-0.09`，与壶间隙 < `POUR_MIN_GAP` 的布局丢弃），隔离目录重建后 153 个倒水布局；`g1d_pour_water_into_cup.yml` 的杯子类别同步为 `water_cup`。step_lim 480 → 600（用户确认）。
- `scripted_demo.py` 倒水：离壶近的手臂抓壶把手，另一臂侧握杯身——杯底以上 9 cm（杯沿下 2.8 cm）、手指下倾 35°、手腕内转 45°，沿手指方向斜着滑入后夹紧，杯口不被遮挡；杯子抬 1.5 cm（4 cm 时壶手倾倒姿态差 7.7 cm 够不到，回正时壶一跳把豆子洒出）；壶嘴落在离杯心 2.5 cm（近侧杯沿内）；倾倒时按“实际壶嘴 vs 实际杯口”闭环修正。
- 可达性探测（隔离目录 `scripts/g1d/probe.py`，不入库，seed 0 杯位）：手指水平在杯底以上 6–10 cm、任何手腕转角都差 6–14 cm；下倾 35°、内转 30–60°、10 cm 高误差 1.1–1.4 cm。
- 验证（隔离目录，各 1 轮，退出码 0，eval_time=1）：**seed 1 成功**（出壶 27%、在杯 26%，比例约 0.96），视频 `output/counter_pour_water_cup/seed1_grid.mp4`。seed 0（壶在右、右手倒）失败：抓壶、抓杯都成功，但倾倒时右手 IK 差约 9 cm，在杯 9%/出壶 26%；把杯子带近到 y≈-0.39 后够得到，但壶倾到 70–100° 时撞上左边持杯手脱落（该改动已撤回）。
- 历史结论（已撤回的尝试）：斜上方抓马克杯时手掌盖住杯口，倾杯迎流 15–25° 和胸前空中对倒（抬 8–20 cm）都撞手或壶手够不到；马克杯版本入杯比例卡在 0.85–0.89。
- ruff check/format 通过（服务器 ruff 0.17）；未跑 task_inventory / doctor（未改任务注册）。

## 官方 G1-D URDF 与真机三路相机对照、腕部相机拟合（2026-10-16）

- 真机（ssh `unitree`，Jetson）上的官方模型：`/unitree/module/unitree_eai/xr_teleoperate/assets/g1_d/g1_d.urdf`（Dex3 手、无相机 link）和 `.../dex1_1_service/urdf/dex1_1.urdf`（无相机）。`~/g1d_infra_ewam/assets/g1_D/g1_d.urdf` 实为 G1 人形（robot name="g1"），不要用。已拷到本机 `.cache/unitree_official/`（gitignored）。
- 与仿真 `g1d_dex1.urdf` 对比：24 个公共关节原点/轴全部一致；唯一差别是 v4 把腰部两个关节名互换了——官方 `torso_Joint` = 腰俯仰（Pitching_Link→Yaw_Link，绕 y），`Yaw_Joint` = 腰偏航（Yaw_Link→torso_link，绕 z）。回放真机关节状态时要按官方名映射。
- 官方没有相机外参/内参。唯一来源是 `~/g1d_infra/teleop/utils/ego_camera_config.py` 抄的服务文档值：头部双目（Cherry Dual Camera，1280×480 SBS），左右眼 ±29.8 mm，FOV 115°×80°，光轴俯 47.6°（= unitree_ros G1 `d435_joint` 俯仰 0.8308 rad）。仿真 `cam_head` 位姿已按此设置。
- 对照方法：真机运行记录 `~/Desktop/Results/G1/g1d_real_run_20260921/`（`observations.jsonl` 每帧 16 维关节状态，夹爪 5.4=全开、0=全闭）。一次性脚本 `21196:/data2/tmp/robodojo/agent-remote-runs/camcheck-20261016-104337/render_real_frame.py`（不入库）在 21196 隔离目录 `camcheck-20261016-104337` 把仿真双臂开到第 5、11 帧关节角（误差 <0.001°），用 `g1d_general_pickup` seed 0（real_scene1）渲染全部相机，并在进程内追加 115° 头部和 dex1_rgb 腕部对照相机。并排图：本机 `output/cam_check/f{5,11}/compare_f*.jpg`。
- 结论：
  1. **真机录制里腕部两路是反的**：`wrist_left` 流 = 右手相机，`wrist_right` 流 = 左手相机（两帧都由物体相对位置确认）。已知问题：这台 Jetson 就是图像服务主机 192.168.123.164，`teleimager.service` 读 `~/unitree_eai_environment/service/teleimager/cam_config_server.yaml`，按序列号绑定 `left_wrist_camera`=JR0001（USB 1-3.2）、`right_wrist_camera`=JR0002（1-3.3）；`~/client/jepa_g1d_client/g1d_jepa_client.py` 记录 2026-09-25 真机遮挡测试：遮物理左腕相机时 `cam_right_wrist` 变黑，即 JR0001 装在右手；采集数据集（capybara_to_box、pick_3objects_to_basket）左右是对的，所以部署客户端默认 `SWAP_WRISTS=True`。09-21 这次运行早于该修正。sim2real 时：图像服务的 `left_wrist_camera` 对应仿真右腕。
  - 硬件：头部 Sunplus `1bcf:2d4f`（"USB2.0 Camera RGB"，序列号 01.00.00），原生每眼 1920×1200（16:10），服务用 1280×480 SBS（每眼 640×480，4:3，是裁剪还是缩放未确认）；腕部 JR0001/JR0002（VID/PID 0001:0001 / 0002:0002，最高 1920×1080，服务用 640×480）。UVC 描述符里没有焦距（wObjectiveFocalLength=0），设备内无可读标定；相机被 root 服务独占，读控制量/切换模式需要停服务（需 sudo 密码）。
  2. **棋盘格内参（真值来源）**：`rtx5090-180:/root/unitree_sim_isaaclab/real2sim_calib_20261015/real_data/20261016/intrinsics/camera_intrinsics_{head_left,head_right,left_wrist,right_wrist}.json`（OpenCV 模型，重投影 0.44–0.51 px）。头 fx≈279.5（约 98°×81°），腕 fx≈313/316（约 91°×75°）；畸变小（边缘 <2%），仿真用针孔。腕部 cx 一个 337、一个 317：按"标定文件名 = 图像服务流名"（仿真左手 ← `right_wrist`）对应时轮廓误差 2.5/0.4 px，反过来 10.2/4.3 px，即标定的腕部命名也跟着图像服务互换。同目录的 `camera_contract_v5.json`（宇树服务台 031354）给了官方外参：腕部在 `dex1_base_link` (0.0627, 0, 0.0555)、俯 30°；头部 FOV 仍是文档值 115°，不可信。
  3. **相机配置（未提交）**：一次性脚本 `21196:/data2/tmp/robodojo/agent-remote-runs/camcheck-20261016-104337/fit_calib.py`（不入库）固定标定内参、只拟合位姿。头部：现位姿 5.9/7.3 px，合同位姿 6.1/9.3 px，拟合到约 5–6 px，但位姿在 ±3 cm/±4° 内定不住（只有夹爪轮廓），保持现位姿，只换 fx。腕部：合同位姿 + 标定内参不拟合就 2.5/6.9 px（左）、0.4/6.6 px（右），拟合只移 2–5 mm/1–2°，直接用合同值。**之前按轮廓自由拟合出的 68°/z=0.081/40.8° 是错的**（夹爪开度不变，焦距与距离耦合），已作废。改动：`env_cfg/camera/template.py` 新增 `G1D_HEAD`（fx 279.5，`camera_manager.py` 注册为 `g1d_head`，不动共享的 `Gemini_345Lg`），`DEX1_RGB` 改为 fx 314、近裁剪 3 cm（夹爪本体在镜头前 22 mm 内，手指从 38 mm 起）；两臂 yml：`cam_head`/`cam_head_right` 用 `g1d_head`，`cam_wrist` 用合同位姿 `pos [0.0627, 0, 0.0555]`、`ori [0.6124, 0.3536, -0.3536, -0.6124]`。Isaac 渲染（隔离目录，进程内读仓库 yml，旧资产网格）：手指压在画面两下角、指尖约一半高度，与真机一致；对比图本机 `output/cam_check/v4_f{5,11}/compare_f*.jpg`。
  4. **共享资产同步与事故（16:14）**：用户同意后在共享库跑了当时的 `build_asset.py`：`Robots/g1d` 已按当前代码重建（新相机配置 + `strip_wrist_camera` 网格；与 17:38 隔离构建逐字节一致）。但 `build_layouts` 先删掉全部 `g1d_*_*.json`，又因共享库缺 `water_cup` 中途退出，G1-D 任务布局被清空；随即用 `52b3d8c` 版 `build_layouts` 恢复（3 任务 396 个，仅依赖已有的 `pitcher`），恢复后 scripted_demo seed 0 的 `g1d_general_pickup`、`g1d_stack_blocks` 均 success_rate=1.0（隔离目录 `camsync-20261016-161434`）。之后 17:42 另一会话按新代码重装布局（见「三个场景」一节）。教训：在共享库跑 `build_asset.py` 前先确认它读的物体都在，或用叠加目录。
  5. **资产重组后重渲（2026-10-16 晚）**：隔离目录 `camrefactor-20261016-172408`，叠加 Assets（只读部分软链共享库，`Robots/g1d`、`Room/real_scene*`、`Eval_Layout/RoboDojo/g1d*`、`Material/Fluid`、`water_cup`/`plastic_cup` 为本地生成；不跑 `format.py`，只生成 `uuid_map.json`）。按 README 生成杯子并跑 `build_asset.py`（`--v4` 默认 `real2sim/robot/g1d_v4`），三个场景配置各 574 个布局。办公室场景第 5/11 帧与真机对比和重组前一致；`g1d`/`g1d_counter`/`g1d_lab` 第 11 帧四路相机都正常。图在本机 `output/cam_check/refactor/`（`compare_real_f*.jpg`、`compare_scenes_f11.jpg`，脚本 `output/cam_check/compare.py`）。
  6. **头部改用合同位姿（胸口约束）**：用户指出真机头部画面底部能看到自己的胸口（深灰色弧形；其上带橙灯的白色件是底盘），仿真看不到。之前我把底部误认为底盘，第 2、3 条里"头部位姿保持现状"的结论因此作废。真机胸口上沿在第 5/8/11 帧完全一致，读 7 个点作约束，和夹爪轮廓一起拟合（脚本 `output/cam_check/fit_chest.py`、`chest.py`，在 camcheck 目录跑）：现位姿（躯干系 x=82.7 mm）躯干投影全在画外，胸口误差 63–106 px；**合同位姿（x=48.1 mm，后移 3.5 cm）不拟合，胸口中间 5 点误差 −3 到 +2 px，夹爪 2.3/3.6 px（比现位姿 5.4/6.2 更好）**，再拟合几乎不动（偏航 1.6°、滚转 0.9°），直接用合同值。镜头在头壳网格内：视锥内面罩 0–40 mm、下巴 40–100 mm（仅最底 30 行），胸口从 128 mm 起、手臂 218 mm 起，所以 `G1D_HEAD` 近裁剪 10 cm（最底 1–2 行可能残留一点头壳边）。改动：两眼 `link: head_link`、`pos [0.0442, ±0.0296/−0.0300, 0.4900]`、`ori [0.6593, 0.2557, -0.2557, -0.6593]`。重渲（同一隔离目录，`output/cam_check/refactor2/`）：三个场景头部底部都出现胸口，办公室场景胸口上沿约第 424 行（真机约 435 行）。仿真胸口为金属浅灰，真机偏深灰。
  4. real_scene1 的桌子/沙发相对机器人的位置与这次运行的真实摆放不同，背景不能用来对比。

## 新任务 `g1d_pour_beans_cup_to_cup`：双手各拿一个塑料杯，空中杯对杯倒红豆（2026-10-16）

- 参照真机片段 `~/Desktop/Results/G1/g1d_task_clips/G1D-PourBeansPlusEps222_ep002.mp4`（两只透明塑料杯，双手侧握抬到胸前，一杯倒进另一杯）。新任务，原壶倒杯任务保留。
- 资产：`make_water_cup.py --plastic` 生成 `Rigid/plastic_cup/00000`（高 9.5 cm、口径 72 mm、30 g、OmniGlass 透明）。
- 任务：`task/RoboDojo/{config,tasks}/g1d_pour_beans_cup_to_cup.*`，标签 `source_cup`/`target_cup`/`beans`，step_lim 600；成功 = 源杯回正（`is_axis_up` 上升沿）时 ≥60% 豆子离开源杯且其中 ≥90% 在目标杯里。已加入 `task_inventory.py` 的 custom 列表。
- 布局：`build_asset.py` 的 `two_cups`（G1D_TASKS 条目，从 arx_x5 `pour_liquid_into_cup` 布局生成）。随机数用原布局瓶子位置做种子（只用布局序号的话每个 seed 文件夹的 `_0` 都一样，seed 0/1/2 拿到同一布局）。**源杯固定在机器人左边（左手倒）**，|x| ∈ [0.10, 0.16]，y ∈ [-0.39, -0.31]。
- 脚本（`scripted_demo.py` 的 `cup_to_cup_actions`）：两手同时按 `cup_grasp` 侧握（杯沿下 2 cm，下倾 35°、内转 45°），先竖直提 6 cm 再送到胸前，目标杯底抬 12 cm；等两手误差 <1 cm 后实测杯子在手里的位姿；源杯**绕手指轴滚转（手腕 roll）**倾倒，杯口最低点按实际方向落在目标杯近侧杯沿内 2.4 cm、高 2.5 cm，闭环修正；60° 前 2°/步，之后豆流弱时 1°/步。
- 排查结论（试过、已撤回）：
  - 提杯同时转手腕（内转 45° → 0°）会把杯子从右手拽掉；绕世界 y 轴倾杯时 35° 下倾的手 15° 内就顶到关节限位；手指朝前（yaw 0/20）滚转幅度不够倒空。
  - **右手倒不出来**：每次都是杯子在两指间转动（手指轴与杯轴夹角 125°→160°），倾到 85° 左右豆子流得很慢。日志显示 Dex1 一根手指停在张开限位（q=-0.02），另一根推到 -0.0013，两只手都这样；但两只夹爪是同一个零件、装法相同，所以这根限位手指相对倒的方向，左右手分别在杯子两侧。已试且无效：目标杯迎向源杯倾 20°/35°（更糟，左手也失败）、杯子 static_friction 1.5、夹得更低 5.5/6.5 cm（提杯时掉）、右手绕手指轴翻 180°（够不到）、右手内转 30°/60°。
  - 运行时刚体只读 `physics.static_friction`/`dynamic_friction`（默认 0.6/1.5），metadata 里的 `friction` 不生效。
- 验证（服务器隔离目录 `/data2/tmp/robodojo/agent-remote-runs/c2c-20261016-105635`，Room=real_scene1，各 1 轮，退出码 0，`_result.json` eval_time=1）：**seed 0、seed 2 成功**（源杯倒出 87–89%，目标杯 87–89%）；**seed 1 失败**：目标杯在 (0.159, -0.320)，右手提杯够不到胸前位置（误差恒为 0.107，杯子比计划低 6 cm），只有 59% 进杯。四宫格视频（第三视角 | 头部 / 左腕 | 右腕）在本机 `output/c2c/final/grid_s{0,1,2}.mp4`。
- ruff check/format 通过。没跑 task_inventory --check、doctor。
- 资产与布局：2026-10-16 已装入共享资产库（`Rigid/plastic_cup`、三个场景配置的布局）；本机需按 README 运行 `make_water_cup.py --plastic` 和 `build_asset.py`。

## 实验室操作台 real2sim 场景 real_scene3（2026-10-14）

- `real2sim/scenes/real_scene3_lab/`：用户提供四张围着实验室白色折叠培训桌拍的照片（`reference/p1–p4.jpg`）。流程照搬 real_scene2：`fit_photo.py` 联合拟合四台相机和尺寸，`build_scene.py` 生成 `usd/` 和 Room 入口 `real_scene3.usd`，`check_scene.py`、`preview.py` 做离线检查和预览。详细尺寸、置信度和接入方法见该目录下的 `PLAN.md`。
- 结果：桌面 1.369 × 0.686 m，高 0.749 m，板厚 21 mm。尺度只来自“桌高 0.75 m”这一个先验，长宽与标准 1.4 × 0.7 m 相差约 2%，可以作为交叉验证。挡板在 +Y（底边离地 0.456 m）；两根斜立柱向外倾约 15°，偏向挡板一侧 8.6 cm；走线盒是桌面贴图，没有几何体。桌面贴图取四张照片矫正后的逐像素中值；墙是照片背景板（墙前的工位等被压平到墙上）；地毯从照片按真实尺度矫正。
- 约定：机器人站在没有挡板的 −Y 一侧（这是假设，真机若站在另一侧就把 Room 转 180°）。Room `default_pos` 取 (0, −0.257, 0)，使桌沿落在 y=−0.60；G1-D 默认站位不会碰到桌腿（桌下净深 0.63 m）；桌面比 real_scene1 高 0.069 m，需要用升降柱补偿、物体同步抬高。
- 验证：本机离线检查 `check_scene.py` 19/19 通过；四个拟合视角下桌子模型边缘与照片贴合，拟合残差大多在 0.3–4 px（脚轮最大约 20 px）。服务器隔离目录 `/data2/tmp/robodojo/agent-remote-runs/lab-render-20261014-*`（共享资产库未改）以 Room=real_scene3 跑 `g1d_pour_water_into_cup` seed 0：物体 +0.069 m，升降柱各 0.0345，`cam_third` 换成 p3 位姿，GPU 0。退出码 0，RTX 渲染正常，第三视角与照片一致，桌面不过曝；倒水失败（脚本问题，同其他场景）。v1 画面偏黄：原因是 Isaac 相机没有白平衡，5000 K 色温再加上没有天花板、RoboDojo 默认的棕色摄影棚 HDR 从上方灌进房间。v2 加了天花板，改为 6500 K、强度 24000（扫了 6000/12000/24000 三档），地毯贴图亮度 ×0.67；第三视角桌面 R/B 1.01、地毯 1.10（照片为 1.05/1.06），桌面与地毯的亮度比 1.98（照片 2.03）。亮度参照的是手机照片，真机头部相机的曝光未核对。视频和帧在本机 `output/lab_render/`（`lab_render_grid_v2.mp4`、`compare_light.jpg`）。未改仓库代码，也未安装到共享 `Assets/Room/`。

## 带相机的 Dex1-1 夹爪接入（2026-10-17，项目成员交付 v1_smooth_wrap）

- 来源：项目成员 zhoushy 的交付包，先 V1（`deliverable.tar.gz`），后改进版 `dex1_camera_delivery_v1_smooth_wrap`（微信 `deliverable.gz`，10-17）；本机解压在 `.cache/dex1_delivery{,_v1sw}/`（gitignored）。保留官方 CAD（`dex1_1_service@ced3216` 的 `base_link.STL`，= `real2sim/robot/g1d_v4/meshes/dex1_1/base_link.STL`）的镜头、前板、倾斜背壳，补后部电子仓、竖直支撑、安装脚（44 mm 高、前移 19 mm，工程估计）；改进版再把手腕侧尾部的双环换成光滑圆柱（`smooth_tail`），并在安装脚后沿加 4 mm 下折边（`add_mount_lip`）。近似几何，惯量沿用原值。
- 接入：交付代码放在 `scripts/g1d/dex1_camera/`（按其清单，不含清单里的根 `README.md`；测试默认 CAD 路径改指 real2sim；README 后半改为仓库用法）。`build_asset.py` 的 `seat_wrist_camera` 在临时目录依次跑作者的 `build` → `smooth_tail` → `add_mount_lip`（保留其输入检查、逐字节保护和三开度外壳-手指碰撞检查），只把两个 `*_dex1_model_base_link` 的视觉/碰撞节点和 `meshes/camera_geometry/` 搬回 `Assets/Robots/g1d/`，删掉不再引用的 `dex1_1/base_link.STL`（取代原 `strip_wrist_camera`）。产物与两版交付模型都逐字节一致（80 个网格，URDF 42 link/41 joint 语义相同）。
- 与相机位姿自洽：镜头前端中心在 `dex1_base_link` (72.4, 0, 49.5) mm，宇树合同镜头位置 (62.7, 0, 55.5) 在其光轴上、玻璃后 11 mm；外壳在腕部视锥里深 5.5–12 mm、本体 19–23 mm，被 3 cm 近裁剪面裁掉。头部画面里两个夹爪上方出现相机模组，和真机一致；腕部画面与标准夹爪相同。图 `output/cam_check/dex1v1/`、`dex1build/`。
- 倒豆子脚本避让（`scripted_demo.py`）：接入后两个倒豆子任务失败。离线诊断（`output/cam_check/debug_pour.py` 记录每步刚体位姿，`analyze_pour.py` 算相机模组碰撞体到容器/另一只手的距离）：最先接触的是接杯那只手夹爪顶上的相机模组——杯对杯里源杯躺平后远端杯沿越过目标杯口扫到它，倒水里倾到约 110° 的壶顶撞到它；之后两只手的模组互碰。模组真机也有，所以改脚本不关碰撞：`LIP_IN, LIP_CLEAR` 0.024/0.025 → 0.010/0.035（源杯杯口少伸进目标杯、略抬高），`CUP_GRIP_UP` 0.09 → 0.065（接杯手握低，模组在杯口以下）。扫过的不行方案：只抬高 `LIP_CLEAR`（4.5 cm 推迟碰撞、6.5 cm 洒出）、目标杯握到 0.05/0.04（指尖顶桌、杯子被甩飞）、倒水间隙 7/9 cm（洒出）。
- 验证（服务器隔离目录 `camdex1-20261017-113230`，叠加 Assets，最新布局）：ruff、`pytest scripts/g1d/dex1_camera/tests`（4 通过）、task_inventory；拾取/叠块 seed 0 在 `g1d`/`g1d_counter`/`g1d_lab` 6/6 成功；三场景四路相机渲染正常。倒豆子（新夹爪 + 新参数 vs 同一布局的标准夹爪 + 原参数，目录 `-stdbase`）：杯对杯 `g1d` seed 0/1/2 = 成功 90%/失败/成功 87%（标准：未跑/成功 86%/失败 54%），`g1d_counter`、`g1d_lab` seed 0 成功 89%/87%；倒水 seed 1 两边都成功 26%，seed 2 两边都失败（19% vs 0%），seed 0 是已知够不到的老问题。杯对杯 seed 1 新夹爪在抬杯阶段（倾倒前）目标杯跟踪误差 5 cm，复跑结果见「最近历史」。每个配置只有 seed 0–2 的布局。注意：多个任务同目录并发跑时按时间找 `_result.json` 会串，按 `<seed>_scripted/` 和各自日志的 `check` 判定。
- 视频：上述验证各次运行的四宫格（头部 / 左腕 / 右腕 / 第三视角，约 14 s）在本机 `output/dex1_videos/`（杯对杯办公室 seed 0、2 成功与 seed 1 两次失败，吧台/实验室 seed 0，倒水 seed 1 成功 / seed 2 失败，共享库上的拾取、叠块），拼接脚本 `output/cam_check/grid_videos.sh`；单路原始视频在服务器 `camdex1-20261017-113230` 和 `shared-dex1-20261017-161826` 的 `eval_result/`。
- 共享资产库：`Robots/g1d` 已换成本次构建（URDF、meshes 含 `camera_geometry/`、physics_contract、两臂 yml；不含 USD 缓存，首次加载重转），先备份到 `/data2/tmp/robodojo/backup-shared-robots-g1d-20261017-161808.tgz`，替换后与构建产物 `diff -rq` 一致。布局未动。

## 三个塑料杯资产（2026-10-17）

- 用户三张桌面照片（白色塑料马克杯 + 两只透明圆角方形量杯，刻度 400/500 ml）。新增 `scripts/objects/make_plastic_cups.py`（写法同 `make_water_cup.py`，复用 `make_pitcher.py` 的 USD 工具）：`Rigid/plastic_mug/00000`（白色 PP，口径 80 mm、高 95 mm、D 形扁平把手在 +X、约 367 ml、90 g，metadata 带与 `mug` 相同的把手抓取帧）；`Rigid/measuring_cup/00000`（400 ml 刻度，高 135 mm，满口 559 ml，45 g）、`00001`（500 ml 刻度，高 158 mm，满口 653 ml，55 g），截面相同（超椭圆 n=4，底宽 64 → 75 mm，卡扣杯沿 82 mm 宽 14 mm 高，杯沿上有小挂耳），半透明偏白哑光 PP（OmniSurface，`CLEAR_MIX`：漫反射权重固定 1、粗糙透射 0.75（乳白份额 0.25；2026-10-19 用户在吧台任务画面里对比 0.08/0.15/0.25 后选定，0.08 在明亮的吧台场景里几乎全透明）、镜面反射权重 0.1、粗糙度 0.3、颜色 (0.94, 0.95, 0.96)）。要点：OmniSurface 分层，漫反射实得 (1 − 透射) × 漫反射权重，之前把漫反射权重设成 0.15–0.4 并令两者和为 1，丢掉了约 16% 能量，杯子越“加雾”越黑（用户对照照片指出真杯偏白）；粗糙度 >0.35 进入强粗糙透射分支也会变暗。在灰地面检查场景里乳白 0.15–0.25 会把杯里豆子洗成浅灰，但在吧台场景里 0.25 隔壁仍能看见豆子（对比图 `output/mcups/milky_counter_compare.jpg`）。另试过：OmniGlass frosting 只会变糊变灰；镜面反射 1.0 时杯面有光泽（真杯没有）。对比图 `output/plastic_cups/milky{,2}_compare_{empty,beans}.png`、`refl_compare_*.png`、`haze_compare_*.png`。每个杯子只有一个视觉网格 `/root/visual/model`，碰撞用杯底 + 杯壁分片凸包（马克杯外加把手 44 个，量杯 51 个），中空。
- 单颗红豆 `scripts/objects/make_red_bean.py` → `Rigid/red_bean/00000`：钝头超椭球 5.8 × 4.3 × 4.0 mm（常见红小豆尺寸），暗酒红有光泽（OmniPBR (0.075, 0.006, 0.008)、粗糙度 0.22），+Y 侧白色种脐（长 45%、窄条，单独网格 `/root/visual/hilum`），单凸包碰撞，6e-5 kg。`define_visual()` 供渲染复用。任务里的红豆仍是 4 mm PBD 球形颗粒（`RED_BEAN_PHYSICS` + `RedBean.mdl`，颜色 0.15 比网格亮），这个网格不参与任务。
- 装豆渲染 `scripts/objects/check_cup_beans.py`：三个杯子（运动学刚体）用任务同款颗粒参数装豆，CPU 状态管线 + GPU PBD（同 `check_pitcher_fluid.py`），落定 2 s；输出 `spheres.png`（仿真实际球形颗粒）、`beans.png`/`top.png`/`closeup.png`（同一落定位置换成红豆网格、随机朝向、缩放 0.85，仅显示用）、`bean.png`（单颗特写）、`metrics.json`。结果：马克杯 2405 粒、400 ml 杯 3465 粒、500 ml 杯 4290 粒，落定后 100% 在杯内，豆面高 50/52/62 mm。量杯从网格填充落定下沉 18–23 mm（杯子上宽下窄，填充区按杯底尺寸）。
- 尺寸：照片里没有尺子，绝对尺寸按常见规格估计；比例按照片对齐：两只量杯宽度相同，400/500 ml 刻度分别在杯高 75%/80%（照片约 74%/80%）。要精确就量一下杯高/口宽改脚本顶部常量。没做刻度浮雕。
- 验证：服务器隔离目录 `/data2/tmp/robodojo/agent-remote-runs/cups-20261017-114621`（资产生成在其 `out_rigid/`）：ruff check/format 通过；脚本自检（绕序、碰撞不侵入空腔）通过；`check_object.py` 三个杯子和红豆都 OK（落地 z 与预期一致、倾角 0°、速度 ≤0.003 m/s），`check_cup_beans.py` 退出码 0；图在本机 `output/plastic_cups/`（`cups_check_v6.png` 为最终材质，`beans/` 为装豆渲染）。未装入共享资产库；量杯已用于 `g1d_pour_beans_measuring_cups`（见下节），马克杯未接入任务。
- 安装：`python scripts/objects/make_plastic_cups.py`（默认写 `Assets/Object/RoboDojo/Rigid/`），改共享库按 README 流程先征得同意。
- 碰撞修正（随量杯任务）：杯底板原为 48 边形（96 顶点、单面 48 顶点），超出 PhysX GPU 凸包限制（≤64 顶点、单面 ≤32），不能和颗粒碰撞，实验室场景提杯时豆子从杯底漏下；改为 `BOTTOM_SIDES = 24`。马克杯同样修正。

## 新任务 `g1d_pour_beans_measuring_cups`：两只量杯空中对倒红豆（2026-10-17）

- 用户要求：两只量杯倒豆子，在吧台场景。新任务，原 `g1d_pour_beans_cup_to_cup` 不变（同一 Common，判定相同：源杯回正时 ≥60% 豆子离开、其中 ≥90% 在目标杯）。500 ml 量杯（`measuring_cup/00001`，源，左手倒）→ 400 ml（`00000`，目标），与照片一致；源杯装约 95 ml（流体椭球缩放 0.9，54 × 54 × 63 mm）。
- 布局 `build_asset.py` 的 `two_measuring_cups`：调用 `two_cups` 取同样的随机位置，再换杯子、抬高 z、换装豆量；两杯绕 z 转 45°（`MEASURING_ORI`），因为侧握时手腕内转 45°，两指合拢方向斜着，转 45° 后夹在方形杯身平面上（72–75 mm）而不是对角（88 mm，接近 95 mm 最大开口）。三个场景配置各 165 个布局，其余任务布局与共享库逐字节一致。
- 脚本 `scripted_demo.py`（按杯子类别取参数，塑料杯参数不变）：夹在杯沿下 `C2C_RIM_DOWN`（塑料杯 2 cm、量杯 3.2 cm，避开 14 mm 卡扣杯沿）；源/目标杯高度分开算（之前目标杯口高度用了源杯高度）；空中抬起 `C2C_LIFT` 塑料杯 12 cm、量杯 6 cm；手指下倾 `C2C_PITCH` 塑料杯 35°、量杯 20°。
- 关键结论：杯子绕手指轴滚转倾倒，cos(倾角) = sin²p + cos²p·cos(滚转)。35° 下倾时滚转 150° 也只倾 105°；量杯高、豆子铺在杯底，摩擦 0.2 需要约 100° 才流出，手臂实际只能滚转约 116°（各关节都没到限位，是可达性），所以卡在 86–89°、只倒出 7–29%。下倾 20° 后同样滚转倾到 101–103°。试过无效或较差：手腕预转 25°/40°（抓杯时斜夹，提杯途中杯子被带歪、撒豆）、相遇点拉近到 y=-0.30（跟踪丢失）、只降抬高（单独无效）。下倾 25° 倾到 95°，15° 也成功（余量更小）。
- 验证（服务器隔离目录 `/data2/tmp/robodojo/agent-remote-runs/mcups-20261017-*`，叠加资产：共享库只读软链，本地 `Rigid/measuring_cup` 和三个场景布局；共享库未改）：scripted_demo 各 1 回合，退出码 0、`eval_time=1`、`success_rate=1.0`：吧台 seed 0/1/2（倒出 88–90%、进杯 87–89%）、办公室 seed 0、实验室 seed 0；原杯对杯任务吧台 seed 0 回归通过（82%/82%）。ruff check/format、`task_inventory.py --check` 通过（`task_inventory.py` 中我未改的旧代码有 format 差异，未动）。四宫格视频 `output/mcups/counter_s0.mp4`（本机）。
- 坑：几个评测同一秒启动会写进同一个时间戳目录，互删临时视频后评测端换下一个布局重跑，日志末行会误报成功；并行要错开 >1 s，并按回合看日志。修杯底前吧台 seed 1 曾在倾倒中杯子在两指间滑出（之后重跑通过，物理每次运行有随机性）。
- 吧台场景每次运行有 6–7 条 PhysX "Non-GPU-compatible convex mesh is not able to collide with particle system"，原杯对杯任务也有，办公室/实验室没有，来自吧台场景自身的碰撞网格（未查具体网格）；豆子掉到这些表面上会穿过去。吧台的 `cam_third` 被场景里一块板挡住。

## Real2Sim 数采管理平台（2026-10-19）

- 需求：参照 Notion「Real2Sim2Real」页，做覆盖场景重建、物体资产、任务采集、数据质检全流程的数采管理 Web。
- 实现 `src/real2sim_platform/`：`server.py`（标准库 `http.server` + SQLite，无新依赖，系统 `python3` 可跑）、`index.html`（单页，原生 JS）、`test_server.py`（自检）。启动和点「从项目同步」时扫描：`real2sim/scenes/*`（参考照片、`fit_photo.json` 残差或 `cameras_scene.json` 相机数、USD、`check_report.json`、预览）；`task_inventory.build_inventory()` + 任务模块里的 `step_lim` + `<assets>/Eval_Layout/RoboDojo/<cfg>/**/<task>_<n>.json` 计数；G1-D（custom）任务配置里 Rigid/Fluid 的 category/index/label（逐行解析，系统 Python 没有 PyYAML）得到物体，再关联 `<assets>/Object/RoboDojo/<id>/{metadata,description}.json` 和 `scripts/objects/make_*.py`；`--eval-root` 下所有 `_result.json` 的 details 与 `episode_<n>_cam_<cam>_<tag>.mp4`。场景与 env 配置的对应取自 `build_asset.SCENES`。
- 数据契约：每条记录的 `auto` 是扫描结果，其余键是人工录入；重扫只替换 `auto`。扫描不到的记录：没有人工内容就删除，有就标 `auto.missing`。回合 id = 运行目录绝对路径 + `#` + 回合序号（移动 eval 目录后录入会和新路径脱钩）。物体 id 必须是 `<类型>/<类别>/<编号>`（服务端校验，防路径穿越）。`/file` 只提供 `real2sim/`、`output/`、资产库和 eval 根目录下的图片、视频、md、json。
- 判定（前端 `*_STAGES`，阈值取自 Notion 月度目标）：场景实测 < 1 cm、叠图 < 5 px 且 ≥ 3 视角；物体每个登记尺寸误差 < max(2 mm, 2%)（按名称比，见下面「物体登记」）、质量差 ≤ 5 g、5 项物理检验；物体角色目标按任务设定，保留 30%；专家（`scripted_demo`）成功率 ≥ 80%；合格回合 = 质检通过且仿真成功，默认目标 500；回合自动检查 = `eval_time` ≥ 1、head/left_wrist/right_wrist 视频齐全（同 `summarize_result.py`）、视频文件名标签与 `_result.json` 一致（能发现并发写进同一时间戳目录造成的串结果）；Sim↔Real 用回合上的「真机对照」算 AA、Cohen's κ、P(真机成功 | 仿真成功)。
- 物体登记（2026-10-19 用户要求：角色不能只按倒水任务写死；登记信息要直接作为 coding agent 建模的输入，照片在登记时就传；质量、材质固定字段，尺寸按物体形状自由填）：
  - 角色：去掉前端写死的 `ROLES`/`ROLE_TARGETS`（源 12、接收 15、干扰 30）。任务记录新增 `roles: [{name, target}]`，在任务抽屉「物体角色」编辑（可一键加任务配置里的标签）；物体的 `role` 改为自由文本（候选来自各任务角色和已有物体角色），新增 `tasks`（手选的用于任务，和配置解析出的 `auto.tasks` 合并）。某角色的件数 = 手选该角色且用于该任务的物体 + 该任务配置里带这个标签的物体。总览「资产泛化目标」和物体页顶部卡片按任务列出。
  - 尺寸：`real_dims`（三个数）换成 `dims: [{name, real, sim}]`，名称随意（预设“圆杯：开口直径/底部直径/高”“方形：长/宽/高”“把手”）。核对按名称：仿真值优先取资产 `metadata.json` 顶层 `measurements_mm[名称]`（扫描进 `auto.measurements`；`utils/load_file.py` 只读 physics/visual/geometry/active/passive，加这个键不影响仿真），没有时用手填的 `sim`；不再拿包围盒排序比对。`5090-2` 上唯一有旧值的 `plastic_mug/00000`（119/84/100）迁移成“总长（含把手）/外径/高”，保留原 `_by`。
  - 建模输入：登记表单一次填类型/类别/编号（编号取资产库、`real2sim/objects/` 和库里记录都没用过的下一个号，`GET /api/object-indices`）、名称、用于任务、角色、材质、颜色、质量、尺寸、外观与结构、建模要求、照片。`twin` 为真的物体每次保存都由服务端写 `real2sim/objects/<id>/spec.json`（`dimensions_mm` 是名称→实测值，空字段省略；取消孪生或删除登记时删掉该文件）。物体抽屉生成「建模任务」文本（未入库时在顶部），内容是输入文件位置、登记信息和交付要求：生成脚本、`physics.mass/size/static_friction/dynamic_friction`（运行时只读这两个摩擦键）、`measurements_mm`、凸包 ≤ 64 顶点/单面 ≤ 32、`check_object.py --image-dir`、更新 `ASSETS.md`。
  - 验证：本机 `test_server.py` 5 项（新增 spec 写入/删除、未命名或未测的尺寸被略去、`measurements_mm` 扫描、编号接口）、ruff、`node --check`；浏览器（本机临时库 + 从 `5090-2` 镜像拉的物体元数据）：任务抽屉一键加配置标签为角色并填目标数、物体页按配置标签计数（`source_cup` 1/12）、登记表单类别输入后编号自动跳过已占用号（measuring_cup → 2）、圆杯预设、选照片预览、提交后抽屉顶部出现建模任务、照片和 `spec.json` 落盘、抽屉里改尺寸/删行后核对表和 `spec.json` 同步、删除登记后 `spec.json` 被删。测试登记用的 `Rigid/zz_test_cup` 目录已清掉。
- 和 `real2sim/ASSETS.md` 的关系：登记表是定稿来源；平台里的实测值是工作记录，定稿后写回生成脚本和登记表（README 与物体页都写明）。
- 登录（2026-10-19 用户要求“加上登录功能取代操作人”，取代了上午的共享 Basic 登录 `REAL2SIM_PLATFORM_AUTH`）：账号存平台 SQLite 的 `users` 表（PBKDF2-SHA256 20 万轮加盐），会话存 `sessions` 表（只存 token 的 SHA-256，14 天过期）。cookie `r2s_session` 为 HttpOnly + SameSite=Lax，没有 Secure（HTTP）。`Handler.parse_request` 统一拦截：只有 `/`（页面本身，不含数据）和 `/api/login` 免登录，其余返回 401 JSON。`_by` 由服务端写成会话账号的显示名，前端传的 `_by` 被忽略；前端去掉了“操作人”输入框，侧栏显示当前账号并可改密码（`/api/password`，改完踢掉该账号的其他会话）、退出（`/api/logout`）。防爆破：同一用户名 5 次失败锁 5 分钟（内存里，重启清零；经隧道来的请求都是 127.0.0.1，没法按 IP 限），失败时等 0.5 s，未知用户名也算一次哈希。
- 管理员（2026-10-19 用户要求“弄个管理员账号，在网页上建号，不要每次用命令行”）：`users` 表加 `admin` 列（旧库启动时 `ALTER TABLE` 自动补，原账号都成为普通成员）。`/api/users` 只对管理员开放（其他人 403）：GET 列表，POST 新建（用户名 `[\w.-]{1,32}`、密码 ≥ 8 位、重名 409），PUT `?username=` 改显示名/角色/密码（新密码踢掉该账号所有会话），DELETE 删除；四种都返回最新列表。不能删除自己、不能取消自己的管理员，所以总至少有一个管理员。前端侧栏对管理员多一个「成员管理」页：表格里直接改显示名和角色，重置密码在浏览器里生成 12 位随机密码（`crypto.getRandomValues`，HTTP 下可用）并只显示一次；新建表单默认填好随机初始密码，建好后显示可复制的“地址/账号/密码”。改名或删号不改已有记录上的 `_by`（存的是署名字符串）。命令行保留：`--add-user NAME [--name] [--admin]`（新建或重置密码）、`--remove-user`、`--list-users`，只用来建第一个管理员或救急；启动时没有管理员会打印提示。验证：本机 `test_server.py` 5 项（新增旧库迁移，以及管理员接口的 400/403/404/409、重置密码踢会话、不能删自己/降自己）、ruff、`node --check`；`5090-2` 上用 Python 3.10.12 在仓库临时副本里跑同样 5 项通过，生产库副本迁移后账号不丢；浏览器（本机临时库）实际操作：管理员建号（显示一次登录信息）、表格改显示名和角色、成员登录后没有「成员管理」入口且直接打开 `#users` 只看到权限提示。浏览器自动化里 `fill` 不触发 `input` 事件，新建表单因此改为提交时直接读输入框；退出登录时清空主区域，避免刚生成的密码留在隐藏的 DOM 里。
- 部署（2026-10-19，用户要求放到 5090-2 并经阿里云穿透）：SSH 别名 `5090-2`（主机名 ecs-5090-cb2-0028，内网与 `rtx5090-180` 同网段）上的 `/root/RoboDojo`。这台机器访问不了 GitHub，仓库是从同网段机器内网拷来的（含 `.git` 和 LFS 文件，以及本机同步的未入库 `real2sim/scenes/*/preview/`、`eval_result/`），apt 装了 git-lfs。tmux 会话 `r2s-platform` 跑 `server.py --port 8765 --assets /root/RoboDojo_assets/Assets`（只监听 127.0.0.1；账号在 `.cache/real2sim_platform.sqlite3`：管理员 `admin`（显示名“管理员”），普通成员 `real2sim` 沿用之前共享登录的密码；日志 `.cache/platform.log`），`r2s-tunnel` 维持 `ssh -N -R 0.0.0.0:10081:127.0.0.1:8765` 到别名 `Aliyun` 的主机（断线 5 s 后重连，日志 `.cache/tunnel.log`）。公网入口是 `Aliyun` 的 10081 端口，用户名 `real2sim`。tmux 不随重启恢复。
- 资产库镜像（2026-10-19，页面曾显示“资产库 /root/RoboDojo/Assets（不存在）”）：`5090-2` 没有资产库，只镜像平台会读的部分——21196 共享库 `/data2/zhongzheng/Project/RoboDojo_assets/Assets` 下的 `Object/RoboDojo/**/{metadata,description}.json` 和 `Eval_Layout/**/*.json`（19,054 个文件，239 MB），放在 `/root/RoboDojo_assets/Assets`（不是完整资产库，不能拿来跑仿真；故意不放在仓库的 `Assets/`，免得被当成完整资产库）。镜像不会自动更新：共享库装了新物体或布局后，在本机重跑（先写临时目录再整体替换，删除的布局也会同步消失；约 3 分钟）：
  `ssh 21196 "cd /data2/zhongzheng/Project/RoboDojo_assets/Assets && { find Object/RoboDojo -name metadata.json -o -name description.json; find Eval_Layout -name '*.json'; } | tar -czf - -T -" | ssh 5090-2 "D=/root/RoboDojo_assets; rm -rf \$D.tmp && mkdir -p \$D.tmp/Assets && tar -xzf - -C \$D.tmp/Assets && rm -rf \$D && mv \$D.tmp \$D"`，然后在网页点「同步项目数据」。`5090-2` 能连通 21196 的 SSH 端口，但上面没有 21196 的凭据；要它自己定时拉，需要在 21196 上加一把只读受限密钥。镜像后扫描结果：4 个 G1-D 任务在三个配置下各有布局（量杯任务 0，共享库未装），62 个物体里 60 个有元数据（缺两只量杯），扫描 0.2 s。仓库里那个 12:25 出现的半截 `Assets.new/`（301 个 json，来源不明，像是中断的同步）已删除。部署（commit `6da0014`）：改了 `.cache/run-platform.sh` 之后必须重建 `r2s-platform` 会话（`tmux kill-session -t r2s-platform; tmux new-session -d -s r2s-platform -c /root/RoboDojo /root/RoboDojo/.cache/run-platform.sh`），只 pkill 服务进程的话，外层循环还是用旧命令拉起。公网验证：未登录 `/api/users` 401；`admin` 登录返回 `admin: true`，`/api/info` 的 `assets_ok` 为 true，任务布局数和物体元数据与上面一致；浏览器打开公网页面，「成员管理」可用，侧栏显示资产库已连接。
- 曾误部署到 `rtx5090-180`（当时本机没有 `5090-2` 别名，按编号推断错了）：已停掉会话，删掉那里的 `/root/RoboDojo`、隧道密钥和阿里云上对应的授权行。
- 阿里云侧：10080–10100 里 10080/10088/10099/10100 已被占用，选 10081。`/root/.ssh/authorized_keys` 里有一行 `5090-2` 专用密钥 `~/.ssh/real2sim_tunnel` 的公钥（注释 `real2sim-platform-tunnel@5090-2`），限制为 `restrict,port-forwarding,permitlisten="0.0.0.0:10081",permitopen="127.0.0.1:9",command="/bin/false"`（同样的限制在旧密钥上验证过：不能执行命令、不能监听其他端口）。每次改动前都备份为 `authorized_keys.bak-real2sim-<时间戳>`。阿里云原有隧道（DSW 反向 SSH 加本地 `autossh -L`）未动。
- 运维：`5090-2` 的仓库设了 `receive.denyCurrentBranch updateInstead` 并让 LFS 检出时不下载（`filter.lfs.smudge/process --skip`），所以在本机 `GIT_LFS_SKIP_PUSH=1 git push 5090-2:/root/RoboDojo main` 就能更新代码，新增的 LFS 文件（照片等）和渲染图再 `rsync -az real2sim/objects/ 5090-2:/root/RoboDojo/real2sim/objects/`，之后在那边 `git add real2sim/objects`（只刷新 LFS 文件的 stat，不产生改动）。两个 tmux 会话分别只跑 `.cache/run-platform.sh`、`.cache/run-tunnel.sh`（服务器本地脚本，含阿里云地址，不入库）。重启服务用 `ssh 5090-2 'pkill -f "[p]ython3 src/real2sim_platform/server.py"'`（循环 3 s 后用新代码拉起）。**不要**在会话里按 Ctrl-C（SIGINT 会连外层循环一起结束），也**不要** `pkill -f src/real2sim_platform/server.py`：tmux 服务进程的命令行是第一次建会话时的命令，曾因此把 tmux 连同隧道一起杀掉（2026-10-19 约 2 分钟中断，已改成脚本启动避免）。停用：`tmux kill-session -t r2s-platform; tmux kill-session -t r2s-tunnel`，再删掉阿里云 `authorized_keys` 里 `real2sim-platform-tunnel@5090-2` 那一行。
- 部署验证（从本机经公网访问）：无凭据返回 401；带凭据后 `/api/info`、首页、场景图片 200，`5090-2` 的访问日志记录到这些请求；`test_server.py` 在 Python 3.10.12 下 4 项通过（在同版本的 `rtx5090-180` 上跑的）。
- 物体图片（2026-10-19，用户问物体页为何没有渲染图和照片）：约定 `real2sim/objects/<类型>/<类别>/<编号>/`，`reference/` 放实物照片（入库，LFS），`preview/` 放仿真渲染（`.gitignore` 已有 `real2sim/**/preview/`，不入库）。扫描器给每个物体加 `auto.photos`、`auto.renders`（`preview/` 加资产目录里的图片）；`real2sim/objects/` 下有目录的物体即使不在任务里也列出（`plastic_mug`、`red_bean`）。官方物体 metadata 没有 `physics.size`，仿真尺寸回退到 `geometry.aligned_bbox.extents`。新接口 `POST /api/photo?id=&name=`（原始字节，≤ 25 MB，jpg/png/webp，文件名清洗、重名加序号）、`DELETE /api/photo?path=`（只能删 `real2sim/objects/**/reference/` 下的图）。
- 已有照片：用户 10-17 的三张桌面照片 IMG_4867/4868/4871（会话附件）分到两只量杯、`plastic_mug`（4871）、`red_bean`（4867/4868）；`plastic_cup/00000` 用真机片段 `G1D-PourBeansPlusEps222_ep002.mp4` 第 0.5 s 帧（头部相机，桌上两只塑料杯）。
- 渲染：`check_object.py` 新增 `--image-dir DIR`，一次启动每个物体配一台 640×480 相机（物体间距改 0.6 m，地板随一排物体加长并居中；相机从侧前上方，距离 = 3.7 × 包围盒对角线，按 USD 默认 50 mm 镜头的 18° 竖直视角算，之前按 47° 估太近），写 `<DIR>/<类型>/<类别>/<编号>/preview/sim.png`。21196 隔离目录 `agent-remote-runs/objrender-20261019-103920`（自建 9 个由当前生成脚本重新生成到 `ovl/Rigid/`，官方 54 个只读共享库，共享库未改）出了 63 张，已同步到本机和 `5090-2` 的 `real2sim/objects/`。落地检查：自建 9/9 OK；官方 34/54 OK，其余多为 z 偏差——官方资产原点在底部而脚本假定在包围盒中心，z 判据对官方物体不成立，倾角才有意义（clock、drawing_compass、toy 倒下 75–90°，piler 6.6°）。锯子 `saw/00000` 资产竖立建模、落地后倒平，按竖立尺寸取景偏远，缩略图很小。
- 界面重做（用户要求用 ui-ux-pro-max 技能，已用官方 CLI `npx ui-ux-pro-max-cli init --ai cursor --global` 装到 `~/.cursor/skills/`，v2.15.0）：按其设计系统检索（内部运营看板、密度 8）取 Swiss 极简 + 海军蓝 `#1E40AF` / 琥珀 `#D97706`；没用它推荐的 Google Fonts（Fira 无中文字形，且国内经阿里云访问会卡），改系统中文字体 + 等宽数字；没用 GSAP。实现：左侧栏导航（计数、待审角标、同步状态、窄屏变顶栏）、设计 token、内联 SVG 图标（Lucide 路径）、状态都带图标和文字（不只靠颜色）、分段进度条、焦点环、`prefers-reduced-motion`、抽屉带遮罩并在关闭后还焦点、灯箱（←/→ 翻页、Esc）、复制命令按钮。物体页默认卡片（渲染主图 + 实拍小图），抽屉里实物与仿真左右对照、上传/删除照片；质检页加批量勾选和批量结论、视频统一播放/暂停/从头/倍速、真机结果分段按钮。
- 界面验证：本机 `test_server.py` 4 项（含上传/删除和路径校验）、ruff、`node --check` 页面脚本；浏览器实际操作总览、物体卡片与抽屉、质检（选回合、勾选出批量栏、4 路视频）、场景抽屉灯箱（翻页、Esc 先关灯箱、焦点回到缩略图、再 Esc 关抽屉）、网页上传中文文件名照片并删除。注意：Cursor 内置浏览器的某个标签页曾进入拦截所有点击的状态（页面内连 body 的点击都到不了 document 监听器），换新标签页后一切正常，属于工具环境问题。
- 验证（本机）：`python3 src/real2sim_platform/test_server.py` 4 项通过（任务配置解析、仓库扫描、回合扫描 + 重扫保留录入 / 标记缺失、HTTP Range、路径限制、非法物体 id 返回 400）；`uvx ruff check`、`ruff format --check` 通过。浏览器里用临时库 `/tmp/r2s_test.sqlite3` 和 `/tmp/r2s_fixture`（假资产库 + 3 回合运行，视频取自本机 `eval_result/real_scene1`）实际操作五个页面：场景录入后阶段判定随之变化、重启服务后录入仍在、量杯关联 metadata 后尺寸核对通过、质检快捷键 P 通过并跳到下一条 / R 拒绝后停留、改标签不打断视频播放、登记新物体、Sim↔Real 指标（AA 0.67、κ 0.40，与手算一致）。未在服务器 21196 的共享资产库和隔离目录上运行（本机 `Assets/` 为空，物体元数据和布局数只用假数据验证过）。

## 剩余事项与风险

- real_scene3：尺寸没有实测（量一下桌长或桌高，再用 `--scale-correction` 修正）；灯光已按手机照片标定，仍需和真机头部相机画面对比。已作为 `g1d_lab` 配置接入（见「三个场景接入」一节）。

- 带相机夹爪 + 新避让参数下（2026-10-17，最新布局）：杯对杯 `g1d` seed 0、2 通过，seed 1 两次都失败（一次抬杯阶段目标杯跟踪误差 5 cm、一次倒出 86% 只进杯 65%），而同一布局下标准夹爪 + 原参数 seed 1 通过、seed 2 失败——seed 1 是这次接入后唯一的退化，下一步可用 `output/cam_check/debug_pour.py` + `analyze_pour.py` 看它是不是仍有模组接触。倒水 seed 2 两种夹爪都失败。
- 杯对杯倒豆（旧记录，标准夹爪、旧布局）：3 个 seed 里 2 个通过。seed 1 的目标杯位置（x 0.159、y -0.320）右手够不到胸前位置，可以把目标杯的 y 收窄到更靠近机器人，或者把胸前位置跟着杯子走。右手倒（真机片段里是右手倒）会让杯子在两指间转动，没解决；值得试的方向：夹紧时用线性 PD 让两指都停在杯面上（参照叠块的做法），而不是全力闭合。
- 双臂倒水：换成自建水杯后 seed 1 成功；seed 0（壶在右）右手倒壶时要么够不到杯口（差 9 cm），要么把杯子带近后壶身撞上持杯手，需单独调右手倒壶的姿态/壶嘴方向。`water_cup`/`plastic_cup` 和新布局已于 2026-10-16 装入共享资产库。杯子更远的布局（y≈-0.46）未测。
- G1 手臂在桌面高度的可达范围约 0.45 m；RoboDojo 现有任务（为桌面上的 X5 设计）有物体距肩部 0.4–0.7 m，部分不可达。自有任务的物体应放在 y≈[-0.55, -0.25]，或利用腰部前倾（`Yaw_Joint`）。
- 相机：内参用棋盘格标定值（头 `g1d_head` 约 98°、腕 `dex1_rgb` 约 91°），头部与腕部外参都用宇树合同值，并经真机帧投影核对（头部含胸口上沿，见 2026-10-16 节）；未建模畸变（<2%）和腕部主点偏移（最多 17 px）；头/腕都靠近裁剪面（10 cm / 3 cm）隐去自身外壳，离相机更近的物体也会被裁掉。共享库 `Robots/g1d/*_robot_config.yml` 已于 2026-10-17 11:49 同步为仓库版本（头部合同位姿）。
- 末端位姿（ee）动作未支持：需写 G1-D cuRobo 配置；且 `RobotManager` 的 planner 以 `robot_name` 为键，coupled 左右臂会共用同一个 planner，需要改成按手臂区分。
- `_task.yml` 中指定了 `robot_config`/`scene_config` 的任务（如 x5+franka 竞赛任务、传送带任务）会覆盖 g1d 机器人或场景。
- 棱纹环按 `elbow_link` 网格的 x 区间切分（换 URDF/网格后需重新核对 `RING_X`）；材质参数为目测，仿真房间光照比真机亮，金属会显得偏亮。
- 服务器上 `/usr/local/bin/ffmpeg` 不带 libx264，已在仿真环境 bin 中链接系统 ffmpeg；新建环境需同样处理，否则录视频时 BrokenPipe。
- 自建资产在 `real2sim/`，`real2sim/**` 下的 usd/usdc/STL/图片走 Git LFS（`.gitattributes`），`preview/`、`demo/`、`*.bak` 不入库。`.gitignore` 用锚定的 `/Assets`：macOS 大小写不敏感，不锚定会把 `real2sim/scene3_pour_pack/assets/` 一起忽略。不要再往 `Assets/` 里放要入库的文件：`init_assets.sh` 发现 `Assets/` 不完整会把它整个挪成 `Assets.partial.<时间戳>`。本机 git-lfs 为官方二进制，装在 `~/.local/bin`（Homebrew 在此机会从源码编译整条依赖链，不要用）。
- 杯对杯倒豆的杯子位置按布局随机抽取（种子取自瓶子三维坐标，含桌面高度），所以三个场景同一编号的杯子位置不同；其余任务同一编号在三个场景里相对机器人一致。
- 自建资产装没装进共享库、和真实的匹配程度，统一看 `real2sim/ASSETS.md`（2026-10-19 核对：共享库缺 `measuring_cup`、`plastic_mug`、`red_bean` 和量杯任务布局；`Robots/g1d` 是 10-17 带相机夹爪版）。
- 自建物体的摩擦没有生效：`rigid.py` 只读 `physics.static_friction`/`dynamic_friction`（默认 0.6/1.5），生成脚本写的 `friction` 被忽略。现有成功率都是在默认摩擦下得到的；改成实测摩擦前要让生成脚本写这两个键，并重跑倒豆/抓取任务。
- 所有自建物体和三个场景都没有卷尺/秤实测，尺寸来自照片拟合或常见规格估计；待测清单在 `real2sim/ASSETS.md`。
- 数采平台：权限只有两级，管理员只多了成员管理，成员登录后也能改所有记录；`5090-2` 的资产库是手动刷新的元数据镜像，共享库更新后要重跑镜像命令；HTTP 明文传输密码；不从网页启动仿真，命令只展示供复制；回合质检只看视频和 `_result.json`，Notion 里的关节速度/加速度/加加速度、接触力、最小间距等运动合理性指标要等轨迹录制器输出关节数据后再接；L0–L4 进度是人工填写，布局还没按级别生成；实测值在平台和 `ASSETS.md` 两处，需要人工同步。渲染图不入库，换部署机器要 rsync `real2sim/objects/`（或在 GPU 机重跑 `check_object.py --image-dir`）；网页上传的照片和生成的 `spec.json` 落在部署机的仓库工作区，coding agent 要先把该物体目录同步过来，入库得在那台机器（或拷回本机）提交。`spec.json` 只由平台写出，手改它不会回到平台数据库；平台数据库丢了也不会从 `spec.json` 恢复。现有生成脚本还没写 `measurements_mm`，老物体的尺寸核对要手填仿真值或给脚本补上。缩略图直接用原图，没有服务端缩图（只用标准库），场景预览较大时经公网加载会慢。

## 下一会话

1. 先看 `git log` 和本文件；服务器环境已可用，直接用 demo_policy 跑 `--env-cfg g1d` 复现。量杯对倒：`CUDA_VISIBLE_DEVICES=N PYTHONPATH=.:XPolicyLab EVAL_NUM=1 python scripts/g1d/scripted_demo.py --env_cfg_type g1d_counter --task_name g1d_pour_beans_measuring_cups --seed 0`，需要先把 `measuring_cup` 和新布局装进资产库（`make_plastic_cups.py` + `build_asset.py`；共享库尚未安装，需用户同意）。
2. 下一步建议：按 `real2sim/ASSETS.md` 的待实测清单量桌子和杯子，写回生成脚本 / 场景尺度并更新登记表；改摩擦前先让生成脚本写 `static_friction`/`dynamic_friction`。
3. 数采平台已部署在 `5090-2`，经 `Aliyun` 的 10081 端口访问（见「Real2Sim 数采管理平台」一节），管理员账号 `admin` 在网页「成员管理」建号。资产库是 21196 共享库的元数据镜像（手动刷新）；评测结果仍只有同步过去的 1 个回合，要看真实回合需要把评测结果同步过去（或改在 21196 上部署，用 `--eval-root <隔离目录>`）。tmux 会话不随机器重启恢复。

## Dex1-1 改回标准版（不带腕部相机模型，2026-10-16；已被 10-17 的带相机夹爪取代）

- 用户要求：在找到真实相机模型之前，夹爪用不带相机的标准版。真机 `dex1_1_service/urdf/meshes/base_link.STL`（已拷到本机 `.cache/unitree_official/urdf/meshes/`）与 v4 的 `base_link.STL` 逐字节相同，本身就带相机支柱和相机，所以没有现成的无相机网格。
- `build_asset.py`：`seat_wrist_camera`（把相机模块下移到机身上）改为 `strip_wrist_camera`，删掉相机模块和线缆插头，把支柱从机身顶面（z=0.0335）截掉，截面以下保留（这部分是机身的盖板，整块删掉会露出齿轮）。腕部相机**传感器**（robot config 的 `cam_wrist`）不变，仍然出腕部画面。对比图在本机 `output/dex1_std_compare2.png`。
- 验证：只在服务器隔离目录 `c2c-20261016-105635-lab` 的 `Assets/Robots/g1d` 叠加副本里换了网格并重新转 USD，实验室场景 seed 0 跑了两次：一次成功（77% 进杯），一次失败（倒到一半源杯从手里滑落）。同一配置结果不同，说明仿真每次运行有随机性，不是网格造成的。四宫格视频 `output/c2c/lab_std/lab_std_grid_s0.mp4`。共享资产库的 `Assets/Robots/g1d` 已于 16:14 换成该网格。

## 资产目录重组：自建内容移到 `real2sim/`（2026-10-16）

- 原因：`Assets/` 应该是拉代码后由 `init_assets.sh` 下载的目录，之前把 `Assets/scene3_pour_pack` 放进 git 会和下载脚本冲突；三个场景和 `build_asset.py` 依赖的 v4 URDF 只在本机或服务器 `/data2/tmp`，别人 clone 后无法复现。
- 移动：`Assets/scene3_pour_pack` → `real2sim/scene3_pour_pack`；`Assets/sim_scene_opus` → `real2sim/scenes/real_scene1_office`；`Assets/sim_scene_counter` → `real2sim/scenes/real_scene2_counter`；`Assets/sim_scene_lab` → `real2sim/scenes/real_scene3_lab`；服务器 `/data2/tmp/robodojo/g1d_v4/`（不含 finalize.log）→ `real2sim/robot/g1d_v4`（47 个文件，46 MB；`sha256.json` 里 `g1_d_upper_dex1_v4.usd` 一项与文件不符，本机与服务器文件哈希一致，是清单本身过期）。`config.yaml` 里的原机器绝对路径是转换记录，未改。
- 代码：`build_asset.py` 的 `--v4` 默认 `real2sim/robot/g1d_v4`（不再必填），文档字符串和 `--scene` 说明改为新路径；`scene3_pour_pack/motion/kinematics.py` 默认 URDF 改为 `real2sim/robot/g1d_v4/g1_d_upper_dex1_v4.urdf`；`make_water_cup.py` 和场景脚本注释里的旧目录名同步更新。
- README「快速开始」改为完整流程：clone（LFS）→ `install.sh -i` → `init_assets.sh` → `update_embodiment_config_path.py` → 生成壶/杯 → `format.py` → `build_asset.py --scene real2sim/scenes/real_scene1_office/usd` → 拷贝场景 2/3 到 `Assets/Room/`；仓库结构加入 `real2sim/`。
- 验证：`G1Kinematics()` 默认路径加载新 URDF（33 个 link、32 个 joint）；ruff、`git diff --check`；`git lfs ls-files` 确认 real2sim 二进制都走 LFS；旧路径在 git 中识别为重命名。未在 Isaac 中重跑 `build_asset.py`（本机无 Isaac，服务器共享资产库改动需用户同意）。

## 三个场景接入 `build_asset.py`，并装入共享资产库（2026-10-16）

- 每个场景一个 env 配置：`g1d`（real_scene1 办公室，0.68 m，不变）、`g1d_counter`（real_scene2 吧台，0.833 m，根节点 y=-0.80，升降柱各 0.0765）、`g1d_lab`（real_scene3 实验室，0.749 m，升降柱各 0.0345）。新增 `env_cfg/g1d_{counter,lab}.yml`、`env_cfg/robot/g1d_{counter,lab}.yml`，`_robot_info.json` 加两项。布局目录和评测结果都按配置名分开。
- `RobotManager._attach_scene_cfg`：机器人 yml 新增可选 `init_joint_pos`，覆盖 robot_config 里的初始关节角（升降柱高度靠它）；不写时行为不变。
- `build_asset.py`：`SCENES` 表（场景目录、Room 名、Room 位置、物体 xy 平移）取代单场景常量；`install_rooms` 每次都把 3 个场景装进 `Assets/Room/`（拷 `usd/` 和入口 `<room>.usd`；real_scene1 的入口从代码字符串改成仓库文件 `real2sim/scenes/real_scene1_office/real_scene1.usd`），去掉 `--scene` 参数；`build_layouts` 为每个配置生成 4 个 G1-D 任务的布局，吧台的物体在换物体之后再整体 y-0.07（跟着机器人后退）。
- 验证（服务器隔离目录 `/data2/tmp/robodojo/agent-remote-runs/scenes-20261016-171414`，共享资产覆盖副本）：每个配置 192 个 G1-D 任务布局；`g1d` 布局与上一版代码生成结果逐字节一致；吧台/实验室布局除杯对杯倒豆外，1299 个物体都等于办公室位置加 (0,-0.07,+0.153)/(0,0,+0.069)。scripted_demo seed 0：`g1d_counter` 和 `g1d_lab` 的 `g1d_stack_blocks`、`g1d_general_pickup`，以及 `g1d` 的 `g1d_stack_blocks`，5 个全部退出码 0、eval_time=1、success_rate=1.0。三场景截图对比在本机 `output/scenes/scenes_grid.png`。倒豆任务未在新场景配置下重跑。
- 共享资产库（用户同意）：新增 `Room/real_scene{2,3}`、`Rigid/{water_cup,plastic_cup}`、`Eval_Layout/RoboDojo/{g1d_counter,g1d_lab}`，`g1d` 的 G1-D 任务布局换成当前版本，`Room/real_scene1` 覆盖为同样内容；与隔离目录验证产物 `diff -r` 一致。改动前备份 `/data2/tmp/robodojo/backup-shared-20261016-174239.tgz`（原 `g1d` 布局和 `real_scene1`）。没有跑 `format.py`（运行时不读 `uuid_map.json`）。

## 最近历史

- 2026-10-19：数采平台物体管理改造：物体角色按任务设定（不再写死倒水角色），尺寸改为按名称自由填写并按 `metadata.json` 的 `measurements_mm` 核对，登记表单一次填完测量信息和照片，登记后写 `real2sim/objects/<id>/spec.json` 并生成给 coding agent 的建模任务。

- 2026-10-19：修复数采平台在 `/#scenes` 这类地址刷新时出现 “Error response”：日志显示用户的浏览器刷新时把 `#` 编码成 `%23`，请求 `GET /%23scenes`，服务端按文件路径返回 404（未登录时是 401 JSON）。`Handler.parse_request` 在认证前把解码后匹配 `/#[\w-]*` 的路径 302 到 `/#<标签>`（只放行字母数字下划线连字符，防止经 `%0D%0A` 注入响应头）；测试覆盖跳转和注入。

- 2026-10-19：数采平台加管理员角色和网页「成员管理」（新建、改名、改角色、重置密码、删除，随机密码只显示一次），命令行只用来建第一个管理员；`5090-2` 上建了管理员 `admin`。`5090-2` 的资产库改为 21196 共享库的元数据镜像 `/root/RoboDojo_assets/Assets`（`--assets`），页面不再显示“资产库不存在”。

- 2026-10-19：数采平台改为账号登录（每人一个账号，署名由服务端按账号写入），取代“操作人”输入框和共享 Basic 登录；侧栏可改密码、退出；账号在服务器用 `server.py --add-user` 管理。

- 2026-10-19：数采平台界面按 ui-ux-pro-max 重做（侧栏导航、设计 token、状态图标、物体卡片、实物 vs 仿真对照、质检批量操作）；物体实拍照片入 `real2sim/objects/<id>/reference/`，`check_object.py --image-dir` 在 21196 为 63 个物体出单独渲染（不入库，已同步到 `5090-2`）。

- 2026-10-19：数采平台部署从误选的 `rtx5090-180` 迁到新加别名的 `5090-2`（GitHub 不通，内网拷仓库，本机 `git push` 更新），公网入口和账号不变，旧机器已清理。
- 2026-10-19：数采平台加 HTTP Basic 登录（`REAL2SIM_PLATFORM_AUTH`），经 `Aliyun` 10081 端口反向隧道对外；隧道密钥在阿里云上限定只能监听该端口。

- 2026-10-19：新增 Real2Sim 数采管理平台 `src/real2sim_platform/`（标准库 Web：场景重建、物体资产、任务采集、数据质检、Sim↔Real 一致率），本机自检、ruff 和浏览器全流程通过；未在服务器数据上运行。

- 2026-10-19：量杯乳白份额 0.08 → 0.25（用户反馈吧台任务里杯子看起来全透明）；吧台 seed 0 用 0.15/0.25 各跑一次均成功（89%/89%、87%/87%），材质只影响渲染，其余场景结果沿用。

- 2026-10-19：新增自建资产登记表 `real2sim/ASSETS.md`（物体、场景、机器人各属性的匹配等级与依据、共享库状态、待实测清单），README 资产规则指向它，修正 README TODO 里过时的相机内参描述。登记时发现自建物体的 `friction` 运行时不生效（见剩余事项）。只改文档；只读核对了服务器共享库，未跑运行时验证。

- 2026-10-17：新任务 `g1d_pour_beans_measuring_cups`（500 ml 量杯倒进 400 ml），吧台 seed 0/1/2 与办公室、实验室 seed 0 全部成功；关键是量杯手指下倾 20°（35° 时倾不过 89°）；修量杯/马克杯杯底凸包超出 GPU 限制。

- 2026-10-17：接入项目成员交付的带相机 Dex1-1 夹爪（v1_smooth_wrap：相机模组 + 光滑尾部 + 安装脚折边），代码 `scripts/g1d/dex1_camera/`，`build_asset.py` 默认生成；倒豆子脚本加避让（`LIP_IN/LIP_CLEAR` 0.010/0.035、`CUP_GRIP_UP` 0.065）。拾取/叠块三场景 6/6 成功，杯对杯 seed 0/2 和吧台/实验室 seed 0 成功、seed 1 退化，倒水与标准夹爪持平。共享库 `Robots/g1d` 已换（备份 `backup-shared-robots-g1d-20261017-161808.tgz`），共享库上叠块/拾取 seed 0 成功。

- 2026-10-17：头部相机合同位姿同步到共享库 `Robots/g1d/*_robot_config.yml`（只拷两个 yml，先备份）。用 HEAD 代码 + 共享库直接渲染（`agent-remote-runs/camsync-render-20261017-114947`），`g1d`/`g1d_counter`/`g1d_lab` 的 `g1d_stack_blocks` seed 0 均退出码 0、eval_time=1、success_rate=1.0；四路相机截图和头部同步前后对比在本机 `output/camsync/`。

- 2026-10-17：修正量杯材质能量损失（漫反射权重固定 1，只用透射权重控制乳白份额 0.08，粗糙度回到 0.3），杯子由发灰变为偏白。
- 2026-10-17：量杯再加雾：漫反射 0.15 → 0.2、粗糙度 0.3 → 0.45。注意：只改数字时文件大小可能不变，`rsync -a`（大小 + 时间）会漏传，向服务器同步脚本要加 `--checksum`；上一轮的 `cups_check_v4.png` 因此是旧材质。
- 2026-10-17：量杯镜面反射权重 1.0 → 0.1、粗糙度 0.2 → 0.3，去掉表面光泽（用户反馈真杯无反光）。
- 2026-10-17：新增单颗红豆网格 `make_red_bean.py` 和装豆渲染 `check_cup_beans.py`；量杯材质先由 OmniGlass 改为乳白 OmniSurface（用户反馈太透明），装豆后发现乳白会把豆子洗成灰色，定为漫反射 0.15。
- 2026-10-17：按照片新建三个塑料杯资产生成脚本 `make_plastic_cups.py`（白色马克杯、400/500 ml 透明方形量杯），Isaac 落地检查通过，未装入共享库。

- 2026-10-17：README 新增「协作开发」：各类资产的位置和归属（`real2sim/`、生成脚本、本机 `Assets/`、服务器共享资产库、隔离目录、HF 官方数据集）、新增资产流程、改共享资产库的五步流程（隔离验证 → 同意 → 备份 → 拷验证产物并 diff → 记 HANDOFF）、分支/提交/多人协作规则。只改文档，未跑运行时验证。

- 2026-10-16：三个场景各自成为 env 配置（`g1d`/`g1d_counter`/`g1d_lab`），`build_asset.py` 一次装好场景并生成布局；已装入共享资产库，叠块/拾取在三个场景都成功。

- 2026-10-16：自建资产从 `Assets/` 移到入库的 `real2sim/`（场景 1–3、倒水场景包、G1-D v4 URDF），README 写明 clone → 安装 → 下载资产 → 生成资产的顺序。

- 2026-10-16：Dex1-1 改为不带相机模型的标准版（`strip_wrist_camera`），16:14 已同步到共享库 `Robots/g1d`。

- 2026-10-16：新任务 `g1d_pour_beans_cup_to_cup`（两只塑料杯空中对倒红豆，左手倒），seed 0/2 成功、seed 1 失败；和 10-15 的壶倒杯 + 自建水杯改动一起提交。

- 2026-10-16：对比真机上的官方 G1-D URDF（运动学一致，腰部关节名在 v4 中互换），并用真机第 5/11 帧关节角渲染三路相机：真机腕部两路标签反了（图像服务端口相对采集互换，已知问题，部署端 SWAP_WRISTS）；之后用 rtx5090-180 的棋盘格内参重做：头部 fx 288→279.5（新类型 `g1d_head`，位姿不变），腕部改为宇树合同外参 + fx 314 + 3 cm 近裁剪，轮廓误差 0.4–2.5 px；先前自由拟合的 68° 作废（代码未提交；共享库 `Robots/g1d` 已同步，同步时误清空过 G1-D 布局并已恢复）。资产重组后在隔离叠加目录重跑 `build_asset.py` 并重渲。随后按用户指出的"真机能看到胸口"，头部改用合同位姿（后移 3.5 cm，挂 `head_link`）+ 10 cm 近裁剪，胸口与夹爪同时对上；共享库头部配置未同步。

- 2026-10-14：用四张照片重建实验室操作台 real_scene3（`Assets/sim_scene_lab/`，本机，gitignored），离线检查 19/19 通过，未在 Isaac 中验证。
- 2026-10-14：修复 `g1d_general_pickup` 与 `g1d_stack_blocks` 脚本抓取与叠块（TCP 雅可比 + 关节限位内缩投影、方块线性 PD 自动定心、侧向抓取手腕朝向肩部），远端验证两任务均拿到 `success_rate = 1.0`；倒水脚本改为双臂协同，倒水布局杯子拉近 9 cm。

- 2026-10-14：`README.md` 按 Notion「Real2Sim2Real」页面改写为中文项目说明（目标、现状、快速开始、TODO、月度目标、零样本上真机路线），保留上游致谢/引用/许可证。Notion 写"scripted_demo 已成功倒过一次水"，与本文件验证记录冲突，README 按验证记录写为三个演示均未成功。
- 2026-10-14：`Assets/scene3_pour_pack`（代码、场景/物体 USD、G1+Dex1 机器人 USD 与网格，约 102 MB，170 个 LFS 文件）入库，commit `9934da5`。
- 2026-10-14：`CLAUDE.md` 增加 Session Handoff 规则，新建 `AGENTS.md`（同一规则，并指向 `CLAUDE.md`）：编辑前读本文件，任务完成后更新并随改动一起提交。
- 2026-10-14：倒水任务改为倒 4 mm 红豆（颗粒粒子）；修复脚本演示误判后三个 G1-D 演示均为失败。
- 2026-10-13：按产品图与真机录像调整全身配色（小臂圆环/夹爪/腰部黑，其余哑光金属）和头部/腕部相机；URDF 转换结果缓存到资产目录。
- 2026-10-13：搭建服务器环境；接入 G1-D + Dex1（关节角控制），X5 与 G1-D 冒烟评测通过。
