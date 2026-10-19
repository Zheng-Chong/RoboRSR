# 办公室量杯全量倒豆采集

目标：办公室 `g1d` / `real_scene1`，500 ml 高量杯向 400 ml 矮量杯倒入所有红豆，采集 **100 个不同初始布局的合格回合**（倒豆的手左右各半、两杯偏航随机、位置随机），交付本地 LeRobot v3 数据集。

## 实现与兼容性

- 复用量杯资产和 `scripted_demo.cup_to_cup_actions` 的双臂 IK；采集入口把离杯目标设为 100%，单回合上限扩为 1200 步，结束后最多保持 60 步。
- 采集专用 `g1d_beans_collect` 配置保持办公室 `g1d` 的布局命名空间，将物理频率提高到 600 Hz，动作/观测仍为 30 Hz。1/300 秒对照中有 2/1840 颗最终落到杯底正下方；1/600 秒对照实现 1840/1840 入杯并保持 2 秒。
- 原任务、原演示及原宽松奖励不改。采集入口自行逐颗计数，最终结果由严格验收决定；不能把原评测的 success 当作本批数据合格证据。
- `trajectory.npz` 保存动作前的实测 state、实际发送的 action、动作后的豆数/逐颗位置/杯位姿，以及初末粒子位置。每臂 7 个关节弧度和 1 个夹爪开度，左臂在前、右臂在后。开度 0 表示闭合，1 表示张开；state 读两个指关节的实测均值，action 保存原始指令。
- 三路 MP4：`cam_head`、`cam_left_wrist`、`cam_right_wrist`，30 Hz，视频第 t 帧与 state[t] 都在执行 action[t] 前记录。`quality.json` 保存位姿、计数、帧数和验收结果。
- 全量判据：初始豆子全部在源杯；总数不变；最终源杯 0、目标杯 N、散落 0、双重归属 0。按实际锥形超椭圆内腔检查粒子中心，排除杯沿挂耳；不忽略任何散落颗粒。两杯轴线与竖直方向夹角 <30°（与原任务的回正定义一致），上述条件连续满足 30 帧。初始方案中额外的 5° 姿态和 0.02 m/s 粒子最大速度筛选已取消：侧握存在静态偏斜，PBD 杯内粒子有微振动；用户要求的 100% 转移条件始终保持不变。
- 导出在独立 LeRobot 环境运行，避免污染 Isaac 的依赖；使用官方 `LeRobotDataset.create/add_frame/save_episode/finalize` 和 H.264 视频，不上传 Hub。已验证的组合为 LeRobot 0.4.4、PyTorch 2.7.0、TorchCodec 0.4.0。

## 执行

需先准备完整 Isaac Sim 5.1 / IsaacLab / cuRobo 环境，以及 `Assets` 中的 G1-D、办公室、两个量杯、红豆材质与量杯布局。只有元数据的网页资产镜像不能运行仿真。

准备新布局目录（例如 seed 9）：倒豆的手按打乱的一半对一半分配（高杯在哪一侧就由哪只手倒），两杯 |x|、y 与偏航角做六维 Latin hypercube：高杯 |x| ∈ [0.05,0.22] m、矮杯 |x| ∈ [0.05,0.18] m、两杯 y ∈ [-0.46,-0.28] m、偏航 ∈ [0,90)°（方形截面每 90° 重复）。范围来自随机手和偏航的 64 回合探测，更外侧会掉杯或够不到。`scripted_demo.flat_turn` 让手腕在可行窗口（相对原 45° 向外 35°、向内 15°）内转到对准一对平面，窗口外取最近边界，指垫最多偏离平面 20°。默认 400 个候选，保留实际成功样本的位姿用于审计拒绝采样后的覆盖范围；右手倒的合格率略低，按手分别凑足一半。

```bash
python scripts/g1d/prepare_office_beans_layouts.py \
  --template Assets/Eval_Layout/RoboDojo/g1d/0/g1d_pour_beans_measuring_cups_0.json \
  --output Assets/Eval_Layout/RoboDojo/g1d/9 --count 400
```

先在独立 GPU 工作目录运行 1 回合，确认严格计数和三视角视频真实有效：

```bash
PYTHONPATH=.:XPolicyLab python scripts/g1d/collect_office_beans.py \
  --output output/office_beans/probe --attempts 1 --seed 0
```

`--attempts` 是尝试数，不是成功数，每个进程仍受任务原生评测数（50）的上限约束；大批量需把候选拆到多个 seed 目录。输出目录中的回合编号来自布局编号，不覆盖已有回合。不同 seed 使用不同输出目录。`--stop-file PATH` 允许协调进程在足数后通知采集器完成当前回合并停止。保留所有失败的诊断数据；不足 100 条时继续新增布局采集，不能复制成功回合凑数。

单回合确认后再运行 seed 9 的候选布局，使用独立的 `output/office_beans/raw/seed9` 输出目录。

多卡采集时每个进程使用不同 seed 和输出目录，并显式指定设备（例如可见所有卡时 `--device cuda:1 --device_id 1`）；采集入口禁用单进程跨卡渲染。`--device` 与 `--device_id` 是不同参数，评测入口已禁用参数缩写，避免 AppLauncher 预解析时混淆。

导出前必须已有 100 个不同位置的合格回合；导出器会检查数量、按 10 µm 精度去重位置、校验每路视频和轨迹帧数，然后回读数据集的首末帧：

```bash
# 在独立环境安装 lerobot==0.4.4 后
python scripts/g1d/export_office_beans.py \
  --raw output/office_beans/raw --output output/office_beans/lerobot --episodes 100
```

## 验证计划与当前边界

CPU：`python -m unittest scripts.g1d.test_office_beans`；覆盖杯内/杯壁/挂耳、坐标旋转、单粒残留/撒落、粒子消失和重复位置/样本不足拒绝。另做 ruff 和语法检查。

GPU：必须先验证单回合粒子状态回读、真实清空源杯、完整接住、图像与动作同步，再做批量和位置覆盖检查。导出必须经官方 LeRobot 加载器回读后才算完成。

初始实现尚未完成 GPU 验证；延长倾倒是否足够清空量杯仍待实验。已用独立的合成格式夹具验证三路视频、30 帧状态/动作的官方 LeRobot 写入与回读，夹具不在真实采集目录下，也不计入机器人数据。不能把脚本或格式检查完成解释为已经生成 100 条数据。
