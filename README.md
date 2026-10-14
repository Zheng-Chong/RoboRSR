<h2 align="center">基于 Multi-Agent 的 Real2Sim2Real 闭环自进化</h2>

<p align="center">Unitree G1-D（Dex1 夹爪）· Isaac Sim 5.1 / IsaacLab 2.3 · 基于 <a href="https://github.com/RoboDojo-Benchmark/RoboDojo">RoboDojo</a></p>

通过视频和多视角图像生成高保真 3D 资产与场景，在 Isaac Lab 中构建真实工位的仿真环境，利用仿真数据增强与 RL/IL 优化策略，再回到真实世界持续自进化。

本仓库是 RoboDojo 的私有分支，在其仿真与评测框架上接入了 G1-D 机器人、自建资产和 G1-D 专用任务。

**要端到端证明的一件事**：用仿真生成的轨迹训练策略，能在真机上零样本（zero-shot）成功。

## 现阶段已实现

**机器人本体**

- 两条 7 自由度手臂（coupled 模式）加 Dex1 两指夹爪，关节角控制，动作频率 30 Hz（`dt = 1/300 s`，每个动作 10 个物理步）。
- 头部双目相机加两个腕部相机，位姿按真机画面对齐；全身配色按产品图和真机画面核对。
- 轮子锁死，升降柱和腰部保持初始值，只做桌面操作。

**资产和物理**

- 程序化生成 3 款玻璃冷水壶（`scripts/objects/make_pitcher.py`），碰撞体保留壶内空腔和出水口。
- 流体倒水物理已验证：开口壶倒出 99.4%，封口对照保留 99.98%（`scripts/objects/check_pitcher_fluid.py`）。
- 倒水任务改为倒 4 mm 红豆（PBD 颗粒），开口壶可倒出 58.7%，封口对照保留 100%。
- 场景包 `Assets/scene3_pour_pack`（场景、物体、G1+Dex1 机器人 USD）已通过 Git LFS 入库。

**任务和脚本专家**

- 三个 G1-D 任务，放在重建的真实办公室场景 `real_scene1` 中：`g1d_pour_water_into_cup`、`g1d_general_pickup`、`g1d_stack_blocks`。
- `scripts/g1d/scripted_demo.py`：用仿真中的物体真值位姿和雅可比逆运动学写出抓取、搬运、倾倒路点，不需要策略服务器。目前三个任务的脚本演示**均未成功**（倒豆量不足、夹持失败），需要重新标定。

## 快速开始

基础环境（Isaac Sim、IsaacLab、cuRobo、官方资产）按 [RoboDojo 文档](https://robodojo-benchmark.com/doc/usage/install-and-download/) 安装，然后：

```bash
# 1. 拉取 LFS 资产（场景包里的 .usdc / .STL）
git lfs install && git lfs pull

# 2. 生成 cuRobo 配置（下载官方资产后必须运行一次）
python utils/update_embodiment_config_path.py

# 3. 构建 G1-D 机器人资产和评测布局（写入 Assets/Robots/g1d/、Assets/Eval_Layout/RoboDojo/g1d/）
python scripts/g1d/build_asset.py

# 4. 生成冷水壶资产
python scripts/objects/make_pitcher.py
```

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

## 仓库结构

在 RoboDojo 原有结构上，与本项目相关的部分：

```text
env/robot_manager/robot_config/g1d.py   G1-D 加载（直接读 URDF）、材质绑定
env/robot_manager/robot_class/g1d.py    G1DLeft / G1DRight 手臂类
env_cfg/g1d.yml                         G1-D 运行配置（30 Hz）
env_cfg/sim/g1d_sim_config.yml          仿真步长 1/300 s
env_cfg/camera/g1d_camera.yml           头部双目 + 双腕相机
scripts/g1d/                            资产构建、脚本专家演示
scripts/objects/                        冷水壶生成、刚体/流体物理检验
task/RoboDojo/{config,tasks}/g1d_*      G1-D 任务
Assets/scene3_pour_pack/                自建倒水场景包（Git LFS）
HANDOFF.md                              跨会话交接记录（当前状态、决策、验证、遗留问题）
```

## TODO

**本体标定**（已向宇树提工单；支持不了就人工棋盘格标定，本周完成）

- 相机内参目前是估计值（fx≈204、fy≈286，不像方像素），位姿是目测的。
- 关节执行器的刚度、阻尼和夹爪力都是拍定的，需要做系统辨识。
- 人工标定：拍棋盘格（双目和两个腕部相机）；手眼标定采集；给关节施加阶跃和正弦指令并记录响应；测夹爪开合时间和夹持力。
- 做一个"同一条轨迹在真机和仿真里各跑一遍再对比"的工具。
- 夹爪（含相机）建模：宇树没能提供与实际使用一致的 URDF。
- 待解决的问题：定制本体如果拿不到 URDF、关节参数和精确标定信息，后续迁移怎么做。

**环境匹配**

- 杯子、壶等资产的实物测量（称重、量尺寸）和 Agent 3D 建模。
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
| 碰撞体 | 与桌面、其他物体初始无穿插；容器口没被碰撞体封住（参照 `sim_scene_opus` 的射线检查） |
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

- 三个 G1-D 脚本演示目前都失败：倒豆需要更大倾角和更长保持（`step_lim` 可能不够），拾取和叠块的夹持需要排查（指尖开度、闭合力、接触）。
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
