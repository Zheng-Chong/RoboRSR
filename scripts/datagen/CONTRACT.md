# G1-D 数据契约

本仓库生成仿真数据、在本仓库做仿真评估、在外部训练模型，三方共用这份约定。改其中任何一条，都要同时改代码、本文件和 `registry.json`，放在同一个提交里。

## 文件

**原始回合**（`scripts/datagen/collect.py` 写出）：一个回合对应一个 `episode_<布局号>.hdf5`，格式是 XPolicyLab 标准轨迹格式（XPolicyLab README「Trajectory Data Format」）；同目录下有一个同名的 `.json` 质检单。

| HDF5 键 | 内容 |
| :-- | :-- |
| `data_format_version` | `v1.0` |
| `instruction` | 评估环境给策略的那句指令（任务的 `gen_instruction`），录制时原样取 `obs["instruction"]` |
| `additional_info/` | `frequency`（30）、`env_cfg`、`physics_dt`、`layout` |
| `state/left_arm_joint_states` 等 | (T, D) float32，取自评估时的 `obs["state"]` |
| `action/` | 与 `state/` 同名，取自发给 `take_action` 的动作 |
| `vision/cam_head/colors` 等 | 用 `encode_image_bit` 编码的 JPEG，存成定长字节列 |
| `privileged/` | 我们自己加的扩展字段，只给审计用（例如豆子位置、杯子位姿），训练时不要读 |

质检单字段：`accepted`、`acceptance_version`、`hold_frames`、`stable_frames`、`task_success`、`env_cfg`、`physics_dt`、`layout_seed`、`layout_id`、`layout_sha256`、`planned_poses`、`state_keys`、`joint_names`、`revision`，以及插件附加的字段。

**发布格式**：LeRobot v3.0，只能由 XPolicyLab 官方转换器 `XPolicyLab/scripts/transform_lerobot_v30_format.py` 生成（`export.py` 负责调用它）。键名：`observation.state`、`action`、`observation.images.cam_high`（对应 `cam_head`）、`observation.images.cam_left_wrist`、`observation.images.cam_right_wrist`；`robot_type` 是 `unified_robot`，电机名为 `left_joint_0..7`、`right_joint_0..7`；视频为 AV1。

## 观测与动作

- **16 维向量的顺序**：左臂 7 个关节、左夹爪、右臂 7 个关节、右夹爪。左右臂各自的关节顺序以质检单里的 `joint_names` 为准；G1-D 是 shoulder_pitch、shoulder_roll、shoulder_yaw、elbow、wrist_roll、wrist_pitch、wrist_yaw。关节角单位是 rad。
- **夹爪 state**：两指关节实测位置的均值，换算到 0（闭合）到 1（张开）。被物体挡住时，读数停在物体宽度对应的开度上，不裁剪。由 env 配置里的 `observation.robot.ee_state: measured` 打开，G1-D 的五个配置（`g1d`、`g1d_counter`、`g1d_lab`、`g1d_school`、`g1d_beans_collect`）都已打开。不写这个键时仍是上一步的指令值，这是 RoboDojo 原来的行为，X5 等任务不受影响。
- **夹爪 action**：指令值，刻度同上，执行前裁剪到 [0, 1]。
- **机械臂 action**：关节位置目标，30 Hz。仿真在每个动作内部做插值，与真机控制器的跟踪方式不同（见 README「控制对齐」）。
- **帧对齐**：第 t 帧的图像和 state 都是在执行 `action[t]` 之前采集的。
- **相机**：`cam_head` 是头部双目的左眼（640×480，fx ≈ 279.5）；`cam_left_wrist`、`cam_right_wrist` 是左、右腕相机（640×480，fx ≈ 314）。都按针孔模型渲染，没有加畸变，腕部主点的偏移也没有建模。

## 仿真配置

- 每个回合都把 env 配置名写在 `additional_info/env_cfg`、质检单和数据集卡片里。**用这份数据训练出的策略，评估时必须用同一个 env 配置**。XPolicyLab 的 checkpoint 目录名里带 `env_cfg_type`，配置用错时会找不到 checkpoint。
- 物理步长：`g1d`、`g1d_counter`、`g1d_lab`、`g1d_school` 是 1/300 s；`g1d_beans_collect` 是 1/600 s，专门给颗粒任务用。颗粒任务不要在 1/300 s 下评估：每步最大位移 8.3 mm，已经超过“豆径 + 杯底厚”的 7.6 mm（判据见 `env_cfg/sim/g1d_beans_collect_sim.yml` 的注释）。
  - 2026-10-19 用 `replay.py` 实测：在 `g1d_beans_collect` 下重放两条录制的量杯回合，机械臂 state 的最大偏差 ≤ 1e-4 rad；同一组动作放到 `g1d` 下，两条各洒出 57–59 颗豆子（约 9%），连任务自身的判据都不通过。
- 环境是否一致，用 `replay.py` 检查：把录下的动作在目标 env 配置里原样重放，对比 state 偏差和验收结果。
- 颗粒仿真不是逐位可复现的：同一配置、同一组动作，上面两条回放里有一条最后剩 1 颗豆子在源杯，没通过 100% 的严格验收（另一条通过）。判断环境是否一致，要看 state 偏差和豆子计数；严格判据本身的方差，要靠同一布局重复评估来估计。

## 布局划分

- `run_batch.py` 会在采集用的 seed 目录里写入 `split.json`（`"split": "train"`）。评估客户端遇到这种目录会直接拒绝，除非加 `--allow_train_layouts`；这个参数只给采集和回放用。
- 评估用的是没有 train 标记的 seed 目录。每份数据集用了哪些布局，记在 `registry.json` 里。

## 两种成功

- **评估成功**：任务自身的 `run_reward`。例如量杯任务的判据是：源杯回正时，至少 60% 的豆子离开源杯，其中至少 90% 进了目标杯。`_result.json` 里的 `success_rate` 就是按它算的。
- **数据验收**：插件定义的严格判据，版本号写在质检单的 `acceptance_version`。例如 `all_beans_retained_v2`：100% 的豆子进目标杯，两只杯子都在 30° 以内回正，并保持 30 帧。录制期间任务判据有没有触发过，记在质检单的 `task_success` 字段。

## 对应真机（sim2real 之前必须逐项核对）

- **夹爪**：真机遥操记录里，夹爪读数 0 是全闭、5.4 是全开，除以 5.4 就换到本契约的刻度。这个读数和指尖行程是否成线性，还没有标定过。
- **腕部相机**：`cam_left_wrist` 永远指左手相机。真机图像服务的 `left_wrist_camera` 流实际是右手相机，部署客户端默认设了 `SWAP_WRISTS=True`；2026-09-21 之前录的真机数据，左右是反的。
- **腰部关节**：v4 URDF 把官方 `torso_Joint`（腰俯仰）和 `Yaw_Joint`（腰偏航）的名字互换了。这两个关节不在 16 维向量里，但回放真机全身状态时要按官方名字映射。
- **头部图像**：真机是 1280×480 的左右拼接图（SBS），取左半边对应 `cam_head`。是裁剪还是缩放得到的，还没确认。

## 旧数据

`registry.json` 里标为 legacy 的数据集，是在这份契约之前导出的（2026-10-19 的 100 条量杯数据）。和本契约相比：
- 头部图像键叫 `cam_head`，不是 `cam_high`；`robot_type` 和电机名也不同。
- 指令是一句自定义的话，和评估时给策略的指令不同。
- 夹爪 state 同样是实测值。

用这份旧数据训练的策略，评估前要先把这几处对齐。
