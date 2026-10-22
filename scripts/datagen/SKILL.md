---
name: robodojo-datagen
description: 在 RoboDojo 里用脚本专家批量生成 G1-D 仿真示教数据并发布：布局分片、多卡采集（XPolicyLab 标准 HDF5）、官方转换器导出 LeRobot、独立审计、开环回放一致性检查、上传 ModelScope、登记 registry.json；也包括给新任务接采集插件、用本仓库评估外部训练的 checkpoint。用于：采一批数据、续跑采集、导出/发布数据集、检查数据和评估环境是否一致、给新任务写采集插件、评估外部模型。
---

# 仿真数据生成与评估

数据格式和语义以 `scripts/datagen/CONTRACT.md` 为准，本文件只讲怎么做。训练不在本仓库。

## 各层放在哪

| 层 | 位置 | 入库 |
| :-- | :-- | :-- |
| 任务登记 | `real2sim/tasks/<任务>/registration.json`：数采平台「登记任务」写的中文任务定义、指令转写要求（每种语言 N 条，中文 / 英文）、成功判据、物体角色和候选物体、干扰物、训练 / L4 保留场景、泛化等级和范围；任务抽屉里有给 coding agent 的实现任务文本。coding agent 从任务定义总结指令并转写，写进同目录的 `instructions.json`（`{"zh": [...], "en": [...]}`），`gen_instruction` 返回这些句子 | 是 |
| 流水线代码、任务插件 | `scripts/datagen/`、`scripts/datagen/plugins/<任务>.py`；专家动作在 `scripts/g1d/scripted_demo.py` | 是 |
| 数据集登记 | `scripts/datagen/registry.json` | 是 |
| 候选布局、训练分片 | GPU 服务器运行目录的 `Assets/Eval_Layout/RoboDojo/<配置>/<seed>/` | 否 |
| 原始回合 | 运行目录的 `output/datagen/<批次>/raw/seed<N>/` | 否 |
| 数据集 | 运行目录的 `output/datagen/<批次>/lerobot/<名称>/`，以及 ModelScope 私有仓库 | 否 |

服务器、环境和运行目录根路径以 `HANDOFF.md` 为准：采集一直在 `5090-2`，运行目录 `/data/robodojo-runs/<批次>/`；仿真环境 `/root/miniconda3/envs/RoboDojo`，导出环境 `/root/miniconda3/envs/robodojo-lerobot`（lerobot 0.4.4、h5py），上传环境 `/data/robodojo-runs/ms-venv`。

## 采集一批数据

复制这份清单，逐项完成：

```
- [ ] 1. 读 HANDOFF.md 和 CONTRACT.md；git pull --ff-only；开 worktree
- [ ] 2. 准备候选布局
- [ ] 3. 建运行目录
- [ ] 4. 单回合探针
- [ ] 5. 批量采集
- [ ] 6. 导出 + 审计
- [ ] 7. 回放一致性检查
- [ ] 8. 发布、登记、提交
```

1. **开工**：同一个工作区里常有别的会话在改，用自己的 `git worktree` 和短分支。
2. **候选布局**：用任务自己的布局脚本生成到一个新的 seed 目录。量杯任务用 `scripts/g1d/prepare_office_beans_layouts.py`（拉丁超立方采样），其他 G1-D 任务的布局来自 `scripts/g1d/build_asset.py`。候选要比目标多留余量，上一批的合格率是 92%。
3. **运行目录**：代码只传 git 跟踪的文件（`git ls-files`，排除子模块）；`XPolicyLab/`、`third_party/` 软链接到服务器上现成的副本；`Assets/` 用一份覆盖副本，只放这个任务要用的部分。**不要往服务器共享资产库写东西**，改共享库要按 README 的五步流程走。
4. **单回合探针**：

   ```bash
   PYTHONPATH=.:XPolicyLab python scripts/datagen/collect.py --task <任务> --env-cfg <配置> --seed <候选 seed> \
       --layout-ids 0 --output output/datagen/probe --device cuda:<卡> --device_id <卡>
   ```

   检查 `episode_000000.json` 里的 `accepted`、`frames`、`joint_names`，再用 XPolicyLab 的 `utils.data_loader.load(path, "RoboDojo", "v1.0")` 读 HDF5，确认图像能解码、state 和 action 的维度正确。
5. **批量采集**（放后台；SSH 断开会把前台进程一起杀掉）：

   ```bash
   nohup setsid python scripts/datagen/run_batch.py --task <任务> --env-cfg <配置> --candidates <候选 seed> \
       --shards 100-107 --gpus 0-7 --target 100 --output output/datagen/<批次> --revision <commit> \
       > output/datagen/<批次>.log 2>&1 &
   ```

   上传过去的运行目录不是 git 检出，要用 `--revision` 传入代码的 commit，否则数据集卡片上写的是 unknown。原始数据每个回合约 100 MB（三路 JPEG），100 条大约 10 GB。进度看 `status.json`，每个 worker 的日志是 `worker_<seed>.log`。中断以后用 `--output ... --resume` 续跑，已经录好的布局会跳过。合格数不够时，就生成更多候选布局，换一批新的 `--shards` 另起一批，不要复制合格回合来凑数。先 `nvidia-smi` 看清楚哪些卡空着，别人正在用的卡不要占。
6. **导出和审计**：

   ```bash
   /root/miniconda3/envs/robodojo-lerobot/bin/python scripts/datagen/export.py --run output/datagen/<批次> --episodes 100 --name <名称>
   python scripts/datagen/audit.py --dataset output/datagen/<批次>/lerobot/<名称>    # 仿真环境的 python 即可，不启动 Isaac
   ```

   `export.py` 选出通过验收、布局互不重复的回合，交给官方转换器，然后逐回合核对长度、指令和首尾帧。`audit.py` 从原始数据重新计算验收结果和布局覆盖，最后写出 `final_validation.json` 和 `sha256.json`。任一步失败都不能发布。
7. **回放一致性**：每个分片至少挑几个回合，先在采集用的配置里重放（应该和录制结果一致），再在打算评估的配置里重放，看偏差有多大：

   ```bash
   PYTHONPATH=.:XPolicyLab python scripts/datagen/replay.py --raw output/datagen/<批次>/raw/seed100 --layout-ids 0,1,2 \
       --report output/datagen/<批次>/replay_seed100.json [--env-cfg <评估配置>] --device cuda:<卡> --device_id <卡>
   ```

   报告里记录了每个 state 键的最大偏差、`accepted`（插件判据）和 `task_success`（任务判据）。
   - 在采集配置下，state 偏差应该在 1e-3 以内（10-19 实测 ≤ 1e-4 rad）。
   - 颗粒任务的豆子计数会有一两颗的出入，所以 100% 判据偶尔不过，这是正常的。
   - 偏差大，或者计数差很多，说明仿真不可复现，先查原因再发布。
   - 在别的配置下重放失败，说明不能用那个配置评估这份数据训练出的策略。
8. **发布和登记**：用 `upload.py check`、`upload`、`verify` 三步（在上传环境里，先 `modelscope login`；令牌不要出现在命令行或聊天里）。在 `registry.json` 加一条记录（状态、格式、验收、代码、布局、位置、发布地址、验证结果），更新 `HANDOFF.md`，一起提交。

## 给新任务接采集

在 `scripts/datagen/plugins/<任务名>.py` 写一个模块（文件名必须和任务名一致，`collect.py` 按任务名加载），参照 `g1d_pour_beans_measuring_cups.py`：

- 常量：`TASK`；`ACCEPTANCE`（验收版本号，判据一改就升版本）；`HOLD`（验收要连续保持的帧数）；`STEP_LIM`（单回合动作上限）；`TAIL`（专家动作放完之后，最后一个动作再保持多少步）。
- `Episode(env)`：
  - `reject`：开局检查不通过时写原因，否则为 None；
  - `actions()`：专家动作的迭代器，动作字典的键和 `obs["state"]` 相同；
  - `step()`：每执行一个动作后调用一次，记录审计用的数据，返回这一帧是否满足验收；
  - `privileged()`：写进 HDF5 的 `privileged/` 组的数组；
  - `summary()`：写进质检单的字段。
- `audit(privileged, quality)`：只用录下来的数据重新计算验收，不通过时抛出 `AssertionError`，返回审计指标。
- 判据要比任务的 `run_reward` 更严格，并且只用实际几何（参照 `scripts/objects/measuring_cup_shape.py`，任务代码和插件读同一个形状模块）。
- 在 `scripts/datagen/test_datagen.py` 加一个用合成数据跑 `audit()` 的用例。

## 用本仓库评估外部训练的 checkpoint

- 评测申请走数采平台「模型评测」：提交人填 checkpoint、部署代码、接口（本体、动作空间、图像、指令）、任务、场景、泛化等级和回合数，写到 `real2sim/evals/<编号>/request.json`；评测抽屉里有给 coding agent 的部署与评测任务。策略名用评测编号；跑完把运行目录的 `eval_result/` 同步到部署机的 `eval_result/`，并写 `real2sim/evals/<编号>/result.json`（`summary`、`runs`：每个任务 × env 配置 × 等级的回合数、成功率、95% 区间、专家对照，`issues`），平台据此显示结果。凭据不要写进这两个文件。
- 策略适配代码属于 XPolicyLab（子模块，不在本仓库里改）。LeRobot 格式的策略（ACT、SmolVLA、Pi0 等）读的是官方转换器的键名，只有按本契约导出的数据集能直接用。
- G1-D 目前只注册在本仓库的 `env_cfg/robot/_robot_info.json` 里。XPolicyLab 的 `utils/robot/_robot_info.json` 还没有 `g1d*`，走 XPolicyLab 的 `train.sh` 会拿不到动作维度，需要给 XPolicyLab 提 PR。
- 命令：`bash scripts/robodojo.sh eval --policy-dir XPolicyLab/policy/<策略> --task <任务> --ckpt <名称> --policy-env <环境> --env-cfg <数据集的配置> --action-type joint --eval-num <N>`。
  - 只用没有 train 标记的 seed 目录（评估客户端会拒绝带 train 标记的目录）。
  - 结果里同时报两种成功率：`success_rate` 是任务判据；严格判据可以对评估视频按插件重算，或者在同一组布局上和专家的回放结果对比。
  - 仿真每次运行有随机性，同一个布局至少重复跑 3 次，成功率要附上区间。

## 常见坑

- 选卡：`collect.py` 和 `replay.py` 都要用 `--device cuda:N --device_id N`，并且让 0 到 N 号卡都可见（`main.py` 已经禁止参数缩写，避免 AppLauncher 预解析时把两个参数混淆）。不要只设 `CUDA_VISIBLE_DEVICES=N`：这两个入口关闭了多卡渲染，渲染会落在 Vulkan 的 0 号卡上，而物理计算在 N 号卡，取相机数据时 replicator 会返回 None（10-19 回放时遇到过）。别人的卡显存快满时，用 `CUDA_VISIBLE_DEVICES=0,...,N` 把可见范围限制在自己用的卡上。
- 每个 worker 的 `ROBODOJO_RUN_ID` 要唯一（`run_batch.py` 已经处理了）。否则同一秒启动的评测会写进同一个 `eval_result` 时间戳目录，互相删掉对方的临时视频。
- 一个采集进程最多跑任务的 `eval_nums` 个布局（默认 50），剩下的由 `run_batch.py` 自动续跑。
- 官方转换器会把输入目录里的所有 `.hdf5` 都转换进去，并且会先删掉同名的输出目录。所以只能通过 `export.py` 调用它：它只暂存选中的回合，输出目录已存在时直接拒绝。
- 官方转换器按自己 `__file__` 解析后的路径来定 `data/` 和 `env_cfg/` 的根目录。服务器运行目录里的 `XPolicyLab` 通常是软链接，解析后会指到另一个检出，结果去那边找数据（报 No matching targets）、读那边的 `env_cfg`。`export.py` 已经把这两个根目录固定为本仓库。
- 服务器访问 PyPI 要走镜像：`pip install -i https://pypi.tuna.tsinghua.edu.cn/simple ...`。
- 颗粒任务用 `g1d_beans_collect`（1/600 s）。
- 向服务器同步代码时，`rsync` 要加 `--checksum`：只改了数字时，文件大小和修改时间可能都没变，会被漏传。
- 服务器上的 `ffmpeg` 如果不带 libx264，录评估视频时会报 BrokenPipe；仿真环境的 `bin` 里要链接系统的 `ffmpeg`。
