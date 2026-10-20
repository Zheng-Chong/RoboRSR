---
name: real2sim-object-asset
description: 按 Real2Sim 数采平台的物体登记（real2sim/objects/<id>/spec.json 和实拍照片）给实物孪生建 RoboDojo 刚体资产：写参数化生成脚本，生成 object.usdz / metadata.json / description.json，在 GPU 服务器隔离目录验证，按登记的尺寸名称回报 measurements_mm，并更新登记表。用于：给平台登记过的实物建模或按实测重建、处理平台物体页给出的「建模任务」、修改 scripts/objects/ 下的生成脚本。
---

# 实物孪生物体建模

平台负责登记（实测值和照片），这里负责把登记变成仿真资产。本流程针对刚体（`Rigid`）；关节物体、衣物、流体等其他类型先问用户。

## 先分清改的是哪一层

| 层 | 位置 | 入库 | 这一步里怎么用 |
| :-- | :-- | :-- | :-- |
| 登记（输入） | 部署机 `<仓库>/real2sim/objects/<id>/`：`spec.json`、`reference/*.jpg` | 拷回本机后入库 | 只读；同步到本机，随脚本一起提交 |
| 生成代码 | `scripts/objects/make_*.py`；任务也要用的几何放 `scripts/objects/<类别>_shape.py`（只依赖 numpy） | 入库 | 新建或修改 |
| 生成结果 | `Assets/Object/RoboDojo/<id>/`（`object.usdz`、`metadata.json`、`description.json`） | 不入库 | 只由脚本写，不手改；不要往 `Assets/` 放要入库的文件 |
| 渲染预览 | `real2sim/objects/<id>/preview/sim.png` | 不入库 | `check_object.py --image-dir` 生成，同步到部署机给平台显示 |
| 验证 | `5090-2` 的隔离目录 `/data/robodojo-runs/<名称>-<时间>/` | 不入库 | 生成到隔离目录的覆盖目录里再检查 |
| 共享资产库 | 21196 的 `/data2/zhongzheng/Project/RoboDojo_assets/Assets`（只当资产来源，不在 21196 上跑） | 不入库 | 只有用户同意后，按 README「改服务器共享资产库」装 |
| 登记表 | `real2sim/ASSETS.md`、README「快速开始」第 4 步、`HANDOFF.md` | 入库 | 更新 |

不在这一步改：任务配置和布局（`task/RoboDojo/`、`scripts/g1d/build_asset.py` 的 `G1D_TASKS`），除非用户要求；子模块 `XPolicyLab/`、`third_party/`。

`<id>` 是 `<类型>/<类别>/<5 位编号>`，如 `Rigid/measuring_cup/00002`。编号由平台登记时分配，资产目录、照片目录、生成脚本里的编号都用它，不要另起。

机器名、路径这类会变的信息以 `HANDOFF.md`「服务器」为准；数采平台的运维细节（部署机仓库、资产库镜像命令等）在本机 `.agent/ops.local.md`，仓库是公开的，这些不入库，本机没有这个文件就问用户。下文写的是写这份说明时的状态。

## 流程

复制这份清单，逐项完成：

```
- [ ] 1. 读 HANDOFF.md，从 origin/main 开 worktree 短分支
- [ ] 2. 取登记：spec.json + reference/
- [ ] 3. 定几何方案，写下每个尺寸的定义
- [ ] 4. 写或改生成脚本（含自检、measurements_mm）
- [ ] 5. 本机静态检查
- [ ] 6. GPU 服务器隔离目录：生成 + check_object
- [ ] 7. 核对 measurements_mm 与实测
- [ ] 8. 渲染图对照实拍照片
- [ ] 9. 登记、提交、交付说明
- [ ] 10. （用户同意后）装共享资产库，刷新平台
```

### 1. 开工

按 `CLAUDE.md`：先读 `HANDOFF.md`（GPU 服务器、共享库、各资产的当前状态都在里面），然后从 `origin/main` 开一个自己的 `git worktree` 和短分支（`<名字>/<范围>-<简述>`，如 `chongzheng/objects-paper-cup`）。主工作区有好几个会话共用，共用暂存区曾把一个会话的文件提交进另一个会话的 commit。别人未提交的改动不要碰。

### 2. 取登记

登记在平台所在的部署机上（SSH 别名和仓库路径见 `.agent/ops.local.md`，当前是 `5090-2:/root/RoboDojo`）。同步到本机：

```bash
rsync -az --exclude preview/ <部署机>:<仓库>/real2sim/objects/<id>/ real2sim/objects/<id>/
```

`spec.json` 字段（空字段不出现）：

| 字段 | 含义 |
| :-- | :-- |
| `id`、`name` | 资产 id、名称 |
| `tasks` | 任务配置里已经用到这件物体的任务；没有就是新物体 |
| `material`、`color` | 材质、颜色（自由文本） |
| `mass_g` | 实测质量，g |
| `static_friction` | 实测静摩擦系数（斜面法），可能没有 |
| `dimensions_mm` | 实测尺寸原文，单位 mm，每项「名称 数值」，如 `开口直径 82，底部直径 64，高 135` |
| `features` | 外观与结构（透明度、把手、刻度、纹理、壁厚等） |
| `requirements` | 建模要求（如中空能装豆子、把手可抓、刻度不用建模） |
| `notes` | 备注，常有补充的实测值 |
| `updated_by`、`updated_at` | 最后修改人和时间，写进脚本 docstring 作为测量来源 |

- 没有 `spec.json`：物体还没登记成实物孪生。让用户先在平台登记，不要自己编测量值。
- 尺寸和质量以 `spec.json` 为准（`dimensions_mm` 和 `notes` 里的实测值都算），照片只用来定形状、比例和外观。都没给的尺寸按照片估，在脚本注释和 `ASSETS.md` 里标「估计」。
- `notes` 和 `dimensions_mm` 矛盾时，问用户。

### 3. 几何方案

- 给 `dimensions_mm` 里的每个名称写清定义：外径还是内径、对边还是对角、含不含把手或翻边。把定义写在脚本常量的注释里，第 4 步按同样的定义去量模型。
- 选构造方式，从最接近的现有脚本改：
  - 回转体（圆杯、碗）：`make_water_cup.py` 的 `lathe`
  - 截面沿高度渐变（圆角方形、超椭圆）：`make_plastic_cups.py` 的 `loft`、`wall_staves`，以及 `measuring_cup_shape.py`
  - 把手、提梁：`mp.sweep`；平板、挂耳：`mp.prism`；圆角：`mp.arc`
  - 小颗粒（豆子）：`make_red_bean.py`
- 任务代码也要用这件物体的几何时（如判断豆子在不在杯里），把尺寸和截面放进只依赖 numpy 的形状模块，生成脚本和任务都从这里读，保证两边一致（参照 `measuring_cup_shape.py`）。

### 4. 生成脚本

**改哪个文件**：这个类别已有生成脚本（平台物体页「生成脚本」一栏，或 `rg -l 'Rigid/<类别>|"<类别>"' scripts/objects`）就改那个；新类别新建 `scripts/objects/make_<类别>.py`。同一族物体放一个脚本，参数表和编号一一对应（如 `measuring_cup_shape.MEASURING`）；运行时会把整族都重新生成一遍。

**约定**（新脚本照 `make_plastic_cups.py`，它是最新的写法）：

- docstring：真实对应物和测量来源（spec 的 `updated_at`）、Usage、坐标系、USD 结构。
- 命令行：`python scripts/objects/make_<类别>.py [TYPE_DIR]`，默认 `Assets/Object/RoboDojo/Rigid`，写到 `<TYPE_DIR>/<类别>/<编号>/`。不要把共享库路径写死。老脚本 `make_pitcher.py`、`make_water_cup.py` 的参数是类别目录，改它们时保持原样。
- 依赖：numpy、scipy、pxr。用 `import make_pitcher as mp` 取共享 USD 工具（`add_xform`、`add_mesh`、`add_material`、`arc`、`sweep`、`prism`、`vertex_normals`、`signed_volume`、`r4`）；直接导入不了 pxr 时它会启动 Kit。脚本结尾照 `make_plastic_cups.py` 处理 `mp.APP`：出异常时 `os._exit(1)`，否则 `APP.close()`。
- 单位：spec 用 mm、g，脚本常量用 m、kg；每个常量注释标「实测」或「估计」。
- 坐标系：Z 向上，单位米，原点在包围盒中心（生成后整体平移 `-(lo + hi) / 2`，见 `save()`）。有把手或壶嘴的物体，把手朝 +X，和官方 `mug` 一致。
- USD 结构（照 `make_plastic_cups.py` 的 `write_usdz` / `add_collision`）：
  - `/root`：defaultPrim，`RigidBodyAPI`，上轴 Z，`metersPerUnit = 1`
  - `/root/visual/model`：主体，必须是层级里第一个 Mesh（加载器的 `get_bbox`、`sample_mesh_vertices` 取第一个 Mesh），其他可见部件放 `/root/visual/<名称>`
  - `/root/_materials/<名称>`：材质
  - `/root/collision/hull_NN`：显式凸包，purpose 设 `guide`，渲染不可见，`CollisionAPI` + `MeshCollisionAPI(convexHull)`
- 碰撞体：容器要空心，用杯底一块 + 杯壁分片（`wall_staves`），开口不能被封住。每个凸包 ≤ 64 个顶点、单个面 ≤ 32 个顶点，超出 PhysX GPU 限制就不能和颗粒碰撞，豆子会穿过去；底板多边形用 24 边（`BOTTOM_SIDES`）。
- 自检（脚本里 assert，不通过不写文件）：每个可见部件 `signed_volume > 0`（绕序朝外）；凸包不侵入空腔（`cavity_probe` + `self_check`）。容器打印容积；spec 写了容量（刻度、满口毫升数）就对一下。
- 材质：不透明用 OmniPBR；半透明塑料用 OmniSurface（漫反射权重固定 1，只用透射权重调乳白程度，粗糙度 ≤ 0.35，原因见 `clear_pp` 的注释）；透明玻璃用 OmniGlass。按 spec 的材质、颜色和照片调，目测接近即可。
- uuid：`uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/{类别}/{编号}")`，编号用整数。
- `description.json`：`uuid`；`caption` 下 `color`、`material`、`size`、`name`、`shape`、`function` 各 3 个英文短语；`description` 是 5 句英文描述。

**`metadata.json`**：照 `make_plastic_cups.py` 的 `metadata()` 从网格计算（下面只是结构示意，数值保留 4 位）。

```json
{
  "uuid": "...",
  "physics": {"mass": 0.045, "size": [0.082, 0.082, 0.135]},
  "measurements_mm": {"开口直径": 82.1, "底部直径": 64.0, "高": 135.0},
  "geometry": {"faces": "...", "vertices": "...", "radius": "...", "bbox": "...", "center": "...",
               "aligned_bbox": {"vertices": "8 个角点", "extents": "..."}, "oriented_bbox": "同上"},
  "active": {"place": {"up": {"orientation": [1, 0, 0, 0], "projection_circle": "...", "contact_circle": "..."}},
             "functional": {}},
  "passive": {"functional": {}, "support": {}}
}
```

- `physics.mass` = `mass_g / 1000`。运行时会把质量截到 ≤ 0.5 kg（`env/scene_manager/objects/rigid.py`），超过的话在 `ASSETS.md` 记一笔。
- **摩擦**：运行时只读 `physics.static_friction` / `dynamic_friction`（不写时默认 0.6 / 1.5）；老脚本写的 `friction` 不生效。spec 有静摩擦才写这两个键：`static_friction` 用实测值，`dynamic_friction` 没测就取同一个值并标「估计」。没有实测就都不写，`ASSETS.md` 里标「默认」。已有任务的成功率都是在默认摩擦下调出来的，改摩擦前先告诉用户。
- **`measurements_mm`**（现有脚本的 `metadata()` 都还没写，要自己加）：
  - 键只取 `dimensions_mm` 里的名称，一字不差（平台按名称核对，对不上就核对不了）；`notes` 里的其他实测值用来建模，但不放进这个字典。
  - 值在 `save()` 平移之后，从生成的网格按第 3 步的定义量出来，不要直接抄常量；只取对应部件的顶点（比如外径只用杯身，不含把手）。单位 mm，保留 1 位小数。
  - 加载器只读 physics / visual / geometry / active / passive，这个键不影响仿真。
- `oriented_bbox`：`env/scene_manager/layout_manager.py` 的 `get_instance_bbox_vertices` 优先取它的 8 个角点（缺了才退回第一个 Mesh 的包围盒），奖励判定（在不在另一物体的包围盒里、叠放等，`env/reward_manager/func_parser.py`）都经它取包围盒。要包住把手、翻边等全部部件。
- `active.functional`：能抓的把手写抓取帧，照 `make_plastic_cups.py`：`build_mug()` 返回的 `grip` 点，在 `metadata()` 里写成 `functional.handle.frame`（和官方 `mug` 一样的 +X 竖直把手）。没有就留空。

### 5. 本机静态检查

```bash
bash scripts/check.sh     # 和 CI 一样：改动过的 Python 文件过 ruff（0.11.7，行宽 120），再跑单测和任务清单检查
python -m py_compile scripts/objects/<脚本>.py
```

本机没有 pxr，不在本机生成 USD。有 numpy 和 scipy 的话，可以把 pxr 打桩，只跑几何构建和 `self_check`，提前发现绕序和空腔问题。

### 6. GPU 服务器隔离目录：生成 + 落地检查

- 服务器：一律用 `5090-2`（Isaac Sim 5.1），隔离目录放 `/data/robodojo-runs/<类别>-<YYYYMMDD-HHMMSS>/`；**不要再在 21196 上跑**。环境怎么激活看 `HANDOFF.md`「服务器」和本机 `.agent/remote-validation.local.toml` 的 `[environment]`。这个文件不入库，连接信息也不要写进仓库。共享资产库只在 21196，需要的官方资产按 `HANDOFF.md` 里的命令从本机中转进隔离目录的 `Assets/`。
- 把当前工作树传进隔离目录，排除 `Assets/`、`.git/`、`output/`、`eval_result/`；子模块不要上传，改成软链接到 `5090-2` 上现成的副本（位置见本机 `.agent/remote-validation.local.toml`）。生成结果写到隔离目录的 `ovl/` 里，**不要直接写共享资产库**。`check_object.py` 只需要物体目录本身；要跑任务或脚本演示，才按 README「改服务器共享资产库」第 1 步建完整的 `Assets/` 覆盖目录。
- 用 `nvidia-smi` 挑一张空闲的卡：

```bash
python scripts/objects/<脚本>.py ovl/Rigid      # 老脚本 make_pitcher.py、make_water_cup.py 传类别目录 ovl/Rigid/<类别>
CUDA_VISIBLE_DEVICES=<卡> PYTHONPATH=. python scripts/objects/check_object.py ovl/Rigid/<类别>/<编号> --image-dir real2sim/objects
```

- `check_object.py` 判 OK 的标准：倾角 < 5°、落地后中心高度与半高差 < 5 mm、速度 < 0.01 m/s，退出码 0。FAIL 时先查原点是否在包围盒中心、底面碰撞体是否平。细长、球形、会滚的物体（如红豆）倾角判据可能不适用，要写明原因，不能直接当成通过。
- 容器要装颗粒时（`requirements` 写了装豆或装水，或任务里要倒），再跑 `check_cup_beans.py` 或 `check_pitcher_fluid.py`。这两个脚本只接受 `--device cpu`（默认就是），GPU 状态管线下粒子坐标不回读，结果无效。
- 预计跑十几分钟以上的用 `nohup setsid` 放后台，SSH 长连接会断，进程会被一起杀掉。

### 7. 尺寸核对

- 读隔离目录里生成的 `metadata.json`，对 `dimensions_mm` 里的每个名称：|`measurements_mm` − 实测| ≤ max(2 mm, 2% × 实测)。质量直接用实测值。
- 超差就改脚本常量后重新生成，**不要改测量值**。怀疑测量本身有问题，问用户。
- 列一张表放进交付说明：名称 / 实测 / 模型 / 误差。

### 8. 外观

- 把 `preview/sim.png` 和 `reference/` 里的照片并排看：轮廓比例、颜色、透明度、把手位置。差得明显就调几何或材质。
- 把 `preview/` 同步回本机和部署机的同一路径（平台读部署机上的），这个目录不入库。

### 9. 登记、提交、交付说明

- **提交**：
  - 生成脚本（和形状模块）
  - `real2sim/objects/<id>/spec.json` 和 `reference/` 照片（从部署机拷来的原件，图片自动走 LFS）
  - 新脚本或新物体：README「快速开始」第 4 步加上生成命令
  - `real2sim/ASSETS.md` 新增或更新一行：真实对应物、生成脚本，尺寸、质量、摩擦、外观各自的匹配等级和依据（spec 里的值算「实测」，照片估的算「估计」），共享库状态
  - `HANDOFF.md`
- **不提交**：`Assets/` 下任何生成结果、`preview/`、隔离目录里的东西。
- 在第 1 步开的 worktree 短分支上提交，`git add` 逐个列出文件，提交前跑 `bash scripts/check.sh`，然后推分支、开 PR，等 CI 通过再合并进 `main`。
- 提交信息：`[scripts] feat: add <类别> twin asset from platform registration`（重建用 `update`）。
- 本机 `Assets/` 里生成后，可以按 README 跑 `format.py` 刷新 `uuid_map.json`。运行时不读这个文件，隔离目录和共享库都不需要跑。
- **交付说明**：资产 id、生成脚本、尺寸对照表、`check_object.py` 输出和渲染图路径、哪些值是估计的、装没装共享库。平台物体页的「物理检验」由人勾选，把对应的检查结果写清楚，方便他们勾。

### 10. 共享资产库（必须先得到用户同意）

- 按 README「改服务器共享资产库」五步：隔离目录验证 → 征得同意 → 备份要覆盖的部分 → 拷贝**验证过的产物**（不在共享库上重新生成）并 `diff -r` 确认 → 在 `HANDOFF.md` 记下改了什么、备份在哪。
- 只拷目标 id 的目录。一个脚本会生成整族物体，同族的其他编号不要顺手覆盖。
- 新 id 不会覆盖已有目录；重建已有物体时先备份原目录。
- 装好之后：`ASSETS.md`「共享库」一栏改成「已装」；在本机重跑 `.agent/ops.local.md` 里的元数据镜像命令，刷新部署机的资产库镜像，再在平台点「同步项目数据」。平台会按 `measurements_mm` 自动核对尺寸，物体页的「尺寸 / 质量核对」应当变成通过。
- 没装共享库、但要让平台先看到：把 `metadata.json`、`description.json` 手动放进部署机镜像的同一 id 目录，并记进 `.agent/ops.local.md`。镜像命令会整体替换镜像目录，下次重跑时手动放的文件会被删掉。

## 重建已有物体

`spec.json` 的 `tasks` 不为空，或资产库里已经有这个 id 时：

- 改现有生成脚本的常量，id 不变，不另起脚本。
- `build_asset.py` 生成布局时读物体的 `metadata.json`，尺寸变了，布局和 `scripted_demo` 的抓取参数可能都要跟着重跑。这属于任务层，先告诉用户，由用户决定要不要做。

## 常见坑

- 单位：spec 是 mm、g，`metadata.json` 是 m、kg；只有 `measurements_mm` 用 mm。
- `measurements_mm` 的名称和 `dimensions_mm` 不一致，平台就核对不了；外径量进了把手，会判成超差。
- 原点不在包围盒中心：落地检查 z 偏差、布局摆放错位。
- 凸包顶点超限：颗粒（红豆）穿过碰撞体。
- 只写了 `friction`：摩擦不生效。
- 质量超过 0.5 kg：运行时被截断。
- 把要入库的东西放进 `Assets/`：`init_assets.sh` 发现目录不完整时会把整个 `Assets/` 挪走。
