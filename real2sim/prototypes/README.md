# 极光 Lite 与柔性物体研究资产

这里接入已有的极光 Lite 外形参考、二维布片和体积软块原型，统一记录来源、
单位、后端、参数与验证状态，入口为 [catalog.json](catalog.json)。这些原型没有
登记为可运行任务，也没有修改现有 G1-D/Dex1、相机、布局、观测或控制接口。

| 资产 | 本目录 提供 | 当前边界 |
|---|---|---|
| 极光 Lite | 官方 STEP 的本地下载/哈希校验/转换工具；USD 和 MuJoCo 静态外形 | 未划分运动连杆、标定安装、碰撞、惯量、指垫或视触觉 |
| 布片 | 可配置二维 MuJoCo flex；同拓扑的 USD 三角网格 | 边长约束代理，没有标定弯曲、剪切或经纬各向异性 |
| 软块 | 三维弹性 MuJoCo flex 测试场景 | 候选杨氏模量/泊松比，不是实测海绵、硅胶或软果 |
| 衣服、丝绸、开口袋 | 记录接入与校准要求 | **尚未实现**缝合拓扑、多层接触或对应实物孪生 |

## 极光 Lite：可复现外形参考

来源：[Xense Robotics 官方模型页面](https://docs.xenserobotics.com/docs/latest/hardware/gripper-models/)。
页面将模型定位为布局、安装和干涉检查参考，不是完整机器人描述。
[source.json](aurora_lite/source.json) 保留官方 URL、79,988,664 字节和 SHA256。
**未找到允许再分发该 CAD/派生网格的明确许可，因此仓库不包含它们或厂商图片，
也不将厂商文件重新标为本仓库 MIT。** 使用者按厂商条款自行获取，脚本只在本地转换。

在独立 Python 3.12 CAD 环境安装 `cadquery-ocp==7.7.2`，从仓库根目录运行：

```bash
python scripts/prototypes/import_aurora_lite.py --download \
  --output .cache/prototypes/aurora_lite
# 已经有官方原件时，使用 --step /path/to/aurora-lite-gripper.STEP 替代 --download。
```

输入尺寸/哈希/STEP 结束标记/毫米声明不匹配即拒绝。输出目录必须是新目录。
原 CAD 原点保持不动，毫米统一转米；网格分块只为 STL 面数上限，**不代表关节连杆**。
输出 `geometry.usda`、`reference.xml`、`meshes/` 和 `conversion.json`；均留在忽略的
`.cache/`。USD 没有 Physics/Collision API；MJCF 没有关节/执行器且关闭碰撞。
不要把整个外壳凸包用作夹爪碰撞，也不要据 CAD 原点猜测 G1-D 安装变换。

## 布片：一个参数源、两个明确区分的输出

```bash
python scripts/prototypes/textile_fixture.py --output .cache/prototypes/cloth_patch
# --config accepts a JSON object with ClothConfig fields (SI units).
```

默认 16×16 cm、9×9 顶点、128 三角形，候选面密度 0.2 kg/m²，总质量 5.12 g，
接触厚度 1 mm。输出 `model.xml`（含地面、初始高 0.1 m）、`asset.json` 和
`geometry.usda`（仅位于 z=0 的局部网格，米、Z-up；没有地面和物理 API）。

RoboRSR 的 Isaac Sim `env/scene_manager/objects/garment.py` 已支持引用 USD Mesh，
该 USD 是后续接入 `GarmentObject` 的几何输入；**MuJoCo 材料参数不能直接映射为
Isaac 粒子弹簧参数**。本目录 没有注册 Garment 布局、任务或宣称 Isaac 抓取通过。
生成器还写出 `isaac_instance.json`：显式以总质量/顶点数设置粒子质量，避免81个粒子误用默认10g得到810g。Isaac弹簧、弯曲、剪切参数是独立求解候选，不宣称与MuJoCo或丝绸等价。

软块的 `discrete` 积分器只属于独立 MuJoCo 夹具，不能覆盖已有机器人环境。

## 独立数值验证

```bash
# 在独立环境安装 scripts/prototypes/requirements.txt，不改变 Isaac/policy 环境。
python scripts/prototypes/check_deformable.py \
  --model real2sim/prototypes/cloth_patch/v1/model.xml \
  --output .cache/prototypes/cloth_check.json
python scripts/prototypes/check_deformable.py \
  --model real2sim/prototypes/soft_block/v1/model.xml \
  --output .cache/prototypes/soft_check.json
python -m pytest scripts/prototypes -q
```

检查逐步有限状态、数值警告、下降、落地接触、接触半径地面穿透；布片还检查
边长应变。2500 步的数值冒烟检查不验证抓取、自碰撞、多层折叠或真实材料。
报告记录模型 SHA256、引擎、阈值和实际结果，失败返回非零。
允许瞬时地面穿透 2 mm 是冒烟阈值，不是薄布重建精度。

## 后续接入顺序

1. 极光 Lite：真实装配变换、开合轴/行程/耦合与执行器；真实指垫及接触面。
2. 布片→丝绸→衣物→开口袋：保持经纬、裁片、接缝、双层和真实开口。
   测量尺寸/面密度/厚度、经纬拉伸、剪切、弯曲、摩擦、阻尼并留出加载轨迹验证。
3. 分别验证 Isaac 后端接触、自接触和夹起/滑移，再注册布局、任务和采集契约。
4. 视触觉：以实际设备 SDK、标定、同步时间戳、RGB/触觉原始帧建立接口；
   接触真值、总力或力/网格面积都不能冒充设备视触觉图像。

## 本地对齐状态（2026-10-22，不推送、不提 PR）

- 极光 Lite：444258三角形到USD的拓扑、坐标和米制边界已读回检查；精确CAD边界约149.2×62.0×163.8mm。48个未三角化小面合计约5.21mm²，细化重试仍有残留；转换器显式记录缺面状态，不宣称表面完整。
- 布片：MuJoCo2500步落地通过；独立Isaac5.1 `GarmentObject` 加载运行180步（1.5s）通过，质量5.12g、有限状态、无网格边伸长异常。仍未验证跨后端材料等价、折叠、自碰撞或机器人抓取。
- 软块：原接触设置冒烟失败（最大地面穿透3.47mm）；只调整地面/flex接触求解参数后降至0.35mm并通过，材料参数不变，原始实物资产未覆盖。
- `check_isaac_cloth.py --asset <生成目录> --output <报告>` 是独立运行入口；需要已安装并接受许可的Isaac环境，可用 `--capture <PNG>` 生成实际Isaac渲染。可选 `--aurora-reference <USD>` 仅加入静态外形展示，不视为机器人挂载。
- 以上都是几何/数值兼容验证；安装、运动、碰撞指垫、接触力、织物物性与视触觉仍需分别标定。

软块额外加载—释放检查：`scripts/prototypes/check_soft_block_load.py` 对顶部网格均分0.3N载荷，数值压缩约0.291mm，释放后高度残差约0.013mm，状态有限/无MuJoCo警告/穿透阈值通过。这不是实测力—位移标定。

## 2026-10-22 平台同步与官方复核

平台新增独立“研究资产”页（`#prototypes`），从各条目的 `platform.json` 扫描，保留官方外形参考、独立柔性夹具与待建模实物的不同状态，不进入任务物体或机器人本体候选。7项：极光Lite、其通用法兰、布片、软块、长条拉链布袋、塑料包装袋、线缆。

官方原件重新下载与哈希校验、传感器家族与版本、CAD小面及标称尺寸差异见 `aurora_lite/alignment_audit.json` 和 `AUDIT.md`。用户提供的通用法兰与官方文件字节不同，BRep双向布尔差体积为0；不据此认定G1-D安装或触觉标定完成。当前未取得传感器序列号、SDK版本或逐设备标定配置。

原始STEP、派生USD/MJCF/STL和法兰资料只传到服务器私有研究目录 `/data/robodojo-assets/research/yangkaiyuan-20261022/`；不放进共享 `Assets`、不提交厂商大文件到公开Git。各条目 `runtime_files.json` 记录实际文件哈希与存储路径。预览在 `preview/`，不入Git，部署时单独同步。

外部部署后要用既有 `Store.update` 写入平台账号显示名并注明agent代更新，自动扫描只刷新auto，不会自动生成最近操作署名。本次署名为“杨开元（agent 代更新）”。
