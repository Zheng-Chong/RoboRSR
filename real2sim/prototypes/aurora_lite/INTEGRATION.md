# 极光 Lite 与柔性资产：建模和获取路径

2026-10-22。当前静态 CAD、柔性数值 fixture 均不能当作实物孪生验收。

## 当前问题与材料辨识

- soft_block：36×36×24 mm、48 个顶点、30 g，Young=20 kPa 和泊松比 0.3 都是候选值。0.3 N 均布顶面加载仅压缩约 0.291 mm；不能据此代表海绵、软胶或实物的非线性、滞后。平地自由落体也不是有效的软硬辨识实验。
- cloth_patch：16×16 cm、9×9 网格、5.12 g；MuJoCo edge equality=true 约束初始边长。缺少材料特定的弯曲、剪切、经纬向拉伸辨识。平铺下落不能说明悬垂、折叠和夹持真实。应在细化网格后做收敛检查，而非只调一个刚度。
- 布料用薄壳/布料模型，分别拟合面内拉伸、剪切和面外弯曲；软块用体积弹性/黏弹性模型。薄膜不能直接套布料参数，弯曲与厚度密切相关。
- 拉链布袋需双层表面、缝线约束、拉链闭合和多层自接触；塑料袋需薄膜、封边和开口；线缆需杆/离散弹性杆的弯曲、扭转及端部接头。现有三项只有待建模记录，未生成这些结构。
- 可以先按照片重建几何并给参数范围；照片不能单独确定质量、厚度、摩擦或本构。

建议实测：尺寸、质量、厚度；软块多档压入深度的力—位移，至少两档速度、保持和卸载；布/薄片做悬臂下垂、悬垂形状、经纬拉伸、剪切与斜面摩擦。用一部分工况拟合，其余工况验证；报告误差、网格/时间步收敛和抓取结果。不能用“看着更软”代替验证。

## 1. 运动关节与 G1-D 安装

官方 STEP 是外形/装配参考，不自带经验证的 URDF/USD/MJCF 运动和动力学。已下载的 STEP 与官方文件哈希一致；v1 静态转换产物 njnt=0、nsensor=0，碰撞关闭；新 v2 候选已按装配树分件，生成两个相向滑动关节和同步约束，胶体网格凸包碰撞独立启用，85/42.5/0 mm 几何检查通过。没有添加未经辨识的实机驱动器；运动件惯量仍是显式数值占位，不能用于宣称真实动力学。

优先向厂家索取对应硬件修订版的：分件装配 CAD、URDF/USD/MJCF、关节轴/零位/限位、传动耦合、各运动件质量和惯量、TCP 和安装坐标系。未在已核查的公开硬件/SDK 页面找到完整的极光 Lite 物理资产下载；这不等于厂家没有。

若只能取得 STEP，可从 XCAF 装配树保留机身、左右滑块、指架、传感器壳和胶体，按刚性连接分组；分别建立相向移动关节，再用 mimic/tendon/约束耦合为一个开口自由度。SDK 的 position 是总开口 0–85 mm；只有核对对称传动、零位与 CAD 姿态后，才可将每侧行程映射为总开口的一半。驱动力、速度、阻尼需按实机与固件确定。

G1-D wrist → 转接法兰 → gripper_base → TCP 逐段明确变换。现有通用法兰几何核对过，但缺 G1-D 对应安装孔位和装配基准核验，不能宣布可直接装配。验收包括开/半开/闭合时指垫间距、TCP、限位、干涉、控制命令与实机读数。

## 2. 指垫碰撞

机壳可用分件简化碰撞，接触胶体应单独建实际外表面；不能用覆盖整个夹爪的凸包代替接触面。低成本软接触近似可以做夹持/摩擦验证，但不会自动生成胶体真实变形或触觉图像。需要形变时采用软体胶层或受标定的降阶接触模型。

索取胶体表面网格、厚度/内部支撑、材料或压入力曲线、摩擦和更换件修订信息；用平头/球头压入、切向加载、滑移测试拟合，并测试碰撞网格密度及求解时间步。传感器外壳 STEP 不能提供所有这些数据。

## 3. 视触觉

链路应是：物体接触 → 胶体位移/应力 → 光学成像 → 与实机一致的输出/标定接口。MuJoCo/Isaac 的接触力或 touch 标量不是视触觉图像。

厂家公开介绍 XENSE SIM 支持动力学与形变、适配 Isaac Sim / MuJoCo；应优先询问软件获取/许可、极光 Lite 对应传感器配置、仿真版本、胶体/光学参数及官方示例。现已取得用户下载的 Isaac/MuJoCo 官方发布包并验证 G1-WS FEM 可独立运行，具体资源、版本与未对齐项见 XENSE_PACKAGE_AUDIT.md。

XenseSDK 的官方离线示例支持 solver.selectSensorInfo(..., rectify_image=frame)。可以把官方 FEM/成像模型的合成图像对齐后输入同一 solver；当前已实现 xense_offline.py 适配入口，需要对应指垫导出的 runtime 才能完成端到端验证。图像生成来自 xensim，而不是 SDK 自身。记录每个指垫的传感器型号/SN、SDK/固件、标定版本、坐标系和时间戳；按 API 查询 rectify_size/grid_coord_size。calibrateSensor 是空载参考刷新，不等于建立整套材料、相机与力标定。

若厂家资源不可得，可移植 TACTO 等开源光学触觉框架，但需自己配置胶面、相机、光照/响应及与动力学耦合，并用实机同步压入/剪切数据验证。它们不是极光 Lite 的即插即用官方模型，也不能保证仿真图像经过厂家 solver 后就得到正确的力。

## 获取清单（可交给厂家，不自动代发）

1. 极光 Lite 硬件版本对应的分件 CAD + URDF/USD/MJCF；关节、耦合、零位、行程、质量惯量、TCP 和法兰图。
2. 指垫型号/修订版、胶体网格与厚度、材料/接触参数、相机内外参、光照与表面响应模型。
3. XENSE SIM 的获取方式、许可、Isaac/MuJoCo 支持版本、极光 Lite 示例与输出接口。
4. 各 SN 的 SDK/runtime 标定文件、输出定义与坐标系；可用于验证的同步原图、压入深度、法向/切向力及夹爪状态。

## 官方及项目来源

- CAD：https://docs.xenserobotics.com/en/docs/latest/hardware/gripper-models/
- 传感器 CAD：https://docs.xenserobotics.com/en/docs/latest/hardware/sensor-models/
- 夹爪控制：https://docs.xenserobotics.com/en/docs/1.0.0/sdk/xensegripper/api/
- 触觉 SDK：https://docs.xenserobotics.com/en/docs/1.0.0/sdk/xensesdk/overview/
- XENSE SIM：https://www.xenserobotics.com/about/371
- MuJoCo flex/elasticity：https://mujoco.readthedocs.io/en/stable/XMLreference.html
- TACTO：https://github.com/facebookresearch/tacto
