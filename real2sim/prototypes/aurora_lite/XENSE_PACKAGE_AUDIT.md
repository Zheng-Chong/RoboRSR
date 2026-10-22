# XENSE SIM 包核对与接入状态

2026-10-22。用户在 Windows 下载的两个原包已同步 sys01 与 5090-2，逐文件 SHA-256 和大小相同。慢速 rsync 被官网同名包的服务端下载替代，最后重新校验并成功解包。签名下载链接、二进制、原始 FEM 和设备 runtime 不入公开仓库。

| 包 | 字节 | SHA-256 |
| --- | ---: | --- |
| mujoco-sim-v1.0.0.zip | 12444907 | 65643c7d3918abb6b866a17a993d4bb64d9a744ae9a74edbfbbbe4820141b523 |
| isaac_xensesim4.5_dist_v1.1.0.zip | 697697087 | e8f40d2e89a6794bbded33f19c5dfd8a4dc14dd8fffacd4a14fd7d8876bf4607 |

## 内容与运行

- MuJoCo 包有 UR5e 双滑指关节 MJCF、传感器深度相机和 `FemSensor.step(obj_pose,sensor_pose,depth,nstep)` 示例。不是 G1-D 的安装资产。附 xensim 0.3.0 Windows CPython 3.9 wheel。
- Isaac 包面向 Isaac 4.5，扩展声明内部版本 0.1.1；含 Linux CPython 3.10 的 xensim 二进制、XenseSDK **1.6.8**、成像和 FEM 数据。没有安装/注册到生产 Isaac 5.1；两者不能直接混用 CPython 二进制。
- 两包的 `g1-ws.npz` 哈希均为 `3fd053c7046999bce2bbdfc21f9d0fbe09071e3a3d6246c4f682097c469b5b51`；`g1-ws_table.npz` 均为 `2cd8f3b989f70d56965cdf70325f0ab2285986af8c72229253661f61a3948b8b`。
- sys01 隔离 Python 3.10 + Xvfb 已实际运行官方 FemSensor：0、0.1、0.2 mm 合成压入，输出图像 700×400×3、节点力 700×3、Marker 20×11×3，均为有限值。节点力数组范数随压入为 0、0.1372、0.2820；这里不是实测力精度或整面合力验收。
- 可复现入口 `scripts/prototypes/check_xense_fem.py`；图像为官方成像模型输出，见预览。它仍是独立合成压入测试，尚未完成机器人接触深度 → FEM → SDK → 控制闭环。

## CAD 与传感器对齐尚需处理

官方 STEP 装配树读到 75 个叶部件。已按导轨/滑块/齿条/左右传感器名称与位置分组为机身、左运动件、右运动件、左右胶体。左右胶面内边界 x=±42.5 mm，定义 q=0 为 CAD 的 85 mm 张开态，q∈[0,42.5] mm 为每侧闭合量，开口 = 85−q_left−q_right mm。两个移动关节已用等式约束同步。

新 `articulate_aurora.py` 输出 MJCF 和逐部件映射；`check_aurora_kinematics.py` 验证开口并输出视频。胶体使用 CAD 网格的 MuJoCo 凸包碰撞候选。壳体碰撞关闭；胶体本构、摩擦和驱动器未标定，运动件质量/惯量为显式占位；不能把 kinematic sweep 当作力控或夹持验收。STEP 三角化仍有已知缺面，不用于宣称全模型封闭或完整机壳碰撞。

FEM 域：2100 节点、1292 个八节点单元；node 坐标范围约 x=[−8.65,8.65]、y=[1,30.14]、z=[−3,0]，按官方示例单位为 mm；有效平面约 17.3×29.14 mm，域深 3 mm。CAD 胶体包围尺寸约 2×17.5×29.344 mm。可能分别对应有效感知区、实体胶体及力学支撑域；必须核对定义、坐标和硬件修订，不应盲目缩放成同一个厚度。文件同属 G1-WS 不代表已经匹配两个实机 SN。

## SDK 图像输入路线

官方接口明确支持离线 `rectify_image`：

https://docs.xenserobotics.com/en/docs/1.0.0/sdk/xensesdk/data-workflow/

已准备 `scripts/prototypes/xense_offline.py`：输入已对齐的 NPY uint8 图像序列和私有 `runtime_<SN>`，明确 RGB/BGR，检查尺寸，顺序请求 Depth/Force，拒绝缺失与非有限输出。不自动缩放、翻转或猜标定。目前只完成输入合同和异常释放测试；没有 runtime，因此尚未进行真实 SDK 推理验证。包内 SDK 确认有 createSolver 接口。

完整接入时，先校准左右指垫坐标、有效平面、投影尺度、旋转/镜像、RGB/BGR、空载参考和光照响应，再对比压入/剪切/滑移的真值与 SDK 输出。FEM 力和主仿真器接触力的双向耦合需要独立设计，避免重复施力；仅并行渲染触觉并不自动使主仿真碰撞变成可变形胶体。

软件原包位于 sys01 `g1d/vendor/xense/20261022`，5090-2 `/data/robodojo-runs/xense-20261022/vendor`，不经平台文件接口提供二进制。
