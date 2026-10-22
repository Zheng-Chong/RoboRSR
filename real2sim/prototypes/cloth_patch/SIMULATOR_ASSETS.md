# 使用仿真器资产建立通用候选

2026-10-22。缺实物测量时，可以直接使用仿真器的网格、物理模型及默认材料作为通用任务资产，无须等实测后才开始仿真。需要保留来源和参数身份，后续再按实物记录拟合。

- **软立方体**：IsaacLab 官方 `DeformableObject` + `MeshCuboidCfg` 创建 PhysX FEM 体积软体，自动生成仿真/碰撞四面体网格；本项目入口 `check_isaac_soft_cube.py` 已设为100mm立方体。用户只给尺寸，质量30g、Young20kPa、泊松比0.3仍为候选。NVIDIA示例自己的材料默认值也不是本实验室物料的实测值。
- **布片**：当前 Isaac Sim 5.1 已通过本项目 `GarmentObject`/`SingleClothPrim` 路径运行过粒子布料。可载入合适的衣物/布网格并用拉伸、剪切、弯曲、阻尼和自接触参数作为候选，不需要先拿实测曲线。
- **薄膜与袋子**：需要合适的薄片或多层网格、开口/封边/缝线约束。仅加载一个同外形的刚体 USD，或把布料材质涂到刚体上，不会得到布或袋子的物理行为。
- 当前 IsaacLab 2.x 的 `DeformableObject` 指 PhysX 体积软体，不能直接当作粒子布片接口。新版 Newton/其他后端需要相应版本与接口；不要为了一个示例升级正在采集的生产环境。

物理参数与外观参数分开记录。通用资产可以先用于操作测试和训练；物理尺寸、质量、摩擦和本构都得到实物支持后，再标为实物对应。不同引擎的刚度参数名称相似也不能直接视作同一个数值模型。

官方示例：
https://isaac-sim.github.io/IsaacLab/v2.3.0/source/tutorials/01_assets/run_deformable_object.html

本机路径：`env/scene_manager/objects/garment.py`、`scripts/prototypes/check_isaac_cloth.py`。
