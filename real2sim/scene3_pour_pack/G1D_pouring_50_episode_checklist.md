# G1-D 首批 50 条倒水数据生成清单

日期：2026-09-16  
基础：单一 `pour_scene_v1.usd`  
目标：验证固定四视角、材质泛化、杯具缩小、task-space 扰动、多抓握 frame 和轻微物理扰动下的自动 episode 生成能力。

---

## 1. 全局固定条件

所有 50 条均保持：

- 同一个 G1-D 机器人资产；
- 固定基座、仅上半身；
- 同一个基础场景拓扑；
- 无障碍物；
- 无行走；
- 固定四视角：
  - `head_left`
  - `head_right`
  - `left_wrist`
  - `right_wrist`
- 不做人为相机位姿扰动；
- 不放大任何杯具；
- 所有动作均由 grasp/task frames + G1-D IK + 阶段式控制生成；
- 所有 episode 均进行完整液体物理执行与自动验收。

---

## 2. 参数约定

### Grasp

- `G0`：默认抓握 frame
- `G1`：较低抓握 frame
- `G2`：较高抓握 frame
- `G3`：绕杯体少量旋转后的备选 frame
- `AUTO`：在全部有效 grasp frame 中自动选择可达性最优者

### 杯具缩放

只允许：

- `1.00`
- `0.95`
- `0.90`
- `0.85`

若任一缩放导致碰撞体、内腔、grasp frame 或液体初始化异常，该 episode 直接拒绝，不继续采集。

### 布局

均相对 baseline 位姿写成 delta。

建议首批只使用保守范围：

```text
source Δx/Δy: ±20~40 mm
receiver Δx/Δy: ±30~60 mm
yaw: 小角度扰动
```

实际值以当前 G1-D baseline 的可达域为准。

### 材质

视觉材质变化默认不改变物理摩擦。

---

## 3. 50 条生成清单

| ID | 执行臂 | 类型 | 桌布/背景 | 源杯 | 接水杯 | 几何/布局 | Grasp | 水量/物理 | 目的 |
|---|---|---|---|---|---|---|---|---|---|
| E001 | 右臂 | Baseline | baseline | baseline | baseline | baseline | G0 | baseline | 基准 |
| E002 | 左臂 | Baseline | baseline | baseline | baseline | baseline | G0 | baseline | 不同 seed |
| E003 | 右臂 | Baseline | baseline | baseline | baseline | baseline | G1 | baseline | 验证替代抓握 |
| E004 | 左臂 | Baseline | baseline | baseline | baseline | baseline | G2 | baseline | 验证替代抓握 |
| E005 | 右臂 | Baseline | baseline | baseline | baseline | baseline | AUTO | baseline | 自动抓握选择 |
| E006 | 左臂 | 桌布材质 | 深色纯色 | baseline | baseline | baseline | AUTO | baseline | 背景颜色 |
| E007 | 右臂 | 桌布材质 | 浅色纯色 | baseline | baseline | baseline | AUTO | baseline | 背景颜色 |
| E008 | 左臂 | 桌布材质 | 中性灰 | baseline | baseline | baseline | AUTO | baseline | 背景颜色 |
| E009 | 右臂 | 桌布材质 | 细纹织物 | baseline | baseline | baseline | AUTO | baseline | 纹理 |
| E010 | 左臂 | 桌布材质 | 粗纹织物 | baseline | baseline | baseline | AUTO | baseline | 纹理 |
| E011 | 右臂 | 桌布材质 | 低 roughness | baseline | baseline | baseline | AUTO | baseline | 表面反射变化 |
| E012 | 左臂 | 源杯外观 | baseline | 哑光浅色 | baseline | baseline | AUTO | baseline | 源杯颜色/粗糙度 |
| E013 | 右臂 | 源杯外观 | baseline | 哑光深色 | baseline | baseline | AUTO | baseline | 源杯颜色 |
| E014 | 左臂 | 源杯外观 | baseline | 高 roughness | baseline | baseline | AUTO | baseline | 杯壁粗糙度 |
| E015 | 右臂 | 源杯外观 | baseline | 低 roughness | baseline | baseline | AUTO | baseline | 杯壁高光 |
| E016 | 左臂 | 源杯外观 | baseline | 金属感低 | baseline | baseline | AUTO | baseline | 杯壁材质 |
| E017 | 右臂 | 接水杯外观 | baseline | baseline | 哑光浅色 | baseline | AUTO | baseline | receiver 颜色 |
| E018 | 左臂 | 接水杯外观 | baseline | baseline | 哑光深色 | baseline | AUTO | baseline | receiver 颜色 |
| E019 | 右臂 | 接水杯外观 | baseline | baseline | 高 roughness | baseline | AUTO | baseline | receiver 粗糙度 |
| E020 | 左臂 | 接水杯外观 | baseline | baseline | 低 roughness | baseline | AUTO | baseline | receiver 高光 |
| E021 | 右臂 | 双杯外观 | baseline | 新材质 A | 新材质 B | baseline | AUTO | baseline | 组合视觉泛化 |
| E022 | 左臂 | 几何缩小 | baseline | baseline | baseline | source scale=0.95 | AUTO | 同比例更新容量/质量 | 轻微源杯几何泛化 |
| E023 | 右臂 | 几何缩小 | baseline | baseline | baseline | source scale=0.90 | AUTO | 同比例更新容量/质量 | 源杯几何泛化 |
| E024 | 左臂 | 几何缩小 | baseline | baseline | baseline | source scale=0.85 | AUTO | 同比例更新容量/质量 | 源杯几何边界 |
| E025 | 右臂 | 几何缩小 | baseline | baseline | baseline | receiver scale=0.95 | AUTO | baseline | receiver 几何泛化 |
| E026 | 左臂 | 几何缩小 | baseline | baseline | baseline | receiver scale=0.90 | AUTO | baseline | receiver 几何泛化 |
| E027 | 右臂 | 几何缩小 | baseline | baseline | baseline | receiver scale=0.85 | AUTO | baseline | receiver 对准难度 |
| E028 | 左臂 | 几何缩小 | baseline | baseline | baseline | source=0.95, receiver=0.95 | AUTO | 更新容量/质量 | 双杯轻缩小 |
| E029 | 右臂 | 几何缩小 | baseline | baseline | baseline | source=0.90, receiver=0.90 | AUTO | 更新容量/质量 | 双杯几何泛化 |
| E030 | 左臂 | 布局 | baseline | baseline | baseline | source Δx=+小扰动 | AUTO | baseline | source 可达域 |
| E031 | 右臂 | 布局 | baseline | baseline | baseline | source Δx=-小扰动 | AUTO | baseline | source 可达域 |
| E032 | 左臂 | 布局 | baseline | baseline | baseline | source Δy=+小扰动 | AUTO | baseline | source 可达域 |
| E033 | 右臂 | 布局 | baseline | baseline | baseline | receiver Δx=+小扰动 | AUTO | baseline | 倾倒目标变化 |
| E034 | 左臂 | 布局 | baseline | baseline | baseline | receiver Δx=-小扰动 | AUTO | baseline | 倾倒目标变化 |
| E035 | 右臂 | 布局 | baseline | baseline | baseline | receiver Δy=+小扰动 | AUTO | baseline | 搬运距离变化 |
| E036 | 左臂 | 布局 | baseline | baseline | baseline | source 少量 yaw | AUTO | baseline | 抓握方向变化 |
| E037 | 右臂 | 布局 | baseline | baseline | baseline | source+receiver 同时小扰动 | AUTO | baseline | 联合 task-space 变化 |
| E038 | 左臂 | 水量 | baseline | baseline | baseline | baseline | AUTO | fill=较低 | 液体质量变化 |
| E039 | 右臂 | 水量 | baseline | baseline | baseline | baseline | AUTO | fill=中等 | 液体质量变化 |
| E040 | 左臂 | 水量 | baseline | baseline | baseline | baseline | AUTO | fill=较高但不溢出 | 液体质量变化 |
| E041 | 右臂 | 轻物理 | baseline | baseline | baseline | baseline | AUTO | source mass 小幅降低 | 动力学泛化 |
| E042 | 左臂 | 轻物理 | baseline | baseline | baseline | baseline | AUTO | friction 小幅变化 | 接触泛化 |
| E043 | 右臂 | 组合泛化 | 深色织物 | 哑光浅色 | 深色 | receiver 小位移 | AUTO | baseline | 未见组合 1 |
| E044 | 左臂 | 组合泛化 | 浅色织物 | 深色 | 哑光浅色 | source 小位移 | AUTO | 较低水量 | 未见组合 2 |
| E045 | 右臂 | 组合泛化 | 中性灰 | 低 roughness | 高 roughness | source scale=0.95 | G1/AUTO | baseline | 材质+几何 |
| E046 | 左臂 | 组合泛化 | 粗纹织物 | 哑光深色 | 浅色 | receiver scale=0.95 | AUTO | 中等水量 | 视觉+几何+水量 |
| E047 | 右臂 | 组合泛化 | 深色纯色 | 新材质 C | 新材质 D | source Δx + receiver Δy | AUTO | baseline | 视觉+布局 |
| E048 | 左臂 | 组合泛化 | 浅色纯色 | 高 roughness | 低 roughness | source=0.90 | AUTO | 较低水量 | 较强组合变化 |
| E049 | 右臂 | 组合泛化 | 细纹织物 | 新材质 E | 哑光深色 | receiver=0.90 + 小位移 | AUTO | 中等水量 | receiver 几何+布局 |
| E050 | 左臂 | 组合留出 | 从未组合过的桌布 | 从未组合过的源杯材质 | 从未组合过的接水杯材质 | 允许范围内最大联合小扰动 + 合法缩小 | AUTO | 合法水量扰动 | 首批最强组合泛化 |

---

## 4. 生成顺序

不要直接从 E001 一口气跑到 E050。

建议按以下顺序执行：

```text
E001–E005
↓
确认基础 skill 和多个 grasp frame

E006–E021
↓
确认材质变化不破坏物理

E022–E029
↓
确认缩小后的碰撞、容量、抓握和液体初始化

E030–E037
↓
确认 task-space 变化能重新求解 IK

E038–E042
↓
确认轻微动力学变化

E043–E050
↓
最后跑组合泛化
```

如果某一层出现系统性失败，停止扩大该维度，先修正生成器或 skill，不要靠重复重跑掩盖问题。

---

## 5. 每条生成前检查

### Asset

- USD 能正常加载；
- visual/collision 对齐；
- 杯口开放；
- 内腔有效；
- GPU 下液体碰撞有效；
- 缩放后质量/惯性/容量已同步；
- grasp/task frames 已重新计算。

### Robot

- 加载的是指定 G1-D；
- joint ordering 正确；
- 初始状态稳定。

### Camera

- 主视角存在；
- 左腕视角存在；
- 右腕视角存在；
- 内外参正确；
- 三路时间同步。

### Planning

- 至少一个 grasp frame IK 可解；
- pre-grasp → grasp 连续；
- lift 可达；
- pour target 可达；
- return/place 可达。

---

## 6. 每条生成后检查

必须输出：

```text
SUCCESS
或
FAIL:<reason>
```

失败原因至少区分：

- `IK_NO_SOLUTION`
- `JOINT_LIMIT`
- `SELF_COLLISION`
- `ENV_COLLISION`
- `GRASP_FAIL`
- `CUP_SLIP`
- `LIFT_FAIL`
- `TRANSPORT_FAIL`
- `ALIGN_FAIL`
- `POUR_FAIL`
- `SPILL_TOO_HIGH`
- `RECEIVER_TOO_LOW`
- `PLACE_FAIL`
- `LIQUID_ACCOUNTING_FAIL`
- `CAMERA_SYNC_FAIL`
- `DATA_MISSING`

失败 episode 不计入 50 条有效交付，但其配置和失败原因必须保存。

---

## 7. 首批完成标准

第一批完成不是“跑了 50 次”，而是：

```text
50 条通过验收的有效 episode
```

同时要求：

- 至少 3 个 grasp frame 被实际使用；
- 至少 3 种桌布外观；
- 至少 5 种杯壁外观组合；
- 至少 3 个缩小档位完成验证；
- source 与 receiver 都有位置扰动成功样本；
- 至少 3 种水量条件；
- 三固定相机全部完整；
- E043–E050 的组合泛化均有通过样本；
- 保存全部失败配置和失败统计。

完成后再根据真实通过率决定第二批是否扩大几何和 task-space 范围。

## 左右臂与生成状态

奇数 ID 使用右臂，偶数 ID 使用左臂；每批各 25 条。左臂布局关于本体 y=-0.045 m 平面对称后，独立求解和运行，不通过翻转图像伪造左臂数据。机器可读参数：`scene3_roundcup/tasks_100.json`。当前选择 E001、E012、E023、E052、E087（右 3、左 2）。归档旧结果不作为新数据。每任务最多 3 次物理尝试；耗尽标记 `IMPOSSIBLE_UNDER_CURRENT_CONFIG` 并跳过，不代表已证明物理不可能。只有物理和视频审查均通过才计入有效数据；最终报告分别统计左右臂成功数。
