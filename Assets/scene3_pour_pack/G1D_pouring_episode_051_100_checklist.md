# G1-D 第二批 50 条倒水数据生成清单（Episode 051–100）

日期：2026-09-17  
基础场景：继续复用第一批的单一 `pour_scene_v1.usd`  
目标：在不重复 Episode 001–050 泛化条件的前提下，将一个场景扩展到总计 100 条有效数据。第二批重点增加 **大幅杯具摆位变化、相对位置关系变化、明显尺寸缩小和强组合泛化**。

---

## 1. 与第一批的明确区别

Episode 051–100 不再重复以下第一批主变化：

- 单纯桌布颜色/纹理变化；
- 单纯杯壁颜色/roughness 变化；
- 单独的小范围 source / receiver 平移；
- `0.95 / 0.90 / 0.85` 的轻度缩小；
- 单独的小幅水量/质量/摩擦变化；
- 单纯替换 grasp frame；
- 单因素视觉泛化。

第二批的主要增量为：

```text
大幅 source / receiver 位置变化
+ 大幅相对距离变化
+ 左右/前后/对角布局变化
+ source/receiver 相对方位翻转
+ < 0.85 的明显缩小
+ 大空间变化与几何/材质/水量的组合
```

---

## 2. 大幅摆位的定义

不要用固定的“±几厘米”作为第二批的主要范围。

首先对当前固定基座 G1-D 做一次快速工作空间扫描，得到：

```text
source feasible region
receiver feasible region
pour feasible pair region
```

然后将可行域分成三个层级：

```text
INNER   = 靠近 baseline 的内部区域
MID     = 可行域中部
OUTER   = 接近当前已验证可行域边界，但仍有足够关节/碰撞余量
```

Episode 051–100 主要采样 `MID` 和 `OUTER`。

### 必须满足

每个候选布局生成前先检查：

- source grasp 至少有一个 grasp frame 可达；
- lift 可达；
- receiver 上方 pour pose 可达；
- transport 路径无明显碰撞；
- 关节限位存在安全余量；
- source 与 receiver 不相互穿插；
- 杯具均位于桌面有效区域；
- source → receiver 的相对距离仍允许稳定搬运和倾倒。

如果候选不满足，不进入正式液体 rollout。

---

## 3. 更强缩小范围

第二批不重复第一批的：

```text
1.00 / 0.95 / 0.90 / 0.85
```

新增明显缩小档位：

```text
0.80
0.75
0.70
0.65
```

`0.60` 只作为探索边界，不直接默认进入有效数据；只有通过以下预检后才能采用：

- 夹爪最小稳定闭合宽度允许；
- 杯壁 collision 不退化；
- 内腔仍可稳定容纳液体粒子；
- particle radius / spacing 与内腔尺寸兼容；
- source capacity 仍足以产生有意义的倒水轨迹；
- receiver 开口仍能用于当前粒子尺度。

### 缩小时同步更新

- visual mesh；
- collision；
- 内腔；
- mass / inertia；
- capacity；
- particle initialization；
- grasp frames；
- gripper width；
- rim / pour frames；
- pouring offset。

---

## 4. 第二批 50 条生成清单

### A. Source 大幅换位：051–060

| ID | 执行臂 | Source 位置 | Receiver 位置 | 几何 | Grasp | 其他变化 | 目的 |
|---|---|---|---|---|---|---|---|
| E051 | 右臂 | MID-Left | baseline | 1.00 | AUTO | 新材质组合 | 大幅左移抓取 |
| E052 | 左臂 | OUTER-Left | baseline | 1.00 | AUTO | 新材质组合 | 左侧可达边界 |
| E053 | 右臂 | MID-Right | baseline | 1.00 | AUTO | 新材质组合 | 大幅右移抓取 |
| E054 | 左臂 | OUTER-Right | baseline | 1.00 | AUTO | 新材质组合 | 右侧可达边界 |
| E055 | 右臂 | MID-Front | baseline | 1.00 | AUTO | baseline 物理 | 前向大位移 |
| E056 | 左臂 | OUTER-Front | baseline | 1.00 | AUTO | baseline 物理 | 前向边界 |
| E057 | 右臂 | MID-Back | baseline | 1.00 | AUTO | 新视觉组合 | 后向大位移 |
| E058 | 左臂 | OUTER-Back | baseline | 1.00 | AUTO | 新视觉组合 | 后向边界 |
| E059 | 右臂 | MID-Front-Left | baseline | 1.00 | AUTO | 中水量 | 对角大位移 |
| E060 | 左臂 | MID-Front-Right | baseline | 1.00 | AUTO | 中水量 | 对角大位移 |

### B. Receiver 大幅换位：061–070

| ID | 执行臂 | Source 位置 | Receiver 位置 | 几何 | Grasp | 其他变化 | 目的 |
|---|---|---|---|---|---|---|---|
| E061 | 右臂 | baseline | MID-Left | 1.00 | AUTO | 新 receiver 材质 | 大幅左侧倾倒 |
| E062 | 左臂 | baseline | OUTER-Left | 1.00 | AUTO | 新 receiver 材质 | 左侧倾倒边界 |
| E063 | 右臂 | baseline | MID-Right | 1.00 | AUTO | 新 receiver 材质 | 大幅右侧倾倒 |
| E064 | 左臂 | baseline | OUTER-Right | 1.00 | AUTO | 新 receiver 材质 | 右侧倾倒边界 |
| E065 | 右臂 | baseline | MID-Front | 1.00 | AUTO | baseline 物理 | 前向远距离运输 |
| E066 | 左臂 | baseline | OUTER-Front | 1.00 | AUTO | baseline 物理 | 前向运输边界 |
| E067 | 右臂 | baseline | MID-Back | 1.00 | AUTO | 新视觉组合 | 后向倾倒 |
| E068 | 左臂 | baseline | OUTER-Back | 1.00 | AUTO | 新视觉组合 | 后向倾倒边界 |
| E069 | 右臂 | baseline | MID-Back-Left | 1.00 | AUTO | 较低水量 | 对角 receiver |
| E070 | 左臂 | baseline | MID-Back-Right | 1.00 | AUTO | 较高合法水量 | 对角 receiver |

### C. Source 与 Receiver 同时大幅换位：071–080

这一组重点改变二者的**相对距离和方向关系**，不是整体平移场景。

| ID | 执行臂 | Source | Receiver | 相对关系 | Grasp | 目的 |
|---|---|---|---|---|---|---|
| E071 | 右臂 | MID-Left | MID-Right | 大横向跨越 | AUTO | 长横向 transport |
| E072 | 左臂 | MID-Right | MID-Left | 与 E071 方向相反 | AUTO | 反向横向 transport |
| E073 | 右臂 | MID-Front | MID-Back | 大前后跨越 | AUTO | 长纵向 transport |
| E074 | 左臂 | MID-Back | MID-Front | 与 E073 方向相反 | AUTO | 反向纵向 transport |
| E075 | 右臂 | MID-Front-Left | MID-Back-Right | 对角跨越 | AUTO | 长对角路径 |
| E076 | 左臂 | MID-Front-Right | MID-Back-Left | 反向对角跨越 | AUTO | 反向对角路径 |
| E077 | 右臂 | OUTER-Left | MID-Front-Right | 边界→内部对角 | AUTO | 大空间跨度 |
| E078 | 左臂 | OUTER-Right | MID-Front-Left | 边界→内部对角 | AUTO | 镜像空间跨度 |
| E079 | 右臂 | MID-Back-Left | OUTER-Front-Right | 内部→边界 | AUTO | 高难布局 |
| E080 | 左臂 | MID-Back-Right | OUTER-Front-Left | 内部→边界 | AUTO | 镜像高难布局 |

### D. 大幅相对距离与方位变化：081–086

| ID | 执行臂 | Source / Receiver 关系 | 几何 | Grasp | 目的 |
|---|---|---|---|---|---|
| E081 | 右臂 | 当前可行域内最短安全间距 | 1.00 | AUTO | 极短 transport |
| E082 | 左臂 | 接近最大可行间距 | 1.00 | AUTO | 极长 transport |
| E083 | 右臂 | receiver 位于 source 左后方 | 1.00 | AUTO | 改变 pour 方位 |
| E084 | 左臂 | receiver 位于 source 右后方 | 1.00 | AUTO | 镜像 pour 方位 |
| E085 | 右臂 | receiver 位于 source 左前方 | 1.00 | AUTO | 改变手腕旋转方向 |
| E086 | 左臂 | receiver 位于 source 右前方 | 1.00 | AUTO | 镜像手腕旋转方向 |

### E. 更强源杯缩小：087–092

| ID | 执行臂 | Source scale | Receiver scale | 布局 | Grasp | 目的 |
|---|---|---:|---:|---|---|---|
| E087 | 右臂 | 0.80 | 1.00 | MID 布局 | AUTO | 明显缩小 |
| E088 | 左臂 | 0.75 | 1.00 | MID-Left → MID-Right | AUTO | 缩小 + 大位移 |
| E089 | 右臂 | 0.70 | 1.00 | MID-Right → MID-Left | AUTO | 更小源杯 |
| E090 | 左臂 | 0.65 | 1.00 | baseline/MID receiver | AUTO | 小源杯边界 |
| E091 | 右臂 | 0.75 | 1.00 | OUTER source → MID receiver | AUTO | 小杯 + 外围抓取 |
| E092 | 左臂 | 0.70 | 1.00 | MID source → OUTER receiver | AUTO | 小杯 + 远距离倾倒 |

### F. 更强 Receiver 缩小：093–096

| ID | 执行臂 | Source scale | Receiver scale | 布局 | Grasp | 目的 |
|---|---|---:|---:|---|---|---|
| E093 | 右臂 | 1.00 | 0.80 | MID 大位移 | AUTO | 更小接水口 |
| E094 | 左臂 | 1.00 | 0.75 | MID 对角布局 | AUTO | 对准精度提升 |
| E095 | 右臂 | 1.00 | 0.70 | source/receiver 大间距 | AUTO | 小 receiver + 长运输 |
| E096 | 左臂 | 1.00 | 0.65 | 保守 MID 布局 | AUTO | 小 receiver 边界 |

### G. 双杯明显缩小 + 强组合：097–100

| ID | 执行臂 | Source scale | Receiver scale | 布局 | 视觉/物理 | 目的 |
|---|---|---:|---:|---|---|---|
| E097 | 右臂 | 0.80 | 0.80 | MID-Left → MID-Right | 全新材质组合 | 双杯明显缩小 |
| E098 | 左臂 | 0.75 | 0.80 | MID-Front → MID-Back | 新桌布 + 新杯壁组合 | 几何+长 transport |
| E099 | 右臂 | 0.70 | 0.75 | MID 对角大布局 | 合法较低水量 | 强几何+空间组合 |
| E100 | 左臂 | 0.65 | 0.70 | 当前验证可行域内的最强联合布局 | 从未出现的材质组合 + 合法水量 | 第二批 stress test |

---

## 5. 关于 OUTER 样本的执行原则

`OUTER` 不是随机把杯子扔到桌角。

生成方法：

```text
候选大范围位置
    ↓
grasp frame 可达性检查
    ↓
pour pose 可达性检查
    ↓
关节限位余量检查
    ↓
粗碰撞检查
    ↓
通过
    ↓
正式液体 rollout
```

建议给每个候选记录：

```text
joint_limit_margin
ik_residual
min_collision_distance
transport_path_length
```

这样后续可以区分内部稳定数据和接近能力边界的数据。

---

## 6. 第二批不重复第一批的视觉策略

第二批仍然可以改变材质，但不能把“换材质”作为该 episode 的唯一泛化因素。

要求：

```text
第二批每次视觉变化
必须同时伴随
大布局 / 大缩小 / 大相对距离
至少其中一项
```

因此第二批的材质变化只作为组合背景，不占独立 episode 配额。

---

## 7. 第二批生成顺序

### Step 1：验证大工作空间

```text
E051–E060
E061–E070
```

确认 source 和 receiver 分别能覆盖更宽区域。

### Step 2：验证相对布局

```text
E071–E086
```

确认大距离、反向、对角 transport 和不同 pour 方位。

### Step 3：验证明显缩小

```text
E087–E096
```

确认小杯条件下：

- collision；
- grasp；
- liquid；
- rim alignment；
- pour success。

### Step 4：组合 stress test

```text
E097–E100
```

只在前三步稳定后运行。

---

## 8. 第二批有效数据要求

第二批完成条件：

```text
50 条新的 Accepted Episodes
```

而不是“执行 50 次”。

同时应满足：

- source 至少覆盖左、右、前、后、对角多个区域；
- receiver 至少覆盖左、右、前、后、对角多个区域；
- 至少 10 条 source 与 receiver 同时大幅移动；
- 至少 6 条改变 source/receiver 的方位关系；
- 至少 6 条 source scale ≤ 0.80；
- 至少 4 条 receiver scale ≤ 0.80；
- 至少 3 条双杯同时明显缩小；
- 第二批不以单纯材质变化作为独立 episode；
- 三相机规范与第一批完全一致。

---

## 9. 一个场景累计 100 条后的覆盖结构

完成两批后，该单一 USD 场景应包含：

```text
001–050
├── baseline
├── 多 grasp
├── 桌布/杯壁视觉变化
├── 小范围 pose 扰动
├── 0.95/0.90/0.85 轻缩放
├── 水量/轻物理扰动
└── 初级组合泛化

051–100
├── source 大幅换位
├── receiver 大幅换位
├── source/receiver 同时大幅换位
├── 大相对距离
├── 反向/对角布局
├── 0.80/0.75/0.70/0.65 强缩放
└── 大空间 + 几何 + 视觉/物理组合
```

这样 100 条不只是“随机化更多”，而是形成从局部扰动到大范围可行域覆盖的两级数据分布。

## 左右臂与生成状态

奇数 ID 使用右臂，偶数 ID 使用左臂；每批各 25 条。左臂布局关于本体 y=-0.045 m 平面对称后，独立求解和运行，不通过翻转图像伪造左臂数据。机器可读参数：`scene3_roundcup/tasks_100.json`。当前选择 E001、E012、E023、E052、E087（右 3、左 2）。归档旧结果不作为新数据。每任务最多 3 次物理尝试；耗尽标记 `IMPOSSIBLE_UNDER_CURRENT_CONFIG` 并跳过，不代表已证明物理不可能。只有物理和视频审查均通过才计入有效数据；最终报告分别统计左右臂成功数。
