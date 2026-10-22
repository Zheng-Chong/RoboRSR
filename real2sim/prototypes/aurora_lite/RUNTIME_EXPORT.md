# 指垫 runtime 的实际导出条件

2026-10-22。可以由 agent 读取设备并导出，用户不必手动操作 GUI。入口 `scripts/prototypes/export_xense_runtime.py`。只读取当前标定，不调用 `calibrateSensor`、写 Flash、驱动夹爪或运动机器人。

检查传入的软件包后：

- SDK 1.6.8 的 `Sensor` 有 `exportRuntimeConfig` 和 `createSolver`；`FemSensor`/`SimpleSensor` 没有 runtime 导出接口。
- 软件包的 `g1-ws_table.npz` 是仿真光学标定，`g1-ws.npz` 是 FEM 数据；不是两个真实指垫各自导出的完整 SDK runtime。
- 实际调用 `Sensor.createSolver` 尝试载入包内 `omni/config/VecTouch`，SDK 返回 False（不是有效 runtime）。没有编辑/伪造这些加密配置，也没有把通用模板命名为实机 SN。
- sys01 实际执行 `Sensor.scanSerialNumber()` 返回空字典。kaiyuan2 的 Windows PnP 查询被 CIM 权限拒绝，因此不能凭此宣称该机器没有设备。
- 要导出每个真实指垫，SDK 必须能连接设备并加载其当前标定，或已有该指垫的完整标定和参考图可供软件载入。安装仿真软件本身不会产生这些设备实测信息。

连接设备后，先运行扫描，再对明确的两个序列号运行：

```
python scripts/prototypes/export_xense_runtime.py --output <私有新目录>
python scripts/prototypes/export_xense_runtime.py --serial <左SN> <右SN> --output <私有新目录>
```

脚本拒绝在空扫描时打开默认 camera0，也不会自动选择普通摄像头。导出目录700、文件600；SN与runtime不上公开仓库。随后用同一个 SDK 的 createSolver 载入，再验证仿真图像、颜色/尺寸、深度与力输出。左右物理映射要另外核验，不能仅凭扫描次序决定左右。

官方接口：
https://docs.xenserobotics.com/en/docs/1.0.0/sdk/xensesdk/overview/
