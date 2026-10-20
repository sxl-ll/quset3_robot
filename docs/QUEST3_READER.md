# Quest 3 原始数据读取

该模块只读取并显示 XR 数据，不包含位姿映射、IK 或机器人控制。

## 运行

先启动 XRoboToolkit PC Service，并按原项目方式连接 Quest 3。确认当前 Python 环境能够执行：

```bash
python -c "import xrobotoolkit_sdk"
```

在仓库根目录运行：

```bash
python -m xrobotoolkit_teleop.cli.quest3_monitor --rate-hz 20
```

若已安装本项目，也可以运行 `quest3-monitor`。按 `Ctrl-C` 停止；SDK 会在退出时调用 `close()`。
短时检查可增加 `--duration-s 10`。

每轮输出包含：

- 头显、左手柄和右手柄的完整 6DoF 位姿；
- 左右 Grip、Trigger；
- A/B/X/Y、左右 Menu、左右摇杆按压；
- 左右摇杆二维值；
- SDK 纳秒时间戳、时间戳是否更新、数据陈旧时间和综合有效性。

## 数据约定与验证边界

下列内容由仓库原有 SDK 封装和 XRoboToolkit SDK 接口定义确认：

- 位姿数组排列为 `[x, y, z, qx, qy, qz, qw]`；
- 四元数排列为 `xyzw`；
- 原始坐标系是左手系：`+X` 向右、`+Y` 向上、`+Z` 向内；
- SDK 文档将原点描述为 XR 应用启动时的头显位姿；
- SDK 时间戳由 `get_time_stamp_ns()` 返回，单位为纳秒。

以下内容仍需在目标 Quest 3、PC Service 和 SDK 版本上实测确认：

- **位置数值的物理单位**。当前 SDK API 文档未明确声明，本模块不会擅自换算；
- PC Service 是否会对原始 OpenXR 坐标作版本相关的附加变换；
- 设备断开时 SDK 返回零值、保持最后一帧还是抛出异常；
- 单个设备的跟踪置信度。当前 Python API 没有头显/手柄各自的 tracking-valid 标志。

因此 `tracking_valid` 是保守推断值：要求全局时间戳持续前进、三个 7 元素位姿均为有限数、三个四元数近似单位四元数，以及模拟量和摇杆值为有限数。它不能替代 SDK 将来可能提供的设备级跟踪置信度。

## 异常行为

- SDK 未安装或其动态库无法加载：给出明确错误并以状态码 2 退出；
- SDK 初始化失败：给出明确错误并以状态码 2 退出；
- 单轮读取抛出异常（例如服务或设备断开）：打印错误，监视器继续重试；
- 时间戳超过阈值不更新：快照标记为 `INVALID` 和 `stale`；
- 位姿维度错误、含 NaN/Inf 或四元数范数异常：对应设备标记为无效。

Mock 测试无需 Quest 3 或 PC Service：

```bash
python -m unittest tests.test_quest3_reader -v
```

硬件验收时应分别移动头显和两个手柄、旋转各设备、逐个按下 A/B/X/Y、Grip、Trigger、Menu 和摇杆，并观察对应字段变化；断开 Quest 或停止 PC Service 后，还应确认程序不会输出看似有效且持续更新的数据。
