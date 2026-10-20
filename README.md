# Quest 3 Piper Arm Teleoperation

本项目保留 XRoboToolkit 原项目中与以下目标直接相关的代码：

- 读取 Quest 3 头显、左右手柄的位姿和按键状态；
- 检测跟踪时间戳、无效数据和设备断开；
- 复用原项目的相对笛卡尔位姿映射和 Grip Clutch 逻辑；
- 复用 Placo 逆运动学方法；
- 使用官方 Piper URDF 和 Mesh 作为松灵单臂运动学模型；
- 仅计算、验证并输出关节角，不发送真实机器人控制命令。

项目不依赖 ROS、MoveIt、MuJoCo、相机、数据录制或真实机械臂驱动。

## 环境

仓库内已使用 Python 3.10 创建本地 Conda prefix 环境：

```bash
conda activate /home/wsco/vscodeproject3/quest3_robot_teleop/.venv
```

从头安装时可执行：

```bash
python -m pip install -e .
```

## Quest 3 数据监视

先启动 XRoboToolkit PC Service 并连接 Quest 3，然后运行：

```bash
quest3-monitor --rate-hz 20
```

该入口只读取并打印 XR 数据，不执行 IK 或机器人控制。详细字段和异常行为见
[`docs/QUEST3_READER.md`](docs/QUEST3_READER.md)。

## 测试

无需 Quest 3 的 Mock 测试：

```bash
python -m unittest discover -v
```

## 模型

Piper 官方模型位于：

```text
assets/agx_arm_urdf/piper/
```

模型许可证保存在 `assets/agx_arm_urdf/LICENSE`。URDF 中的
`package://agx_arm_description/...` Mesh URI 需要由 IK 加载模块解析到仓库内实际资源路径。

## 安全边界

- 不连接或控制真实机械臂；
- 不发送关节命令；
- 不包含 ROS/MoveIt 集成；
- 真实 Quest 3 数据单位和坐标约定仍需在目标设备上验收。
