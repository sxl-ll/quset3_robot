# Quest 3 Piper Arm Teleoperation

本项目保留 XRoboToolkit 原项目中与以下目标直接相关的代码：

- 读取 Quest 3 头显、左右手柄的位姿和按键状态；
- 检测跟踪时间戳、无效数据和设备断开；
- 复用原项目的相对笛卡尔位姿映射和 Grip Clutch 逻辑；
- 复用 Placo 逆运动学方法；
- 使用官方 Piper URDF 和 Mesh 作为松灵单臂运动学模型；
- 可选地从 ROS 2 `sensor_msgs/msg/JointState` 读取一次当前关节角作为启动 FK；
- 默认只计算、验证并打印关节角；显式指定参数后可把有效 IK 结果发布给右臂。

默认运行不依赖 ROS。使用 `--initial-joint-topic` 或 `--publish-joint-topic` 时需要
已配置的 ROS 2 环境。项目不依赖 MoveIt、MuJoCo、相机或数据录制。

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
adb devices
adb shell am broadcast -a com.oculus.vrpowermanager.prox_close
quest3-monitor --rate-hz 20
```

该入口只读取并打印 XR 数据，不执行 IK 或机器人控制。详细字段和异常行为见
[`docs/QUEST3_READER.md`](docs/QUEST3_READER.md)。

## Quest 3 到 Piper 关节角

先完成原始数据、坐标方向和位置单位验收，再运行完整的只读计算流水线：

```bash
quest3-piper-ik --rate-hz 20 --position-scale 1.0
```

也可以直接运行模块：

```bash
python -m xrobotoolkit_teleop.cli.quest3_piper_ik \
  --rate-hz 20 \
  --position-scale 1.0
```

若要从松灵右臂当前关节状态计算启动末端位姿：

```bash
python -m xrobotoolkit_teleop.cli.quest3_piper_ik \
  --initial-joint-topic /puppet/joint_right \
  --initial-joint-timeout-s 5 \
  --rate-hz 20 \
  --position-scale 1.0
```

该话题已验证为 `sensor_msgs/msg/JointState`：`joint0` 到 `joint5` 依次映射到
Piper URDF 的 `joint1` 到 `joint6`；`joint6` 是夹爪位置。解析按名称而不是数组
下标进行。

右手柄 Grip 用作 clutch。程序输出目标末端位姿、IK 状态、关节限位检查、
FK 位置/姿态误差，以及按 `joint1` 到 `joint6` 顺序排列的弧度关节角。
默认情况下它不发布真实机械臂命令。

## 显式发布到右臂

已确认 `/joint_right_states` 是 `piper_right_ctrl_node` 订阅的关节控制话题；
`/pos_right_cmd` 是末端笛卡尔命令，不用于发布本程序已经求出的关节角。
下面的命令会让真实机械臂运动，只有在急停可用、工作空间清空并完成上述只读验收后才能运行：

```bash
export ROS_DOMAIN_ID=222
python -m xrobotoolkit_teleop.cli.quest3_piper_ik \
  --initial-joint-topic /puppet/joint_right \
  --publish-joint-topic /joint_right_states \
  --initial-joint-timeout-s 5 \
  --command-subscriber-timeout-s 5 \
  --max-command-joint-step-rad 0.05 \
  --max-command-joint-speed-rad-s 0.5 \
  --rate-hz 20 \
  --position-scale 0.1
```

发布消息仍为 `sensor_msgs/msg/JointState`：`joint0…joint5` 写入经过 IK 成功、
FK 误差和 URDF 限位验证的六个关节角（弧度），`joint6` 保持启动反馈中的夹爪
位置（米）。程序仅在 Grip 激活并生成新目标且 IK 验证成功时发布；跟踪失效、
IK 失败时不发布；跳变量或速度保护超限时拒绝该帧并立即停止程序，要求重新读取
真机状态后再启动。发布前还会确认控制话题确实存在订阅者。

程序不会发布 `/enable_flag`，因此不会自动使能机械臂。`0.05 rad` 单步限制和
`0.5 rad/s` 变化率限制是本程序的额外软件保护参数，不代表驱动器自身的速度、
加速度或安全额定值；首次真机联调仍应使用更小的位置缩放并遵循机器人厂商要求。

`--position-scale` 同时决定 XR 位移到机器人米制位移的换算和运动缩放；
在 Quest 3 实机确认 SDK 位置单位之前，不应把默认值视为已标定参数。

## 测试

无需 Quest 3 或真实机械臂的自动化测试：

```bash
python -m unittest discover -v
```

完整的逐步验收过程和各阶段命令见
[`docs/TESTING_GUIDE.md`](docs/TESTING_GUIDE.md)。

## 模型

Piper 官方模型位于：

```text
assets/agx_arm_urdf/piper/
```

模型许可证保存在 `assets/agx_arm_urdf/LICENSE`。加载器在内存中将 URDF 的
`package://agx_arm_description/...` Mesh URI 解析到仓库资源；官方 URDF 本身保持不变。

## 安全边界

- 默认不开启关节命令发布；
- 只有同时指定初始状态和 `--publish-joint-topic` 才建立控制发布器；
- 不自动发布使能指令，也不控制夹爪；
- 发布端带有限状态、订阅者、单步和变化率检查；
- 不包含 MoveIt 集成；
- 真实 Quest 3 数据单位、坐标约定和完整流水线仍需在目标设备上验收。
