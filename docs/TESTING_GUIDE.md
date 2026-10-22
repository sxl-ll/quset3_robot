# Quest 3 到 Piper 关节角：分阶段测试指南

本文说明当前仓库中**哪些命令现在可以运行**、每一步应该观察什么，以及进入下一阶段前必须满足的条件。

> 安全边界：默认模式只读取 Quest 3 数据、计算目标位姿、求逆解和打印关节角。
> 只有显式提供 `--publish-joint-topic` 才发布真实机械臂关节命令；程序不会自动使能
> 机械臂。真机模式必须在急停可用、工作空间清空且只读验收通过后使用。

## 1. 当前实现状态

| 阶段 | 当前状态 | 入口或相关文件 |
| --- | --- | --- |
| Quest 3 原始数据读取 | 已实现，可进行实机测试 | `xrobotoolkit_teleop/cli/quest3_monitor.py` |
| XR 数据 Mock 测试 | 已实现 | `tests/test_quest3_reader.py` |
| 相对位姿映射 | 已实现并有独立测试 | `xrobotoolkit_teleop/common/pose_mapping.py`、`tests/test_pose_mapping.py` |
| Piper URDF | 模型文件存在 | `assets/agx_arm_urdf/piper/urdf/piper_description.urdf` |
| Piper 模型加载 | 已实现并有独立测试 | `xrobotoolkit_teleop/robot/piper_model.py`、`tests/test_piper_model.py` |
| Piper IK | 已复用 Placo 方法实现，并验证限位和 FK 误差 | `xrobotoolkit_teleop/robot/piper_ik.py`、`tests/test_piper_ik.py` |
| ROS 2 当前关节状态启动 FK | 可选，只读一帧 `JointState` | `xrobotoolkit_teleop/common/ros2_joint_state.py`、`tests/test_ros2_joint_state.py` |
| Quest 3 → 映射 → IK → 关节角 | 已实现软件入口和合成集成测试；待 Quest 3 实机验收 | `quest3-piper-ik`、`tests/test_teleop_pipeline.py` |
| ROS 2 右臂关节命令发布 | 显式开启、带本地保护；尚未做真机运动验收 | `xrobotoolkit_teleop/common/ros2_joint_command.py`、`tests/test_ros2_joint_command.py` |

目前有两个可执行主程序。仅监视原始数据：

```bash
python -m xrobotoolkit_teleop.cli.quest3_monitor
```

安装项目后也可以使用等价命令：

```bash
quest3-monitor
```

运行完整的只读关节角计算流水线：

```bash
python -m xrobotoolkit_teleop.cli.quest3_piper_ik --rate-hz 20 --position-scale 1.0
```

安装项目后也可以运行：

```bash
quest3-piper-ik --rate-hz 20 --position-scale 1.0
```

不要直接运行 `base_teleop_controller.py`。它是抽象基类，而且包含尚未由 Piper 后端实现的 `_robot_setup()`、`_update_robot_state()`、`_send_command()`、`_get_link_pose()` 和 `run()`。

## 2. 总体测试顺序

必须按以下顺序推进，前一阶段未通过时不要进入后一阶段：

```text
环境与软件测试
    ↓
Quest 3 原始数据读取
    ↓
Quest 3 坐标、单位、按键和断连验收
    ↓
离线位姿映射测试
    ↓
Piper 模型加载与 FK 测试
    ↓
离线 IK 与有效性测试
    ↓
Quest 3 → 映射 → IK → 关节角集成测试
    ↓
默认只输出关节角
    ↓
隔离 ROS 2 话题发布测试
    ↓
人工确认后低速真机测试
```

## 3. 阶段 0：进入项目和检查环境

在终端中执行：

```bash
cd /home/wsco/vscodeproject3/quest3_robot_teleop
conda activate /home/wsco/vscodeproject3/quest3_robot_teleop/.venv
python --version
python -c "import numpy, meshcat, placo, xrobotoolkit_sdk; print('核心依赖导入成功')"
python -m pip show xrobotoolkit_teleop xrobotoolkit-sdk placo
```

当前仓库已验证的环境是：

- Python 3.10.22；
- `xrobotoolkit-sdk==1.1.0`；
- `placo==0.10.1`；
- 项目以 editable 方式安装。

如果需要从头安装，在激活环境后执行：

```bash
python -m pip install -e .
```

通过条件：四个核心依赖均可导入，`pip show` 指向当前仓库或当前 `.venv`。

## 4. 阶段 1：运行不需要硬件的软件测试

### 4.1 语法检查

```bash
python -m compileall -q xrobotoolkit_teleop tests
```

通过条件：命令状态码为 0，没有错误输出。

### 4.2 运行全部现有单元测试

```bash
python -m unittest discover -v
```

也可以只运行 Quest 3 Reader 测试：

```bash
python -m unittest tests.test_quest3_reader -v
```

当前基线为 52 个测试全部通过。Quest 3 Reader 的 6 个测试验证：

- 完整快照读取；
- 时间戳更新和数据陈旧检测；
- 零时间戳和零位姿不会被误报为有效；
- 非法四元数会被拒绝；
- SDK 读取异常会被包装为明确错误；
- SDK `close()` 可安全重复调用。

其余测试覆盖模型加载、位姿映射、Piper IK 和合成集成链路。它们仍不代表 Quest 3 实机已经通过。

### 4.3 检查 Quest 3 监视命令

```bash
python -m xrobotoolkit_teleop.cli.quest3_monitor --help
```

通过条件：能看到 `--rate-hz`、`--stale-timeout-s` 和 `--duration-s` 参数。

## 5. 阶段 2：Quest 3 实机原始数据测试

### 5.1 启动顺序

1. 启动 XRoboToolkit PC Service。
2. 按当前 XRoboToolkit 配置方式连接 Quest 3。
3. 确认头显以及左右控制器都已被系统跟踪。
4. 在项目根目录启动 10 秒短测：

```bash
python -m xrobotoolkit_teleop.cli.quest3_monitor \
  --rate-hz 20 \
  --stale-timeout-s 0.5 \
  --duration-s 10
```

5. 短测正常后，启动持续监视；按 `Ctrl-C` 停止：

```bash
python -m xrobotoolkit_teleop.cli.quest3_monitor --rate-hz 20
```

PC Service 和 Quest 端程序不在本仓库中，因此本仓库没有用于启动它们的脚本。

### 5.2 正常数据的判断标准

每帧第一行应类似：

```text
timestamp_ns=... timestamp=advanced age=0.000s tracking=VALID
```

重点确认：

- `timestamp_ns` 大于 0，并持续增加；
- 正常连续跟踪时大多数帧为 `timestamp=advanced`；
- `age` 不持续超过 `0.500s`；
- `tracking=VALID`；
- `headset`、`left_controller`、`right_controller` 均显示 3 个位置值和 `quat_xyzw`；
- 不出现 `NaN`、`Inf`、`INVALID` 或持续的 `warning`。

当前原始位姿顺序已经由代码确认是：

```text
[x, y, z, qx, qy, qz, qw]
```

四元数顺序为 `xyzw`。原始 XR 坐标说明为左手系：`+X` 向右、`+Y` 向上、`+Z` 向内。位置单位仍需在目标设备上实测，不能直接假定为米。

### 5.3 逐项动作验收

保持监视器运行，并记录每项结果：

| 操作 | 应观察字段 | 通过条件 |
| --- | --- | --- |
| 静置所有设备 10 秒 | 三个位姿、时间戳 | 时间戳继续增加；位姿仅有小幅噪声；保持 `VALID` |
| 单独平移头显 | `headset pos` | 只有头显位姿按预期明显变化 |
| 单独旋转头显 | `headset quat_xyzw` | 四元数连续变化，范数有效，无跳变为零 |
| 单独平移右手柄 | `right_controller pos` | 右手柄位置连续变化 |
| 单独旋转右手柄 | `right_controller quat_xyzw` | 右手柄姿态连续变化 |
| 平移、旋转左手柄 | `left_controller` | 左手柄字段对应变化 |
| 按右 Grip | `right_grip` | 从接近 0 连续增大，完全按压时接近其实测最大值 |
| 按右 Trigger | `right_trigger` | 从接近 0 连续增大 |
| 按左 Grip/Trigger | `left_grip`、`left_trigger` | 对应字段变化 |
| 按 A/B/X/Y/Menu/摇杆 | `buttons` | 只有对应按键在按下时变为 1 |
| 推动左右摇杆 | `axes` | 对应二维值连续变化 |

不要仅凭字段“有数字”判定通过；必须验证移动方向、设备对应关系和按键对应关系。

### 5.4 坐标方向和单位标定

1. 把右手柄放在固定起点，记录 `right_controller pos`。
2. 只向使用者右侧移动一段可测量距离，记录三轴增量。
3. 回到起点，再依次向上、向前移动相同距离。
4. 比较每次的主变化轴、正负号和数值幅度。
5. 用直尺测量例如 100 mm 的移动，并计算“SDK 数值增量 / 实际距离”。
6. 将实测结论记录下来后，才能确定映射中的单位换算和坐标变换。

当前代码打印的坐标约定来自 SDK 接口说明，但 PC Service 是否额外变换坐标以及数值单位，仍必须以本机实测为准。

### 5.5 陈旧数据和断连测试

监视器运行时依次测试：

1. 暂停或关闭 Quest 端数据源；
2. 断开 Quest 与 PC Service；
3. 停止 PC Service；
4. 恢复服务和连接。

通过条件：

- 时间戳停止更新超过 0.5 秒后显示 `tracking=INVALID` 和陈旧数据警告；
- 读取异常显示 `XR read error`，而不是继续把最后一帧当作有效目标；
- 程序不崩溃，并能继续重试；
- `Ctrl-C` 能正常退出并关闭 SDK。

只有本节全部通过后，才应把 Quest 3 数据接入位姿映射。

## 6. 阶段 3：Piper 模型静态检查

这一步只检查仓库中的机械臂模型，不连接真实机械臂。

### 6.1 确认 URDF 存在

```bash
python -c "from xrobotoolkit_teleop.utils.path_utils import PIPER_URDF_PATH; print(PIPER_URDF_PATH); assert PIPER_URDF_PATH.is_file()"
```

### 6.2 查看 URDF 中的关节定义

```bash
rg -n '<joint name=|<limit ' assets/agx_arm_urdf/piper/urdf/piper_description.urdf
```

由当前 URDF 可确认：

- 运动关节名为 `joint1` 到 `joint6`；
- 角度和角度限位使用 URDF 标准的弧度；
- 当前模型末端 link 为 `link6`；
- 每个运动关节的上下限应从 URDF 读取，不能在应用代码中另行猜测。

实际控制器的关节顺序、零位、方向和限位仍需与松灵机械臂型号及控制器文档逐项核对。URDF 存在并不等于真机参数已经验证。

### 6.3 验证模型加载

运行模型加载和 FK 测试：

```bash
python -m unittest tests.test_piper_model -v
```

加载器不会修改官方 URDF，而是在内存中将已知前缀
`package://agx_arm_description/agx_arm_urdf/piper/` 转换为仓库内资源的绝对
`file://` URI。测试确认 Placo 能识别 6 个关节、`link6`、URDF 限位和有限的 FK 变换。

## 7. 阶段 4：位姿映射测试

运行：

```bash
python -m unittest tests.test_pose_mapping -v
```

当前测试不依赖 Quest 3 或 IK，并覆盖：

- 第一次激活时控制器零位移产生零目标位移；
- 右、上、前三个方向映射正确；
- 位置缩放系数正确；
- `xyzw` 到内部四元数顺序的转换正确；
- 旋转增量作用在正确坐标系；
- Grip 松开时目标保持；
- Grip 再按下时重新锚定，不发生目标跳变；
- `tracking_valid=False` 或陈旧帧不会更新目标。

## 8. 阶段 5：离线 FK/IK 测试

在模型加载测试通过后，应先做完全离线的 IK，不连接 Quest 3，更不能连接真机。

必须依次验证：

1. Placo 能加载 Piper URDF；
2. 求解器识别的关节名和顺序与 URDF 一致；
3. 能从已知合法初始关节角计算 FK；
4. 将该 FK 位姿作为 IK 目标，能求回一个合法解；
5. 输出关节角维度、顺序和单位明确；
6. 每个关节满足 URDF 限位；
7. 对 IK 结果再次做 FK，计算位置误差和姿态误差；
8. 不可达目标明确返回失败；
9. 连续小步目标不会出现无法解释的关节跳变；
10. 奇异位形附近能报告失败或误差，而不是只输出一个数值向量。

运行：

```bash
python -m unittest tests.test_piper_model -v
python -m unittest tests.test_piper_ik -v
```

测试使用 Placo `KinematicsSolver`、固定基座、URDF 关节限位和原项目的 frame task/
manipulability task 方法。默认验收阈值为位置误差不超过 2 mm、姿态误差不超过
0.001 rad；每个成功结果都会重新读取 FK 和关节限位进行验证。不可达目标不会返回关节角，
并会恢复到求解前的最后有效关节状态。

IK 结果至少应打印：

```text
target_position: ...
target_orientation_xyzw: ...
ik_status: SUCCESS/FAILURE/NOT_RUN
joint_names: [joint1, ..., joint6]
joint_angles_rad: [...]
joint_limits_valid: true/false
position_error_m: ...
orientation_error_rad: ...
```

不能因为 `solver.solve()` 没有抛异常就判定 IK 成功。

## 9. 阶段 6：Quest 3 到关节角集成测试

完整入口连接以下已测试模块：

- 可选的 ROS 2 初始关节状态读取器；
- 已验证的 `Quest3Reader`；
- 独立且已测试的相对位姿映射器；
- 已测试的 Piper 模型加载器；
- 已测试并带 FK 误差和关节限位检查的 Placo IK 封装。

集成程序的处理顺序应是：

```text
可选：从 /puppet/joint_right 读取一次 joint0…joint5
  → 校验名称、数值和启动限位偏差
  → FK 计算当前 link6 位姿并设为映射初始目标
  → 读取 Quest 快照
  → 检查时间戳新鲜度和右手柄位姿有效性
  → 检查右 Grip clutch
  → 计算相对控制器位姿
  → 生成 Piper link6 目标位姿
  → Placo IK
  → 关节限位检查
  → FK 回算误差检查
  → 只打印关节角和状态
```

先运行无需硬件的集成测试：

```bash
python -m unittest tests.test_teleop_pipeline -v
```

启动 PC Service、连接 Quest 3，并完成第 5 节的实机原始数据验收后，先做 10 秒短测：

```bash
python -m xrobotoolkit_teleop.cli.quest3_piper_ik \
  --rate-hz 20 \
  --duration-s 10 \
  --position-scale 1.0
```

若 `/puppet/joint_right` 发布节点已经启动，可用当前机械臂状态初始化：

```bash
python -m xrobotoolkit_teleop.cli.quest3_piper_ik \
  --initial-joint-topic /puppet/joint_right \
  --initial-joint-timeout-s 5 \
  --rate-hz 20 \
  --duration-s 10 \
  --position-scale 1.0
```

已验证的话题包含 `joint0` 到 `joint6`。程序按名称提取 `joint0…joint5` 作为
Piper 六个臂关节，并保存 `joint6` 作为夹爪位置。前六项 `JointState.position`
按控制器接口使用弧度，夹爪位置使用米。若实测值只在配置的启动容差内轻微越过 URDF 限位，程序保存原始 FK 位姿
作为映射零点，并将内部 IK 状态求解回严格限位内；超出启动容差则拒绝启动。

确认目标方向、尺度和 IK 输出均正确后，再持续运行：

```bash
quest3-piper-ik --rate-hz 20 --position-scale 1.0
```

`--position-scale` 必须根据第 5.4 节测得的位置单位和期望运动比例配置，不能把默认
`1.0` 当作已经完成硬件标定。右 Grip 大于默认阈值 `0.9` 时激活 clutch。

集成验收顺序：

1. 不按 Grip：目标位姿和关节角保持不变；
2. 按下 Grip但保持手柄不动：目标不跳变；
3. 小幅平移手柄：目标位姿按比例变化，IK 连续；
4. 小幅旋转手柄：末端姿态按约定变化；
5. 松开 Grip：保持最后目标；
6. 移动手柄后重新按 Grip：重新锚定，不跳变；
7. 制造跟踪丢失：停止更新目标和 IK，明确输出无效状态；
8. 给出不可达目标：输出 IK 失败，不复用旧结果冒充新解；
9. 本阶段不要提供 `--publish-joint-topic`，确认程序显示发布已禁用。

右手单臂流水线不要求左手柄有效。左手柄休眠时，原始监视器的全局
`tracking_valid` 可以为 `False`，但只要时间戳新鲜且右手柄位姿有效，集成入口的
`right_tracking_valid` 应为 `True`，右手柄映射应继续工作。

## 10. 阶段 7：ROS 2 发布和真机联调

`piper_right_ctrl_node` 已确认订阅两个不同控制入口：

- `/joint_right_states`：`sensor_msgs/msg/JointState`，接收关节角；
- `/pos_right_cmd`：`piper_msgs/msg/PosCmd`，接收末端位置和 RPY。

本流水线已经完成 IK，因此使用 `/joint_right_states`。`/piper_right_ctrl_node/arm_status`、
`/piper_right_ctrl_node/end_pose` 和 `/puppet/joint_right` 都是反馈，不是本程序的控制输出。

先设置正确 ROS 2 域并确认图连接：

```bash
export ROS_DOMAIN_ID=222
ros2 node info /piper_right_ctrl_node
ros2 topic info /puppet/joint_right --verbose
ros2 topic info /joint_right_states --verbose
```

预期 `/puppet/joint_right` 有发布者，`/joint_right_states` 有订阅者。首次真机前，先在
与控制器无关的临时话题验证消息格式；不要用 `ros2 topic pub` 向真实控制话题试发。

完成第 6 阶段全部只读验收，确认急停、机器人使能流程和安全空间后，才运行：

```bash
export ROS_DOMAIN_ID=222
python -m xrobotoolkit_teleop.cli.quest3_piper_ik \
  --initial-joint-topic /puppet/joint_right \
  --publish-joint-topic /joint_right_states \
  --initial-joint-timeout-s 5 \
  --command-subscriber-timeout-s 5 \
  --max-command-joint-step-rad 0.05 \
  --max-command-joint-speed-rad-s 0.5 \
  --max-command-joint-acceleration-rad-s2 2.0 \
  --driver-speed-percent 20 \
  --rate-hz 50 \
  --position-scale 1.0 \
  --orientation-scale 0.3
```

`--publish-joint-topic` 是刻意设置的显式开关。程序启动时先读取当前六轴角度和
`joint6` 夹爪位置；发布时写入 `joint0…joint5` 的有效 IK 结果并原样保持夹爪。
只有新目标通过 IK、FK 误差和 URDF 限位检查才会发布。IK 目标先经过关节速度和
加速度限制，必要时以 `trajectory=RATE_LIMITED` 发布中间点，而不是因普通速度
超限退出。以下情况会停发或拒绝：

- 右手柄跟踪或时间戳无效；
- Grip 未激活或刚刚建立 clutch 锚点；
- IK 失败或关节限位无效；
- 相邻 IK 目标超过硬单步限制（程序立即退出，需重新读取状态）；
- `/joint_right_states` 的订阅者消失。

程序不会写 `/enable_flag`，不会替操作者使能机械臂。上述单步、速度和加速度限制
仅是应用层附加保护，不替代驱动器中的速度、加速度、碰撞、通信看门狗和急停保护。
`controller_delta_raw_units` 显示 Grip 锚点以来的累计手柄位移；若实际移动 10 cm
时其范数约为 `0.10`，`--position-scale 1.0` 表示约 1:1 平移，`0.1` 则只映射为
约 1 cm。`--driver-speed-percent 20` 按 Piper 驱动约定写入
`JointState.velocity[6]`，避免全零 velocity 被驱动解释为 100% 速度。初次联调
仍应从极小手柄位移开始，并由操作者随时准备急停。

## 11. 每次测试的记录模板

建议每次测试记录以下内容：

```text
日期时间：
Git commit：
Python 版本：
xrobotoolkit-sdk 版本：
placo 版本：
Quest/PC Service 版本：
执行命令：
是否连接 Quest 3：
是否连接真实机械臂：
是否启用 --publish-joint-topic：
通过项：
失败项：
关键输出或错误：
坐标/单位实测结论：
下一步：
```

可用下面的命令记录当前 Git 版本和依赖版本：

```bash
git rev-parse --short HEAD
python --version
python -m pip show xrobotoolkit-sdk placo
```

## 12. 当前最短可执行清单

现在可立即执行：

```bash
cd /home/wsco/vscodeproject3/quest3_robot_teleop
conda activate /home/wsco/vscodeproject3/quest3_robot_teleop/.venv
python -m compileall -q xrobotoolkit_teleop tests
python -m unittest discover -v
python -m xrobotoolkit_teleop.cli.quest3_monitor --help
python -m xrobotoolkit_teleop.cli.quest3_piper_ik --help

# 启动 PC Service 并连接 Quest 3 后：
python -m xrobotoolkit_teleop.cli.quest3_monitor --rate-hz 20 --duration-s 10
python -m xrobotoolkit_teleop.cli.quest3_monitor --rate-hz 20

# 完成坐标和单位验收后，只计算并打印关节角：
python -m xrobotoolkit_teleop.cli.quest3_piper_ik --rate-hz 20 --duration-s 10 --position-scale 1.0
quest3-piper-ik --rate-hz 20 --position-scale 1.0
```

只有在上述只读验收全部通过后，才进入第 10 节的显式发布流程。当前代码具备发布
入口，但这不等于真实机械臂运动已经通过验证。
