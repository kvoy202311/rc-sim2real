# kvoy_quadruped

自制四足机器狗的 ROS2 sim2real 工程。当前仓库围绕以下链路组织：

- `gamepad_input` 读取手柄并发布 `/gamepad_cmd`
- `robot_fsm` 管理 `WAITING / STANDUP / STANDING / RUNNING / LIEDOWN / ESTOP`
- `robot_fsm` 进程内部组合 `robot_kinematics` 和 `rl_controller`
- `imu_driver` 读取 HWT906 串口 IMU 并发布 `/imu/data`
- `serial_comm` 负责 Jetson Orin <-> MCU 的串口通信
- `rl_controller` 使用 TensorRT engine 做策略推理

本文档按当前代码和 Jetson Orin Nano 上的实际 bringup 情况更新，重点说明：

- 当前工程的真实状态
- 各节点、各消息、各参数的单位
- Orin 与 MCU 串口协议中的单位
- RL 观测、动作和最终关节目标的单位

## 1. 当前工程状态

更新时间：`2026-05-17`

当前代码与文档一致的实际情况如下：

- 当前实测 ROS 发行版是 `ROS2 Foxy`，机器上存在 `/opt/ros/foxy`
- 当前串口实现已经从 `libserialport` 改为 Linux 原生 `termios`
- `serial_comm` 和 `imu_driver` 都不再依赖 `libserialport-dev`
- `colcon build` 已在 Jetson Orin Nano 上通过
- `bringup.launch.py` 已在 Jetson 上拉起
- `imu_driver_node` 已在 Jetson 上实测持续发布 `/imu/data`
- `gamepad_input_node` 已在 Jetson 上实测识别 `GameSir-G7 Pro`
- `robot_fsm_node` 已在 Jetson 上实测完成状态切换：
  - `WAITING -> STANDUP -> STANDING -> RUNNING`
  - 也验证过 `ESTOP` 和 `LIEDOWN -> WAITING`
- CH343 USB 转 TTL 在 Jetson 上做过 `TX-RX` 自环测试，`2000000 / 3000000 / 4000000 baud` 均通过
- 当前 MCU 串口默认使用 CH343 `by-id` 路径，波特率为 `2000000 baud`
- 当前已接入 MCU 并收到 `/motor_state`，Jetson 上层链路可以发布 `/motor_cmd`
- 当前默认 RL 策略是 `policy_2`，TensorRT engine 路径在 `policies.yaml` 中配置

当前还没有完成的部分：

- 当前只做过局部电机/散电机阶段测试，还不是完整装机负载测试
- 最近一次测试中，`RUNNING` 状态下 `/motor_cmd` 会随摇杆和 RL 输出变化，但电机侧是否正确跟随仍需要继续在 MCU/电机控制侧验证
- 当前默认 `serial_comm_node.serial_port` 已固定为 MCU 侧稳定路径 `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5792019699-if00`

当前工程有两个非常重要的现状说明：

1. 手柄触发条件以 `gamepad_input_node.cpp` 为准，当前起立/运行切换是 `LT + LB + Y` 组合键。
2. 当前 `rl_controller`、`robot_kinematics` 都只发送关节位置，`velocity` 和 `torque` 目前始终发 `0.0`。

## 2. 工程结构

```text
src/
├── kvoy_msgs/        # 自定义 ROS2 消息
├── serial_comm/      # Orin <-> MCU 串口通信
├── imu_driver/       # HWT906 IMU 驱动
├── gamepad_input/    # SDL2 手柄输入
├── robot_kinematics/ # 起立/趴下双段插值
├── rl_controller/    # TensorRT 策略推理
├── robot_fsm/        # 顶层状态机，内部组合 kinematics + rl_controller
└── robot_bringup/    # launch 和参数文件
```

## 3. 环境与依赖

### 3.1 当前实测环境

- 计算平台：Jetson Orin Nano
- ROS：`ROS2 Foxy`
- 编译器标准：`C++17`

### 3.2 系统依赖

当前工程和当前代码一致的系统依赖：

```bash
sudo apt update
sudo apt install libsdl2-dev
```

说明：

- 串口节点当前使用 `termios`，不再需要 `libserialport-dev`
- `rl_controller` 在编译时需要本机已经安装 `TensorRT` 和 `CUDA runtime/dev`
- 因为 `robot_fsm` 会链接 `rl_controller`，所以要构建完整 bringup，机器上必须能找到 TensorRT/CUDA 头文件和库

## 4. 构建

```bash
source /opt/ros/foxy/setup.bash

cd ~/Desktop/RC2026/kvoy_quadruped
colcon build
source install/setup.bash
```

如果只想单独重编某几个包：

```bash
source /opt/ros/foxy/setup.bash

cd ~/Desktop/RC2026/kvoy_quadruped
colcon build --packages-select kvoy_msgs serial_comm imu_driver gamepad_input robot_kinematics rl_controller robot_fsm robot_bringup
source install/setup.bash
```

## 5. 串口权限与设备路径

### 5.1 `dialout` 组

普通用户访问 `/dev/ttyUSB*`、`/dev/ttyACM*` 通常需要在 `dialout` 组中：

```bash
sudo usermod -aG dialout $USER
```

执行后需要重新登录，或者开启一个新的 shell 会话，新的组权限才会生效。

### 5.2 当前 Jetson 上实测到的设备

当前这台 Jetson 上实测到：

- IMU：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
- 该 IMU 的内核设备：`/dev/ttyUSB0`
- MCU 串口：`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5792019699-if00`
- 该 MCU 的内核设备：`/dev/ttyACM0`

### 5.3 `by-id` 是否会因为改 IMU 回传频率而变化

一般不会。

`/dev/serial/by-id/...` 绑定的是 USB 设备身份信息，主要取决于：

- USB 转串口芯片
- USB 产品字符串
- USB 序列号
- interface 信息

修改 IMU 的回传频率、回传内容、滤波参数，通常不会改变 `by-id` 路径。更容易导致变化的是：

- 换了另一块 USB 转串口设备
- 换了另一块同型号但序列号不同的设备
- 底层 USB 描述符发生变化

## 6. 在新终端启动完整工程

### 6.1 新终端启动前必须执行的准备

每打开一个新的终端，都必须重新 `source` ROS2 和当前工作空间。否则终端里找不到本工程的 ROS2 包、消息类型和 launch 文件。

推荐固定按下面顺序执行：

```bash
source /opt/ros/foxy/setup.bash

cd ~/Desktop/RC2026/kvoy_quadruped
source install/setup.bash
```

如果刚刚改过代码，还没有重新编译，需要先构建再启动：

```bash
source /opt/ros/foxy/setup.bash

cd ~/Desktop/RC2026/kvoy_quadruped
colcon build
source install/setup.bash
```

注意：

- `source /opt/ros/foxy/setup.bash` 只加载 ROS2 Foxy 环境
- `source install/setup.bash` 才会加载当前工程编译出的包
- `source install/setup.bash` 必须在 `colcon build` 之后执行
- 每个新开的终端都要执行这两条 `source`，不能只在旧终端里执行一次

### 6.2 启动完整 bringup

当前 `bringup.launch.py` 默认启动 4 个独立进程：

- `serial_comm_node`
- `imu_driver_node`
- `gamepad_input_node`
- `robot_fsm_node`

其中 `robot_fsm_node` 进程内部还会创建两个节点：

- `kinematics_node`
- `rl_controller_node`

完整工程默认启动命令：

```bash
source /opt/ros/foxy/setup.bash

cd ~/Desktop/RC2026/kvoy_quadruped
source install/setup.bash
ros2 launch robot_bringup bringup.launch.py
```

默认启动时会使用：

- 主运行参数：`src/robot_bringup/config/robot_params.yaml`
- RL 策略参数：`src/robot_bringup/config/policies.yaml`

当前默认配置要求以下设备已经接入 Jetson：

- IMU 串口设备：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
- MCU 串口设备：`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5792019699-if00`
- 手柄接收机：由 SDL2 自动枚举

如果启动前想确认两个串口是否存在，可以在新终端执行：

```bash
ls -l /dev/serial/by-id/
```

如果这两个 `by-id` 路径不存在，默认 launch 会因为找不到串口而无法完整工作。此时应先检查 USB 连接、供电、线缆、设备枚举和 `robot_params.yaml` 里的串口路径。

### 6.3 启动串口诊断模式

串口诊断参数文件已经统一放在：

```text
src/robot_bringup/config/robot_params_serial_diag.yaml
```

它不是主配置文件，只是覆盖 `serial_comm_node` 的调试参数。需要查看 MCU 串口原始收发、帧头、长度、CRC、尾字节时，用下面命令启动：

```bash d ~/Desktop/RC2026/kvoy_quadruped
source /opt/ros/foxy/setup.bash


source install/setup.bash
ros2 launch robot_bringup bringup.launch.py \
  params_override_file:=src/robot_bringup/config/robot_params_serial_diag.yaml
```

诊断模式仍然会加载默认 `robot_params.yaml` 和 `policies.yaml`，只是额外打开串口调试输出。

正常测试和跑机器人时，不建议长期打开 `debug_serial_hex_bytes`，因为大量串口日志会淹没终端信息。

### 6.4 当前 launch 参数

`bringup.launch.py` 当前有 3 个 launch argument：

| 参数 | 含义 | 单位 |
|---|---|---|
| `params_file` | 运行参数 YAML 路径 | 路径，无单位 |
| `params_override_file` | 可选运行参数覆盖 YAML 路径，默认空 | 路径，无单位 |
| `policies_file` | RL policy 配置 YAML 路径 | 路径，无单位 |

示例：

```bash
ros2 launch robot_bringup bringup.launch.py \
  params_file:=/abs/path/to/robot_params.yaml \
  policies_file:=/abs/path/to/policies.yaml
```

如果要临时叠加一个覆盖参数文件：

```bash
ros2 launch robot_bringup bringup.launch.py \
  params_override_file:=/abs/path/to/override.yaml
```

当前 launch 已经没有 `policy_path` 参数。策略路径统一在 `policies.yaml` 里配置。

### 6.5 启动后快速确认链路

完整 bringup 启动后，建议另开一个新终端，先执行：

```bash
source /opt/ros/foxy/setup.bash

cd ~/Desktop/RC2026/kvoy_quadruped
source install/setup.bash
```

然后按下面顺序检查。

查看节点：

```bash
ros2 node list
```

期望至少看到：

```text
/serial_comm_node
/imu_driver_node
/gamepad_input_node
/robot_fsm_node
/kinematics_node
/rl_controller_node
```

查看 topic：

```bash
ros2 topic list
```

核心 topic 应包含：

```text
/motor_cmd
/motor_state
/imu/data
/gamepad_cmd
/robot_state
```

检查 IMU 是否有数据：

```bash
ros2 topic hz /imu/data
```

检查 MCU 是否持续回传关节状态：

```bash
ros2 topic hz /motor_state
```

查看当前 FSM 状态：

```bash
ros2 topic echo /robot_state
```

查看手柄命令是否变化：

```bash
ros2 topic echo /gamepad_cmd
```

查看上层是否正在发送电机目标：

```bash
ros2 topic hz /motor_cmd
ros2 topic echo /motor_cmd
```

### 6.6 手柄进入运行状态的当前流程

当前真实组合键以 `gamepad_input_node.cpp` 为准：

| 操作 | 组合键 | 结果 |
|---|---|---|
| 起立 | `LT + LB + Y` | `WAITING -> STANDUP -> STANDING` |
| 进入/退出 RUNNING | `LT + LB + Y` | `STANDING <-> RUNNING` |
| 趴下 | `RT + RB + X` | 进入 `LIEDOWN` |
| 急停 | `LB + RB` | 进入 `ESTOP` |
| 上一个策略 | D-pad left | 切换 policy |
| 下一个策略 | D-pad right | 切换 policy |

运行完整测试时，建议顺序是：

1. 确认 `/imu/data` 正常发布
2. 确认 `/motor_state` 正常发布
3. 确认 `/gamepad_cmd` 能随手柄变化
4. 使用 `LT + LB + Y` 从 `WAITING` 进入 `STANDUP`
5. 等待进入 `STANDING`
6. 再次使用 `LT + LB + Y` 进入 `RUNNING`
7. 推动摇杆，观察 `/gamepad_cmd` 和 `/motor_cmd`

### 6.7 当前默认配置文件

- `src/robot_bringup/config/robot_params.yaml`
- `src/robot_bringup/config/policies.yaml`
- `src/robot_bringup/config/robot_params_serial_diag.yaml`，仅用于可选串口诊断覆盖

## 7. 当前数据流

```text
Gamepad -> /gamepad_cmd -> robot_fsm + rl_controller
IMU     -> /imu/data    -> rl_controller
MCU     -> /motor_state -> robot_fsm + robot_kinematics + rl_controller

robot_fsm        -> /robot_state -> serial_comm
robot_kinematics -> /motor_cmd   -> serial_comm
rl_controller    -> /motor_cmd   -> serial_comm
```

## 8. 全局单位与关节约定

### 8.1 关节顺序

整个工程统一使用以下 12 关节顺序：

```text
0=FL_hip   1=FL_thigh  2=FL_calf
3=FR_hip   4=FR_thigh  5=FR_calf
6=RL_hip   7=RL_thigh  8=RL_calf
9=RR_hip  10=RR_thigh 11=RR_calf
```

### 8.2 全局物理单位

| 量 | 单位 |
|---|---|
| 关节角度 `position` | `rad` |
| 关节角速度 `velocity` | `rad/s` |
| 关节力矩 `torque` | `N*m` |
| 机身前向/侧向速度命令 `vx, vy` | `m/s` |
| 机身偏航角速度命令 `yaw_rate` | `rad/s` |
| IMU 角速度 | `rad/s` |
| IMU 线加速度 | `m/s^2` |
| 四元数 | 无单位 |
| 时间 `*_duration_s`, `max_imu_age_s` | `s` |
| 频率 `*_rate_hz` | `Hz` |
| 串口重连/超时 `*_ms` | `ms` |
| 串口波特率 `baud_rate` | `baud` |

### 8.3 无量纲量

以下量当前应理解为无量纲：

| 量 | 说明 |
|---|---|
| 手柄原始轴 `axis_*` | 归一化轴值，范围通常在 `[-1, 1]` |
| `btn_*` | 布尔量 |
| `fsm_state` | 状态枚举 |
| `ctrl_mode` | 模式枚举 |
| `motor_polarity` | 只允许 `+1 / -1` |
| `imu_to_body_rotation` | 旋转矩阵元素 |
| RL 单步观测 `obs` | 送入网络前为无量纲 |
| RL 原始输出 `action` | 网络原始动作，当前代码按无量纲处理 |
| `clip_obs`, `clip_actions` | 无量纲裁剪阈值 |

## 9. Topic 与消息单位

### 9.1 Topic 总表

| Topic | 类型 | 方向 |
|---|---|---|
| `/motor_cmd` | `kvoy_msgs/msg/MotorCmd` | `robot_kinematics` / `rl_controller` -> `serial_comm` |
| `/motor_state` | `kvoy_msgs/msg/MotorState` | `serial_comm` -> 其他节点 |
| `/imu/data` | `kvoy_msgs/msg/ImuData` | `imu_driver` -> `rl_controller` |
| `/gamepad_cmd` | `kvoy_msgs/msg/GamepadCmd` | `gamepad_input` -> `robot_fsm` / `rl_controller` |
| `/robot_state` | `kvoy_msgs/msg/RobotState` | `robot_fsm` -> `serial_comm` / 监控 |

### 9.2 `MotorCmd`

文件：`src/kvoy_msgs/msg/MotorCmd.msg`

| 字段 | 含义 | 单位 |
|---|---|---|
| `header.stamp` | ROS 时间戳 | `s + ns` |
| `position[12]` | 目标关节角 | `rad` |
| `velocity[12]` | 目标关节角速度 | `rad/s` |
| `torque[12]` | 前馈关节力矩 | `N*m` |

当前代码里的真实使用情况：

- `robot_kinematics` 只写 `position`
- `rl_controller` 只写 `position`
- 这两个节点都会把 `velocity` 和 `torque` 置为 `0.0`

### 9.3 `MotorState`

文件：`src/kvoy_msgs/msg/MotorState.msg`

| 字段 | 含义 | 单位 |
|---|---|---|
| `header.stamp` | ROS 时间戳 | `s + ns` |
| `position[12]` | 当前关节角 | `rad` |
| `velocity[12]` | 当前关节角速度 | `rad/s` |
| `torque[12]` | 当前关节力矩 | `N*m` |

### 9.4 `ImuData`

文件：`src/kvoy_msgs/msg/ImuData.msg`

| 字段 | 含义 | 单位 |
|---|---|---|
| `header.stamp` | ROS 时间戳 | `s + ns` |
| `header.frame_id` | 坐标系名 | 字符串，无单位 |
| `orientation` | 四元数 `(x, y, z, w)` | 无单位 |
| `angular_velocity` | IMU 原始角速度 | `rad/s` |
| `linear_acceleration` | IMU 原始加速度，包含重力 | `m/s^2` |

### 9.5 `GamepadCmd`

文件：`src/kvoy_msgs/msg/GamepadCmd.msg`

高层命令字段：

| 字段 | 含义 | 单位 |
|---|---|---|
| `vx` | 前向速度命令 | `m/s` |
| `vy` | 侧向速度命令 | `m/s` |
| `yaw_rate` | 偏航角速度命令 | `rad/s` |
| `btn_standup` | 起立/切换运行态触发 | 布尔，无单位 |
| `btn_liedown` | 趴下触发 | 布尔，无单位 |
| `btn_estop` | 急停触发 | 布尔，无单位 |
| `btn_policy_prev` | 切换到上一个策略 | 布尔，无单位 |
| `btn_policy_next` | 切换到下一个策略 | 布尔，无单位 |

原始手柄字段：

| 字段 | 含义 | 单位 |
|---|---|---|
| `axis_lx, axis_ly, axis_rx, axis_ry` | 摇杆轴原始值 | 无量纲，通常 `[-1, 1]` |
| `axis_lt, axis_rt` | 扳机原始值 | 无量纲，通常 `[-1, 1]` |
| `axis_dpad_x, axis_dpad_y` | 十字键轴值 | 无量纲，通常 `[-1, 1]` |
| `btn_a/b/x/y/lb/rb/lt/rt/dpad_*` | 原始按钮状态 | 布尔，无单位 |

### 9.6 `RobotState`

文件：`src/kvoy_msgs/msg/RobotState.msg`

| 字段 | 含义 | 单位 |
|---|---|---|
| `header.stamp` | ROS 时间戳 | `s + ns` |
| `fsm_state` | 状态枚举值 | 枚举，无单位 |

枚举定义：

| 名称 | 数值 | 含义 |
|---|---:|---|
| `FSM_WAITING` | `0` | 等待态 |
| `FSM_STANDING` | `1` | 站立稳定态 |
| `FSM_RUNNING` | `2` | RL 运行态 |
| `FSM_STANDUP` | `3` | 起立插值过程 |
| `FSM_LIEDOWN` | `4` | 趴下插值过程 |
| `FSM_ESTOP` | `5` | 急停态 |

## 10. `gamepad_input` 真实行为与单位

文件：`src/gamepad_input/src/gamepad_input_node.cpp`

### 10.1 当前真实手柄映射

当前代码不是“单按钮直接起立/趴下/急停”，而是组合键：

| 功能 | 当前真实触发条件 |
|---|---|
| 起立 / `WAITING -> STANDUP` / `STANDING <-> RUNNING` | `LT && LB && Y` |
| 趴下 | `RT && RB && X` |
| 急停 | `LB && RB` |
| 上一个策略 | D-pad left |
| 下一个策略 | D-pad right |

注意：

- `robot_fsm_node` 启动时打印的日志仍然是简化提示
- 真正生效的是 `gamepad_input_node.cpp` 里生成的 `btn_standup / btn_liedown / btn_estop`
- 阅读和调试按钮触发时，以 `gamepad_input` 代码为准

### 10.2 速度命令计算

当前速度命令按以下公式生成：

```text
vx       = -deadzone(axis_ly) * 1.1
vy       = -deadzone(axis_lx) * 1.0
yaw_rate = -deadzone(axis_rx) * 2.0
```

因此上层收到的速度命令单位是：

- `vx`：`m/s`
- `vy`：`m/s`
- `yaw_rate`：`rad/s`

默认常量：

| 常量 | 当前值 | 单位 |
|---|---:|---|
| `VX_MAX` | `1.1` | `m/s` |
| `VY_MAX` | `1.0` | `m/s` |
| `YAW_MAX` | `2.0` | `rad/s` |
| `STICK_DEADZONE` | `0.1` | 无量纲 |
| `DIGITAL_AXIS_THRESHOLD` | `0.5` | 无量纲 |

### 10.3 节点参数

| 参数 | 含义 | 单位 | 当前默认值 |
|---|---|---|---:|
| `publish_rate_hz` | 手柄消息发布频率 | `Hz` | `50.0` |

## 11. `imu_driver` 真实行为与单位

文件：

- `src/imu_driver/src/imu_driver_node.cpp`
- `src/imu_driver/src/wit_motion_imu_node.cpp`

### 11.1 当前后端

当前 `imu_driver` 使用 Linux 原生 `termios` 打开串口，不使用 `libserialport`。

### 11.2 当前 IMU 串口参数

`robot_params.yaml` 当前默认：

| 参数 | 含义 | 单位 | 当前值 |
|---|---|---|---|
| `serial_port` | IMU 串口路径 | 路径 | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` |
| `baud_rate` | 串口波特率 | `baud` | `115200` |
| `frame_id` | ROS frame 名 | 字符串 | `imu_link` |
| `publish_rate_hz` | ROS 读取/发布循环频率 | `Hz` | `200.0` |
| `reconnect_interval_ms` | 掉线重连间隔 | `ms` | `500` |

重要说明：

- `publish_rate_hz` 是 ROS 节点定时器频率
- 当前驱动不会向 IMU 下发“修改回传频率/波特率”的配置命令
- IMU 模块本身必须预先被配置成你需要的波特率和回传率

### 11.3 当前协议与物理量换算

WIT 串口包固定为 `11` 字节：

```text
0x55 + type(1B) + data(8B) + checksum(1B)
```

当前驱动使用三类包：

| 包类型 | 含义 |
|---|---|
| `0x51` | 加速度 |
| `0x52` | 角速度 |
| `0x59` | 四元数 |

当前代码里的换算公式：

```text
accel[m/s^2] = raw_i16 / 32768 * 16 * 9.80665
gyro[rad/s]  = raw_i16 / 32768 * 2000 deg/s * pi/180

WIT quaternion packet order:
q0, q1, q2, q3 = w, x, y, z

ROS publish order:
x, y, z, w
```

也就是说：

- `/imu/data.orientation` 是单位四元数，发布顺序为 `x,y,z,w`
- `/imu/data.angular_velocity` 已经是 `rad/s`
- `/imu/data.linear_acceleration` 已经是 `m/s^2`

### 11.4 当前 IMU 坐标处理

`imu_driver` 当前不做安装补偿，不做 body frame 旋转。

发布出去的 `/imu/data` 含义是：

- `orientation`：IMU 传感器坐标系对应的姿态四元数
- `angular_velocity`：IMU 传感器坐标系角速度
- `linear_acceleration`：IMU 传感器坐标系加速度

`imu_to_body_rotation` 当前只在两个地方使用：

- `rl_controller`
- `imu_diagnostics.py`

也就是说，`imu_driver` 本身不使用 `imu_to_body_rotation`。

### 11.5 当前建议的 IMU 回传频率

在当前工程 `50 Hz` 控制频率下，当前更合适的 IMU 回传频率是：

```text
100-200 Hz
```

当前配置使用 `200 Hz`，这也是当前代码和 Jetson 实测一致的默认值。

### 11.6 单独测试 IMU

```bash
source /opt/ros/foxy/setup.bash
source ~/Desktop/RC2026/kvoy_quadruped/install/setup.bash

ros2 run imu_driver imu_driver_node --ros-args \
  -p serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  -p baud_rate:=115200 \
  -p publish_rate_hz:=200.0
```

诊断脚本：

```bash
source /opt/ros/foxy/setup.bash
source ~/Desktop/RC2026/kvoy_quadruped/install/setup.bash

ros2 run imu_driver imu_diagnostics.py --ros-args \
  -p imu_to_body_rotation:="[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]"
```

## 12. `robot_kinematics` 真实行为与单位

文件：

- `src/robot_kinematics/include/robot_kinematics/joint_config.hpp`
- `src/robot_kinematics/src/kinematics_node.cpp`

### 12.1 默认姿态单位

`STAND_POS` 和 `LIE_POS` 的单位都是 `rad`。

例如当前编译时默认站姿：

```text
FL: [0.0, 0.8, -1.5]
FR: [0.0, 0.8, -1.5]
RL: [0.0, 1.0, -1.5]
RR: [0.0, 1.0, -1.5]
```

全部都是关节角 `rad`。

### 12.2 当前运动方式

当前起立/趴下不是 IK/动力学控制，而是双段关节空间插值：

```text
current_pos -> transition_pose -> target_pose
```

插值函数使用 `smoothstep`，当前第一段占总时长比例固定为 `0.5`。

### 12.3 节点参数

| 参数 | 含义 | 单位 | 当前默认值 |
|---|---|---|---:|
| `standup_duration_s` | 起立总时长 | `s` | `5.0` |
| `liedown_duration_s` | 趴下总时长 | `s` | `5.0` |
| `control_rate_hz` | 插值控制频率 | `Hz` | `50.0` |
| `stand_pose[12]` | 站姿目标 | `rad` | 见 `robot_params.yaml` |
| `stand_transition_pose[12]` | 起立中间姿态 | `rad` | 见 `robot_params.yaml` |
| `lie_pose[12]` | 趴下目标 | `rad` | 见 `robot_params.yaml` |
| `lie_transition_pose[12]` | 趴下中间姿态 | `rad` | 见 `robot_params.yaml` |

### 12.4 输出单位

`robot_kinematics` 发布到 `/motor_cmd` 的量：

- `position[i]`：关节目标角，单位 `rad`
- `velocity[i]`：固定 `0.0 rad/s`
- `torque[i]`：固定 `0.0 N*m`

## 13. `rl_controller` 真实行为、单位与网络接口

文件：

- `src/rl_controller/include/rl_controller/rl_controller_node.hpp`
- `src/rl_controller/src/rl_controller_node.cpp`

### 13.1 编译和运行要求

`rl_controller` 是 TensorRT 后端：

- 编译时需要 `NvInfer.h`
- 编译时需要 `cuda_runtime_api.h`
- 链接时需要 `nvinfer` 和 `cudart`

如果没有 TensorRT/CUDA dev，`rl_controller` 会在 CMake 阶段直接报错。

### 13.2 当前策略装载行为

`robot_fsm` 总是会创建 `rl_controller_node`，但是否能真正推理取决于 `policies.yaml` 里是否配置了有效的 engine 路径。

当前 `policies.yaml` 默认是空路径，因此真实行为是：

- 节点会启动
- 会打印 “no valid policy paths were provided”
- `RUNNING` 状态下也不会真正执行推理控制

### 13.3 当前 IMU 处理

`rl_controller` 从 `/imu/data` 中使用：

- `angular_velocity`
- `orientation`

处理方式：

```text
gravity_sensor = quat_rotate_inverse(sensor_quat, [0, 0, -1])
ang_vel_body   = R_imu_to_body * sensor_ang_vel
gravity_body   = R_imu_to_body * gravity_sensor
```

因此：

- `sensor_ang_vel` 输入单位是 `rad/s`
- `gravity_sensor` / `gravity_body` 是单位重力方向向量，无量纲
- `imu_to_body_rotation` 是 `3x3` 行优先旋转矩阵，无量纲

### 13.4 当前单步观测定义

当前代码固定支持单步观测 `45` 维：

```text
commands(3)
ang_vel(3)
projected_gravity(3)
dof_pos(12)
dof_vel(12)
last_action(12)
```

更精确的单位关系如下：

| 观测项 | 原始物理量 | 原始单位 | 送入网络前 |
|---|---|---|---|
| `commands[0:2]` | `vx, vy, yaw_rate` | `m/s, m/s, rad/s` | 乘以 `cmd_lin_vel_scale / cmd_ang_vel_scale` 后变为无量纲 |
| `ang_vel[0:2]` | body angular velocity | `rad/s` | 乘以 `obs_ang_vel_scale` 后变为无量纲 |
| `projected_gravity[0:2]` | body frame gravity direction | 无量纲 | 直接输入 |
| `dof_pos[0:11]` | `joint_pos - default_joint_pos` | `rad` | 乘以 `obs_dof_pos_scale` 后变为无量纲 |
| `dof_vel[0:11]` | joint velocity | `rad/s` | 乘以 `obs_dof_vel_scale` 后变为无量纲 |
| `last_action[0:11]` | 上一时刻网络原始动作 | 无量纲 | 直接输入 |

之后所有观测会再经过：

```text
obs[i] = clamp(obs[i], -clip_obs, clip_obs)
```

所以送入 TensorRT 的最终观测是无量纲。

### 13.5 历史长度和输入输出形状

当前默认：

| 参数 | 当前值 | 单位 |
|---|---:|---|
| `history_steps` | `6` | 步 |
| `one_step_obs` | `45` | 维 |

因此当前默认 TensorRT 输入形状是：

```text
[1, 270]
```

当前允许的输出形状：

```text
[1, 12] 或 [12]
```

### 13.6 当前网络输出到底是什么单位

这是最重要的单位说明之一。

当前代码里，TensorRT engine 的原始输出 `action[12]` 被当作“无量纲动作”使用：

```text
action[i] = clamp(raw_network_output[i], -clip_actions, clip_actions)
target_pos[i] = default_joint_pos[i] + action[i] * action_scale
```

因此：

- 网络原始输出 `raw_network_output[i]`：无量纲
- 裁剪后的 `action[i]`：无量纲
- `action_scale`：把无量纲动作变成关节角偏移的尺度，等价于 `rad / action_unit`
- `action[i] * action_scale`：关节角偏移，单位 `rad`
- `default_joint_pos[i]`：基准关节角，单位 `rad`
- 最终 `/motor_cmd.position[i]`：关节目标角，单位 `rad`

结论：

- 网络输出不是 `degree`
- 网络输出也不是直接的绝对关节角 `rad`
- 网络输出当前表示“无量纲动作偏移”
- 上位机最终发给 `serial_comm` 的关节目标角才是 `rad`

### 13.7 当前 `policies.yaml` 中各参数的单位

文件：`src/robot_bringup/config/policies.yaml`

每个 policy slot 都有独立参数：

| 参数 | 含义 | 单位 |
|---|---|---|
| `policy_X.name` | 策略名 | 字符串，无单位 |
| `policy_X.path` | engine 路径 | 路径，无单位 |
| `policy_X.history_steps` | 观测历史步数 | 步 |
| `policy_X.one_step_obs` | 单步观测维度 | 维 |
| `policy_X.default_joint_pos[12]` | 策略基准姿态 | `rad` |
| `policy_X.action_scale` | 动作到关节偏移的缩放 | `rad / action_unit` |
| `policy_X.clip_obs` | 观测裁剪阈值 | 无量纲 |
| `policy_X.clip_actions` | 动作裁剪阈值 | 无量纲 |
| `policy_X.obs_ang_vel_scale` | `rad/s -> normalized` 的缩放 | 结果无量纲 |
| `policy_X.obs_dof_pos_scale` | `rad -> normalized` 的缩放 | 结果无量纲 |
| `policy_X.obs_dof_vel_scale` | `rad/s -> normalized` 的缩放 | 结果无量纲 |
| `policy_X.cmd_lin_vel_scale` | `m/s -> normalized` 的缩放 | 结果无量纲 |
| `policy_X.cmd_ang_vel_scale` | `rad/s -> normalized` 的缩放 | 结果无量纲 |

### 13.8 `policy_new.pt` 转 ONNX 和 TensorRT engine

当前工程的模型转换必须以 `rl_controller` 的真实接口为准：

```text
input_name  = obs
input_shape = [1, 270]
output_name = action
output_shape = [1, 12] 或 [12]
input/output dtype = float32
```

从 `policy_new.pt` 导出 ONNX：

```bash
cd /home/robocon-2026/Desktop/RC2026/kvoy_quadruped

source .venv_policy_export/bin/activate
python tools/export_policy_onnx.py --input models/policy_2_blind_terrain/policy_new.pt --output models/policy_2_blind_terrain/policy_new.onnx
```

从 ONNX 转 TensorRT engine：

```bash
cd /home/robocon-2026/Desktop/RC2026/kvoy_quadruped

python3 tools/convert_policy_onnx_to_trt.py --input models/policy_2_blind_terrain/policy_new.onnx --output models/policy_2_blind_terrain/policy_new.engine --skip-onnx-check
```

注意：

- 第二步必须用 `python3`，不能用系统默认 `python`，因为当前系统默认 `python` 是 Python 2。
- 以上命令只生成模型文件，不会自动修改 `src/robot_bringup/config/policies.yaml`。
- 如果要让 bringup 默认加载新模型，需要手动把 `policy_2.path` 指向 `policy_new.engine`。
- `trtexec` 构建过程中出现 `INT64 -> INT32` 的 TensorRT 警告通常是 ONNX 常量类型转换提示，不等于转换失败；以最终 `PASSED` 和脚本 `[OK]` 为准。

### 13.9 当前 RL 输出到电机命令的单位

`rl_controller` 发布到 `/motor_cmd` 的值：

- `position[i]`：`rad`
- `velocity[i]`：固定 `0.0 rad/s`
- `torque[i]`：固定 `0.0 N*m`

### 13.10 当前运行保护

当前已实现：

- `max_imu_age_s`：IMU 超时后跳过当前推理周期
- 策略切换或进入 `RUNNING` 时，先保持当前关节位置 `10` 个控制周期

当前 `kPolicySwitchHoldCycles = 10`，在 `50 Hz` 下约等于：

```text
10 / 50 = 0.2 s
```

### 13.11 当前没有做的保护

当前代码里没有做上位机关节限位裁剪：

- `rl_controller` 不裁剪最终 `target_pos`
- `robot_kinematics` 不裁剪插值姿态
- `serial_comm` 只检查是否为有限值，不做角度上下限保护

这意味着真机安全还依赖：

- 你的 `default_joint_pos`
- `action_scale`
- engine 输出分布
- MCU 端自己的保护

## 14. `robot_fsm` 真实行为与单位

文件：`src/robot_fsm/src/robot_fsm_node.cpp`

### 14.1 当前状态机

```text
WAITING  --[press standup]--> STANDUP  --[kinematics done]--> STANDING
STANDING --[press standup]--> RUNNING
RUNNING  --[press standup]--> STANDING
STANDING --[press liedown && zero velocity cmd]--> LIEDOWN --[done]--> WAITING
any      --[press estop]--> ESTOP
ESTOP    --[press standup]--> STANDUP
```

### 14.2 `RUNNING` 是否等于 RL 真正输出

不一定。

当前要区分两层含义：

1. `FSM` 层面进入 `RUNNING`
2. `rl_controller` 实际已经装载有效 TensorRT policy 并开始发布命令

如果 `policy path` 为空，那么即使进入 `RUNNING`，RL 也不会真正推理。

### 14.3 零速度判定阈值

`robot_fsm` 当前趴下前要求速度命令接近零，阈值为：

```text
kZeroCmdThreshold = 0.05
```

对应物理含义：

- `|vx| <= 0.05 m/s`
- `|vy| <= 0.05 m/s`
- `|yaw_rate| <= 0.05 rad/s`

### 14.4 与 `robot_state` / `serial_comm` 的关系

`robot_fsm` 会发布 `/robot_state`，`serial_comm` 根据它切换 `ctrl_mode`：

- `WAITING` -> damping
- `ESTOP` -> damping
- 其他状态 -> motion

## 15. `serial_comm` 真实行为、单位与协议

文件：

- `src/serial_comm/src/serial_comm_node.cpp`
- `src/serial_comm/include/serial_comm/protocol.hpp`

### 15.1 当前后端

当前 `serial_comm` 使用 Linux 原生 `termios`，不使用 `libserialport`。

### 15.2 当前默认参数

文件：`src/robot_bringup/config/robot_params.yaml`

| 参数 | 含义 | 单位 | 当前值 |
|---|---|---|---|
| `serial_port` | MCU 串口路径 | 路径 | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AF7082020-if00` |
| `baud_rate` | 串口波特率 | `baud` | `2000000` |
| `reconnect_interval_ms` | 断线重连节流时间 | `ms` | `200` |
| `rx_timeout_ms` | 合法状态帧接收超时 | `ms` | `200` |
| `motor_polarity[12]` | 电机方向符号 | 无量纲，`+1/-1` | `[1, 1, 1, 1, -1, -1, -1, 1, 1, -1, -1, -1]` |
| `motor_zero_offset[12]` | 手动零点偏移 | `rad` | 全 `0.0` |
| `debug_serial_io` | 调试日志开关 | 布尔，无单位 | `false` |
| `debug_serial_hex_bytes` | 是否打印原始十六进制帧 | 布尔，无单位 | `false` |

重要说明：

- 当前默认 `serial_port` 已固定为 MCU 的稳定 `by-id` 路径，不依赖 `/dev/ttyACM0` 这种会漂移的内核名

### 15.3 当前支持的波特率

当前 `termios` 后端代码显式支持：

```text
9600
19200
38400
57600
115200
230400
460800
500000
576000
921600
1000000
1152000
1500000
2000000
2500000
3000000
3500000
4000000
```

Jetson 上已经实测 CH343 自环通过：

- `2000000`
- `3000000`
- `4000000`

### 15.4 `motor_polarity` 和 `motor_zero_offset` 的单位与换算

上位机始终希望上层节点工作在 URDF/训练坐标系。

因此 `serial_comm` 会在 ROS 侧与 MCU 侧之间做一层换算。

参数定义：

- `motor_polarity[i] = +1`：实物安装方向与 URDF/训练正方向一致
- `motor_polarity[i] = -1`：实物安装方向相反
- `motor_zero_offset[i]`：固定手动零点偏移，单位 `rad`

从 MCU `state frame` 到上位机 `/motor_state`：

```text
orin_pos = mcu_pos * polarity + motor_zero_offset
orin_vel = mcu_vel * polarity
orin_tau = mcu_tau * polarity
```

从上位机 `/motor_cmd` 到 MCU `cmd frame`：

```text
mcu_target_pos = (orin_target_pos - motor_zero_offset) * polarity
mcu_target_vel = orin_target_vel * polarity
mcu_target_tau = orin_target_tau * polarity
```

这意味着：

- 上层 ROS 节点看到和发送的 `position` 单位始终是 `rad`
- 串口线上发送给 MCU 的 `position` 也仍然是 `rad`
- 当前 MCU 传回的角度已经按 URDF 零位参考表达，因此默认 `motor_zero_offset` 为全 `0.0`
- 如果后续只需要临时补偿固定零偏，改 `motor_zero_offset`，不再有启动自动采样校准逻辑
- 发送前会按手动零偏和方向把 URDF 坐标系还原到 MCU 命令坐标系

### 15.5 `ctrl_mode` 切换规则

当前 `ctrl_mode` 为全局单字节枚举：

| 数值 | 含义 |
|---|---|
| `0` | `damping` |
| `1` | `motion` |

当前 `serial_comm` 依据 `/robot_state.fsm_state` 自动切换：

| FSM 状态 | 当前 `ctrl_mode` |
|---|---|
| `WAITING` | `damping` |
| `ESTOP` | `damping` |
| `STANDUP` | `motion` |
| `STANDING` | `motion` |
| `RUNNING` | `motion` |
| `LIEDOWN` | `motion` |

### 15.6 当前串口 on-wire 单位

这是另一个最关键的单位结论。

当前 Orin 与 MCU 之间二进制帧中的 36 个 `float32`，单位就是：

- `position[12]`：`rad`
- `velocity[12]`：`rad/s`
- `torque[12]`：`N*m`

不是 `degree`，不是编码器 tick，也不是百分比。

换句话说，CH343 这条线上当前发送和接收的关节角度单位是：

```text
rad
```

### 15.7 `cmd frame`：Orin -> MCU

总长 `155` 字节：

```text
[0]       header0      = 0xAA
[1]       header1      = 0x55
[2]       version      = 0x01
[3]       type         = 0x01
[4:5]     payload_len  = uint16 LE = 145
[6]       ctrl_mode    = uint8
[7:150]   payload      = 36 x float32 LE
[151:152] crc16        = CRC16-CCITT-FALSE
[153]     tail0        = 0x0D
[154]     tail1        = 0x0A
```

payload 顺序：

```text
position[12]  # rad
velocity[12]  # rad/s
torque[12]    # N*m
```

### 15.8 `state frame`：MCU -> Orin

总长 `154` 字节：

```text
[0]       header0      = 0xAA
[1]       header1      = 0x55
[2]       version      = 0x01
[3]       type         = 0x02
[4:5]     payload_len  = uint16 LE = 144
[6:149]   payload      = 36 x float32 LE
[150:151] crc16        = CRC16-CCITT-FALSE
[152]     tail0        = 0x0D
[153]     tail1        = 0x0A
```

payload 顺序：

```text
position[12]  # rad
velocity[12]  # rad/s
torque[12]    # N*m
```

所有浮点都是：

- `float32`
- little-endian

MCU 端不要假设结构体内存布局完全一致，最好按字节序列解析。

### 15.9 当前串口保护

当前节点已经实现：

- 串口打开失败自动后台重连
- 串口读异常自动关闭并重连
- 串口写异常自动关闭并重连
- 部分写出视为故障
- 超过 `rx_timeout_ms` 没有收到合法状态帧则判定故障
- 帧头重同步
- CRC 检查
- `NaN / Inf` 过滤
- 接收缓存溢出后丢弃旧字节重新同步

## 16. `robot_params.yaml` 单位总表

文件：`src/robot_bringup/config/robot_params.yaml`

### 16.1 `serial_comm_node`

| 参数 | 单位 |
|---|---|
| `serial_port` | 路径，无单位 |
| `baud_rate` | `baud` |
| `reconnect_interval_ms` | `ms` |
| `rx_timeout_ms` | `ms` |
| `motor_polarity[12]` | 无量纲，`+1/-1` |
| `motor_zero_offset[12]` | `rad` |

### 16.2 `imu_driver_node`

| 参数 | 单位 |
|---|---|
| `serial_port` | 路径，无单位 |
| `baud_rate` | `baud` |
| `frame_id` | 字符串，无单位 |
| `publish_rate_hz` | `Hz` |
| `reconnect_interval_ms` | `ms` |

### 16.3 `gamepad_input_node`

| 参数 | 单位 |
|---|---|
| `publish_rate_hz` | `Hz` |

### 16.4 `robot_fsm_node`

| 参数 | 单位 |
|---|---|
| `stand_transition_pose[12]` | `rad` |
| `stand_pose[12]` | `rad` |
| `lie_transition_pose[12]` | `rad` |
| `lie_pose[12]` | `rad` |
| `rl_control_rate_hz` | `Hz` |
| `action_scale` | `rad / action_unit` |
| `clip_obs` | 无量纲 |
| `clip_actions` | 无量纲 |
| `obs_ang_vel_scale` | 使 `rad/s` 归一化后的缩放 |
| `obs_dof_pos_scale` | 使 `rad` 归一化后的缩放 |
| `obs_dof_vel_scale` | 使 `rad/s` 归一化后的缩放 |
| `cmd_lin_vel_scale` | 使 `m/s` 归一化后的缩放 |
| `cmd_ang_vel_scale` | 使 `rad/s` 归一化后的缩放 |
| `max_imu_age_s` | `s` |
| `policy_switch_sound_enabled` | 布尔，无单位 |
| `policy_switch_sound_file` | 路径，无单位 |
| `policy_switch_sound_interval_ms` | `ms` |
| `imu_to_body_rotation[9]` | 无量纲 |
| `standup_duration_s` | `s` |
| `liedown_duration_s` | `s` |
| `kinematics_control_rate_hz` | `Hz` |

## 17. 当前 bringup 验证边界

截至本文档更新时间，当前工程在 Jetson 上已经验证到的范围：

- `imu_driver` 通
- `gamepad_input` 通
- `robot_fsm` 状态切换通
- `serial_comm` 串口打开、波特率生效、CH343 高波特率链路通

还没有验证到的范围：

- MCU 真实 `state frame` 回传
- `/motor_state` 驱动真实起立姿态闭环
- 带 TensorRT engine 的 RL 推理闭环
- 真机全链路安全保护

因此当前工程应理解为：

- “Jetson bringup 基础链路已跑通”
- “完整真机 RL 闭环还没有全部跑通”

## 18. 建议的下一步验证顺序

1. 用当前固定的 MCU `by-id` 路径验证 `state frame` 和 `/motor_state` 单位、方向、零偏是否正确。
2. 用 `motor_polarity` 和 `motor_zero_offset` 校准 Orin/URDF 坐标系与真实机构。
3. 验证 `RUNNING` 状态下 RL 是否真的开始发布关节目标。
4. 在 MCU 端补齐关节限位、超时和急停保护，不要只依赖上位机。

## 19. 快速结论

如果只问当前工程里几个最容易混淆的单位结论：

- CH343/串口线上发送和接收的关节角度单位：`rad`
- `/motor_cmd.position` 和 `/motor_state.position`：`rad`
- 网络原始输出 `action`：无量纲
- 网络输出乘 `action_scale` 后得到的关节偏移：`rad`
- 最终发给 MCU 的目标关节角：`rad`
- IMU 角速度：`rad/s`
- IMU 线加速度：`m/s^2`
- 手柄速度命令 `vx/vy`：`m/s`
- 手柄速度命令 `yaw_rate`：`rad/s`
