# kvoy_quadruped

`kvoy_quadruped` 是一个面向自制四足机器狗的 ROS 2 sim-to-real 工程。当前仓库已经具备完整上位机链路：手柄输入、IMU 读取、Jetson 与 MCU 串口通信、起立/趴下状态机、以及基于 TensorRT 的强化学习策略推理。

这份 README 只描述当前仓库里真实存在的结构、脚本、配置和默认行为，不再保留过时的实现细节说明。

## 1. 当前工程概览

当前工程由以下几部分组成：

- `gamepad_input`：读取 GameSir 手柄并发布 `/gamepad_cmd`
- `imu_driver`：读取串口 IMU 并发布 `/imu/data`
- `serial_comm`：负责 Jetson 与 MCU 之间的串口收发
- `robot_kinematics`：负责起立/趴下过程中的关节轨迹插值
- `rl_controller`：加载 TensorRT engine，基于 IMU、关节状态和手柄命令做策略推理
- `robot_fsm`：顶层状态机，组合 `robot_kinematics` 和 `rl_controller`
- `robot_bringup`：统一 launch、参数文件、systemd 服务脚本和辅助脚本
- `kvoy_msgs`：自定义消息定义

当前工程是一个标准 `colcon` 工作空间，核心代码位于 `src/`，策略模型位于 `models/`，导出和转换工具位于 `tools/`。

## 2. 目录结构

```text
.
├── config/                         # 额外系统配置，如 udev 规则
├── models/                         # 已导出的 ONNX / TensorRT 策略模型
├── tools/                          # 策略导出与 TensorRT 转换脚本
├── src/
│   ├── gamepad_input/
│   ├── imu_driver/
│   ├── kvoy_msgs/
│   ├── rl_controller/
│   ├── robot_bringup/
│   ├── robot_fsm/
│   ├── robot_kinematics/
│   └── serial_comm/
└── README.md
```

## 3. 节点与数据流

### 3.1 启动后的主要节点

`robot_bringup/launch/bringup.launch.py` 当前会启动：

- `serial_comm_node`
- `imu_driver_node`
- `gamepad_input_node`
- `robot_fsm_node`
- `bringup_restart_supervisor`，默认启用，可通过 launch 参数关闭
- `startup_sound_node`，默认启用，可通过 launch 参数关闭

其中 `robot_fsm_node` 进程内部还会创建两个组合节点：

- `kinematics_node`
- `rl_controller_node`

### 3.2 当前数据流

```text
Gamepad -> /gamepad_cmd -> robot_fsm + rl_controller
IMU     -> /imu/data    -> rl_controller
MCU     -> /motor_state -> robot_fsm + rl_controller

robot_kinematics -> /motor_cmd -> serial_comm -> MCU
rl_controller    -> /motor_cmd -> serial_comm -> MCU
robot_fsm        -> /robot_state -> serial_comm + supervisor
```

### 3.3 核心 topic

| Topic | 类型 | 说明 |
|---|---|---|
| `/gamepad_cmd` | `kvoy_msgs/msg/GamepadCmd` | 手柄速度命令和组合键状态 |
| `/imu/data` | `kvoy_msgs/msg/ImuData` | IMU 四元数、角速度、线加速度 |
| `/motor_state` | `kvoy_msgs/msg/MotorState` | MCU 返回的 12 关节状态 |
| `/motor_cmd` | `kvoy_msgs/msg/MotorCmd` | 上位机发给 MCU 的 12 关节目标 |
| `/robot_state` | `kvoy_msgs/msg/RobotState` | 顶层有限状态机状态 |

## 4. 状态机与控制逻辑

当前 `robot_fsm` 的状态包括：

- `WAITING`
- `STANDING`
- `RUNNING`
- `STANDUP`
- `LIEDOWN`
- `ESTOP`

状态切换逻辑以当前源码为准：

- `WAITING -> STANDUP`：按下起立组合键，且已经收到有效 `/motor_state`
- `STANDUP -> STANDING`：起立轨迹完成
- `STANDING <-> RUNNING`：再次按下起立组合键
- `STANDING -> LIEDOWN`：按下趴下组合键，且速度命令接近零
- `LIEDOWN -> WAITING`：趴下轨迹完成
- 任意状态 `-> ESTOP`：按下急停组合键
- `ESTOP -> WAITING`：按下趴下组合键
- `ESTOP -> STANDUP`：按下起立组合键，且已收到有效 `/motor_state`

当前实现里，`robot_kinematics` 和 `rl_controller` 都只发布关节位置目标；`velocity` 和 `torque` 统一发 `0.0`。

## 5. 手柄映射与当前按键行为

`gamepad_input` 当前使用 SDL2 读取手柄，默认优先选择名称包含 `GameSir` 的设备。

当前高层控制逻辑如下：

- 左摇杆：平动速度命令 `vx / vy`
- 右摇杆 X 轴：偏航角速度命令 `yaw_rate`
- `LT + LB + Y`：起立，或在 `STANDING / RUNNING` 之间切换
- `RT + RB + X`：趴下；在 `ESTOP` 状态下用于回到 `WAITING`
- `LB + RB`：急停
- D-pad 左长按：切换到上一个策略
- D-pad 右长按：切换到下一个策略
- D-pad 上下：仅当当前策略启用高度命令且速度命令接近零时，用于调节高度命令

策略切换不是短按，而是长按触发。长按时长由 `gamepad_input_node.policy_switch_long_press_s` 控制。

## 6. 消息与单位

整个工程当前统一采用以下关节顺序：

```text
0=FL_hip   1=FL_thigh  2=FL_calf
3=FR_hip   4=FR_thigh  5=FR_calf
6=RL_hip   7=RL_thigh  8=RL_calf
9=RR_hip  10=RR_thigh 11=RR_calf
```

核心物理单位如下：

- 关节位置：`rad`
- 关节速度：`rad/s`
- 关节力矩：`N*m`
- 机身速度命令 `vx / vy`：`m/s`
- 偏航角速度命令 `yaw_rate`：`rad/s`
- IMU 角速度：`rad/s`
- IMU 线加速度：`m/s^2`

自定义消息定义位于 `src/kvoy_msgs/msg/`：

- `MotorCmd.msg`
- `MotorState.msg`
- `ImuData.msg`
- `GamepadCmd.msg`
- `RobotState.msg`

## 7. 依赖环境

从当前源码看，构建和运行至少依赖：

- ROS 2 Foxy
- `colcon`
- `ament_cmake`
- `libsdl2-dev`
- TensorRT 开发库和 CUDA runtime/dev
- Python 3 与 `rclpy`

最基本的系统依赖安装可以从这里开始：

```bash
sudo apt update
sudo apt install libsdl2-dev
```

说明：

- `serial_comm` 和 `imu_driver` 当前都基于 `termios`，不再依赖 `libserialport`
- 只要构建 `rl_controller` 或 `robot_fsm`，系统里就必须能找到 TensorRT 和 CUDA 头文件/库
- 如果要播放启动提示音，系统中还需要 `aplay`，通常来自 `alsa-utils`

## 8. 构建

```bash
source /opt/ros/foxy/setup.bash

cd ~/Desktop/RC2026/kvoy_quadruped
colcon build
source install/setup.bash
```

如果只构建本工程相关包：

```bash
source /opt/ros/foxy/setup.bash

cd ~/Desktop/RC2026/kvoy_quadruped
colcon build --packages-select \
  kvoy_msgs \
  serial_comm \
  imu_driver \
  gamepad_input \
  robot_kinematics \
  rl_controller \
  robot_fsm \
  robot_bringup
source install/setup.bash
```

## 9. 运行

### 9.1 默认启动

```bash
source /opt/ros/foxy/setup.bash

cd ~/Desktop/RC2026/kvoy_quadruped
source install/setup.bash
ros2 launch robot_bringup bringup.launch.py
```

### 9.2 当前 launch 参数

当前 `bringup.launch.py` 支持以下参数：

- `params_file`
- `params_override_file`
- `policies_file`
- `enable_restart_supervisor`
- `enable_startup_sound`

例如：

```bash
ros2 launch robot_bringup bringup.launch.py \
  params_override_file:=src/robot_bringup/config/robot_params_serial_diag.yaml \
  enable_restart_supervisor:=false \
  enable_startup_sound:=false
```

### 9.3 启动后建议检查

```bash
ros2 node list
ros2 topic list
ros2 topic hz /imu/data
ros2 topic hz /motor_state
ros2 topic echo /robot_state
```

正常情况下，至少应能看到：

- `/serial_comm_node`
- `/imu_driver_node`
- `/gamepad_input_node`
- `/robot_fsm_node`
- `/kinematics_node`
- `/rl_controller_node`

## 10. 配置文件

当前运行时最重要的配置文件有三个：

- `src/robot_bringup/config/robot_params.yaml`
- `src/robot_bringup/config/policies.yaml`
- `src/robot_bringup/config/robot_params_serial_diag.yaml`

### 10.1 `robot_params.yaml`

当前用于集中配置：

- MCU 串口路径与波特率
- IMU 串口路径与波特率
- 手柄速度上限、死区和重连间隔
- 起立/站立/趴下姿态
- RL 控制频率与安全限幅
- IMU 观测滤波
- restart supervisor 参数
- 启动提示音参数

注意：当前默认串口路径使用的是 `/dev/serial/by-path/...`，不是旧文档中的 `/dev/serial/by-id/...`。

### 10.2 `policies.yaml`

当前策略系统支持三个 slot：

- `policy_1`
- `policy_2`
- `policy_3`

默认策略是 `policy_2`。当前配置中：

- `policy_1`：46 维单步观测，支持高度命令
- `policy_2`：45 维单步观测，不带高度命令
- `policy_3`：预留 slot，默认空路径

每个 slot 都可以独立配置：

- 模型路径
- `history_steps`
- `one_step_obs`
- 默认关节姿态
- 动作缩放
- 观测缩放
- 是否启用高度命令

### 10.3 `robot_params_serial_diag.yaml`

这是一个覆盖文件，只额外打开串口调试相关参数，便于观察原始串口收发，不是主配置文件。

## 11. 模型文件与工具脚本

### 11.1 当前仓库中的模型

`models/` 目录下当前已经包含：

- `policy_1_blind_plane/`
- `policy_2_blind_terrain/`

目录中同时存在：

- `policy.pt`
- 多个 `.onnx`
- 多个 `.engine`

README 不再对每个模型做逐个解释，实际使用哪个模型由 `policies.yaml` 决定。

### 11.2 导出与转换工具

`tools/` 中当前有四个脚本：

- `export_policy_onnx.py`
- `export_policy_46_onnx.py`
- `convert_policy_onnx_to_trt.py`
- `convert_policy_46_onnx_to_trt.py`

它们对应两类策略接口：

- 45 维单步观测版本
- 46 维单步观测版本（含高度命令）

如果只是在机器人上运行现有策略，通常不需要改这些脚本。

## 12. 自启动与辅助服务

`robot_bringup` 当前包含两类 systemd 服务模板：

- `kvoy-bringup.service`
- `kvoy-gamesir-watchdog.service`

相关脚本位于：

- `src/robot_bringup/scripts/install_user_service.sh`
- `src/robot_bringup/scripts/install_gamesir_watchdog_service.sh`
- `src/robot_bringup/scripts/kvoy_bringup_autostart.sh`
- `src/robot_bringup/scripts/gamesir_usb_watchdog.py`

### 12.1 `kvoy-bringup.service`

这是用户级 `systemd --user` 服务，用于开机自启动整套 ROS bringup。

### 12.2 `kvoy-gamesir-watchdog.service`

这是系统级 watchdog，用于处理部分 GameSir 接收器错误枚举为 `3537:0575`、导致没有 Linux joystick 设备的问题。恢复成功后它会重启 bringup 服务，让 SDL 重新打开手柄。

### 12.3 当前服务模板中的硬编码路径

这一点很重要：当前仓库里的 service 模板和安装脚本仍然保留了部署机器上的硬编码默认值，例如：

- `/home/robocon-2026/Desktop/RC2026/kvoy_quadruped`
- `KVOY_USER=robocon-2026`
- `KVOY_UID=1001`

而当前工作区实际路径是：

`/home/kang/Desktop/RC2026/kvoy_quadruped`

因此，如果你要直接使用这些 service 模板或安装脚本，通常需要先按你的实际账户和工作区路径调整相关环境变量或文件内容。

同样地，`policies.yaml` 里的模型路径当前也是绝对路径，默认指向 `/home/robocon-2026/...`。

## 13. 权限与设备访问

当前仓库中与设备权限相关的文件有：

- `config/70-kvoy-gamesir-input.rules`

它用于给 GameSir 输入设备设置合适的访问权限，便于 systemd 用户服务读取手柄。

另外，串口设备通常还需要用户具备相应权限，例如加入 `dialout` 组。

## 14. 当前工程的几个实际注意点

- launch 默认使用 YAML 文件中的参数值，而不是节点源码里声明参数时的后备默认值
- `serial_comm` 会同时订阅 `/motor_cmd` 和 `/robot_state`，并依据 FSM 状态决定 MCU 控制模式
- `robot_fsm` 自己不做策略推理，它负责在不同状态下启用 `robot_kinematics` 或 `rl_controller`
- `rl_controller` 当前依赖实时 IMU 数据；如果 IMU 数据过旧，会主动跳过推理
- 当前策略系统支持运行时切换 policy slot

## 15. 适合先看的文件

如果你要继续维护这个仓库，建议优先阅读：

- `src/robot_bringup/launch/bringup.launch.py`
- `src/robot_bringup/config/robot_params.yaml`
- `src/robot_bringup/config/policies.yaml`
- `src/robot_fsm/src/robot_fsm_node.cpp`
- `src/rl_controller/src/rl_controller_node.cpp`
- `src/serial_comm/src/serial_comm_node.cpp`
- `src/gamepad_input/src/gamepad_input_node.cpp`

这几处已经覆盖了当前工程的大部分真实运行逻辑。
