# JetRover — UGV 上位机 ROS 2 工作区

Waveshare UGV 系列（RaspRover / UGV Rover / UGV Beast）+ Jetson Orin Nano Super 上位机的 ROS 2 工作区。下位机为 ESP32 固件 [`ugv_base_ros`](https://github.com/effectsmachine/ugv_base_ros)（本仓库的姊妹目录 `base_driver/`），两者通过 **UART (115200) 上的 JSON 行协议** 通信。

## 系统架构

```
手柄 joy_node                       LD06 激光雷达
   │ Joy                               │ scan
   ▼                                   ▼
joyparser_node ──cmd_vel──► serial_node            ugv_slam (cartographer)
   │ ugv/led_strl            │  ▲ JSON/UART           ▲ imu/data, odometry/filtered
   │ ugv/servos              │  └── ESP32 (ugv_base_ros)
   ▼                         ▼
        imu/data_raw, imu/mag, encoder, voltage, joint_states
                             │
                             ▼
                       encoder_node ──► odom/odom_raw
```

## 包结构

| 包 | 类型 | 说明 |
|---|---|---|
| `ugv_bringup_py` | ament_python | 核心：串口桥接、IMU/编码器/电压/云台发布、手柄解析、相机、协方差标定。节点：`serial_node`、`joyparser_node`、`sensordebug_node`、`camera_node` |
| `ugv_bringup_cpp` | ament_cmake | 上者的 C++ 移植（nlohmann/json 实现） |
| `ugv_localization_py` | ament_python | `encoder_node`：轮式里程计积分 |
| `ugv_slam` | ament_python | Cartographer 2D SLAM 封装（launch + lua，无节点） |
| `ugv_description` | ament_python | URDF 模型（目前仅 `ugv_rover.urdf`） |

## 环境依赖

- Jetson（Ubuntu 22.04）+ ROS 2（建议 Humble），安装笔记见 `Note.md`
- `ros-<distro>-cartographer-ros`
- `ros-<distro>-cv-bridge`、`python3-opencv`（camera_node）
- `ros-<distro>-joy`（手柄）
- Python: `pyserial`、`numpy`
- **LD06 雷达驱动 `ldlidar_stl_ros2` 不在本仓库中**（`src/ldlidar_stl_ros2` 为空占位目录，但 launch 有引用），需单独克隆到 `src/` 下：

  ```bash
  git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git src/ldlidar_stl_ros2
  ```

## 构建与测试

在 Jetson 上：

```bash
source /opt/ros/<distro>/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# 测试（ament lint）
colcon test --packages-select ugv_bringup_py
colcon test-result --verbose
```

## 使用

```bash
# 1. 底盘 + 手柄 + 雷达 + 相机（需先插好手柄，串口默认 /dev/ttyTHS1）
ros2 launch ugv_bringup_py joy_teleop.launch.py

# 2. 手柄节点（joy_teleop.launch.py 不包含它，需另开终端）
ros2 run joy joy_node

# 3. SLAM
ros2 launch ugv_slam ugv_cartographer.launch.py
```

手柄映射（`joyparser_node`）：左摇杆控制云台（`ugv/servos`），右摇杆 `cmd_vel` 线速度/角速度，按键 8 复位云台偏置；电压低于 `vol_threshold`（默认 10.5V）会告警。

`serial_node` 首次使用可通过 `do_servo_calib:=true`（默认 true；`joy_teleop.launch.py` 中已关闭）做云台舵机中位校准，流程：上电后手动摆正云台 → 10s 后自动以当前位置为零位。

## 串口协议摘要

与固件 `base_driver/ROS_Driver/json_cmd.h` 的命令表对应，一行一个 JSON 对象（`\n` 结尾）。上位机常用命令：

| T | 含义 | 关键字段 |
|---|---|---|
| 1 | 左右轮目标速度 (m/s) | `L`, `R` |
| 13 | ROS 差速控制（`cmd_vel` 主路径） | `X` (m/s), `Z` (rad/s) |
| 0 | 紧急停止 | — |
| 131 | 底盘反馈流开关 | `cmd` |
| 142 | 反馈流间隔 (ms) | `cmd` |
| 132 | LED/探照灯 PWM | `IO4`, `IO5` |
| 134 | 云台绝对角度移动 | `X`(yaw°), `Y`(pitch°), `SX`, `SY` |
| 210 | 舵机扭矩开关 | `cmd` |
| 502 | 设置舵机中位 | `id` |

反馈流（`T=1001`）字段：`L/R`（轮速）、`ax/ay/az`（原始加速度）、`gx/gy/gz`（原始角速度）、`mx/my/mz`（磁力计）、`odl/odr`（累计里程，cm）、`v`（电压，1/100V）、`pan/tilt`（云台角度）。

> 修改协议时必须同步改固件的 `json_cmd.h`/`uart_ctrl.h` 和本工作区的 `serial_node`（py 与 cpp 两份）。

## 常用参数

`serial_node`（以 `param:=value` 形式传入 launch 或单独运行）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `port` / `baudrate` | `/dev/ttyTHS1` / 115200 | 与 ESP32 的串口 |
| `sample_period_ms` | 50 | 请求固件的反馈间隔 |
| `do_servo_calib` | true | 启动时做云台零位校准 |
| `pub_name_imu` 等 | — | 各话题重命名 |

`encoder_node`：`track_width`（轮距，注意与固件 `mm_settings` 保持一致）、`odom_frame_id`、`base_frame_id`。

## 已知事项 / Roadmap

- [ ] `joy_teleop.launch.py` 未启动 joy 节点与 `robot_state_publisher`；Cartographer 需要的 `base_imu_link → base_footprint` TF 链尚无来源（URDF 也仅覆盖 Rover）
- [ ] Cartographer 的 odom 重映射到 `odometry/filtered`，但工作区内无 EKF 节点发布它；接入 `robot_localization` 融合 `odom/odom_raw` + `imu/data_raw` 后 SLAM 才完整
- [ ] `ugv_bringup_py` 与 `ugv_bringup_cpp` 双实现需人工保持同步
- [ ] 目前 `test/` 仅有 ament lint，无单元测试

## 参考

- Waveshare WIKI：UGV Rover Jetson Orin ROS2
- 下位机仓库：`effectsmachine/ugv_base_ros`（本地 `../base_driver/`）
