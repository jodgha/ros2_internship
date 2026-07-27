# vehicle_4axle_8steer

四桥八轮全轮转向车辆仿真包，基于 ROS2 Jazzy + RViz2。

阿克曼转向几何计算、3D 可视化、里程计、键盘/方向盘遥控驾驶。

## 架构

```
                         ┌──────────────────────┐
                         │   remote_controller  │  键盘交互 (内置物理积分)
                         │   gb29_controller    │  GB29 方向盘 (可选)
                         │   demo_steer         │  演示模式
                         │   path_tracker       │  自动巡航
                         └──────┬───────────────┘
                                │ theta1L_cmd    → steering_controller
                                │ linear_vel_cmd → odometry_publisher
                                ▼
┌─────────────────────────────────────────────────────┐
│                  steering_controller                │
│  calc_8wheel_steer_angles()  ← vehicle_4axle 库     │
│  发布: /joint_states (16关节) → robot_state_publisher → TF → RViz │
│        ~/wheel_angles (8转角) → odometry_publisher              │
└─────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────┐
│                  odometry_publisher                 │
│  阿克曼里程计: 位姿积分 → /odom + odom→base_link TF    │
└─────────────────────────────────────────────────────┘

GB29 方向盘模式（可选）:
  /dev/input/jsX → joy_node → gb29_controller → vehicle_dynamics → linear_vel_cmd
```

## 快速开始

```bash
# 构建
cd ~/fishros_ros2/pratical_projects
colcon build --packages-select vehicle_4axle_8steer
source install/setup.bash

# ── 键盘遥控（最常用） ──
# 终端1: 启动可视化
ros2 launch vehicle_4axle_8steer display.launch.py

# 终端2: 键盘遥控器
ros2 run vehicle_4axle_8steer remote_controller.py

# ── GB29 方向盘遥控 ──
ros2 launch vehicle_4axle_8steer gb29_drive.launch.py
```

> **VSCode 开发提示**：项目已配置 `.vscode/settings.json`，为 Pylance 添加了 `vehicle_4axle` 包的解析路径。如果导入语句显示红色波浪线，重新加载窗口即可消除（`Ctrl+Shift+P` → `Developer: Reload Window`）。构建前确保已 source ROS2 环境（如 `source /opt/ros/jazzy/setup.bash`）。

## 文件结构

```
vehicle_4axle_8steer/
├── vehicle_4axle/                    # Python 库（转向计算核心）
│   ├── __init__.py
│   └── steer_calc.py                 # 阿克曼几何算法 + 车辆参数
├── scripts/
│   ├── core/                         # 核心节点
│   │   ├── steering_controller.py    # 转向 + 阿克曼计算
│   │   ├── odometry_publisher.py     # 里程计
│   │   └── vehicle_dynamics.py       # 纵向动力学 (GB29用)
│   ├── keyboard/                     # 键盘遥控
│   │   └── remote_controller.py      # 挂档油门控制
│   ├── gb29/                         # GB29 方向盘
│   │   ├── gb29_controller.py        # 手柄 → 车辆指令映射
│   │   └── gb29_test.py              # 手柄轴/按钮诊断工具
│   ├── demo_steer.py                 # 正弦摆动演示
│   └── path_tracker.py               # PID 路径跟踪
├── launch/
│   ├── display.launch.py             # 键盘模式启动文件
│   └── gb29_drive.launch.py          # GB29 方向盘启动文件
├── urdf/
│   └── vehicle_4axle_8steer.urdf     # 车辆 3D 模型
├── rviz/
│   └── display.rviz                  # RViz 配置
├── env-hooks/
│   └── pythonpath.dsv.in             # PYTHONPATH 环境钩子
├── .gitignore                          # 忽略 build/install/log/ 构建产物
├── CMakeLists.txt
├── package.xml
├── LICENSE                             # Apache-2.0
└── README.md
```

## 控制模式

**挂档 + 油门控制**，内置物理积分模型。模拟真实驾驶：挂档 → 踩油门 → 加速 → 松油滑行 → 刹车减速。

### 档位与油门

| 命令 | 说明 |
| --- | --- |
| `gear=d` | 前进 **D** 档 (v ≥ 0) |
| `gear=n` | 空档 **N** (v → 0) |
| `gear=r` | 倒车 **R** 档 (v ≤ 0) |
| `w` / `s` | 油门 ±10%（状态栏 `[▐▐▐░░░░░░░]` = 30%油门） |
| `b` | 刹车切换（点按刹死🛑 / 再按松开） |
| `brk=0.5` | 设定刹车深度 0~1，`b` = 全刹 |
| `throttle=0.5` | 直接设定油门 0~1（同 `acl=0.5`） |

物理模型（50Hz 积分）：

```
驱动力 = 油门 × 3.0 m/s²
制动力 = 刹车 × 6.0 m/s²
滑行阻力 = 0.5 m/s²（松油松刹时自然减速）
D 档: v ∈ [0, 20 m/s]   R 档: v ∈ [-5 m/s, 0]   N 档: v → 0
```

### 转向

| 命令 | 说明 |
| --- | --- |
| `a` / `d` | 左转 / 右转 ±3° |
| `enter` | 回正方向盘 |
| `rad=0.26` | 设定转角 (rad) |
| `+15` / `-10` | 快捷左转 15° / 右转 10° |

### 其他

| 命令 | 说明 |
| --- | --- |
| `0` | 全复位（回正 + 松油松刹） |
| `home` | 回到原点并复位里程计 |
| `q` | 退出 |

### 演示模式

| 命令 | 说明 |
| --- | --- |
| `demo` | 正弦摆动演示 (±30°, 4s 周期) |
| `circle` | 圆周运动 (15°转角, R≈20m) |
| `crab` | 蟹行模式切换 (low_speed ↔ high_speed) |
| `track=10,0` | PID 驶向目标点 |
| `track=0,0;10,0;10,10;0,0` | 多航点循环巡航 |

## 转向模式

| 模式 | 参数值 | 后桥行为 | 效果 |
| --- | --- | --- | --- |
| 低速转向 | `low_speed` | 与前桥反向偏转 | 减小转弯半径 |
| 高速蟹行 | `high_speed` | 与前桥同向偏转 | 斜向平移 |

## GB29 方向盘

```bash
# 先确认设备 ID
ros2 run joy joy_enumerate_devices

# 启动（默认 /dev/input/js0）
ros2 launch vehicle_4axle_8steer gb29_drive.launch.py

# 指定设备
ros2 launch vehicle_4axle_8steer gb29_drive.launch.py device_id:=1
```

| 方向盘控件 | 映射 |
| --- | --- |
| 方向盘 | 转向角 ±45° |
| 油门踏板 | 油门 0~1 |
| 刹车踏板 | 刹车 0~1 |
| 右拨片 | D 档 |
| 左拨片 | R 档 |
| A 按钮 | N 档 |
| B 按钮 | 复位里程计 |

手柄断连自动回正 + 全力刹车。所有轴/按钮索引可通过 `ros2 param set` 调整。

### 诊断工具

```bash
# 终端1: 启动 joy_node
ros2 run joy joy_node --ros-args -p device_id:=0

# 终端2: 实时查看所有轴和按钮的值
ros2 run vehicle_4axle_8steer gb29_test.py
```

操作方向盘/踏板/按钮，屏幕会逐帧打印 `axis0~N` 和 `button0~N` 的原始值，用于确认映射后再启动完整驾驶模式。

## 节点详解

### steering_controller

| 订阅 | 类型 | 说明 |
| --- | --- | --- |
| `~/theta1L_cmd` | Float32 | 1 桥左轮转角指令 (rad) |
| `~/wheel_vel_cmd` | Float32 | 车轮滚动速度 (rad/s) |

| 发布 | 类型 | 说明 |
| --- | --- | --- |
| `/joint_states` | JointState | 16 关节状态 → robot_state_publisher → RViz |
| `~/wheel_angles` | Float32MultiArray | 8 轮转角数组，供里程计消费 |

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `mode` | `low_speed` | `low_speed` 或 `high_speed` |
| `wheel_velocity` | 8.0 | 默认车轮滚动速度 (rad/s) |
| `auto_spin` | true | 是否自动滚动车轮 |

发布频率：50 Hz

### odometry_publisher

| 订阅 | 类型 | 说明 |
| --- | --- | --- |
| `/steering_controller/wheel_angles` | Float32MultiArray | 8 轮转角 |
| `/remote_controller/linear_vel_cmd` | Float32 | 线速度 (m/s) |
| `~/reset` | Bool | 复位里程计 |

| 发布 | 类型 | 说明 |
| --- | --- | --- |
| `/odom` | Odometry | 里程计 |
| `odom → base_link` | TF | 坐标变换 |

### vehicle_dynamics（仅 GB29 模式使用）

| 订阅 | 类型 | 说明 |
| --- | --- | --- |
| `~/gear_cmd` | Int8 | 0=N, 1=D, -1=R |
| `~/throttle_cmd` | Float32 | 油门 0~1 |
| `~/brake_cmd` | Float32 | 刹车 0~1 |

| 发布 | 类型 | 说明 |
| --- | --- | --- |
| `/remote_controller/linear_vel_cmd` | Float32 | 积分速度 |

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `max_accel` | 3.0 | 最大加速度 m/s² |
| `max_brake_decel` | 6.0 | 最大制动减速度 m/s² |
| `drag_decel` | 0.5 | 滑行阻力 m/s² |
| `max_speed_fwd` | 20.0 | 前进极速 m/s |
| `max_speed_rev` | 5.0 | 倒车极速 m/s |

### path_tracker

独立 PID 路径跟踪节点。遥控器中用 `track=x,y` 效果相同。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `waypoints` | `10,0;10,10;0,10;0,0` | 航点串 |
| `speed` | 3.0 | 巡航线速度 m/s |
| `waypoint_threshold` | 1.0 | 到达判定距离 m |
| `loop` | true | 是否循环 |

## 话题总览

| 话题 | 类型 | 发布者 | 订阅者 |
| --- | --- | --- | --- |
| `/joint_states` | JointState | steering_controller | robot_state_publisher |
| `/steering_controller/theta1L_cmd` | Float32 | remote / demo / path / gb29 | steering_controller |
| `/steering_controller/wheel_vel_cmd` | Float32 | remote_controller | steering_controller |
| `/steering_controller/wheel_angles` | Float32MultiArray | steering_controller | odometry_publisher |
| `/remote_controller/linear_vel_cmd` | Float32 | remote / vehicle_dynamics | odometry_publisher |
| `/vehicle_dynamics/gear_cmd` | Int8 | gb29_controller | vehicle_dynamics |
| `/vehicle_dynamics/throttle_cmd` | Float32 | gb29_controller | vehicle_dynamics |
| `/vehicle_dynamics/brake_cmd` | Float32 | gb29_controller | vehicle_dynamics |
| `/odometry_publisher/reset` | Bool | remote / gb29 | odometry_publisher |
| `/odom` | Odometry | odometry_publisher | path_tracker 等 |
| `/joy` | Joy | joy_node | gb29_controller |

## 车辆几何参数

定义于 [vehicle_4axle/steer_calc.py](vehicle_4axle/steer_calc.py)，各节点统一引用。

| 参数 | 值 | 单位 | 说明 |
| --- | --- | --- | --- |
| 车长 | 14.980 | m | 底盘 box 总长 |
| 车宽 | 2.460 | m | 底盘 box 总宽 |
| 桥 1 x | +3.100 | m | 前转向桥 |
| 桥 2 x | +4.750 | m | 前转向桥 |
| 桥 3 x | −3.100 | m | 后转向桥 |
| 桥 4 x | −4.750 | m | 后转向桥 |
| 轮距 | 2.260 | m | 每桥左右主销间距 |
| 车轮半径 | 0.700 | m | 轮胎圆柱半径 |
| 转向限位 | ±0.785 | rad | ±45° |
| 总质量 | 12000 | kg | 整备质量 |

## 启动参数

```bash
# 低速模式（默认）
ros2 launch vehicle_4axle_8steer display.launch.py

# 蟹行模式
ros2 launch vehicle_4axle_8steer display.launch.py mode:=high_speed

# 圆周运动
ros2 launch vehicle_4axle_8steer display.launch.py linear_velocity:=3.0
ros2 topic pub /steering_controller/theta1L_cmd std_msgs/msg/Float32 "data: 0.1628"
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `mode` | `low_speed` | 后桥反向转向；`high_speed` 为蟹行 |
| `wheel_velocity` | 8.0 | RViz 车轮显示转速 (rad/s) |
| `auto_spin` | true | 是否自动旋转车轮 |
| `linear_velocity` | 0.0 | 里程计默认线速度 (m/s) |

### 独立运行节点

```bash
# 键盘遥控器（内置挂档模式）
ros2 run vehicle_4axle_8steer remote_controller.py

# PID 路径跟踪自动巡航
ros2 run vehicle_4axle_8steer path_tracker.py

# 正弦摆动演示
ros2 run vehicle_4axle_8steer demo_steer.py --ros-args -p period:=4.0 -p amplitude_deg:=30.0

# 直接发转向指令
ros2 topic pub /steering_controller/theta1L_cmd std_msgs/msg/Float32 "data: 0.26"

# 运行时切换蟹行模式
ros2 param set /steering_controller mode high_speed
```

## 依赖

- rclpy
- sensor_msgs / std_msgs / geometry_msgs / nav_msgs
- tf2_ros
- robot_state_publisher / rviz2
- joy（GB29 方向盘模式需要）

全部在 package.xml 中显式声明。
