"""
gb29_drive.launch.py  GB29 方向盘驾驶模式 — 一键启动完整仿真。

启动的节点:
  1. joy_node                  读取 GB29 手柄/方向盘 → /joy
  2. gb29_controller           映射 /joy → 转向/油门/刹车/档位
  3. vehicle_dynamics          挂档+加速踏板 → 速度积分 → linear_vel_cmd
  4. robot_state_publisher     解析 URDF → /tf_static + /tf
  5. steering_controller       订阅转向指令 → /joint_states 驱动 RViz
  6. odometry_publisher        订阅速度+转角 → /odom 位姿积分
  7. rviz2                     3D 可视化 (固定参考系 odom)

使用方式:
  # 默认设备 /dev/input/js0
  ros2 launch vehicle_4axle_8steer gb29_drive.launch.py

  # 指定设备
  ros2 launch vehicle_4axle_8steer gb29_drive.launch.py device_id:=1

  # 低速模式 + 自定义极速
  ros2 launch vehicle_4axle_8steer gb29_drive.launch.py mode:=low_speed max_speed_fwd:=15.0

  # 用 jstest / ros2 run joy joy_enumerate_devices 先确定设备 ID
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── 参数声明 ──
    device_arg = DeclareLaunchArgument(
        "device_id",
        default_value="0",
        description="方向盘设备 ID (ros2 run joy joy_enumerate_devices 查看)",
    )
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="low_speed",
        description="转向模式: low_speed (后桥反向) | high_speed (蟹行)",
    )
    wheel_vel_arg = DeclareLaunchArgument(
        "wheel_velocity",
        default_value="8.0",
        description="车轮滚动显示速度 rad/s",
    )
    auto_spin_arg = DeclareLaunchArgument(
        "auto_spin",
        default_value="true",
        description="是否自动旋转车轮",
    )
    linear_vel_arg = DeclareLaunchArgument(
        "linear_velocity",
        default_value="0.0",
        description="初始线速度 m/s (让 odometry_publisher 从 0 开始)",
    )
    max_accel_arg = DeclareLaunchArgument(
        "max_accel",
        default_value="3.0",
        description="最大驱动力加速度 m/s²",
    )
    max_brake_arg = DeclareLaunchArgument(
        "max_brake_decel",
        default_value="6.0",
        description="最大制动减速度 m/s²",
    )
    drag_arg = DeclareLaunchArgument(
        "drag_decel",
        default_value="0.5",
        description="滑行阻力减速度 m/s²",
    )
    max_fwd_arg = DeclareLaunchArgument(
        "max_speed_fwd",
        default_value="20.0",
        description="前进极速 m/s",
    )
    max_rev_arg = DeclareLaunchArgument(
        "max_speed_rev",
        default_value="5.0",
        description="倒车极速 m/s",
    )
    max_steer_arg = DeclareLaunchArgument(
        "max_steer_angle",
        default_value="0.785",
        description="最大转向角 rad (±45°)",
    )
    device_name_arg = DeclareLaunchArgument(
        "device_name",
        default_value="",
        description="joystick 设备名 (如 'Logitech G29'), 留空则用 device_id",
    )

    # ── URDF 路径 ──
    urdf_path = PathJoinSubstitution([
        FindPackageShare("vehicle_4axle_8steer"),
        "urdf",
        "vehicle_4axle_8steer.urdf",
    ])

    # ── 1. joy_node — 读取物理方向盘 ──
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{
            "device_id": LaunchConfiguration("device_id"),
            "device_name": LaunchConfiguration("device_name"),
            "deadzone": 0.02,
            "autorepeat_rate": 20.0,
            "coalesce_interval_ms": 1,
        }],
    )

    # ── 2. GB29 控制器 — 映射手柄 → 车辆指令 ──
    gb29_ctrl = Node(
        package="vehicle_4axle_8steer",
        executable="gb29_controller.py",
        name="gb29_controller",
        parameters=[{
            "max_steer_angle": LaunchConfiguration("max_steer_angle"),
        }],
        output="screen",
    )

    # ── 3. 车辆动力学 — 油门/刹车 → 速度积分 ──
    vehicle_dynamics = Node(
        package="vehicle_4axle_8steer",
        executable="vehicle_dynamics.py",
        name="vehicle_dynamics",
        parameters=[{
            "max_accel": LaunchConfiguration("max_accel"),
            "max_brake_decel": LaunchConfiguration("max_brake_decel"),
            "drag_decel": LaunchConfiguration("drag_decel"),
            "max_speed_fwd": LaunchConfiguration("max_speed_fwd"),
            "max_speed_rev": LaunchConfiguration("max_speed_rev"),
        }],
        output="screen",
    )

    # ── 4. robot_state_publisher ──
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["cat ", urdf_path]),
                value_type=str,
            ),
        }],
    )

    # ── 5. steering_controller ──
    steering_ctrl = Node(
        package="vehicle_4axle_8steer",
        executable="steering_controller.py",
        name="steering_controller",
        parameters=[{
            "mode": LaunchConfiguration("mode"),
            "wheel_velocity": LaunchConfiguration("wheel_velocity"),
            "auto_spin": LaunchConfiguration("auto_spin"),
        }],
        output="screen",
    )

    # ── 6. 里程计 ──
    odometry = Node(
        package="vehicle_4axle_8steer",
        executable="odometry_publisher.py",
        name="odometry_publisher",
        parameters=[{
            "linear_velocity": LaunchConfiguration("linear_velocity"),
        }],
        output="screen",
    )

    # ── 7. RViz ──
    rviz_config = PathJoinSubstitution([
        FindPackageShare("vehicle_4axle_8steer"),
        "rviz",
        "display.rviz",
    ])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config, "-f", "odom"],
    )

    # ── 启动提示 ──
    usage_hint = LogInfo(
        msg=[
            "\n╔══════════════════════════════════════════════════════╗\n",
            "║   🎮 GB29 方向盘驾驶模式                              ║\n",
            "╠══════════════════════════════════════════════════════╣\n",
            "║   操作说明:                                            ║\n",
            "║   方向盘  → 转向                                       ║\n",
            "║   油门踏板 → 加速                                       ║\n",
            "║   刹车踏板 → 减速                                       ║\n",
            "║   右拨片   → D 档 (前进)                                ║\n",
            "║   左拨片   → R 档 (倒车)                                ║\n",
            "║   A 按钮   → N 档 (空档)                                ║\n",
            "║   B 按钮   → 复位里程计                                 ║\n",
            "╠══════════════════════════════════════════════════════╣\n",
            "║   命令行辅助控制:                                        ║\n",
            "║   查看状态: ros2 topic echo /odom                       ║\n",
            "║   调速性: ros2 param set /vehicle_dynamics max_accel 5.0║\n",
            "║   切换蟹行: ros2 param set /steering_controller mode high_speed ║\n",
            "║   查看 Joy: ros2 topic echo /joy                        ║\n",
            "╚══════════════════════════════════════════════════════╝",
        ]
    )

    return LaunchDescription([
        device_arg,
        mode_arg,
        wheel_vel_arg,
        auto_spin_arg,
        linear_vel_arg,
        max_accel_arg,
        max_brake_arg,
        drag_arg,
        max_fwd_arg,
        max_rev_arg,
        max_steer_arg,
        device_name_arg,
        joy_node,
        gb29_ctrl,
        vehicle_dynamics,
        robot_state_pub,
        steering_ctrl,
        odometry,
        rviz,
        TimerAction(period=3.0, actions=[usage_hint]),
    ])
