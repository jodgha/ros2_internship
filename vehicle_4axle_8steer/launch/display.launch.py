"""
display.launch.py  启动 4 轴 8 轮车辆可视化 + 里程计

启动的节点:
  1. robot_state_publisher    解析 URDF → 发布 /tf_static + /tf
  2. steering_controller      订阅转向指令 → 发布 /joint_states 驱动 RViz
  3. odometry_publisher       订阅转角 + 线速度 → 发布 odom → base_link 位姿积分
  4. rviz2                    3D 可视化 固定参考系:odom

使用方式:
  # 默认低速模式
    ros2 launch vehicle_4axle_8steer display.launch.py

  # 圆周运动 R=20m v=3m/s
    ros2 launch vehicle_4axle_8steer display.launch.py linear_velocity:=3.0
  # 另起终端发转向指令
    ros2 topic pub /steering_controller/theta1L_cmd std_msgs/msg/Float32 "data: 0.1628"

  # 高速蟹行模式
    ros2 launch vehicle_4axle_8steer display.launch.py mode:=high_speed

  # 已封装的演示脚本
    ros2 run vehicle_4axle_8steer demo_steer.py          # 轮子正弦摆动
    ros2 run vehicle_4axle_8steer odometry_publisher.py  # 单独启动里程计
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 参数声明
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="low_speed",
        description="转向模式: low_speed (后桥反向) | high_speed (后桥同向蟹行)",
    )
    wheel_vel_arg = DeclareLaunchArgument(
        "wheel_velocity",
        default_value="8.0",
        description="车轮滚动显示速度 rad/s, 仅影响 RViz 中轮子转动快慢",
    )
    auto_spin_arg = DeclareLaunchArgument(
        "auto_spin",
        default_value="true",
        description="是否自动让车轮旋转",
    )
    linear_vel_arg = DeclareLaunchArgument(
        "linear_velocity",
        default_value="0.0",
        description="车辆前进线速度 m/s, 用于里程计位姿积分, 默认静止",
    )

    # URDF 路径
    urdf_path = PathJoinSubstitution([
        FindPackageShare("vehicle_4axle_8steer"),
        "urdf",
        "vehicle_4axle_8steer.urdf",
    ])

    # robot_state_publisher
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

    # steering_controller
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

    # 里程计
    odometry = Node(
        package="vehicle_4axle_8steer",
        executable="odometry_publisher.py",
        name="odometry_publisher",
        parameters=[{
            "linear_velocity": LaunchConfiguration("linear_velocity"),
        }],
        output="screen",
    )

    # rviz2 固定参考系: odom, 可看到车辆移动
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

    # 启动提示
    usage_hint = LogInfo(
        msg=[
            "\n============================================\n",
            "  车辆已启动！RViz 固定参考系: odom\n",
            "\n",
            "  [20m 圆周运动] 另开终端执行:\n",
            "    ros2 topic pub /steering_controller/theta1L_cmd \\\n",
            "      std_msgs/msg/Float32 'data: 0.1628'\n",
            "\n",
            "  [手动控制] 发送转向指令:\n",
            "    ros2 topic pub /steering_controller/theta1L_cmd \\\n",
            "      std_msgs/msg/Float32 'data: 0.26'\n",
            "\n",
            "  [调整速度]:\n",
            "    ros2 param set /odometry_publisher linear_velocity 5.0\n",
            "============================================",
        ]
    )

    return LaunchDescription([
        mode_arg,
        wheel_vel_arg,
        auto_spin_arg,
        linear_vel_arg,
        robot_state_pub,
        steering_ctrl,
        odometry,
        rviz,
        TimerAction(period=2.0, actions=[usage_hint]),
    ])
