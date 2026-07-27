#!/usr/bin/env python3
"""
转向控制器节点：订阅转向指令 → 计算 8 轮阿克曼转角 → 发布 JointState 驱动 RViz

【数据流】
  /steering_controller/theta1L_cmd  (Float32, 输入)
       │
       ▼
  calc_8wheel_steer_angles()        (vehicle_4axle.steer_calc)
       │
       ▼
  /joint_states                     (JointState, 16关节 → robot_state_publisher → RViz)
  /steering_controller/wheel_angles (Float32MultiArray, 8转角, 调试用)

【订阅话题】
  ~/theta1L_cmd     - 1桥左轮转向角指令 (rad), ±45° = ±0.785 rad
  ~/wheel_vel_cmd   - 车轮滚动速度指令 (rad/s), 可选

【发布话题】
  /joint_states     - 全部 16 个关节: 8 转向角 + 8 车轮滚动角
  ~/wheel_angles    - 8 个转向角数组 [ax1L, ax1R, ax2L, ax2R, ax3L, ax3R, ax4L, ax4R]

【参数】可通过 `ros2 param set` 或 launch 文件动态调整:
  mode              - "low_speed"(后桥反向,减小转弯半径) | "high_speed"(后桥同向,蟹行)
  wheel_velocity    - 默认车轮滚动速度 (rad/s)
  auto_spin         - 是否自动让车轮持续滚动 (bool)
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import JointState

from vehicle_4axle.steer_calc import(
    calc_8wheel_steer_angles,
    STEER_NAMES,
    JOINT_NAMES,
)

# JOINT_NAMES 来自 steer_calc，与 URDF joint name 严格一致
# 前 8 个为转向关节，后 8 个为车轮关节

class SteeringController(Node):
    """四桥八轮转向控制器 —— 连接转向计算与 RViz 可视化的桥梁"""
    _PUBLISH_PERIOD = 0.02  # 50Hz 关节状态发布周期
    def __init__(self):
        super().__init__("steering_controller")

        # ROS2参数
        self.declare_parameter("mode", "low_speed")
        self.declare_parameter("wheel_velocity", 8.0)
        self.declare_parameter("auto_spin", True)

        # 内部状态
        self.steer_angles:list[float]=[0.0]*8   # 当前 8 转向角 [rad]
        self.wheel_positions:list[float]=[0.0]*8   # 累积车轮滚动位置 [rad]
        self.wheel_velocity=self.get_parameter("wheel_velocity").value
        self._last_time=self.get_clock().now()      # 用于计算实际 dt

        # 订阅:转向指令
        self.create_subscription(
            Float32,"~/theta1L_cmd",self._on_steer_cmd,10
        )
        self.create_subscription(
            Float32,"~/wheel_vel_cmd",self._on_wheel_vel_cmd,10
        )

        # 发布:关节状态 + 调试用转角数组
        self.joint_state_pub=self.create_publisher(
            JointState,"/joint_states",10
        )
        self.wheel_angles_pub=self.create_publisher(
            Float32MultiArray,"~/wheel_angles",10
        )

        # 定时器:50Hz 发布关节状态
        self.timer=self.create_timer(self._PUBLISH_PERIOD,self._publish_joint_states)
        self.get_logger().info(
            f"SteeringController 已启动 | "
            f"模式: {self.get_parameter('mode').value} | "
            f"默认轮速: {self.wheel_velocity:.1f} rad/s | "
            f"等待转向指令 {self.get_name()}/theta1L_cmd ..."
        )

    # 回调
    def _on_steer_cmd(self,msg:Float32):
        """接收 1 桥左轮转角指令，调用阿克曼模型计算全部 8 轮转角"""
        mode=self.get_parameter("mode").value
        theta1L=msg.data

        # 限幅 ±45° [物理限位]，因此 calc_8wheel_steer_angles 不会抛出 ValueError
        theta1L=max(-math.pi/4,min(math.pi/4,theta1L))
        self.steer_angles=calc_8wheel_steer_angles(theta1L,mode)

        self.get_logger().info(
            f"转向指令 θ1L={math.degrees(theta1L):.1f}° → "
            + " | ".join(
                f"{n}: {math.degrees(a):.1f}°"
                for n,a in zip(STEER_NAMES,self.steer_angles)
            ),
            throttle_duration_sec=0.5,  # 快速按键时最多2条/秒
        )

    def _on_wheel_vel_cmd(self,msg:Float32):
        """接收车轮滚动速度指令"""
        self.wheel_velocity=msg.data
        self.get_logger().debug(f"车轮速度更新: {msg.data:.2f} rad/s")

    # 定时发布
    def _publish_joint_states(self):
        """50Hz 定时器：累积车轮滚动角 → 发布 JointState。"""
        now=self.get_clock().now()

        # 基于实际 dt 累加，避免定时器抖动导致车轮旋转不均匀
        dt=(now-self._last_time).nanoseconds/1e9
        self._last_time=now
        if dt>1.0:  # 启动后首帧保护，回退到标称周期
            dt=self._PUBLISH_PERIOD

        if self.get_parameter("auto_spin").value:
            for i in range(8):
                self.wheel_positions[i]+=self.wheel_velocity*dt

        # 组装 JointState 消息
        msg=JointState()
        msg.header.stamp=now.to_msg()
        msg.name=JOINT_NAMES
        msg.position=list(self.steer_angles)+list(self.wheel_positions)
        # velocity 字段供 robot_state_publisher 平滑插值
        msg.velocity=[0.0]*8+[self.wheel_velocity]*8
        self.joint_state_pub.publish(msg)

        # 同时发布转角数组 供其他节点消费
        angle_msg=Float32MultiArray()
        angle_msg.data=self.steer_angles
        self.wheel_angles_pub.publish(angle_msg)


# 入口
def main():
    rclpy.init()
    node=SteeringController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
