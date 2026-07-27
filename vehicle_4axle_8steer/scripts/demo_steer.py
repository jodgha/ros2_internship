#!/usr/bin/env python3
"""发送连续变化的转向角，演示轮子转向效果。"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class DemoSteerNode(Node):
    """正弦摆动演示节点 —— 发布周期性转向角指令驱动 steering_controller。"""

    def __init__(self):
        super().__init__("demo_steer_publisher")

        self.declare_parameter("period", 4.0)
        self.declare_parameter("amplitude_deg", 30.0)
        self.declare_parameter("frequency", 20.0)  # Hz

        period = self.get_parameter("period").value
        amplitude_deg = self.get_parameter("amplitude_deg").value
        frequency = self.get_parameter("frequency").value

        self._amplitude = math.radians(amplitude_deg)
        self._omega = 2 * math.pi / period  # 角频率 rad/s
        self._t = 0.0
        self._dt = 1.0 / frequency

        self._pub = self.create_publisher(
            Float32, "/steering_controller/theta1L_cmd", 10
        )
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"开始摆动演示: ±{amplitude_deg:.0f}°, {period:.0f}s周期, {frequency:.0f}Hz, Ctrl+C 停止"
        )

    def _tick(self):
        theta = self._amplitude * math.sin(self._omega * self._t)
        self._pub.publish(Float32(data=theta))
        self.get_logger().info(
            f"θ1L={math.degrees(theta):6.1f}°", throttle_duration_sec=0.5
        )
        self._t += self._dt

    def destroy_node(self):
        """退出前回正方向盘。"""
        self._pub.publish(Float32(data=0.0))
        self.get_logger().info("已回正，退出")
        super().destroy_node()


def main():
    rclpy.init()
    node = DemoSteerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
