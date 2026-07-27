#!/usr/bin/env python3
"""
路径跟踪节点：订阅里程计 → 控制转向 → 沿航点自动巡航。

【数据流】
  waypoints (参数输入)
       │
       ▼
  计算目标方向 ─→ [控制器] ─→ theta1L_cmd → steering_controller → 车辆转向
       ▲                        │
       │                        ▼
       └──── /odom (位姿) ← odometry_publisher

【订阅】
  /odom  (Odometry) — 车辆当前位姿

【发布】
  /steering_controller/theta1L_cmd  (Float32) — 转向指令
  /steering_controller/wheel_vel_cmd (Float32) — 车轮转速
  /remote_controller/linear_vel_cmd  (Float32) — 线速度

【参数】
  waypoints          航点列表 "x,y;x,y;..."        (默认 "10,0;10,10;0,10;0,0")
  speed              巡航线速度 m/s                (默认 3.0)
  waypoint_threshold 到达航点判定距离 m             (默认 1.0)
  loop               是否循环航点                  (默认 true)
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from vehicle_4axle.steer_calc import WHEEL_RADIUS


class PathTracker(Node):
    """路径跟踪节点 —— 沿航点自动巡航，到点即停。"""

    def __init__(self):
        super().__init__("path_tracker")

        # ── 参数 ──
        self.declare_parameter("waypoints", "10,0;10,10;0,10;0,0")
        self.declare_parameter("speed", 3.0)
        self.declare_parameter("waypoint_threshold", 1.0)
        self.declare_parameter("loop", True)

        self._parse_waypoints()
        self._speed = self.get_parameter("speed").value
        self._threshold = self.get_parameter("waypoint_threshold").value
        self._loop = self.get_parameter("loop").value

        # 内部状态
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._odom_ready = False
        self._wp_idx = 0
        self._arrived = False
        self._last_time = self.get_clock().now()

        # ── 订阅与发布 ──
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._steer_pub = self.create_publisher(
            Float32, "/steering_controller/theta1L_cmd", 10
        )
        self._wheel_pub = self.create_publisher(
            Float32, "/steering_controller/wheel_vel_cmd", 10
        )
        self._speed_pub = self.create_publisher(
            Float32, "/remote_controller/linear_vel_cmd", 10
        )

        # 控制循环：20Hz
        self.create_timer(0.05, self._control_tick)

        self.get_logger().info(
            f"PathTracker 启动 | {len(self._waypoints)} 航点 "
            f"| 速度 {self._speed:.1f}m/s | 阈值 {self._threshold:.1f}m "
            f"| {'循环' if self._loop else '到点即停'}"
        )

    # ── 航点解析 ──
    def _parse_waypoints(self):
        raw = self.get_parameter("waypoints").value
        self._waypoints = []
        for pair in raw.split(";"):
            pair = pair.strip()
            if not pair:
                continue
            parts = pair.split(",")
            if len(parts) != 2:
                continue
            self._waypoints.append((float(parts[0]), float(parts[1])))
        if not self._waypoints:
            self.get_logger().warning("无有效航点，使用默认矩形")
            self._waypoints = [(10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)]

    # ── 回调 ──
    def _on_odom(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)
        self._odom_ready = True

    # ── 控制循环 ──
    def _control_tick(self):
        if not self._odom_ready or self._arrived:
            return

        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt > 1.0:
            return

        wx, wy = self._waypoints[self._wp_idx]
        dx = wx - self._x
        dy = wy - self._y
        dist = math.hypot(dx, dy)

        # ── 到达 ← 停车 ──
        if dist < self._threshold:
            if self._loop and len(self._waypoints) > 1:
                self._wp_idx = (self._wp_idx + 1) % len(self._waypoints)
                self.get_logger().info(f"→ WP{self._wp_idx}")
            else:
                self._arrived = True
                self._publish(0.0, 0.0)
                self._publish(0.0, 0.0)  # 确保停车指令发出
                self.get_logger().info(f"到达 ({wx:.0f},{wy:.0f}) ✓ 已停车")
            return

        # ── 近距: 直走不拐 ──
        if dist < 3.0:
            steer = 0.0
            speed = 0.5
        else:
            # ── 远距: P 控制追方位 ──
            target_heading = math.atan2(dy, dx)
            heading_err = target_heading - self._yaw
            heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))
            Kp = 0.6
            steer = max(-math.pi / 4, min(math.pi / 4, Kp * heading_err))
            speed = min(self._speed, 0.5 * dist)

        self._publish(steer, speed)
        self.get_logger().info(
            f"→({wx:.0f},{wy:.0f}) "
            f"| 偏差{math.degrees(math.atan2(dy,dx)-self._yaw):+5.1f}° "
            f"| 打轮{math.degrees(steer):+5.1f}° "
            f"| 距{dist:5.1f}m "
            f"| 速{speed:4.2f}m/s",
            throttle_duration_sec=0.5,
        )

    def _publish(self, steer: float, speed: float):
        self._steer_pub.publish(Float32(data=steer))
        self._wheel_pub.publish(Float32(data=speed / WHEEL_RADIUS))
        self._speed_pub.publish(Float32(data=speed))


def main():
    rclpy.init()
    node = PathTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
