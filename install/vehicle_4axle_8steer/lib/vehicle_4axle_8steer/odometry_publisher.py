#!/usr/bin/env python3
"""
里程计节点：基于阿克曼转向几何 + 车轮速度 → 发布 odom → base_link 位姿。

【核心功能】
  1. 订阅 /steering_controller/wheel_angles → 获取 8 轮转角
  2. 订阅 /remote_controller/linear_vel_cmd → 获取实时线速度
  3. 从 θ1L 反算瞬时转弯半径 R
  4. 由线速度 v 和 R 计算横摆角速度 ω=v/R
  5. 积分得到车辆在 odom 坐标系中的位姿 (x,y,yaw)
  6. 发布 odom → base_link 的 TF 变换

【话题】
  Sub: /steering_controller/wheel_angles  (Float32MultiArray)  8轮转角
  Sub: /remote_controller/linear_vel_cmd  (Float32)            线速度指令
  Pub: /odom                               (Odometry)          里程计
  TF:  odom → base_link

【参数】
  linear_velocity     默认线速度 (m/s), 默认 0.0 (静止)
  publish_rate        里程计发布频率 (Hz), 默认 50
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool,Float32,Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped,Quaternion
from tf2_ros import TransformBroadcaster

from vehicle_4axle.steer_calc import X_LIST, W_LIST, EPS

# 1 桥参数用于阿克曼转弯半径计算
_X1 = X_LIST[0]
_W1 = W_LIST[0]
_MAX_TURN_RADIUS = 1e6  # 超过此值视为直线行驶 (R≈∞)


def quaternion_from_yaw(yaw:float)->Quaternion:
    q=Quaternion()
    q.z=math.sin(yaw/2.0)
    q.w=math.cos(yaw/2.0)
    return q


class OdometryPublisher(Node):
    """阿克曼里程计发布节点。"""

    def __init__(self):
        super().__init__("odometry_publisher")

        self.declare_parameter("linear_velocity",0.0)
        self.declare_parameter("publish_rate",50.0)

        self.x=0.0
        self.y=0.0
        self.yaw=0.0
        self.theta1L=0.0
        self.turn_radius=float("inf")
        self.linear_vel=self.get_parameter("linear_velocity").value
        self.last_time=self.get_clock().now()

        # 订阅：转角
        self.create_subscription(
            Float32MultiArray,
            "/steering_controller/wheel_angles",
            self._on_wheel_angles,10,
        )
        # 订阅：遥控器速度指令
        self.create_subscription(
            Float32,
            "/remote_controller/linear_vel_cmd",
            self._on_vel_cmd,10,
        )

        # 订阅：里程计复位
        self.create_subscription(
            Bool,"~/reset",self._on_reset,10,
        )

        self.odom_pub=self.create_publisher(Odometry,"/odom",10)
        self.tf_broadcaster=TransformBroadcaster(self)

        rate=self.get_parameter("publish_rate").value
        self.timer=self.create_timer(1.0/rate,self._update)

        self.get_logger().info(
            f"OdometryPublisher 已启动 | 线速度: {self.linear_vel:.0f} m/s | 频率: {rate:.0f} Hz"
        )

    def _on_wheel_angles(self,msg:Float32MultiArray):
        if len(msg.data)>=1:
            self.theta1L=msg.data[0]
            if abs(self.theta1L)>EPS:
                self.turn_radius=_X1/math.tan(self.theta1L)+_W1/2.0
            else:
                self.turn_radius=float("inf")

    def _on_vel_cmd(self,msg:Float32):
        """遥控器实时速度指令。"""
        self.linear_vel=msg.data

    def _on_reset(self,msg:Bool):
        """重置里程计位姿到原点。"""
        if msg.data:
            self.x=0.0
            self.y=0.0
            self.yaw=0.0
            self.get_logger().info("里程计已复位 → (0, 0, 0)")

    def _update(self):
        now=self.get_clock().now()
        dt=(now-self.last_time).nanoseconds/1e9
        self.last_time=now

        if dt<=0.0 or dt>1.0:
            return

        v=self.linear_vel

        if abs(self.turn_radius)>EPS and abs(self.turn_radius)<_MAX_TURN_RADIUS:
            omega=v/self.turn_radius
        else:
            omega=0.0

        self.yaw+=omega*dt
        self.x+=v*math.cos(self.yaw)*dt
        self.y+=v*math.sin(self.yaw)*dt

        tf_msg=TransformStamped()
        tf_msg.header.stamp=now.to_msg()
        tf_msg.header.frame_id="odom"
        tf_msg.child_frame_id="base_link"
        tf_msg.transform.translation.x=self.x
        tf_msg.transform.translation.y=self.y
        tf_msg.transform.rotation=quaternion_from_yaw(self.yaw)
        self.tf_broadcaster.sendTransform(tf_msg)

        odom=Odometry()
        odom.header.stamp=now.to_msg()
        odom.header.frame_id="odom"
        odom.child_frame_id="base_link"
        odom.pose.pose.position.x=self.x
        odom.pose.pose.position.y=self.y
        odom.pose.pose.orientation=quaternion_from_yaw(self.yaw)
        odom.twist.twist.linear.x=v
        odom.twist.twist.angular.z=omega
        self.odom_pub.publish(odom)

        self.get_logger().info(
            f"x={self.x:6.1f} y={self.y:6.1f} yaw={math.degrees(self.yaw):5.1f}°"
            f" | θ1L={math.degrees(self.theta1L):5.1f}° R={'   ∞  ' if math.isinf(self.turn_radius) else f'{self.turn_radius:6.1f}m'}"
            f" | v={v:.1f} ω={math.degrees(omega):5.2f}°/s",
            throttle_duration_sec=2.0,
        )


def main():
    rclpy.init()
    node=OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
