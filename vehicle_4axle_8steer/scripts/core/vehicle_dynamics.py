#!/usr/bin/env python3
"""
车辆纵向动力学节点：挂档 + 油门/刹车 → 加速度积分 → 速度指令。

【数据流】
  gear_cmd (Int8)       ─┐
  throttle_cmd (Float32) ─┤  → 物理积分 → linear_vel_cmd (Float32) → odometry_publisher
  brake_cmd (Float32)    ─┘

【档位 Gear】
  1 = D (前进)  → 速度限制 [0, max_speed_fwd]
  0 = N (空档)  → 速度自然衰减到 0
  -1 = R (倒车) → 速度限制 [-max_speed_rev, 0]

【物理模型 (50Hz)】
  驱动力:  throttle × max_accel
  制动力:  brake   × max_brake_decel (刹车)
  滑行阻力: drag_decel (松开油门和刹车时)
  v += (驱动力 - 制动力) × dt
  if throttle==0 and brake==0: v → 0 (滑行阻力衰减)

【订阅话题】
  ~/gear_cmd     (Int8)     — 0=N, 1=D, -1=R
  ~/throttle_cmd (Float32)  — 油门踏板 0.0~1.0
  ~/brake_cmd    (Float32)  — 刹车踏板 0.0~1.0

【发布话题】
  /remote_controller/linear_vel_cmd (Float32) — 积分后的当前速度 (m/s)

【参数】运行时可通过 ros2 param set 调整:
  max_accel        最大驱动力加速度 m/s²    默认 3.0
  max_brake_decel  最大制动减速度 m/s²      默认 6.0
  drag_decel       滑行阻力减速度 m/s²      默认 0.5
  max_speed_fwd    前进极速 m/s             默认 20.0
  max_speed_rev    倒车极速 m/s             默认 5.0
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8


class VehicleDynamics(Node):
    """车辆纵向动力学仿真 —— 挂档/油门→加速度→速度积分。"""

    _PUBLISH_RATE = 50.0  # Hz

    def __init__(self):
        super().__init__("vehicle_dynamics")

        # ── 参数 ──
        self.declare_parameter("max_accel", 3.0)
        self.declare_parameter("max_brake_decel", 6.0)
        self.declare_parameter("drag_decel", 0.5)
        self.declare_parameter("max_speed_fwd", 20.0)
        self.declare_parameter("max_speed_rev", 5.0)

        # ── 状态 ──
        self._gear = 1        # 默认 D 档，踩油门就走
        self._throttle = 0.0  # 油门 0~1
        self._brake = 0.0     # 刹车 0~1
        self._velocity = 0.0  # 当前车速 m/s
        self._last_time = self.get_clock().now()

        # ── 订阅 ──
        self.create_subscription(Int8, "~/gear_cmd", self._on_gear, 10)
        self.create_subscription(Float32, "~/throttle_cmd", self._on_throttle, 10)
        self.create_subscription(Float32, "~/brake_cmd", self._on_brake, 10)

        # ── 发布 ──
        self._vel_pub = self.create_publisher(
            Float32, "/remote_controller/linear_vel_cmd", 10
        )

        # ── 积分定时器 ──
        self._timer = self.create_timer(1.0 / self._PUBLISH_RATE, self._integrate)

        self.get_logger().info(
            f"VehicleDynamics 已启动 | "
            f"加速度 {self.get_parameter('max_accel').value:.1f} m/s² | "
            f"制动 {self.get_parameter('max_brake_decel').value:.1f} m/s² | "
            f"极速 FWD {self.get_parameter('max_speed_fwd').value:.0f} / "
            f"REV {self.get_parameter('max_speed_rev').value:.0f} m/s | "
            f"等待指令 {self.get_name()}/gear_cmd ..."
        )

    # ── 回调：更新控制输入 ──
    def _on_gear(self, msg: Int8):
        if msg.data not in (-1, 0, 1):
            self.get_logger().warn(f"无效档位 {msg.data}，有效值: -1(R) 0(N) 1(D)")
            return
        prev = self._gear
        self._gear = msg.data
        gear_label = {-1: "R", 0: "N", 1: "D"}
        self.get_logger().info(
            f"档位 → {gear_label[self._gear]}"
            + (f" | 车速 {self._velocity:.1f} m/s → 换档中" if prev != self._gear else "")
        )

    def _on_throttle(self, msg: Float32):
        self._throttle = max(0.0, min(1.0, msg.data))

    def _on_brake(self, msg: Float32):
        self._brake = max(0.0, min(1.0, msg.data))

    # ── 积分核心 ──
    def _integrate(self):
        """50Hz 定时器：由当前油门/刹车/档位计算加速度并积分速度。"""
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        # 读取参数（允许运行时调整）
        max_accel = self.get_parameter("max_accel").value
        max_brake = self.get_parameter("max_brake_decel").value
        drag = self.get_parameter("drag_decel").value
        max_fwd = self.get_parameter("max_speed_fwd").value
        max_rev = self.get_parameter("max_speed_rev").value

        # ── 加速度计算 ──
        # 驱动力跟随档位方向: D→正, R→负, N→无
        gear_sign = 1 if self._gear == 1 else (-1 if self._gear == -1 else 0)
        drive_accel = self._throttle * max_accel * gear_sign
        brake_decel = self._brake * max_brake

        # 刹车始终抵抗当前运动方向 (不会刹到反向)
        if self._velocity > 0:
            accel = drive_accel - brake_decel
        elif self._velocity < 0:
            accel = drive_accel + brake_decel
        else:
            accel = drive_accel  # 已停稳，刹车不产生加速度

        # 滑行阻力：无油门无刹车时自然减速
        if self._throttle < 1e-6 and self._brake < 1e-6:
            if self._velocity > 0.0:
                accel = -drag
            elif self._velocity < 0.0:
                accel = +drag  # 倒车滑行阻力朝正向

        # ── 速度积分 ──
        self._velocity += accel * dt

        # ── 档位约束 + 过零处理 ──
        if self._gear == 0:  # N 空档
            # 速度自然衰减到 0
            if abs(self._velocity) < drag * dt:
                self._velocity = 0.0
            elif self._velocity > 0.0:
                self._velocity = max(0.0, self._velocity - drag * dt)
            else:
                self._velocity = min(0.0, self._velocity + drag * dt)
            self._velocity = max(-max_rev, min(max_fwd, self._velocity))

        elif self._gear == 1:  # D 前进档
            self._velocity = max(0.0, min(self._velocity, max_fwd))

        elif self._gear == -1:  # R 倒档
            self._velocity = min(0.0, max(self._velocity, -max_rev))

        # ── 发布速度 ──
        self._vel_pub.publish(Float32(data=self._velocity))

        # 状态日志 (2Hz)
        gear_label = {1: "D", 0: "N", -1: "R"}
        self.get_logger().info(
            f"[{gear_label.get(self._gear, '?')}] "
            f"油门 {self._throttle:.0%} "
            f"刹车 {self._brake:.0%} "
            f"→ accel {accel:+5.2f} m/s² "
            f"v {self._velocity:+6.2f} m/s "
            f"({self._velocity*3.6:+5.1f} km/h)",
            throttle_duration_sec=0.5,
        )


def main():
    rclpy.init()
    node = VehicleDynamics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前发送零速指令
        node._vel_pub.publish(Float32(data=0.0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
