#!/usr/bin/env python3
"""
GB29 方向盘控制器：订阅 ROS2 /joy → 映射到车辆控制指令。

【数据流】
  /dev/input/jsX ─→ joy_node ─→ /joy (sensor_msgs/Joy)
                                     │
                                     ▼
                              gb29_controller
                                     │
              ┌──────────────────────┼──────────────────────┐
              ▼                      ▼                      ▼
    theta1L_cmd (Float32)   gear/throttle/brake     reset odom (Bool)
    → steering_controller   → vehicle_dynamics      → odometry_publisher

【GB29 默认映射[所有索引可通过参数调整]】
  ┌────────────┬───────────┬──────────────────────────┐
  │ 控件        │ 默认索引   │ 输出                      │
  ├────────────┼───────────┼──────────────────────────┤
  │ 方向盘      │ axis 0    │theta1L = axis × max_steer│
  │ 油门踏板    │ axis 1    │throttle = remap(axis)    │
  │ 刹车踏板    │ axis 2    │brake = remap(axis)       │
  │ 右拨片      │ button 5  │gear = D (前进)           │
  │ 左拨片      │ button 4  │gear = R (倒车)           │
  │ A 按钮      │ button 0  │gear = N (空档)           │
  │ B 按钮      │ button 1  │reset odom               │
  └────────────┴───────────┴──────────────────────────┘

【发布话题】
  /steering_controller/theta1L_cmd   (Float32)  — 1桥左轮转角 (rad)
  /vehicle_dynamics/gear_cmd         (Int8)      — 0=N, 1=D, -1=R
  /vehicle_dynamics/throttle_cmd     (Float32)   — 油门 0.0~1.0
  /vehicle_dynamics/brake_cmd        (Float32)   — 刹车 0.0~1.0
  /odometry_publisher/reset          (Bool)      — 里程计复位

【参数】
  max_steer_angle   最大转向角 rad               默认 0.785 (±45°)
  steer_axis        方向盘轴索引                  默认 0
  steer_deadzone    方向盘死区                    默认 0.02
  throttle_axis     油门轴索引                    默认 1
  brake_axis        刹车轴索引                    默认 2
  pedal_input_min   踏板原始最小值                默认 -1.0
  pedal_input_max   踏板原始最大值                默认 1.0
  button_gear_d     D 档按钮索引                  默认 5
  button_gear_r     R 档按钮索引                  默认 4
  button_gear_n     N 档按钮索引                  默认 0
  button_reset      复位按钮索引                  默认 1

【使用方式】
  # 1. 先确认方向盘设备:
  ros2 run joy joy_enumerate_devices

  # 2. 通过 launch 文件启动（推荐）:
  ros2 launch vehicle_4axle_8steer gb29_drive.launch.py

  # 3. 手动启动（调试用）:
  ros2 run joy joy_node --ros-args -p device_id:=0
  ros2 run vehicle_4axle_8steer gb29_controller.py

  # 4. 运行时调整映射（如踏板极性反了）:
  ros2 param set /gb29_controller pedal_input_min 0.0
  ros2 param set /gb29_controller pedal_input_max 1.0
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, Int8


class GB29Controller(Node):
    """GB29 方向盘 → 车辆控制指令映射节点"""

    def __init__(self):
        super().__init__("gb29_controller")

        # ── 参数：轴映射 ──
        self.declare_parameter("max_steer_angle", math.pi / 4)  # ±45°
        self.declare_parameter("steer_axis", 0)
        self.declare_parameter("steer_deadzone", 0.02)
        self.declare_parameter("steer_invert", False)
        self.declare_parameter("throttle_axis", 1)
        self.declare_parameter("brake_axis", 2)
        self.declare_parameter("pedal_input_min", -1.0)
        self.declare_parameter("pedal_input_max", 1.0)

        # ── 参数：按钮映射 ──
        self.declare_parameter("button_gear_d", 5)
        self.declare_parameter("button_gear_r", 4)
        self.declare_parameter("button_gear_n", 0)
        self.declare_parameter("button_reset", 1)

        # ── 状态 ──
        self._gear = 0
        self._button_state = {}  # 按钮边沿检测 (防重复触发)

        # ── 订阅 ──
        self.create_subscription(Joy, "/joy", self._on_joy, 10)

        # ── 发布 ──
        self._steer_pub = self.create_publisher(
            Float32, "/steering_controller/theta1L_cmd", 10
        )
        self._gear_pub = self.create_publisher(Int8, "/vehicle_dynamics/gear_cmd", 10)
        self._throttle_pub = self.create_publisher(
            Float32, "/vehicle_dynamics/throttle_cmd", 10
        )
        self._brake_pub = self.create_publisher(
            Float32, "/vehicle_dynamics/brake_cmd", 10
        )
        self._reset_pub = self.create_publisher(Bool, "/odometry_publisher/reset", 10)

        self.get_logger().info(
            "GB29 方向盘控制器已启动 | "
            f"方向盘轴: {self.get_parameter('steer_axis').value} | "
            f"油门轴: {self.get_parameter('throttle_axis').value} | "
            f"刹车轴: {self.get_parameter('brake_axis').value} | "
            f"等待 /joy 数据..."
        )

    # ── 回调 ──
    def _on_joy(self, msg: Joy):
        """处理手柄/方向盘输入 — 保持最后一次值，不自动归零"""
        # 前5帧打印原始数据用于诊断
        if not hasattr(self, '_diag_count'):
            self._diag_count = 0
        if self._diag_count < 5:
            self._diag_count += 1
            self.get_logger().info(
                f"[诊断 {self._diag_count}/5] axes({len(msg.axes)}): {list(msg.axes)} | "
                f"buttons({len(msg.buttons)}): {list(msg.buttons)}"
            )

        # ── 方向盘 → 转向角 ──
        steer_axis = self.get_parameter("steer_axis").value
        if steer_axis < len(msg.axes):
            raw = msg.axes[steer_axis]
            deadzone = self.get_parameter("steer_deadzone").value
            if abs(raw) < deadzone:
                raw = 0.0
            max_steer = self.get_parameter("max_steer_angle").value
            steer_invert = self.get_parameter("steer_invert").value
            sign = -1.0 if steer_invert else 1.0
            theta1L = sign * raw * max_steer
            self._steer_pub.publish(Float32(data=theta1L))
        else:
            self.get_logger().error(  # 改成 error，不再 throttle
                f"steer_axis={steer_axis} 超出范围 (axes 共 {len(msg.axes)} 个)"
            )

        # ── 油门踏板 → throttle 0~1 ──
        throttle_axis = self.get_parameter("throttle_axis").value
        if throttle_axis < len(msg.axes):
            throttle = self._remap_pedal(msg.axes[throttle_axis])
            self._throttle_pub.publish(Float32(data=throttle))
        else:
            self.get_logger().error(
                f"throttle_axis={throttle_axis} 超出范围 (axes 共 {len(msg.axes)} 个)"
            )

        # ── 刹车踏板 → brake 0~1 ──
        brake_axis = self.get_parameter("brake_axis").value
        if brake_axis < len(msg.axes):
            brake = self._remap_pedal(msg.axes[brake_axis])
            self._brake_pub.publish(Float32(data=brake))
        else:
            self.get_logger().error(
                f"brake_axis={brake_axis} 超出范围 (axes 共 {len(msg.axes)} 个)"
            )

        # ── 按钮：检测所有按钮按下（打印日志确认收到的按钮索引） ──
        for i in range(len(msg.buttons)):
            pressed = msg.buttons[i] == 1
            was_pressed = self._button_state.get(i, 0) == 1
            if pressed != was_pressed:
                self._button_state[i] = 1 if pressed else 0
                self.get_logger().info(f"🎮 按钮{i} {'按下' if pressed else '松开'}")

        # ── 档位按钮（上升沿触发） ──
        btn_d = self.get_parameter("button_gear_d").value
        btn_r = self.get_parameter("button_gear_r").value
        btn_n = self.get_parameter("button_gear_n").value

        if btn_d < len(msg.buttons) and msg.buttons[btn_d] == 1 and not self._button_state.get(f"gear_{btn_d}", False):
            self._gear_pub.publish(Int8(data=1))
            self.get_logger().info("🕹 档位 → D")
        self._button_state[f"gear_{btn_d}"] = (btn_d < len(msg.buttons) and msg.buttons[btn_d] == 1)

        if btn_r < len(msg.buttons) and msg.buttons[btn_r] == 1 and not self._button_state.get(f"gear_{btn_r}", False):
            self._gear_pub.publish(Int8(data=-1))
            self.get_logger().info("🕹 档位 → R")
        self._button_state[f"gear_{btn_r}"] = (btn_r < len(msg.buttons) and msg.buttons[btn_r] == 1)

        if btn_n < len(msg.buttons) and msg.buttons[btn_n] == 1 and not self._button_state.get(f"gear_{btn_n}", False):
            self._gear_pub.publish(Int8(data=0))
            self.get_logger().info("🕹 档位 → N")
        self._button_state[f"gear_{btn_n}"] = (btn_n < len(msg.buttons) and msg.buttons[btn_n] == 1)

        # ── 按钮：里程计复位 ──
        reset_btn = self.get_parameter("button_reset").value
        if reset_btn < len(msg.buttons) and msg.buttons[reset_btn] == 1 and not self._button_state.get(f"reset_{reset_btn}", False):
            self._reset_pub.publish(Bool(data=True))
            self.get_logger().info("🔁 里程计复位")
        self._button_state[f"reset_{reset_btn}"] = (reset_btn < len(msg.buttons) and msg.buttons[reset_btn] == 1)

    def _remap_pedal(self, raw: float) -> float:
        """将踏板原始输入 [pedal_input_min, pedal_input_max] 线性映射到 [0, 1]

        GB29 踏板常见报告范围:
          - [-1.0, 1.0]  → 松开=-1, 踩到底=1  (默认参数)
          - [0.0, 1.0]   → 松开=0,  踩到底=1  (调整 pedal_input_min=0)
        """
        lo = self.get_parameter("pedal_input_min").value
        hi = self.get_parameter("pedal_input_max").value
        if abs(hi - lo) < 1e-9:
            return 0.0
        return max(0.0, min(1.0, (raw - lo) / (hi - lo)))

def main():
    rclpy.init()
    node = GB29Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前发送安全指令
        node._steer_pub.publish(Float32(data=0.0))
        node._throttle_pub.publish(Float32(data=0.0))
        node._brake_pub.publish(Float32(data=1.0))
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
