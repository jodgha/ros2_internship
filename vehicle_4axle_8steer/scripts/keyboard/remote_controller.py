#!/usr/bin/env python3
"""
车辆遥控器 — 命令行交互，挂档油门控制。

【挂档 + 油门控制】
  gear=d   前进 D 档     gear=n   空档 N     gear=r   倒车 R 档
  w/s      油门 ±10%     b        全刹
  acl=0.5   设油门深度   brk=0.3  设刹车深度 (0~1)

【转向】
  a/d      左转/右转 ±3°      enter    回正方向盘
  rad=数字  设定转角 (rad)     +15      快捷左转 15°

【其他】
  0        全复位 (回正+松油松刹)    home     回原点复位里程计
  demo     正弦摆动演示              circle   圆周运动
  crab     蟹行模式切换              track=x,y  PID 驶向目标点
  q        退出

物理模型 (50Hz):
  驱动力 = 油门 × 3.0 m/s²   制动力 = 刹车 × 6.0 m/s²
  滑行阻力 = 0.5 m/s² (松油松刹自然减速)
  D 档: v ∈ [0, 20]   R 档: v ∈ [-5, 0]   N 档: v → 0

状态栏: [🕹D [▐▐▐▐░░░░░░]] = D档 40%油门 | 转向角 | 转弯半径 | 速度 | 横摆角速度
"""
import math
import sys
import threading
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry

from vehicle_4axle.steer_calc import X_LIST, W_LIST, WHEEL_RADIUS

_X1 = X_LIST[0]
_W1 = W_LIST[0]


class RemoteController(Node):

    def __init__(self):
        super().__init__("remote_controller")

        self.steer_pub = self.create_publisher(
            Float32, "/steering_controller/theta1L_cmd", 10
        )
        self.wheel_spin_pub = self.create_publisher(
            Float32, "/steering_controller/wheel_vel_cmd", 10
        )
        self.linear_vel_pub = self.create_publisher(
            Float32, "/remote_controller/linear_vel_cmd", 10
        )
        self.reset_odom_pub = self.create_publisher(
            Bool, "/odometry_publisher/reset", 10
        )

        self.theta = 0.0          # 转向角 (rad)
        self.linear_vel = 0.0     # 当前速度 (m/s)

        # ── 挂档状态 ──
        self._gear_val = 0        # -1=R 0=N 1=D
        self._throttle_val = 0.0  # 油门 0~1
        self._brake_val = 0.0     # 刹车 0~1

        # ── 物理参数 ──
        self._max_accel = 3.0
        self._max_brake_decel = 6.0
        self._drag_decel = 0.5
        self._max_speed_fwd = 20.0
        self._max_speed_rev = 5.0
        self._physics_last = None
        self._physics_timer = self.create_timer(0.02, self._physics_tick)

        # ── 演示 ──
        self._demo_active = False
        self._circle_active = False
        self._crab_mode = False
        self._demo_t = 0.0
        self._demo_last_time = None
        self._demo_timer = None

        # ── 路径跟踪 ──
        self._tracking = False
        self._track_wp = 0
        self._track_x = 0.0
        self._track_y = 0.0
        self._track_yaw = 0.0
        self._odom_ready = False
        self._track_arrived = False
        self._track_timer = None
        self._track_last_time = None
        self._track_waypoints = []
        self._track_loop = True

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._publish_all()

    # ═══════════════ 物理积分 ═══════════════

    def _physics_tick(self):
        """50Hz 物理积分：油门/刹车/档位 → 加速度 → 速度。"""
        now = self.get_clock().now()
        if self._physics_last is None:
            self._physics_last = now
            return
        dt = (now - self._physics_last).nanoseconds / 1e9
        self._physics_last = now
        if dt <= 0.0 or dt > 1.0:
            return

        # 油门驱动力 跟随档位方向: D→正 R→负 N→无
        gear_sign = 1 if self._gear_val == 1 else (-1 if self._gear_val == -1 else 0)
        drive = self._throttle_val * self._max_accel * gear_sign

        # 刹车: 始终抵抗当前运动方向 (不会把车刹到反向)
        brake_force = self._brake_val * self._max_brake_decel
        if self.linear_vel > 0:
            accel = drive - brake_force
        elif self.linear_vel < 0:
            accel = drive + brake_force
        else:
            accel = drive  # 已停稳，刹车不产生加速度

        # 滑行阻力: 无油门无刹车时自然减速
        if self._throttle_val < 1e-6 and self._brake_val < 1e-6:
            if self.linear_vel > 0.0:
                accel = -self._drag_decel
            elif self.linear_vel < 0.0:
                accel = +self._drag_decel

        self.linear_vel += accel * dt

        # 档位约束
        if self._gear_val == 0:  # N
            d = self._drag_decel * dt
            if abs(self.linear_vel) < d:
                self.linear_vel = 0.0
            elif self.linear_vel > 0.0:
                self.linear_vel = max(0.0, self.linear_vel - self._drag_decel * dt)
            else:
                self.linear_vel = min(0.0, self.linear_vel + self._drag_decel * dt)
        elif self._gear_val == 1:  # D
            self.linear_vel = max(0.0, min(self.linear_vel, self._max_speed_fwd))
        elif self._gear_val == -1:  # R
            self.linear_vel = min(0.0, max(self.linear_vel, -self._max_speed_rev))

        # 演示/跟踪不发布（它们有自己的 _publish_all 调用）
        if not self._demo_active and not self._circle_active and not self._tracking:
            self._publish_all()

    # ═══════════════ 发布 ═══════════════

    def _publish_all(self):
        self.steer_pub.publish(Float32(data=self.theta))
        self.wheel_spin_pub.publish(Float32(data=abs(self.linear_vel) / WHEEL_RADIUS))
        self.linear_vel_pub.publish(Float32(data=self.linear_vel))

    # ═══════════════ 控制 ═══════════════

    def steer_by(self, delta_deg: float):
        self.theta += math.radians(delta_deg)
        self.theta = max(-math.pi / 4, min(math.pi / 4, self.theta))
        self._publish_all()

    def set_steer(self, theta_rad: float):
        self.theta = max(-math.pi / 4, min(math.pi / 4, theta_rad))
        self._publish_all()

    def set_steer_deg(self, deg: float):
        self.set_steer(math.radians(deg))

    def set_gear(self, gear_val: int):
        self._gear_val = gear_val
        gear_label = {1: "D", 0: "N", -1: "R"}
        self.get_logger().info(f"🕹 档位 → {gear_label[gear_val]}")

    def change_throttle(self, dv: float):
        self._throttle_val = max(0.0, min(1.0, self._throttle_val + dv))
        self._brake_val = 0.0

    def set_throttle(self, val: float):
        self._throttle_val = max(0.0, min(1.0, val))
        self._brake_val = 0.0

    def set_brake(self, val: float):
        self._brake_val = max(0.0, min(1.0, val))

    def reset_all(self):
        self.theta = 0.0
        self._throttle_val = 0.0
        self._brake_val = 0.0
        self._publish_all()

    def reset_odometry(self):
        self._demo_active = False
        self._circle_active = False
        self._tracking = False
        self._stop_demo_timer()
        self._stop_tracking()
        self.theta = 0.0
        self.linear_vel = 0.0
        self._throttle_val = 0.0
        self._brake_val = 0.0
        self._physics_last = None
        self._publish_all()
        self.reset_odom_pub.publish(Bool(data=True))
        self.get_logger().info("🏠 已回到初始位置 (回正 + 停车 + 归零)")

    # ═══════════════ 演示 ═══════════════

    def _start_demo_timer(self):
        if self._demo_timer is None:
            self._demo_timer = self.create_timer(0.05, self._demo_tick)

    def _stop_demo_timer(self):
        if self._demo_timer is not None:
            self.destroy_timer(self._demo_timer)
            self._demo_timer = None

    def _demo_tick(self):
        if not self._demo_active and not self._circle_active:
            self._stop_demo_timer()
            return
        now = self.get_clock().now()
        if self._demo_last_time is None:
            self._demo_last_time = now
            return
        dt = (now - self._demo_last_time).nanoseconds / 1e9
        self._demo_last_time = now
        if dt > 1.0:
            return
        self._demo_t += dt
        if self._demo_active:
            self.theta = math.radians(30) * math.sin(2 * math.pi * self._demo_t / 4.0)
        elif self._circle_active:
            self.theta = 0.26
        self._publish_all()

    def toggle_demo(self):
        self._circle_active = False
        self._tracking = False
        self._stop_tracking()
        self._demo_active = not self._demo_active
        if self._demo_active:
            self._demo_t = 0.0
            self._demo_last_time = None
            self._start_demo_timer()
            self.get_logger().info("🔁 正弦摆动演示 开启 (±30°, 4s周期)")
        else:
            self.theta = 0.0
            self._stop_demo_timer()
            self._publish_all()
            self.get_logger().info("🔁 正弦摆动演示 关闭, 已回正")

    def toggle_circle(self):
        self._demo_active = False
        self._tracking = False
        self._stop_tracking()
        self._circle_active = not self._circle_active
        if self._circle_active:
            self._demo_last_time = None
            self.theta = 0.26
            self._start_demo_timer()
            self._publish_all()
            self.get_logger().info("⭕ 圆周运动演示 开启 (15°转角, R≈20m)")
        else:
            self.theta = 0.0
            self._stop_demo_timer()
            self._publish_all()
            self.get_logger().info("⭕ 圆周运动演示 关闭, 已回正")

    def toggle_crab(self):
        self._crab_mode = not self._crab_mode
        mode = "high_speed" if self._crab_mode else "low_speed"
        try:
            result = subprocess.run(
                ["ros2", "param", "set", "/steering_controller", "mode", mode],
                capture_output=True, text=True, timeout=5,
            )
            if result.returncode != 0:
                self.get_logger().error(f"ros2 param set 失败: {result.stderr.strip()}")
                self._crab_mode = not self._crab_mode
                return
            icon = "🦀" if self._crab_mode else "🚛"
            self.get_logger().info(f"{icon} 转向模式已切换为: {mode}")
        except (FileNotFoundError, subprocess.TimeoutExpired) as e:
            self.get_logger().error(f"切换蟹行失败: {e}")
            self._crab_mode = not self._crab_mode

    # ═══════════════ 路径跟踪 ═══════════════

    def _on_odom(self, msg: Odometry):
        self._track_x = msg.pose.pose.position.x
        self._track_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._track_yaw = math.atan2(siny, cosy)
        self._odom_ready = True

    def _start_tracking(self):
        if self._track_timer is None:
            self._track_timer = self.create_timer(0.05, self._track_tick)

    def _stop_tracking(self):
        if self._track_timer is not None:
            self.destroy_timer(self._track_timer)
            self._track_timer = None
        self.theta = 0.0
        self.linear_vel = 0.0
        self._throttle_val = 0.0
        self._brake_val = 0.0
        self._publish_all()

    def toggle_track(self, waypoints_str: str = ""):
        if self._tracking:
            self._tracking = False
            self._track_arrived = False
            self._stop_tracking()
            self.get_logger().info("📍 路径跟踪 关闭, 已停车")
            return

        if waypoints_str:
            self._track_waypoints = []
            for pair in waypoints_str.split(";"):
                pair = pair.strip()
                if not pair:
                    continue
                parts = pair.split(",")
                if len(parts) != 2:
                    continue
                self._track_waypoints.append((float(parts[0]), float(parts[1])))

        if len(self._track_waypoints) == 0:
            self.get_logger().error("用法: track=x,y  例: track=10,0")
            return

        self._track_loop = len(self._track_waypoints) > 1
        self._demo_active = False
        self._circle_active = False
        self._stop_demo_timer()
        self._tracking = True
        self._track_arrived = False
        self._track_wp = 0
        self._track_last_time = None
        self._start_tracking()
        self.get_logger().info(
            f"📍 路径跟踪 开启 | {len(self._track_waypoints)} 航点 | "
            f"{'循环' if self._track_loop else '到点即停'}"
        )

    def _track_tick(self):
        if not self._tracking or self._track_arrived:
            self._stop_tracking()
            return
        if not self._odom_ready:
            return

        now = self.get_clock().now()
        if self._track_last_time is None:
            self._track_last_time = now
            return
        dt = (now - self._track_last_time).nanoseconds / 1e9
        self._track_last_time = now
        if dt > 1.0:
            return

        wx, wy = self._track_waypoints[self._track_wp]
        dx = wx - self._track_x
        dy = wy - self._track_y
        dist = math.hypot(dx, dy)

        if dist < 1.0:
            if self._track_loop:
                self._track_wp = (self._track_wp + 1) % len(self._track_waypoints)
                self.get_logger().info(f"📍 → WP{self._track_wp}")
            else:
                self._track_arrived = True
                self.theta = 0.0
                self.linear_vel = 0.0
                self._publish_all()
                self._stop_tracking()
                self.get_logger().info(f"📍 到达 ({wx:.0f},{wy:.0f}) ✓ 已停车")
            return

        if dist < 3.0:
            self.theta = 0.0
            self.linear_vel = 0.5
        else:
            target_heading = math.atan2(dy, dx)
            heading_err = target_heading - self._track_yaw
            heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))
            self.theta = max(-math.pi / 4, min(math.pi / 4, 0.6 * heading_err))
            self.linear_vel = min(3.0, 0.5 * dist)

        self._publish_all()

        self.get_logger().info(
            f"→({wx:.0f},{wy:.0f}) "
            f"偏差{math.degrees(math.atan2(dy, dx) - self._track_yaw):+5.1f}° "
            f"打轮{math.degrees(self.theta):+5.1f}° "
            f"距{dist:5.1f}m 速{self.linear_vel:4.2f}m/s",
            throttle_duration_sec=0.5,
        )

    # ═══════════════ 状态栏 ═══════════════

    def status_line(self) -> str:
        vel = self.linear_vel
        R = (_X1 / math.tan(self.theta) + _W1 / 2.0) if abs(self.theta) > 1e-9 else float("inf")
        omega = 0.0 if math.isinf(R) else vel / R
        steer_dir = "◀左" if self.theta > 1e-9 else ("右▶" if self.theta < -1e-9 else "直行")
        R_str = f"{R:6.1f}m" if not math.isinf(R) else "   ∞  "

        tags = []
        gear_label = {1: "D", 0: "N", -1: "R"}
        bar = "▐" * int(self._throttle_val * 10) + "░" * (10 - int(self._throttle_val * 10))
        brake_icon = f"🛑{self._brake_val:.0%}" if self._brake_val > 1e-6 else ""
        tags.append(f"🕹{gear_label.get(self._gear_val, '?')} [{bar}]{brake_icon}")

        if self._demo_active:
            tags.append("🔁demo")
        if self._circle_active:
            tags.append("⭕circle")
        if self._crab_mode:
            tags.append("🦀crab")
        if self._tracking:
            wp = self._track_waypoints[self._track_wp] if self._track_wp < len(self._track_waypoints) else (0, 0)
            tags.append(f"📍→({wp[0]:.0f},{wp[1]:.0f})")
        tag_str = ("[" + "][".join(tags) + "] ") if tags else ""

        return (
            f"{tag_str}"
            f"θ={math.degrees(self.theta):+5.1f}° {steer_dir}"
            f" | R={R_str}"
            f" | v={vel:+5.1f} m/s ({abs(vel) * 3.6:4.1f}km/h)"
            f" | ω={math.degrees(omega):+5.1f}°/s"
        )


def main():
    print("启动遥控器...", flush=True)
    rclpy.init()
    ctrl = RemoteController()

    spin_thread = threading.Thread(target=rclpy.spin, args=(ctrl,), daemon=True)
    spin_thread.start()

    print("""
╔══════════════════════════════════════════════════╗
║   🚛 四桥八轮 — 遥控驾驶                                    ║
╠══════════════════════════════════════════════════╣
║   【挂档】  gear=d 前进    gear=n 空档    gear=r 倒车       ║
║   【踏板】  w/s 油门 ±10%     b  全刹                        ║
║            acl=0.3  油门深度     brk=0.5  刹车深度 (0~1)       ║
║   【转向】  a/d 转向 ±3°       enter 回正                   ║
║            rad=0.26 设定转角    +15  快捷左转15°            ║
║──────────────────────────────────────────────────║
║   0 全复位    home 回原点    q 退出                        ║
║──────────────────────────────────────────────────║
║   demo 正弦摆动    circle 圆周    crab 蟹行               ║
║   track=x,y  PID驶向目标点    track=x,y;... 多航点循环      ║
╚══════════════════════════════════════════════════╝
""")

    try:
        while rclpy.ok():
            sys.stdout.write(f"\r{ctrl.status_line()}")
            sys.stdout.flush()

            try:
                cmd = input()
            except EOFError:
                break

            raw = cmd.strip()
            cmd_lower = raw.lower()

            # ── 油门: acl=数字 或 throttle=数字 ──
            if cmd_lower.startswith("acl=") or cmd_lower.startswith("throttle="):
                try:
                    tv = float(raw.split("=", 1)[1])
                    ctrl.set_throttle(tv)
                    print(f"  ✓ 油门: {tv:.0%}")
                    continue
                except ValueError:
                    print("  用法: acl=0.5 (0~1)")
                    continue

            # ── 转角: rad=数字 ──
            if raw.startswith("rad="):
                try:
                    rad_val = float(raw[4:])
                    ctrl.set_steer(rad_val)
                    print(f"  ✓ 转角: {rad_val:+.3f} rad ({math.degrees(rad_val):+.1f}°)")
                    continue
                except ValueError:
                    print("  用法: rad=0.26")
                    continue

            # ── 快捷转角: +数字 或 -数字 ──
            if raw.startswith("+") or raw.startswith("-"):
                try:
                    deg = float(raw)
                    ctrl.set_steer_deg(deg)
                    print(f"  ✓ 转角: {deg:+.1f}°")
                    continue
                except ValueError:
                    pass

            # ── 单键命令 ──
            if cmd_lower in ("w",):
                ctrl.change_throttle(0.1)
            elif cmd_lower in ("s",):
                ctrl.change_throttle(-0.1)
            elif cmd_lower in ("a",):
                ctrl.steer_by(3.0)
            elif cmd_lower in ("d",):
                ctrl.steer_by(-3.0)
            elif cmd_lower in ("", " "):
                ctrl.set_steer(0.0)
            elif cmd_lower == "0":
                ctrl.reset_all()
            elif cmd_lower == "b":
                ctrl.set_brake(1.0)  # 快捷全刹
            elif cmd_lower.startswith("brk="):
                try:
                    bv = float(raw.split("=", 1)[1])
                    ctrl.set_brake(max(0.0, min(1.0, bv)))
                    print(f"  ✓ 刹车: {bv:.0%}")
                    continue
                except ValueError:
                    print("  用法: brk=0.5 (0~1)  b=全刹")
                    continue
            elif cmd_lower in ("demo",):
                ctrl.toggle_demo()
            elif cmd_lower in ("circle",):
                ctrl.toggle_circle()
            elif cmd_lower in ("crab",):
                ctrl.toggle_crab()
            elif cmd_lower in ("gear=d", "gear=r", "gear=n"):
                gear_map = {"gear=d": 1, "gear=r": -1, "gear=n": 0}
                ctrl.set_gear(gear_map[cmd_lower])
            elif cmd_lower.startswith("gear="):
                try:
                    gv = int(raw[5:])
                    ctrl.set_gear(gv)
                except ValueError:
                    print("  用法: gear=d (前进) gear=n (空档) gear=r (倒车)")
            elif cmd_lower in ("track",):
                print("  用法: track=x,y  例: track=10,0")
            elif cmd_lower.startswith("track="):
                ctrl.toggle_track(raw[6:])
            elif cmd_lower in ("home",):
                ctrl.reset_odometry()
            elif cmd_lower in ("q", "quit", "exit"):
                break
            else:
                print(
                    f"  未知命令: '{raw}'  "
                    "试试: gear=d w s b a d throttle=0.5 rad=0.26 +15 enter 0 home q demo circle crab track"
                )

    except KeyboardInterrupt:
        pass
    finally:
        ctrl._demo_active = False
        ctrl._circle_active = False
        ctrl._tracking = False
        ctrl._stop_demo_timer()
        ctrl._stop_tracking()
        ctrl.reset_all()
        print("\n已退出，车辆已复位。")
        ctrl.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
