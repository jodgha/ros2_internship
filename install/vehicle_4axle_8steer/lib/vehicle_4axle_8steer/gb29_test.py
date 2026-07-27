#!/usr/bin/env python3
"""
GB29 方向盘测试工具 — 逐帧打印所有轴和按钮，用于确认映射。

用法:
  ros2 run joy joy_node --ros-args -p device_id:=0
  ros2 run vehicle_4axle_8steer gb29_test.py

输出示例:
  axes (4):  [  0.00  -1.00  -1.00  -1.00 ]  |  buttons (12):  [0 0 0 0 0 0 0 0 0 0 0 0]
                                            ▲ 有变化时高亮
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class GB29Test(Node):

    def __init__(self):
        super().__init__("gb29_test")
        self._last_axes = None
        self._last_buttons = None
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.get_logger().info("等待 /joy 数据... 操作方向盘/踏板/按钮")

    def _on_joy(self, msg: Joy):
        changed = False

        # 检查变化
        if self._last_axes is None or self._last_buttons is None:
            changed = True
        else:
            for i, v in enumerate(msg.axes):
                if abs(v - self._last_axes[i]) > 0.001:
                    changed = True
                    break
            if not changed:
                for i, v in enumerate(msg.buttons):
                    if v != self._last_buttons[i]:
                        changed = True
                        break

        if not changed:
            return

        self._last_axes = list(msg.axes)
        self._last_buttons = list(msg.buttons)

        # 轴
        ax_parts = []
        for i, v in enumerate(msg.axes):
            ax_parts.append(f"ax{i}:{v:+6.3f}")
        ax_str = "  ".join(ax_parts)

        # 按钮
        btn_str = " ".join(str(b) for b in msg.buttons)

        self.get_logger().info(
            f"\n  axes ({len(msg.axes)}):  [ {ax_str} ]\n"
            f"  buttons ({len(msg.buttons)}):  [ {btn_str} ]"
        )


def main():
    rclpy.init()
    node = GB29Test()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
