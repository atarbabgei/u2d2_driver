#!/usr/bin/env python3
"""Relay a PX4 RC channel onto /u2d2/cmd_abs.

This node owns no hardware. It subscribes to px4_msgs/msg/InputRc, maps one
channel's pulse width onto an absolute logical position, and publishes it on
`cmd_abs` (std_msgs/Float64MultiArray, deg) -- the absolute-from-boot-origin
topic the separately-launched u2d2_driver_ros node drives the motor with. Run
them side by side:

    ros2 launch u2d2_driver_ros u2d2.launch.py        # owns the U2D2 / motor
    ros2 launch u2d2_px4_ros   u2d2_px4.launch.py     # this relay

The driver's boot position is logical 0 ("wherever it starts"), so there is no
encoder calibration. Because cmd_abs targets an absolute position (origin + value)
rather than a relative delta, a steady stick always maps to the exact same goal --
no drift or random-walk under streaming. The servo follows the stick
proportionally and holds its last position on RC loss.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

from px4_msgs.msg import InputRc


class U2D2Px4Node(Node):
    def __init__(self):
        super().__init__('u2d2_px4_node')

        self.declare_parameter('rc_topic', '/fmu/out/input_rc')
        self.declare_parameter('cmd_topic', 'cmd_abs')
        self.declare_parameter('rc_channel', 10)      # 0-based; 10 == RC channel 11
        self.declare_parameter('rc_min', 982)
        self.declare_parameter('rc_max', 2006)
        self.declare_parameter('pos_min', 0.0)        # logical deg at rc_min
        self.declare_parameter('pos_max', 700.0)      # logical deg at rc_max
        self.declare_parameter('update_rate', 20.0)   # control loop Hz
        self.declare_parameter('pos_deadband_deg', 2.0)
        self.declare_parameter('failsafe_hold', True)

        self._rc_channel = int(self.get_parameter('rc_channel').value)
        self._rc_min = int(self.get_parameter('rc_min').value)
        self._rc_max = int(self.get_parameter('rc_max').value)
        self._pos_min = float(self.get_parameter('pos_min').value)
        self._pos_max = float(self.get_parameter('pos_max').value)
        self._pos_deadband = float(self.get_parameter('pos_deadband_deg').value)
        self._failsafe_hold = bool(self.get_parameter('failsafe_hold').value)
        update_rate = float(self.get_parameter('update_rate').value)
        rc_topic = str(self.get_parameter('rc_topic').value)
        cmd_topic = str(self.get_parameter('cmd_topic').value)

        # Last absolute logical position (deg) we published, for the deadband.
        self._last = None

        # Latest RC sample, cached by the subscription and consumed by the
        # control timer (so cmd_abs is emitted at update_rate, not RC rate).
        self._rc_values: list[int] | None = None
        self._rc_blocked = False  # rc_failsafe or rc_lost

        self._cmd_pub = self.create_publisher(Float64MultiArray, cmd_topic, 10)
        # PX4 topics are published BEST_EFFORT; the default reliable QoS would
        # silently receive nothing.
        self.create_subscription(InputRc, rc_topic, self._on_rc,
                                 qos_profile_sensor_data)

        if update_rate > 0:
            self.create_timer(1.0 / update_rate, self._control_tick)

        self.get_logger().info(
            f'u2d2_px4_node ready: channel={self._rc_channel}, '
            f'{self._rc_min}->{self._pos_min:.0f} deg .. '
            f'{self._rc_max}->{self._pos_max:.0f} deg -> {cmd_topic}')

    # -- helpers ----------------------------------------------------------
    def _map_rc(self, value: int) -> float:
        """Clamp a pulse width to [rc_min, rc_max] and linear-map to logical deg."""
        lo, hi = self._rc_min, self._rc_max
        v = max(min(value, max(lo, hi)), min(lo, hi))
        frac = (v - lo) / (hi - lo) if hi != lo else 0.0
        return self._pos_min + frac * (self._pos_max - self._pos_min)

    # -- RC relay ---------------------------------------------------------
    def _on_rc(self, msg: InputRc) -> None:
        self._rc_blocked = bool(msg.rc_failsafe) or bool(msg.rc_lost)
        self._rc_values = list(msg.values)

    def _control_tick(self) -> None:
        if self._rc_values is None:
            return
        if self._rc_blocked and self._failsafe_hold:
            self.get_logger().warning(
                'RC failsafe/lost; holding last position',
                throttle_duration_sec=5.0)
            return
        if self._rc_channel >= len(self._rc_values):
            self.get_logger().error(
                f'rc_channel {self._rc_channel} out of range '
                f'(have {len(self._rc_values)} channels)',
                throttle_duration_sec=5.0)
            return

        desired = self._map_rc(self._rc_values[self._rc_channel])
        if self._last is not None and abs(desired - self._last) < self._pos_deadband:
            return
        self._cmd_pub.publish(Float64MultiArray(data=[desired]))
        self._last = desired


def main(args=None) -> None:
    rclpy.init(args=args)
    node = U2D2Px4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
