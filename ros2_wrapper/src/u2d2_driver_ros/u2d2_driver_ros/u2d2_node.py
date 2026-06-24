#!/usr/bin/env python3
"""ROS 2 node wrapping u2d2_driver.Driver.

Subscribes to deg/rpm command topics (std_msgs/Float64MultiArray) and drives
Dynamixel X-series motors through a ROBOTIS U2D2; publishes joint telemetry as
sensor_msgs/JointState in SI units (radians, rad/s).

Single-threaded by design (default executor): command callbacks and the
telemetry timer never run concurrently, so the serial bus is only ever touched
by one of them at a time -- no locking needed.

Command arrays map positionally to the `ids` parameter; a single value
broadcasts to every motor, mirroring the CLI's per-id behaviour.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from u2d2_driver import (
    COMMON_BAUDS,
    OP_MODE_EXTENDED_POSITION,
    OP_MODE_VELOCITY,
    Driver,
    autodetect_port,
    detect_baud,
)

DEG2RAD = math.pi / 180.0
RPM2RADPS = 2.0 * math.pi / 60.0


class U2D2Node(Node):
    def __init__(self):
        super().__init__('u2d2_node')

        self.declare_parameter('port', '')
        self.declare_parameter('baud', 0)
        self.declare_parameter('ids', [1])
        self.declare_parameter('profile_vel', 0.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('release_on_shutdown', True)

        port = self.get_parameter('port').value or autodetect_port()
        baud = int(self.get_parameter('baud').value)
        self._ids = [int(i) for i in self.get_parameter('ids').value]
        self._profile_vel = float(self.get_parameter('profile_vel').value)
        self._release = bool(self.get_parameter('release_on_shutdown').value)
        rate = float(self.get_parameter('publish_rate').value)

        # Connect: pinned baud, or scan the common bauds and lock onto the
        # first one where motors answer (same policy as the CLI).
        self._driver = Driver(port, baud if baud else COMMON_BAUDS[0])
        if baud:
            self.get_logger().info(f'U2D2 on {port} @ {baud} baud')
            for i in self._ids:
                if not self._driver.ping(i):
                    self.get_logger().warning(
                        f'motor ID {i} did not answer at {baud} baud')
        else:
            detected, found = detect_baud(self._driver, COMMON_BAUDS)
            if detected is None:
                raise RuntimeError('no motors answered at any common baud; '
                                   'check power/wiring or set the baud param')
            self.get_logger().info(
                f'U2D2 on {port} @ {detected} baud (auto-detected), motors {found}')

        # Per-id operating-mode cache so streaming velocity commands don't
        # toggle torque (and thus the operating mode) on every message.
        self._mode: dict[int, int] = {}

        self.create_subscription(Float64MultiArray, 'cmd_velocity',
                                 self._on_velocity, 10)
        self.create_subscription(Float64MultiArray, 'cmd_position',
                                 self._on_position, 10)
        self.create_subscription(Float64MultiArray, 'cmd_turn',
                                 self._on_turn, 10)

        self._js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._names = [f'id_{i}' for i in self._ids]
        if rate > 0:
            self.create_timer(1.0 / rate, self._publish_joint_states)

        self.get_logger().info(f'u2d2_node ready: ids={self._ids}')

    # -- helpers ----------------------------------------------------------
    def _expand(self, data, what: str):
        """Map a Float64MultiArray to {id: value}: one value broadcasts to all
        ids, otherwise the count must match len(ids)."""
        vals = list(data)
        if len(vals) == 1:
            return {i: vals[0] for i in self._ids}
        if len(vals) == len(self._ids):
            return dict(zip(self._ids, vals))
        self.get_logger().error(
            f'{what}: expected 1 or {len(self._ids)} values, got {len(vals)}')
        return None

    def _ensure_mode(self, dxl_id: int, mode: int) -> None:
        """Switch a motor's operating mode only when it actually changes;
        switching toggles torque, so we avoid doing it per streamed command."""
        if self._mode.get(dxl_id) == mode:
            return
        if mode == OP_MODE_VELOCITY:
            self._driver.enable_velocity_mode(dxl_id)
        else:
            self._driver.enable_position_mode(dxl_id)
            if self._profile_vel > 0:
                self._driver.set_profile_velocity(dxl_id, self._profile_vel)
        self._mode[dxl_id] = mode

    # -- command callbacks ------------------------------------------------
    def _on_velocity(self, msg: Float64MultiArray) -> None:
        goals = self._expand(msg.data, 'cmd_velocity')
        if goals is None:
            return
        for i, rpm in goals.items():
            try:
                self._ensure_mode(i, OP_MODE_VELOCITY)
                self._driver.set_velocity(i, float(rpm))
            except RuntimeError as exc:
                self.get_logger().warning(f'[ID {i}] cmd_velocity failed: {exc}')

    def _on_position(self, msg: Float64MultiArray) -> None:
        goals = self._expand(msg.data, 'cmd_position')
        if goals is None:
            return
        for i, deg in goals.items():
            try:
                self._ensure_mode(i, OP_MODE_EXTENDED_POSITION)
                self._driver.move_shortest_to(i, float(deg))
            except RuntimeError as exc:
                self.get_logger().warning(f'[ID {i}] cmd_position failed: {exc}')

    def _on_turn(self, msg: Float64MultiArray) -> None:
        goals = self._expand(msg.data, 'cmd_turn')
        if goals is None:
            return
        for i, deg in goals.items():
            try:
                self._ensure_mode(i, OP_MODE_EXTENDED_POSITION)
                self._driver.turn_by(i, float(deg))
            except RuntimeError as exc:
                self.get_logger().warning(f'[ID {i}] cmd_turn failed: {exc}')

    # -- telemetry --------------------------------------------------------
    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._names
        positions, velocities = [], []
        for i in self._ids:
            try:
                positions.append(self._driver.present_position_deg(i) * DEG2RAD)
                velocities.append(self._driver.present_velocity_rpm(i) * RPM2RADPS)
            except RuntimeError as exc:
                # A dropped telemetry read is non-fatal; skip this tick.
                self.get_logger().warning(
                    f'[ID {i}] telemetry read failed: {exc}',
                    throttle_duration_sec=5.0)
                return
        msg.position = positions
        msg.velocity = velocities
        self._js_pub.publish(msg)

    # -- shutdown ---------------------------------------------------------
    def shutdown(self) -> None:
        try:
            self._driver.stop_all(disable_torque=self._release)
            self._driver.close()
        except Exception as exc:  # best-effort on the way out
            self.get_logger().warning(f'shutdown warning: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = U2D2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
