# u2d2_px4_ros

ROS 2 (Humble) relay that maps a **PX4 RC channel** onto the `u2d2_driver_ros`
node's `/u2d2/cmd_abs` topic. It owns no hardware: it subscribes to
`/fmu/out/input_rc` (`px4_msgs/msg/InputRc`), maps one channel's pulse width onto
an absolute logical position, and publishes it. Launch the driver separately:

```bash
ros2 launch u2d2_driver_ros u2d2.launch.py        # owns the U2D2 / motor
ros2 launch u2d2_px4_ros   u2d2_px4.launch.py     # this relay
```

The driver's boot position is logical 0 ("wherever it starts"), so there is no
encoder calibration. Because `cmd_abs` targets an absolute position
(`origin + value`) rather than a relative delta, a steady stick always maps to the
exact same goal — no drift or random-walk under streaming (which is why this relay
uses `cmd_abs`, not `cmd_turn`). The servo follows the stick proportionally and
holds its last position on RC loss. Built with
[colcon-uv](https://github.com/atarbabgei/colcon-uv); lives on the `ros` branch.

## Build

```bash
# one-time: install the colcon-uv build tool
pip install git+https://github.com/atarbabgei/colcon-uv.git#subdirectory=colcon_uv --break-system-packages

# source ROS and the workspace that provides px4_msgs (required)
source /opt/ros/humble/setup.bash
source /home/afb23/Dev/px4_dev/ros2_ws/install/setup.bash

# build the workspace
cd ros2_wrapper
colcon build
source install/setup.bash
```

`px4_msgs` must be on the path at build *and* run time. The relay also needs the
PX4 micro-XRCE-DDS agent running so `/fmu/out/input_rc` is published, and the
`u2d2_driver_ros` node running to actually move the motor.

## Run

```bash
ros2 launch u2d2_px4_ros u2d2_px4.launch.py                      # ch 11, 0..700 deg
ros2 launch u2d2_px4_ros u2d2_px4.launch.py rc_channel:=10 pos_max:=700.0
```

Keep the same `namespace` (default `u2d2`) on both launches so the relay's
`cmd_abs` reaches the driver's `/u2d2/cmd_abs`.

### Launch arguments

| Arg                | Default            | Meaning                                            |
|--------------------|--------------------|----------------------------------------------------|
| `namespace`        | `u2d2`             | Topic prefix; must match `u2d2_driver_ros`.        |
| `rc_topic`         | `/fmu/out/input_rc`| `px4_msgs/InputRc` topic.                          |
| `cmd_topic`        | `cmd_abs`          | Absolute-position topic to publish (deg).          |
| `rc_channel`       | `10`               | 0-based channel index (`10` = RC channel 11).      |
| `rc_min`           | `982`              | RC pulse mapped to `pos_min`.                      |
| `rc_max`           | `2006`             | RC pulse mapped to `pos_max`.                      |
| `pos_min`          | `0.0`              | Logical degrees at `rc_min`.                       |
| `pos_max`          | `700.0`            | Logical degrees at `rc_max`.                       |
| `update_rate`      | `20.0`             | Control-loop rate (Hz).                            |
| `pos_deadband_deg` | `2.0`              | Min target change before publishing a move.        |
| `failsafe_hold`    | `true`             | Hold position on `rc_lost` / `rc_failsafe`.        |
| `log_level`        | `info`             | rclpy log level.                                   |

To **invert** direction, swap `pos_min`/`pos_max` (or `rc_min`/`rc_max`).

## Mapping

The selected channel's pulse width is clamped to `[rc_min, rc_max]` and linearly
mapped to `[pos_min, pos_max]` logical degrees; the relay publishes that absolute
position (the driver applies `origin + value`):

| RC pulse | Logical position |
|----------|------------------|
| `982`    | `0` deg          |
| `2006`   | `700` deg        |
| mid      | proportional     |

A steady stick within `pos_deadband_deg` publishes nothing; on RC loss it stops
publishing so the driver holds its last goal (`failsafe_hold`).

## Topics

| Topic               | Type                        | Direction | Effect                                          |
|---------------------|-----------------------------|-----------|-------------------------------------------------|
| `/fmu/out/input_rc` | `px4_msgs/msg/InputRc`      | in        | RC source; channel `rc_channel` drives output.   |
| `/u2d2/cmd_abs`     | `std_msgs/Float64MultiArray`| out       | Absolute position (deg) consumed by `u2d2_driver_ros`.|

The RC subscription uses sensor-data QoS (BEST_EFFORT) to match the PX4 bridge.

### Test

```bash
# is the relay producing commands? (driver node not required to observe this)
ros2 topic echo /u2d2/cmd_abs

# check the RC source and which channel is moving
ros2 topic echo /fmu/out/input_rc      # values[10] should track your stick

# drive the motor by hand (bypasses the relay; same topic)
ros2 topic pub -1 /u2d2/cmd_abs std_msgs/Float64MultiArray "{data: [700]}"  # retract
ros2 topic pub -1 /u2d2/cmd_abs std_msgs/Float64MultiArray "{data: [0]}"    # back to boot
```

## Notes

- This relay holds no serial port, so it runs alongside `u2d2_driver_ros`
  without contention. The driver node still publishes `/u2d2/joint_states`.
- Logical 0 is the position captured when the **driver** node starts; restart
  `u2d2_driver_ros` to re-zero. The relay itself is stateless (absolute targets).
