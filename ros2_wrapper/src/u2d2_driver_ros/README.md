# u2d2_driver_ros

ROS 2 (Humble) wrapper around [`u2d2_driver`](../../../README.md) — control
ROBOTIS U2D2 + Dynamixel X-series motors over topics, and read back joint
state. Built with [colcon-uv](https://github.com/atarbabgei/colcon-uv) so the
ROS package reuses the driver's `uv` environment directly.

Lives on the `ros` branch only; `main` stays a pure CLI/library.

## Build

```bash
# one-time: install the colcon-uv build tool
pip install git+https://github.com/atarbabgei/colcon-uv.git#subdirectory=colcon_uv --break-system-packages

# build the workspace
cd ros2_wrapper
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch u2d2_driver_ros u2d2.launch.py                 # ID 1 @ 57600, auto port
ros2 launch u2d2_driver_ros u2d2.launch.py ids:="[1,2]" baud:=0   # two motors, auto-scan baud
```

### Launch arguments

| Arg                   | Default | Meaning                                            |
|-----------------------|---------|----------------------------------------------------|
| `namespace`           | `u2d2`  | Topic prefix (`/u2d2/...`).                         |
| `port`                | `''`    | Serial port; empty = autodetect FTDI/ttyUSB.       |
| `baud`                | `57600` | Baud rate; `0` = scan common bauds and auto-detect.|
| `ids`                 | `[1]`   | Motor IDs, e.g. `ids:="[1,2]"`.                     |
| `profile_vel`         | `0.0`   | Position travel speed (rpm); `0` = max.            |
| `publish_rate`        | `20.0`  | `joint_states` rate (Hz); `0` disables telemetry.  |
| `release_on_shutdown` | `true`  | Release torque when the node stops.                |
| `log_level`           | `info`  | rclpy log level.                                   |

## Topics

Commands use `std_msgs/Float64MultiArray` in **degrees / rpm** (matching the
CLI). The array maps positionally to `ids`; a single value broadcasts to all
motors.

| Topic                  | Type                        | Effect                                        |
|------------------------|-----------------------------|-----------------------------------------------|
| `/u2d2/cmd_velocity`   | `Float64MultiArray` (rpm)   | Velocity mode; signed rpm per motor.          |
| `/u2d2/cmd_position`   | `Float64MultiArray` (deg)   | Position mode; absolute 0–360°, shortest path.|
| `/u2d2/cmd_turn`       | `Float64MultiArray` (deg)   | Relative multi-turn rotation (720 = 2 turns). |
| `/u2d2/cmd_abs`        | `Float64MultiArray` (deg)   | Absolute multi-turn from the boot origin (0 = startup position). Drift-free under streaming. |
| `/u2d2/joint_states`   | `sensor_msgs/JointState`    | Telemetry: position (rad), velocity (rad/s).  |

The node switches each motor's operating mode automatically and only when it
changes, so streaming velocity commands don't thrash torque.

### Examples

```bash
# spin ID 1 at 5 rpm
ros2 topic pub -1 /u2d2/cmd_velocity std_msgs/Float64MultiArray "{data: [5]}"

# go to 90°, hold
ros2 topic pub -1 /u2d2/cmd_position std_msgs/Float64MultiArray "{data: [90]}"

# rotate two full turns from current position
ros2 topic pub -1 /u2d2/cmd_turn std_msgs/Float64MultiArray "{data: [720]}"

# absolute: go to 700° from the boot position, then back to the boot position
ros2 topic pub -1 /u2d2/cmd_abs std_msgs/Float64MultiArray "{data: [700]}"
ros2 topic pub -1 /u2d2/cmd_abs std_msgs/Float64MultiArray "{data: [0]}"

# two motors: ID1 -10 rpm, ID2 +20 rpm
ros2 topic pub -1 /u2d2/cmd_velocity std_msgs/Float64MultiArray "{data: [-10, 20]}"

# watch telemetry
ros2 topic echo /u2d2/joint_states
```

## Notes

- `joint_states` uses SI units (rad, rad/s); command topics use deg/rpm.
- `cmd_abs` is referenced to the position captured at node startup (logical 0);
  restart the node to re-zero it where the motor currently sits. Prefer `cmd_abs`
  over `cmd_turn` for streamed setpoints — relative deltas accumulate on the live
  present position and drift, absolute targets do not.
- The node is single-threaded by design so the serial bus is only ever
  accessed by one callback at a time.
- Linux serial access: `sudo usermod -aG dialout $USER` (log out/in once).
