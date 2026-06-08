# u2d2-driver

Velocity and position driver for a ROBOTIS **U2D2** driving **Dynamixel
X-series** servos over Protocol 2.0. Tested with the XL330-M288; other
X-series motors (XL430, XM430, …) share the same control table.

## Install

```bash
# 1. install uv (https://docs.astral.sh/uv/)
curl -LsSf https://astral.sh/uv/install.sh | sh

# 2. get the code and its deps (dynamixel-sdk + pyserial)
git clone https://github.com/atarbabgei/u2d2_driver.git
cd u2d2_driver
uv sync
```

**Linux only:** add yourself to the serial group once, then log out/in:
`sudo usermod -aG dialout $USER`. On macOS/Windows the FTDI driver is
built in / auto-installed — nothing to do.

## Usage

The port and baud are optional and flags may appear in any order. When `--baud`
is not given, the driver **scans the bus first**: it broadcast-pings at each
common baud rate (57600, 1000000, 115200, 2M, 3M, 4M) and locks onto the first
one where motors answer, so it works with both factory-fresh motors (57600)
and reconfigured ones (e.g. 1 Mbps) without any flags.

```bash
uv run python u2d2_driver.py --scan                        # probe all bauds, list motors per baud

uv run python u2d2_driver.py --id 1 --vel 5                # spin at 5 rpm (negative = reverse)
uv run python u2d2_driver.py --id 1 --vel 10 --duration 3  # spin 3 s, then stop
uv run python u2d2_driver.py --id 1 2 3 --vel 8            # same speed on several motors
uv run python u2d2_driver.py --id 1 2 --vel -10 20         # per-motor: ID1 -10 rpm, ID2 +20 rpm

uv run python u2d2_driver.py --id 1 --pos 90               # go to 90°, hold
uv run python u2d2_driver.py --id 1 2 --pos 0 90           # per-motor: ID1 → 0°, ID2 → 90°
uv run python u2d2_driver.py --id 1 --pos 270 --profile-vel 30  # travel at 30 rpm
uv run python u2d2_driver.py --id 1 --pos 0 --release      # move, then go limp

uv run python u2d2_driver.py --port COM4 --id 1 --vel 3    # explicit port (Windows)
uv run python u2d2_driver.py --id 1 --set-id 2             # change motor ID 1 → 2
```

### Options

| Flag            | Default     | Meaning                                          |
|-----------------|-------------|--------------------------------------------------|
| `--port`        | autodetect  | Serial port (`/dev/ttyUSB0`, `COM4`, …).         |
| `--baud`        | auto-scan   | Bus baud rate. Default: scan common bauds and auto-detect. Pass a value to pin it (skips the scan). |
| `--id`          | `1`         | One or more motor IDs (`--id 1 2 3`).            |
| `--vel`         | —           | Velocity mode: goal rpm (signed). One value for all motors, or one per `--id`. |
| `--pos`         | —           | Position mode: goal degrees (0–360). One value for all motors, or one per `--id`. |
| `--profile-vel` | max         | Position mode: travel speed in rpm.              |
| `--duration`    | run forever | Stop after N seconds.                            |
| `--release`     | hold        | Position mode: disable torque on exit.           |
| `--scan`        | —           | List motor IDs on the bus and exit. Without `--baud`, probes every common baud and reports which baud each motor answered on. |
| `--set-id`      | —           | Change a motor's ID (`--id` = its current ID).   |

`--vel` and `--pos` are mutually exclusive; the driver sets the motor's
operating mode automatically. Position moves always take the **shortest
path** to the target (e.g. 359° → 0° moves +1°, not −359°). On exit (including Ctrl+C) it always secures
the motor: velocity mode stops and releases torque, position mode holds the
target unless `--release` is passed.

### Assigning IDs (two motors on one bus)

Dynamixel motors ship as **ID 1**, so two fresh motors clash on the bus.
Program them one at a time:

1. Connect **only the second motor** (the first stays ID 1).
2. `uv run python u2d2_driver.py --id 1 --set-id 2`
3. Reconnect both → `--scan` should show `[1, 2]`.

## Licence
MIT