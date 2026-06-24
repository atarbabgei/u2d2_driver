#!/usr/bin/env python3
"""Velocity + position driver for ROBOTIS U2D2 + Dynamixel X-series (Protocol 2.0).

Tested with the XL330-M288 and XC330-M288. See README.md for usage, or --help.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS, MAX_ID
from serial.tools import list_ports

# ---------------------------------------------------------------------------
# Dynamixel X-series control table (Protocol 2.0). Addresses are stable
# across the X-series. Sizes are in bytes.
# ---------------------------------------------------------------------------
ADDR_ID = 7                # 1 byte (EEPROM)
ADDR_OPERATING_MODE = 11   # 1 byte
ADDR_VELOCITY_LIMIT = 44   # 4 bytes (unit: 0.229 rpm)
ADDR_TORQUE_ENABLE = 64    # 1 byte
ADDR_PROFILE_VELOCITY = 112  # 4 bytes (unit: 0.229 rpm) -- travel speed in position mode
ADDR_GOAL_VELOCITY = 104   # 4 bytes, signed (unit: 0.229 rpm)
ADDR_GOAL_POSITION = 116   # 4 bytes, signed (unit: pulse, 4096/rev; multi-turn in extended mode)
ADDR_PRESENT_VELOCITY = 128  # 4 bytes, signed (unit: 0.229 rpm)
ADDR_PRESENT_POSITION = 132  # 4 bytes (unit: pulse)

OP_MODE_VELOCITY = 1       # Velocity Control Mode
OP_MODE_EXTENDED_POSITION = 4  # multi-turn position; lets moves cross the 0/360 seam
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

PROTOCOL_VERSION = 2.0
RPM_PER_UNIT = 0.229       # one Goal Velocity LSB = 0.229 rev/min
PULSES_PER_REV = 4096      # X-series encoder resolution (0.088 deg/pulse)

# U2D2 is an FTDI FT232 device.
U2D2_VID = 0x0403

# Baud rates probed when --baud is not given, most likely first.
COMMON_BAUDS = [57600, 1000000, 115200, 2000000, 3000000, 4000000]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def to_signed32(value: int) -> int:
    """Interpret an unsigned 32-bit register read as signed."""
    return value - (1 << 32) if value >= (1 << 31) else value


def rpm_to_raw(rpm: float) -> int:
    return int(round(rpm / RPM_PER_UNIT))


def raw_to_rpm(raw: int) -> float:
    return raw * RPM_PER_UNIT


def pulse_to_deg(pulse: int) -> float:
    return pulse / PULSES_PER_REV * 360.0


def ang_diff_deg(a: float, b: float) -> float:
    """Smallest angular distance between two angles, in degrees (0..180)."""
    d = abs(a - b) % 360.0
    return min(d, 360.0 - d)


def autodetect_port() -> str:
    """Return the first FTDI (U2D2) serial port, else first /dev/ttyUSB*."""
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid == U2D2_VID:
            return p.device
    for p in ports:
        if "ttyUSB" in p.device:
            return p.device
    raise RuntimeError(
        "No serial port found. Plug in the U2D2, or pass --port /dev/ttyUSBx."
    )


class Driver:
    def __init__(self, port: str, baud: int):
        self.port = PortHandler(port)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.active_ids: list[int] = []
        self.mode = OP_MODE_VELOCITY  # set by enable_*_mode()

        if not self.port.openPort():
            raise RuntimeError(f"Failed to open port {port}.")
        self.set_baud(baud)

    def set_baud(self, baud: int) -> None:
        if not self.port.setBaudRate(baud):
            raise RuntimeError(f"Failed to set baud rate {baud}.")

    # -- low-level checked writes/reads -----------------------------------
    def _check(self, dxl_id: int, comm: int, err: int, what: str) -> None:
        if comm != COMM_SUCCESS:
            raise RuntimeError(
                f"[ID {dxl_id}] {what}: {self.packet.getTxRxResult(comm)}"
            )
        if err != 0:
            raise RuntimeError(
                f"[ID {dxl_id}] {what}: {self.packet.getRxPacketError(err)}"
            )

    def _txrx(self, fn: Callable[[], tuple], retries: int = 3) -> tuple:
        """Run a *TxRx call, retrying and recovering the bus on failure.

        Handles two failure modes so a transient glitch can't wedge the bus:
        - Comm/packet error (a dropped status packet from USB jitter): flush the
          port and retry. Every register write here is idempotent, so re-issuing
          is safe.
        - Serial exception mid-transaction: the SDK sets ``port.is_using`` at the
          start of a transaction and only clears it on a clean return, so an
          exception leaves it stuck ``True`` -- after which every later call
          returns COMM_PORT_BUSY ("Port is in use!") until the port is reopened.
          We clear it before each attempt (safe: callers are single-threaded)
          so the bus self-heals instead of needing a restart.

        Raises RuntimeError if every attempt raised; otherwise returns the last
        result tuple for the caller's _check to interpret."""
        last = None
        for _ in range(retries):
            self.port.is_using = False  # clear any stuck busy flag from a prior fault
            try:
                last = fn()
            except Exception:
                self.port.clearPort()   # drop partial bytes, then retry
                last = None
                continue
            if last[-2] == COMM_SUCCESS and last[-1] == 0:
                return last
            self.port.clearPort()       # drop any stale/partial reply before retrying
        if last is None:
            raise RuntimeError("serial transaction failed: port error (bus reset)")
        return last

    def write1(self, dxl_id: int, addr: int, value: int, what: str) -> None:
        comm, err = self._txrx(
            lambda: self.packet.write1ByteTxRx(self.port, dxl_id, addr, value))
        self._check(dxl_id, comm, err, what)

    def write4(self, dxl_id: int, addr: int, value: int, what: str) -> None:
        comm, err = self._txrx(
            lambda: self.packet.write4ByteTxRx(self.port, dxl_id, addr, value))
        self._check(dxl_id, comm, err, what)

    def read1(self, dxl_id: int, addr: int, what: str) -> int:
        value, comm, err = self._txrx(
            lambda: self.packet.read1ByteTxRx(self.port, dxl_id, addr))
        self._check(dxl_id, comm, err, what)
        return value

    def read4(self, dxl_id: int, addr: int, what: str) -> int:
        value, comm, err = self._txrx(
            lambda: self.packet.read4ByteTxRx(self.port, dxl_id, addr))
        self._check(dxl_id, comm, err, what)
        return value

    def operating_mode(self, dxl_id: int) -> int:
        return self.read1(dxl_id, ADDR_OPERATING_MODE, "read operating mode")

    def torque_is_on(self, dxl_id: int) -> bool:
        return self.read1(dxl_id, ADDR_TORQUE_ENABLE,
                          "read torque enable") == TORQUE_ENABLE

    # -- discovery --------------------------------------------------------
    def ping(self, dxl_id: int, retries: int = 3) -> bool:
        """Ping a motor, retrying a few times. A single status packet is
        cheap to lose to USB scheduling jitter at low baud, so one dropped
        reply shouldn't read as 'motor absent' -- retry before giving up."""
        for _ in range(retries):
            _, comm, err = self.packet.ping(self.port, dxl_id)
            if comm == COMM_SUCCESS and err == 0:
                return True
        return False

    def scan(self) -> list[int]:
        """Broadcast-ping the bus and return the IDs that answer."""
        found, comm = self.packet.broadcastPing(self.port)
        if comm == COMM_SUCCESS and found is not None:
            return sorted(found.keys())
        return []

    def set_id(self, current_id: int, new_id: int) -> None:
        """Change a motor's bus ID (EEPROM). Address it by its current ID."""
        if not 1 <= new_id <= MAX_ID:
            raise ValueError(f"new ID must be 1-{MAX_ID}, got {new_id}")
        self.write1(current_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, "torque off")
        self.write1(current_id, ADDR_ID, new_id, "set ID")

    # -- motor setup ------------------------------------------------------
    def _set_mode(self, dxl_id: int, op_mode: int, what: str) -> None:
        """Operating Mode can only be changed while torque is disabled."""
        self.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, "torque off")
        self.write1(dxl_id, ADDR_OPERATING_MODE, op_mode, what)
        self.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, "torque on")
        if dxl_id not in self.active_ids:
            self.active_ids.append(dxl_id)
        self.mode = op_mode

    def enable_velocity_mode(self, dxl_id: int) -> None:
        self._set_mode(dxl_id, OP_MODE_VELOCITY, "set velocity mode")

    def enable_position_mode(self, dxl_id: int) -> None:
        """Extended Position Control Mode: multi-turn, so goals may cross the
        0/360 seam and moves can always take the shortest path."""
        self._set_mode(dxl_id, OP_MODE_EXTENDED_POSITION, "set position mode")

    def set_profile_velocity(self, dxl_id: int, rpm: float) -> None:
        """Cap travel speed in position mode (0 = max). Set after torque-on."""
        self.write4(dxl_id, ADDR_PROFILE_VELOCITY, rpm_to_raw(abs(rpm)),
                    "set profile velocity")

    def velocity_limit_rpm(self, dxl_id: int) -> float:
        return raw_to_rpm(self.read4(dxl_id, ADDR_VELOCITY_LIMIT, "read velocity limit"))

    def set_velocity(self, dxl_id: int, rpm: float) -> None:
        self.write4(dxl_id, ADDR_GOAL_VELOCITY, rpm_to_raw(rpm), "set goal velocity")

    def move_shortest_to(self, dxl_id: int, deg: float) -> float:
        """Command the nearest multi-turn equivalent of `deg` (mod 360), so the
        motor takes the shortest path. Requires extended position mode.
        Returns the absolute goal in degrees (unwrapped)."""
        present = to_signed32(self.read4(dxl_id, ADDR_PRESENT_POSITION,
                                         "read present position"))
        delta = (deg - pulse_to_deg(present)) % 360.0
        if delta > 180.0:
            delta -= 360.0
        goal = present + int(round(delta / 360.0 * PULSES_PER_REV))
        self.write4(dxl_id, ADDR_GOAL_POSITION, goal, "set goal position")
        return pulse_to_deg(goal)

    def turn_by(self, dxl_id: int, deg: float) -> float:
        """Rotate `deg` relative to the present position (negative = reverse),
        keeping the full magnitude -- e.g. 720 is two turns, not 0. Requires
        extended position mode. Returns the absolute goal in degrees."""
        present = to_signed32(self.read4(dxl_id, ADDR_PRESENT_POSITION,
                                         "read present position"))
        goal = present + int(round(deg / 360.0 * PULSES_PER_REV))
        self.write4(dxl_id, ADDR_GOAL_POSITION, goal, "set goal position")
        return pulse_to_deg(goal)

    def present_velocity_rpm(self, dxl_id: int) -> float:
        raw = to_signed32(self.read4(dxl_id, ADDR_PRESENT_VELOCITY, "read present velocity"))
        return raw_to_rpm(raw)

    def present_position_deg(self, dxl_id: int) -> float:
        raw = to_signed32(self.read4(dxl_id, ADDR_PRESENT_POSITION, "read present position"))
        return pulse_to_deg(raw)

    # -- shutdown ---------------------------------------------------------
    def stop_all(self, disable_torque: bool) -> None:
        """Zero the speed in velocity mode, then drop torque if asked.
        Best-effort: runs during shutdown, so errors only warn."""
        for dxl_id in self.active_ids:
            try:
                if self.mode == OP_MODE_VELOCITY:
                    self.set_velocity(dxl_id, 0.0)
                if disable_torque:
                    self.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, "torque off")
            except Exception as exc:
                print(f"[ID {dxl_id}] warning during stop: {exc}", file=sys.stderr)

    def close(self) -> None:
        self.port.closePort()


def fail(msg: str, code: int = 1) -> int:
    print(f"error: {msg}", file=sys.stderr)
    return code


def scan_all_bauds(driver: Driver, bauds: list[int]) -> dict[int, list[int]]:
    """Broadcast-ping at every baud; return {baud: ids} for bauds that answered."""
    results: dict[int, list[int]] = {}
    for baud in bauds:
        driver.set_baud(baud)
        ids = driver.scan()
        if ids:
            results[baud] = ids
    return results


def detect_baud(driver: Driver, bauds: list[int]) -> tuple[int | None, list[int]]:
    """Scan bauds in order and stop at the first one where motors answer.
    Leaves the port set to that baud. Returns (baud, ids) or (None, [])."""
    for baud in bauds:
        driver.set_baud(baud)
        ids = driver.scan()
        if ids:
            return baud, ids
    return None, []


def per_id(values: list[float], ids: list[int]) -> dict[int, float] | None:
    """Pair one goal value with each motor ID. A single value broadcasts to
    all IDs; otherwise there must be exactly one value per ID."""
    if len(values) == 1:
        return {i: values[0] for i in ids}
    if len(values) == len(ids):
        return dict(zip(ids, values))
    return None


def monitor(ids: list[int], sample: Callable[[int], float], fmt: str, label: str,
            duration: float | None,
            until: Callable[[dict[int, float]], bool] | None = None,
            max_read_failures: int = 6) -> None:
    """Poll `sample(id)` every 0.5 s and print the readings. Stops after
    `duration` seconds if given, else when `until(values)` is true (or
    never, if `until` is None).

    A dropped telemetry read is non-fatal: the motor keeps moving/holding
    regardless, so a failed poll only warns and retries on the next tick.
    Give up (raising) only after `max_read_failures` consecutive failed polls,
    which means the motor is likely unpowered or unplugged."""
    start = time.monotonic()
    failures = 0
    while True:
        time.sleep(0.5)
        try:
            values = {i: sample(i) for i in ids}
        except RuntimeError as exc:
            failures += 1
            print(f"warning: telemetry read failed "
                  f"({failures}/{max_read_failures}): {exc}", file=sys.stderr)
            if failures >= max_read_failures:
                raise RuntimeError(
                    f"lost contact with motor(s) after {max_read_failures} "
                    "consecutive read failures -- check power and wiring."
                ) from exc
            continue
        failures = 0
        readings = ", ".join(f"ID{i}={fmt.format(v)}" for i, v in values.items())
        print(f"present {label}: {readings}")
        if duration is not None:
            if time.monotonic() - start >= duration:
                return
        elif until is not None and until(values):
            print("target reached.")
            return


def run_set_id(driver: Driver, current_id: int, new_id: int) -> int:
    """Reprogram one motor's bus ID."""
    if current_id == new_id:
        return fail(f"motor is already ID {current_id}.", 2)
    if not driver.ping(current_id):
        return fail(f"motor ID {current_id} did not respond. "
                    f"Check power, baud, and wiring.")
    found = driver.scan()
    if len(found) > 1:
        print("warning: multiple distinct IDs on the bus. Make sure "
              f"only the motor you want to change is ID {current_id}.",
              file=sys.stderr)
    elif found == [current_id]:
        print("tip: if two motors are daisy-chained but only one ID "
              "shows up, they likely share the same ID — disconnect "
              "one servo, run --set-id again, then reconnect it.",
              file=sys.stderr)
    driver.set_id(current_id, new_id)
    if driver.ping(new_id):
        print(f"OK: ID {current_id} -> {new_id}")
        return 0
    return fail(f"wrote ID {new_id} but ping failed. Power-cycle "
                f"the motor and run --scan.")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Velocity/position driver for U2D2 + Dynamixel X-series."
    )
    p.add_argument("--port", default=None,
                   help="Serial port. Default: autodetect FTDI/ttyUSB.")
    p.add_argument("--baud", type=int, default=None,
                   help="Baud rate. Default: scan the bus first and auto-detect "
                        f"(probes {', '.join(str(b) for b in COMMON_BAUDS)}).")
    p.add_argument("--id", type=int, nargs="+", default=[1], metavar="ID",
                   help="Motor ID(s). Default: 1.")
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--vel", type=float, nargs="+", metavar="RPM",
                      help="Velocity mode: goal velocity in rpm (may be negative). "
                           "One value for all motors, or one per --id.")
    mode.add_argument("--pos", type=float, nargs="+", metavar="DEG",
                      help="Position mode: goal position in degrees (0..360); "
                           "moves take the shortest path. "
                           "One value for all motors, or one per --id.")
    mode.add_argument("--turn", type=float, nargs="+", metavar="DEG",
                      help="Position mode: rotate this many degrees relative to "
                           "the current position (negative = reverse), keeping "
                           "full magnitude (720 = two turns). "
                           "One value for all motors, or one per --id.")
    p.add_argument("--profile-vel", type=float, default=None, metavar="RPM",
                   help="Position mode: travel speed in rpm (0/unset = max).")
    p.add_argument("--duration", type=float, default=None,
                   help="Run for N seconds then stop. Default: run until Ctrl+C.")
    p.add_argument("--release", action="store_true",
                   help="Position mode: disable torque on exit instead of holding.")
    p.add_argument("--scan", action="store_true",
                   help="List the motor IDs on the bus and exit.")
    p.add_argument("--set-id", type=int, metavar="NEW_ID",
                   help="Change one motor's ID (pass current ID via --id). "
                        "If two motors share an ID, disconnect one first.")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    port = args.port or autodetect_port()
    driver = Driver(port, args.baud if args.baud is not None else COMMON_BAUDS[0])
    exit_code = 0
    try:
        if args.scan:
            if args.baud is not None:
                print(f"Port: {port} @ {args.baud} baud")
                ids = driver.scan()
                print(f"Motors found: {ids if ids else 'none'}")
                return 0 if ids else 1
            print(f"Port: {port} -- scanning bauds: "
                  f"{', '.join(str(b) for b in COMMON_BAUDS)}")
            results = scan_all_bauds(driver, COMMON_BAUDS)
            if not results:
                print("Motors found: none")
                return 1
            for baud, ids in results.items():
                print(f"  {baud} baud: {ids}")
            return 0

        # Scan first: find which baud the motors answer on, unless the
        # user pinned one with --baud.
        if args.baud is not None:
            print(f"Port: {port} @ {args.baud} baud")
        else:
            baud, ids_found = detect_baud(driver, COMMON_BAUDS)
            if baud is None:
                return fail("no motors answered at any common baud "
                            f"({', '.join(str(b) for b in COMMON_BAUDS)}). "
                            "Check power and wiring, or pass --baud.")
            print(f"Port: {port} @ {baud} baud (auto-detected, "
                  f"motors: {ids_found})")

        if args.set_id is not None:
            if len(args.id) != 1:
                return fail("--set-id needs exactly one --id (the motor's "
                            "current ID).", 2)
            return run_set_id(driver, args.id[0], args.set_id)

        if args.vel is None and args.pos is None and args.turn is None:
            return fail("pass --vel <rpm>, --pos <deg>, or --turn <deg>. "
                        "Use --scan to list motors.", 2)

        # Confirm each requested motor is present.
        for dxl_id in args.id:
            if not driver.ping(dxl_id):
                return fail(f"motor ID {dxl_id} did not respond. "
                            f"Check power, ID, and baud.")

        requested = (args.vel if args.vel is not None else
                     args.pos if args.pos is not None else args.turn)
        goals = per_id(requested, args.id)
        if goals is None:
            flag = ("--vel" if args.vel is not None else
                    "--pos" if args.pos is not None else "--turn")
            return fail(f"{flag} takes one value for all motors or one per "
                        f"--id ({len(args.id)} ids given).", 2)

        if args.vel is not None:
            # --- Velocity mode: apply speed, clamped to each motor's limit. ---
            for dxl_id, rpm in goals.items():
                driver.enable_velocity_mode(dxl_id)
                limit = driver.velocity_limit_rpm(dxl_id)
                target = rpm
                if abs(target) > limit:
                    print(f"[ID {dxl_id}] requested {target} rpm exceeds limit "
                          f"{limit:.2f} rpm; clamping.", file=sys.stderr)
                    target = limit if target > 0 else -limit
                driver.set_velocity(dxl_id, target)
                print(f"[ID {dxl_id}] goal velocity = {target} rpm (limit {limit:.2f})")

            monitor(args.id, driver.present_velocity_rpm, "{:+.2f}",
                    "velocity (rpm)", args.duration)
        elif args.pos is not None:
            # --- Position mode: shortest path to target degrees, hold. ---
            speed = f"{args.profile_vel} rpm" if args.profile_vel else "max"
            for dxl_id, deg in goals.items():
                driver.enable_position_mode(dxl_id)
                if args.profile_vel is not None:
                    driver.set_profile_velocity(dxl_id, args.profile_vel)
                driver.move_shortest_to(dxl_id, deg)
                print(f"[ID {dxl_id}] goal position = {deg} deg (travel {speed})")

            targets = {i: d % 360.0 for i, d in goals.items()}
            monitor(args.id, lambda i: driver.present_position_deg(i) % 360.0,
                    "{:.1f}", "position (deg)", args.duration,
                    until=lambda degs: all(ang_diff_deg(degs[i], t) < 1.0
                                           for i, t in targets.items()))
        else:
            # --- Position mode: relative multi-turn rotation, hold. The goal
            # may span many turns, so track the *unwrapped* present position
            # (not mod 360) against the absolute goal. ---
            speed = f"{args.profile_vel} rpm" if args.profile_vel else "max"
            targets: dict[int, float] = {}
            for dxl_id, deg in goals.items():
                driver.enable_position_mode(dxl_id)
                if args.profile_vel is not None:
                    driver.set_profile_velocity(dxl_id, args.profile_vel)
                targets[dxl_id] = driver.turn_by(dxl_id, deg)
                print(f"[ID {dxl_id}] turn {deg} deg -> goal "
                      f"{targets[dxl_id]:.1f} deg (travel {speed})")

            monitor(args.id, driver.present_position_deg,
                    "{:.1f}", "position (deg)", args.duration,
                    until=lambda degs: all(abs(degs[i] - t) < 1.0
                                           for i, t in targets.items()))

    except KeyboardInterrupt:
        print("\ninterrupted -- stopping motors.")
    except RuntimeError as exc:
        # A persistent bus failure (motor unplugged/unpowered mid-run) bubbles
        # up here. Report it cleanly instead of dumping a traceback.
        print(f"error: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        # Velocity mode always releases torque on exit; position moves (--pos
        # and --turn) hold the target unless --release was passed.
        driver.stop_all(disable_torque=args.vel is not None or args.release)
        driver.close()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
