#!/usr/bin/env python3
"""One command that checks everything the robot needs before it moves.

    python3 tools/preflight.py                     # check everything
    python3 tools/preflight.py --skip-board        # no Servo2040 attached
    python3 tools/preflight.py --imu-seconds 10    # longer gyro bias average

Exit code is 0 if nothing failed, 1 otherwise. Warnings do not fail the run.

Each check is independent and catches its own errors, so an unplugged board or
a missing IMU reports as one bad line rather than a traceback. Run it on the Pi.
"""

from __future__ import annotations

import argparse
import fcntl
import glob
import math
import os
import struct
import sys
import time
from typing import List, Optional, Tuple

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PASS, WARN, FAIL, SKIP = "PASS", "WARN", "FAIL", "SKIP"
_TALLY = {PASS: 0, WARN: 0, FAIL: 0, SKIP: 0}


def report(state: str, message: str) -> None:
    _TALLY[state] += 1
    print(f"  [{state}] {message}")


def section(title: str) -> None:
    print(f"\n{title}")


# --- environment ---------------------------------------------------------------------


def check_python() -> None:
    version = ".".join(str(n) for n in sys.version_info[:3])
    report(PASS, f"python {version} at {sys.executable}")
    try:
        import serial

        report(PASS, f"pyserial {serial.__version__}")
    except Exception as exc:
        report(FAIL, f"pyserial not importable: {exc}")


def check_zero_filled() -> None:
    """An unclean shutdown can leave a .py file the right size and all zeros.

    It imports as a SyntaxError about null bytes, which looks nothing like the
    disk fault it is. Cheap to check, so check it.
    """
    roots = [os.path.join(REPO, part) for part in ("hexapod", "tools", "tests", "config")]
    roots += glob.glob(os.path.join(REPO, ".venv", "lib", "python3*", "site-packages"))
    scanned, bad = 0, []
    for root in roots:
        for folder, dirs, files in os.walk(root):
            dirs[:] = [d for d in dirs if d != "__pycache__"]
            for name in files:
                if not name.endswith((".py", ".yaml", ".html")):
                    continue
                path = os.path.join(folder, name)
                scanned += 1
                try:
                    with open(path, "rb") as handle:
                        blob = handle.read()
                except OSError as exc:
                    bad.append(f"{path} unreadable: {exc}")
                    continue
                if blob and blob.count(0) == len(blob):
                    bad.append(f"{path} is {len(blob)} bytes of nulls")
    if bad:
        for line in bad:
            report(FAIL, line)
    else:
        report(PASS, f"no zero-filled or unreadable files ({scanned} scanned)")


# --- config and maths ----------------------------------------------------------------


def check_config(path: Optional[str]):
    """Loads the YAML through the real loader, so its validation runs too."""
    sys.path.insert(0, REPO)
    try:
        from hexapod import config as config_mod
    except Exception as exc:
        report(FAIL, f"cannot import hexapod package: {exc}")
        return None
    try:
        config = config_mod.load(path or config_mod.DEFAULT_CONFIG)
    except Exception as exc:
        report(FAIL, f"config rejected: {exc}")
        return None
    channels = sum(len(leg.servos) for leg in config.legs.values())
    report(PASS, f"config loads: {len(config.legs)} legs, {channels} servo channels, "
                 f"{len(config.touch)} touch sensors")

    worst = []
    for joint in config_mod.JOINTS:
        low, high = config.limits.joint_range(joint)
        attach = config.geometry.attach_angle(joint)
        for name, leg in config.legs.items():
            cal = leg.servos[joint]
            for angle in (low, high):
                pulse = cal.pulse_for(cal.direction * (angle - attach))
                if not config.limits.pulse_us[0] <= pulse <= config.limits.pulse_us[1]:
                    worst.append(f"{name}.{joint} at {angle:+.0f}deg wants {pulse:.0f}us")
    if worst:
        report(FAIL, f"joint limits reach outside pulse_us: {worst[0]} (+{len(worst) - 1} more)")
    else:
        report(PASS, f"joint limits sit inside pulse_us {config.limits.pulse_us}")
    return config


def check_kinematics(config) -> None:
    """Walks the gait at ride height and counts frames the IK had to clamp."""
    try:
        from hexapod.controller import POSE_LIMITS
        from hexapod.gait import TripodGait, Velocity
        from hexapod.kinematics import BodyPose, HexapodKinematics
    except Exception as exc:
        report(FAIL, f"cannot import kinematics: {exc}")
        return

    kinematics = HexapodKinematics(config)
    height = config.stance.ride_height
    shift = POSE_LIMITS["shift"]
    poses = {"flat": BodyPose(), "shift": BodyPose(x=shift, y=shift, yaw=POSE_LIMITS["yaw"])}
    # Same command set as `hexapod check`. One command is not enough: the worst
    # clamping at ride height comes from translating and turning at once.
    commands = [Velocity(1, 0, 0), Velocity(0, 1, 0), Velocity(0, -1, 0), Velocity(-1, 0, 0),
                Velocity(0, 0, 1), Velocity(0, 0, -1), Velocity(0.7, 0.7, 1)]
    for label, pose in poses.items():
        worst = 0.0
        for command in commands:
            gait = TripodGait(config)
            scaled = command.scaled(config)
            hits = total = 0
            for _ in range(int(3 * config.rate_hz)):
                feet = gait.step(1 / config.rate_hz, scaled, height)
                _, limited = kinematics.solve_reporting(feet, pose)
                hits += 1 if limited else 0
                total += 1
            worst = max(worst, 100 * hits / total)
        note = (f"{label} at {height:.0f}mm: worst {worst:.0f}% of frames clamped "
                f"across {len(commands)} commands")
        report(PASS if worst == 0 else WARN, note)
    report(SKIP, "run `hexapod check` for the full height sweep")


# --- IMU: GY-521 / MPU-6050 over i2c -------------------------------------------------
# Straight ioctl so this needs no smbus2. The MPU-6050 accepts a register write
# followed by a read on the same open handle.

I2C_SLAVE = 0x0703
REG_PWR_MGMT_1 = 0x6B
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_WHO_AM_I = 0x75
ACCEL_LSB_PER_G = 16384.0  # +/-2 g, the default after reset
GYRO_LSB_PER_DPS = 131.0  # +/-250 deg/s, ditto


class Imu:
    def __init__(self, bus: int, address: int):
        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, address)

    def close(self) -> None:
        os.close(self.fd)

    def read(self, register: int, count: int) -> bytes:
        os.write(self.fd, bytes((register,)))
        return os.read(self.fd, count)

    def write(self, register: int, value: int) -> None:
        os.write(self.fd, bytes((register, value)))

    def wake(self) -> None:
        self.write(REG_PWR_MGMT_1, 0x00)  # clear sleep, internal 8MHz clock
        self.write(REG_GYRO_CONFIG, 0x00)  # +/-250 deg/s
        self.write(REG_ACCEL_CONFIG, 0x00)  # +/-2 g
        time.sleep(0.05)

    def sample(self) -> Tuple[List[float], List[float], float]:
        raw = self.read(REG_ACCEL_XOUT_H, 14)
        ax, ay, az, temp, gx, gy, gz = struct.unpack(">hhhhhhh", raw)
        accel = [v / ACCEL_LSB_PER_G for v in (ax, ay, az)]
        gyro = [v / GYRO_LSB_PER_DPS for v in (gx, gy, gz)]
        return accel, gyro, temp / 340.0 + 36.53


def check_imu(bus: int, address: int, seconds: float) -> None:
    node = f"/dev/i2c-{bus}"
    if not os.path.exists(node):
        report(FAIL, f"{node} missing. Enable i2c with `sudo raspi-config nonint do_i2c 0` and reboot")
        return
    if not os.access(node, os.R_OK | os.W_OK):
        report(FAIL, f"{node} not writable. Add yourself to the i2c group: sudo adduser $USER i2c")
        return
    report(PASS, f"{node} present and writable")

    try:
        imu = Imu(bus, address)
    except OSError as exc:
        report(FAIL, f"cannot open {node} at {address:#04x}: {exc}")
        return
    try:
        who = imu.read(REG_WHO_AM_I, 1)[0]
        if who != 0x68:
            report(FAIL, f"WHO_AM_I is {who:#04x}, expected 0x68. Wrong chip or wrong address")
            return
        report(PASS, f"MPU-6050 answers at {address:#04x}, WHO_AM_I 0x68")

        imu.wake()
        accel_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]
        temps: List[float] = []
        count = 0
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            accel, gyro, temp = imu.sample()
            for i in range(3):
                accel_sum[i] += accel[i]
                gyro_sum[i] += gyro[i]
            temps.append(temp)
            count += 1
            time.sleep(0.01)
        if not count:
            report(FAIL, "no samples read")
            return

        accel_avg = [v / count for v in accel_sum]
        gyro_avg = [v / count for v in gyro_sum]
        magnitude = math.sqrt(sum(v * v for v in accel_avg))

        axes = "  ".join(f"{n}{v:+.3f}" for n, v in zip("xyz", accel_avg))
        if magnitude == 0.0:
            report(FAIL, "accelerometer reads exactly zero on every axis")
        elif abs(magnitude - 1.0) <= 0.10:
            report(PASS, f"|a| = {magnitude:.3f} g   ({axes})")
        else:
            report(FAIL, f"|a| = {magnitude:.3f} g, expected 1.00 +/-0.10   ({axes})")

        bias = "  ".join(f"{v:+.2f}" for v in gyro_avg)
        worst = max(abs(v) for v in gyro_avg)
        detail = f"gyro bias [{bias}] deg/s over {seconds:.0f}s at {sum(temps) / len(temps):.1f}C"
        if worst > 20.0:
            report(FAIL, f"{detail}. Too large to be bias; was the board moving?")
        elif worst > 10.0:
            report(WARN, f"{detail}. High for a stationary board")
        else:
            report(PASS, detail)
        report(WARN, "gyro bias drifts with temperature. Re-measure warm, and after the board is bolted down")
    finally:
        imu.close()


# --- Servo2040 -----------------------------------------------------------------------


def check_board(config, port_override: Optional[str]) -> None:
    try:
        import serial

        from hexapod import protocol
    except Exception as exc:
        report(SKIP, f"board check needs pyserial and the hexapod package: {exc}")
        return

    port_path = port_override or (config.port if config else None)
    if not port_path:
        report(SKIP, "no serial port configured")
        return

    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    if not os.path.exists(port_path):
        hint = f" Found instead: {', '.join(by_id)}" if by_id else " Nothing in /dev/serial/by-id/"
        report(FAIL, f"{port_path} does not exist.{hint}")
        return
    report(PASS, f"{port_path} present")
    if by_id and not port_path.startswith("/dev/serial/by-id/"):
        report(WARN, f"{port_path} can move between reboots. Prefer {by_id[0]}")

    try:
        with serial.Serial(port_path, config.baudrate if config else 115200, timeout=0.2) as link:
            time.sleep(0.3)  # the firmware waits for the CDC connection before it parses
            values = _get(link, protocol, protocol.CH_CURRENT, 2)
            if values is None:
                report(FAIL, "no reply to GET current/voltage. Is this the driver firmware, "
                             "not servoCalibration.uf2?")
                return
            report(PASS, "board replies to GET, protocol codec round-trips")

            amps = protocol.counts_to_amps(values[0])
            volts = protocol.counts_to_volts(values[1])
            if volts < 1.0:
                report(WARN, f"rail {volts:.2f} V, {amps:+.2f} A. Servo rail unpowered. Expected "
                             f"with the USB/Ext power trace cut and no battery on the terminals. "
                             f"Servos cannot move like this")
            elif 4.5 <= volts <= 17.0:
                report(PASS, f"rail {volts:.2f} V, {amps:+.2f} A")
            else:
                report(WARN, f"rail {volts:.2f} V is neither dead nor a sane pack voltage. "
                             f"Check _B1024_3V3_RATIO against a multimeter")

            touch = _get(link, protocol, protocol.TOUCH_BASE, protocol.NUM_TOUCH)
            if touch is None:
                report(WARN, "no reply to GET touch pins")
            else:
                volts_each = [protocol.counts_to_sensor_volts(v) for v in touch]
                threshold = config.touch_threshold_v if config else 1.6
                asserted = [i + 1 for i, v in enumerate(volts_each) if v > threshold]
                pins = " ".join(f"{v:.2f}" for v in volts_each)
                note = f"touch pins [{pins}] V, asserted: {asserted or 'none'}"
                report(PASS if len(volts_each) == protocol.NUM_TOUCH else WARN, note)
    except serial.SerialException as exc:
        report(FAIL, f"cannot open {port_path}: {exc}")


def _get(link, protocol, start: int, count: int, timeout: float = 1.0) -> Optional[List[int]]:
    link.reset_input_buffer()
    link.write(protocol.encode_get(start, count))
    buffer = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        buffer += link.read(link.in_waiting or 1)
        replies, buffer = protocol.decode_replies(buffer)
        for reply in replies:
            if reply.start_idx == start and len(reply.values) == count:
                return reply.values
    return None


# --- main ----------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", help="path to hexapod.yaml")
    parser.add_argument("--port", help="serial port, overriding the config")
    parser.add_argument("--imu-bus", type=int, default=1, help="i2c bus number (default 1)")
    parser.add_argument("--imu-address", type=lambda s: int(s, 0), default=0x68,
                        help="MPU-6050 address, 0x68 or 0x69 (default 0x68)")
    parser.add_argument("--imu-seconds", type=float, default=3.0,
                        help="how long to average the gyro bias (default 3)")
    parser.add_argument("--skip-imu", action="store_true")
    parser.add_argument("--skip-board", action="store_true")
    args = parser.parse_args()

    print(f"hexapod preflight on {os.uname().nodename}, {time.strftime('%Y-%m-%d %H:%M:%S')}")

    section("environment")
    check_python()
    check_zero_filled()

    section("config and maths")
    config = check_config(args.config)
    if config:
        check_kinematics(config)

    section(f"imu (GY-521 / MPU-6050, bus {args.imu_bus}, {args.imu_address:#04x})")
    if args.skip_imu:
        report(SKIP, "skipped")
    else:
        print("  hold still, averaging the gyro")
        check_imu(args.imu_bus, args.imu_address, args.imu_seconds)

    section("servo2040")
    if args.skip_board:
        report(SKIP, "skipped")
    else:
        check_board(config, args.port)

    print(f"\n{_TALLY[PASS]} passed, {_TALLY[WARN]} warnings, "
          f"{_TALLY[FAIL]} failed, {_TALLY[SKIP]} skipped")
    return 1 if _TALLY[FAIL] else 0


if __name__ == "__main__":
    sys.exit(main())
