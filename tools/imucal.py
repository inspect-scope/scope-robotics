#!/usr/bin/env python3
"""Turn the robot through six positions and check the accelerometer on each axis.

    python3 tools/imucal.py              # six positions: axis_map, offsets, scale
    python3 tools/imucal.py --axes z     # upright and upside down only

In each position gravity lies along one body axis (+X right, +Y forward, +Z up),
so one chip axis reads about +1 g one way up and -1 g the other. Half the sum of
the two readings is that axis's offset; half the difference is its scale. Which
chip axis moved, and which way, is the `imu.axis_map` entry for that body axis.

An offset with a good scale means the chip works and reads shifted. A scale far
from 1 means the chip is faulty. Left and right are the robot's own: stand
behind it and look where the camera looks.

Stop the server first: it reads the same chip. Nothing here imports the hexapod
package, same as i2cwatch.py.
"""

import argparse
import fcntl
import math
import os
import struct
import subprocess
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

I2C_SLAVE = 0x0703
REG_PWR_MGMT_1 = 0x6B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_WHO_AM_I = 0x75
ACCEL_LSB_PER_G = 16384.0  # +/-2 g
CHIP_AXES = "xyz"
# MPU-6050 datasheet: zero-g tolerance +/-50 mg on x and y, +/-80 mg on z; sensitivity +/-3%.
OFFSET_LIMIT_G = {"x": 0.05, "y": 0.05, "z": 0.08}
SCALE_LIMIT = 0.03
TILT_LIMIT_DEG = 5.0  # axis_map only expresses 90 degree turns
STILL_STD_G = 0.03  # spread above this means the robot moved during the average

Vec3 = Tuple[float, float, float]

# (label, body axis pointing at the ceiling, what to do)
POSITIONS = {
    "z": (("+z", "upright, flat on the bench"),
          ("-z", "upside down, roof on the bench")),
    "y": (("+y", "nose up: camera end pointing at the ceiling"),
          ("-y", "nose down: camera end on the bench")),
    "x": (("+x", "lying on its left side, right side facing the ceiling"),
          ("-x", "lying on its right side, left side facing the ceiling")),
}


# --- analysis, pure ------------------------------------------------------------------


def analyse(readings: Dict[str, Vec3], axes: str) -> Dict[str, object]:
    """Mean chip readings per position ("+z", "-z", ...) -> per body axis results.

    Returns {"axes": {body axis: {...}}, "axis_map": [x, y, z entries] or None,
    "problems": [str]}. axis_map is only given when all three axes were measured
    and they land on three different chip axes.
    """
    out: Dict[str, Dict[str, object]] = {}
    problems: List[str] = []
    for body in axes:
        plus, minus = readings[f"+{body}"], readings[f"-{body}"]
        diff = [p - m for p, m in zip(plus, minus)]
        chip = max(range(3), key=lambda i: abs(diff[i]))
        sign = 1 if diff[chip] > 0 else -1
        letter = CHIP_AXES[chip]
        offset = (plus[chip] + minus[chip]) / 2.0
        scale = abs(diff[chip]) / 2.0
        others = [diff[i] for i in range(3) if i != chip]
        tilt = math.degrees(math.atan2(math.hypot(*others), abs(diff[chip])))
        row = {
            "entry": ("-" if sign < 0 else "") + letter,
            "chip": letter,
            "plus": plus[chip],
            "minus": minus[chip],
            "offset": offset,
            "scale": scale,
            "tilt_deg": tilt,
            "offset_ok": abs(offset) <= OFFSET_LIMIT_G[letter],
            "scale_ok": abs(scale - 1.0) <= SCALE_LIMIT,
        }
        out[body] = row
        if not row["offset_ok"]:
            problems.append(f"chip {letter} offset {offset:+.3f} g, datasheet allows "
                            f"+/-{OFFSET_LIMIT_G[letter]:.3f}")
        if not row["scale_ok"]:
            problems.append(f"chip {letter} scale {scale:.3f}, datasheet allows 1 +/-{SCALE_LIMIT:.2f}")
        if tilt > TILT_LIMIT_DEG:
            problems.append(f"body {body} is {tilt:.0f} deg off chip {letter}: board mounted at an angle, "
                            f"or the robot was not square to the bench")

    axis_map = None
    if sorted(axes) == sorted(CHIP_AXES):
        chips = [out[b]["chip"] for b in CHIP_AXES]
        if sorted(chips) == sorted(CHIP_AXES):
            axis_map = [out[b]["entry"] for b in CHIP_AXES]
        else:
            problems.append(f"body x, y, z landed on chip {', '.join(chips)}; each chip axis should appear once")
    return {"axes": out, "axis_map": axis_map, "problems": problems}


def report(result: Dict[str, object]) -> List[str]:
    lines = []
    for body, row in result["axes"].items():
        verdict = "ok" if row["offset_ok"] and row["scale_ok"] else "OUT OF SPEC"
        lines.append(f"  body {body}: chip {row['entry']:>2}   reads {row['plus']:+.3f} / {row['minus']:+.3f} g   "
                     f"offset {row['offset']:+.3f} g   scale {row['scale']:.3f}   {verdict}")
    if result["axis_map"]:
        lines.append(f"\n  imu.axis_map: [{', '.join(result['axis_map'])}]")
    for problem in result["problems"]:
        lines.append(f"  ! {problem}")
    bad_scale = any(not row["scale_ok"] for row in result["axes"].values())
    bad_offset = any(not row["offset_ok"] for row in result["axes"].values())
    if bad_scale:
        lines.append("\n  A scale this far from 1 is a faulty chip. Replace the GY-521.")
    elif bad_offset:
        lines.append("\n  Scale is good, so the chip works and reads shifted. Tilt from it is slightly off "
                     "until the offset is subtracted.")
    elif not result["problems"]:
        lines.append("\n  All measured axes inside the datasheet tolerance.")
    return lines


# --- chip access -----------------------------------------------------------------------


class Chip:
    def __init__(self, bus: int, address: int):
        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, address)
        who = self._read(REG_WHO_AM_I, 1)[0]
        if who != 0x68:
            raise OSError(f"WHO_AM_I is {who:#04x}, expected 0x68")
        self._write(REG_PWR_MGMT_1, 0x00)  # awake, internal clock
        self._write(REG_ACCEL_CONFIG, 0x00)  # +/-2 g, self-test off
        time.sleep(0.05)

    def _read(self, register: int, count: int) -> bytes:
        for attempt in range(3):
            try:
                os.write(self.fd, bytes((register,)))
                data = os.read(self.fd, count)
                if len(data) == count:
                    return data
            except OSError:
                if attempt == 2:
                    raise
            time.sleep(0.002)
        raise OSError(f"short reads from {register:#04x}")

    def _write(self, register: int, value: int) -> None:
        os.write(self.fd, bytes((register, value)))

    def accel(self) -> Vec3:
        ax, ay, az = struct.unpack(">hhh", self._read(REG_ACCEL_XOUT_H, 6))
        return ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G

    def close(self) -> None:
        os.close(self.fd)


def average(chip: Chip, seconds: float) -> Tuple[Vec3, float]:
    """Mean reading and the largest per-axis standard deviation, in g."""
    samples: List[Vec3] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        samples.append(chip.accel())
        time.sleep(0.01)
    n = len(samples)
    mean = tuple(sum(s[i] for s in samples) / n for i in range(3))
    spread = max(math.sqrt(sum((s[i] - mean[i]) ** 2 for s in samples) / n) for i in range(3))
    return mean, spread  # type: ignore[return-value]


def server_running() -> bool:
    try:
        return subprocess.run(["systemctl", "is-active", "--quiet", "hexapod"], check=False).returncode == 0
    except OSError:
        return False


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--axes", default="zyx", help="body axes to measure, e.g. z (default all three)")
    parser.add_argument("--seconds", type=float, default=2.0, help="average per position (default 2)")
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda s: int(s, 0), default=0x68)
    parser.add_argument("--force", action="store_true", help="run even if hexapod.service is up")
    args = parser.parse_args(argv)

    axes = "".join(a for a in args.axes.lower() if a in POSITIONS)
    if not axes or len(set(axes)) != len(axes):
        print("--axes takes x, y and z, each at most once")
        return 2
    if server_running() and not args.force:
        print("hexapod.service is running and reads the same chip. Stop it first:\n"
              "  sudo systemctl stop hexapod")
        return 1

    try:
        chip = Chip(args.bus, args.address)
    except OSError as exc:
        print(f"cannot talk to the MPU-6050 on bus {args.bus} at {args.address:#04x}: {exc}")
        return 1

    readings: Dict[str, Vec3] = {}
    try:
        for body in axes:
            for label, how in POSITIONS[body]:
                while True:
                    answer = input(f"\n{label}: put the robot {how}. Hold still, Enter to measure (q quits) ")
                    if answer.strip().lower() == "q":
                        return 1
                    time.sleep(0.5)  # let a hand leave the robot
                    mean, spread = average(chip, args.seconds)
                    shown = "  ".join(f"{a}{v:+.3f}" for a, v in zip(CHIP_AXES, mean))
                    if spread > STILL_STD_G:
                        print(f"  moved during the average (spread {spread:.3f} g); again")
                        continue
                    print(f"  {shown}   |a| {math.sqrt(sum(v * v for v in mean)):.3f} g")
                    readings[label] = mean
                    break
    except (KeyboardInterrupt, EOFError):
        print("\nstopped")
        return 1
    finally:
        chip.close()

    print("\nresult")
    for line in report(analyse(readings, axes)):
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
