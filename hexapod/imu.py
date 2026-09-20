"""GY-521 (MPU-6050) over i2c: tilt and turn rate for the status horizon.

Talks to the chip with plain ioctl on /dev/i2c-N, the same way tools/preflight.py
does, so there is nothing to pip install. The gyro bias is averaged at startup
while the robot is known to be still and subtracted from every reading.

Frames. Readings come out in the body frame used everywhere else: +X right,
+Y forward, +Z up. `ImuConfig.axis_map` says how the chip is bolted on. Pitch is
positive nose up, roll is positive right side down; both come from the
accelerometer, so they are gravity-referenced and drift-free but noisy while
the legs are moving. The state poller smooths them.
"""

from __future__ import annotations

import logging
import math
import os
import struct
import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from .config import AXES, ImuConfig

log = logging.getLogger(__name__)

Vec3 = Tuple[float, float, float]

I2C_SLAVE = 0x0703
REG_PWR_MGMT_1 = 0x6B
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_WHO_AM_I = 0x75
WHO_AM_I = 0x68
ACCEL_LSB_PER_G = 16384.0  # +/-2 g, the default after reset
GYRO_LSB_PER_DPS = 131.0  # +/-250 deg/s, ditto
MAX_PLAUSIBLE_BIAS = 20.0  # deg/s; more than this means the robot was moving
MAX_CALIBRATION_MISSES = 20  # consecutive failed reads before calibration gives up


class ImuError(RuntimeError):
    pass


@dataclass(frozen=True)
class ImuReading:
    pitch: float  # deg, + nose up
    roll: float  # deg, + right side down
    gyro: Vec3  # deg/s about body X, Y, Z, bias removed
    accel: Vec3  # g, body frame
    temp_c: float
    at: float  # time.monotonic()


def remap(vector: Sequence[float], axis_map: Sequence[str]) -> Vec3:
    """Rotate a chip-frame vector into the body frame using the config's axis map."""
    out = []
    for entry in axis_map:
        sign = -1.0 if entry.startswith("-") else 1.0
        out.append(sign * vector[AXES.index(entry.lstrip("-"))])
    return out[0], out[1], out[2]


def tilt(accel: Vec3) -> Tuple[float, float]:
    """(pitch, roll) in degrees from a body-frame gravity vector."""
    ax, ay, az = accel
    pitch = math.degrees(math.atan2(ay, math.hypot(ax, az)))
    roll = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
    return pitch, roll


class Imu:
    def __init__(self, config: ImuConfig):
        self.config = config
        self._fd: Optional[int] = None
        self._bias: Vec3 = (0.0, 0.0, 0.0)
        self.bias_at: float = 0.0
        self.read_errors = 0  # bus errors since open; a rising count means a loose lead

    # --- lifecycle ----------------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._fd is not None

    @property
    def bias(self) -> Vec3:
        return self._bias

    def open(self) -> None:
        node = f"/dev/i2c-{self.config.bus}"
        try:
            import fcntl

            self._fd = os.open(node, os.O_RDWR)
            fcntl.ioctl(self._fd, I2C_SLAVE, self.config.address)
            who = self._read(REG_WHO_AM_I, 1)[0]
        except OSError as exc:
            self.close()
            raise ImuError(f"cannot open MPU-6050 at {node} {self.config.address:#04x}: {exc}") from exc
        if who != WHO_AM_I:
            self.close()
            raise ImuError(f"WHO_AM_I is {who:#04x}, expected 0x68; wrong chip or wrong address")
        self._write(REG_PWR_MGMT_1, 0x00)  # clear sleep, internal 8 MHz clock
        self._write(REG_GYRO_CONFIG, 0x00)  # +/-250 deg/s
        self._write(REG_ACCEL_CONFIG, 0x00)  # +/-2 g
        time.sleep(0.05)
        log.info("MPU-6050 up on %s at %#04x", node, self.config.address)

    def close(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None

    def __enter__(self) -> "Imu":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # --- calibration and reading ----------------------------------------------------

    def calibrate(self, seconds: Optional[float] = None) -> Vec3:
        """Average the gyro while still and use that as the bias from now on."""
        seconds = self.config.bias_seconds if seconds is None else seconds
        total = [0.0, 0.0, 0.0]
        count = failed = 0
        deadline = time.monotonic() + seconds
        while count == 0 or time.monotonic() < deadline:
            try:
                _, gyro, _ = self._raw()
            except OSError as exc:
                # A single NACK must not abort a two-second average, but a chip that
                # never answers is not calibratable.
                failed += 1
                if failed >= MAX_CALIBRATION_MISSES and count == 0:
                    raise ImuError(f"{failed} reads failed during calibration: {exc}") from exc
                time.sleep(0.01)
                continue
            for i in range(3):
                total[i] += gyro[i]
            count += 1
            time.sleep(0.01)
        if failed:
            log.warning("%d of %d reads failed during gyro calibration; check the IMU leads", failed, failed + count)
        bias = (total[0] / count, total[1] / count, total[2] / count)
        worst = max(abs(v) for v in bias)
        if worst > MAX_PLAUSIBLE_BIAS:
            log.warning("gyro bias %s deg/s is too large to be bias; was the robot moving? keeping the old one", bias)
            return self._bias
        self._bias = bias
        self.bias_at = time.monotonic()
        log.info("gyro bias %+.2f %+.2f %+.2f deg/s over %d samples", *bias, count)
        return bias

    def read(self) -> ImuReading:
        accel, gyro, temp = self._raw()
        pitch, roll = tilt(accel)
        corrected = (gyro[0] - self._bias[0], gyro[1] - self._bias[1], gyro[2] - self._bias[2])
        return ImuReading(pitch=pitch, roll=roll, gyro=corrected, accel=accel, temp_c=temp, at=time.monotonic())

    # --- chip access ------------------------------------------------------------------

    def _raw(self) -> Tuple[Vec3, Vec3, float]:
        """One burst read: accel (g), gyro (deg/s), temperature, all in the body frame."""
        raw = self._read(REG_ACCEL_XOUT_H, 14)
        ax, ay, az, temp, gx, gy, gz = struct.unpack(">hhhhhhh", raw)
        accel = remap((ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G), self.config.axis_map)
        gyro = remap((gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS), self.config.axis_map)
        return accel, gyro, temp / 340.0 + 36.53

    def _read(self, register: int, count: int) -> bytes:
        if self._fd is None:
            raise ImuError("imu is not open")
        try:
            os.write(self._fd, bytes((register,)))
            data = os.read(self._fd, count)
        except OSError:
            self.read_errors += 1
            raise
        if len(data) != count:
            raise ImuError(f"short read from register {register:#04x}: {len(data)} of {count} bytes")
        return data

    def _write(self, register: int, value: int) -> None:
        if self._fd is None:
            raise ImuError("imu is not open")
        os.write(self._fd, bytes((register, value)))


class FakeImu(Imu):
    """No chip. Rocks gently so the horizon on the status page visibly works."""

    def open(self) -> None:
        self._fd = -1
        self._t0 = time.monotonic()

    def close(self) -> None:
        self._fd = None

    def _raw(self) -> Tuple[Vec3, Vec3, float]:
        t = time.monotonic() - getattr(self, "_t0", 0.0)
        pitch = math.radians(4.0 * math.sin(t / 3.0))
        roll = math.radians(6.0 * math.sin(t / 5.0))
        ax, ay = -math.sin(roll), math.sin(pitch)
        az = math.sqrt(max(0.0, 1.0 - ax * ax - ay * ay))
        # A constant offset so the bias subtraction is visibly doing something.
        gyro = (0.8 + 2.0 * math.cos(t / 3.0), -1.2 + 2.0 * math.cos(t / 5.0), 0.3)
        return (ax, ay, az), gyro, 31.5
