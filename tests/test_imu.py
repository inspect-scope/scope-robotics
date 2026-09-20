import errno
import math
import struct

import pytest

from hexapod import config as config_mod
from hexapod.config import ImuConfig
from hexapod.imu import FakeImu, Imu, remap, tilt


class StillImu(Imu):
    """A chip that is level, with a constant gyro offset. No i2c."""

    gyro = (1.5, -0.5, 0.25)

    def open(self):
        self._fd = -1

    def close(self):
        self._fd = None

    def _raw(self):
        return (0.0, 0.0, 1.0), self.gyro, 30.0


def test_remap_follows_the_axis_map():
    assert remap((1.0, 2.0, 3.0), ("x", "y", "z")) == (1.0, 2.0, 3.0)
    assert remap((1.0, 2.0, 3.0), ("y", "-x", "z")) == (2.0, -1.0, 3.0)
    assert remap((1.0, 2.0, 3.0), ("-z", "y", "x")) == (-3.0, 2.0, 1.0)


def test_tilt_signs_match_the_body_frame():
    assert tilt((0.0, 0.0, 1.0)) == pytest.approx((0.0, 0.0))
    ten = math.radians(10.0)
    nose_up = (0.0, math.sin(ten), math.cos(ten))
    assert tilt(nose_up) == pytest.approx((10.0, 0.0), abs=1e-6)
    right_side_down = (-math.sin(ten), 0.0, math.cos(ten))
    assert tilt(right_side_down) == pytest.approx((0.0, 10.0), abs=1e-6)


def test_gyro_bias_is_averaged_then_subtracted():
    imu = StillImu(ImuConfig(bias_seconds=0.05))
    imu.open()
    assert imu.read().gyro == pytest.approx(StillImu.gyro)  # before calibration
    bias = imu.calibrate()
    assert bias == pytest.approx(StillImu.gyro, abs=1e-9)
    reading = imu.read()
    assert reading.gyro == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert (reading.pitch, reading.roll) == pytest.approx((0.0, 0.0))
    assert reading.temp_c == 30.0


def test_implausible_bias_is_rejected():
    class Moving(StillImu):
        gyro = (50.0, 0.0, 0.0)

    imu = Moving(ImuConfig(bias_seconds=0.02))
    imu.open()
    assert imu.calibrate() == (0.0, 0.0, 0.0)
    assert imu.read().gyro[0] == pytest.approx(50.0)


def test_fake_imu_rocks_gently():
    imu = FakeImu(ImuConfig(bias_seconds=0.02))
    imu.open()
    imu.calibrate()
    reading = imu.read()
    assert abs(reading.pitch) <= 4.5 and abs(reading.roll) <= 6.5
    assert math.hypot(*reading.accel) == pytest.approx(1.0, abs=1e-6)
    imu.close()
    assert not imu.is_open


@pytest.mark.parametrize("bad", [["x", "y"], ["x", "x", "z"], ["x", "y", "w"], ["--x", "y", "z"]])
def test_axis_map_is_validated(bad):
    with pytest.raises(ValueError):
        config_mod._axis_map(bad)


def test_config_defaults_apply_without_an_imu_block(tmp_path):
    import yaml

    raw = yaml.safe_load(open(config_mod.DEFAULT_CONFIG))
    raw.pop("imu")
    raw.pop("camera")
    path = tmp_path / "hexapod.yaml"
    path.write_text(yaml.safe_dump(raw))
    config = config_mod.load(str(path))
    assert config.imu == ImuConfig()
    assert config.camera.still == (4608, 2592)


class FlakyImu(StillImu):
    """Every third bus transaction NACKs, like a lead that is barely making contact."""

    def __init__(self, config):
        super().__init__(config)
        self.calls = 0

    def _raw(self):
        self.calls += 1
        if self.calls % 3 == 0:
            self.read_errors += 1
            raise OSError(121, "Remote I/O error")
        return super()._raw()


def test_calibration_survives_the_odd_nack():
    imu = FlakyImu(ImuConfig(bias_seconds=0.1))
    imu.open()
    assert imu.calibrate() == pytest.approx(StillImu.gyro, abs=1e-9)
    assert imu.read_errors > 0


def test_calibration_gives_up_on_a_chip_that_never_answers():
    from hexapod.imu import ImuError

    class Dead(StillImu):
        def _raw(self):
            raise OSError(121, "Remote I/O error")

    imu = Dead(ImuConfig(bias_seconds=5.0))
    imu.open()
    with pytest.raises(ImuError):
        imu.calibrate()


def nack() -> OSError:
    return OSError(121, "Remote I/O error")  # EREMOTEIO, what a NACK gives on the Pi


class BenchImu(Imu):
    """A chip whose two syscalls are scripted, so the retry layer is what is tested.

    `faults` holds one entry per read attempt: an OSError to raise, an int to cut
    the burst short at, or None for a clean read. Attempts past the end succeed.
    """

    level = struct.pack(">hhhhhhh", 0, 0, 16384, 0, 0, 0, 0)  # flat, still, 36.5 C

    def __init__(self, config, faults=(), write_faults=()):
        super().__init__(config)
        self.faults = list(faults)
        self.write_faults = list(write_faults)
        self.attempts = 0
        self.writes = []

    def open(self):
        self._fd = -1

    def close(self):
        self._fd = None

    def _bus_read(self, register, count):
        self.attempts += 1
        fault = self.faults.pop(0) if self.faults else None
        if isinstance(fault, BaseException):
            raise fault
        if isinstance(fault, int):
            return self.level[:fault]
        return self.level[:count]

    def _bus_write(self, register, value):
        fault = self.write_faults.pop(0) if self.write_faults else None
        if isinstance(fault, BaseException):
            raise fault
        self.writes.append((register, value))


def test_a_read_retries_through_errno_121():
    imu = BenchImu(ImuConfig(bus_retries=2), faults=[nack(), nack()])
    imu.open()
    reading = imu.read()
    assert imu.attempts == 3
    assert (reading.pitch, reading.roll) == pytest.approx((0.0, 0.0))
    assert imu.read_errors == 2 and imu.read_failures == 0


def test_a_read_gives_up_after_the_configured_retries():
    imu = BenchImu(ImuConfig(bus_retries=2), faults=[nack() for _ in range(4)])
    imu.open()
    with pytest.raises(OSError):
        imu.read()
    assert imu.attempts == 3
    assert imu.read_errors == 3 and imu.read_failures == 1


def test_zero_retries_means_one_attempt():
    imu = BenchImu(ImuConfig(bus_retries=0), faults=[nack()])
    imu.open()
    with pytest.raises(OSError):
        imu.read()
    assert imu.attempts == 1


def test_an_error_that_will_not_clear_is_not_retried():
    imu = BenchImu(ImuConfig(bus_retries=2), faults=[OSError(errno.ENODEV, "No such device")])
    imu.open()
    with pytest.raises(OSError):
        imu.read()
    assert imu.attempts == 1 and imu.read_failures == 1


def test_a_truncated_burst_is_retried():
    imu = BenchImu(ImuConfig(bus_retries=2), faults=[6])
    imu.open()
    assert imu.read().accel == pytest.approx((0.0, 0.0, 1.0))
    assert imu.attempts == 2 and imu.read_errors == 1 and imu.read_failures == 0


def test_register_writes_are_retried_too():
    imu = BenchImu(ImuConfig(bus_retries=2), write_faults=[nack()])
    imu.open()
    imu._write(0x6B, 0x00)
    assert imu.writes == [(0x6B, 0x00)]
    assert imu.read_errors == 1 and imu.read_failures == 0


def test_reads_on_a_closed_chip_do_not_touch_the_bus():
    from hexapod.imu import ImuError

    imu = BenchImu(ImuConfig())
    with pytest.raises(ImuError):
        imu.read()
    assert imu.attempts == 0


def test_calibration_rides_out_nacks_the_retries_could_not_clear():
    # Three NACKs in a row beats bus_retries=1, so the sample is lost, not the average.
    imu = BenchImu(ImuConfig(bias_seconds=0.05, bus_retries=1), faults=[nack(), nack(), nack()])
    imu.open()
    assert imu.calibrate() == pytest.approx((0.0, 0.0, 0.0))
    assert imu.read_failures == 1
