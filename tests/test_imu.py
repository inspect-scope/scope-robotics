import math

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
