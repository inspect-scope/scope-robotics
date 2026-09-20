import time

import pytest

from hexapod import config as config_mod
from hexapod.board import FakeBoard
from hexapod.camera import Camera, FakeCamera
from hexapod.config import CameraConfig, ImuConfig
from hexapod.controller import Controller
from hexapod.imu import FakeImu, Imu, ImuError
from hexapod.state import RobotState


def _wait_for(predicate, seconds=3.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


@pytest.fixture()
def stack():
    config = config_mod.load()
    board = FakeBoard(config)
    board.open()
    controller = Controller(config, board)
    controller.start()
    yield config, controller
    controller.stop()
    board.close()


def test_snapshot_merges_board_imu_camera_and_network(stack, tmp_path):
    config, controller = stack
    state = RobotState(config, controller,
                       imu=FakeImu(ImuConfig(bias_seconds=0.05)),
                       camera=FakeCamera(CameraConfig(survey_dir=str(tmp_path))))
    state.start()
    try:
        assert _wait_for(lambda: state.snapshot()["imu"]["ok"])
        s = state.snapshot()
        assert s["state"] == "off"  # controller fields are still there
        assert isinstance(s["imu"]["pitch"], float) and isinstance(s["imu"]["roll"], float)
        assert len(s["imu"]["gyro"]) == 3
        assert s["imu"]["age_s"] >= 0
        assert s["camera"]["ok"] and s["camera"]["captures"] == 0
        assert "ip" in s and isinstance(s["addresses"], list)
        assert s["uptime_s"] >= 0
        assert s["last_error"] is None
    finally:
        state.stop()
    assert not state.camera.ok and not state.imu.is_open


def test_missing_imu_is_reported_not_fatal(stack):
    config, controller = stack

    class NoChip(Imu):
        def open(self):
            raise ImuError("cannot open /dev/i2c-1")

    state = RobotState(config, controller, imu=NoChip(config.imu))
    state.start()
    try:
        assert _wait_for(lambda: state.snapshot()["imu"]["error"] is not None)
        s = state.snapshot()
        assert not s["imu"]["ok"] and s["imu"]["pitch"] is None
        assert "i2c-1" in s["imu"]["error"]
        assert s["last_error"] == s["imu"]["error"]
    finally:
        state.stop()


def test_broken_camera_is_reported_not_fatal(stack):
    config, controller = stack

    class NoLens(Camera):
        def open(self):
            raise RuntimeError("no cameras available")

    state = RobotState(config, controller, camera=NoLens(config.camera))
    state.start()
    try:
        s = state.snapshot()
        assert not s["camera"]["ok"]
        assert "no cameras available" in s["camera"]["error"]
    finally:
        state.stop()


def test_without_sensors_snapshot_says_so(stack):
    config, controller = stack
    state = RobotState(config, controller)
    s = state.snapshot()
    assert s["imu"]["error"] == "no imu configured"
    assert s["camera"]["error"] == "no camera configured"


def test_flaky_imu_reads_are_ridden_out(stack):
    config, controller = stack

    class Flaky(FakeImu):
        calls = 0

        def _raw(self):
            self.calls += 1
            if self.calls % 3 == 0:
                self.read_errors += 1
                raise OSError(121, "Remote I/O error")
            return super()._raw()

    state = RobotState(config, controller, imu=Flaky(ImuConfig(bias_seconds=0.05)))
    state.start()
    try:
        assert _wait_for(lambda: state.snapshot()["imu"]["ok"])
        time.sleep(0.6)  # several misses happen in here
        s = state.snapshot()
        assert s["imu"]["ok"] and s["imu"]["error"] is None
        assert s["imu"]["read_errors"] > 0
        assert s["imu"]["read_failures"] == 0  # nothing for the status page to complain about
        assert state.imu.is_open  # never closed and reopened
    finally:
        state.stop()


def test_a_run_of_failed_reads_reopens_the_chip(stack):
    config, controller = stack

    class DiesAfterCalibration(FakeImu):
        def read(self):
            raise OSError(121, "Remote I/O error")

    state = RobotState(config, controller, imu=DiesAfterCalibration(ImuConfig(bias_seconds=0.05)))
    state.start()
    try:
        assert _wait_for(lambda: state.snapshot()["imu"]["error"] is not None, seconds=4.0)
        s = state.snapshot()
        assert not s["imu"]["ok"] and "Remote I/O error" in s["imu"]["error"]
        assert not state.imu.is_open
    finally:
        state.stop()
