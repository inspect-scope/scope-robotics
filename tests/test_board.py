import time

import pytest

from hexapod import config as config_mod
from hexapod.board import BoardError, FakeBoard


@pytest.fixture()
def board():
    device = FakeBoard(config_mod.load())
    device.open()
    yield device
    device.close()


def _feed(board, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        board.set_frame([1500] * 18)
        time.sleep(0.02)


def test_torque_stays_on_while_frames_keep_arriving(board):
    board.set_torque(True)
    _feed(board, 0.6)
    assert board.torque_on


def test_watchdog_drops_torque_when_frames_stop(board):
    board.set_torque(True)
    _feed(board, 0.3)
    assert board.torque_on
    time.sleep(board.config.watchdog_ms / 1000.0 + 0.2)
    assert not board.torque_on


def test_estop_latches_until_cleared(board):
    board.set_torque(True)
    _feed(board, 0.2)
    board.estop()
    _feed(board, 0.2)
    assert not board.torque_on
    with pytest.raises(BoardError):
        board.set_torque(True)
    board.clear_estop()
    board.set_torque(True)
    _feed(board, 0.2)
    assert board.torque_on


def test_frame_must_have_one_pulse_per_servo(board):
    with pytest.raises(ValueError):
        board.set_frame([1500] * 17)
