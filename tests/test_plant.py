"""A stance foot with an open switch drops a little, then stops."""

import time

import pytest

from hexapod import config as config_mod
from hexapod.board import FakeBoard, Telemetry
from hexapod.controller import PLANT_DROP, PLANT_RATE, Controller
from hexapod.gait import Velocity


def _open():
    config = config_mod.load()
    board = FakeBoard(config)
    board.open()
    board.fake_telemetry_frozen = True
    board.set_frame([1500] * 18)
    board.set_torque(True)
    made = Controller(config, board)
    return made, board


def _contact(board, down):
    with board._lock:
        prev = board._telemetry
        contacts = {name: down for name in board.config.touch}
        board._sim_contacts = dict(contacts)
        board._telemetry = Telemetry(
            volts=prev.volts,
            amps=prev.amps,
            contacts=contacts,
            touch_volts=prev.touch_volts,
            updated_at=prev.updated_at,
        )


def _feet(controller):
    return controller.snapshot().feet


def test_an_open_switch_drops_the_foot_and_then_stops():
    controller, board = _open()
    try:
        controller._tick(0.05, time.monotonic())
        ground = -controller.snapshot().height
        _contact(board, False)
        controller._tick(0.05, time.monotonic())
        dropped = _feet(controller)["L2"][2]
        assert dropped == pytest.approx(ground - PLANT_RATE * 0.05, abs=0.2)
        for _ in range(20):
            _contact(board, False)
            controller._tick(0.05, time.monotonic())
        assert _feet(controller)["L2"][2] == pytest.approx(ground - PLANT_DROP, abs=0.2)
        for name, point in _feet(controller).items():
            assert point[2] >= ground - PLANT_DROP - 0.2, name
    finally:
        controller.estop()
        board.close()


def test_a_closed_switch_holds_the_drop():
    controller, board = _open()
    try:
        controller._tick(0.05, time.monotonic())
        _contact(board, False)
        controller._tick(0.05, time.monotonic())
        planted = _feet(controller)["L2"][2]
        _contact(board, True)
        controller._tick(0.05, time.monotonic())
        controller._tick(0.05, time.monotonic())
        assert _feet(controller)["L2"][2] == pytest.approx(planted, abs=0.2)
    finally:
        controller.estop()
        board.close()


def test_a_swing_foot_is_not_pulled_down():
    controller, board = _open()
    try:
        controller.stand()
        for _ in range(40):
            controller._tick(0.05, time.monotonic())
        seen = None
        for _ in range(16):
            controller.command("test", Velocity(1, 0, 0))
            _contact(board, False)
            controller._tick(0.05, time.monotonic())
            ground = -controller.snapshot().height
            zs = [point[2] for point in _feet(controller).values()]
            if max(zs) > ground + 5:
                seen = (ground, zs)
        assert seen is not None
        ground, zs = seen
        assert min(zs) >= ground - PLANT_DROP - 0.2
    finally:
        controller.estop()
        board.close()
