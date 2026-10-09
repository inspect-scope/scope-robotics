"""The control loop pulls the estop on sustained overcurrent or undervoltage."""

import time

import pytest

from hexapod import config as config_mod
from hexapod.board import FakeBoard
from hexapod.controller import Controller


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
    made = {}

    def build(amps=1.2, volts=7.4):
        board.fake_amps_on, board.fake_volts = amps, volts
        board.open()
        controller = Controller(config, board)
        controller.start()
        made["c"], made["b"] = controller, board
        return controller, board

    yield build
    if made:
        made["c"].stop(); made["b"].close()


def test_overcurrent_trips_the_estop(stack):
    controller, board = stack(amps=14.0)
    controller.stand()
    assert _wait_for(lambda: controller.snapshot().estopped)
    s = controller.snapshot()
    assert s.safety_trip and s.safety_trip.startswith("overcurrent: 14.0 A")
    # torque_on is what the board's IO thread last applied; it follows the estop
    # by one board tick (20 ms), so wait for it rather than read it instantly.
    assert _wait_for(lambda: not board.torque_on, seconds=1.0)


def test_current_under_the_cut_does_not_trip(stack):
    controller, _ = stack(amps=3.5)
    controller.stand()
    time.sleep(1.8)  # longer than current_cut_s and still_s
    s = controller.snapshot()
    assert not s.estopped and s.safety_trip is None


def test_low_battery_trips_after_its_own_delay(stack):
    controller, _ = stack(volts=5.8)
    controller.stand()
    time.sleep(1.0)
    assert not controller.snapshot().estopped  # volts_cut_s is 2 s
    assert _wait_for(lambda: controller.snapshot().estopped, seconds=3.0)
    assert "battery 5.80 V" in controller.snapshot().safety_trip


def test_clear_estop_clears_the_trip(stack):
    controller, board = stack(amps=14.0)
    controller.stand()
    assert _wait_for(lambda: controller.snapshot().estopped)
    board.fake_amps_on = 1.2
    controller.clear_estop()
    assert controller.snapshot().safety_trip is None


def test_trips_are_off_when_torque_is_off(stack):
    # FakeBoard masks current while torque is off, so drive voltage, which it does
    # not mask. On the robot, a pack unplugged with the board on USB reads ~0 V.
    controller, _ = stack(volts=5.8)
    time.sleep(2.6)
    assert not controller.snapshot().estopped


def test_config_rejects_nonsense(tmp_path):
    import yaml
    raw = yaml.safe_load(open(config_mod.DEFAULT_CONFIG))
    raw["safety"] = {"volts_warn": 5.0, "volts_cut": 6.0}
    path = tmp_path / "hexapod.yaml"; path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match="volts_warn"):
        config_mod.load(str(path))
    for bad in ({"current_cut_a": 0}, {"volts_cut_s": 0}, {"current_cut_s": -1}, {"sit_cut_a": 0}):
        raw["safety"] = bad
        path.write_text(yaml.safe_dump(raw))
        with pytest.raises(ValueError, match="positive"):
            config_mod.load(str(path))
    raw["safety"] = {"sit_cut_a": 6, "stand_cut_a": 1.5}
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match="nondecreasing"):
        config_mod.load(str(path))


def test_tolerant_open_survives_a_missing_port():
    config = config_mod.load()
    from dataclasses import replace
    from hexapod.board import Servo2040
    board = Servo2040(replace(config, port="/dev/does-not-exist-hexapod"))
    board.open_tolerant(retry_s=60.0)  # must not raise
    try:
        assert not board.connected
        assert board.error and "does-not-exist" in board.error
        assert board._retry is not None  # a retry is armed
    finally:
        board.close()
        assert board._retry is None  # and close cancelled it


def test_current_swinging_around_the_cut_still_trips(stack):
    # Alternating 12 and 9.5 A averages 10.75. The old every-sample-over timer
    # reset on each 9.5 and never tripped.
    readings = iter([12.0, 9.5] * 200)
    controller, _ = stack(amps=lambda: next(readings))
    controller.stand()
    assert _wait_for(lambda: controller.snapshot().estopped, seconds=3.0)
    assert controller.snapshot().safety_trip.startswith("overcurrent")


def test_one_spike_does_not_trip(stack):
    controller, board = stack()
    controller.stand()
    assert _wait_for(lambda: board.torque_on)
    readings = iter([15.0] + [1.2] * 400)    # armed after torque is on, so the spike is judged
    board.fake_amps_on = lambda: next(readings)
    time.sleep(1.8)
    assert not controller.snapshot().estopped


def test_lost_telemetry_trips(stack):
    controller, board = stack()
    controller.stand()
    assert _wait_for(lambda: board.torque_on)
    board.fake_telemetry_frozen = True
    assert _wait_for(lambda: controller.snapshot().estopped, seconds=4.0)
    assert "no telemetry" in controller.snapshot().safety_trip


def test_settled_sit_current_catches_one_stall(stack):
    controller, board = stack(amps=4.0)
    board.set_torque(True)
    assert _wait_for(lambda: controller.snapshot().estopped, seconds=3.0)
    assert "settled sit current" in controller.snapshot().safety_trip


def test_settled_stand_current_catches_one_stall(stack):
    controller, _ = stack(amps=7.0)
    controller.stand()
    assert _wait_for(lambda: controller.snapshot().estopped, seconds=3.0)
    trip = controller.snapshot().safety_trip
    assert trip and trip.startswith("settled stand current")
    assert "7.0 A" in trip


def test_a_still_window_longer_than_the_current_window_still_trips():
    """Both current checks read one queue of samples. The shorter window must
    not throw away samples the longer one needs."""
    from dataclasses import replace

    config = config_mod.load()
    config = replace(config, safety=replace(config.safety, current_cut_s=1.0, still_s=2.0))
    board = FakeBoard(config)
    board.fake_amps_on = 3.0  # one stall at a held sit: over sit_cut_a, under current_cut_a
    board.open()
    try:
        board.set_torque(True)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and not board.safety_trip:
            board.set_frame([1500] * 18)
            time.sleep(0.05)
        assert board.safety_trip and board.safety_trip.startswith("settled sit current")
    finally:
        board.close()


def test_moving_skips_the_stand_cut(stack):
    from hexapod.gait import Velocity

    controller, _ = stack(amps=6.5)
    controller.stand()
    assert _wait_for(lambda: controller.snapshot().state == "standing")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        controller.command("test", Velocity(0, 1, 0), priority=10)
        assert not controller.snapshot().estopped
        time.sleep(0.05)
    assert _wait_for(lambda: controller.snapshot().estopped, seconds=3.0)
    assert "settled stand current" in controller.snapshot().safety_trip


def test_the_trip_covers_jog_too():
    """jog and neutral drive the board with no Controller. The trip lives on the
    board's IO thread, so they are covered."""
    config = config_mod.load()
    board = FakeBoard(config)
    board.fake_amps_on = 14.0
    board.open()
    try:
        frame = [1500] * 18
        board.set_frame(frame)
        board.set_torque(True)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not board.safety_trip:
            board.set_frame(frame)   # what jog's pump thread does
            time.sleep(0.05)
        assert board.safety_trip and board.safety_trip.startswith("overcurrent")
        assert _wait_for(lambda: not board.torque_on, seconds=1.0)
        board.clear_estop()
        assert board.safety_trip is None
    finally:
        board.close()


def test_controller_resets_its_pose_on_a_board_trip(stack):
    controller, board = stack()
    controller.stand()
    assert _wait_for(
        lambda: controller.snapshot().height > controller.config.stance.ride_height - 1, seconds=3.0
    )
    board.fake_amps_on = 14.0
    assert _wait_for(lambda: controller.snapshot().estopped)
    assert _wait_for(lambda: controller.snapshot().height < controller.config.stance.ride_height - 1, seconds=3.0)
    assert controller._height_target == controller.config.stance.sit_height
    assert not controller._standing
    assert _wait_for(lambda: not board.torque_on, seconds=1.0)   # the IO thread applies it next tick
    board.fake_amps_on = 1.2
    controller.clear_estop()
    assert controller.snapshot().safety_trip is None
    time.sleep(0.2)
    assert not board.torque_on, "clearing the trip must not turn torque back on by itself"


def test_torque_is_refused_while_the_board_is_offline():
    from dataclasses import replace
    from hexapod.board import BoardError, Servo2040
    board = Servo2040(replace(config_mod.load(), port="/dev/does-not-exist-hexapod"))
    board.open_tolerant(retry_s=60.0)
    try:
        with pytest.raises(BoardError, match="offline"):
            board.set_torque(True)
        board.set_torque(False)   # turning it off is always allowed
    finally:
        board.close()


def test_a_torque_request_is_not_replayed_when_the_port_opens(monkeypatch):
    """A Stand pressed while the port was held by poke.py must not fire on reconnect."""
    import serial as pyserial
    from hexapod.board import Servo2040

    class DummySerial:
        in_waiting = 0
        def __init__(self, *a, **k): self.written = []
        def write(self, data): self.written.append(bytes(data))
        def read(self, n): return b""
        def flush(self): pass
        def close(self): pass

    made = []
    monkeypatch.setattr(pyserial, "Serial", lambda *a, **k: made.append(DummySerial()) or made[-1])
    board = Servo2040(config_mod.load())
    board._torque_wanted = True            # what set_torque left behind while offline
    board.open()
    try:
        time.sleep(0.15)
        assert board._torque_wanted is False
        assert not board.torque_on
    finally:
        board.close()
