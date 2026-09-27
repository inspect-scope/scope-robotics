import math

import pytest

from hexapod import config as config_mod
from hexapod.gait import GaitKind, STANCE_FRACTION, TripodGait, Velocity


@pytest.fixture()
def setup():
    config = config_mod.load()
    return config, TripodGait(config)


def _run(gait, config, command, seconds, height=None):
    height = height if height is not None else config.stance.ride_height
    dt = 1.0 / config.rate_hz
    frames = []
    for _ in range(int(seconds * config.rate_hz)):
        frames.append(gait.step(dt, command, height))
    return frames


def test_feet_never_jump(setup):
    config, gait = setup
    frames = _run(gait, config, Velocity(0.7, 0.7, 1.0).scaled(config), 6)
    for before, after in zip(frames, frames[1:]):
        for name in before:
            assert math.dist(before[name], after[name]) < 12.0, name


def test_the_two_tripods_are_never_airborne_together(setup):
    config, gait = setup
    ground = -config.stance.ride_height
    for feet in _run(gait, config, Velocity(0, 1, 0).scaled(config), 4):
        down = [name for name, p in feet.items() if p[2] <= ground + 1e-6]
        assert len(down) >= 3, down


def test_zero_command_settles_back_to_neutral(setup):
    config, gait = setup
    _run(gait, config, Velocity(0, 1, 0).scaled(config), 3)
    feet = _run(gait, config, Velocity(), 5)[-1]
    for name, point in feet.items():
        nx, ny = config.legs[name].neutral_xy
        assert (nx, ny) == pytest.approx((point[0], point[1]))
    assert not gait.walking


def test_stroke_follows_the_commanded_velocity(setup):
    config, gait = setup
    command = Velocity(0, 60, 0)
    stroke = gait.strokes(command)["R2"]
    assert stroke == pytest.approx((0.0, 60 * config.stance.cycle_s * STANCE_FRACTION))


def test_turning_pushes_opposite_sides_opposite_ways(setup):
    config, gait = setup
    strokes = gait.strokes(Velocity(0, 0, 30))
    assert strokes["L2"][1] * strokes["R2"][1] < 0


def test_stroke_is_capped(setup):
    config, gait = setup
    for stroke in gait.strokes(Velocity(0, 10_000, 0)).values():
        assert math.hypot(*stroke) <= config.stance.max_stride + 1e-6


def test_lift_is_capped_so_feet_stay_below_the_body(setup):
    config, gait = setup
    height = 30.0
    for feet in _run(gait, config, Velocity(0, 1, 0).scaled(config), 3, height=height):
        for name, point in feet.items():
            assert point[2] <= -config.stance.min_foot_depth + 1e-6, name


def test_diagonal_input_does_not_exceed_full_speed(setup):
    config, _ = setup
    command = Velocity(1, 1, 0).scaled(config)
    assert math.hypot(command.vx, command.vy) == pytest.approx(config.stance.max_speed)


def test_ripple_keeps_five_feet_down(setup):
    config, gait = setup
    gait.set_pattern(GaitKind.Ripple)
    ground = -config.stance.ride_height
    for feet in _run(gait, config, Velocity(0, 1, 0).scaled(config), 4):
        down = [name for name, p in feet.items() if p[2] <= ground + 1e-6]
        assert len(down) >= 5, down


def test_wave_keeps_five_feet_down(setup):
    config, gait = setup
    gait.set_pattern(GaitKind.Wave)
    ground = -config.stance.ride_height
    for feet in _run(gait, config, Velocity(0, 1, 0).scaled(config), 4):
        down = [name for name, p in feet.items() if p[2] <= ground + 1e-6]
        assert len(down) >= 5, down


def test_unknown_pattern_is_rejected(setup):
    _, gait = setup
    with pytest.raises(ValueError):
        gait.set_pattern("gallop")
