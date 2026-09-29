import math

import pytest

from hexapod import config as config_mod
from hexapod.gait import (
    Gait, GaitKind, LEAN_DEG, SHIFT_MM, STANCE_FRACTION, TrickKind, Velocity,
    walk_catalog,
)
from hexapod.kinematics import BodyPose


@pytest.fixture()
def setup():
    config = config_mod.load()
    return config, Gait(config)


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


def test_chica_name_selects_the_walk(setup):
    _, gait = setup
    gait.set_pattern("walk1")
    assert gait.pattern == GaitKind.Ripple
    gait.set_pattern("walk3")
    assert gait.pattern == GaitKind.Tripod


def test_triple_keeps_four_feet_down(setup):
    config, gait = setup
    gait.set_pattern(GaitKind.Triple)
    ground = -config.stance.ride_height
    for feet in _run(gait, config, Velocity(0, 1, 0).scaled(config), 4):
        down = [name for name, p in feet.items() if p[2] <= ground + 1e-6]
        assert len(down) >= 4, down


def test_ripple15_keeps_four_feet_down(setup):
    config, gait = setup
    gait.set_pattern(GaitKind.Ripple15)
    ground = -config.stance.ride_height
    for feet in _run(gait, config, Velocity(0, 1, 0).scaled(config), 4):
        down = [name for name, p in feet.items() if p[2] <= ground + 1e-6]
        assert len(down) >= 4, down


def test_catalog_lists_chica_walk_names(setup):
    config, _ = setup
    rows = walk_catalog(config.tripod_groups)
    assert {row["chica"] for row in rows} == {"walk3", "walk2", "walk25", "walk1", "walk15", "walkwave"}


def test_jump_lifts_the_body_then_lands(setup):
    config, gait = setup
    gait.start_jump()
    pose = BodyPose()
    zs = []
    dt = 1.0 / config.rate_hz
    for _ in range(int(0.8 * config.rate_hz)):
        gait.step(dt, Velocity(), config.stance.ride_height)
        zs.append(gait.pose_overlay(pose).z)
    assert max(zs) >= 20.0
    assert zs[-1] == pytest.approx(0.0, abs=0.05)
    assert gait.trick is None


def test_bounce_bobs_without_leaving_the_ground(setup):
    config, gait = setup
    gait.set_bounce(True)
    ground = -config.stance.ride_height
    zs = []
    dt = 1.0 / config.rate_hz
    for _ in range(int(0.6 * config.rate_hz)):
        feet = gait.step(dt, Velocity(), config.stance.ride_height)
        assert all(p[2] == pytest.approx(ground) for p in feet.values())
        zs.append(gait.pose_overlay(BodyPose()).z)
    assert max(zs) > 2.0
    assert min(zs) < -2.0
    assert gait.trick == TrickKind.Bounce


def _pose_run(gait, config, seconds):
    dt = 1.0 / config.rate_hz
    poses = []
    for _ in range(int(seconds * config.rate_hz)):
        gait.step(dt, Velocity(), config.stance.ride_height)
        poses.append(gait.pose_overlay(BodyPose()))
    return poses


def test_flex_leans_pitch_then_roll(setup):
    config, gait = setup
    gait.start_trick(TrickKind.Flex)
    poses = _pose_run(gait, config, 4.0)
    pitches = [p.pitch for p in poses]
    rolls = [p.roll for p in poses]
    assert max(pitches) > LEAN_DEG * 0.8
    assert max(rolls) > LEAN_DEG * 0.8
    early, late = poses[:40], poses[80:160]
    assert max(abs(p.roll) for p in early) < 0.2
    assert max(abs(p.pitch) for p in late) < 0.2
    assert gait.trick is None


def test_lean_pitch_loops_on_that_axis(setup):
    config, gait = setup
    gait.start_trick(TrickKind.LeanPitch)
    poses = _pose_run(gait, config, 3.0)
    assert max(p.pitch for p in poses) > 4.0
    assert min(p.pitch for p in poses) < -4.0
    assert all(p.roll == 0 and p.yaw == 0 for p in poses)
    assert gait.trick == TrickKind.LeanPitch


def test_dance_swings_the_body_around(setup):
    config, gait = setup
    gait.start_trick(TrickKind.Dance)
    poses = _pose_run(gait, config, 4.0)
    xs = [p.x for p in poses]
    ys = [p.y for p in poses]
    assert max(xs) > SHIFT_MM * 0.8
    assert max(ys) > SHIFT_MM * 0.8
    assert gait.trick == TrickKind.Dance


def test_spin_ripples_in_place_then_restores(setup):
    config, gait = setup
    gait.set_pattern(GaitKind.Tripod)
    gait.start_trick(TrickKind.Spin)
    assert gait.pattern == GaitKind.Ripple
    dt = 1.0 / config.rate_hz
    for _ in range(int(2.0 * config.rate_hz)):
        gait.step(dt, Velocity(), config.stance.ride_height)
    assert gait.velocity.yaw_rate > 10.0
    assert gait.pattern == GaitKind.Ripple
    gait.stop_trick()
    assert gait.trick is None
    assert gait.pattern == GaitKind.Tripod
