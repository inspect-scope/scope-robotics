import math

import pytest

from hexapod import config as config_mod
from hexapod.kinematics import BodyPose, HexapodKinematics


@pytest.fixture(scope="module")
def kinematics():
    return HexapodKinematics(config_mod.load())


def test_fk_inverts_ik_everywhere_in_the_stance(kinematics):
    for name, leg in kinematics.legs.items():
        for height in (45, 60, 80, 100):
            for dx, dy in ((0, 0), (40, 0), (-40, 0), (0, 45), (0, -45), (30, 30)):
                nx, ny = kinematics.config.legs[name].neutral_xy
                target = leg.body_to_leg((nx + dx, ny + dy, -height))
                back = leg.fk(leg.ik(target))
                assert math.dist(target, back) < 1e-6, (name, height, dx, dy)


def test_neutral_stance_leaves_the_coxa_centred(kinematics):
    for name, angles in kinematics.solve(kinematics.neutral_feet()).items():
        assert abs(angles.coxa) < 0.5, name


def test_neutral_stance_is_within_every_joint_limit(kinematics):
    feet = kinematics.neutral_feet()
    for height in (45, 60, 80, 100):
        _, limited = kinematics.solve_reporting({n: (p[0], p[1], -height) for n, p in feet.items()})
        assert limited == [], height


def test_knee_ik_recovers_coxa_and_femur(kinematics):
    from hexapod.kinematics import JointAngles

    for name, leg in kinematics.legs.items():
        angles = JointAngles(coxa=12.0, femur=35.0, tibia=90.0)
        knee = leg.body_to_leg(leg.chain(angles)[2])
        coxa, femur = leg.ik_knee(knee)
        assert coxa == pytest.approx(angles.coxa, abs=1e-6), name
        assert femur == pytest.approx(angles.femur, abs=1e-6), name


def test_chain_foot_matches_fk(kinematics):
    for name, leg in kinematics.legs.items():
        angles = kinematics.solve(kinematics.neutral_feet())[name]
        foot = leg.chain(angles)[3]
        assert math.dist(foot, leg.leg_to_body(leg.fk(angles))) < 1e-6, name


def test_body_to_ground_inverts_foot_to_body():
    pose = BodyPose(x=5, y=-3, roll=4, pitch=-6, yaw=10)
    point = (120.0, -80.0, -70.0)
    assert math.dist(pose.body_to_ground(pose.foot_to_body(point)), point) < 1e-6


def test_unclamped_chain_lands_on_the_foot_target(kinematics):
    pose = BodyPose(roll=3, pitch=-2, yaw=4, x=5, y=-4)
    feet = kinematics.neutral_feet(80)
    angles, limited = kinematics.solve_reporting(feet, pose)
    assert limited == []
    chains = kinematics.chains(angles, pose)
    for name, target in feet.items():
        assert math.dist(chains[name][3], target) < 0.05, name


def test_pose_axes_point_the_way_their_names_say(kinematics):
    front = (0.0, 200.0, -80.0)
    right = (200.0, 0.0, -80.0)
    # nose up: the front foot has further to reach, so it sits lower in the body frame
    assert BodyPose(pitch=10).foot_to_body(front)[2] < front[2]
    # right side down: the right foot is closer to the body
    assert BodyPose(roll=10).foot_to_body(right)[2] > right[2]
    # yaw is a pure rotation, so the radius is unchanged
    turned = BodyPose(yaw=15).foot_to_body(front)
    assert math.hypot(*turned[:2]) == pytest.approx(200.0)


def test_out_of_reach_targets_land_on_the_workspace_boundary(kinematics):
    leg = kinematics.legs["R2"]
    g = kinematics.config.geometry
    far = leg.ik((g.coxa_len + g.femur_len + g.tibia_len + 200.0, 0.0, 0.0))
    reached = math.hypot(leg.fk(far)[0] - g.coxa_len, leg.fk(far)[2])
    assert reached == pytest.approx(g.femur_len + g.tibia_len, abs=0.05)
    with pytest.raises(Exception):
        leg.ik((1000.0, 0.0, 0.0), strict=True)


def test_pulse_frame_covers_all_eighteen_channels(kinematics):
    frame = kinematics.pulse_frame(kinematics.solve(kinematics.neutral_feet()))
    low, high = kinematics.config.limits.pulse_us
    assert len(frame) == 18
    assert all(low <= pulse <= high for pulse in frame)


def test_calibration_maps_the_two_reference_angles(kinematics):
    cal = kinematics.config.legs["R2"].servos["femur"]
    assert cal.pulse_for(-45) == pytest.approx(cal.us_neg45)
    assert cal.pulse_for(45) == pytest.approx(cal.us_pos45)
    assert cal.pulse_for(0) == pytest.approx((cal.us_neg45 + cal.us_pos45) / 2)
