import time

import pytest

from hexapod import config as config_mod
from hexapod.board import FakeBoard
from hexapod.controller import PINCER_FWD_MM, PINCER_LIFT_MM, POSE_LIMITS, Controller
from hexapod.hand import (
    Follow, HandBlob, HandFollower, Puppet, pincers_from_hand, pose_from_blob,
    pose_from_hand,
)
from hexapod.kinematics import BodyPose


def _flat_hand():
    pts = [(0.50, 0.55, 0.0)] * 21
    pts = list(pts)
    for mcp, pip, tip, x in ((5, 6, 8, 0.40), (9, 10, 12, 0.50), (13, 14, 16, 0.60), (17, 18, 20, 0.68)):
        pts[mcp] = (x, 0.62, 0.0)
        pts[pip] = (x, 0.48, 0.0)
        pts[tip] = (x, 0.30, 0.0)
    return pts


def _point_hand():
    pts = _flat_hand()
    for mcp, pip, tip, x in ((13, 14, 16, 0.60), (17, 18, 20, 0.68)):
        pts[mcp] = (x, 0.58, 0.0)
        pts[pip] = (x, 0.56, 0.0)
        pts[tip] = (x + 0.01, 0.59, 0.0)
    pts[8] = (0.36, 0.22, 0.0)
    pts[12] = (0.58, 0.20, 0.0)
    return pts


@pytest.fixture()
def controller():
    config = config_mod.load()
    board = FakeBoard(config)
    board.open()
    ctrl = Controller(config, board)
    ctrl.start()
    yield ctrl
    ctrl.stop()
    board.close()


def test_rest_pose_is_zero_inside_the_deadzone():
    pose = pose_from_blob(HandBlob(nx=0.1, ny=-0.05, tilt=0.0, area=0.1))
    assert pose == BodyPose()


def test_hand_mirrors_and_stays_inside_limits():
    right = pose_from_blob(HandBlob(nx=1.0, ny=0.0, tilt=0.0, area=0.2))
    assert right.x == -POSE_LIMITS["shift"]
    assert right.yaw == -POSE_LIMITS["yaw"]
    assert right.y == 0.0 and right.z == 0.0

    forward = pose_from_blob(HandBlob(nx=0.0, ny=1.0, tilt=0.0, area=0.2))
    assert forward.y == POSE_LIMITS["shift"]
    assert forward.pitch == POSE_LIMITS["pitch"]

    tilt = pose_from_blob(HandBlob(nx=0.0, ny=0.0, tilt=1.0, area=0.2))
    assert tilt.roll == POSE_LIMITS["roll"]


def test_landmarks_map_inside_limits():
    def landmarks(cx, cy):
        pts = [(cx, cy, 0.0)] * 21
        pts = list(pts)
        pts[5] = (cx - 0.06, cy, 0.0)
        pts[17] = (cx + 0.06, cy, 0.0)
        pts[9] = (cx, cy - 0.1, 0.0)
        return pts

    rest = pose_from_hand(landmarks(0.5, 0.5))
    assert rest.x == 0.0 and rest.y == 0.0

    right = pose_from_hand(landmarks(0.95, 0.5))
    assert right.x < 0 and abs(right.x) <= POSE_LIMITS["shift"]
    assert abs(right.roll) <= POSE_LIMITS["roll"]
    assert abs(right.pitch) <= POSE_LIMITS["pitch"]
    assert abs(right.yaw) <= POSE_LIMITS["yaw"]
    assert right.z == 0.0


def test_follower_accepts_landmarks(controller):
    follower = HandFollower(controller)
    pts = [(0.95, 0.5, 0.0)] * 21
    pts = list(pts)
    pts[5] = (0.89, 0.5, 0.0)
    pts[17] = (0.99, 0.5, 0.0)
    pts[9] = (0.95, 0.4, 0.0)
    follower.set_follow(Follow.On)
    snap = follower.see_landmarks(pts)
    assert snap["source"] == "mediapipe"
    assert snap["tracking"] is True
    assert len(snap["points"]) == 21
    assert snap["pose"]["x"] < 0


def test_pincer_assigns_left_tip_to_l1():
    grips = pincers_from_hand(_point_hand())
    assert set(grips) == {"L1", "R1"}
    assert grips["L1"][2] > 0 and grips["R1"][2] > 0
    assert grips["L1"][0] <= grips["R1"][0]


def test_pincer_amplifies_short_finger_travel():
    pts = [(0.50, 0.60, 0.0)] * 21
    pts = list(pts)
    pts[5] = (0.46, 0.62, 0.0)
    pts[9] = (0.50, 0.62, 0.0)
    pts[13] = (0.54, 0.62, 0.0)
    pts[17] = (0.57, 0.62, 0.0)
    pts[8] = (0.45, 0.53, 0.0)
    pts[12] = (0.55, 0.53, 0.0)
    grips = pincers_from_hand(pts)
    assert grips["L1"][2] >= 0.7 * PINCER_LIFT_MM
    assert grips["R1"][1] >= 0.7 * PINCER_FWD_MM


def test_pincers_lift_front_feet(controller):
    controller.board.set_torque(True)
    controller.set_height(80)
    time.sleep(0.05)
    before = controller.snapshot().feet["L1"][2]
    controller.set_pincers({"L1": (0, 0, 20), "R1": (0, 0, 20)})
    time.sleep(0.08)
    assert controller.snapshot().feet["L1"][2] > before


def test_follower_stays_lean_until_toggled(controller):
    follower = HandFollower(controller)
    follower.set_follow(Follow.On)
    snap = follower.see_landmarks(_point_hand())
    assert snap["mode"] == Puppet.Lean
    follower.set_puppet(Puppet.Pincer)
    grab = follower.see_landmarks(_flat_hand())
    assert grab["mode"] == Puppet.Pincer
    assert "L1" in grab["pincers"]
    follower.set_puppet(Puppet.Lean)
    assert follower.snapshot()["mode"] == Puppet.Lean


def test_turning_follow_off_zeros_pose(controller):
    controller.set_pose(x=10, roll=4)
    follower = HandFollower(controller, camera=object())
    follower.set_follow(Follow.On)
    follower.set_follow(Follow.Off)
    pose = controller.snapshot().pose
    assert pose["x"] == 0.0 and pose["roll"] == 0.0
    snap = follower.snapshot()
    assert snap["follow"] == Follow.Off and snap["tracking"] is False
