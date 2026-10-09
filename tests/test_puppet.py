"""Puppeteer owns the joints only while torque is on, and only at a capped rate."""

import time

import pytest
from fastapi.testclient import TestClient

from hexapod import config as config_mod
from hexapod.board import FakeBoard
from hexapod.controller import PUPPET_COXA_STEP, PUPPET_RATE, LimbMode, Controller
from hexapod.server import create_app
from hexapod.state import RobotState


def _wait_for(predicate, seconds=3.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def _open():
    config = config_mod.load()
    board = FakeBoard(config)
    board.open()
    board.set_frame([1500] * 18)
    board.set_torque(True)
    assert _wait_for(lambda: board.torque_on)
    made = Controller(config, board)
    return made, board


def _femur(controller, leg="L1"):
    return controller.snapshot().angles[leg]["femur"]


def test_puppeteer_refuses_without_torque():
    config = config_mod.load()
    board = FakeBoard(config)
    board.open()
    controller = Controller(config, board)
    assert controller.arm_puppeteer() is False
    assert controller.snapshot().puppeteer == LimbMode.Off
    board.close()


def test_one_tick_cannot_jump_a_joint():
    controller, board = _open()
    try:
        assert controller.arm_puppeteer()
        start = _femur(controller)
        controller.aim_joint("L1", "femur", start + 30)
        controller._tick(0.05, time.monotonic())
        moved = _femur(controller) - start
        assert moved == pytest.approx(PUPPET_RATE * 0.05, abs=0.15)
        assert moved < 5
        low, high = controller.config.limits.pulse_us
        assert all(low <= pulse <= high for pulse in controller.snapshot().pulses)
    finally:
        controller.estop()
        board.close()


def test_target_is_clamped_to_the_joint_range():
    controller, board = _open()
    try:
        assert controller.arm_puppeteer()
        high = controller.config.limits.femur_deg[1]
        controller.aim_joint("L1", "femur", 999)
        controller.aim_joint("L1", "nope", 10)
        controller.aim_joint("L1", "femur", float("nan"))
        assert controller._puppet_goal["L1"].femur == high
    finally:
        controller.estop()
        board.close()


def test_release_freezes_short_of_the_ask():
    controller, board = _open()
    try:
        assert controller.arm_puppeteer()
        start = _femur(controller)
        controller.aim_joint("L1", "femur", start + 40)
        controller._tick(0.05, time.monotonic())
        controller.hold_puppeteer()
        held = _femur(controller)
        controller._tick(0.2, time.monotonic() + 1)
        assert _femur(controller) == pytest.approx(held, abs=0.05)
        assert held - start < 10
    finally:
        controller.estop()
        board.close()


def test_quiet_page_does_not_finish_the_move():
    controller, board = _open()
    try:
        assert controller.arm_puppeteer()
        start = _femur(controller)
        controller.aim_joint("L1", "femur", start + 40)
        controller._puppet_at = time.monotonic() - 5
        controller._tick(0.05, time.monotonic())
        assert _femur(controller) == pytest.approx(start, abs=0.05)
    finally:
        controller.estop()
        board.close()


def test_home_slews_back_to_the_stance():
    controller, board = _open()
    try:
        assert controller.arm_puppeteer()
        start = _femur(controller)
        controller.aim_joint("L1", "femur", start + 25)
        for i in range(20):
            controller._tick(0.05, time.monotonic() + i * 0.05)
        assert _femur(controller) > start + 10
        controller.home_puppeteer()
        for i in range(40):
            controller._tick(0.05, time.monotonic() + i * 0.05)
            if controller.snapshot().puppeteer == LimbMode.Off:
                break
        assert controller.snapshot().puppeteer == LimbMode.Off
        assert _femur(controller) == pytest.approx(start, abs=1.0)
    finally:
        controller.estop()
        board.close()


def _ground(controller, leg, index):
    kin = controller.kinematics.legs[leg]
    body = kin.chain(controller._angles[leg])[index]
    return controller._pose.body_to_ground(body)


def test_placing_the_current_foot_does_not_move_it():
    controller, board = _open()
    try:
        assert controller.arm_puppeteer()
        foot = _ground(controller, "L1", 3)
        before = controller._angles["L1"].as_tuple()
        controller.aim_point("L1", "foot", *foot)
        after = controller._puppet_goal["L1"].as_tuple()
        assert after == pytest.approx(before, abs=0.2)
    finally:
        controller.estop()
        board.close()


def test_lifting_the_foot_slews_and_a_flip_cannot_queue():
    controller, board = _open()
    try:
        assert controller.arm_puppeteer()
        foot = _ground(controller, "L1", 3)
        held = controller._angles["L1"]
        controller.aim_point("L1", "foot", foot[0], foot[1], foot[2] + 40)
        goal = controller._puppet_goal["L1"]
        assert abs(goal.coxa - held.coxa) <= PUPPET_COXA_STEP
        assert goal.as_tuple() != pytest.approx(held.as_tuple(), abs=0.5)
        start = _femur(controller)
        controller._tick(0.05, time.monotonic())
        assert abs(_femur(controller) - start) <= PUPPET_RATE * 0.05 + 0.15

        controller.aim_point("L1", "foot", -foot[0], -foot[1], foot[2])
        assert abs(controller._puppet_goal["L1"].coxa - controller._puppet_held["L1"].coxa) <= PUPPET_COXA_STEP + 0.1
    finally:
        controller.estop()
        board.close()


def test_grabbing_the_knee_leaves_the_tibia():
    controller, board = _open()
    try:
        assert controller.arm_puppeteer()
        knee = _ground(controller, "L1", 2)
        tibia = controller._angles["L1"].tibia
        controller.aim_point("L1", "knee", knee[0], knee[1], knee[2] + 30)
        goal = controller._puppet_goal["L1"]
        assert goal.tibia == pytest.approx(tibia, abs=1e-6)
        assert goal.femur != pytest.approx(controller._angles["L1"].femur, abs=0.5)
        controller.aim_point("L1", "nope", *knee)
        assert controller._puppet_goal["L1"].tibia == pytest.approx(tibia, abs=1e-6)
    finally:
        controller.estop()
        board.close()


def test_ghost_shows_the_ask_until_the_leg_catches_it():
    controller, board = _open()
    try:
        assert controller.arm_puppeteer()
        assert controller.snapshot().ghost is None
        start = _femur(controller)
        controller.aim_joint("L1", "femur", start + 25)
        ghost = controller.snapshot().ghost
        assert ghost is not None and set(ghost) == {"L1"}
        assert len(ghost["L1"]) == 4
        controller.hold_puppeteer()
        assert controller.snapshot().ghost is None
        controller.aim_joint("L1", "femur", start + 25)
        controller.home_puppeteer()
        homing = controller.snapshot()
        assert homing.puppeteer == "home" and homing.ghost is None
    finally:
        controller.estop()
        board.close()


def test_estop_drops_the_pose():
    controller, board = _open()
    try:
        assert controller.arm_puppeteer()
        controller.estop()
        assert controller.snapshot().puppeteer == LimbMode.Off
        assert controller.arm_puppeteer() is False
    finally:
        board.close()


def test_websocket_arms_only_with_torque_and_homes(tmp_path):
    config = config_mod.load()
    board = FakeBoard(config)
    board.open()
    controller = Controller(config, board)
    controller.start()
    state = RobotState(config, controller)
    state.start()
    try:
        with TestClient(create_app(state, config)) as client:
            with client.websocket_connect("/ws") as socket:
                socket.send_json({"type": "puppeteer", "on": True})
                time.sleep(0.15)
                assert controller.snapshot().puppeteer == LimbMode.Off

                socket.send_json({"type": "torque", "on": True})
                assert _wait_for(lambda: controller.snapshot().torque)
                socket.send_json({"type": "puppeteer", "on": True})
                assert _wait_for(lambda: controller.snapshot().puppeteer == LimbMode.Hold)

                start = controller.snapshot().angles["L1"]["femur"]
                high = config.limits.femur_deg[1]
                socket.send_json({"type": "puppeteer", "leg": "L1", "joint": "femur", "deg": 999})
                assert _wait_for(lambda: controller.snapshot().angles["L1"]["femur"] > start + 1)
                assert controller.snapshot().angles["L1"]["femur"] < high
                socket.send_json({"type": "puppeteer", "hold": True})
                time.sleep(0.2)
                frozen = controller.snapshot().angles["L1"]["femur"]
                time.sleep(0.4)
                assert controller.snapshot().angles["L1"]["femur"] == pytest.approx(frozen, abs=0.6)

                socket.send_json({"type": "jump"})
                time.sleep(0.15)
                assert controller.snapshot().state != "jumping"

                socket.send_json({"type": "puppeteer", "on": False})
                assert _wait_for(lambda: controller.snapshot().puppeteer == LimbMode.Off, seconds=4)
            cfg = client.get("/api/config").json()
            assert cfg["joint_deg"]["femur"] == list(config.limits.femur_deg)

            with client.websocket_connect("/ws") as posing:
                posing.send_json({"type": "torque", "on": True})
                assert _wait_for(lambda: controller.snapshot().torque)
                posing.send_json({"type": "puppeteer", "on": True})
                assert _wait_for(lambda: controller.snapshot().puppeteer == LimbMode.Hold)
                with client.websocket_connect("/ws") as watcher:
                    watcher.send_json({"type": "ping"})
                time.sleep(0.2)
                assert controller.snapshot().puppeteer == LimbMode.Hold
    finally:
        state.stop()
        controller.stop()
        board.close()
