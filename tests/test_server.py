import time

import pytest
from fastapi.testclient import TestClient

from hexapod import config as config_mod
from hexapod.board import FakeBoard
from hexapod.controller import Controller
from hexapod.server import create_app


@pytest.fixture()
def client():
    config = config_mod.load()
    board = FakeBoard(config)
    board.open()
    controller = Controller(config, board)
    controller.start()
    with TestClient(create_app(controller, config)) as test_client:
        test_client.controller = controller
        yield test_client
    controller.stop()
    board.close()


def test_index_and_state_are_served(client):
    assert client.get("/").status_code == 200
    body = client.get("/api/state").json()
    assert body["state"] == "off"
    assert set(body["feet"]) == set(body["coxae"])


def test_estop_works_over_plain_http(client):
    assert client.post("/api/estop").json()["ok"] is True
    assert client.get("/api/state").json()["estopped"] is True


def test_websocket_drives_the_robot(client):
    with client.websocket_connect("/ws") as socket:
        socket.send_json({"type": "stand"})
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and client.controller.snapshot().state != "standing":
            time.sleep(0.05)
        assert client.controller.snapshot().state == "standing"

        for _ in range(20):
            socket.send_json({"type": "drive", "vx": 0, "vy": 1, "yaw": 0})
            time.sleep(0.03)
        assert client.controller.snapshot().velocity["vy"] > 0

        socket.send_json({"type": "pose", "roll": 4})
        socket.send_json({"type": "unknown-to-us"})
        time.sleep(0.2)
        assert client.controller.snapshot().pose["roll"] == 4


def test_pose_and_gait_values_are_clamped(client):
    with client.websocket_connect("/ws") as socket:
        socket.send_json({"type": "pose", "roll": 90, "x": -500})
        socket.send_json({"type": "gait", "cycle_s": 99, "step_lift": -5})
        time.sleep(0.2)
        pose = client.controller.snapshot().pose
        assert pose["roll"] == 8.0 and pose["x"] == -15.0
        assert client.controller.config.stance.cycle_s == 2.5
        assert client.controller.config.stance.step_lift == 5.0


def test_dropping_the_socket_stops_the_robot(client):
    with client.websocket_connect("/ws") as socket:
        socket.send_json({"type": "stand"})
        for _ in range(10):
            socket.send_json({"type": "drive", "vx": 0, "vy": 1, "yaw": 0})
            time.sleep(0.03)
    time.sleep(0.8)
    assert client.controller.snapshot().active_source is None
