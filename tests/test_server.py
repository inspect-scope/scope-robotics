import asyncio
import os
import threading
import time
import urllib.request

import pytest
from fastapi.testclient import TestClient

from hexapod import config as config_mod
from hexapod.board import FakeBoard
from hexapod.camera import FakeCamera
from hexapod.config import CameraConfig, ImuConfig
from hexapod.controller import Controller
from hexapod.imu import FakeImu
from hexapod.server import _mjpeg, create_app
from hexapod.state import RobotState


def _wait_for(predicate, seconds=3.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def _stack(tmp_path, camera=True):
    config = config_mod.load()
    board = FakeBoard(config)
    board.open()
    controller = Controller(config, board)
    controller.start()
    state = RobotState(
        config, controller,
        imu=FakeImu(ImuConfig(bias_seconds=0.05)),
        camera=FakeCamera(CameraConfig(stream_fps=30.0, survey_dir=str(tmp_path))) if camera else None,
    )
    state.start()
    return config, board, controller, state


def _teardown(board, controller, state):
    state.stop()
    controller.stop()
    board.close()


@pytest.fixture()
def client(tmp_path):
    config, board, controller, state = _stack(tmp_path)
    with TestClient(create_app(state, config)) as test_client:
        test_client.controller = controller
        test_client.state = state
        yield test_client
    _teardown(board, controller, state)


def test_pages_and_state_are_served(client):
    assert client.get("/").status_code == 200
    status = client.get("/status")
    assert status.status_code == 200 and b"STOP" in status.content
    body = client.get("/api/state").json()
    assert body["state"] == "off"
    assert set(body["feet"]) == set(body["coxae"]) == set(body["joints"]) == set(body["chains"])
    assert {"imu", "camera", "ip", "uptime_s", "last_error", "actuators", "pulses", "touch_volts"} <= set(body)
    assert len(body["pulses"]) == 18
    assert all(len(points) == 4 and len(points[0]) == 3 for points in body["chains"].values())
    l1 = body["actuators"]["L1"]["coxa"]
    assert {"ch", "us", "joint", "servo"} <= set(l1)
    cfg = client.get("/api/config").json()
    assert cfg["servos"]["L1"]["coxa"]["channel"] == l1["ch"]
    assert cfg["pulse_us"] == [600, 2400]
    assert cfg["geometry"]["coxa_len"] == 43


def test_stop_and_estop_work_over_plain_http(client):
    assert client.post("/api/estop").json()["ok"] is True
    assert client.get("/api/state").json()["estopped"] is True
    client.controller.clear_estop()
    assert client.post("/stop").json() == {"ok": True, "state": "estop"}
    assert client.controller.snapshot().estopped


def test_move_posts_a_velocity_that_expires(client):
    body = client.post("/move", json={"direction": "forward", "speed": 0.5}).json()
    assert body["ok"] and body["velocity"] == {"vx": 0.0, "vy": 0.5, "yaw": 0.0}
    assert body["state"] == "off" and body["moving"] is False
    assert _wait_for(lambda: client.controller.snapshot().active_source == "rest")
    assert _wait_for(lambda: client.controller.snapshot().active_source is None, seconds=1.5)

    body = client.post("/move", json={"vx": 2.0, "yaw": -0.25}).json()
    assert body["velocity"] == {"vx": 1.0, "vy": 0.0, "yaw": -0.25}
    assert client.post("/move", json={"direction": "sideways"}).status_code == 400
    assert client.post("/move", json={}).status_code == 400


def test_capture_saves_a_still_and_reports_the_path(client, tmp_path):
    body = client.post("/capture").json()
    assert body["ok"] and body["captures"] == 1
    assert body["path"].startswith(str(tmp_path)) and os.path.getsize(body["path"]) == body["bytes"]
    assert os.path.exists(body["path"][:-4] + ".json")
    assert client.get("/api/state").json()["camera"]["captures"] == 1


def test_stream_and_capture_are_503_without_a_camera(tmp_path):
    config, board, controller, state = _stack(tmp_path, camera=False)
    try:
        with TestClient(create_app(state, config)) as test_client:
            assert test_client.get("/stream").status_code == 503
            assert test_client.post("/capture").status_code == 503
    finally:
        _teardown(board, controller, state)


def test_mjpeg_generator_frames_each_jpeg(client):
    async def two_parts():
        parts = []
        async for part in _mjpeg(client.state.camera):
            parts.append(part)
            if len(parts) == 2:
                break
        return parts

    parts = asyncio.run(two_parts())
    for part in parts:
        assert part.startswith(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ")
        header, _, payload = part.partition(b"\r\n\r\n")
        length = int(header.rsplit(b": ", 1)[1])
        assert payload == payload[:length] + b"\r\n" and payload[:2] == b"\xff\xd8"


def test_stream_over_a_real_socket_is_multipart(tmp_path):
    """TestClient buffers whole responses, so the endless stream needs a real server."""
    import uvicorn

    config, board, controller, state = _stack(tmp_path)
    server = uvicorn.Server(uvicorn.Config(create_app(state, config), host="127.0.0.1", port=0, log_level="warning"))
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: server.started, seconds=5)
        port = server.servers[0].sockets[0].getsockname()[1]
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/stream", timeout=5) as response:
            assert response.headers["Content-Type"] == "multipart/x-mixed-replace; boundary=frame"
            data = b""
            while data.count(b"--frame\r\n") < 3 and len(data) < 200_000:
                chunk = response.read1(8192)
                if not chunk:
                    break
                data += chunk
        assert data.count(b"--frame\r\n") >= 3
        assert data.count(b"\xff\xd8") >= 2
    finally:
        server.should_exit = True
        thread.join(timeout=5)
        _teardown(board, controller, state)


def test_telemetry_socket_only_pushes(client):
    with client.websocket_connect("/telemetry") as socket:
        message = socket.receive_json()
        assert message["type"] == "state" and "imu" in message and "volts" in message
        socket.send_text("stop")  # ignored, must not break the connection
        assert socket.receive_json()["type"] == "state"


def test_dry_run_contacts_open_in_swing(client):
    snap = client.get("/api/state").json()
    assert snap["contacts"] and all(snap["contacts"].values())

    with client.websocket_connect("/ws") as socket:
        socket.send_json({"type": "stand"})
        assert _wait_for(lambda: client.controller.snapshot().state == "standing")
        assert all(client.controller.snapshot().contacts.values())

        def tripod():
            socket.send_json({"type": "drive", "vx": 0, "vy": 1, "yaw": 0})
            down = client.controller.snapshot().contacts
            return any(down.values()) and not all(down.values())

        assert _wait_for(tripod)


def test_websocket_drives_the_robot(client):
    with client.websocket_connect("/ws") as socket:
        socket.send_json({"type": "stand"})
        assert _wait_for(lambda: client.controller.snapshot().state == "standing")

        def driving():
            socket.send_json({"type": "drive", "vx": 0, "vy": 1, "yaw": 0})
            return client.controller.snapshot().velocity["vy"] > 0

        assert _wait_for(driving)  # the body has to finish rising first

        socket.send_json({"type": "pose", "roll": 4})
        socket.send_json({"type": "unknown-to-us"})
        time.sleep(0.2)
        assert client.controller.snapshot().pose["roll"] == 4
        assert socket.receive_json()["type"] == "state"


def test_jump_runs_and_lands(client):
    with client.websocket_connect("/ws") as socket:
        socket.send_json({"type": "stand"})
        assert _wait_for(lambda: client.controller.snapshot().state == "standing")
        socket.send_json({"type": "jump"})
        assert _wait_for(lambda: client.controller.snapshot().state == "jumping")
        assert _wait_for(lambda: client.controller.snapshot().state == "standing", seconds=2.0)


def test_trick_flex_while_standing(client):
    with client.websocket_connect("/ws") as socket:
        socket.send_json({"type": "trick", "name": "dance"})
        time.sleep(0.15)
        assert client.controller.snapshot().trick is None
        socket.send_json({"type": "stand"})
        assert _wait_for(lambda: client.controller.snapshot().state == "standing")
        socket.send_json({"type": "trick", "name": "dance"})
        assert _wait_for(lambda: client.controller.snapshot().state == "dancing")
        socket.send_json({"type": "trick", "on": False})
        assert _wait_for(lambda: client.controller.snapshot().trick is None)
        socket.send_json({"type": "trick", "name": "dance"})
        assert _wait_for(lambda: client.controller.snapshot().trick == "dance")
        socket.send_json({"type": "gait", "pattern": "ripple"})
        assert _wait_for(lambda: client.controller.snapshot().trick is None)
        ids = [row["id"] for row in client.get("/api/config").json()["gait"]["catalog_tricks"]]
        assert ids == ["flex", "lean-pitch", "lean-roll", "lean-yaw", "spin", "dance"]


def test_stance_mode_retunes(client):
    with client.websocket_connect("/ws") as socket:
        socket.send_json({"type": "stand"})
        assert _wait_for(lambda: client.controller.snapshot().state == "standing")
        socket.send_json({"type": "mode", "name": "speed"})
        assert _wait_for(lambda: client.controller.snapshot().mode == "speed")
        speed = client.controller.snapshot()
        assert speed.max_speed > 120
        assert speed.cycle_s < 0.8
        socket.send_json({"type": "mode", "name": "offroad"})
        assert _wait_for(lambda: client.controller.snapshot().mode == "offroad")
        offroad = client.controller.snapshot()
        assert offroad.step_lift > speed.step_lift
        assert offroad.max_speed < speed.max_speed
        cfg = client.get("/api/config").json()["mode"]
        assert cfg["current"] == "offroad"
        assert {row["chica"] for row in cfg["catalog"]} == {"standard", "race", "offroad"}


def test_gait_pattern_switches(client):
    with client.websocket_connect("/ws") as socket:
        socket.send_json({"type": "gait", "pattern": "ripple"})
        assert _wait_for(lambda: client.controller.snapshot().gait == "ripple")
        assert "ripple" in client.get("/api/config").json()["gait"]["patterns"]
        socket.send_json({"type": "gait", "pattern": "walk3"})
        assert _wait_for(lambda: client.controller.snapshot().gait == "tripod")


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


def test_state_reports_joint_angles_and_the_pulse_frame(client):
    body = client.get("/api/state").json()
    assert len(body["pulses"]) == 18 and all(isinstance(p, int) for p in body["pulses"])
    assert set(body["angles"]) == set(body["feet"])
    sitting = {leg: a["femur"] for leg, a in body["angles"].items()}
    before = list(body["pulses"])

    client.controller.board.set_torque(True)
    client.controller.stand()
    assert _wait_for(lambda: client.controller.snapshot().state == "standing")
    assert _wait_for(lambda: abs(client.controller.snapshot().height - 80) < 0.5, seconds=3.0)

    body = client.get("/api/state").json()
    cfg = client.get("/api/config").json()
    for leg, angles in body["angles"].items():
        # sit 40 -> stand 80 drops every femur by ~31.6 deg; the pulse must follow
        assert sitting[leg] - angles["femur"] == pytest.approx(31.6, abs=0.5)
        femur_channel = cfg["servos"][leg]["femur"]["channel"]
        assert body["pulses"][femur_channel] != before[femur_channel]


def test_config_names_every_channel_once(client):
    cfg = client.get("/api/config").json()
    channels = [s["channel"] for joints in cfg["servos"].values() for s in joints.values()]
    assert sorted(channels) == list(range(18))
    assert all(s["direction"] in (1, -1) for joints in cfg["servos"].values() for s in joints.values())
    assert cfg["pulse_us"] == [600, 2400]
