import json
import os
import threading
import time
from datetime import datetime

import pytest

from hexapod.camera import CameraError, FakeCamera, FrameBuffer
from hexapod.config import CameraConfig

JPEG_SOI = b"\xff\xd8"


def _wait_for(predicate, seconds=2.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


@pytest.fixture()
def camera(tmp_path):
    device = FakeCamera(CameraConfig(stream_fps=30.0, survey_dir=str(tmp_path)))
    device.open()
    yield device
    device.close()


def test_frame_buffer_hands_out_newer_frames_only():
    frames = FrameBuffer()
    assert frames.wait(0, 0.05) is None
    threading.Timer(0.05, frames.write, args=(b"one",)).start()
    seq, frame = frames.wait(0, 1.0)
    assert (seq, frame) == (1, b"one")
    assert frames.wait(seq, 0.05) is None  # nothing newer yet
    frames.write(b"two")
    assert frames.latest() == (2, b"two")


def test_fake_camera_streams_jpegs(camera):
    assert _wait_for(lambda: camera.frames.seq >= 3)
    _, frame = camera.frames.latest()
    assert frame.startswith(JPEG_SOI)
    assert camera.ok and camera.streaming


def test_capture_writes_a_still_and_a_sidecar(camera, tmp_path):
    path = camera.capture({"volts": 7.4, "note": "test"})
    assert path.startswith(str(tmp_path))
    assert open(path, "rb").read(2) == JPEG_SOI
    sidecar = json.load(open(path[:-4] + ".json"))
    assert sidecar["volts"] == 7.4
    assert camera.captures == 1 and camera.last_capture == path
    # Every still from one server run lands in the same directory.
    second = camera.capture()
    assert second != path
    assert second.rsplit("/", 1)[0] == path.rsplit("/", 1)[0]


def test_two_stills_in_one_millisecond_do_not_collide(camera, monkeypatch):
    from hexapod import camera as camera_mod

    class Frozen:  # the stamp only has milliseconds, so pin it and take two
        @staticmethod
        def now():
            return datetime(2026, 9, 20, 16, 46, 31, 344000)

    monkeypatch.setattr(camera_mod, "datetime", Frozen)
    first, second = camera.capture(), camera.capture()
    assert first != second
    assert os.path.exists(first) and os.path.exists(second)


def test_capture_before_open_fails(tmp_path):
    device = FakeCamera(CameraConfig(survey_dir=str(tmp_path)))
    with pytest.raises(CameraError):
        device.capture()
    assert not device.ok


def test_close_stops_the_stream(camera):
    camera.close()
    seq = camera.frames.seq
    time.sleep(0.15)
    assert camera.frames.seq == seq
    assert not camera.ok
