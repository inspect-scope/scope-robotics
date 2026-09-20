"""Owns the Pi camera. Nothing else in the process may open it.

Picamera2 runs two streams off the same sensor frames: a lores one that the
encoder turns into the MJPEG live view, and the full-resolution one that stills
are saved from. Both run all the time, so a capture is just "save the main
buffer of the next request" and never interrupts the stream.

Stills land in `survey_dir/<session timestamp>/<capture timestamp>.jpg`, with a
`.json` beside each holding the robot state at the moment of capture.
"""

from __future__ import annotations

import io
import json
import logging
import os
import threading
import time
from datetime import datetime
from typing import Any, Dict, Optional, Tuple

from .config import CameraConfig

log = logging.getLogger(__name__)

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
PLACEHOLDER = os.path.join(STATIC_DIR, "no-camera.jpg")
STREAM_QUALITY = 70  # JPEG quality of the live view when a software encoder is used
STALE_AFTER_S = 2.0  # no frame for this long and the stream counts as down


class CameraError(RuntimeError):
    pass


class FrameBuffer(io.BufferedIOBase):
    """Latest encoded frame. The encoder writes; stream readers wait for a newer one."""

    def __init__(self) -> None:
        self._frame = b""
        self._seq = 0
        self._cond = threading.Condition()
        self.updated_at = 0.0

    def writable(self) -> bool:
        return True

    def write(self, buf) -> int:  # type: ignore[override]
        data = bytes(buf)
        with self._cond:
            self._frame = data
            self._seq += 1
            self.updated_at = time.monotonic()
            self._cond.notify_all()
        return len(data)

    @property
    def seq(self) -> int:
        with self._cond:
            return self._seq

    def latest(self) -> Tuple[int, bytes]:
        with self._cond:
            return self._seq, self._frame

    def wait(self, after: int, timeout: float) -> Optional[Tuple[int, bytes]]:
        """Block until a frame newer than `after` exists. None on timeout."""
        with self._cond:
            if not self._cond.wait_for(lambda: self._seq > after, timeout):
                return None
            return self._seq, self._frame


def _pick_encoder(config: CameraConfig):
    """Hardware MJPEG on Pi 4 and earlier. Pi 5 has no hardware JPEG encoder, so
    software JPEG on an RGB lores stream there. Returns (encoder, lores format)."""
    try:
        from picamera2.platform import Platform, get_platform

        pisp = get_platform() == Platform.PISP
    except Exception:
        pisp = False
    if pisp:
        from picamera2.encoders import JpegEncoder

        return JpegEncoder(q=STREAM_QUALITY), "RGB888"
    from picamera2.encoders import MJPEGEncoder

    return MJPEGEncoder(), "YUV420"


class Camera:
    def __init__(self, config: CameraConfig):
        self.config = config
        self.frames = FrameBuffer()
        self.error: Optional[str] = None
        self.captures = 0
        self.last_capture: Optional[str] = None
        self._open = False
        self._picam = None
        self._capture_lock = threading.Lock()
        self._session_dir: Optional[str] = None
        self._started_at = datetime.now()

    # --- lifecycle ----------------------------------------------------------------

    @property
    def ok(self) -> bool:
        return self._open and self.error is None

    @property
    def streaming(self) -> bool:
        return self.ok and self.frames.updated_at > 0 and time.monotonic() - self.frames.updated_at < STALE_AFTER_S

    def open(self) -> None:
        try:
            from picamera2 import Picamera2
            from picamera2.outputs import FileOutput
        except ImportError as exc:
            raise CameraError(
                "picamera2 is not importable. It is apt-installed, so create the venv with "
                "`python3 -m venv --system-site-packages .venv`"
            ) from exc

        encoder, lores_format = _pick_encoder(self.config)
        picam = Picamera2()
        try:
            video = picam.create_video_configuration(
                main={"size": tuple(self.config.still), "format": "RGB888"},
                lores={"size": tuple(self.config.lores), "format": lores_format},
                buffer_count=self.config.buffers,
                controls={"FrameRate": self.config.stream_fps},
            )
            picam.configure(video)
            picam.options["quality"] = self.config.jpeg_quality
            picam.start_encoder(encoder, FileOutput(self.frames), name="lores")
            picam.start()
        except Exception:
            picam.close()
            raise
        self._picam = picam
        self._open = True
        self.error = None
        log.info("camera up: live view %dx%d via %s, stills %dx%d",
                 *self.config.lores, type(encoder).__name__, *self.config.still)

    def close(self) -> None:
        self._open = False
        picam, self._picam = self._picam, None
        if picam is None:
            return
        for step in (picam.stop_encoder, picam.stop, picam.close):
            try:
                step()
            except Exception:
                log.debug("camera shutdown step %s failed", step.__name__, exc_info=True)

    def __enter__(self) -> "Camera":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # --- stills -------------------------------------------------------------------

    def session_dir(self) -> str:
        """One directory per server run, created on the first capture."""
        if self._session_dir is None:
            root = os.path.expanduser(self.config.survey_dir)
            path = os.path.join(root, self._started_at.strftime("%Y%m%d-%H%M%S"))
            os.makedirs(path, exist_ok=True)
            self._session_dir = path
        return self._session_dir

    def capture(self, metadata: Optional[Dict[str, Any]] = None) -> str:
        """Save a full-resolution still. Blocks for about one frame. Returns the path."""
        if not self.ok:
            raise CameraError(self.error or "camera is not open")
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")[:-3]
        path = os.path.join(self.session_dir(), f"{stamp}.jpg")
        with self._capture_lock:
            self._save_still(path)
        if metadata is not None:
            with open(path[:-4] + ".json", "w") as handle:
                json.dump(metadata, handle, indent=1, default=str)
        self.captures += 1
        self.last_capture = path
        log.info("captured %s", path)
        return path

    def _save_still(self, path: str) -> None:
        self._picam.capture_file(path, name="main")


class FakeCamera(Camera):
    """No camera. Pushes a placeholder frame at the stream rate; stills save the same image."""

    def open(self) -> None:
        with open(PLACEHOLDER, "rb") as handle:
            self._frame = handle.read()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="camera-fake", daemon=True)
        self._open = True
        self.error = None
        self._thread.start()
        log.info("dry run: placeholder camera")

    def close(self) -> None:
        self._open = False
        if getattr(self, "_thread", None) is not None:
            self._stop.set()
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run(self) -> None:
        period = 1.0 / max(self.config.stream_fps, 1.0)
        while not self._stop.wait(period):
            self.frames.write(self._frame)

    def _save_still(self, path: str) -> None:
        with open(path, "wb") as handle:
            handle.write(self._frame)
