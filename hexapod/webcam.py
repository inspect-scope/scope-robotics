"""Laptop webcam as a Camera. Optional USB view for /hand/stream. The robot camera stays Picamera2."""

from __future__ import annotations

import logging
import threading
from typing import Any, Dict, Optional

from .camera import Camera, CameraError

log = logging.getLogger(__name__)


class WebcamCamera(Camera):
    """OpenCV VideoCapture -> JPEG. Draws a hand skeleton when set_mark is given a view."""

    def __init__(self, config, index: int = 0):
        super().__init__(config)
        self.index = index
        self._cap = None
        self._bgr = None
        self._mark: Optional[Dict[str, Any]] = None
        self._frame_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def open(self) -> None:
        try:
            import cv2
        except ImportError as exc:
            raise CameraError("opencv-python is not installed; pip install opencv-python") from exc

        cap = cv2.VideoCapture(self.index)
        if not cap.isOpened():
            cap.release()
            raise CameraError(f"webcam {self.index} would not open")
        width, height = self.config.lores
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap = cap
        self._stop.clear()
        self._open = True
        self.error = None
        self._thread = threading.Thread(target=self._run, name="webcam", daemon=True)
        self._thread.start()
        log.info("webcam %s up for hand preview, asked for %dx%d", self.index, width, height)

    def close(self) -> None:
        self._open = False
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def latest_bgr(self):
        with self._frame_lock:
            return None if self._bgr is None else self._bgr.copy()

    def set_mark(self, mark: Optional[Dict[str, Any]]) -> None:
        with self._frame_lock:
            self._mark = mark

    def _save_still(self, path: str) -> None:
        import cv2

        frame = self.latest_bgr()
        if frame is None:
            raise CameraError("no webcam frame yet")
        if not cv2.imwrite(path, frame):
            raise CameraError(f"failed to write {path}")

    def _run(self) -> None:
        import cv2

        period = 1.0 / max(self.config.stream_fps, 1.0)
        while not self._stop.wait(period):
            ok, frame = self._cap.read()
            if not ok or frame is None:
                continue
            frame = cv2.flip(frame, 1)
            with self._frame_lock:
                self._bgr = frame
                mark = self._mark
            if mark:
                frame = _draw_hand(frame.copy(), mark)
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.config.jpeg_quality])
            if ok:
                self.frames.write(buf.tobytes())


def _draw_hand(frame, mark: Dict[str, Any]):
    import cv2

    h, w = frame.shape[:2]
    points = mark.get("points") or []
    px = []
    for pt in points:
        px.append((int(pt[0] * w), int(pt[1] * h)))
    color = (0, 163, 12)
    for a, b in mark.get("connections") or []:
        if a < len(px) and b < len(px):
            cv2.line(frame, px[a], px[b], color, 2)
    for i, pt in enumerate(px):
        cv2.circle(frame, pt, 5 if i == 0 else 3, color, -1)
    pose = mark.get("pose") or {}
    src = mark.get("source") or ""
    text = f"{src}  x {pose.get('x', 0):+.1f}  y {pose.get('y', 0):+.1f}  r {pose.get('roll', 0):+.1f}  p {pose.get('pitch', 0):+.1f}  yaw {pose.get('yaw', 0):+.1f}"
    cv2.putText(frame, text, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    return frame
