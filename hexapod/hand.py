"""Map a seen hand onto body pose, inside the operator limits.

The browser WASM landmarker posts 21 MediaPipe points. USB frames fall back
to a skin blob. Height is left on the slider. Preview only.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from .controller import (
    POSE_LIMITS, PINCER_FWD_MM, PINCER_LEGS, PINCER_LIFT_MM, PINCER_SIDE_MM,
    Controller,
)
from .kinematics import BodyPose

log = logging.getLogger(__name__)

DEADZONE = 0.18
SMOOTH = 0.25
LOST_AFTER_S = 0.4
LANDMARKS = 21
PINCER_GAIN = 1.6
PINCER_SPAN_MIN = 0.04
TILT_SPAN_DEG = 45.0
POINT = Tuple[float, float, float]

# MediaPipe Hands: wrist, then thumb / index / middle / ring / pinky.
HAND_BONES: Tuple[Tuple[int, int], ...] = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
)

WRIST = 0
INDEX_MCP, INDEX_TIP = 5, 8
MIDDLE_MCP, MIDDLE_TIP = 9, 12
RING_MCP = 13
PINKY_MCP = 17
PALM = (WRIST, INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP)


class Follow:
    Off = "off"
    On = "on"


class Sense:
    MediaPipe = "mediapipe"
    Blob = "blob"


class Puppet:
    Lean = "lean"
    Pincer = "pincer"


@dataclass(frozen=True)
class HandBlob:
    """Normalised selfie frame. +x is the viewer's right (robot -x after map)."""

    nx: float
    ny: float
    tilt: float
    area: float


@dataclass
class HandView:
    source: str
    pose: BodyPose
    points: List[POINT]
    connections: List[Tuple[int, int]]
    puppet: str = Puppet.Lean
    pincers: Optional[dict] = None

    def as_dict(self) -> dict:
        return {
            "source": self.source,
            "pose": asdict(self.pose),
            "points": [list(p) for p in self.points],
            "connections": [list(c) for c in self.connections],
            "mode": self.puppet,
            "pincers": self.pincers,
        }


def _dead(value: float) -> float:
    if abs(value) < DEADZONE:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * min(1.0, (abs(value) - DEADZONE) / (1.0 - DEADZONE))


def _unit(value: float) -> float:
    return max(-1.0, min(1.0, value))


def _as_points(raw) -> Optional[List[POINT]]:
    if raw is None or len(raw) < LANDMARKS:
        return None
    return [(float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0) for p in raw]


def _palm_xy(points: Sequence[POINT]) -> Tuple[float, float]:
    palm = [points[i] for i in PALM]
    return sum(p[0] for p in palm) / 5.0, sum(p[1] for p in palm) / 5.0


def _rest_pincers() -> Dict[str, Tuple[float, float, float]]:
    return {name: (0.0, 0.0, 0.0) for name in PINCER_LEGS}


def _mix(current: float, target: float) -> float:
    return current + (target - current) * SMOOTH


def pose_from_blob(blob: HandBlob) -> BodyPose:
    """Lean toward the hand. Mirror x so it reads like a mirror."""

    nx = _dead(blob.nx)
    ny = _dead(blob.ny)
    tilt = _dead(blob.tilt)
    shift = POSE_LIMITS["shift"]
    return BodyPose(
        x=-nx * shift,
        y=ny * shift,
        roll=tilt * POSE_LIMITS["roll"],
        pitch=ny * POSE_LIMITS["pitch"],
        yaw=-nx * POSE_LIMITS["yaw"],
    )


def pose_from_hand(points: Sequence[POINT], world: Optional[Sequence[POINT]] = None) -> BodyPose:
    if len(points) < LANDMARKS:
        raise ValueError(f"need {LANDMARKS} MediaPipe landmarks")
    cx, cy = _palm_xy(points)
    nx = _dead((cx - 0.5) * 2.0)
    ny = _dead((0.5 - cy) * 2.0)

    basis = world if world is not None and len(world) >= LANDMARKS else points
    normal = _palm_normal(basis)
    # Image y is down. A palm facing the camera has +z toward us.
    roll_n = _dead(math.degrees(math.atan2(normal[0], max(abs(normal[2]), 1e-6))) / TILT_SPAN_DEG)
    pitch_n = _dead(math.degrees(math.atan2(-normal[1], max(abs(normal[2]), 1e-6))) / TILT_SPAN_DEG)
    dx = points[MIDDLE_MCP][0] - points[WRIST][0]
    dy = points[WRIST][1] - points[MIDDLE_MCP][1]
    yaw_n = _dead(math.atan2(dx, dy) / (math.pi / 2.0))

    shift = POSE_LIMITS["shift"]
    return BodyPose(
        x=-nx * shift,
        y=ny * shift,
        roll=roll_n * POSE_LIMITS["roll"],
        pitch=pitch_n * POSE_LIMITS["pitch"],
        yaw=-yaw_n * POSE_LIMITS["yaw"],
    )


def pincers_from_hand(points: Sequence[POINT]) -> Dict[str, Tuple[float, float, float]]:
    """Left-of-frame tip → L1, right-of-frame tip → R1. Lift when the tip is above the palm."""

    cx, cy = _palm_xy(points)
    span = max(PINCER_SPAN_MIN, math.hypot(
        points[INDEX_MCP][0] - points[PINKY_MCP][0],
        points[INDEX_MCP][1] - points[PINKY_MCP][1],
    ))
    index, middle = points[INDEX_TIP], points[MIDDLE_TIP]
    left, right = (index, middle) if index[0] <= middle[0] else (middle, index)

    def offset(tip: POINT) -> Tuple[float, float, float]:
        nx = _unit((tip[0] - cx) / span * PINCER_GAIN)
        ny = _unit((cy - tip[1]) / span * PINCER_GAIN)
        return (nx * PINCER_SIDE_MM, ny * PINCER_FWD_MM, max(0.0, ny) * PINCER_LIFT_MM)

    return {"L1": offset(left), "R1": offset(right)}


def _palm_normal(points: Sequence[POINT]) -> POINT:
    wx, wy, wz = points[WRIST]
    ix, iy, iz = points[INDEX_MCP]
    px, py, pz = points[PINKY_MCP]
    v1 = (ix - wx, iy - wy, iz - wz)
    v2 = (px - wx, py - wy, pz - wz)
    n = (
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    )
    mag = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2) or 1.0
    return (n[0] / mag, n[1] / mag, n[2] / mag)


def blob_from_mask(mask, width: int, height: int) -> Optional[HandBlob]:
    import cv2

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < 0.01 * width * height:
        return None
    if len(contour) >= 5:
        (cx, cy), _, angle = cv2.fitEllipse(contour)
    else:
        (cx, cy), _, angle = cv2.minAreaRect(contour)
    tilt = math.sin(math.radians(angle))
    return HandBlob(
        nx=(cx / width - 0.5) * 2.0,
        ny=(0.5 - cy / height) * 2.0,
        tilt=_unit(tilt),
        area=area / (width * height),
    )


def skin_mask(bgr):
    import cv2

    ycrcb = cv2.cvtColor(bgr, cv2.COLOR_BGR2YCrCb)
    mask = cv2.inRange(ycrcb, (0, 133, 77), (255, 173, 127))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def read_hand(bgr) -> Optional[HandView]:
    """USB-cam fallback: a skin blob. The browser posts MediaPipe landmarks."""

    try:
        mask = skin_mask(bgr)
        blob = blob_from_mask(mask, bgr.shape[1], bgr.shape[0])
    except Exception:
        return None
    if blob is None:
        return None
    return HandView(
        source=Sense.Blob,
        pose=pose_from_blob(blob),
        points=[(blob.nx * 0.5 + 0.5, 0.5 - blob.ny * 0.5, 0.0)],
        connections=[],
    )


class HandFollower:
    """Accepts browser landmarks or an optional USB cam. Off until set_follow(On)."""

    def __init__(self, controller: Controller, camera=None):
        self.controller = controller
        self.camera = camera
        self.follow = Follow.Off
        self.tracking = False
        self.error: Optional[str] = None
        self.source: Optional[str] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._pose = BodyPose()
        self._points: List[POINT] = []
        self._connections: List[Tuple[int, int]] = []
        self._puppet = Puppet.Lean
        self._pincers = _rest_pincers()
        self._seen_at = 0.0

    def start(self) -> None:
        # Python MediaPipe 1.0 aborts on this Mac (Metal). Do not start it here.
        if self.camera is None:
            return
        try:
            self.camera.open()
        except Exception as exc:
            log.warning("hand usb camera unavailable: %s", exc)
            self.camera.error = f"{type(exc).__name__}: {exc}"
            return
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="hand-follow", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self.camera is not None:
            self.camera.close()

    def set_follow(self, state: str) -> None:
        if state not in (Follow.On, Follow.Off):
            raise ValueError(f"unknown follow state {state!r}")
        with self._lock:
            self.follow = state
        if state == Follow.Off:
            self._park()

    def set_puppet(self, state: str) -> None:
        if state not in (Puppet.Lean, Puppet.Pincer):
            raise ValueError(f"unknown puppet {state!r}")
        with self._lock:
            self._puppet = state
        if self.follow == Follow.On:
            self._apply()

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "follow": self.follow,
                "tracking": self.tracking,
                "error": self.error,
                "source": self.source,
                "pose": asdict(self._pose),
                "points": [list(p) for p in self._points],
                "connections": [list(c) for c in self._connections],
                "mode": self._puppet,
                "pincers": {name: list(delta) for name, delta in self._pincers.items()},
            }

    def see_landmarks(self, points, world=None) -> dict:
        pts = _as_points(points)
        if pts is None:
            return self.snapshot()
        wts = _as_points(world)
        with self._lock:
            puppet = self._puppet
        return self._ingest(HandView(
            source=Sense.MediaPipe,
            pose=pose_from_hand(pts, wts),
            points=pts,
            connections=list(HAND_BONES),
            puppet=puppet,
            pincers=pincers_from_hand(pts) if puppet == Puppet.Pincer else None,
        ))

    def see_jpeg(self, data: bytes) -> dict:
        import cv2
        import numpy as np

        frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return self.snapshot()
        return self.see_bgr(frame)

    def see_bgr(self, bgr) -> dict:
        try:
            view = read_hand(bgr)
        except Exception as exc:
            with self._lock:
                self.error = f"{type(exc).__name__}: {exc}"
            return self.snapshot()
        return self._ingest(view)

    def _ingest(self, view: Optional[HandView]) -> dict:
        now = time.monotonic()
        if view is None:
            with self._lock:
                lost = now - self._seen_at > LOST_AFTER_S
                self.tracking = False
                self.source = None
                self._points = []
                self._connections = []
                following = self.follow == Follow.On
            if lost and following:
                self._ease_to(BodyPose())
                self._ease_pincers(_rest_pincers())
                self._apply()
            self._paint(None)
            return self.snapshot()

        self._ease_to(view.pose)
        self._ease_pincers(view.pincers or _rest_pincers())
        with self._lock:
            self.tracking = True
            self._seen_at = now
            self.error = None
            self.source = view.source
            self._points = list(view.points)
            self._connections = list(view.connections)
            following = self.follow == Follow.On
        self._paint(view)
        if following:
            self._apply()
        return self.snapshot()

    def _park(self) -> None:
        self.controller.set_pose(x=0, y=0, roll=0, pitch=0, yaw=0)
        self.controller.set_pincers(None)

    def _apply(self) -> None:
        if self._puppet == Puppet.Pincer:
            self.controller.set_pose(x=0, y=0, roll=0, pitch=0, yaw=0)
            self.controller.set_pincers(self._pincers)
            return
        self.controller.set_pincers(None)
        pose = self._pose
        self.controller.set_pose(x=pose.x, y=pose.y, roll=pose.roll, pitch=pose.pitch, yaw=pose.yaw)

    def _paint(self, view: Optional[HandView]) -> None:
        mark = getattr(self.camera, "set_mark", None)
        if mark is None:
            return
        mark(None if view is None else view.as_dict())

    def _run(self) -> None:
        period = 1.0 / 20.0
        while not self._stop.wait(period):
            grab = getattr(self.camera, "latest_bgr", None)
            if grab is None:
                continue
            frame = grab()
            if frame is not None:
                self.see_bgr(frame)

    def _ease_to(self, target: BodyPose) -> None:
        pose = self._pose
        self._pose = BodyPose(
            x=_mix(pose.x, target.x),
            y=_mix(pose.y, target.y),
            roll=_mix(pose.roll, target.roll),
            pitch=_mix(pose.pitch, target.pitch),
            yaw=_mix(pose.yaw, target.yaw),
        )

    def _ease_pincers(self, target: Dict[str, Tuple[float, float, float]]) -> None:
        out: Dict[str, Tuple[float, float, float]] = {}
        for name in PINCER_LEGS:
            cur = self._pincers.get(name, (0.0, 0.0, 0.0))
            tgt = target.get(name, (0.0, 0.0, 0.0))
            out[name] = (_mix(cur[0], tgt[0]), _mix(cur[1], tgt[1]), _mix(cur[2], tgt[2]))
        self._pincers = out
