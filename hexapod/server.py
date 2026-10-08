"""HTTP + WebSocket interface, served on the local network.

Two websockets. `/ws` carries commands in and state out for the client page; it
is the only stateful path, every message is a complete command, and the
controller expires the web source's velocity 0.5 s after the last one, so a
dropped link stops the robot on its own. `/telemetry` is state only, for the
status panel and anything else that just watches.

`/move` and `/stop` do the same over plain HTTP for scripts and other clients.
A `/move` command has the same 0.5 s lifetime: keep posting or the robot halts.
"""

from __future__ import annotations

import asyncio
import logging
import os
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .board import BoardError
from .camera import Camera, CameraError
from .config import Config
from .controller import COMMAND_TTL, POSE_LIMITS, Controller
from .gait import TrickKind, WALK_KINDS, walk_catalog, trick_catalog, Velocity
from .hand import Follow
from .mode import MODE_KINDS, mode_catalog
from .state import RobotState

log = logging.getLogger(__name__)

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
WEB_SOURCE = "web"
REST_SOURCE = "rest"
WEB_PRIORITY = 10
STATE_HZ = 30.0
STREAM_BOUNDARY = b"frame"

# `/move` directions as normalised (vx, vy, yaw) axes in the body frame.
DIRECTIONS: Dict[str, tuple] = {
    "forward": (0.0, 1.0, 0.0),
    "back": (0.0, -1.0, 0.0),
    "backward": (0.0, -1.0, 0.0),
    "left": (-1.0, 0.0, 0.0),
    "right": (1.0, 0.0, 0.0),
    "turn_left": (0.0, 0.0, 1.0),
    "turn_right": (0.0, 0.0, -1.0),
    "stop": (0.0, 0.0, 0.0),
}


class HandLandmarks(BaseModel):
    points: List[List[float]]
    world: Optional[List[List[float]]] = None


class MoveRequest(BaseModel):
    """Either a named direction with a speed, or raw -1..1 axes."""

    direction: Optional[str] = None
    speed: float = 1.0
    vx: Optional[float] = None
    vy: Optional[float] = None
    yaw: Optional[float] = None


HAND_DISABLED = "hand follow is not enabled"


def _hand(state: RobotState):
    if state.hand is None:
        raise HTTPException(404, HAND_DISABLED)
    return state.hand


def _clamp_axis(value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return max(-1.0, min(1.0, number))


def _velocity_from(move: MoveRequest) -> Velocity:
    if move.direction is not None:
        try:
            vx, vy, yaw = DIRECTIONS[move.direction.lower()]
        except KeyError:
            raise HTTPException(400, f"unknown direction {move.direction!r}; one of {', '.join(DIRECTIONS)}")
        speed = max(0.0, min(1.0, float(move.speed)))
        return Velocity(vx=vx * speed, vy=vy * speed, yaw_rate=yaw * speed)
    if move.vx is None and move.vy is None and move.yaw is None:
        raise HTTPException(400, "give a direction or at least one of vx, vy, yaw")
    return Velocity(vx=_clamp_axis(move.vx), vy=_clamp_axis(move.vy), yaw_rate=_clamp_axis(move.yaw))


def create_app(state: RobotState, config: Config) -> FastAPI:
    controller = state.controller
    app = FastAPI(title="scope-hexapod", docs_url=None, redoc_url=None)
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
    last_move: Dict[str, Any] = {"text": None}

    # --- pages ------------------------------------------------------------------

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(os.path.join(STATIC_DIR, "index.html"))

    @app.get("/status")
    async def status_page() -> FileResponse:
        return FileResponse(os.path.join(STATIC_DIR, "status.html"))

    # --- state --------------------------------------------------------------------

    @app.get("/api/state")
    async def api_state() -> JSONResponse:
        return JSONResponse(state.snapshot())

    @app.get("/api/config")
    async def config_view() -> JSONResponse:
        s = config.stance
        g = config.geometry
        return JSONResponse(
            {
                "pose_limits": POSE_LIMITS,
                "height": {"min": s.sit_height, "max": 105.0, "default": s.ride_height},
                "gait": {
                    "cycle_s": s.cycle_s,
                    "step_lift": s.step_lift,
                    "max_speed": s.max_speed,
                    "pattern": state.controller.gait.pattern,
                    "patterns": list(WALK_KINDS),
                    "catalog": walk_catalog(config.tripod_groups),
                    "tricks": [TrickKind.Bounce, TrickKind.Jump],
                    "catalog_tricks": trick_catalog(),
                },
                "mode": {
                    "current": state.controller.mode,
                    "kinds": list(MODE_KINDS),
                    "catalog": mode_catalog(state.controller.modes),
                },
                "legs": list(config.leg_order),
                "coxae": {name: list(leg.coxa_xy) for name, leg in config.legs.items()},
                "yaws": {name: leg.yaw_deg for name, leg in config.legs.items()},
                "geometry": {
                    "coxa_len": g.coxa_len,
                    "femur_len": g.femur_len,
                    "tibia_len": g.tibia_len,
                    "leg_connection_z": g.leg_connection_z,
                },
                "shell": {
                    "floor_xy": list(config.shell.floor_xy),
                    "chamfer": config.shell.chamfer,
                    "frame_h": config.shell.frame_h,
                    "roof_z": config.shell.roof_z,
                    "cam_tilt": config.shell.cam_tilt,
                },
                "pulse_us": list(config.limits.pulse_us),
                "servos": {
                    name: {joint: {"channel": cal.channel, "direction": cal.direction}
                           for joint, cal in leg.servos.items()}
                    for name, leg in config.legs.items()
                },
                "camera": {"enabled": state.camera is not None, "lores": list(config.camera.lores),
                           "still": list(config.camera.still)},
                "hand": state.hand is not None,
                "panel": {"front": config.panel.front},
            }
        )

    # --- motion -------------------------------------------------------------------

    @app.post("/stop")
    @app.post("/api/estop")
    async def stop() -> JSONResponse:
        """Latches torque off and clears every velocity source. Reachable without a
        websocket, so a wedged page is never the only way to stop."""
        controller.estop()
        log.warning("stop requested over http")
        return JSONResponse({"ok": True, "state": controller.snapshot().state})

    @app.post("/move")
    async def move(request: MoveRequest) -> JSONResponse:
        velocity = _velocity_from(request)
        controller.command(REST_SOURCE, velocity, priority=WEB_PRIORITY)
        snapshot = controller.snapshot()
        text = f"{request.direction or 'axes'} vx={velocity.vx:+.2f} vy={velocity.vy:+.2f} yaw={velocity.yaw_rate:+.2f}"
        if text != last_move["text"]:
            log.info("move %s (robot %s)", text, snapshot.state)
            last_move["text"] = text
        return JSONResponse(
            {
                "ok": True,
                "velocity": {"vx": velocity.vx, "vy": velocity.vy, "yaw": velocity.yaw_rate},
                "expires_in_s": COMMAND_TTL,
                "state": snapshot.state,
                "moving": snapshot.state == "standing" or snapshot.state == "walking",
            }
        )

    # --- camera -------------------------------------------------------------------

    def _camera() -> Camera:
        camera = state.camera
        if camera is None or not camera.ok:
            raise HTTPException(503, (camera.error if camera else None) or "camera not available")
        return camera

    @app.get("/stream")
    async def stream() -> StreamingResponse:
        camera = _camera()
        return StreamingResponse(
            _mjpeg(camera),
            media_type=f"multipart/x-mixed-replace; boundary={STREAM_BOUNDARY.decode()}",
            headers={"Cache-Control": "no-cache, no-store, must-revalidate", "Pragma": "no-cache"},
        )

    @app.get("/hand/stream")
    async def hand_stream() -> StreamingResponse:
        camera = getattr(_hand(state), "camera", None)
        if camera is None or not camera.ok:
            raise HTTPException(503, "hand camera not available")
        return StreamingResponse(
            _mjpeg(camera),
            media_type=f"multipart/x-mixed-replace; boundary={STREAM_BOUNDARY.decode()}",
            headers={"Cache-Control": "no-cache, no-store, must-revalidate", "Pragma": "no-cache"},
        )

    @app.post("/api/hand-frame")
    async def hand_frame(request: Request) -> JSONResponse:
        data = await request.body()
        if not data:
            raise HTTPException(400, "empty frame")
        loop = asyncio.get_running_loop()
        view = await loop.run_in_executor(None, _hand(state).see_jpeg, data)
        return JSONResponse(view)

    @app.post("/api/hand-landmarks")
    async def hand_landmarks(body: HandLandmarks) -> JSONResponse:
        return JSONResponse(_hand(state).see_landmarks(body.points, body.world))

    @app.post("/capture")
    async def capture() -> JSONResponse:
        camera = _camera()
        metadata = state.snapshot()
        loop = asyncio.get_running_loop()
        try:
            path = await loop.run_in_executor(None, camera.capture, metadata)
        except CameraError as exc:
            raise HTTPException(503, str(exc))
        except Exception as exc:
            log.exception("capture failed")
            raise HTTPException(500, f"capture failed: {type(exc).__name__}: {exc}")
        return JSONResponse({"ok": True, "path": path, "bytes": os.path.getsize(path), "captures": camera.captures})

    # --- websockets ---------------------------------------------------------------

    @app.websocket("/ws")
    async def ws(socket: WebSocket) -> None:
        await socket.accept()
        peer = socket.client.host if socket.client else "?"
        log.info("client connected: %s", peer)
        pusher = asyncio.create_task(_push_state(socket, state))
        try:
            while True:
                message = await socket.receive_json()
                _handle(controller, config, message, hand=state.hand)
        except WebSocketDisconnect:
            log.info("client disconnected: %s", peer)
        except Exception:
            log.exception("websocket error")
        finally:
            pusher.cancel()
            # Let the command expire rather than stopping hard: a phone that drops
            # off wifi mid-stride should coast to a halt, not drop on its face.
            controller.drop_source(WEB_SOURCE)

    @app.websocket("/telemetry")
    async def telemetry(socket: WebSocket) -> None:
        """State only. Incoming messages are read and dropped so a close is noticed."""
        await socket.accept()
        pusher = asyncio.create_task(_push_state(socket, state))
        try:
            while True:
                await socket.receive_text()
        except WebSocketDisconnect:
            pass
        except Exception:
            log.debug("telemetry socket closed", exc_info=True)
        finally:
            pusher.cancel()

    return app


async def _push_state(socket: WebSocket, state: RobotState) -> None:
    period = 1.0 / STATE_HZ
    try:
        while True:
            await socket.send_json({"type": "state", **state.snapshot()})
            await asyncio.sleep(period)
    except asyncio.CancelledError:
        raise
    except Exception:
        log.debug("state push stopped", exc_info=True)


async def _mjpeg(camera: Camera):
    """multipart/x-mixed-replace body: one JPEG per part, forever."""
    loop = asyncio.get_running_loop()
    seq = 0
    while True:
        got = await loop.run_in_executor(None, camera.frames.wait, seq, 1.0)
        if got is None:
            if not camera.ok:
                return
            continue
        seq, frame = got
        yield (b"--" + STREAM_BOUNDARY + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
               + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")


def _handle(controller: Controller, config: Config, message: Dict[str, Any], hand=None) -> None:
    kind = message.get("type")
    if kind == "drive":
        controller.command(
            WEB_SOURCE,
            Velocity(
                vx=_clamp_axis(message.get("vx")),
                vy=_clamp_axis(message.get("vy")),
                yaw_rate=_clamp_axis(message.get("yaw")),
            ),
            priority=WEB_PRIORITY,
        )
    elif kind == "pose":
        fields = {key: message[key] for key in ("x", "y", "roll", "pitch", "yaw") if key in message}
        if fields:
            controller.set_pose(**fields)
    elif kind == "height":
        controller.set_height(message.get("value", config.stance.ride_height))
    elif kind == "stand":
        try:
            controller.stand()
        except BoardError as exc:  # offline or estop latched: the page already shows why
            log.warning("stand refused: %s", exc)
    elif kind == "sit":
        controller.sit()
    elif kind == "torque":
        if message.get("on"):
            try:
                controller.board.set_torque(True)
            except BoardError as exc:
                log.warning("torque refused: %s", exc)
        else:
            controller.torque_off()
    elif kind == "estop":
        controller.estop()
    elif kind == "clear_estop":
        controller.clear_estop()
    elif kind == "gait":
        if "pattern" in message:
            try:
                controller.set_pattern(str(message["pattern"]))
            except ValueError:
                log.warning("bad gait pattern %r", message["pattern"])
        _tune(config, message)
    elif kind == "mode":
        name = message.get("name", message.get("mode"))
        try:
            controller.set_mode(str(name))
        except ValueError:
            log.warning("bad stance mode %r", name)
    elif kind == "jump":
        controller.jump()
    elif kind == "bounce":
        controller.set_bounce(bool(message.get("on")))
    elif kind == "trick":
        name = message.get("name")
        if not message.get("on", True) or not name:
            controller.stop_trick()
        else:
            try:
                controller.start_trick(str(name))
            except ValueError:
                log.warning("bad trick %r", name)
    elif kind == "hand":
        if hand is None:
            log.warning("hand follow is not available")
        else:
            if "on" in message:
                try:
                    hand.set_follow(Follow.On if message.get("on") else Follow.Off)
                except ValueError:
                    log.warning("bad hand follow %r", message.get("on"))
            if "puppet" in message:
                try:
                    hand.set_puppet(str(message["puppet"]))
                except ValueError:
                    log.warning("bad hand puppet %r", message.get("puppet"))
    elif kind == "ping":
        pass
    else:
        log.warning("ignoring unknown message type %r", kind)


def _tune(config: Config, message: Dict[str, Any]) -> None:
    """Live gait tuning. Ranges are what the machine survives, not what it likes."""
    bounds = {"cycle_s": (0.3, 2.5), "step_lift": (5.0, 70.0), "max_speed": (10.0, 250.0),
              "max_yaw_rate": (5.0, 90.0)}
    for key, (low, high) in bounds.items():
        if key in message:
            try:
                setattr(config.stance, key, max(low, min(high, float(message[key]))))
            except (TypeError, ValueError):
                log.warning("bad gait value for %s: %r", key, message[key])
