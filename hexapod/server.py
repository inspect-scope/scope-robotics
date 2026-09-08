"""HTTP + WebSocket interface, served on the local network.

The websocket carries control in one direction and state in the other. It is
deliberately the only stateful path: every message is a complete command, so a
dropped connection needs no resync, and the controller stops on its own once the
web source's commands go stale.
"""

from __future__ import annotations

import asyncio
import logging
import os
from typing import Any, Dict, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from .config import Config
from .controller import POSE_LIMITS, Controller
from .gait import Velocity

log = logging.getLogger(__name__)

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
WEB_SOURCE = "web"
WEB_PRIORITY = 10
STATE_HZ = 10.0


def _clamp_axis(value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return max(-1.0, min(1.0, number))


def create_app(controller: Controller, config: Config) -> FastAPI:
    app = FastAPI(title="scope-hexapod", docs_url=None, redoc_url=None)
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(os.path.join(STATIC_DIR, "index.html"))

    @app.get("/api/state")
    async def state() -> JSONResponse:
        return JSONResponse(controller.snapshot().__dict__)

    @app.get("/api/config")
    async def config_view() -> JSONResponse:
        s = config.stance
        return JSONResponse(
            {
                "pose_limits": POSE_LIMITS,
                "height": {"min": s.sit_height, "max": 105.0, "default": s.ride_height},
                "gait": {"cycle_s": s.cycle_s, "step_lift": s.step_lift, "max_speed": s.max_speed},
                "legs": list(config.leg_order),
                "coxae": {name: list(leg.coxa_xy) for name, leg in config.legs.items()},
            }
        )

    @app.post("/api/estop")
    async def estop() -> JSONResponse:
        """Reachable without the websocket, so a wedged UI is never the only way to stop."""
        controller.estop()
        return JSONResponse({"ok": True, "state": controller.snapshot().state})

    @app.websocket("/ws")
    async def ws(socket: WebSocket) -> None:
        await socket.accept()
        peer = socket.client.host if socket.client else "?"
        log.info("client connected: %s", peer)
        pusher = asyncio.create_task(_push_state(socket, controller))
        try:
            while True:
                message = await socket.receive_json()
                _handle(controller, config, message)
        except WebSocketDisconnect:
            log.info("client disconnected: %s", peer)
        except Exception:
            log.exception("websocket error")
        finally:
            pusher.cancel()
            # Let the command expire rather than stopping hard: a phone that drops
            # off wifi mid-stride should coast to a halt, not drop on its face.
            controller.drop_source(WEB_SOURCE)
    return app


async def _push_state(socket: WebSocket, controller: Controller) -> None:
    period = 1.0 / STATE_HZ
    try:
        while True:
            await socket.send_json({"type": "state", **controller.snapshot().__dict__})
            await asyncio.sleep(period)
    except asyncio.CancelledError:
        raise
    except Exception:
        log.debug("state push stopped", exc_info=True)


def _handle(controller: Controller, config: Config, message: Dict[str, Any]) -> None:
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
        controller.stand()
    elif kind == "sit":
        controller.sit()
    elif kind == "torque":
        if message.get("on"):
            controller.board.set_torque(True)
        else:
            controller.torque_off()
    elif kind == "estop":
        controller.estop()
    elif kind == "clear_estop":
        controller.clear_estop()
    elif kind == "gait":
        _tune(config, message)
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
