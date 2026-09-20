"""One object that every frontend reads from.

Nothing in the web layer touches hardware. The board thread already caches
volts, amps and contacts; the controller caches gait state. This adds the IMU,
polled on its own thread at 10 Hz, and the camera's health, and merges them into
one snapshot for `/api/state`, both websockets and the still sidecar files.

The IMU and camera are opened here, not by the caller, so a missing sensor is a
line on the status page rather than a server that will not start.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import asdict
from typing import Any, Dict, List, Optional, Tuple

from .camera import Camera
from .config import Config
from .controller import Controller
from .imu import Imu
from .net import interface_addresses

log = logging.getLogger(__name__)

POLL_HZ = 10.0
IMU_RETRY_S = 5.0
IMU_MAX_MISSES = 5  # consecutive failed reads before the chip is closed and reopened
ADDRESS_REFRESH_S = 10.0


class RobotState:
    def __init__(self, config: Config, controller: Controller,
                 imu: Optional[Imu] = None, camera: Optional[Camera] = None):
        self.config = config
        self.controller = controller
        self.imu = imu
        self.camera = camera

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._started_at = time.monotonic()
        self._imu_retry_at = 0.0
        self._imu_misses = 0
        self._imu_failed_with: Optional[str] = None
        self._addresses: List[Tuple[str, str]] = []
        self._attitude: Dict[str, Any] = {
            "ok": False, "calibrating": False, "pitch": None, "roll": None,
            "gyro": None, "temp_c": None, "updated_at": 0.0, "error": None,
        }

    # --- lifecycle ----------------------------------------------------------------

    def start(self) -> None:
        if self.camera is not None:
            try:
                self.camera.open()
            except Exception as exc:
                self.camera.error = f"{type(exc).__name__}: {exc}"
                log.error("camera unavailable: %s", exc)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="state-poll", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self.imu is not None:
            self.imu.close()
        if self.camera is not None:
            self.camera.close()

    # --- poll thread --------------------------------------------------------------

    def _run(self) -> None:
        period = 1.0 / POLL_HZ
        next_addresses = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_addresses:
                found = interface_addresses()
                with self._lock:
                    self._addresses = found
                next_addresses = now + ADDRESS_REFRESH_S
            if self.imu is not None:
                self._poll_imu()
            self._stop.wait(period)

    def _poll_imu(self) -> None:
        imu = self.imu
        try:
            if not imu.is_open:
                if time.monotonic() < self._imu_retry_at:
                    return
                with self._lock:
                    self._attitude.update(calibrating=True)
                imu.open()
                imu.calibrate()
                with self._lock:
                    self._attitude.update(calibrating=False)
                self._imu_failed_with = None
            reading = imu.read()
        except Exception as exc:
            # A loose lead gives the odd NACK. Ride those out on the last good reading;
            # only a run of them means the chip has gone and needs reopening.
            if imu.is_open and self._imu_misses < IMU_MAX_MISSES:
                self._imu_misses += 1
                log.debug("imu read failed (%d in a row): %s", self._imu_misses, exc)
                return
            self._imu_misses = 0
            imu.close()
            self._imu_retry_at = time.monotonic() + IMU_RETRY_S
            message = f"{type(exc).__name__}: {exc}"
            if message != self._imu_failed_with:
                log.warning("imu: %s; retrying every %.0f s", exc, IMU_RETRY_S)
                self._imu_failed_with = message
            with self._lock:
                self._attitude.update(ok=False, calibrating=False, error=message)
            return

        self._imu_misses = 0
        alpha = self.config.imu.smoothing
        with self._lock:
            a = self._attitude
            pitch = reading.pitch if a["pitch"] is None else a["pitch"] + alpha * (reading.pitch - a["pitch"])
            roll = reading.roll if a["roll"] is None else a["roll"] + alpha * (reading.roll - a["roll"])
            a.update(ok=True, pitch=pitch, roll=roll, gyro=reading.gyro, temp_c=reading.temp_c,
                     updated_at=reading.at, error=None)

    # --- snapshot -----------------------------------------------------------------

    def attitude(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._attitude)

    def snapshot(self) -> Dict[str, Any]:
        now = time.monotonic()
        base: Dict[str, Any] = asdict(self.controller.snapshot())
        with self._lock:
            att = dict(self._attitude)
            addresses = list(self._addresses)

        base["imu"] = {
            "ok": att["ok"],
            "calibrating": att["calibrating"],
            "pitch": None if att["pitch"] is None else round(att["pitch"], 1),
            "roll": None if att["roll"] is None else round(att["roll"], 1),
            "gyro": None if att["gyro"] is None else [round(v, 1) for v in att["gyro"]],
            "temp_c": None if att["temp_c"] is None else round(att["temp_c"], 1),
            "age_s": round(now - att["updated_at"], 2) if att["updated_at"] else -1.0,
            "read_errors": getattr(self.imu, "read_errors", 0),
            "error": att["error"] if self.imu is not None else "no imu configured",
        }
        camera = self.camera
        base["camera"] = {
            "ok": bool(camera and camera.ok),
            "streaming": bool(camera and camera.streaming),
            "captures": camera.captures if camera else 0,
            "last_capture": camera.last_capture if camera else None,
            "error": (camera.error if camera else "no camera configured"),
        }
        base["ip"] = addresses[0][1] if addresses else None
        base["addresses"] = [list(entry) for entry in addresses]
        base["uptime_s"] = round(now - self._started_at, 1)
        # The board error matters most, then the sensors. One line for the panel.
        base["last_error"] = base["error"] or base["imu"]["error"] or base["camera"]["error"]
        return base
