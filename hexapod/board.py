"""Serial transport to the Servo2040.

One thread owns the port. Everything else hands it a target servo frame and
reads a telemetry snapshot; nobody else touches `serial`. That thread is also
the watchdog: if the frame it is holding goes stale it drops torque, so a hung
control loop or a dead websocket parks the robot instead of leaving it powered
with whatever it was last told.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from . import protocol
from .config import Config, TouchCal

log = logging.getLogger(__name__)

NEUTRAL_PULSE = 1500
_TELEMETRY_START = protocol.TOUCH_BASE          # 18
_TELEMETRY_COUNT = protocol.CH_VOLTAGE - protocol.TOUCH_BASE + 1  # touch x6 + current + voltage


@dataclass
class Telemetry:
    volts: Optional[float] = None
    amps: Optional[float] = None
    contacts: Dict[str, bool] = field(default_factory=dict)
    touch_volts: Dict[str, float] = field(default_factory=dict)
    updated_at: float = 0.0

    @property
    def age_s(self) -> float:
        return float("inf") if self.updated_at == 0.0 else time.monotonic() - self.updated_at


class BoardError(RuntimeError):
    pass


class Servo2040:
    """Threaded driver for the Chica firmware running on a Servo2040."""

    def __init__(self, config: Config, port: Optional[str] = None):
        self.config = config
        self.port_name = port or config.port
        self._serial = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self._frame: List[int] = [NEUTRAL_PULSE] * protocol.NUM_SERVOS
        self._frame_at: float = 0.0
        self._torque_wanted = False
        self._torque_actual: Optional[bool] = None
        self._estopped = False
        self._telemetry = Telemetry()
        self._rx = b""
        self._last_telemetry_req = 0.0
        self._tx_count = 0
        self._error: Optional[str] = None

    # --- lifecycle ----------------------------------------------------------------

    def open(self) -> None:
        import serial  # imported here so dry-run works without the port present

        log.info("opening %s", self.port_name)
        self._serial = serial.Serial(
            self.port_name, baudrate=self.config.baudrate, timeout=self.config.timeout_s, write_timeout=1.0
        )
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="servo2040-io", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._serial is not None:
            try:
                self._serial.write(protocol.encode_torque(False))
                self._serial.flush()
            except Exception:  # closing down; a failed final write must not mask the exit
                log.warning("could not send torque-off while closing", exc_info=True)
            self._serial.close()
            self._serial = None

    def __enter__(self) -> "Servo2040":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # --- commands -----------------------------------------------------------------

    def set_frame(self, pulses: Sequence[int]) -> None:
        """Hand over the next 18 pulse widths. Also feeds the watchdog."""
        if len(pulses) != protocol.NUM_SERVOS:
            raise ValueError(f"expected {protocol.NUM_SERVOS} pulses, got {len(pulses)}")
        with self._lock:
            self._frame = [int(p) for p in pulses]
            self._frame_at = time.monotonic()

    def set_torque(self, enabled: bool) -> None:
        with self._lock:
            if enabled and self._estopped:
                raise BoardError("estop is latched; call clear_estop() first")
            self._torque_wanted = enabled

    def estop(self) -> None:
        """Latch torque off. Takes effect on the IO thread's next tick."""
        with self._lock:
            self._estopped = True
            self._torque_wanted = False

    def clear_estop(self) -> None:
        with self._lock:
            self._estopped = False

    # --- state --------------------------------------------------------------------

    @property
    def telemetry(self) -> Telemetry:
        with self._lock:
            return self._telemetry

    @property
    def torque_on(self) -> bool:
        with self._lock:
            return bool(self._torque_actual)

    @property
    def estopped(self) -> bool:
        with self._lock:
            return self._estopped

    @property
    def connected(self) -> bool:
        return self._thread is not None and self._thread.is_alive() and self._error is None

    @property
    def error(self) -> Optional[str]:
        return self._error

    # --- IO thread ----------------------------------------------------------------

    def _run(self) -> None:
        period = 1.0 / self.config.rate_hz
        telemetry_period = 1.0 / self.config.telemetry_hz
        next_tick = time.monotonic()
        try:
            while not self._stop.is_set():
                self._tick(telemetry_period)
                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()  # fell behind; do not spiral
        except Exception as exc:
            self._error = f"{type(exc).__name__}: {exc}"
            log.exception("serial IO thread died")
            try:
                self._serial.write(protocol.encode_torque(False))
            except Exception:
                pass

    def _tick(self, telemetry_period: float) -> None:
        now = time.monotonic()
        out = bytearray()

        with self._lock:
            stale = (now - self._frame_at) * 1000.0 > self.config.watchdog_ms
            want_torque = self._torque_wanted and not self._estopped and not stale
            frame = list(self._frame)
            torque_actual = self._torque_actual

        if want_torque != torque_actual:
            if not want_torque and torque_actual:
                log.warning("torque off (%s)", "watchdog" if stale else "commanded")
            out += protocol.encode_torque(want_torque)
            with self._lock:
                self._torque_actual = want_torque

        # Sending the servo frame while torque is off is deliberate: the firmware
        # remembers the pulse widths, so enabling torque snaps to the pose we
        # already asked for rather than to 1500 on every channel.
        out += protocol.encode_servo_pulses(frame)

        if now - self._last_telemetry_req >= telemetry_period:
            out += protocol.encode_get(_TELEMETRY_START, _TELEMETRY_COUNT)
            self._last_telemetry_req = now

        # One write per tick: the firmware's parser gives up after 100us without a
        # byte, so a frame split across writes can be dropped halfway through.
        self._serial.write(bytes(out))
        self._tx_count += 1

        waiting = self._serial.in_waiting
        if waiting:
            self._rx += self._serial.read(waiting)
            replies, self._rx = protocol.decode_replies(self._rx)
            if len(self._rx) > 256:
                log.warning("dropping %d unparsed rx bytes", len(self._rx))
                self._rx = b""
            for reply in replies:
                self._ingest(reply)

    def _ingest(self, reply: protocol.GetReply) -> None:
        volts = reply.value_for(protocol.CH_VOLTAGE)
        amps = reply.value_for(protocol.CH_CURRENT)
        contacts: Dict[str, bool] = {}
        touch_volts: Dict[str, float] = {}
        for name, cal in self.config.touch.items():
            counts = reply.value_for(cal.channel)
            if counts is None:
                continue
            volt = protocol.counts_to_sensor_volts(counts)
            touch_volts[name] = volt
            asserted = volt >= self.config.touch_threshold_v
            contacts[name] = asserted if cal.active_high else not asserted

        with self._lock:
            self._telemetry = Telemetry(
                volts=protocol.counts_to_volts(volts) if volts is not None else self._telemetry.volts,
                amps=protocol.counts_to_amps(amps) if amps is not None else self._telemetry.amps,
                contacts=contacts or self._telemetry.contacts,
                touch_volts=touch_volts or self._telemetry.touch_volts,
                updated_at=time.monotonic(),
            )


class FakeBoard(Servo2040):
    """Same interface, no hardware. Lets the whole stack run on a laptop."""

    def open(self) -> None:
        log.info("dry run: no serial port opened")
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="servo2040-fake", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _tick(self, telemetry_period: float) -> None:
        now = time.monotonic()
        with self._lock:
            stale = (now - self._frame_at) * 1000.0 > self.config.watchdog_ms
            want = self._torque_wanted and not self._estopped and not stale
            self._torque_actual = want
            self._tx_count += 1
            if now - self._last_telemetry_req >= telemetry_period:
                self._last_telemetry_req = now
                self._telemetry = Telemetry(
                    volts=7.4,
                    amps=1.2 if want else 0.05,
                    contacts={name: True for name in self.config.touch},
                    touch_volts={name: 3.3 for name in self.config.touch},
                    updated_at=now,
                )
