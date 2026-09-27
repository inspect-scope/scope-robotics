"""Serial transport to the Servo2040.

One thread owns the port. Everything else hands it a target servo frame and
reads a telemetry snapshot; nobody else touches `serial`. That thread is also
the watchdog: if the frame it is holding goes stale it drops torque, so a hung
control loop or a dead websocket parks the robot instead of leaving it powered
with whatever it was last told. It also runs the current and voltage trips, so
every user of the board gets them: `hexapod serve`, `jog` and `neutral` alike.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Sequence, Tuple

from . import protocol
from .config import Config, TouchCal

log = logging.getLogger(__name__)

NEUTRAL_PULSE = 1500
STALE_TELEMETRY_S = 2.0  # torque on and no fresh current/voltage reading for this long: estop, we are blind
_TELEMETRY_START = protocol.TOUCH_BASE          # 18
_TELEMETRY_COUNT = protocol.CH_VOLTAGE - protocol.TOUCH_BASE + 1  # touch x6 + current + voltage
CONTACT_EPS_MM = 1.0  # commanded tip this close to the ground plane counts as switch closed
_TOUCH_DOWN_V = 3.3
_TOUCH_UP_V = 0.2


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


def _held_mean(seen: Deque[Tuple[float, float]], now: float, span: float) -> Optional[float]:
    """Mean of the readings in the last `span` seconds, once they cover most of it.
    A mean rather than every-sample-over, so one low reading cannot reset a trip."""
    while seen and seen[0][0] < now - span:
        seen.popleft()
    if not seen or now - seen[0][0] < 0.8 * span:
        return None
    return sum(v for _, v in seen) / len(seen)


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
        self._retry: Optional[threading.Timer] = None
        # trips, touched only by the IO thread except _safety_trip (read under _lock)
        self._amps_seen: Deque[Tuple[float, float]] = deque()
        self._volts_seen: Deque[Tuple[float, float]] = deque()
        self._last_seen_at = 0.0
        self._torque_on_since: Optional[float] = None
        self._volt_warned_at = -1e9
        self._safety_trip: Optional[str] = None

    # --- lifecycle ----------------------------------------------------------------

    def open(self) -> None:
        import serial  # imported here so dry-run works without the port present

        log.info("opening %s", self.port_name)
        # exclusive: a second opener (poke.py, preflight) fails at once instead of
        # silently sharing the port and fighting this loop for the servos.
        self._serial = serial.Serial(
            self.port_name, baudrate=self.config.baudrate, timeout=self.config.timeout_s, write_timeout=1.0,
            exclusive=True,
        )
        # A torque request made while the port was missing or held elsewhere must
        # not be replayed the moment it opens: someone may have hands on the legs.
        with self._lock:
            self._torque_wanted = False
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="servo2040-io", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._retry is not None:
            self._retry.cancel()
            self._retry = None
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

    def open_tolerant(self, retry_s: float = 5.0) -> None:
        """For the server. A missing port (USB out) or a locked one (poke.py or
        preflight has it) becomes a line on the status page and a retry every
        `retry_s`, not an exit and a systemd restart storm. SerialException is an
        OSError, so one clause covers both."""
        try:
            self.open()
        except OSError as exc:
            self._error = f"{type(exc).__name__}: {exc}"
            log.error("board unavailable: %s; retrying every %.0f s", exc, retry_s)
            self._retry = threading.Timer(retry_s, self.open_tolerant, kwargs={"retry_s": retry_s})
            self._retry.daemon = True
            self._retry.start()
        else:
            self._error = None
            self._retry = None

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
        if enabled and not self.connected:
            raise BoardError("board offline; torque not enabled")
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
            self._safety_trip = None

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

    @property
    def safety_trip(self) -> Optional[str]:
        """Why the board latched its own estop, until clear_estop()."""
        with self._lock:
            return self._safety_trip

    # --- trips --------------------------------------------------------------------

    def _guard(self, now: float) -> None:
        """Latch the estop on sustained overcurrent, undervoltage or lost telemetry
        while torque is on. Judges the mean over each window, so a reading that
        swings around the cut still trips. The current is the TOTAL for all 18
        servos: one stalled servo adds about 4 A and does not reach
        safety.current_cut_a on its own."""
        s = self.config.safety
        with self._lock:
            t = self._telemetry
            on = bool(self._torque_actual)
        if not on:
            self._amps_seen.clear()
            self._volts_seen.clear()
            self._torque_on_since = None
            return
        if self._torque_on_since is None:
            self._torque_on_since = now
        # Only readings taken after torque came on: one from before would dilute
        # the first window with idle current.
        if t.updated_at and t.updated_at != self._last_seen_at and t.updated_at >= self._torque_on_since:
            self._last_seen_at = t.updated_at
            if t.amps is not None:
                self._amps_seen.append((t.updated_at, t.amps))
            if t.volts is not None:
                self._volts_seen.append((t.updated_at, t.volts))
        # A reading from before torque came on does not use up the grace period.
        if now - max(t.updated_at, self._torque_on_since) > STALE_TELEMETRY_S:
            since = f"{now - t.updated_at:.1f} s" if t.updated_at else "ever"
            self._safety_estop(f"no telemetry for {since} with torque on; current and voltage unwatched")
            return
        amps = _held_mean(self._amps_seen, now, s.current_cut_s)
        if amps is not None and amps > s.current_cut_a:
            self._safety_estop(f"overcurrent: {amps:.1f} A mean over {s.current_cut_s:.1f} s, "
                               f"cut is {s.current_cut_a:.0f} A")
            return
        volts = _held_mean(self._volts_seen, now, s.volts_cut_s)
        if volts is not None and volts < s.volts_cut:
            self._safety_estop(f"battery {volts:.2f} V mean over {s.volts_cut_s:.0f} s, "
                               f"below the {s.volts_cut:.1f} V cut")
            return
        if t.volts is not None and t.volts < s.volts_warn and now - self._volt_warned_at > 30.0:
            log.warning("battery %.2f V, below the %.1f V warning", t.volts, s.volts_warn)
            self._volt_warned_at = now

    def _safety_estop(self, reason: str) -> None:
        log.error("SAFETY TRIP: %s. Torque off, estop latched.", reason)
        with self._lock:
            self._safety_trip = reason
            self._estopped = True
            self._torque_wanted = False
        self._amps_seen.clear()
        self._volts_seen.clear()

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

        self._guard(time.monotonic())

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

    fake_volts = 7.4      # tests set these to drive the safety trips
    fake_amps_on = 1.2    # a float, or a callable returning one per reading
    fake_telemetry_frozen = False  # stop producing readings, as a dead sensor would

    def __init__(self, config: Config, port: Optional[str] = None):
        super().__init__(config, port)
        self._sim_contacts: Dict[str, bool] = {name: True for name in config.touch}

    def simulate_contacts(self, feet: Dict[str, Sequence[float]], height: float) -> None:
        """Close each foot switch when the commanded tip is on the ground plane.

        Live hardware reads the microswitch. Dry-run has no rod, so the gait's
        foot z is the stand-in.
        """
        ground = -height
        contacts = {
            name: float(point[2]) <= ground + CONTACT_EPS_MM
            for name, point in feet.items()
            if name in self.config.touch
        }
        with self._lock:
            self._sim_contacts = contacts
            prev = self._telemetry
            # Contacts only. A new updated_at here would look like a current
            # reading and dilute the safety trip windows.
            self._telemetry = Telemetry(
                volts=prev.volts,
                amps=prev.amps,
                contacts=contacts,
                touch_volts={
                    name: _TOUCH_DOWN_V if contacts.get(name) else _TOUCH_UP_V
                    for name in self.config.touch
                },
                updated_at=prev.updated_at,
            )

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
            if now - self._last_telemetry_req >= telemetry_period and not self.fake_telemetry_frozen:
                self._last_telemetry_req = now
                amps_on = self.fake_amps_on() if callable(self.fake_amps_on) else self.fake_amps_on
                contacts = dict(self._sim_contacts)
                self._telemetry = Telemetry(
                    volts=self.fake_volts,
                    amps=amps_on if want else 0.05,
                    contacts=contacts,
                    touch_volts={
                        name: _TOUCH_DOWN_V if contacts.get(name) else _TOUCH_UP_V
                        for name in self.config.touch
                    },
                    updated_at=now,
                )
        self._guard(time.monotonic())
