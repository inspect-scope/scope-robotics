"""The control loop: velocity command in, servo frames out.

Runs on its own thread, not on the web server's event loop. A stalled or
disconnected websocket therefore cannot stall the robot, and a hung control loop
is caught one layer down by the board's watchdog.

Velocity commands arrive from named sources with a priority and an expiry. The
web UI is one source; a vision behaviour is another. Highest live priority wins,
which is how autonomy gets added later without touching the loop.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional

from .board import Servo2040
from .config import JOINTS, Config
from .gait import Gait, TrickKind, Velocity
from .kinematics import BodyPose, HexapodKinematics, JointAngles
from .mode import StanceMode, lookup_mode, modes_for

log = logging.getLogger(__name__)

# What the operator is allowed to ask for. Past these the legs start hitting
# their joint limits at low ride heights; see `hexapod check`.
POSE_LIMITS = {"shift": 15.0, "roll": 8.0, "pitch": 8.0, "yaw": 8.0}
HEIGHT_RATE = 60.0  # mm/s of ride-height change
COMMAND_TTL = 0.5  # a source's command is ignored once it is this old


@dataclass
class Source:
    velocity: Velocity = field(default_factory=Velocity)
    priority: int = 0
    at: float = 0.0


@dataclass
class Snapshot:
    state: str
    torque: bool
    estopped: bool
    connected: bool
    walking: bool
    gait: str
    mode: str
    trick: Optional[str]
    height: float
    cycle_s: float
    step_lift: float
    max_speed: float
    pose: Dict[str, float]
    velocity: Dict[str, float]
    active_source: Optional[str]
    limited_legs: List[str]
    feet: Dict[str, List[float]]
    coxae: Dict[str, List[float]]
    angles: Dict[str, Dict[str, float]]  # joint degrees per leg, what the solver asked for
    joints: Dict[str, Dict[str, float]]
    actuators: Dict[str, Dict[str, Dict[str, float]]]
    pulses: List[int]
    chains: Dict[str, List[List[float]]]
    loop_hz: float
    volts: Optional[float]
    amps: Optional[float]
    contacts: Dict[str, bool]
    touch_volts: Dict[str, float]
    telemetry_age_s: float
    error: Optional[str]
    safety_trip: Optional[str]  # why the board latched its estop, until clear_estop


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, float(value)))


class Controller:
    def __init__(self, config: Config, board: Servo2040):
        self.config = config
        self.board = board
        self.kinematics = HexapodKinematics(config)
        self.gait = Gait(config)
        self.modes = modes_for(config.stance)
        self.mode = StanceMode.Normal

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._sources: Dict[str, Source] = {}
        self._pose = BodyPose()
        self._height = config.stance.sit_height
        self._height_target = config.stance.sit_height
        self._standing = False
        self._limited: List[str] = []
        self._feet: Dict[str, tuple] = self.kinematics.neutral_feet(config.stance.sit_height)
        self._angles: Dict[str, JointAngles] = self.kinematics.solve(self._feet)
        self._pulses: List[int] = self.kinematics.pulse_frame(self._angles)
        self._chains: Dict[str, List[tuple]] = self.kinematics.chains(self._angles, self._pose)
        self._active_source: Optional[str] = None
        self._loop_hz = 0.0
        self._trip_seen: Optional[str] = None  # the board trip this loop has already reacted to

    # --- lifecycle ----------------------------------------------------------------

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="control", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    # --- operator commands --------------------------------------------------------

    def command(self, source: str, velocity: Velocity, priority: int = 0) -> None:
        """Post a normalised (-1..1) velocity from a named source."""
        with self._lock:
            self._sources[source] = Source(velocity=velocity, priority=priority, at=time.monotonic())

    def drop_source(self, source: str) -> None:
        with self._lock:
            self._sources.pop(source, None)

    def set_pose(self, **fields: float) -> None:
        with self._lock:
            current = asdict(self._pose)
            for key, value in fields.items():
                if key not in current:
                    raise ValueError(f"unknown pose field {key!r}")
                if key in ("x", "y"):
                    value = _clamp(value, POSE_LIMITS["shift"])
                elif key in POSE_LIMITS:
                    value = _clamp(value, POSE_LIMITS[key])
                current[key] = float(value)
            current["z"] = 0.0  # ride height is the height slider's job, not the pose's
            self._pose = BodyPose(**current)

    def set_height(self, height: float) -> None:
        s = self.config.stance
        with self._lock:
            self._height_target = max(s.sit_height, min(105.0, float(height)))
            self._standing = self._height_target > s.sit_height + 1.0

    def stand(self) -> None:
        self.board.set_torque(True)
        self.set_height(self.config.stance.ride_height)

    def sit(self) -> None:
        self.gait.reset()
        self.set_height(self.config.stance.sit_height)

    def set_pattern(self, pattern: str) -> None:
        self.gait.set_pattern(pattern)

    def set_mode(self, name: str) -> None:
        spec = lookup_mode(self.modes, name)
        if spec is None:
            raise ValueError(f"unknown stance mode {name!r}")
        spec.apply(self.config.stance)
        self.mode = spec.kind
        if self._standing and self.gait.trick != TrickKind.Jump:
            self.set_height(spec.ride_height)

    def jump(self) -> None:
        if not self._standing or self.board.estopped:
            return
        self.gait.start_jump()

    def set_bounce(self, enabled: bool) -> None:
        self.gait.set_bounce(enabled)

    def torque_off(self) -> None:
        self.board.set_torque(False)

    def estop(self) -> None:
        self.board.estop()
        with self._lock:
            self._sources.clear()
            self._height_target = self.config.stance.sit_height
            self._standing = False
        self.gait.reset()

    def clear_estop(self) -> None:
        self.board.clear_estop()
        with self._lock:
            self._trip_seen = None

    def _react_to_trip(self) -> None:
        """The board latches its own estop on a current or voltage trip. Reset the
        gait state the same way a manual estop does, so clearing it does not snap
        straight back into the pose that tripped."""
        trip = self.board.safety_trip
        if trip and trip != self._trip_seen:
            self._trip_seen = trip
            self.estop()

    # --- loop ---------------------------------------------------------------------

    def _pick_command(self, now: float) -> tuple:
        with self._lock:
            live = [(name, src) for name, src in self._sources.items() if now - src.at <= COMMAND_TTL]
            if not live:
                self._active_source = None
                return Velocity(), None
            name, src = max(live, key=lambda item: (item[1].priority, item[1].at))
            self._active_source = name
            return src.velocity, name

    def _run(self) -> None:
        period = 1.0 / self.config.rate_hz
        next_tick = time.monotonic()
        last = next_tick
        while not self._stop.is_set():
            now = time.monotonic()
            dt = min(max(now - last, 1e-4), 0.2)  # a long stall must not produce a huge step
            last = now
            try:
                self._tick(dt, now)
            except Exception:
                log.exception("control tick failed; dropping torque")
                self.board.estop()
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    def _tick(self, dt: float, now: float) -> None:
        self._react_to_trip()
        raw, _ = self._pick_command(now)

        with self._lock:
            target_height = self._height_target
            standing = self._standing
            pose = self._pose
            delta = target_height - self._height
            step = HEIGHT_RATE * dt
            self._height += max(-step, min(step, delta))
            height = self._height

        settling = abs(height - target_height) > 1.0
        # Walking while the body is still rising, or while sat down, is how you
        # snap a leg. Hold the stance until the height has arrived.
        command = Velocity() if (settling or not standing) else raw.scaled(self.config)

        feet = self.gait.step(dt, command, height)
        pose = self.gait.pose_overlay(pose)
        angles, limited = self.kinematics.solve_reporting(feet, pose)
        pulses = self.kinematics.pulse_frame(angles)
        self.board.set_frame(pulses)
        simulate = getattr(self.board, "simulate_contacts", None)
        if simulate is not None:
            simulate(feet, height)

        with self._lock:
            self._limited = limited
            self._feet = feet
            self._angles = angles
            self._pulses = pulses
            self._chains = self.kinematics.chains(angles, pose)
            self._loop_hz = 1.0 / dt if dt > 0 else 0.0

    # --- state --------------------------------------------------------------------

    def snapshot(self) -> Snapshot:
        telemetry = self.board.telemetry
        with self._lock:
            height, pose, limited = self._height, self._pose, list(self._limited)
            feet = {name: [round(v, 1) for v in point] for name, point in self._feet.items()}
            solved = dict(self._angles)
            pulses = list(self._pulses)
            chains = {
                name: [[round(v, 1) for v in point] for point in points]
                for name, points in self._chains.items()
            }
            source, loop_hz = self._active_source, self._loop_hz
            standing = self._standing
        estopped = self.board.estopped
        torque = self.board.torque_on
        if estopped:
            state = "estop"
        elif not torque:
            state = "off"
        elif self.gait.trick == TrickKind.Jump:
            state = "jumping"
        elif self.gait.walking:
            state = "walking"
        elif self.gait.trick == TrickKind.Bounce:
            state = "bouncing"
        elif standing:
            state = "standing"
        else:
            state = "sitting"
        return Snapshot(
            state=state,
            torque=torque,
            estopped=estopped,
            connected=self.board.connected,
            walking=self.gait.walking,
            gait=self.gait.pattern,
            mode=self.mode,
            trick=self.gait.trick,
            height=round(height, 1),
            cycle_s=round(self.config.stance.cycle_s, 3),
            step_lift=round(self.config.stance.step_lift, 1),
            max_speed=round(self.config.stance.max_speed, 1),
            pose={k: round(v, 2) for k, v in asdict(pose).items()},
            velocity={
                "vx": round(self.gait.velocity.vx, 1),
                "vy": round(self.gait.velocity.vy, 1),
                "yaw_rate": round(self.gait.velocity.yaw_rate, 1),
            },
            active_source=source,
            limited_legs=limited,
            feet=feet,
            coxae={name: list(leg.coxa_xy) for name, leg in self.config.legs.items()},
            angles={
                name: {joint: round(value, 1) for joint, value in zip(JOINTS, solved[name].as_tuple())}
                for name in solved
            },
            joints={
                name: {joint: round(value, 2) for joint, value in zip(JOINTS, solved[name].as_tuple())}
                for name in solved
            },
            actuators=self._actuators(solved, pulses),
            pulses=pulses,
            chains=chains,
            loop_hz=round(loop_hz, 1),
            volts=round(telemetry.volts, 2) if telemetry.volts is not None else None,
            amps=round(telemetry.amps, 2) if telemetry.amps is not None else None,
            contacts=telemetry.contacts,
            touch_volts={name: round(volt, 2) for name, volt in telemetry.touch_volts.items()},
            telemetry_age_s=round(telemetry.age_s, 2) if telemetry.updated_at else -1.0,
            error=self.board.error,
            safety_trip=self.board.safety_trip,
        )

    def _actuators(self, angles: Dict[str, JointAngles], pulses: List[int]) -> Dict[str, Dict[str, Dict[str, float]]]:
        """Per-joint commanded pulse and both angle spaces, labelled for the client.

        The firmware has no GET for servo position. `us` is the last SET. `joint`
        is the IK target. `servo` is that angle after attach and direction.
        """
        out: Dict[str, Dict[str, Dict[str, float]]] = {}
        for name, value in angles.items():
            leg = self.kinematics.legs[name]
            row: Dict[str, Dict[str, float]] = {}
            for joint, joint_deg in zip(JOINTS, value.as_tuple()):
                cal = leg.leg.servos[joint]
                servo_deg = leg.servo_angle(joint, joint_deg)
                row[joint] = {
                    "ch": cal.channel,
                    "us": pulses[cal.channel],
                    "joint": round(joint_deg, 2),
                    "servo": round(servo_deg, 2),
                }
            out[name] = row
        return out
