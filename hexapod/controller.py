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
import math
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional

from .board import LoadKind, Servo2040
from .config import JOINTS, Config
from .gait import Gait, TrickKind, Velocity, MENU_TRICKS, POSE_TRICKS
from .kinematics import BodyPose, HexapodKinematics, JointAngles
from .mode import RIDE_MAX_MM, StanceMode, lookup_mode, modes_for

log = logging.getLogger(__name__)

# What the operator is allowed to ask for. Past these the legs start hitting
# their joint limits at low ride heights; see `hexapod check`.
POSE_LIMITS = {"shift": 15.0, "roll": 8.0, "pitch": 8.0, "yaw": 8.0}
HEIGHT_RATE = 60.0  # mm/s of ride-height change
COMMAND_TTL = 0.5  # a source's command is ignored once it is this old
PUPPET_RATE = 40.0  # deg/s. A joint takes a few seconds to cross its range.
PUPPET_TTL = 0.35  # a quiet page freezes the target; it does not finish the move
PUPPET_DONE = 0.5  # degrees. Close enough to hand the legs back to the gait.
GHOST_DEG = 0.8  # below this the ask and the slew are the same pose, so no ghost
PUPPET_COXA_STEP = 45.0  # one IK solve may not queue more coxa swing than this
PLACE_XY = 420.0  # mm. A grabbed point cannot be asked for past this radius.
PLACE_Z = (-170.0, 80.0)


class LimbMode:
    """Who owns the joint angles. `hold` is the operator, `home` is the return slew."""

    Off = "off"
    Hold = "hold"
    Home = "home"


class PlaceAt:
    """Which point an IK grab is solving for."""

    Foot = "foot"
    Knee = "knee"
PINCER_LEGS = ("L1", "R1")
PINCER_SIDE_MM = 64.0
PINCER_FWD_MM = 40.0
PINCER_LIFT_MM = 55.0


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
    ghost: Optional[Dict[str, List[List[float]]]]  # goal pose, only while it leads the slew
    loop_hz: float
    volts: Optional[float]
    amps: Optional[float]
    contacts: Dict[str, bool]
    touch_volts: Dict[str, float]
    telemetry_age_s: float
    error: Optional[str]
    safety_trip: Optional[str]  # why the board latched its estop, until clear_estop
    puppeteer: str


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, float(value)))


def _bound(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


def _copy_angles(angles: Dict[str, JointAngles]) -> Dict[str, JointAngles]:
    return {name: JointAngles(*value.as_tuple()) for name, value in angles.items()}


def _slew_angles(held: Dict[str, JointAngles], goal: Dict[str, JointAngles], step: float) -> Dict[str, JointAngles]:
    out: Dict[str, JointAngles] = {}
    for name, current in held.items():
        target = goal.get(name, current)
        out[name] = JointAngles(*[_clamp_step(a, b, step) for a, b in zip(current.as_tuple(), target.as_tuple())])
    return out


def _clamp_step(current: float, target: float, step: float) -> float:
    delta = target - current
    if delta > step:
        return current + step
    if delta < -step:
        return current - step
    return target


def _angles_close(held: Dict[str, JointAngles], goal: Dict[str, JointAngles], tol: float) -> bool:
    for name, current in held.items():
        target = goal.get(name)
        if target is None:
            return False
        if any(abs(a - b) > tol for a, b in zip(current.as_tuple(), target.as_tuple())):
            return False
    return True


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
        self._pincers: Optional[Dict[str, tuple]] = None
        self._puppet_held: Optional[Dict[str, JointAngles]] = None
        self._puppet_goal: Optional[Dict[str, JointAngles]] = None
        self._puppet_at = 0.0
        self._puppet_home = False

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
            if self._puppet_held is not None:
                return
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

    def set_pincers(self, offsets: Optional[Dict[str, tuple]]) -> None:
        """L1/R1 foot offsets from the current stance, mm. None clears them."""

        if not offsets:
            with self._lock:
                self._pincers = None
            return
        if self._busy():
            return
        clamped: Dict[str, tuple] = {}
        for name, delta in offsets.items():
            if name not in PINCER_LEGS:
                continue
            clamped[name] = (
                _clamp(delta[0], PINCER_SIDE_MM),
                _clamp(delta[1], PINCER_FWD_MM),
                max(0.0, min(PINCER_LIFT_MM, float(delta[2]))),
            )
        with self._lock:
            self._pincers = clamped or None

    def set_height(self, height: float) -> None:
        s = self.config.stance
        with self._lock:
            self._height_target = max(s.sit_height, min(RIDE_MAX_MM, float(height)))
            self._standing = self._height_target > s.sit_height + 1.0
            # A height change under a hand-posed leg jams the joint. Start the return.
            if self._puppet_held is not None:
                self._puppet_home = True

    def _drop_menu_trick(self) -> None:
        if self.gait.trick in MENU_TRICKS:
            self.gait.stop_trick()

    def stand(self) -> None:
        self._drop_menu_trick()
        self.board.set_torque(True)
        self.set_height(self.config.stance.ride_height)

    def sit(self) -> None:
        self.gait.reset()
        self.set_height(self.config.stance.sit_height)

    def set_pattern(self, pattern: str) -> None:
        self._drop_menu_trick()
        self.gait.set_pattern(pattern)

    def set_mode(self, name: str) -> None:
        self._drop_menu_trick()
        spec = lookup_mode(self.modes, name)
        if spec is None:
            raise ValueError(f"unknown stance mode {name!r}")
        spec.apply(self.config.stance)
        self.mode = spec.kind
        if self._standing and self.gait.trick != TrickKind.Jump:
            self.set_height(spec.ride_height)

    def jump(self) -> None:
        if self._busy() or not self._standing or self.board.estopped:
            return
        self.gait.start_jump()

    def set_bounce(self, enabled: bool) -> None:
        if self._busy():
            return
        self.gait.set_bounce(enabled)

    def start_trick(self, kind: str) -> None:
        if self._busy() or not self._standing or self.board.estopped:
            return
        self.gait.start_trick(kind)

    def stop_trick(self) -> None:
        self.gait.stop_trick()

    def torque_off(self) -> None:
        self.board.set_torque(False)
        # Drop the pose rather than slew home with torque off. The next frames are
        # the stance solution, which is what torque will apply if it comes back.
        with self._lock:
            self._clear_puppet()

    def estop(self) -> None:
        self.board.estop()
        with self._lock:
            self._sources.clear()
            self._pincers = None
            self._clear_puppet()
            self._height_target = self.config.stance.sit_height
            self._standing = False
        self.gait.reset()

    def clear_estop(self) -> None:
        self.board.clear_estop()
        with self._lock:
            self._trip_seen = None

    def arm_puppeteer(self) -> bool:
        """Take the joints. Refuses unless torque is already on and estop is clear.

        Does not enable torque. The hold starts at the angles last sent, so arming
        itself does not move a servo.
        """
        if self.board.estopped or not self.board.torque_on:
            return False
        self.gait.reset()
        with self._lock:
            self._pincers = None
            held = _copy_angles(self._angles)
            self._puppet_held = held
            self._puppet_goal = _copy_angles(held)
            self._puppet_at = time.monotonic()
            self._puppet_home = False
        return True

    def aim_joint(self, leg: str, joint: str, deg: float) -> None:
        """Set one joint's target. Clamped to the config range. Ignored unless holding."""
        if joint not in JOINTS or leg not in self.kinematics.legs:
            return
        try:
            deg = float(deg)
        except (TypeError, ValueError):
            return
        if deg != deg or abs(deg) == float("inf"):
            return
        low, high = self.config.limits.joint_range(joint)
        deg = _bound(deg, low, high)
        with self._lock:
            goal = self._puppet_goal
            if self._puppet_held is None or self._puppet_home or goal is None or leg not in goal:
                return
            parts = dict(zip(JOINTS, goal[leg].as_tuple()))
            parts[joint] = deg
            goal[leg] = JointAngles(**parts)
            self._puppet_at = time.monotonic()

    def aim_point(self, leg: str, at: str, x: float, y: float, z: float) -> None:
        """Solve one leg so its foot or knee moves toward a ground-frame point.

        Joint limits still apply, and the coxa target cannot jump past
        `PUPPET_COXA_STEP` from where the leg is now. A knee grab leaves the tibia
        where it was. Ignored unless holding.
        """
        if leg not in self.kinematics.legs or at not in (PlaceAt.Foot, PlaceAt.Knee):
            return
        try:
            x, y, z = float(x), float(y), float(z)
        except (TypeError, ValueError):
            return
        if not all(math.isfinite(v) for v in (x, y, z)):
            return
        flat = math.hypot(x, y)
        if flat > PLACE_XY:
            scale = PLACE_XY / flat
            x, y = x * scale, y * scale
        z = _bound(z, PLACE_Z[0], PLACE_Z[1])

        with self._lock:
            goal = self._puppet_goal
            held_map = self._puppet_held
            if held_map is None or self._puppet_home or goal is None or leg not in goal:
                return
            pose = self._pose
            held = held_map[leg]
            tibia = goal[leg].tibia

        kin = self.kinematics.legs[leg]
        local = kin.body_to_leg(pose.foot_to_body((x, y, z)))
        if at == PlaceAt.Foot:
            solved = self._solve_foot(kin, local, held.coxa)
            parts = {"coxa": solved.coxa, "femur": solved.femur, "tibia": solved.tibia}
        else:
            coxa, femur = kin.ik_knee(local)
            low, high = self.config.limits.joint_range("femur")
            parts = {
                "coxa": _clamp_step(held.coxa, coxa, PUPPET_COXA_STEP),
                "femur": _bound(femur, low, high),
                "tibia": tibia,
            }

        with self._lock:
            if self._puppet_goal is None or leg not in self._puppet_goal or self._puppet_home:
                return
            self._puppet_goal[leg] = JointAngles(**parts)
            self._puppet_at = time.monotonic()

    def _solve_foot(self, kin, local: tuple, coxa_now: float):
        solved = kin.clamp_angles(kin.ik(local))
        coxa = _clamp_step(coxa_now, solved.coxa, PUPPET_COXA_STEP)
        if abs(coxa - solved.coxa) <= 1e-3:
            return solved
        # The heading was limited, so solve the foot again on that heading.
        # Reach and height stay, which keeps the triangle closed.
        flat = math.hypot(local[0], local[1])
        rad = math.radians(coxa)
        swung = (flat * math.cos(rad), flat * math.sin(rad), local[2])
        solved = kin.clamp_angles(kin.ik(swung))
        return JointAngles(
            coxa=_clamp_step(coxa_now, solved.coxa, PUPPET_COXA_STEP),
            femur=solved.femur,
            tibia=solved.tibia,
        )

    def hold_puppeteer(self) -> None:
        """Stop chasing. The joint stays at the angle it has already reached."""
        with self._lock:
            if self._puppet_held is None or self._puppet_home:
                return
            self._puppet_goal = _copy_angles(self._puppet_held)
            self._puppet_at = time.monotonic()

    def home_puppeteer(self) -> None:
        """Slew back to the stance for this height, then return the legs to the gait."""
        with self._lock:
            if self._puppet_held is None:
                return
            self._puppet_home = True

    def _busy(self) -> bool:
        with self._lock:
            return self._puppet_held is not None

    def _clear_puppet(self) -> None:
        self._puppet_held = None
        self._puppet_goal = None
        self._puppet_home = False

    def _limb_mode(self) -> str:
        if self._puppet_held is None:
            return LimbMode.Off
        if self._puppet_home:
            return LimbMode.Home
        return LimbMode.Hold

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
            pincers = None if self._pincers is None else dict(self._pincers)
            delta = target_height - self._height
            step = HEIGHT_RATE * dt
            self._height += max(-step, min(step, delta))
            height = self._height
            puppet = self._puppet_held is not None

        if puppet and self._apply_puppet(dt, now, height, pose, standing):
            return

        settling = abs(height - target_height) > 1.0
        # Walking while the body is still rising, or while sat down, is how you
        # snap a leg. Hold the stance until the height has arrived.
        command = Velocity() if (settling or not standing) else raw.scaled(self.config)

        feet = self.gait.step(dt, command, height)
        if pincers:
            feet = dict(feet)
            for name, (dx, dy, dz) in pincers.items():
                x, y, z = feet[name]
                feet[name] = (x + dx, y + dy, z + dz)
        pose = self.gait.pose_overlay(pose)
        angles, limited = self.kinematics.solve_reporting(feet, pose)
        pulses = self.kinematics.pulse_frame(angles)
        self.board.set_frame(pulses)
        self.board.set_load(LoadKind.Stand if standing else LoadKind.Sit)
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

    def _apply_puppet(self, dt: float, now: float, height: float, pose: BodyPose, standing: bool) -> bool:
        """Slew toward the operator target, or back to the stance. False if disarmed mid-tick."""
        with self._lock:
            if self._puppet_held is None or self._puppet_goal is None:
                return False
            held = _copy_angles(self._puppet_held)
            goal = _copy_angles(self._puppet_goal)
            homing = self._puppet_home
            stamped = self._puppet_at

        limited: List[str] = []
        if homing:
            goal, limited = self.kinematics.solve_reporting(self.kinematics.neutral_feet(height), pose)
        elif now - stamped > PUPPET_TTL:
            goal = held

        slewed = _slew_angles(held, goal, PUPPET_RATE * dt)
        arrived = homing and _angles_close(slewed, goal, PUPPET_DONE)
        feet = self._feet_of(slewed, pose)
        pulses = self.kinematics.pulse_frame(slewed)
        self.board.set_frame(pulses)
        self.board.set_load(LoadKind.Stand if standing else LoadKind.Sit)
        simulate = getattr(self.board, "simulate_contacts", None)
        if simulate is not None:
            simulate(feet, height)

        with self._lock:
            if arrived or self._puppet_held is None:
                self._clear_puppet()
            else:
                self._puppet_held = slewed
                if homing:
                    self._puppet_goal = goal
            self._limited = limited
            self._feet = feet
            self._angles = slewed
            self._pulses = pulses
            self._chains = self.kinematics.chains(slewed, pose)
            self._loop_hz = 1.0 / dt if dt > 0 else 0.0
        return True

    def _ghost_chains(self, solved: Dict[str, JointAngles], pose: BodyPose):
        """Goal pose for legs the slew has not caught. None when the ask matches."""
        goal = self._puppet_goal
        if goal is None or self._limb_mode() != LimbMode.Hold:
            return None
        ahead = {
            name: angles
            for name, angles in goal.items()
            if name in solved and any(
                abs(a - b) > GHOST_DEG for a, b in zip(angles.as_tuple(), solved[name].as_tuple())
            )
        }
        if not ahead:
            return None
        return {
            name: [[round(v, 1) for v in point] for point in points]
            for name, points in self.kinematics.chains(ahead, pose).items()
        }

    def _feet_of(self, angles: Dict[str, JointAngles], pose: BodyPose) -> Dict[str, tuple]:
        feet: Dict[str, tuple] = {}
        for name, value in angles.items():
            leg = self.kinematics.legs[name]
            feet[name] = pose.body_to_ground(leg.leg_to_body(leg.fk(value)))
        return feet

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
            ghost = self._ghost_chains(solved, pose)
            source, loop_hz = self._active_source, self._loop_hz
            standing = self._standing
            puppeteer = self._limb_mode()
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
        elif self.gait.trick == TrickKind.Flex:
            state = "flexing"
        elif self.gait.trick == TrickKind.Dance:
            state = "dancing"
        elif self.gait.trick in POSE_TRICKS:
            state = "leaning"
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
            ghost=ghost,
            loop_hz=round(loop_hz, 1),
            volts=round(telemetry.volts, 2) if telemetry.volts is not None else None,
            amps=round(telemetry.amps, 2) if telemetry.amps is not None else None,
            contacts=telemetry.contacts,
            touch_volts={name: round(volt, 2) for name, volt in telemetry.touch_volts.items()},
            telemetry_age_s=round(telemetry.age_s, 2) if telemetry.updated_at else -1.0,
            error=self.board.error,
            safety_trip=self.board.safety_trip,
            puppeteer=puppeteer,
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
