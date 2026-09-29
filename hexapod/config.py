"""Load and validate config/hexapod.yaml."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

import yaml

JOINTS = ("coxa", "femur", "tibia")
AXES = ("x", "y", "z")
DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "hexapod.yaml")


@dataclass(frozen=True)
class ServoCal:
    channel: int
    us_neg45: float
    us_pos45: float
    direction: int = 1

    def pulse_for(self, servo_angle_deg: float) -> float:
        """Linear interpolation through the two calibration points."""
        span = self.us_pos45 - self.us_neg45
        return self.us_neg45 + (servo_angle_deg + 45.0) / 90.0 * span


@dataclass(frozen=True)
class TouchCal:
    channel: int
    active_high: bool


@dataclass(frozen=True)
class LegConfig:
    name: str
    coxa_xy: Tuple[float, float]
    yaw_deg: float
    neutral_xy: Tuple[float, float]
    servos: Dict[str, ServoCal]


@dataclass(frozen=True)
class Geometry:
    coxa_len: float
    femur_len: float
    tibia_len: float
    leg_connection_z: float
    coxa_attach_angle: float
    femur_attach_angle: float
    tibia_attach_angle: float

    def attach_angle(self, joint: str) -> float:
        return getattr(self, f"{joint}_attach_angle")


@dataclass  # not frozen: cycle_s / step_lift / max_speed are tunable at runtime
class Stance:
    ride_height: float
    sit_height: float
    step_lift: float
    cycle_s: float
    max_speed: float
    max_yaw_rate: float
    max_stride: float
    min_foot_depth: float


@dataclass(frozen=True)
class Limits:
    pulse_us: Tuple[float, float]
    coxa_deg: Tuple[float, float]
    femur_deg: Tuple[float, float]
    tibia_deg: Tuple[float, float]

    def joint_range(self, joint: str) -> Tuple[float, float]:
        return getattr(self, f"{joint}_deg")


@dataclass(frozen=True)
class ImuConfig:
    """GY-521 / MPU-6050 on the Pi's i2c bus.

    `axis_map` turns the chip's axes into the body frame (+X right, +Y forward,
    +Z up). Each entry names the chip axis that points along that body axis, with
    an optional minus sign: ["y", "-x", "z"] means the chip's +y arrow points to
    the robot's right and its +x arrow points backwards.
    """

    enabled: bool = True
    bus: int = 1
    address: int = 0x68
    bias_seconds: float = 2.0
    axis_map: Tuple[str, str, str] = ("x", "y", "z")
    smoothing: float = 0.3
    bus_retries: int = 2  # extra attempts after a NACK, per transaction


@dataclass(frozen=True)
class CameraConfig:
    """Pi camera through Picamera2. Two streams at once: lores for the live
    view, full resolution for stills, so a capture never interrupts the stream."""

    enabled: bool = True
    lores: Tuple[int, int] = (640, 360)
    still: Tuple[int, int] = (4608, 2592)
    stream_fps: float = 15.0
    jpeg_quality: int = 85
    buffers: int = 2
    survey_dir: str = "~/surveys"
    rotation: int = 0  # 0 or 180. 180 for a camera mounted upside down


@dataclass(frozen=True)
class SafetyConfig:
    """Trips the board's IO thread applies while torque is on, judged on the mean
    over each window. The current is the total for all 18 servos: one stalled
    FT5330M adds about 4 A. current_cut_a is the hard ceiling. sit_cut_a and
    stand_cut_a apply only after pulses have been still for still_s, so a single
    stall at a held pose trips. Walking (pulses moving) only uses current_cut_a.
    sit_cut_a matches poke.py --centre. stand_cut_a sits between estimated
    standing load (about 3 to 4 A) and standing plus one stall (about 7 A)."""

    current_cut_a: float = 10.0
    current_cut_s: float = 1.0
    still_s: float = 1.0
    sit_cut_a: float = 1.5
    stand_cut_a: float = 5.5
    volts_warn: float = 6.4
    volts_cut: float = 6.0
    volts_cut_s: float = 2.0


@dataclass(frozen=True)
class Config:
    port: str
    baudrate: int
    timeout_s: float
    rate_hz: float
    telemetry_hz: float
    watchdog_ms: float
    geometry: Geometry
    stance: Stance
    limits: Limits
    legs: Dict[str, LegConfig]
    tripod_groups: List[List[str]]
    touch: Dict[str, TouchCal]
    touch_threshold_v: float
    leg_order: List[str] = field(default_factory=list)
    imu: ImuConfig = field(default_factory=ImuConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)


def _axis_map(raw: Any) -> Tuple[str, str, str]:
    entries = tuple(str(v).strip().lower() for v in (raw or AXES))
    if len(entries) != 3:
        raise ValueError(f"imu.axis_map needs three entries, got {entries}")
    letters = []
    for entry in entries:
        letter = entry.lstrip("-")
        if letter not in AXES or entry.count("-") > 1:
            raise ValueError(f"imu.axis_map entry {entry!r} is not one of x, y, z, -x, -y, -z")
        letters.append(letter)
    if sorted(letters) != list(AXES):
        raise ValueError(f"imu.axis_map must use each of x, y, z once, got {entries}")
    return entries  # type: ignore[return-value]


def _imu(raw: Dict[str, Any]) -> ImuConfig:
    defaults = ImuConfig()
    return ImuConfig(
        enabled=bool(raw.get("enabled", defaults.enabled)),
        bus=int(raw.get("bus", defaults.bus)),
        address=int(raw.get("address", defaults.address)),
        bias_seconds=float(raw.get("bias_seconds", defaults.bias_seconds)),
        axis_map=_axis_map(raw.get("axis_map")),
        smoothing=max(0.0, min(1.0, float(raw.get("smoothing", defaults.smoothing)))),
        bus_retries=max(0, int(raw.get("bus_retries", defaults.bus_retries))),
    )


def _camera(raw: Dict[str, Any]) -> CameraConfig:
    defaults = CameraConfig()
    lores = tuple(int(v) for v in raw.get("lores", defaults.lores))
    still = tuple(int(v) for v in raw.get("still", defaults.still))
    if len(lores) != 2 or len(still) != 2:
        raise ValueError("camera.lores and camera.still are [width, height]")
    if lores[0] > still[0] or lores[1] > still[1]:
        raise ValueError("camera.lores must not be larger than camera.still")
    rotation = int(raw.get("rotation", defaults.rotation))
    if rotation not in (0, 180):
        # A sensor can only flip, not rotate. 90 would need a transpose we do not do.
        raise ValueError("camera.rotation must be 0 or 180")
    return CameraConfig(
        enabled=bool(raw.get("enabled", defaults.enabled)),
        lores=lores,  # type: ignore[arg-type]
        still=still,  # type: ignore[arg-type]
        stream_fps=float(raw.get("stream_fps", defaults.stream_fps)),
        jpeg_quality=int(raw.get("jpeg_quality", defaults.jpeg_quality)),
        buffers=max(1, int(raw.get("buffers", defaults.buffers))),
        survey_dir=str(raw.get("survey_dir", defaults.survey_dir)),
        rotation=rotation,
    )


def _safety(raw: Dict[str, Any]) -> SafetyConfig:
    d = SafetyConfig()
    cfg = SafetyConfig(
        current_cut_a=float(raw.get("current_cut_a", d.current_cut_a)),
        current_cut_s=float(raw.get("current_cut_s", d.current_cut_s)),
        still_s=float(raw.get("still_s", d.still_s)),
        sit_cut_a=float(raw.get("sit_cut_a", d.sit_cut_a)),
        stand_cut_a=float(raw.get("stand_cut_a", d.stand_cut_a)),
        volts_warn=float(raw.get("volts_warn", d.volts_warn)),
        volts_cut=float(raw.get("volts_cut", d.volts_cut)),
        volts_cut_s=float(raw.get("volts_cut_s", d.volts_cut_s)),
    )
    if min(cfg.current_cut_a, cfg.current_cut_s, cfg.still_s, cfg.sit_cut_a,
           cfg.stand_cut_a, cfg.volts_cut, cfg.volts_cut_s) <= 0:
        raise ValueError("safety cuts and windows must be positive; remove the block to use the defaults")
    if cfg.volts_warn < cfg.volts_cut:
        raise ValueError("safety.volts_warn must not be below safety.volts_cut")
    if not cfg.sit_cut_a <= cfg.stand_cut_a <= cfg.current_cut_a:
        raise ValueError("safety sit/stand/current cuts must be nondecreasing")
    return cfg


def load(path: str = DEFAULT_CONFIG) -> Config:
    with open(path) as handle:
        raw = yaml.safe_load(handle)

    geometry = Geometry(**raw["geometry"])
    stance = Stance(**raw["stance"])
    limits = Limits(
        pulse_us=tuple(raw["limits"]["pulse_us"]),
        coxa_deg=tuple(raw["limits"]["coxa_deg"]),
        femur_deg=tuple(raw["limits"]["femur_deg"]),
        tibia_deg=tuple(raw["limits"]["tibia_deg"]),
    )

    legs: Dict[str, LegConfig] = {}
    for name, leg in raw["legs"].items():
        servos = {joint: ServoCal(**raw["servos"][name][joint]) for joint in JOINTS}
        legs[name] = LegConfig(
            name=name,
            coxa_xy=tuple(leg["coxa"]),
            yaw_deg=float(leg["yaw"]),
            neutral_xy=tuple(leg["neutral"]),
            servos=servos,
        )

    touch_raw = dict(raw.get("touch_sensors") or {})
    threshold = float(touch_raw.pop("threshold_v", 1.6))
    touch = {name: TouchCal(**cal) for name, cal in touch_raw.items()}

    used: Dict[int, str] = {}
    for leg in legs.values():
        for joint, cal in leg.servos.items():
            if cal.channel in used:
                raise ValueError(f"servo channel {cal.channel} claimed by both {used[cal.channel]} and {leg.name}.{joint}")
            used[cal.channel] = f"{leg.name}.{joint}"
    if len(used) != 18:
        raise ValueError(f"expected 18 servo channels, config defines {len(used)}")

    groups = [list(group) for group in raw["tripod_groups"]]
    flat = [name for group in groups for name in group]
    if sorted(flat) != sorted(legs):
        raise ValueError("tripod_groups must name every leg exactly once")

    return Config(
        port=raw["serial"]["port"],
        baudrate=int(raw["serial"]["baudrate"]),
        timeout_s=float(raw["serial"]["timeout_s"]),
        rate_hz=float(raw["control"]["rate_hz"]),
        telemetry_hz=float(raw["control"]["telemetry_hz"]),
        watchdog_ms=float(raw["control"]["watchdog_ms"]),
        geometry=geometry,
        stance=stance,
        limits=limits,
        legs=legs,
        tripod_groups=groups,
        touch=touch,
        touch_threshold_v=threshold,
        leg_order=list(raw["legs"].keys()),
        imu=_imu(dict(raw.get("imu") or {})),
        camera=_camera(dict(raw.get("camera") or {})),
        safety=_safety(raw.get("safety") or {}),
    )
