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
    return CameraConfig(
        enabled=bool(raw.get("enabled", defaults.enabled)),
        lores=lores,  # type: ignore[arg-type]
        still=still,  # type: ignore[arg-type]
        stream_fps=float(raw.get("stream_fps", defaults.stream_fps)),
        jpeg_quality=int(raw.get("jpeg_quality", defaults.jpeg_quality)),
        buffers=max(1, int(raw.get("buffers", defaults.buffers))),
        survey_dir=str(raw.get("survey_dir", defaults.survey_dir)),
    )


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
    )
