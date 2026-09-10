"""Drive the hexapod's body tilt from a laptop's accelerometer.

The robot already accepts posture over the websocket: a `{"type": "pose",
"roll": r, "pitch": p}` message sets body roll and pitch, clamped server-side to
POSE_LIMITS (+/- 8 deg). This bridge is just another websocket client. It reads
laptop tilt, smooths it, scales it into the bot's range, and streams pose.

    laptop tilt  --lowpass-->  scale/clamp  --ws /pose-->  Controller.set_pose

Tilt only. It never stands, walks, or touches torque, so the worst it can do is
lean a standing robot. Stand and pick a safe ride height from the web UI first;
body tilt at low height can push a leg past its joint limit (the UI flags that
leg yellow).

Two properties of the sensor decide the design:

- Static tilt is gravity-referenced, so it is absolute and does not drift. That
  is the signal we use.
- Height is not: double-integrating acceleration runs away in about a second, so
  there is deliberately no up/down-position control here.

Run it against a dry-run server with no hardware to see the pipeline work:

    hexapod --dry-run serve                 # in one shell
    python3 tools/tilt_bridge.py --source demo

Then, on real tilt input:

    sudo python3 tools/tilt_bridge.py --source macos --host ws://<pi>:8000/ws

Only needs `websockets` (pip install websockets).
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import signal
import time

import websockets

# Server clamps to this anyway (controller.POSE_LIMITS); we clamp too so the wire
# carries sane values and the mapping is readable here.
TILT_LIMIT_DEG = 8.0

# Laptop tilt that maps to full bot deflection. Laptop travel is +/- 90 deg; the
# bot only has +/- 8, so a gentle 1:3-ish ratio keeps it from pegging instantly.
DEFAULT_FULL_SCALE_DEG = 25.0

# Low-pass smoothing factor per sample. Lower is smoother and laggier. The pose
# path applies instantly with no server-side slew, and raw accel catches typing
# and desk bumps, so this filter is what stops the body juddering.
DEFAULT_ALPHA = 0.15

DEFAULT_RATE_HZ = 30.0
DEFAULT_HOST = "ws://localhost:8000/ws"

SOURCE_DEMO = "demo"
SOURCE_MACOS = "macos"


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def accel_to_tilt(ax: float, ay: float, az: float) -> tuple[float, float]:
    """Gravity vector (any consistent units) to (roll_deg, pitch_deg).

    roll  is rotation about the fore-aft axis, from ay/az.
    pitch is rotation about the left-right axis, from ax and the ay/az magnitude.
    """
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
    return roll, pitch


# --- tilt sources ---------------------------------------------------------------------


class DemoSource:
    """Synthetic slow sway, so the pipeline can be tested with no sensor."""

    def __init__(self, amplitude_deg: float = 18.0, period_s: float = 6.0):
        self._amp = amplitude_deg
        self._w = 2.0 * math.pi / period_s
        self._t0 = time.monotonic()

    def read(self) -> tuple[float, float]:
        t = time.monotonic() - self._t0
        return self._amp * math.sin(self._w * t), self._amp * math.cos(self._w * t)


class MacOSSource:
    """Real laptop accelerometer.

    TODO(wire from motion_live.py): fill in `_read_raw_accel` with the same read
    you use in ~/apple-silicon-accelerometer/motion_live.py. Return the gravity
    vector as three floats in a consistent unit (g or m/s^2 both work; the
    conversion only uses ratios). Everything downstream is done.
    """

    def _read_raw_accel(self) -> tuple[float, float, float]:
        raise NotImplementedError(
            "wire _read_raw_accel to motion_live.py's accelerometer read; "
            "until then run with --source demo"
        )

    def read(self) -> tuple[float, float]:
        ax, ay, az = self._read_raw_accel()
        return accel_to_tilt(ax, ay, az)


def make_source(name: str):
    if name == SOURCE_DEMO:
        return DemoSource()
    if name == SOURCE_MACOS:
        return MacOSSource()
    raise ValueError(f"unknown source {name!r}")


# --- bridge ---------------------------------------------------------------------------


async def run(args: argparse.Namespace) -> None:
    source = make_source(args.source)
    gain = TILT_LIMIT_DEG / max(args.full_scale, 1e-6)
    period = 1.0 / args.rate
    roll_sign = -1.0 if args.invert_roll else 1.0
    pitch_sign = -1.0 if args.invert_pitch else 1.0

    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    async with websockets.connect(args.host) as ws:
        print(f"connected to {args.host}; streaming pose from {args.source} source")
        fr = fp = 0.0
        try:
            while not stop.is_set():
                raw_roll, raw_pitch = source.read()
                fr += args.alpha * (raw_roll - fr)
                fp += args.alpha * (raw_pitch - fp)
                roll = _clamp(roll_sign * gain * fr, TILT_LIMIT_DEG)
                pitch = _clamp(pitch_sign * gain * fp, TILT_LIMIT_DEG)
                await ws.send(json.dumps({"type": "pose", "roll": roll, "pitch": pitch}))
                await asyncio.sleep(period)
        finally:
            # Level the body on the way out rather than leaving it leaning.
            await ws.send(json.dumps({"type": "pose", "roll": 0.0, "pitch": 0.0}))
            print("\nlevelled and disconnected")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Drive hexapod body tilt from laptop accelerometer.")
    p.add_argument("--host", default=DEFAULT_HOST, help="hexapod websocket URL")
    p.add_argument("--source", choices=(SOURCE_DEMO, SOURCE_MACOS), default=SOURCE_DEMO)
    p.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ, help="send rate (Hz)")
    p.add_argument("--alpha", type=float, default=DEFAULT_ALPHA, help="low-pass factor 0..1")
    p.add_argument("--full-scale", type=float, default=DEFAULT_FULL_SCALE_DEG,
                   help="laptop tilt (deg) mapped to full bot deflection")
    p.add_argument("--invert-roll", action="store_true")
    p.add_argument("--invert-pitch", action="store_true")
    return p.parse_args(argv)


def main() -> None:
    args = parse_args()
    try:
        asyncio.run(run(args))
    except NotImplementedError as exc:
        raise SystemExit(str(exc))


if __name__ == "__main__":
    main()
