# scope-hexapod

Pi-side control for a MakeYourPet hexapod. Replaces the Chica Server and Chica Client Android apps.

The Servo2040 keeps its existing Chica firmware. Everything above it moves to the Pi.

```
browser on the LAN  --wifi-->  Pi  --USB serial-->  Servo2040  -->  18 servos
joystick, sliders              IK, gait, web server  firmware unchanged
```

## Install

On the Pi (or a laptop, to try it without hardware):

```sh
git clone <this repo> && cd scope-robotics
python3 -m venv .venv && .venv/bin/pip install -e '.[dev]'
```

Try the whole stack with no board attached:

```sh
.venv/bin/hexapod --dry-run serve
```

Open the URL it prints.

## Bring-up

Do these in order. Each step must work before you start the next. Keep the robot on a stand until step 6.

**1. Find the board.**

```sh
hexapod ports
```

Put the device path in `serial.port` in [config/hexapod.yaml](config/hexapod.yaml).

**2. Check the link.** No servo movement. Reads voltage, current and the touch pins.

```sh
python3 tools/poke.py /dev/ttyACM0
```

This script imports nothing from this repo. If it works, the wiring, firmware and port are fine.

Check the voltage it prints against a multimeter. The firmware scales that reading by 1024/3.3 before sending it, and the constant has not been verified on hardware here. If it is off, fix `_B1024_3V3_RATIO` in [hexapod/protocol.py](hexapod/protocol.py).

**3. Check the maths.** Offline. No board needed.

```sh
hexapod check
```

It prints joint travel over the full command envelope and flags any limit the gait would hit.

**4. Centre the servos.** Legs off the ground.

```sh
python3 tools/poke.py /dev/ttyACM0 --centre
```

Every servo goes to 1500 us. This is your mechanical zero.

**5. Fix the joint directions.** One leg at a time, still elevated.

```sh
hexapod jog
```

Two checks per leg:

- `L1 -8 35 68` (the three attach angles from the config) must put all three servos back at 1500 us.
- `L1 0 0 0` must point the leg straight out sideways, horizontal, knee straight.

If a joint moves the wrong way, flip `direction` from `1` to `-1` for that servo in the config. If a joint is centred but at the wrong angle, correct its attach angle.

**6. Neutral stance.** Still elevated. Then lower it onto the ground.

```sh
hexapod neutral --height 80
```

**7. Walk.**

```sh
hexapod serve
```

Open the URL it prints from any device on the same network.

## Web interface

- Left pad moves, right pad turns. `W A S D` and `Q E` on a keyboard.
- Space is e-stop. So is the red button, and `POST /api/estop` if the page is wedged.
- Stand, Sit and Torque are separate. Sit before you cut torque.
- The top-down view shows live foot positions. Green is on the ground, hollow blue is mid swing, yellow means that leg hit a joint limit.
- Posture and gait sliders tune ride height, body tilt, cycle time, step lift and top speed while it walks.

## Configuration

Everything lives in [config/hexapod.yaml](config/hexapod.yaml).

Servo channels and calibration are ported from MakeYourPet's `chica-config-2040.txt`. The `P` numbers in that file are the same integers the firmware uses as channel indices, so `P15` is `channel: 15`. If you already calibrated your servos with Chica, copy the two microsecond values per servo straight across.

The stance block is ours, not a copy of Chica's `MODE_STANDARD`. It puts every foot about 138.5 mm out from its own coxa, which is what Chica's radius, corner angle and elongation constants work out to.

## Safety

- `limits.pulse_us` is the hard clamp. Nothing reaches a servo outside it.
- The board's IO thread cuts torque if it goes `control.watchdog_ms` without a fresh servo frame. A hung control loop, a crashed server or a dropped websocket all park the robot.
- E-stop latches. Clear it before the robot will stand again.
- Body shift plus tilt at low ride height can push a leg past its joint limits. The UI shows that leg in yellow and `hexapod check` prints how often it happens.
- If your servo supply is above 5 V, cut the *Separate USB and Ext. Power* trace on the back of the Servo2040 first.

## Adding sensors

`Controller.command(source, velocity, priority)` takes velocity from any named source. Highest live priority wins, and a command expires after 0.5 s. The web UI is one source at priority 10. A vision behaviour is another call to the same method, no changes to the control loop.

## Layout

| Path | What it does |
|---|---|
| [hexapod/protocol.py](hexapod/protocol.py) | Chica wire format. Pure functions, no IO. |
| [hexapod/board.py](hexapod/board.py) | Serial thread, telemetry, watchdog. `FakeBoard` for dry runs. |
| [hexapod/kinematics.py](hexapod/kinematics.py) | Body pose and 3-DOF leg IK. |
| [hexapod/gait.py](hexapod/gait.py) | Tripod gait. |
| [hexapod/controller.py](hexapod/controller.py) | 50 Hz control loop and state. |
| [hexapod/server.py](hexapod/server.py) | HTTP and WebSocket. |
| [hexapod/static/index.html](hexapod/static/index.html) | The client. One file, no build step, no CDN. |
| [tools/poke.py](tools/poke.py) | Standalone link test. Only needs pyserial. |

## Protocol

Read off the firmware source in [EddieCarrera/chica-servo2040-simpleDriver](https://github.com/EddieCarrera/chica-servo2040-simpleDriver).

```
host -> board  SET   0xD3  start_idx  count  (lo hi) * count
host -> board  GET   0xC7  start_idx  count
board -> host  reply 0xC7  start_idx  count  (lo hi) * count
```

Values are 14 bit, split into two 7-bit bytes, low first. The command byte is the only byte with bit 7 set, which is how the parser resynchronises.

Channels: `0..17` servos, `18..23` touch sensors, `24` current, `25` voltage, `26` relay and torque enable, `27..28` spare GPIO.

Setting channel 26 to zero closes the relay and drops PWM on every servo. Servo positions are remembered while torque is off, so enabling torque snaps to the last commanded pose rather than to 1500 us.
