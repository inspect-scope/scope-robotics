# scope-hexapod

Pi-side control for a MakeYourPet hexapod. Replaces the Chica Server and Chica Client Android apps.

The Servo2040 keeps its existing Chica firmware. Everything above it moves to the Pi.

```
browser on the LAN  --wifi-->  Pi  --USB serial-->  Servo2040  -->  18 servos
joystick, sliders   /          |   --i2c-------->  GY-521 IMU
320x480 panel   ---            |   --CSI--------->  camera
                        one FastAPI process: IK, gait, state, MJPEG
```

One process owns the hardware. The serial port, the i2c bus and the camera
can each be opened by a single process, so `hexapod serve` holds all three and
both pages (`/` for the operator, `/status` for the panel) are plain HTTP
clients of it. Do not run `poke.py`, `preflight.py` or anything else that
touches a device while the server is up; the "device busy" errors look
intermittent and are not.

## Install

On the Pi:

```sh
git clone <this repo> && cd scope-robotics
python3 -m venv --system-site-packages .venv && .venv/bin/pip install -e '.[dev]'
```

`--system-site-packages` matters: `picamera2` is apt-installed
(`sudo apt install python3-picamera2`) and a plain venv cannot import it.

On a laptop, to try it without hardware:

```sh
python3 -m venv .venv && .venv/bin/pip install -e '.[dev]'
.venv/bin/hexapod --dry-run serve
```

Open the URL it prints. `/` is the client, `/status` is the panel page. The
dry run fakes the board, a rocking IMU and a placeholder camera, so every page
and endpoint works with nothing attached.

## Preflight

One command that checks everything the robot needs, on the Pi:

```sh
python3 tools/preflight.py
```

It checks the Python environment, that no file got zero-filled by a bad
shutdown, that the config loads and its two clamps agree, how often the gait
would need to clamp a joint, that the accelerometer answers and reads 1 g,
that `picamera2` imports and sees a camera, and that the Servo2040 replies
over serial. Exit code is 0 if nothing failed. Stop `hexapod.service` first;
the checks open the same devices.

Every check is independent and catches its own errors, so missing hardware
reports as one bad line instead of a traceback. Skip what is not attached:

```sh
python3 tools/preflight.py --skip-board        # no Servo2040 on this Pi
python3 tools/preflight.py --skip-imu          # no IMU wired yet
python3 tools/preflight.py --skip-camera       # no camera fitted
python3 tools/preflight.py --imu-seconds 10    # longer gyro average
```

Run it before every session once the robot is assembled. Run the numbered
bring-up below the first time, and after changing geometry or calibration.

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

What the voltage means depends on how the board is powered:

| State | Expected |
|---|---|
| USB only, *Separate USB and Ext. Power* trace intact | ~5.0 V |
| Trace cut, no battery on the terminals | ~0 V. The rail is genuinely dead, which is the point of cutting it |
| 2S pack on the terminals | 7.4 to 8.4 V |

The firmware scales the reading by 1024/3.3 before sending it. Measured 5.08 V on USB before the trace was cut, which rules out the codec having grabbed the firmware's other constant (`b1024_5V_RATIO` would have shown 7.7 V or 3.35 V). It has still not been checked against a meter on battery. If it is off, fix `_B1024_3V3_RATIO` in [hexapod/protocol.py](hexapod/protocol.py).

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

It lists every address it is reachable at, and the panel page is `/status` on
any of them. Use the one on your wifi or ethernet interface (`en0`, `wlan0`, `eth0`). If you are on a VPN, the tunnel address is listed too and marked. It will not work from other devices.

## IMU (GY-521)

The GY-521 breakout carries an MPU-6050: 3-axis accelerometer plus 3-axis
gyro, over i2c at address `0x68` (`0x69` if AD0 is pulled high).

**Wiring.** Four pins, nothing else needs connecting.

| GY-521 | Pi |
|---|---|
| VCC | 3.3 V (pin 1) |
| GND | GND (pin 6) |
| SDA | GPIO2 (pin 3) |
| SCL | GPIO3 (pin 5) |

**Enable i2c** once, then reboot:

```sh
sudo raspi-config nonint do_i2c 0
sudo reboot
```

**Check it.**

```sh
python3 tools/preflight.py --skip-board
```

Three things have to be true:

- `WHO_AM_I 0x68`. Anything else means the wrong chip or the wrong address.
  Nothing at all usually means SDA and SCL are swapped.
- `|a|` within 0.10 of 1.000 g. Hold the board still for this.
- Gyro bias under 10 deg/s on every axis.

If the check reports `Errno 121`, nothing acknowledged the address. Scan the
bus before touching any code:

```sh
i2cdetect -y 1
```

Expect `68` for the IMU. There is also a device at `5d` that shows as `UU`
once its driver has loaded: that is the screen's Goodix GT9271 touch
controller, on the same two pins. i2c is a shared bus and the two do not
clash. Nothing at `68` or `69` means the GY-521 is not powered or its leads
are not on pins 3 and 5; the touch controller answering proves the pins
themselves work.

If the check reports a percentage of failed reads, the chip answers but keeps
dropping off the bus. That is a physical contact, not a setting: the failure
rate is the same at every read length and is no better with a repeated-START
transaction. Find the lead:

```sh
python3 tools/i2cwatch.py
```

It reads WHO_AM_I at 20 Hz and prints a rolling success rate. Wiggle one lead
at a time, VCC then GND then SDA then SCL. The one that moves the number is
the bad one. A sound connection sits at 100% and does not move when you flex
the loom. On a GY-521 the usual cause is the pin header being pushed through
the board but never soldered.

The check is on the *magnitude* of the acceleration vector, not on z alone.
Gravity is 1 g whichever way the board is facing, and tilt is computed from the
ratio between axes rather than their absolute size, so the magnitude is the one
number that means "this sensor works" at any mounting angle. A board resting on
its edge reads almost all of its gravity on x, and that is fine.

On this build it reads 0.980 g with a bias of -3.15, +1.33, -0.61 deg/s. Both
are normal for a GY-521. The 2% shortfall is scale-factor error and washes out
of any tilt calculation.

Two things about the bias. It moves with temperature, so measure it after the
electronics have been running ten minutes, not from cold. And prefer averaging
it at startup while the robot is known to be still over hardcoding a constant.

**In the server.** [hexapod/imu.py](hexapod/imu.py) reads the chip with the
same ioctl calls as the preflight, averages the gyro bias for `imu.bias_seconds`
when the server starts (keep the robot still), and subtracts it from every
reading. Pitch and roll come from the accelerometer, positive nose up and right
side down, smoothed by `imu.smoothing`. The state poller reads it at 10 Hz.

**Bus errors are retried.** On this loom about one read in twenty comes back
`OSError 121`, a NACK, and more once the legs are moving. Each i2c transaction
gets `imu.bus_retries` extra attempts 2 ms apart, which clears nearly all of
them; the state poller still rides out a few that get through and reopens the
chip after five in a row. `/api/state` reports both counts: `imu.read_errors` is
every bus error seen, `imu.read_failures` only the ones the retries did not
clear. A rising `read_failures` means a lead, not noise, and is what `/status`
complains about.

`imu.axis_map` in [config/hexapod.yaml](config/hexapod.yaml) says how the
chip is mounted. Each entry is the chip axis pointing along the body axis
(+X right, +Y forward, +Z up), with an optional minus sign. Set it once the
board is bolted down: tip the robot nose up and check the horizon on `/status`
moves down; lean it right and check the horizon tilts. If the IMU is missing
the server still starts and the panel says so.

## Web interface

- Live view top left, from the camera's low-resolution stream. **Capture** (or `C`) saves a full-resolution still and shows the file name.
- Left pad moves, right pad turns. `W A S D` and `Q E` on a keyboard.
- Space is e-stop. So is the red button, and `POST /stop` if the page is wedged.
- Stand, Sit and Torque are separate. Sit before you cut torque.
- The top-down view shows live foot positions. Green is on the ground, hollow blue is mid swing, yellow means that leg hit a joint limit.
- Posture and gait sliders tune ride height, body tilt, cycle time, step lift and top speed while it walks.
- The attitude tile shows pitch and roll from the IMU and the turn rates.

## Status panel

`/status` is built for the screen on the roof: battery voltage large enough to
read from across the room, servo current, the address to type into a laptop, an
artificial horizon from the IMU, the robot's state, and a STOP button. No other
controls. STOP posts to `/stop`, which latches e-stop; clear it from the client
page.

The screen is a 320x480 ST7796S over SPI, mounted on its side, so the page runs
at 480x320. [deploy/kiosk.sh](deploy/kiosk.sh) rotates the output with
`wlr-randr --transform 90` before it starts Chromium, because Chromium sizes
itself to the output once and does not follow a later rotation. Set
`HEXAPOD_PANEL_TRANSFORM=270` if the panel is mounted the other way up, or
`normal` for a portrait screen. `HEXAPOD_PANEL_OUTPUT` overrides the connector
name if yours is not `SPI-1`.

The page has both layouts. Landscape puts the readings on the left and the
horizon on the right; portrait stacks them. It switches on the media query, so
the dry run in a browser window works either way.

The panel is Chromium in kiosk mode pointed at `http://localhost:8000/status`.
Same server, same stack, nothing else to maintain; see Deploy.

## Camera

[hexapod/camera.py](hexapod/camera.py) owns `Picamera2` and runs two streams
off the same sensor frames: a 640x360 one that the encoder turns into the
MJPEG live view, and the full 4608x2592 one that stills are saved from. Both
run all the time, so a capture never interrupts the stream. Sizes are in the
`camera` block of the config.

Stills go to `camera.survey_dir/<server start time>/<capture time>.jpg`, one
directory per server run, with a `.json` beside each holding the full state
snapshot (voltage, pitch, roll, position of every foot) at the moment of
capture. `POST /capture` returns the path.

The live view is MJPEG over `multipart/x-mixed-replace`. It is fine on a LAN or
a tether. Over a bad link WebRTC would do better; that is not built.

Pi 4 and earlier encode the stream in hardware. Pi 5 has no hardware JPEG
encoder, so it uses a software one on an RGB low-resolution stream; the module
picks based on the platform. The full-resolution stream costs about 35 MB of
CMA per buffer (`camera.buffers`, default 2). If the camera fails to start with
an allocation error, lower `still` or add `cma=320M` to the kernel command line.

## API

| Method | Path | What |
|---|---|---|
| GET | `/` | Client page |
| GET | `/status` | Panel page |
| GET | `/api/state` | The state snapshot, once |
| WS | `/telemetry` | The state snapshot, pushed at 10 Hz, nothing accepted |
| WS | `/ws` | Commands in, state out. What the client page uses |
| GET | `/stream` | MJPEG live view |
| POST | `/capture` | Full-resolution still. Returns `{path, bytes}` |
| POST | `/move` | `{"direction": "forward", "speed": 0.5}` or `{"vx", "vy", "yaw"}` in -1..1 |
| POST | `/stop` | Latches e-stop. `/api/estop` is the same thing |

A `/move` command lives for 0.5 s. Keep posting or the robot halts; that is the
watchdog, not a bug. Directions are `forward`, `back`, `left`, `right`,
`turn_left`, `turn_right`, `stop`. `/move` only produces motion once the robot
is standing with torque on, so on a robot that has not been through bring-up
it logs the intent and nothing turns.

## Deploy

On the Pi, once the venv exists:

```sh
deploy/install.sh
```

It installs [deploy/hexapod.service](deploy/hexapod.service) as a system
service (`After=network-online.target`, `Restart=always`, binds `0.0.0.0:8000`
so the laptop can reach it) and
[deploy/hexapod-kiosk.service](deploy/hexapod-kiosk.service) as a user service
that runs [deploy/kiosk.sh](deploy/kiosk.sh): wait for the display and the
server, then Chromium in kiosk mode on `/status`. It also adds the user to
`dialout`, `i2c` and `video` and enables lingering so the kiosk starts at boot.

```sh
journalctl -u hexapod -f               # server
journalctl --user -u hexapod-kiosk -f  # panel
```

If the kiosk unit does not start on your image, put the same line in the
compositor's autostart instead: `~/.config/labwc/autostart` on current
Raspberry Pi OS, `[autostart]` in `~/.config/wayfire.ini` on 2023 images. Panel
rotation is a display setting, not ours.

This was written against the Bookworm desktop and has not been run on the
robot's Pi yet. Expect to adjust the display detection in `kiosk.sh`.

## Configuration

Everything lives in [config/hexapod.yaml](config/hexapod.yaml).

Servo channels and calibration are ported from MakeYourPet's `chica-config-2040.txt`. The `P` numbers in that file are the same integers the firmware uses as channel indices, so `P15` is `channel: 15`. If you already calibrated your servos with Chica, copy the two microsecond values per servo straight across.

The stance block is ours, not a copy of Chica's `MODE_STANDARD`. It puts every foot about 138.5 mm out from its own coxa, which is what Chica's radius, corner angle and elongation constants work out to.

## Safety

- `limits.pulse_us` is the hard clamp. Nothing reaches a servo outside it.
- A velocity command expires 0.5 s after it arrives, whether it came over `/ws` or `/move`. A dropped websocket, a frozen page or a client that stops posting means the robot stops walking. In a steel tank the link is the thing most likely to fail.
- The board's IO thread cuts torque if it goes `control.watchdog_ms` without a fresh servo frame. A hung control loop or a crashed server parks the robot.
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
| [hexapod/controller.py](hexapod/controller.py) | 50 Hz control loop and gait state. |
| [hexapod/imu.py](hexapod/imu.py) | MPU-6050 over i2c. Bias calibration, pitch and roll. `FakeImu` for dry runs. |
| [hexapod/camera.py](hexapod/camera.py) | Owns `Picamera2`. Live stream and stills. `FakeCamera` for dry runs. |
| [hexapod/state.py](hexapod/state.py) | Polls the IMU at 10 Hz and merges board, gait, IMU, camera and network into one snapshot. |
| [hexapod/net.py](hexapod/net.py) | Which addresses the server is reachable at. |
| [hexapod/server.py](hexapod/server.py) | HTTP, MJPEG and WebSockets. |
| [hexapod/static/index.html](hexapod/static/index.html) | The client. One file, no build step, no CDN. |
| [hexapod/static/status.html](hexapod/static/status.html) | The 320x480 panel page. |
| [deploy/](deploy/) | systemd units, kiosk launcher, installer. |
| [tools/poke.py](tools/poke.py) | Standalone link test. Only needs pyserial. |
| [tools/i2cwatch.py](tools/i2cwatch.py) | Rolling i2c success rate. Finds a loose IMU lead. No dependencies. |
| [tools/preflight.py](tools/preflight.py) | Checks env, config, maths, IMU, camera and board in one run. |
| [tools/sync.sh](tools/sync.sh) | rsync the working tree to the Pi over ssh. |

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
