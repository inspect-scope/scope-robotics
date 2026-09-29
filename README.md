# scope-hexapod

Pi-side control for a MakeYourPet hexapod. Replaces the Chica Server and Chica Client Android apps.

The Servo2040 keeps its existing Chica firmware. Everything above it moves to the Pi.

```
browser on the LAN  --wifi-->  Pi  --USB serial-->  Servo2040  -->  18 servos
joystick, sliders   /          |   --i2c-------->  GY-521 IMU
480x320 panel   ---            |   --CSI--------->  camera
                        one FastAPI process: IK, gait, state, MJPEG
```

One process owns the hardware. The serial port, the i2c bus and the camera
can each be opened by a single process, so `hexapod serve` holds all three and
both pages (`/` for the operator, `/status` for the panel) are plain HTTP
clients of it. Do not run `poke.py`, `preflight.py` or anything else that
touches a device while the server is up. The serial port is opened exclusively,
so a second opener fails at once with "Could not exclusively lock port"; stop
`hexapod.service` first. Before that lock existed, two processes could share
the port and both drive the servos, which looks like random twitching and is
not a hardware fault.

Hardware problems hit during bring-up, and what fixed them, are in
[docs/troubleshooting.md](docs/troubleshooting.md). Check there first when a
device does not appear, the bus starts failing, or the board will not
enumerate. Diagnoses that turned out to be wrong are kept there too, next to
what replaced them; the i2c one was believed for weeks.

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

Every servo goes to 1500 us. This is your mechanical zero, and it is the pose
the horns must be fitted in, each joint at its `geometry.*_attach_angle`.
Chica's fit is thigh **35° above horizontal**, knee **folded 68° from
straight** (112° interior, a fairly open knee), leg 8° back from its mount
line, and it keeps the full knee range. This build's tibia horns sit at about
134° fold (46° interior), so `tibia_attach_angle` is 134 to match; if you
refit them at 68, set it back to 68 in the same change. All six identical, and
one value covers all six legs, so a replacement servo must match it. Do not fit
the arms in a pose that "looks like standing" or "looks like sitting". The
attach angles must describe the pose you actually fitted; check `hexapod check`
for lost range.

**5. Fix the joint directions.** One leg at a time, still elevated.

```sh
hexapod jog
```

Two checks per leg:

- `L1 -8 35 134` (the three `geometry.*_attach_angle` values; use whatever the config says) must put all three servos back at 1500 us. Jog opens at the sit pose, so this is also how you centre a leg.
- `L1 0 0 0` must point the leg straight out sideways, horizontal, with the knee as open as the pulse clamp allows: straight with a tibia attach angle of 68, about 53° fold at 134.

If a joint moves the wrong way, flip `direction` from `1` to `-1` for that servo in the config. If a joint is centred but at the wrong angle, correct its attach angle.

Femur and tibia are easier to read as a sit-to-stand move than as a single
angle. These are the neutral poses the solver produces at `sit_height` 40 and
`ride_height` 80, the same for all six legs to within 0.3°:

| joint | sit | stand | sit → stand |
|---|---|---|---|
| coxa | 0.1° | 0.1° | nothing |
| femur | 77.8° | 46.2° | drops 31.6° |
| tibia | 131.7° | 118.6° | knee opens 13.1° |

In jog, `R3 0 77.8 131.7` then `R3 0 46.2 118.6`. The femur must rotate **down**
and the knee must **open**, pushing the foot further below the body. A femur
that rises is an inverted femur, a knee that closes is an inverted tibia, and
both can be wrong at once. Coxa barely moves between the two poses, so check it
on its own: `R3 coxa 20` swings the leg counter-clockwise seen from above.

`tibia` is the fold at the knee, 0 = tibia in line with the femur. The interior
femur-to-tibia angle you would put a protractor on is 180 minus that, so 48°
sitting and 61° standing. Same movement, and the two conventions run opposite
ways, so be clear which one a number is in before acting on it.

Flipping `direction` does not move the servo's centre. `servo_angle =
direction * (joint_angle - attach_angle)`, so the servo still sits at 1500 us
when the joint is at its attach angle whichever sign `direction` has; only the
travel either side of it mirrors. Nothing needs remounting after a flip. A horn
that is a spline tooth out shows up as the wrong *angle* in the attach-angle
check above, not as the wrong direction.

Do all six legs. Left and right are mirror images, so the signs are not
guaranteed to match across a pair, and copying R3's answer to the other five is
how you get one leg walking backwards.

**Watch what actually went to the servos.** With `hexapod serve` running, on
the Pi or a laptop:

```sh
python3 tools/servolog.py                    # localhost
python3 tools/servolog.py --host hexapod.local --csv stand.csv
```

Then press Stand, Sit or Torque on `/`. It prints one line per channel that
changed, with the leg and joint, the SERVO header number, the joint angle the
solver asked for and the pulse that went out:

```
14:02:11.318  R3 femur   SERVO  2 (ch  1)    77.8 ->   46.2 deg   1500 -> 1622 us (+122)
```

Ctrl-C prints a summary: start, end, net travel and range for every channel,
which ones never moved, which hit the pulse clamp, and whether every leg was
asked for the same joint change (for Stand and Sit they should be). It cannot
see which way the leg physically turned, so read it next to the robot: if
SERVO 2 went +122 us and the knee went up when the table says the femur should
drop, that is the servo whose `direction` flips.

**If a commanded joint does not move at all, take a census.** The log says
which header was driven; the census says what is on it, with the config out of
the loop. Stop the service first, legs off the ground:

```sh
sudo systemctl stop hexapod
python3 tools/poke.py /dev/ttyACM0 --census
```

**Is a servo dead, suspect, or fine?** Same tool, no eyes needed:

```sh
sudo systemctl stop hexapod
python3 tools/poke.py /dev/ttyACM0 --probe
```

Each header is wiggled from where it is and the board's current sensor is read
the whole time. One line per servo with the peak while moving and what it
settled to a second later, then a summary. Read it like this:

| reading | means | do |
|---|---|---|
| peak 0.15 A or more over idle, settles to idle | moved; healthy FT5330Ms have read 1.3 to 3.7 A | nothing |
| peak under 0.15 A over idle | **dead**, or unplugged, or lead broken | swap its plug with a neighbour: fault follows the plug = servo or lead |
| peak over 4.5 A (above the FT5330M's 3.9 A stall spec) | **suspect**: an external bind, or internal damage | torque off, turn the joint by hand: a catch is a bind to fix first; free, suspect the servo |
| still pulling 0.8 A+ a second after the move | **stalled or bound** | something is in the way, find it before it cooks |
| "well above the pack" | 40%+ hungrier than its siblings | watch it; re-run after a session and compare |

The HIGH line is set for the FT5330M. A healthy DS3235 PRO peaks up to 4.4 A,
close to it.

In five probe runs one afternoon, SERVO 17 (L1 femur) peaked at 4.4 to 5.4 A
while the others read 1.3 to 3.7 A, and it was dead by evening. SERVO 8 read
normal about 40 minutes before it died, so a clean probe does not clear a
servo. Torque comes on at the `--base` pose (1500 us without it); the probe
then stops before wiggling any header if the robot already pulls more than
0.5 A at rest, because a servo stalled at rest hides inside every per-header
reading. It also stops the run, relay off, when a header is still pulling a
second after it returns, and when the board stops answering for 2 s. Run it
legs free, at the start of a session and after any mechanical change.

**Mapping headers to joints** is `--census` instead of `--probe`, and does
need eyes. Each of the 18 headers wiggles ±200 us in turn and you type what moved as
position and joint, `RR femur`, `FL coxa`. Positions are FL FR ML MR RL RR;
front is the camera end, left and right are the robot's own. At the end it
prints the header-to-joint table and a `servos:` block to paste into
[config/hexapod.yaml](config/hexapod.yaml). Because you name legs by where
they are, not by what the config calls them, this also settles whether R1 is
really at the front.

**Or let the board tell you which headers have a live servo.** No eyes needed:

```sh
P=$(curl -s localhost:8000/api/state | python3 -c 'import json,sys; print(json.dumps(json.load(sys.stdin)["pulses"]))')
sudo systemctl stop hexapod
python3 tools/poke.py /dev/ttyACM0 --probe --base "$P"
```

It wiggles each header from where the servo already is and watches the current
sensor. A servo that moves pulls 1 to 5 A; a header with nothing on it, or a
dead servo, pulls nothing. On this robot that found SERVO 2 silent while the
other 17 answered, which was the whole of "only the tibia moves".

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
transaction. It is also not the power supply, however much it correlates with
running off the battery. Find the lead:

```sh
python3 tools/i2cwatch.py
```

It reads WHO_AM_I at 20 Hz and prints a rolling success rate. Wiggle one lead
at a time, VCC then GND then SDA then SCL. The one that moves the number is
the bad one. A sound connection sits at 100% and does not move when you flex
the loom. On a GY-521 the usual cause is the pin header being pushed through
the board but never soldered.

Keep going after the first bad contact. On this build there were two in
series, a crimp in the Grove cable and the i2c hub, and each masked the other:
fixing one moved the number without ever reaching 100%, which sent days of
work into filtering and grounding that was never needed. Anything short of
100% means you are not done.

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

**Bus errors are retried.** Each i2c transaction gets `imu.bus_retries` extra
attempts 2 ms apart; the state poller rides out the ones that get through and
reopens the chip after five in a row. `/api/state` reports both counts:
`imu.read_errors` is every bus error seen, `imu.read_failures` only the ones
the retries did not clear.

A sound loom sits at zero. This one ran at 5 to 10% errors through bring-up
and that was two bad contacts, not noise; with them fixed it holds 13,639
reads at 100% on battery, screen on the same two pins. So the retries are
insurance against vibration, not a way to live with a loom that needs
fixing. Any sustained `read_errors` is a contact to go and find. A rising
`read_failures` is the same thing, worse, and is what `/status` complains
about.

`imu.axis_map` in [config/hexapod.yaml](config/hexapod.yaml) says how the
chip is mounted. Each entry is the chip axis pointing along the body axis
(+X right, +Y forward, +Z up), with an optional minus sign. Set it once the
board is bolted down: tip the robot nose up and check the horizon on `/status`
moves down; lean it right and check the horizon tilts. If the IMU is missing
the server still starts and the panel says so.

## Web interface

- Live view top left, from the camera's low-resolution stream. **Capture** (or `C`) saves a full-resolution still and shows the file name.
- A 3D commanded-pose view sits next to the top-down stance. orbit / world / follow. drag to look around. green pads are closed foot switches.
- Left pad moves, right pad turns. Click the pad block to capture the keyboard:
  `W A S D` moves, `Q E` turns, `Esc` releases. Those keys do nothing until then. `J` twice jumps, same as the button.
- Space is e-stop from anywhere. So is the red button, and `POST /stop` if the page is wedged.
- Stand, Sit and Torque are separate. Sit before you cut torque. Jump is two clicks. Bounce loops until off.
- Gait and stance-mode dropdowns sit on that row. walks are the Chica set (tripod / triple / ripple / wave). modes are normal / speed / offroad.
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

The camera is mounted upside down, so `camera.rotation` is `180` and both
streams get a libcamera `Transform(hflip=1, vflip=1)`. Only 0 and 180 are
allowed; a sensor can flip but not transpose. The device tree can do the same
job for every tool on the Pi, but note the `imx708` overlay's default is
already `rotation=180`, so the flip there is `rotation=0`. Use one or the
other, not both; two flips cancel.

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
| GET | `/api/state` | The state snapshot, once. Includes `angles` per leg and the 18 `pulses` last sent |
| GET | `/api/config` | Static: leg names, coxa positions, servo channels and directions, pulse clamp |
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
- **Trips.** The board reports one total current for all 18 servos and the
  pack voltage. Its IO thread latches the e-stop, torque on, when the total's
  mean over `safety.current_cut_s` (1 s) exceeds `current_cut_a` (10 A), when
  pulses have been still for `still_s` (1 s) and the mean exceeds `sit_cut_a`
  (1.5 A, sat / legs free) or `stand_cut_a` (5.5 A, standing still), when
  the voltage's mean over `volts_cut_s` (2 s) drops under `volts_cut` (6.0 V),
  or when telemetry stops for 2 s. That covers `hexapod serve`, `jog` and
  `neutral`. The reason shows as the status line and as `safety_trip` in
  `/api/state` until you clear the e-stop (`clear` in jog). The 10 A ceiling
  still needs three simultaneous stalls. The settled sit/stand cuts catch
  one hung servo at a held pose, which is how the femurs died. Walking
  (pulses moving) only uses the 10 A ceiling. The total cannot name the
  servo; run `poke.py --probe` afterwards. The board also refuses torque
  while it is offline, and does not replay an earlier request when the port
  comes back. See [docs/troubleshooting.md](docs/troubleshooting.md).
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
| [hexapod/gait.py](hexapod/gait.py) | Walk catalog and bounce/jump overlays. |
| [hexapod/mode.py](hexapod/mode.py) | Stance presets: normal, speed, offroad. |
| [hexapod/controller.py](hexapod/controller.py) | 50 Hz control loop and gait state. |
| [hexapod/imu.py](hexapod/imu.py) | MPU-6050 over i2c. Bias calibration, pitch and roll. `FakeImu` for dry runs. |
| [hexapod/camera.py](hexapod/camera.py) | Owns `Picamera2`. Live stream and stills. `FakeCamera` for dry runs. |
| [hexapod/state.py](hexapod/state.py) | Polls the IMU at 10 Hz and merges board, gait, IMU, camera and network into one snapshot. |
| [hexapod/net.py](hexapod/net.py) | Which addresses the server is reachable at. |
| [hexapod/server.py](hexapod/server.py) | HTTP, MJPEG and WebSockets. |
| [hexapod/static/index.html](hexapod/static/index.html) | The client. One file, no build step, no CDN. |
| [hexapod/static/status.html](hexapod/static/status.html) | The 320x480 panel page. |
| [deploy/](deploy/) | systemd units, kiosk launcher, installer. |
| [tools/poke.py](tools/poke.py) | Standalone link test; `--census` maps headers to joints by eye, `--probe` finds dead headers by current. Only needs pyserial. |
| [tools/i2cwatch.py](tools/i2cwatch.py) | Rolling i2c success rate. Finds a loose IMU lead. No dependencies. |
| [tools/servolog.py](tools/servolog.py) | Prints every servo that moves while you press buttons, and a summary. Stdlib only. |
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
