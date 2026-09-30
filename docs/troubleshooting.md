# Troubleshooting log

Problems hit during hardware bring-up, and what actually fixed them.
Newest sections at the bottom of each area. Symptoms are written as they
appeared, so they're searchable when something recurs.

---

## I2C bus

### `i2cdetect` says `/dev/i2c-1` doesn't exist

```
Error: Could not open file `/dev/i2c-1' or `/dev/i2c/1': No such file or directory
```

The interface isn't enabled. Nothing to do with wiring.

```
sudo raspi-config nonint do_i2c 0
sudo reboot
```

`/boot/firmware/config.txt` must end up containing `dtparam=i2c_arm=on` —
that's the line that creates the device node at boot.

### Device wired correctly but never appears

Grove cable wire order and GY-521 pin order **do not match**:

| | order |
|---|---|
| Grove cable | GND, VCC, SDA, SCL |
| GY-521 header | VCC, GND, SCL, SDA |

Pushing four wires on as a block crosses every pair — power backwards and
data backwards. Every wire ends in its own female socket precisely so it can
be placed individually.

**Rule: match by the printed label, never by colour or position.** This bit
us twice. The screen's 13-pin cable has yellow, blue and purple appearing
twice each, so colour is actively misleading there.

### Two devices, one pin

The IMU and the screen's touch controller both need pins 3 and 5.
Electrically fine — I2C is a bus, and 0x68 and 0x5d don't collide. The
problem is purely mechanical: two Dupont sockets, one pin.

First solved with a Grove I2C hub (Seeed 103321, £3.10 direct from Seeed,
£12.64 from Amazon resellers for the identical SKU). All four sockets are
wired in parallel — there's no dedicated input, whichever socket the Pi goes
into becomes the input. It needs Grove-to-female-Dupont conversion cables
(Seeed 103316, 5-pack £3.40), and because the screen's touch wires are female
and so are the conversion cables, two male-male jumper pins to bridge TP_SDA
and TP_SCL.

**The hub is out of the loom.** It was one of the two bad contacts behind the
read failures below. IMU and touch now share pins 3 and 5 directly. The
mechanical problem it was bought to solve is back, so whatever replaces it has
to be a joint you'd trust under vibration. Prove it with `tools/i2cwatch.py`
before you rely on it; this one passed a casual look.

If you do fit a hub, feed its VCC from Pi pin 1 (3V3), not a 5 V pin. It
passes straight through to all four sockets.

The body's `body_v3` floor 2 still has a 20 × 40 pocket for the hub at
`(52, 45)`. Unused for now; leave it or reclaim it on the next print.

### `dtparam` line silently ignored

Tried to slow the bus with `dtparam=i2c_arm_baudrate=50000` and nothing
changed. The line had been added at the end of `config.txt`, after the
`dtoverlay=` lines.

**A `dtparam=` applies to the most recently loaded overlay, not to the base
device tree.** Placed after `dtoverlay=imx708,cam0`, the firmware was trying
to set a camera-overlay parameter that doesn't exist, and ignoring it.

Base parameters must come **before** any `dtoverlay` line:

```
[all]
usb_max_current_enable=1
dtparam=spi=on
dtparam=i2c_arm_baudrate=50000     <-- before the overlays
dtoverlay=mipi-dbi-spi,speed=48000000
dtparam=compatible=st7796s\0panel-mipi-dbi-spi
dtparam=width=320,height=480,width-mm=49,height-mm=79
dtparam=reset-gpio=27,dc-gpio=22,backlight-gpio=18
dtoverlay=goodix,addr=0x5d
dtoverlay=imx708,cam0
```

The `compatible`, `width` and `reset-gpio` lines are correctly placed — they
*are* parameters of the `mipi-dbi-spi` overlay above them.

**The 50 kHz line came out on 29 Sep 2026.** The read failures turned out to be
two bad contacts, not bus speed. It is commented out in `config.txt` (backup at
`config.txt.bak-2026-09-29`), and `clock-frequency` under
`/proc/device-tree/axi/pcie@1000120000/rp1/i2c@74000/` reads 100000. First run
at 100 kHz: 0 errors over 259 s of service reads, about 2,600, with the touch
controller on the same pins. If failures return, restore the backup before
anything else.

### Phantom device at 0x14

An address appeared that matches nothing in the build, and persisted with the
IMU physically unplugged. Not a real device — it's `i2cdetect` reading a
corrupted bus and seeing an acknowledgement that isn't there. A symptom of an
unhealthy bus, not a discovery.

### Read failures: `OSError: [Errno 121] Remote I/O error`

This was the long one, and the first diagnosis was wrong. Both are recorded
here, because the wrong one was written down and read convincingly for weeks.

**Two bad contacts in series: the Grove cable's VCC crimp, and the hub
itself.** Either alone is intermittent. Together they mask each other, so
fixing one never restored 100% and every attempt looked like a partial fix
that needed one more thing on top. Replace the cable, take the hub out of the
loom, and the bus is clean:

```
13,639 reads, 100%, on battery, with the screen's touch controller on the same two pins
```

**The UBEC was innocent.** Its switching noise is real and measurable on the
5 V rail, but it never was the cause. With sound contacts the bus holds at
100% on battery.

What that retracts, all of it previously written down as fact here:

| earlier conclusion | status |
|---|---|
| the screen on the bus costs ~6% of reads | wrong. touch is connected for the 100% run |
| GY-521 pull-ups overload the lines | no evidence. don't desolder the `472` resistors |
| UBEC noise destroys the bus | wrong |
| a 2S-to-USB-C buck module is needed | not needed. nothing to fix |
| extra grounds on pins 9 and 14 took it 10% → 94% | the wires got disturbed, that's all |
| 1000 µF across the UBEC output | harmless, and not required |

The 1000 µF and the extra ground wires are still fitted. Leave them or don't;
neither is doing anything.

How it looked while the fault was in place. Every one of these was measured
through the bad crimp, which is why none of them agree with each other:

| configuration | success rate |
|---|---|
| IMU direct to Pi, USB-C power | 100% over 1078 reads |
| IMU via hub, screen unplugged, USB-C | 100% over 250 reads |
| IMU via hub, screen connected, USB-C | ~93% |
| IMU via hub, screen connected, UBEC power | starts ~94%, collapses to 0% within ~15 s |

An intermittent contact responds to anything that flexes the loom, and every
"fix" here involved unplugging and replugging something. That is the whole
reason the table reads like two independent causes.

**Finding it.** `tools/i2cwatch.py` at 20 Hz, then wiggle one lead at a time,
VCC, GND, SDA, SCL, and watch which one moves the number. A sound connection
sits at 100% and does not flinch when you flex the loom. Check continuity
while bending the cable at the crimp, not with it lying still; a crimp that
has bitten the insulation instead of the conductor reads fine at rest.

Then keep going after the first fault. Two in series is what made this take as
long as it did.

### Bus latching

Once a transfer is corrupted mid-byte, a slave can be left holding SDA low
waiting for clocks that never come. The bus is then stuck — 2611 consecutive
failures observed. No amount of retrying recovers it; only power-cycling the
device or pulsing SCL nine times to walk the slave to a byte boundary.

Signature: high success rate, then a hard drop to 0% that persists across
process restarts (but not across a reboot).

What corrupted the transfer here was the bad crimp dropping VCC mid-read, not
noise. The latch-up itself is real either way, and a walking robot with
vibrating connectors will reproduce it, so bus recovery in `imu.py` is still
worth building.

### `|a| = 1.235 g, expected 1.00 +/-0.10`

Preflight, 29 Sep 2026, 22:40, service stopped, servos out of the robot:

```
[PASS] 227 reads, none failed
[FAIL] |a| = 1.235 g, expected 1.00 +/-0.10   (x+0.024  y-0.062  z-1.233)
```

**Open.** What is known:

- On 13 Sep, on the bench before the body existed, it read 0.980 g.
- Magnitude does not depend on orientation, so the reading has shifted. A
  hand cannot hold an extra 0.235 g steady through a 3 s average.
- z is negative, so the chip's z axis pointed down at the time. Either the
  body was upside down on the bench, or the GY-521 is mounted face-down and
  `imu.axis_map` (still the default `[x, y, z]`) is wrong. The body's
  orientation during the run was not recorded.
- The bus is sound: no failed reads, before or after the move to 100 kHz.
- `/status` did not show it. Pitch and roll read -2.8 and -1.2 deg; tilt comes
  from ratios and stays inside ±90°, so an inverted chip reads near level.

One position cannot tell an offset from a scale error. Next is
`tools/imucal.py`, with the service stopped: z about +0.77 / -1.23 g is a
-0.23 g offset and a working chip; about ±1.23 g is a scale fault, replace the
GY-521. The six-position run also prints the `axis_map`.

---

## Servo2040

### `lsusb` shows only root hubs

Nothing plugged in, or a charge-only USB cable. `dmesg -w` while unplugging
and replugging is the definitive test — a working cable produces device
messages within a second, total silence means nothing electrical is happening.

### Board enumerates as `2e8a:0003 RP2 Boot`

That's the RP2040 bootloader. It means **empty flash** — no application to
run, so it falls back to the bootloader and presents no serial port.

Expected on a factory-fresh board. Flash the Chica driver `.uf2`:

```
sudo mount /dev/sda1 /mnt/rp2
sudo cp chica-driver.uf2 /mnt/rp2/
```

The drive vanishing mid-copy is success, not an error — the board reboots the
instant the write completes.

Afterwards it enumerates as `2e8a:000a` and `/dev/serial/by-id/` has an entry.

**Firmware source:** MakeYourPet's README points to
`EddieCarrera/chica-servo2040-simpleDriver` — that repo *is* the stock Chica
firmware. Use the hexapod driver image, **not** `servoCalibration.uf2`, which
speaks its own protocol and will leave `poke.py` silent.

### `usb-Raspberry_Pi_Pico_E66598541B2A2F33-if00`

The stable path. `/dev/ttyACM0` moves depending on what else enumerates and
in what order — use the by-id path in `config/hexapod.yaml`.

Board serial numbers differ, so this path identifies *which* Servo2040 it is,
which matters once there's a spare on the bench.

### `FileNotFoundError` or `Could not exclusively lock port` kills `hexapod serve`

The board was the one device whose absence was fatal at startup. IMU and
camera already degrade to a line on the status page.

Made non-fatal, because: an unplugged USB cable shouldn't take down a screen
whose job is partly to report unplugged cables; systemd with `Restart=always`
would thrash; and it's the field failure mode (a jolt inside a tank).

Two requirements when doing this — reconnect periodically rather than just
tolerating absence, and make `/move` fail **loudly** while the board is
offline rather than silently accepting commands.

**Written 26 Sep 2026, deployed to the Pi 29 Sep.** The text above described
the intent; `cmd_serve` still called `board.open()` bare, so the server exited
and systemd restarted it every 2 s. Adding the exclusive port lock made this
visible: the moment `poke.py --centre` held the port, the service crash-looped
eleven times behind the operator. In the repo, `board.open_tolerant()` catches
the `OSError` (missing port and locked port are both that), puts the message on
the status line and retries every 5 s. A mid-session USB drop is still not
reconnected; that is the remaining gap.

### Trace cut

The pad marked "Separate USB and Ext. Power" on the back of the board **must**
be cut before connecting anything above 5 V.

2S is 7.4 V nominal, 8.4 V off the charger. With the trace intact that reaches
the RP2040 and backfeeds up the USB cable into the Pi or laptop. Confirm with
a multimeter in continuity mode — no beep across the two pads.

Board external input is rated to 11 V, so 8.4 V is fine once separated.

**USB cannot power the servos**, trace cut or not: 18× FT5330M draw 4–6 A
walking, the Pi passes at most 1.6 A across all USB ports, and 5 V won't reach
the torque figures the servos are rated for at 7.4 V. The battery on the screw
terminals is mandatory.

---

## Screen — Waveshare 3.5" RPi LCD (F)

ST7796S display over SPI, GT911 capacitive touch over I2C at 0x5d.

### Two connection methods

The Pigo pin block sits on the GPIO header and collides with the UBEC power
wiring and the IMU. **Use the GH1.25 13-pin cable instead** — it breaks out to
individual female Dupont jumpers and keeps the header free.

### Pin conflicts

The screen wants pins 4 and 6 for power, which is where the UBEC was
originally planned. Resolved by moving the UBEC to pins 2 and 9.

Better arrangement for the final build: splice the screen's VCC and GND
directly onto the UBEC's second output lead. Same 5.2 V rail, and it keeps
both header 5 V pins free for the Pi's own current.

### Wiring (13-pin cable)

| Wire | Pi pin | | Wire | Pi pin |
|---|---|---|---|---|
| VCC | 4 | | LCD_DC | 15 |
| GND | 6 | | MOSI | 19 |
| TP_INT | 7 | | MISO | 21 |
| TP_RST | 11 | | SCLK | 23 |
| LCD_BL | 12 | | LCD_CS | 24 |
| LCD_RST | 13 | | | |
| TP_SDA / TP_SCL | 3 / 5 | | | |

### Testing the backlight before any driver work

```
sudo pinctrl set 18 op dh     # on
sudo pinctrl set 18 op dl     # off
```

This proves power, ground and GPIO 18 independently of any software. A subtle
grey wash is correct — the panel is lit with nothing driving the pixels yet.

Blinking it in a loop is far easier to see than a static state:

```
for i in 1 2 3 4 5 6; do
  sudo pinctrl set 18 op dh; sleep 1
  sudo pinctrl set 18 op dl; sleep 1
done
```

**If there's no backlight, stop and fix wiring before touching the driver.**

### Driver install (Method 1, recommended)

```
sudo raspi-config nonint do_spi 0
wget https://files.waveshare.com/wiki/common/St7796s.zip
unzip St7796s.zip
sudo cp st7796s.bin /lib/firmware/
```

Then the six config lines (see the `dtparam` ordering section above) and
reboot.

**Avoid Waveshare's Method 2** — it sets `i2c_arm_baudrate=50000`, which slows
the whole bus including the IMU.

### Rotation

Display and touch must rotate together, or taps land in the wrong place:

```
wlr-randr --output SPI-1 --transform 90
```

### Backlight control

`sudo pinctrl set 18 op dl` blanks the screen — useful in a tank where stray
light would contaminate inspection photos.

GPIO 18 is the Pi's usual PWM pin, so use GPIO 12 or 13 for PWM-dimmed
inspection LEDs later.

---

## Camera — IMX708 Camera Module 3 Wide

### `No cameras available!` with `camera_auto_detect=1` set

`dmesg | grep -i imx708` returned nothing and `/dev/i2c-10` and `-11` didn't
exist — the firmware probed both connectors at boot, found nothing, and
created no bus.

Fixed by forcing the overlay rather than relying on detection:

```
dtoverlay=imx708,cam0
```

After reboot, `/dev/i2c-10` exists and `i2cdetect -y 10` shows:

- `UU` at 0x1a — the sensor, with the kernel driver bound (`UU` means busy,
  which is correct)
- `0x50` — the module's EEPROM
- `UU` at 0x0c — the VCM focus controller

The hardware had been fine the whole time.

### Image is upside down

The camera is mounted inverted. **Fixed in `config/hexapod.yaml`:**

```yaml
camera:
  rotation: 180
```

[hexapod/camera.py](../hexapod/camera.py) turns that into
`Transform(hflip=1, vflip=1)` on the video configuration. Both flips together
are a 180, the sensor does them during readout, and it costs nothing. Restart
the service to pick it up, no reboot.

**The device-tree route was tried first and appeared not to work.** The line
was:

```
dtoverlay=imx708,cam0,rotation=180
```

It did apply (the property is on the node in `/proc/device-tree`), and it
changed nothing, because **180 is the imx708 overlay's default**:
`imx708.dtsi` has `rotation = <180>;`. Camera Module 3 is built with the
sensor inverted relative to the board, so the default already corrects for
that. To flip the image through the device tree you set `rotation=0`, not
180. Found on 26 Sep 2026; for three days this section said "reason unknown".

So the two settings do **not** cancel today: `rotation=180` in `config.txt`
is the default restated, and `camera.rotation: 180` in the yaml is the one
real flip. Take `,rotation=180` off the overlay line anyway, it is noise that
reads like a setting. What would cancel is `rotation=0` in the device tree
together with `180` in the yaml. Pick one:

- yaml `camera.rotation: 180` (current). Fixes the server, not `rpicam-still`.
- device tree `rotation=0` and yaml `rotation: 0`. Fixes every tool on the Pi.

An image that inverts after an unrelated change is one of these two having
moved, not the camera.

### `No camera number 0 found` after it had been working

Journal line from the server: `camera unavailable: No camera number 0 found`.
`rpicam-hello --list-cameras` says `No cameras available!`. Hit 26 Sep 2026,
after the body had been apart and back together; `config.txt` untouched for
three days.

First, is it the overlay or the module? `i2cdetect` and `xxd` are not on this
image, so:

```sh
for d in /sys/bus/i2c/devices/1[01]-00*; do echo "$d $(cat $d/name) $(readlink $d/driver)"; done
sudo dmesg | grep -E "imx708|dw9807"
```

`10-001a imx708` with no driver after it means the overlay applied and the
probe failed. Then dmesg says why. This time:

```
dw9807 10-000c: I2C write CTL fail ret = -121
imx708 10-001a: failed to read chip id 708, with error -5
imx708 10-001a: probe with driver imx708 failed with error -5
```

The driver powered the module and read the chip ID; nothing answered (-5),
and the focus motor did not ACK either (-121). That is electrical, not config.
`rotation=180` on the overlay line was applied and is irrelevant here.

**A raw i2c scan proves nothing on its own.** After a failed probe the kernel
leaves `cam0_reg` disabled (`/sys/class/regulator/*/state`), so 0x1a and 0x50
are silent whether or not the ribbon is good. Re-run the probe, which powers
the rail, and read dmesg again:

```sh
echo 10-001a | sudo tee /sys/bus/i2c/drivers/imx708/bind
sudo dmesg | tail -8
```

**Rule out the other connector without touching the ribbon.** The overlay says
`cam0`. Load the CAM1 variant at runtime; it creates bus 11 and probes there:

```sh
sudo dtoverlay imx708        # no cam0 = CAM1 on a Pi 5
sudo dmesg | tail -8
```

Same `failed to read chip id` on `11-001a` means it is not the connector.
The runtime overlay stays until reboot; `dtoverlay -r` did not see it.

Both connectors silent leaves the ribbon, its two connectors, or the module.
Reseat both ends, power off, per the Cable section below, and check the small
FPC latch on the module itself, not only the Pi end. `camera_auto_detect=1`
is still in `config.txt` next to the manual overlay; it never found this
module even when the module worked, so it is no help as a test.

**Outcome, 26 Sep 2026.** Reseated in CAM/DISP 0: same. Ribbon moved to
CAM/DISP 1 with the overlay line changed to plain `dtoverlay=imx708` (CAM1 is
the Pi 5 default; backup at `config.txt.bak-2026-09-26`): same errors on
`11-001a`. So the Pi's connectors are cleared. Three chips on the module are
silent at once, sensor 0x1a, focus motor 0x0c and EEPROM 0x50, and the EEPROM
does not depend on the sensor, so it is the shared path: ribbon, its two
connectors, or the module's regulator, not one dead chip. Next is a known-good
22-to-15 pin ribbon, then the module. The ribbon is still in CAM/DISP 1; going
back to CAM/DISP 0 needs `,cam0` on the overlay line again.

### Cable

The Pi 5 needs the **22-pin to 15-pin** cable — narrow fine-pitch end at the
Pi. The cable in the camera box is the older 15-pin one and won't fit. If both
ends of a ribbon are the same width, it's the wrong cable.

Silver contacts face the board. Backwards looks identical from above, so flip
and retry before assuming anything else is wrong.

### Preview errors over SSH

```
Failed to create egl preview
Failed to create drm preview
```

Expected — there's no display to draw into. The capture succeeds anyway. Add
`-n` to suppress.

### `picamera2` not importable in the venv

It's an apt package, not pip-installable. The venv must be created with
system site packages:

```
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -e '.[dev]'
```

### JPEG quality

Default captures came out at 0.6 MB for a 4608×2592 frame — aggressive
compression. For inspection photos where weld seams and corrosion detail
matter, quality 90–95 gives 2–4 MB files. Worth checking a full-size capture
before deciding.

---

## Power

### UBEC output measured 7.2 V

Exactly the pack voltage. A buck converter can't boost, so with the jumper on
7.4 V or 8.4 V it passes the input straight through.

Moved the jumper to **5.2V** on the green header. **Always measure the output
with a meter before connecting to the Pi** — jumper position and actual output
are two different claims, and 7.2 V into the GPIO header destroys the Pi
instantly.

### Sparks from the balance plug

Touched the white 3-pin balance connector while probing. That plug is for
charging only — it breaks out the junction between the two cells so a charger
can monitor each one. No fuse, thin wires, and 7.4 V across the outer pins.

**XT60 for power, balance plug for charging, nothing else.**

Damage check afterwards:

| test | probes | expect |
|---|---|---|
| Pack voltage | across XT60 | 7.4–8.4 V |
| Cell 1 | balance pin 1 → pin 2 | 3.6–4.2 V |
| Cell 2 | balance pin 2 → pin 3 | 3.6–4.2 V |

Cells must be within 0.05 V of each other. Ours read 3.60/3.60 — no damage.

Common mistake: probing the two **outer** pins measures the whole pack again,
not a cell. The middle pin is the point.

### "Reset due to low power event" on boot, but `get_throttled` is `0x0`

`vcgencmd get_throttled` reports sticky bits **since the current boot**. The
on-screen warning reports the *reset cause* from the power chip. A supply swap
— unplugging USB-C, plugging in the UBEC — is itself a low-power event.

If it doesn't reappear after a clean reboot, it was the swap. If it returns
every boot, the supply is genuinely sagging at startup, when inrush current
peaks.

### The UBEC is not the problem

Written down here for a while as the cause of the I2C read failures, with a
2S-to-USB-C buck module as the planned fix. It wasn't, and there's nothing to
buy. See the I2C read-failures section above.

At 5.2 V into pins 2 and 9 the Pi runs the full stack on battery with the bus
at 100%. Switching noise on the rail is measurable and harmless; I2C sits at
3.3 V off a separate regulator and only ever sees this rail as noise.

The general lesson, which cost days: a power supply is the easiest thing in
the build to blame and the hardest to exonerate. Noise is always present, so
any fault that correlates with switching to battery will look like a supply
problem. Rule it in with a measurement of the thing you think is failing, not
with a correlation.

### Pi header power wiring

Pins 2 and 4 are the only 5 V pins, and they're adjacent — a single 3-pin plug
covers both, so only one intact connector can sit there. Header pins carry
~3 A each and the Pi can pull 5, so split across both for the final build by
de-pinning the connectors.

Ground pins: 6, 9, 14, 20, 25, 30, 34, 39.

**Insulate any unused live crimp.** A loose 5 V lead touching pin 1 (3V3) or
pin 3 (SDA) takes out the IMU and possibly the Pi.

### Running the Pi from a laptop USB-C port

A Mac port negotiates 1.5 A or 3 A, not the 5 A the Pi 5 wants.
`usb_max_current_enable=1` tells the Pi to allow full current regardless,
which assumes a supply that can deliver. Fine for bench work with little
attached; use the official 27 W supply or the UBEC for anything real.

---

## Servos

### One joint never moves, everything else does

Reported as "only the tibia is moving" on Stand, on the leg wired to
headers 1 to 3. `tools/servolog.py` showed the femur channel (SERVO 2) being
commanded 1024 -> 1376 us like all the others, so the software was fine and the
question was what sat on that header.

Found without eyes, 26 Sep 2026, service stopped, robot on the stand:
`tools/poke.py --probe` wiggles each header +-250 us from where it is and
watches the board's current sensor.

```
header   peak A  delta
SERVO 1   +3.42  +3.17  servo moved
SERVO 2   +0.24  +0.00  NOTHING drew current
SERVO 3   +3.09  +2.85  servo moved
... 15 more, all +1.3 to +5.0 A
```

Seventeen headers pull one to five amps when driven. SERVO 2 pulls exactly
nothing, so there is no working servo on it: unplugged, lead in the wrong way
round, broken lead, dead servo, or a dead output on the board. Swapping the
SERVO 2 and SERVO 3 plugs tells the first four from the last.

**Swapped, re-probed:** SERVO 2 now +3.26 A with the servo that came from
header 3, SERVO 3 +0.33 A idle, nothing, with the servo that came from
header 2. The fault moved with the plug, so the board is fine and it is that
servo or its lead. Check the plug orientation against its neighbours first
(Feetech leads: brown −, red +, orange S; a reversed plug gives exactly this),
then the crimps, then the servo itself on the bench tester. Put the plugs back
on their original headers afterwards, or the census will record the swap.

This is also the explanation for the Stand/Sit observation. Coxa sits still
on Stand by design, the femur was dead, so the tibia was the only thing left
to move on that leg. The other five legs were never in question; the channel
map still needs the census, but no other header is dead.

Note for the probe numbers: an unloaded FT5330M pulls far more than expected
when it starts, 3 A peaks are normal, so the threshold for "moved" is set low
(0.15 A) and anything under it is a real absence, not a quiet servo.

### Left and right legs do opposite things on Sit and Stand

Symptom: press Torque or Stand and one side tucks up while the other stretches
out flat. After the refit the legs looked alike at 1500 us, but the horns were
part of the problem too: see the next section.

`tools/servolog.py` shows why: every leg is sent the **same** pulse. Sit is
1022 us on all six femurs, 789 us on all six tibias. Left and right legs are
mirror images, so the same shaft rotation moves them opposite ways in the body
frame. From a correctly centred leg, sit asks the shaft for +43 deg:

| joint | pulse | leg that is right | leg that is mirrored |
|---|---|---|---|
| coxa | 1411 | 0 deg | -16 deg |
| femur | 1022 | +78 deg, thigh up | -8 deg, thigh down |
| tibia | 789 | +132 deg, knee folded | +4 deg, knee straight |

`direction` in [config/hexapod.yaml](../config/hexapod.yaml) exists for exactly
this and every servo shipped as `1`. One side needs `-1` on all three joints,
nine servos. On this build it is probably the **right** legs: inferred from
photos, not yet confirmed by watching a joint move.

**First put on the left, then moved.** Asked which side sat correctly, the
operator said "left seems more like standing up, while right seems more like
sitting". `-1` went on the left. Torque still looked wrong, now symmetrically:
photos of the sit pose showed level thighs and knees bent about 90 to 110 deg
on all six legs. With the horn angles estimated at 1500 us (femur ~33, tibia
~134), that pose comes out only if both sides were mirrored, so the `-1`
belongs on the right. Whether the operator meant their own right or the
robot's was not recorded. Nobody has yet watched a joint move with the right
side at `-1`. Left and right are the robot's own: stand behind it, look where
the camera looks. The confirming test is `hexapod jog` from centre:
jog opens at the sit pose, not at centre, so centre the leg first: `R2 -8 35
134` (1500 us on channels 6, 7 and 8), then `R2 -8 60 134` (femur +25, 1778 us
on channel 7). With the right side at `-1` the thigh must lift. If it drops,
the right side is `1` and the `-1` goes back on the left.

**Why the port missed it.** Chica's `chica-config-2040.txt` has no direction
field: all 18 servos read `2000 1000`, identical. Chica's app applied the
mirror itself from the `L`/`R` in the servo name (`L11`, `R31`). We copied the
pin numbers and the calibration pair and not that behaviour, and nothing in
`kinematics.py` mirrors anything.

**Flipping direction does not move the centre.** `servo_angle = direction *
(joint_angle - attach_angle)`, so at the attach angle the servo reads 1500 us
whichever sign it has. Horn calibration survives a direction flip; only the
travel either side of centre mirrors. Do the horns first, then the directions.

After the flip, left and right are symmetric about 1500 us:

```
L2 femur ch10   centre 1500   sit 1022   stand 1375   (+353)
R2 femur ch 7   centre 1500   sit 1978   stand 1625   (-353)
```

Which side gets `-1` is a property of the build, not a rule. Read it off Sit:
the side whose thighs rise and knees fold is correct, the side that stretches
out flat gets the flip. Test it on a stand, and only once the horns match their
attach angles: with a horn far off its attach angle, the wrong choice drives a
joint into a stop even in the air.

### Torque looks nothing like sit, on both sides equally

With left `-1`, right `+1` (later judged the wrong way round) and centring
"done", Torque gave level thighs and knees at about 110 deg interior on all six
legs. The level thighs are the direction; the knees are the horns. Photos of
two legs at 1500 us, +-5 deg: femur horns at about 33 deg (config 35, fine),
tibia horns folded to about 134 deg where the config's `tibia_attach_angle`
said 68. The knees look to have been fitted near the *sit* angle. "Knee bent 68 deg from straight" is easy to read as a 68 deg interior
angle, which is a 112 deg fold, and a few teeth past that is 134.

Two ways out. Refit the six tibia horns with the knee at 68 deg fold (112 deg
interior) and set `tibia_attach_angle` back to 68 in the same change. Or set the config to the hardware, which is what
was done: `tibia_attach_angle: 134`. The cost is range. The pulse clamp gives
+-81 deg about centre, so with centre at 134 the knee cannot open past 53 deg.
The gait uses 118 to 132, so walking is unaffected; a straight leg is not
reachable until the horns are refitted. `hexapod check` reports it as
`tibia at +30 deg wants 344us, outside pulse_us`; the `shift` clamps in the
same output are older and unrelated (identical with 68).

Measure, do not eyeball: a protractor on the knee at 1500 us, interior angle,
then attach = 180 minus that. The photo estimate here was +-5 deg.

### Three femurs died on 26 Sep 2026: what is known and what is not

SERVO 2 (R3 femur), SERVO 17 (L1 femur), SERVO 8 (R2 femur). The robot sat on
blocks with its legs free all day; no foot touched anything. **Why they died is
not established**, and there is no evidence of one shared cause. The accounts
first written here on the day claimed more than the evidence held; they are
listed at the end of this section so nobody revives them.

**What the board can measure.** One current sensor for the whole servo rail,
the total of all 18, and the pack voltage. No per-servo current. A reading can
be pinned to one servo only while that servo is the only thing moving, which is
what `poke.py --probe` arranges. A hold or a gait cannot.

| servo | what was seen | what is established |
|---|---|---|
| SERVO 2, R3 femur | Silent on the first probe of the day, +0.00 A over idle. The silence followed the servo through a plug swap | The fault is the servo, its lead or its plug orientation, not the header. Not bench-tested. When and why: unknown. It was already dead when first probed, in the afternoon |
| SERVO 17, L1 femur | Probe peaks of 5.2, 4.4, 5.4, 4.4 and 5.0 A over five runs one afternoon, while the others read 1.3 to 3.7 A. Unchanged by re-centring every horn. Then 0.33 A: dead | Consistently the hungriest servo, 13 to 38% above the FT5330M's 3.9 A stall spec on brief peaks. Cause unknown. The refit not changing it counts against an off-centre arm |
| SERVO 8, R2 femur | Normal in the 18-channel sweep (2.52 A peak, settled to 0.17 A). About 40 minutes later, during `poke.py --centre`, the operator reported the total over 10 A, and about a minute after that reported SERVO 8 dead | One stalled FT5330M draws about 4 A, so over 10 A total means three joints near stall, or a failed unit. Which ones was not measured. Three minutes after the death report a newly fitted femur servo was reported very hot; which one (L1, R2, or the DS3235 on R3) was not recorded |

**What holds for all three.** Nothing in the stack acted on current or voltage.
The server and `poke.py` displayed the total and nothing cut torque, including
when the operator saw it pass 10 A shortly before SERVO 8 died. Chica's config has `WARN_CUR 2 8 10` and `WARN_VOL 2 6.4 6`,
but those are enforced by Chica's Android host app. The Servo2040 firmware
(`chica-servo2040.cpp` in EddieCarrera/chica-servo2040-simpleDriver) only
reports current and voltage when asked, and switches the relay when told.
Replacing the app with the Pi stack dropped the cuts.

**Hypotheses, and where they stand:**

| hypothesis | status |
|---|---|
| Arms fitted off-centre, so 1500 us drives a thigh into the frame | Possible for SERVO 8, not observed. Does not fit SERVO 17. Unknown for SERVO 2. The two femurs photographed at 1500 us after the refit sat at about 33 deg against config 35 |
| Tibia overdrive: horns at ~134 with the old config asked the left knees for 198 deg | Would stall the knee servos, not the femurs. Shin into thigh is internal to the femur subassembly, so no static load reaches the femur servo. Every knee servo survived. Possible only between the refit and the 16:48 config change |
| Direction bug driving right feet into the floor | The robot was never on the floor that day. Unknown for earlier days |
| Overvoltage | Not supported. The pack read 6.95 to 7.78 V all afternoon (uncalibrated), inside the 4 to 8.4 V rating. The voltage at each death was not recorded |
| Relay | Songle's datasheet rates the SRD-05VDC-SL-C changeover contacts at 7 A 28 VDC resistive and 3 A inductive, below the 10 A on the case. Over 10 A through them is out of rating. No arcing was observed |

**What the probe can and cannot tell you.** It showed SERVO 17 as the hungriest
servo on all five runs before it died, so a high reading is worth acting on. It read SERVO 8 normal an
hour before it died, so **a clean probe does not clear a servo**. It exercises
one header at a time with the legs free; standing and walking loads never
happen during it.

**What the trips catch, and what they miss.** On the Pi since 29 Sep 2026,
22:30. Until then the Pi ran code from 25 Sep with none of them, which a
checksum dry run of `tools/sync.sh` showed:

- The board's IO thread estops, torque on, on a total over 10 A (mean over 1 s),
  a held sit over 1.5 A, a held stand over 5.5 A, under 6.0 V (mean over 2 s),
  or no telemetry for 2 s. It runs under `hexapod serve`, `jog` and `neutral`
  alike. **10 A needs three simultaneous stalls**. A single stall at a still
  pose now trips on the sit/stand cuts. Walking only has the 10 A ceiling. The
  board also refuses torque while it is offline, and does not replay an earlier
  request when the port comes back. Both web pages show the trip reason.
- `poke.py --centre` cuts at 10 A, and also at 1.5 A mean over 1.5 s once the
  pose has settled. Legs free at a static pose read about 0.2 A, so one stalled
  servo trips it. This is the situation SERVO 8 died in.
- `poke.py --probe` cuts at 10 A over the whole run, stops the run when a
  header is still pulling 0.8 A over idle a second after it returns (the
  single-stall case), and stops before wiggling any header if the robot
  already pulls more than 0.5 A at rest.
- `poke.py --census` drops torque while you type each answer.
- Every `poke.py` mode drops the relay after 2 s without a current reading, and
  a second Ctrl-C cannot interrupt the relay-off.

`hexapod serve` now uses that mode-aware check for a **held** pose: after
pulses are still for `safety.still_s` (1 s), sit/legs-free trips at
`sit_cut_a` (1.5 A) and standing still trips at `stand_cut_a` (5.5 A). Walking
(pulses moving) still only has the 10 A ceiling. Standing current is estimated
(about 3 to 4 A); if a healthy stand trips, raise `stand_cut_a` after you log
it. The ceilings still want a clamp-meter calibration, a weighed robot, and a
deliberate two-second stall to confirm the sensor reads about +4 A.

**Fitting horns.** With the servo powered and holding 1500 us: on the bench
tester in neutral, or under `poke.py --centre` with only that leg plugged in.
Never with torque off. Then with torque off, swing the joint by hand. A
well-fitted arm reaches the servo's end stops about equally either side of its
working range, and nothing touches the frame.

**Retracted.** Written here on 26 Sep and not supported by the evidence:
"held at 10 A+" for all three (only SERVO 8's death came with a 10 A reading);
R3 femur "already dead at 06:00" (no such observation); the relay "was
arcing"; "femurs go first because they have the most leverage into a stop";
the tibia overdrive as the cause of the femur deaths; "the shafts moved while
the legs were handled" (unpowered Feetech legs hold position); "the only way
that pulls 10 A is arms fitted off-centre"; "this alone would have saved both
servos"; "will a third go? Not from these causes" (a third died that evening);
"both right after the probe flagged them high" (only SERVO 17 was flagged);
"20 to 30 s" stall holds while photographing (never timed).

### Centre before fitting the horn

The servo's travel is fixed inside the servo. The horn only decides where that
arc sits relative to the printed arm.

1. Command centre (`poke.py --centre`)
2. **Don't touch the servo**
3. Fit the arm at that joint's `geometry.*_attach_angle` in
   `config/hexapod.yaml`, not at a sit or stand pose. On this build: coxa -8
   (leg 8 deg back from its mount line), femur 35 (thigh 35 deg above
   horizontal), tibia 134 (knee folded 134 deg, a 46 deg interior angle). One
   value covers all six legs, so a single replacement must match it. To fit at
   a different angle, refit every leg and change the config to match.
4. Screw it down

Fit the horn while the servo is anywhere but centre and the arm runs out of
travel one way and hits the end stop early. Nothing in software recovers those
degrees — only unscrewing and starting again.

The spline has discrete teeth, so landing a few degrees off is normal and
expected. Trim the residual per servo by shifting its `us_neg45`/`us_pos45`
pair together in `hexapod.yaml`: `2050/1050` moves centre +50 us, about 4.5 deg.
There is no separate offset field.

### Horn screws

M2.5 × 6 socket head through the horn into the heat-set inserts in the printed
part. Feetech round horns are often drilled ~2 mm, so the holes may need
opening to 2.6 mm — the horn should be **clamped** between screw head and
printed part, not threaded.

### Servo tester (HJ / DollaTek 4-channel)

Amazon lists it as "Battery Powered", which means powered *from* an external
battery — there's no cell inside and no separate power socket on some units.
Power goes in through the dedicated 5th header, or through any servo port if
there isn't one (all ports share the rail).

Feed it from the UBEC at 5.2 V. **Not 2S directly** — 7.4 V exceeds its
4.8–6 V limit.

Pin order is printed as `S + −`: signal, positive, ground. Match wire colours
to the labels, not to position.

Centring: neutral mode, or manual showing **1500** µs.

### Servo2040 pin numbering

Config is zero-indexed `P00`–`P17`; physical headers are labelled
`SERVO 1`–`18`. **Physical = config pin + 1.**

---

## Process notes

Things that cost time and were avoidable.

**Read the label, never the colour or the position.** Every connector in this
build. Grove cable vs GY-521 pin order, the screen's 13-pin cable with three
duplicated colours, the UBEC output plugs. This caused more lost time than any
other single thing.

**Establish a long enough baseline before drawing conclusions.** The first
"clean" USB-C run was 118 reads — about six seconds, shorter than the interval
at which the failure appeared. Several hours were spent treating an
unestablished baseline as fact. The eventual three-minute run settled it
immediately.

**Change one thing at a time.** Late in the session, multiple changes went in
together and the state got worse than the starting point — a working IMU
became an absent one plus a phantom bus address. Everything recovered, but
nothing was learned from that hour.

**Stop when tired.** The last hour of the long I2C session produced three
changes that each made things worse, including ones that should have been
neutral. The findings that mattered were already established by then.

**Keep a known-good state to return to.** Before changing wiring, note exactly
what's plugged where. After a session of moving wires, that note is the
difference between a two-minute recovery and an hour of bisection.

**Two faults in series look like one weird fault.** The bad crimp and the bad
hub each hid the other. Every fix produced a partial improvement, which read
as "right direction, not enough" and pulled the next fix further down the
wrong road: filtering, grounds, bus speed, a buck module. The tell was there
the whole time: nothing ever reached 100%. A fix that only moves the number is
evidence you haven't found the fault.

**Don't compare measurements taken through an intermittent connection.** The
whole read-failure table disagreed with itself because unplugging things to
change configuration also reseated the bad crimp. Every number in it was real
and none of them meant what they appeared to.

**Write the conclusion down with its confidence.** The UBEC diagnosis sat in
this file as fact, and would have been believed in three months by someone
with no memory of how thin the evidence was. Retractions stay in the file next
to what they replace, for the same reason.
