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

### `FileNotFoundError: /dev/ttyACM0` kills `hexapod serve`

The board was the one device whose absence was fatal at startup. IMU and
camera already degrade to a line on the status page.

Made non-fatal, because: an unplugged USB cable shouldn't take down a screen
whose job is partly to report unplugged cables; systemd with `Restart=always`
would thrash; and it's the field failure mode (a jolt inside a tank).

Two requirements when doing this — reconnect periodically rather than just
tolerating absence, and make `/move` fail **loudly** while the board is
offline rather than silently accepting commands.

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

### Centre before fitting the horn

The servo's travel is fixed inside the servo. The horn only decides where that
arc sits relative to the printed arm.

1. Command centre (`poke.py --centre`)
2. **Don't touch the servo**
3. Fit the arm at the angle the joint needs at rest
4. Screw it down

Fit the horn while the servo is anywhere but centre and the arm runs out of
travel one way and hits the end stop early. Nothing in software recovers those
degrees — only unscrewing and starting again.

The spline has discrete teeth, so landing a few degrees off is normal and
expected. That residual is what the per-servo offsets in `hexapod.yaml` trim.

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
