"""
"Scope" – electronics body for the MYP Chica hexapod frame.
Coordinate system = MYP frame.stl: origin at frame centre, Z=0 is the TOP face of the 16 mm frame. Units mm.

Stack, bottom to top:
  floor1_battery_ubec.stl        sits on the frame (M2.5 into the frame's pilot holes). 2S LiPo bay + UBEC pocket.
  floor2_pi_servo2040_relay.stl  on 4 posts, z = 45. Pi 5 (GPIO on the left, ports to the rear), Servo2040, relay.
  roof_screen.stl                on 4 posts, z = 87. 3.5" RPi LCD (F) touch screen from below.
  skirt_walls.stl                plain shell, NO fasteners: rests on 8 pads at floor 1's edge, captured at the top by a
                                 5 mm lip on the roof. Camera pod, vents, port cut-outs, switch clearance hole.
Teardown: 4 roof screws -> roof off -> skirt stays standing -> floor 2 lifts straight out through it.

Fasteners (v2 – everything threads straight into PLA, no inserts):
  M3 x 6 pozi pan   structural: roof -> floor-2 posts, roof -> skirt tabs, floor 2 -> floor-1 posts (counterbored)
  M2.5 x 5          Pi 5 standoffs, GY-521 bosses, screen retainer bosses
  M1.6 x 4 cap      Camera Module 3 bosses
  Servo2040 / relay / UBEC / Grove hub sit in lipped pockets with zip ties (hole-pattern agnostic).
Edit PARAMETERS and re-run:  python3 chica_body.py
"""
import math
import cadquery as cq

# ============================================================== PARAMETERS
T = 3.0                                  # plate thickness
FLOOR_W, FLOOR_L = 130, 160              # both floors share this outline so the skirt slides over them
CHAMFER = 12                             # clipped corners
F2_Z = 45                                # underside of floor 2 (Pi floor)
ROOF_Z = 87                              # underside of roof
ENGRAVE, FONT = 0.6, 6

# --- fasteners (thread-forming pilots for PLA; drill out with 1.3 / 2.1 / 2.6 if a screw won't start)
M16_TAP = 1.3                            # M1.6 pilot
M25_TAP = 2.1                            # M2.5 pilot
M25_CLEAR = 2.7                          # M2.5 clearance
M3_TAP = 2.6                             # M3 pilot
M3_CLEAR = 3.3                           # M3 clearance
CB_D, CB_H = 6.5, 1.2                    # counterbore for the M3 pan head (Ø5.6 x 2): head sits 1.2 mm into the plate
POST_PILOT_DEPTH = 8                     # M3 pilot depth in post tops (screw reaches 4.2 mm after a counterbored plate)
POST = 12                                # floor-1 posts: square
POST_FOOT = 16                           # flared collar at the post base
FOOT_H = 3
POST_THIN, POST_WIDE = 7, 16             # floor-2 posts: rectangular, thin side toward the boards (Pi clearance)

# --- frame attachment (measured from frame.stl): 1.5 mm pilot holes on the side rails, x = ±44, y = -40 … 40
FRAME_HOLES = [(sx * 44, y) for sx in (-1, 1) for y in range(-40, 41, 10)]

# --- floor 1 (bottom): battery + UBEC
BAT_W, BAT_L, BAT_H = 48, 140, 27        # HOOVO 2S pack
BAT_CENTER = (-3, -5)                    # bay centre; leads exit the rear end
BAT_RAIL_H, BAT_STOP_H = 6, 8            # side rails; front end-stop ("right angle") so the pack can't slide forward
UBEC_W, UBEC_L = 45, 35                  # Nuofany 2-8S 8A UBEC, 45 x 35 (X x Y)
UBEC_CENTER = (44.5, 5)                  # offset so its zip slots miss the frame screw holes at (44, ±20)
UBEC_PILLAR_H = 4
POSTS_1 = [(48, 72), (-48, 72), (49, -62), (-58, -24)]     # floor1 -> floor2. Rear-right moved aft, clear of the switch;
                                                           # rear-left moved forward, out from under the relay's corner pillar
LEFT_SLOT = ((-34.5, 0), 9, 110)         # servo leads down through the frame opening, left of the pack
RIGHT_FRONT_SLOT = ((33, 46), 12, 32)    # straps / leads, fore of the UBEC
RIGHT_REAR_SLOT = ((33, -42), 12, 44)    # switch + UBEC leads, aft of the UBEC
# switch bracket: the switch clamps to floor 1, not to the skirt -> the skirt carries no wires and lifts straight off
SWITCH_BRACKET_W, SWITCH_BRACKET_T, SWITCH_GUSSET = 30, 3, 4
SWITCH_BRACKET_HOLE = 20.2               # the switch's panel hole (Ø20 body)
SWITCH_BEZEL_D = 24                      # MEASURE: outer Ø of the switch bezel/cap -> skirt gets a Ø(bezel + 0.5) clearance hole

# --- floor 2 (top): Pi 5 + Servo2040 + relay
PI_W, PI_L = 56, 85
PI_CENTER = (-25, 25.5)                  # long axis along Y, USB/Ethernet to the rear (-Y), GPIO along the left edge; y -17..68
PI_HOLE_DX = 49                          # across the board
PI_HOLES_FROM_PORT_EDGE = (23.5, 81.5)   # Pi 5 drawing: holes 3.5 mm from the SD-card edge, 58 apart -> 23.5 / 81.5 from the port edge
PI_STANDOFF_H, PI_STANDOFF_D = 6, 7
S2040_W, S2040_L = 42, 62                # fits perfectly – don't change
S2040_CENTER = (28, -19)                 # right column, y -50..12: 30 mm free behind it (USB-C + screw terminal), everything in front free (feet sensors)
S2040_PILLAR_H = 5
RELAY_W, RELAY_L = 65, 40                # relay module 65 x 40, long axis along X
RELAY_CENTER = (-29.5, -54.5)            # rear left, long axis along X, behind the Pi's plugs
RELAY_PILLAR_H = 4
POSTS_2 = [(30, 76.5, "y"), (-30, 76.5, "y"), (61, -60, "x"), (-61, -5, "x")]   # floor2 -> roof; 3rd item = thin axis
SERVO_SLOT = ((57.5, -21), 11, 52)       # main servo-lead pass-through beside the Servo2040's outboard edge
SERVO_SLOT_REAR = ((28, -70), 32, 10)    # behind the board: more leads + the 7.4 V feed to its screw terminal
SENSOR_SLOT = ((23, 20.5), 30, 10)       # in front of the board: feet-sensor leads + Grove cables down to the hub
RELAY_SLOT = ((-40, -75.5), 20, 6)       # relay wiring down to the UBEC / Pi rail
PI_REAR_SLOT = ((-25, -13.5), 55, 10)    # along the Pi's port edge: UBEC 5 V + relay control up to the GPIO
GPIO_SLOT = ((-57, 40), 6, 24)           # beside the GPIO edge
FRONT_NOTCH = ((0, 76.5), 43, 10)        # clears the camera pod (inside the skirt's front wall) when floor 2 lifts straight out
ANCHOR_Y = (13, 19)                      # zip-tie anchors for the lead bundle, along the right edge (clear of the lead slot)
# GY-521 IMU on two bosses (its Ø3 holes, M2.5 x 5) – header edge faces -Y (rear), holes on the +Y edge
IMU_W, IMU_L = 20, 16
IMU_CENTER = (17, 40)
IMU_HOLE_PITCH, IMU_HOLE_FROM_EDGE = 15.24, 2.5   # MEASURE with calipers; bosses are Ø6 so ±1.5 mm can be drilled out
IMU_BOSS_D, IMU_BOSS_H = 6.0, 3.5
# Grove I2C hub (6 port, 40 x 20) in a lipped pocket, long axis along Y; use its inboard socket row
HUB_W, HUB_L = 20, 40
HUB_CENTER = (52, 45)
HUB_PILLAR_H = 4

# --- roof + screen (Waveshare 3.5inch RPi LCD (F): PCB 61.00 x 92.44, display 49.36 x 73.84), portrait, mounted from below
SCREEN_MODULE = (61.0, 92.44)
SCREEN_POCKET_CLEAR = 0.4
SCREEN_WINDOW = (50.4, 74.8)
SCREEN_WINDOW_DX, SCREEN_WINDOW_DY = 0.0, 0.0    # measure the panel offset on the PCB, then set
SCREEN_CENTER = (0, 18)
SCREEN_LIP_H, SCREEN_LIP_T = 6.0, 1.5
# Screen retention: a printed retainer plate clamps the PCB into the pocket from below, on 4 bosses (M2.5 x 6 self-tap)
SCREEN_STACK_T = 4.5                     # glass top face -> PCB back face (MEASURE; typical 4-5 mm)
RET_T = 2.5                              # retainer plate thickness
RET_GAP = 0.4                            # retainer clears the lip end by this much
CLAMP = 0.4                              # pads reach this far past the PCB back -> clamp by flex
BOSS_D, BOSS_HOLE = 7.0, M25_TAP         # Ø7 bosses, M2.5 x 5
RET_BOSSES = [(38, 60), (-38, 60), (36.5, -34.5), (-36.5, -34.5)]      # outside the lip, clear of roof posts and vents
RET_OUTLINE = (83, -38, 66)              # width, y_min, y_max of the retainer plate
RET_CONNECTOR_END = -1                   # the GH1.25 / pogo / header end of the PCB faces this way (-1 = -Y = rear)
# What's on the back of the PCB (measured from the connector edge and the side edges, in mm) – all of it must hang
# through the retainer's opening untouched:
SCREEN_HOLES_FROM_SIDE, SCREEN_HOLES_FROM_END = 5.5, (5.0, 65.0)   # 4 mounting holes, with M2.5 standoffs fitted
SCREEN_STANDOFF_D, SCREEN_STANDOFF_H = 6.0, 12.0
SCREEN_HEADER = (5.0, 15.0, 7.0, 25.0)   # pin header on one side: x from side edge, y from connector edge, width, length
SCREEN_CLEAR_TOP = 20.0                  # the far (top) 20 mm strip of the PCB back is bare (logo area)
RET_TOP_OVERLAP = 6.0                    # retainer bar overlaps that bare strip by this much -> two clamp pads live there
RET_PAD_W = 6.0
RET_SIDE_PAD_Y = 48.0                    # side fingers press the edge strip here (above the header, below the top standoffs)
RET_SIDE_PAD_REACH = 7.0                 # ... reaching this far in from the PCB edge

# --- skirt: nothing protrudes inside it, so floor 2 (130 x 160) passes through it in either direction
SKIRT_CLEAR, SKIRT_T = 1.0, 2.4
PAD_H, PAD_L = 2.0, 12                   # floor-1 landing pads under the skirt wall: 8 of them, z 0..2
PAD_OUT = SKIRT_CLEAR + SKIRT_T          # pad reaches from the floor edge to the skirt's outer face
SKIRT_Z0 = PAD_H                         # skirt bottom edge sits on the pads
SKIRT_TOP_GAP = 0.4                      # skirt top stops this far below the roof so the roof always seats on the posts
SKIRT_PADS = [(35, "front"), (-35, "front"), (35, "rear"), (-35, "rear"),
              (40, "left"), (-50, "left"), (45, "right"), (-62, "right")]   # (position along the wall, wall)
LIP_T, LIP_H, LIP_CLEAR = 2.0, 5.0, 0.4  # roof lip wraps the skirt's top edge from outside
PI_PORT_CUTOUT = (-56, 6, 50, 76)        # rear wall, top level: x0, x1, z0, z1 (Ethernet + USB)
XT60_CUTOUT = (-23, 17, 6, 30)           # rear wall, bottom level: pack leads / charging
VENT_Y, VENT_Z = list(range(-20, 61, 7)), (50, 78)   # both side walls, top level (Pi cooler)
SWITCH_HOLE = (20.0, -40, 20)            # right wall: body diameter, y, z – the switch itself clamps to floor 1's bracket
WALL_TEXT, WALL_TEXT_SIZE, WALL_TEXT_POS = "SCOPE", 14, (10, 25)

# --- camera pod (Camera Module 3 Wide) in the skirt's front wall, recessed, 15° nose-down
CAM_TILT = 15
CAM_W, CAM_H, CAM_T = 34, 30, 3
CAM_HOLES = (21.0, 12.5)                 # official Camera Module 3 pattern (X x Z)
CAM_HOLE_D = M16_TAP                     # M1.6 x 4
CAM_BOARD_Z = 15                         # lens centre above the plate's bottom edge
CAM_LENS_D = 13
CAM_BOSS_H = 1.5
POD_Z0 = F2_Z + T + 1                    # pod sits just above floor 2 (nothing of floor 2 reaches y > 66 within |x| < 20)


# ============================================================== HELPERS
def outline(w, l, h, chamfer=CHAMFER, r=2):
    p = cq.Workplane("XY").box(w, l, h, centered=(True, True, False))
    p = p.edges("|Z").chamfer(chamfer)
    return p.edges("|Z").fillet(r)

def clip_to_outline(part, w=FLOOR_W, l=FLOOR_L):
    """nothing may stick out past the floor outline (the skirt has 1 mm clearance to it)"""
    return part.intersect(outline(w, l, 400).translate((0, 0, -100)))

def engrave(part, txt, xy, size=FONT, z_top=T, angle=0):
    t = (cq.Workplane("XY").workplane(offset=z_top).center(*xy).transformed(rotate=(0, 0, angle))
         .text(txt, size, -ENGRAVE, halign="center", valign="center"))
    return part.cut(t)

def engrave_outline(part, center, w, l, z_top=T, line=0.8):
    o = cq.Workplane("XY").workplane(offset=z_top - ENGRAVE).center(*center).rect(w, l).extrude(ENGRAVE)
    i = cq.Workplane("XY").workplane(offset=z_top - ENGRAVE).center(*center).rect(w - 2 * line, l - 2 * line).extrude(ENGRAVE)
    return part.cut(o.cut(i))

def pocket_pillars(part, center, w, l, h, z_base, lip=1.5, pillar=6, slot_axis="y", slot_offset=None):
    """PCB pocket: 4 corner pillars + outer lips (pocket = exactly w x l), 2 zip-tie slots. Hole-pattern agnostic."""
    cx, cy = center
    for sx in (-1, 1):
        for sy in (-1, 1):
            px, py = cx + sx * (w / 2 - pillar / 2), cy + sy * (l / 2 - pillar / 2)
            part = part.union(cq.Workplane("XY").workplane(offset=z_base).center(px, py).rect(pillar, pillar).extrude(h))
            part = part.union(cq.Workplane("XY").workplane(offset=z_base).center(cx + sx * (w / 2 + lip / 2), py).rect(lip, pillar).extrude(h + 3))
            part = part.union(cq.Workplane("XY").workplane(offset=z_base).center(px, cy + sy * (l / 2 + lip / 2)).rect(pillar + 2 * lip, lip).extrude(h + 3))
    off = (lip + 2.5) if slot_offset is None else slot_offset
    for sgn in (-1, 1):
        if slot_axis == "x":
            s = cq.Workplane("XY").workplane(offset=z_base - T).center(cx + sgn * (w / 2 + off), cy).rect(1.8, 4.0).extrude(T)
        else:
            s = cq.Workplane("XY").workplane(offset=z_base - T).center(cx, cy + sgn * (l / 2 + off)).rect(4.0, 1.8).extrude(T)
        part = part.cut(s)
    return part

def posts(part, xy_list, z0, height, foot=True, foot_size=None):
    """posts with a flared collar at the base and an M3 pilot in the top.
    (x, y)        -> square POST x POST, collar foot_size
    (x, y, axis)  -> rectangular POST_THIN (along axis) x POST_WIDE, collar +1 mm each side"""
    for p_ in xy_list:
        x, y = p_[0], p_[1]
        if len(p_) == 3:
            w, l = (POST_THIN, POST_WIDE) if p_[2] == "x" else (POST_WIDE, POST_THIN)
            fw, fl, ch = w + 2, l + 2, 0.99
        else:
            w = l = POST
            fw = fl = foot_size or POST_FOOT
            ch = (fw - POST) / 2 - 0.01
        p = cq.Workplane("XY").workplane(offset=z0).center(x, y).rect(w, l).extrude(height)
        if foot:
            f = (cq.Workplane("XY").workplane(offset=z0).center(x, y).rect(fw, fl).extrude(FOOT_H)
                 .faces(">Z").edges().chamfer(ch))
            p = p.union(f)
        hole = cq.Workplane("XY").workplane(offset=z0 + height - POST_PILOT_DEPTH).center(x, y).circle(M3_TAP / 2).extrude(POST_PILOT_DEPTH)
        part = part.union(p).cut(hole)
    return part

def xy(lst):
    return [(p[0], p[1]) for p in lst]

def m3_counterbored(part, xy_list, z_top):
    """M3 clearance through the plate with a counterbore from the top face at z_top"""
    for (x, y) in xy(xy_list):
        part = part.cut(cq.Workplane("XY").workplane(offset=z_top - T - 0.5).center(x, y).circle(M3_CLEAR / 2).extrude(T + 1))
        part = part.cut(cq.Workplane("XY").workplane(offset=z_top - CB_H).center(x, y).circle(CB_D / 2).extrude(CB_H + 0.5))
    return part

def switch_bracket(part):
    """floor-1 bracket for the Ø20 switch: face plate flush with the floor outline (the skirt's inner wall is 1 mm
    outside it), two gussets to the floor, hole on the skirt's SWITCH_HOLE axis"""
    d, sy, sz = SWITCH_HOLE
    w, t, g = SWITCH_BRACKET_W, SWITCH_BRACKET_T, SWITCH_GUSSET
    x_out = FLOOR_W / 2
    z_top = sz + d / 2 + 4
    b = cq.Workplane("XY").workplane(offset=T).center(x_out - t / 2, sy).rect(t, w).extrude(z_top - T)
    for sgn in (-1, 1):
        b = b.union(cq.Workplane("XY").workplane(offset=T).center(x_out - t - g / 2, sy + sgn * (w / 2 - t / 2)).rect(g, t).extrude(z_top - T))
    hole = cq.Workplane("YZ").workplane(offset=x_out - t - g - 1).center(sy, sz).circle(SWITCH_BRACKET_HOLE / 2).extrude(t + g + 2)
    return part.union(b).cut(hole)

def through_holes(part, xy_list, d, z0, t):
    for (x, y) in xy_list:
        part = part.cut(cq.Workplane("XY").workplane(offset=z0).center(x, y).circle(d / 2).extrude(t))
    return part

def slot(part, center, w, l, z0, t=T, r=None):
    s = cq.Workplane("XY").workplane(offset=z0).center(*center).rect(w, l).extrude(t)
    if r:
        s = s.edges("|Z").fillet(r)
    return part.cut(s)


# ============================================================== FLOOR 1 – BATTERY + UBEC (on the frame)
f1 = outline(FLOOR_W, FLOOR_L, T)
f1 = through_holes(f1, FRAME_HOLES, M25_CLEAR, 0, T)

# battery bay: two rails + front end-stop
bx, by = BAT_CENTER
for sx in (-1, 1):
    f1 = f1.union(cq.Workplane("XY").workplane(offset=T).center(bx + sx * (BAT_W / 2 + 1.5), by).rect(3, BAT_L).extrude(BAT_RAIL_H))
f1 = f1.union(cq.Workplane("XY").workplane(offset=T).center(bx, by + BAT_L / 2 + 1.5).rect(BAT_W + 6, 3).extrude(BAT_STOP_H))
f1 = engrave(f1, "BATTERY  2S LiPo", (bx, 0), angle=90)
f1 = engrave(f1, "XT60 > REAR", (bx, -52), angle=90, size=4)

# servo-lead / strap slots: left of the pack (over the frame opening) and right of the pack fore/aft of the UBEC
f1 = slot(f1, *LEFT_SLOT, 0, r=3)
f1 = slot(f1, *RIGHT_FRONT_SLOT, 0, r=3)
f1 = slot(f1, *RIGHT_REAR_SLOT, 0, r=3)

# UBEC pocket
f1 = pocket_pillars(f1, UBEC_CENTER, UBEC_W, UBEC_L, UBEC_PILLAR_H, T, slot_axis="y")
f1 = engrave(f1, "UBEC 5V", UBEC_CENTER, size=5)
f1 = switch_bracket(f1)
f1 = engrave(f1, "SWITCH", (54, -14), size=3.5, angle=90)

# posts up to floor 2
f1 = posts(f1, POSTS_1, T, F2_Z - T)
f1 = engrave(f1, "FRONT", (0, 74), size=4)
f1 = clip_to_outline(f1)
for (a, wall) in SKIRT_PADS:                          # landing pads for the skirt, outside the floor outline
    if wall in ("front", "rear"):
        y = (FLOOR_L / 2 + PAD_OUT / 2) * (1 if wall == "front" else -1)
        f1 = f1.union(cq.Workplane("XY").center(a, y).rect(PAD_L, PAD_OUT + 0.2).extrude(PAD_H))
    else:
        x = (FLOOR_W / 2 + PAD_OUT / 2) * (1 if wall == "right" else -1)
        f1 = f1.union(cq.Workplane("XY").center(x, a).rect(PAD_OUT + 0.2, PAD_L).extrude(PAD_H))

# ============================================================== FLOOR 2 – PI 5 + SERVO2040 + RELAY (top floor)
f2 = outline(FLOOR_W, FLOOR_L, T).translate((0, 0, F2_Z))
f2 = m3_counterbored(f2, POSTS_1, F2_Z + T)

# Pi 5 – port edge at PI_CENTER.y - PI_L/2 (rear); holes 23.5 and 81.5 mm from that edge, 49 apart across
px, py = PI_CENTER
port_edge_y = py - PI_L / 2
pi_holes = [(px + sx * PI_HOLE_DX / 2, port_edge_y + d) for sx in (-1, 1) for d in PI_HOLES_FROM_PORT_EDGE]
for (x, y) in pi_holes:
    f2 = f2.union(cq.Workplane("XY").workplane(offset=F2_Z + T).center(x, y).circle(PI_STANDOFF_D / 2).extrude(PI_STANDOFF_H))
f2 = through_holes(f2, pi_holes, M25_TAP, F2_Z, T + PI_STANDOFF_H)
f2 = engrave_outline(f2, PI_CENTER, PI_W, PI_L, z_top=F2_Z + T)
f2 = engrave(f2, "RASPBERRY PI 5", (px, py + 14), z_top=F2_Z + T)
f2 = engrave(f2, "PORTS > REAR", (px, py + 4), z_top=F2_Z + T, size=4)
f2 = engrave(f2, "GPIO < LEFT", (px, py - 4), z_top=F2_Z + T, size=4)

# Servo2040 pocket (unchanged size) – rear right
f2 = pocket_pillars(f2, S2040_CENTER, S2040_W, S2040_L, S2040_PILLAR_H, F2_Z + T, slot_axis="y")
f2 = engrave(f2, "USB-C > REAR", (S2040_CENTER[0], S2040_CENTER[1] - 13), z_top=F2_Z + T, size=3.2)
f2 = engrave(f2, "SERVO LEADS", (S2040_CENTER[0], S2040_CENTER[1] - 42), z_top=F2_Z + T, size=3.5)
f2 = engrave(f2, "SERVO 2040", (S2040_CENTER[0], S2040_CENTER[1] + 3), z_top=F2_Z + T, size=5)
f2 = engrave(f2, "P00 = SERVO 1", (S2040_CENTER[0], S2040_CENTER[1] - 5), z_top=F2_Z + T, size=3.2)

# relay pocket 65 x 40 (zip slots hugging the board edge – the plate edge is close behind it)
f2 = pocket_pillars(f2, RELAY_CENTER, RELAY_W, RELAY_L, RELAY_PILLAR_H, F2_Z + T, slot_axis="y", slot_offset=0.5)
f2 = engrave(f2, "RELAY", RELAY_CENTER, z_top=F2_Z + T, size=5)

# cable slots: servo leads beside/behind the Servo2040 (plugs are 7.6 x 2.5, they pass edge-on), UBEC 5 V up to the GPIO
f2 = slot(f2, *SERVO_SLOT, F2_Z, r=3)
f2 = slot(f2, *SERVO_SLOT_REAR, F2_Z, r=2)
f2 = slot(f2, *SENSOR_SLOT, F2_Z, r=2)
f2 = slot(f2, *RELAY_SLOT, F2_Z, r=2)
f2 = slot(f2, *PI_REAR_SLOT, F2_Z, r=3)
f2 = slot(f2, *GPIO_SLOT, F2_Z, r=2)
f2 = slot(f2, *FRONT_NOTCH, F2_Z, r=3)

# GY-521 IMU on two bosses (M2.5 x 5 through its Ø3 holes)
ix, iy = IMU_CENTER
imu_holes = [(ix + sx * IMU_HOLE_PITCH / 2, iy + IMU_L / 2 - IMU_HOLE_FROM_EDGE) for sx in (-1, 1)]
for (x, y) in imu_holes:
    f2 = f2.union(cq.Workplane("XY").workplane(offset=F2_Z + T).center(x, y).circle(IMU_BOSS_D / 2).extrude(IMU_BOSS_H))
f2 = through_holes(f2, imu_holes, M25_TAP, F2_Z, T + IMU_BOSS_H)
f2 = engrave_outline(f2, IMU_CENTER, IMU_W + 2, IMU_L + 2, z_top=F2_Z + T)
f2 = engrave(f2, "IMU", (ix, iy - 3), z_top=F2_Z + T, size=4)

# Grove I2C hub pocket (lips + zip ties, like the relay)
f2 = pocket_pillars(f2, HUB_CENTER, HUB_W, HUB_L, HUB_PILLAR_H, F2_Z + T, pillar=5, slot_axis="y", slot_offset=2)
f2 = engrave(f2, "I2C HUB", HUB_CENTER, z_top=F2_Z + T, size=4, angle=90)
for y in ANCHOR_Y:                                   # zip-tie anchor pairs for the lead bundle
    for x in (61, 64):
        f2 = slot(f2, (x, y), 1.8, 6, F2_Z)

# posts up to the roof
f2 = posts(f2, POSTS_2, F2_Z + T, ROOF_Z - (F2_Z + T))
f2 = engrave(f2, "FRONT", (-60, 60), z_top=F2_Z + T, size=4, angle=90)
f2 = clip_to_outline(f2)

# ============================================================== SKIRT dimensions (needed by the roof)
in_w, in_l = FLOOR_W + 2 * SKIRT_CLEAR, FLOOR_L + 2 * SKIRT_CLEAR
out_w, out_l = in_w + 2 * SKIRT_T, in_l + 2 * SKIRT_T
SKIRT_CH_OUT = CHAMFER + SKIRT_T * 0.83                     # the skirt's outer corner chamfer (as printed)
lip_in_w, lip_in_l = out_w + 2 * LIP_CLEAR, out_l + 2 * LIP_CLEAR
ROOF_W, ROOF_L = lip_in_w + 2 * LIP_T, lip_in_l + 2 * LIP_T

# ============================================================== ROOF – touch screen from below
roof = outline(ROOF_W, ROOF_L, T, chamfer=SKIRT_CH_OUT + 0.83 * (LIP_CLEAR + LIP_T)).translate((0, 0, ROOF_Z))
roof = m3_counterbored(roof, xy(POSTS_2), ROOF_Z + T)
# lip: hugs the skirt's top edge from outside (LIP_CLEAR all round) -> locates the skirt, no screws into it
lip = outline(ROOF_W, ROOF_L, LIP_H, chamfer=SKIRT_CH_OUT + 0.83 * (LIP_CLEAR + LIP_T)).translate((0, 0, ROOF_Z - LIP_H))
lip = lip.cut(outline(lip_in_w, lip_in_l, LIP_H + 2, chamfer=SKIRT_CH_OUT + 0.83 * LIP_CLEAR).translate((0, 0, ROOF_Z - LIP_H - 1)))
roof = roof.union(lip)
sx_, sy_ = SCREEN_CENTER
pw, pl = SCREEN_MODULE[0] + 2 * SCREEN_POCKET_CLEAR, SCREEN_MODULE[1] + 2 * SCREEN_POCKET_CLEAR
roof = slot(roof, (sx_ + SCREEN_WINDOW_DX, sy_ + SCREEN_WINDOW_DY), *SCREEN_WINDOW, ROOF_Z, r=1.5)
lip = cq.Workplane("XY").workplane(offset=ROOF_Z - SCREEN_LIP_H).center(sx_, sy_).rect(pw + 2 * SCREEN_LIP_T, pl + 2 * SCREEN_LIP_T).extrude(SCREEN_LIP_H)
lip = lip.cut(cq.Workplane("XY").workplane(offset=ROOF_Z - SCREEN_LIP_H).center(sx_, sy_).rect(pw, pl).extrude(SCREEN_LIP_H))
roof = roof.union(lip)
boss_face_z = ROOF_Z - SCREEN_LIP_H - RET_GAP                         # 80.6: retainer plate top face
for (bx, by) in RET_BOSSES:
    boss = cq.Workplane("XY").workplane(offset=boss_face_z).center(bx, by).circle(BOSS_D / 2).extrude(ROOF_Z - boss_face_z)
    boss = boss.cut(cq.Workplane("XY").workplane(offset=boss_face_z).center(bx, by).circle(BOSS_HOLE / 2).extrude(6.0))
    roof = roof.union(boss)
for y in (-70, -59, -48, -37):                       # vents over the rear (battery side)
    roof = slot(roof, (0, y), 40, 6, ROOF_Z, r=2.5)
roof = engrave(roof, "SCOPE", (-50, 0), z_top=ROOF_Z + T, size=9, angle=90)
roof = engrave(roof, "TANK PRE-SURVEY HEXAPOD", (53, 0), z_top=ROOF_Z + T, size=3.5, angle=90)

# ============================================================== SCREEN RETAINER (separate print)
rw, ry0, ry1 = RET_OUTLINE
ret_top = boss_face_z
ret = (cq.Workplane("XY").workplane(offset=ret_top - RET_T).center(0, (ry0 + ry1) / 2).rect(rw, ry1 - ry0).extrude(RET_T)
       .edges("|Z").fillet(4))
# opening: wider than the PCB on both sides and open past the connector end, so standoffs, header, pogo pins,
# GH1.25 and FPC all hang through; only a 6 mm bar overlaps the bare far-end strip
e = RET_CONNECTOR_END
ox0, ox1 = -pw / 2 - 0.5, pw / 2 + 0.5
y_conn_edge = sy_ + e * pl / 2                      # PCB connector edge (in roof coords): -28.6 for e = -1
y_far_edge = sy_ - e * pl / 2                       # PCB far edge: +64.6
oy_conn = y_conn_edge + e * 1.0                     # opening runs 1 mm past the connector edge
oy_far = y_far_edge + e * RET_TOP_OVERLAP           # ... and stops 6 mm short of the far edge
oy0, oy1 = min(oy_conn, oy_far), max(oy_conn, oy_far)
ret = ret.cut(cq.Workplane("XY").workplane(offset=ret_top - RET_T - 1).center((ox0 + ox1) / 2, (oy0 + oy1) / 2)
              .rect(ox1 - ox0, oy1 - oy0).extrude(RET_T + 2).edges("|Z").fillet(3))
ret = through_holes(ret, RET_BOSSES, M25_CLEAR, ret_top - RET_T, RET_T)
pad_h = (ROOF_Z - SCREEN_STACK_T + CLAMP) - ret_top                   # pads reach CLAMP past the PCB back
half_w = SCREEN_MODULE[0] / 2
pads = []
for sx in (-1, 1):
    # far-end pads: on the bar over the bare strip, 0.5..5.5 mm from the far edge, 2.5..8.5 mm from the side edge
    pads.append(((sx * (half_w - 5.5), y_far_edge + e * 3.0), (RET_PAD_W, 5.0)))
    # side fingers: a plate-level tongue passes UNDER the lip (0.4 mm clearance) from the side strip inward, and the
    # raised pad sits only on the part of the tongue that is inside the pocket
    reach_in = half_w - RET_SIDE_PAD_REACH
    y_f = y_conn_edge - e * RET_SIDE_PAD_Y
    tongue = cq.Workplane("XY").workplane(offset=ret_top - RET_T).center(sx * (reach_in + ox1 + 1.0) / 2, y_f).rect((ox1 + 1.0) - reach_in, RET_PAD_W).extrude(RET_T)
    ret = ret.union(tongue)
    pad_out = pw / 2 - 1.0                            # 1 mm inside the lip's inner face
    pads.append(((sx * (reach_in + pad_out) / 2, y_f), (pad_out - reach_in, RET_PAD_W)))
for (pc, psz) in pads:
    ret = ret.union(cq.Workplane("XY").workplane(offset=ret_top).center(*pc).rect(*psz).extrude(pad_h))
ret = engrave(ret, "SCREEN RETAINER - CONNECTORS THIS END", (0, ry0 + 4.5), z_top=ret_top, size=3.0)

# ============================================================== SKIRT
sk_h = ROOF_Z - SKIRT_TOP_GAP - SKIRT_Z0
skirt = outline(out_w, out_l, sk_h, chamfer=SKIRT_CH_OUT).translate((0, 0, SKIRT_Z0))
skirt = skirt.cut(outline(in_w, in_l, sk_h + 2, chamfer=CHAMFER + SKIRT_CLEAR * 0.83).translate((0, 0, SKIRT_Z0 - 1)))

def wall_cut(part, a0, a1, z0, z1, wall):
    """rectangular opening: wall='rear' -> a0..a1 are x; wall='right' -> a0..a1 are y"""
    if wall == "rear":
        c = cq.Workplane("XY").workplane(offset=z0).center((a0 + a1) / 2, -out_l / 2 - 2).rect(a1 - a0, 24).extrude(z1 - z0).edges("|Y").fillet(2)
    else:
        c = cq.Workplane("XY").workplane(offset=z0).center(out_w / 2, (a0 + a1) / 2).rect(24, a1 - a0).extrude(z1 - z0).edges("|X").fillet(2)
    return part.cut(c)

skirt = wall_cut(skirt, *PI_PORT_CUTOUT, "rear")     # Pi Ethernet / USB, top level
skirt = wall_cut(skirt, *XT60_CUTOUT, "rear")        # pack leads, bottom level
for sx in (-1, 1):                                   # vents, both side walls, top level
    for y in VENT_Y:
        skirt = skirt.cut(cq.Workplane("XY").workplane(offset=VENT_Z[0]).center(sx * out_w / 2, y).rect(2 * SKIRT_T + 4, 3).extrude(VENT_Z[1] - VENT_Z[0]).edges("|X").fillet(1.2))
d, sy, sz = SWITCH_HOLE                              # right wall: clearance for the switch bezel (switch clamps to floor 1)
skirt = skirt.cut(cq.Workplane("YZ").workplane(offset=out_w / 2 - SKIRT_T - 2).center(sy, sz).circle((SWITCH_BEZEL_D + 0.5) / 2).extrude(SKIRT_T + 4))

# camera pod
pod_w, pod_d, pod_h = CAM_W + 6, 11, CAM_H + 1
y_out = out_l / 2
pod_y0 = y_out - pod_d
skirt = skirt.cut(cq.Workplane("XY").workplane(offset=POD_Z0).center(0, y_out - SKIRT_T / 2).rect(pod_w, SKIRT_T + 2).extrude(pod_h))
pod = cq.Workplane("XY").box(pod_w, pod_d, pod_h, centered=(True, False, False)).translate((0, pod_y0, POD_Z0))
tilt = math.radians(CAM_TILT)
base_y = y_out - CAM_H * math.sin(tilt) - CAM_T / 2 * math.cos(tilt)
base_z = POD_Z0 + 2
def tilted_box(w, y0, y1, z0, z1):
    b = cq.Workplane("XY").box(w, y1 - y0, z1 - z0, centered=(True, False, False)).translate((0, y0, z0))
    return b.rotate((0, 0, 0), (1, 0, 0), -CAM_TILT).translate((0, base_y, base_z))
pod = pod.cut(tilted_box(CAM_W + 2, CAM_T / 2, 60, 3, CAM_H + 20))        # air in front of the plate (3 mm floor stays)
pod = pod.cut(tilted_box(CAM_W + 2, -60, -CAM_T / 2, 0, CAM_H + 20))      # cavity behind the plate (camera board)
pod = pod.cut(tilted_box(CAM_LENS_D, -CAM_T, CAM_T, CAM_BOARD_Z - CAM_LENS_D / 2, CAM_BOARD_Z + CAM_LENS_D / 2))
for hx in (-CAM_HOLES[0] / 2, CAM_HOLES[0] / 2):
    for hz in (CAM_BOARD_Z - CAM_HOLES[1] / 2, CAM_BOARD_Z + CAM_HOLES[1] / 2):
        boss = (cq.Workplane("XY").circle(2.2).extrude(CAM_BOSS_H).rotate((0, 0, 0), (1, 0, 0), 90)
                .translate((hx, -CAM_T / 2, hz)).rotate((0, 0, 0), (1, 0, 0), -CAM_TILT).translate((0, base_y, base_z)))
        hole = (cq.Workplane("XY").circle(CAM_HOLE_D / 2).extrude(CAM_T + CAM_BOSS_H + 2).rotate((0, 0, 0), (1, 0, 0), 90)
                .translate((hx, CAM_T / 2 + 1, hz)).rotate((0, 0, 0), (1, 0, 0), -CAM_TILT).translate((0, base_y, base_z)))
        pod = pod.union(boss).cut(hole)
skirt = skirt.union(pod)

for sx in (-1, 1):                                   # SCOPE on both side walls, readable from outside
    wp = cq.Workplane("YZ" if sx > 0 else "ZY").workplane(offset=out_w / 2)
    ty, tz = WALL_TEXT_POS
    txt = wp.center(ty, tz) if sx > 0 else wp.center(tz, ty)
    txt = txt.transformed(rotate=(0, 0, 0 if sx > 0 else -90)).text(WALL_TEXT, WALL_TEXT_SIZE, -ENGRAVE, halign="center", valign="center")
    skirt = skirt.cut(txt)

# ============================================================== EXPORT
out = "/mnt/user-data/outputs/"
kw = dict(tolerance=0.05, angularTolerance=0.1)
cq.exporters.export(f1, out + "floor1_battery_ubec.stl", **kw)
cq.exporters.export(f2.translate((0, 0, -F2_Z)), out + "floor2_pi_servo2040_relay.stl", **kw)
cq.exporters.export(roof.translate((0, 0, -ROOF_Z)), out + "roof_screen.stl", **kw)
cq.exporters.export(ret.translate((0, 0, -(ret_top - RET_T))), out + "screen_retainer.stl", **kw)
cq.exporters.export(skirt.translate((0, 0, -SKIRT_Z0)), out + "skirt_walls.stl", **kw)
cq.exporters.export(f1.union(f2).union(roof).union(skirt), out + "assembly_preview.stl", **kw)
print(f"done. floors {FLOOR_W}x{FLOOR_L}, skirt {out_w:.1f}x{out_l:.1f}x{sk_h:.1f} on pads at z={SKIRT_Z0}, roof {ROOF_W:.1f}x{ROOF_L:.1f} with {LIP_H} mm lip, top at z={ROOF_Z + T}")
