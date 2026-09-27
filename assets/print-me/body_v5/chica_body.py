"""
"Scope" – electronics body for the MYP Chica hexapod frame.   v3 (cable clamps, floor-1 brackets, snap-on skirt)

Coordinate system = MYP frame: origin at the frame centre, z = 0 is the top of the frame, +Y = front, +X = right.
Parts (all print flat as exported):
  floor1_battery_ubec.stl        on the frame: battery bay with 25 mm guide walls (pack slides in through the rear door),
                                 UBEC + relay on vertical brackets (right wall), switch bracket (left wall), 6 cable clamps
  floor2_pi_servo2040.stl        Pi 5 (ports rear, GPIO left), Servo2040 across the rear, IMU, 6 cable clamps
  roof_screen.stl / screen_retainer.stl   unchanged
  skirt_walls.stl                snaps onto floor 1 (4 tabs into post grooves), rests on 8 pads, captured by the roof lip.
                                 Rear battery door, left-wall switch U-slot + J2 power button, camera pod, vents.
  camera_carrier.stl             flat plate that holds Camera Module 3 and bolts into the pod (4 x M2.5) – prints flat,
                                 so its holes are accurate; the pod only needs a window and four pilots
Cable path per leg: 3 servo leads + 1 sensor lead come up through a floor-1 clamp, straight up (right side) or 25 mm
outboard (left side, beside the Pi) through the floor-2 clamp, then to the Servo2040's header row at its front edge.
Teardown: 4 roof screws (+4 lip screws to leave the skirt standing) -> roof off -> floor 2 lifts straight out.
Fasteners: M3 x 6 (12), M2.5 x 5 (14), M1.6 x 4 (4). Everything threads into PLA.
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

# --- fasteners (thread-forming pilots for PLA; drill out with 1.45 / 2.1 / 2.6 if a screw won't start)
M16_TAP = 1.45                           # M1.6 pilot
M25_TAP = 2.1                            # M2.5 pilot
M25_CLEAR = 2.7                          # M2.5 clearance
M3_TAP = 2.6                             # M3 pilot
M3_CLEAR = 3.3                           # M3 clearance
CB_D, CB_H = 6.5, 1.2                    # counterbore for the M3 pan head
POST_PILOT_DEPTH = 8
POST = 12                                # floor-1 posts: square
POST_FOOT = 14                           # flared collar at the post base
FOOT_H = 3
POST_THIN, POST_WIDE = 7, 16             # floor-2 posts: rectangular, thin side toward the boards

# --- frame attachment: pilot holes on the frame's ribs, x = ±44, y = -40 … 40
FRAME_HOLES = [(sx * 44, y) for sx in (-1, 1) for y in range(-40, 41, 10)]

# --- legs / cable slots + clamps. One BIG slot per side on both floors, three C-ring clamps straddling each slot.
# Floor-1 slots must be over the frame's central opening (±39 wide, rounds off past y ±45) and outside the battery
# walls (±27): x 28..38.5, y ±42. Floor 2's right slot sits exactly above; the left one is beside the Pi (x -63..-55).
LEG_Y = (31, 0, -26)                     # front / middle / rear leg bundles = clamp centres, same on both floors
SLOT_X1 = (28, 38.5)                     # floor 1, mirrored for the left side
SLOT_Y1 = (-42, 42)
SLOT_X2 = {"left": (-63, -55), "right": (28, 38.5)}
SLOT_Y2 = {"left": (-42, 39), "right": (-34, 42)}      # right stops short of the Servo2040, left short of the roof post
CLAMP_L = 12                             # ring inside length along the slot (holds 3 servo leads + 1 sensor lead)
CLAMP_WALL, CLAMP_H, CLAMP_GAP = 1.2, 8, 6   # ring wall, height, opening in the bridge that faces the open slot
CLIP_OPEN = ("-y", "+y", "+y")           # per leg: which bridge is open (front clip opens aft, the others open forward)
LEG_LABELS = {"left": ("L1", "L2", "L3"), "right": ("R1", "R2", "R3")}

# --- floor 1 (bottom): battery + UBEC + relay + switch
BAT_W, BAT_L, BAT_H = 48, 140, 27        # HOOVO 2S pack
BAT_CENTER = (0, -10)                    # bay centred, pushed aft: the pack goes in and out through the skirt's rear door
BAT_WALL_H, BAT_STOP_H = 25, 12          # guide walls both sides (keep the pack off the wiring), front end-stop
BAT_STRAP = (-20, 20, 3, 15)             # strap slot through both walls: y, width, height, z-centre
UBEC_L, UBEC_H = 45, 35                  # Nuofany UBEC stands on its 12 mm edge against the right wall: 45 along Y, 35 tall
RELAY_L, RELAY_H = 65, 40                # relay module stands against the right wall too: 65 along Y, 40 tall, ~18 deep
RIGHT_WALL = {"relay": (-3, 69), "ubec": (-55, -4)}     # y spans of the two right-wall brackets (module + 3 mm gussets + 1 mm)
BRACKET_T, BRACKET_G = 3, 4              # bracket plate thickness, gusset depth
SWITCH_Y, SWITCH_Z = -53, 20             # switch on the LEFT wall now, aft of the rear-leg bundle (bracket y -70..-36)
SWITCH_BRACKET_W = 34                    # nut Ø26 must clear both gussets
SWITCH_BRACKET_HOLE = 20.2               # the switch's panel hole (Ø20 body)
SWITCH_BEZEL_D = 24                      # MEASURE: outer Ø of the switch bezel -> skirt U-slot width = bezel + 0.5
POSTS_1 = [(48, 74), (-59, 63), (60, -63), (-35, -74)]  # floor1 -> floor2; one per wall, each flush with it for a snap tab
# floor-1 posts are flush with the plate edge on one face; the skirt's snap tabs click into a groove there
TAB_GROOVE = (1.5, 4.5, 9.5)             # depth, z0, z1

# --- floor 2 (top): Pi 5 + Servo2040 + IMU
PI_W, PI_L = 56, 85
PI_CENTER = (-25, 24.5)                  # long axis along Y, USB/Ethernet to the rear (-Y), GPIO along the left edge; y -18..67
PI_HOLE_DX = 49
PI_HOLES_FROM_PORT_EDGE = (23.5, 81.5)
PI_STANDOFF_H, PI_STANDOFF_D = 6, 7
PI_BUTTON = (-33, 57)                    # onboard power button: 21 mm from the GPIO edge along the front (SD) edge (MEASURE), z
PI_BUTTON_SLOT = (14, 6)                 # access slot in the front wall (length x height): covers ±7 mm on x
S2040_W, S2040_L = 62, 42                # ROTATED: long axis along X, across the rear. Header row on its FRONT edge.
S2040_CENTER = (24, -56)                 # x -7..55, y -77..-35: USB-C + screw terminal at its LEFT end (x -7), plugs from above
S2040_PILLAR_H = 5
POSTS_2 = [(30, 76.5, "y"), (-50, 76.5, "y"), (61, -12, "x"), (-61, 49, "x")]   # floor2 -> roof; front-right unchanged from v2 (roof hole reused)
PI_REAR_SLOT = ((-25, -14.5), 50, 8)     # along the Pi's port edge: UBEC 5 V + relay control up to the GPIO
FRONT_NOTCH = ((0, 75.5), 40, 13)        # clears the camera pod (38 wide, inside the skirt's front wall) when floor 2 lifts straight out
IMU_W, IMU_L = 20, 16
IMU_CENTER = (20, 55)
IMU_HOLE_PITCH, IMU_HOLE_FROM_EDGE = 15.24, 2.5
IMU_BOSS_D, IMU_BOSS_H = 6.0, 3.5

# --- roof + screen (unchanged)
SCREEN_MODULE = (61.0, 92.44)
SCREEN_POCKET_CLEAR = 0.4
SCREEN_WINDOW = (50.4, 74.8)
SCREEN_WINDOW_DX, SCREEN_WINDOW_DY = 0.0, 0.0
SCREEN_CENTER = (0, 18)
SCREEN_LIP_H, SCREEN_LIP_T = 6.0, 1.5
SCREEN_STACK_T = 4.5
RET_T = 2.5
RET_GAP = 0.4
CLAMP = 0.4
BOSS_D, BOSS_HOLE = 7.0, M25_TAP
RET_BOSSES = [(38, 60), (-38, 60), (36.5, -34.5), (-36.5, -34.5)]
RET_OUTLINE = (83, -38, 66)
RET_CONNECTOR_END = -1
SCREEN_HOLES_FROM_SIDE, SCREEN_HOLES_FROM_END = 5.5, (5.0, 65.0)
SCREEN_STANDOFF_D, SCREEN_STANDOFF_H = 6.0, 12.0
SCREEN_HEADER = (5.0, 15.0, 7.0, 25.0)
SCREEN_CLEAR_TOP = 20.0
RET_TOP_OVERLAP = 6.0
RET_PAD_W = 6.0
RET_SIDE_PAD_Y = 48.0
RET_SIDE_PAD_REACH = 7.0

# --- skirt: nothing protrudes inside it, so floor 2 (130 x 160) passes through it in either direction
SKIRT_CLEAR, SKIRT_T = 1.0, 2.4
PAD_H, PAD_L = 2.0, 12
PAD_OUT = SKIRT_CLEAR + SKIRT_T
SKIRT_Z0 = PAD_H
SKIRT_TOP_GAP = 0.4
SKIRT_PADS = [(35, "front"), (-35, "front"), (35, "rear"), (-48, "rear"),
              (20, "left"), (-32, "left"), (45, "right"), (-40, "right")]
# snap tabs: a cantilever cut into the wall's bottom edge with a ridge on its inner face that clicks into the post groove
SKIRT_TABS = [("front", 48), ("left", 63), ("right", -63), ("rear", -35)]   # (wall, position) – one per floor-1 post, one per wall
TAB_W, TAB_SLIT, TAB_H, NUB_P = 9.0, 1.2, 20.0, 1.8
BAND_T, BAND_Z0 = 3.6, 79.6
LIP_T, LIP_H, LIP_CLEAR = 2.0, 7.5, 0.4
LIP_SCREWS_Y = (45, -45)
LIP_SCREW_Z = 83.1
PI_PORT_CUTOUT = (-56, 6, 50, 76)        # rear wall, top level: x0, x1, z0, z1 (Ethernet + USB)
BATTERY_DOOR = (-26, 26, 34)             # rear wall, open to the bottom edge: x0, x1, z1 – the pack slides in and out
VENT_Y, VENT_Z = list(range(-20, 61, 7)), (50, 75)
J2_BUTTON = (46, 22, 12.5)               # left wall: y, z, hole Ø – 12 mm momentary pushbutton wired to the Pi 5's J2 pads
WALL_TEXT, WALL_TEXT_SIZE, WALL_TEXT_POS = "SCOPE", 14, (10, 40)

# --- camera pod (Camera Module 3 Wide) in the skirt's front wall, recessed, 15° nose-down; camera on a separate carrier
CAM_TILT = 15
CAM_W, CAM_H, CAM_T = 32, 30, 3          # pod plate = carrier outline
CAM_HOLES = (21.0, 12.5)                 # official Camera Module 3 pattern (holes Ø2.2 on the board)
CAM_LENS_DZ = -0.5                       # sensor housing centre sits 0.5 below the hole-pair centre (drawing: holes 2.0 / 14.5 from the top)
CAM_WINDOW = 15                          # carrier window (square): passes the 10.8 mm housing and the Wide lens
POD_WINDOW = 18                          # pod plate window, larger than the carrier's
CARRIER_HOLES = (26, 22)                 # carrier -> pod screws (M2.5 x 5), pattern X x Z
CAM_BOARD_Z = 15                         # hole-pair centre above the plate's bottom edge
CAM_BOSS_H = 1.5
POD_D = 13                               # pod depth: plate 3 + 10 mm cavity (carrier 3 + bosses 1.5 + board + FFC connector)
POD_Z0 = F2_Z + T + 1


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

def wall_bracket(part, wall, y0, y1, z_top, hole=None, zips=()):
    """floor-1 vertical plate flush with the floor outline on a side wall (the skirt's inner face is 1 mm outside),
    gussets at both ends. hole=(y, z, d): horizontal through-hole. zips=[(y, z), ...]: pairs of zip-tie slots
    through the plate with a shallow channel between them on the outer face (the tie sits flush)."""
    s = 1 if wall == "right" else -1
    t, g = BRACKET_T, BRACKET_G
    x_face = s * FLOOR_W / 2
    b = cq.Workplane("XY").workplane(offset=T).center(x_face - s * t / 2, (y0 + y1) / 2).rect(t, y1 - y0).extrude(z_top - T)
    for y in (y0 + t / 2, y1 - t / 2):
        b = b.union(cq.Workplane("XY").workplane(offset=T).center(x_face - s * (t + g / 2), y).rect(g, t).extrude(z_top - T))
    x0 = (x_face - t - g - 1) if s > 0 else (x_face - 1)
    if hole:
        y, z, d = hole
        b = b.cut(cq.Workplane("YZ").workplane(offset=x0).center(y, z).circle(d / 2).extrude(t + g + 2))
    for (y, z) in zips:
        for dy in (-4, 4):
            b = b.cut(cq.Workplane("YZ").workplane(offset=x0).center(y + dy, z).rect(1.8, 5).extrude(t + g + 2))
        b = b.cut(cq.Workplane("XY").workplane(offset=z - 2.5).center(x_face + s * 0.4, y).rect(3.2, 10).extrude(5))   # channel 1.2 deep on the outer face
    return part.union(b)

def clip(part, x0, x1, yc, z_base, opening):
    """cable clamp straddling a big slot (x0..x1): a C-ring whose two long walls stand on the slot's rims and whose two
    short walls bridge over the slot (1.2 mm bridges, printable). One bridge has a 6 mm opening so the bundle, once it is
    up through the open part of the slot, slides along the slot into the ring."""
    w, h = CLAMP_WALL, CLAMP_H
    cx = (x0 + x1) / 2
    ring = cq.Workplane("XY").workplane(offset=z_base).center(cx, yc).rect(x1 - x0 + 2 * w, CLAMP_L + 2 * w).extrude(h).edges("|Z").fillet(1.5)
    ring = ring.cut(cq.Workplane("XY").workplane(offset=z_base - 1).center(cx, yc).rect(x1 - x0, CLAMP_L).extrude(h + 2))
    s = 1 if opening == "+y" else -1
    ring = ring.cut(cq.Workplane("XY").workplane(offset=z_base - 1).center(cx, yc + s * (CLAMP_L / 2 + w / 2)).rect(CLAMP_GAP, w + 1).extrude(h + 2))
    return part.union(ring)

def tab_groove(part, wall, a):
    """groove across a floor-1 post's outer (edge-flush) face for the skirt's snap tab"""
    d, z0, z1 = TAB_GROOVE
    if wall in ("left", "right"):
        s = 1 if wall == "right" else -1
        g = cq.Workplane("XY").workplane(offset=z0).center(s * (FLOOR_W / 2 - d / 2 + 0.5), a).rect(d + 1, 12).extrude(z1 - z0)
    else:
        s = 1 if wall == "front" else -1
        g = cq.Workplane("XY").workplane(offset=z0).center(a, s * (FLOOR_L / 2 - d / 2 + 0.5)).rect(12, d + 1).extrude(z1 - z0)
    return part.cut(g)

def through_holes(part, xy_list, d, z0, t):
    for (x, y) in xy_list:
        part = part.cut(cq.Workplane("XY").workplane(offset=z0).center(x, y).circle(d / 2).extrude(t))
    return part

def slot(part, center, w, l, z0, t=T, r=None):
    s = cq.Workplane("XY").workplane(offset=z0).center(*center).rect(w, l).extrude(t)
    if r:
        s = s.edges("|Z").fillet(r)
    return part.cut(s)


# ============================================================== FLOOR 1 – BATTERY + UBEC + RELAY + SWITCH (on the frame)
f1 = outline(FLOOR_W, FLOOR_L, T)
f1 = through_holes(f1, FRAME_HOLES, M25_CLEAR, 0, T)

# battery bay: 25 mm guide walls both sides + front end-stop; the pack slides in from the rear through the skirt door
bx, by = BAT_CENTER
for sx in (-1, 1):
    wall = cq.Workplane("XY").workplane(offset=T).center(bx + sx * (BAT_W / 2 + 1.5), by).rect(3, BAT_L).extrude(BAT_WALL_H)
    sy_, sw_, sh_, sz_ = BAT_STRAP                                        # strap slot through the wall
    wall = wall.cut(cq.Workplane("XY").workplane(offset=sz_ - sh_ / 2).center(bx + sx * (BAT_W / 2 + 1.5), sy_).rect(5, sw_).extrude(sh_))
    f1 = f1.union(wall)
f1 = f1.union(cq.Workplane("XY").workplane(offset=T).center(bx, by + BAT_L / 2 + 1.5).rect(40, 3).extrude(BAT_STOP_H))
f1 = engrave(f1, "BATTERY  2S LiPo", (bx, by), angle=90)
f1 = engrave(f1, "XT60 > REAR", (bx, by - 50), angle=90, size=4)

# one big slot per side over the frame opening, three C-ring clamps straddling it (labels outboard, between the frame screws)
for side, sx in (("left", -1), ("right", 1)):
    x0, x1 = sorted((sx * SLOT_X1[0], sx * SLOT_X1[1]))
    f1 = slot(f1, ((x0 + x1) / 2, (SLOT_Y1[0] + SLOT_Y1[1]) / 2), x1 - x0, SLOT_Y1[1] - SLOT_Y1[0], 0, r=3)
    for i, y in enumerate(LEG_Y):
        f1 = clip(f1, x0, x1, y, T, CLIP_OPEN[i])
        f1 = engrave(f1, LEG_LABELS[side][i], (sx * 50, y), size=3.5)

# right wall: relay + UBEC standing on vertical brackets (zip-tied), outboard of the right-side bundles
ry0, ry1 = RIGHT_WALL["relay"]
f1 = wall_bracket(f1, "right", ry0, ry1, T + RELAY_H + 1, zips=[(ry0 + 14, 14), (ry0 + 14, 30), (ry1 - 14, 14), (ry1 - 14, 30)])
uy0, uy1 = RIGHT_WALL["ubec"]
f1 = wall_bracket(f1, "right", uy0, uy1, T + UBEC_H + 1, zips=[(uy0 + 11, 12), (uy0 + 11, 26), (uy1 - 11, 12), (uy1 - 11, 26)])
f1 = engrave(f1, "RELAY", (41.5, (ry0 + ry1) / 2), size=3, angle=90)
f1 = engrave(f1, "UBEC 5V", (41.5, (uy0 + uy1) / 2), size=3, angle=90)
# left wall: switch bracket
f1 = wall_bracket(f1, "left", SWITCH_Y - SWITCH_BRACKET_W / 2, SWITCH_Y + SWITCH_BRACKET_W / 2, SWITCH_Z + SWITCH_BRACKET_HOLE / 2 + 4,
                  hole=(SWITCH_Y, SWITCH_Z, SWITCH_BRACKET_HOLE))
f1 = engrave(f1, "SWITCH", (-42, SWITCH_Y), size=3, angle=90)

# posts up to floor 2, each flush with a wall; groove on that face for the skirt's snap tab
f1 = posts(f1, POSTS_1, T, F2_Z - T)
f1 = engrave(f1, "FRONT", (0, 72), size=4)
f1 = clip_to_outline(f1)
for (wall, a) in SKIRT_TABS:
    f1 = tab_groove(f1, wall, a)
for (a, wall) in SKIRT_PADS:                          # landing pads for the skirt, outside the floor outline
    if wall in ("front", "rear"):
        y = (FLOOR_L / 2 + PAD_OUT / 2) * (1 if wall == "front" else -1)
        f1 = f1.union(cq.Workplane("XY").center(a, y).rect(PAD_L, PAD_OUT + 0.2).extrude(PAD_H))
    else:
        x = (FLOOR_W / 2 + PAD_OUT / 2) * (1 if wall == "right" else -1)
        f1 = f1.union(cq.Workplane("XY").center(x, a).rect(PAD_OUT + 0.2, PAD_L).extrude(PAD_H))

# ============================================================== FLOOR 2 – PI 5 + SERVO2040 + IMU (top floor)
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

# Servo2040 across the rear, rotated: 62 along X. Header row on its front edge -> both sides' leads reach it.
f2 = pocket_pillars(f2, S2040_CENTER, S2040_W, S2040_L, S2040_PILLAR_H, F2_Z + T, slot_axis="x", slot_offset=2.5)
f2 = f2.cut(cq.Workplane("XY").workplane(offset=F2_Z + T + S2040_PILLAR_H + 1.5).center(S2040_CENTER[0] - S2040_W / 2 - 1, S2040_CENTER[1] + S2040_L / 2 - 0.5)
            .rect(9, 4).extrude(5))                            # no lip on the front-left corner: the Pi's USB plugs pass just above it
f2 = engrave(f2, "SERVO 2040", (S2040_CENTER[0], S2040_CENTER[1] + 4), z_top=F2_Z + T, size=5)
f2 = engrave(f2, "SERVOS > FRONT   USB-C < LEFT", (S2040_CENTER[0], S2040_CENTER[1] - 5), z_top=F2_Z + T, size=3.2)

# cable slots
f2 = slot(f2, *PI_REAR_SLOT, F2_Z, r=3)
f2 = slot(f2, *FRONT_NOTCH, F2_Z, r=3)
# big slots: right one straight above floor 1's, left one beside the Pi; three C-ring clamps each
for side in ("left", "right"):
    x0, x1 = SLOT_X2[side]
    y0, y1 = SLOT_Y2[side]
    f2 = slot(f2, ((x0 + x1) / 2, (y0 + y1) / 2), x1 - x0, y1 - y0, F2_Z, r=3)
    for i, y in enumerate(LEG_Y):
        f2 = clip(f2, x0, x1, y, F2_Z + T, CLIP_OPEN[i])
        if side == "right":
            f2 = engrave(f2, LEG_LABELS[side][i], (42.5, y), z_top=F2_Z + T, size=3.5)

# GY-521 IMU on two bosses (M2.5 x 5 through its Ø3 holes)
ix, iy = IMU_CENTER
imu_holes = [(ix + sx * IMU_HOLE_PITCH / 2, iy + IMU_L / 2 - IMU_HOLE_FROM_EDGE) for sx in (-1, 1)]
for (x, y) in imu_holes:
    f2 = f2.union(cq.Workplane("XY").workplane(offset=F2_Z + T).center(x, y).circle(IMU_BOSS_D / 2).extrude(IMU_BOSS_H))
f2 = through_holes(f2, imu_holes, M25_TAP, F2_Z, T + IMU_BOSS_H)
f2 = engrave_outline(f2, IMU_CENTER, IMU_W + 2, IMU_L + 2, z_top=F2_Z + T)
f2 = engrave(f2, "IMU", (ix, iy - 3), z_top=F2_Z + T, size=4)

# posts up to the roof
f2 = posts(f2, POSTS_2, F2_Z + T, ROOF_Z - (F2_Z + T))
f2 = engrave(f2, "FRONT", (12, 66), z_top=F2_Z + T, size=4)
f2 = clip_to_outline(f2)

# ============================================================== SKIRT dimensions (needed by the roof)
in_w, in_l = FLOOR_W + 2 * SKIRT_CLEAR, FLOOR_L + 2 * SKIRT_CLEAR
out_w, out_l = in_w + 2 * SKIRT_T, in_l + 2 * SKIRT_T
SKIRT_CH_OUT = CHAMFER + SKIRT_T * 0.83                     # the skirt's outer corner chamfer (as printed)
band_w, band_l = out_w + 2 * BAND_T, out_l + 2 * BAND_T     # thickened top band
BAND_CH = SKIRT_CH_OUT + 0.83 * BAND_T
lip_in_w, lip_in_l = band_w + 2 * LIP_CLEAR, band_l + 2 * LIP_CLEAR
ROOF_W, ROOF_L = lip_in_w + 2 * LIP_T, lip_in_l + 2 * LIP_T

# ============================================================== ROOF – touch screen from below
roof = outline(ROOF_W, ROOF_L, T, chamfer=BAND_CH + 0.83 * (LIP_CLEAR + LIP_T)).translate((0, 0, ROOF_Z))
roof = m3_counterbored(roof, xy(POSTS_2), ROOF_Z + T)
# lip: hugs the skirt's thickened top band from outside (LIP_CLEAR all round); 4 horizontal M3 through it into the band
lip = outline(ROOF_W, ROOF_L, LIP_H, chamfer=BAND_CH + 0.83 * (LIP_CLEAR + LIP_T)).translate((0, 0, ROOF_Z - LIP_H))
lip = lip.cut(outline(lip_in_w, lip_in_l, LIP_H + 2, chamfer=BAND_CH + 0.83 * LIP_CLEAR).translate((0, 0, ROOF_Z - LIP_H - 1)))
roof = roof.union(lip)
for sx in (-1, 1):
    for y in LIP_SCREWS_Y:
        x0 = (ROOF_W / 2 - LIP_T - 1) if sx > 0 else -(ROOF_W / 2 + 1)
        roof = roof.cut(cq.Workplane("YZ").workplane(offset=x0).center(y, LIP_SCREW_Z).circle(M3_CLEAR / 2).extrude(LIP_T + 2))
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
sk_top = SKIRT_Z0 + sk_h
skirt = outline(out_w, out_l, sk_h, chamfer=SKIRT_CH_OUT).translate((0, 0, SKIRT_Z0))

def octagon(w, l, c):
    return [(-w / 2 + c, -l / 2), (w / 2 - c, -l / 2), (w / 2, -l / 2 + c), (w / 2, l / 2 - c),
            (w / 2 - c, l / 2), (-w / 2 + c, l / 2), (-w / 2, l / 2 - c), (-w / 2, -l / 2 + c)]

# outward band at the top: filleted ring from BAND_Z0 up, 45° lofted transition below it (self-supporting when printed)
band = outline(band_w, band_l, sk_top - BAND_Z0, chamfer=BAND_CH).translate((0, 0, BAND_Z0))
trans = (cq.Workplane("XY").workplane(offset=BAND_Z0 - BAND_T).polyline(octagon(out_w - 0.2, out_l - 0.2, SKIRT_CH_OUT)).close()
         .workplane(offset=BAND_T).polyline(octagon(band_w - 0.4, band_l - 0.4, BAND_CH)).close().loft(combine=True))
skirt = skirt.union(band).union(trans)
skirt = skirt.cut(outline(in_w, in_l, sk_h + 2, chamfer=CHAMFER + SKIRT_CLEAR * 0.83).translate((0, 0, SKIRT_Z0 - 1)))
for sx in (-1, 1):                                   # M3 pilots through the band, coaxial with the roof lip holes
    for y in LIP_SCREWS_Y:
        x0 = (in_w / 2 - 1) if sx > 0 else -(band_w / 2 + 1)
        skirt = skirt.cut(cq.Workplane("YZ").workplane(offset=x0).center(y, LIP_SCREW_Z).circle(M3_TAP / 2).extrude(band_w / 2 - in_w / 2 + 2))

def wall_cut(part, a0, a1, z0, z1, wall):
    """rectangular opening: wall='rear' -> a0..a1 are x; wall='right'/'left' -> a0..a1 are y"""
    if wall == "rear":
        c = cq.Workplane("XY").workplane(offset=z0).center((a0 + a1) / 2, -out_l / 2 - 2).rect(a1 - a0, 24).extrude(z1 - z0).edges("|Y").fillet(2)
    else:
        s = 1 if wall == "right" else -1
        c = cq.Workplane("XY").workplane(offset=z0).center(s * out_w / 2, (a0 + a1) / 2).rect(24, a1 - a0).extrude(z1 - z0).edges("|X").fillet(2)
    return part.cut(c)

skirt = wall_cut(skirt, *PI_PORT_CUTOUT, "rear")                         # Pi Ethernet / USB, top level
skirt = wall_cut(skirt, BATTERY_DOOR[0], BATTERY_DOOR[1], SKIRT_Z0 - 1, BATTERY_DOOR[2], "rear")   # battery door, open at the bottom
for sx in (-1, 1):                                   # vents, both side walls, top level
    for y in VENT_Y:
        skirt = skirt.cut(cq.Workplane("XY").workplane(offset=VENT_Z[0]).center(sx * out_w / 2, y).rect(2 * SKIRT_T + 4, 3).extrude(VENT_Z[1] - VENT_Z[0]).edges("|X").fillet(1.2))
# left wall: U-slot for the switch bezel, open to the bottom edge so the skirt slides down over the switch
sw_w = SWITCH_BEZEL_D + 0.5
uslot = (cq.Workplane("YZ").workplane(offset=-out_w / 2 - 2).center(SWITCH_Y, SWITCH_Z).circle(sw_w / 2).extrude(SKIRT_T + 4)
         .union(cq.Workplane("XY").workplane(offset=SKIRT_Z0 - 1).center(-out_w / 2, SWITCH_Y).rect(SKIRT_T + 4, sw_w).extrude(SWITCH_Z - SKIRT_Z0 + 1)))
skirt = skirt.cut(uslot)
jy, jz, jd = J2_BUTTON                               # left wall: 12 mm momentary pushbutton -> Pi 5 J2 pads
skirt = skirt.cut(cq.Workplane("YZ").workplane(offset=-out_w / 2 - 2).center(jy, jz).circle(jd / 2).extrude(SKIRT_T + 4))
bxp, bzp = PI_BUTTON                                 # front wall: slot in line with the Pi's onboard button (a toothpick reaches it)
skirt = skirt.cut(cq.Workplane("XZ").workplane(offset=-out_l / 2 - 2).center(bxp, bzp).slot2D(*PI_BUTTON_SLOT).extrude(SKIRT_T + 4))   # XZ normal is -Y: +extrude goes inward

# snap tabs: two slits from the bottom edge free a 9 mm wide cantilever; a 45°/45° ridge on its inner face clicks
# into the groove on the matching floor-1 post (deflection 0.8 mm past floor 2's edge and the post face)
for (wall, a) in SKIRT_TABS:
    if wall in ("left", "right"):
        s = 1 if wall == "right" else -1
        for da in (-(TAB_W + TAB_SLIT) / 2, (TAB_W + TAB_SLIT) / 2):
            skirt = skirt.cut(cq.Workplane("XY").workplane(offset=SKIRT_Z0 - 1).center(s * (in_w / 2 + SKIRT_T / 2), a + da).rect(SKIRT_T + 3, TAB_SLIT).extrude(TAB_H + 1))
        xw = s * (in_w / 2 + 0.5)
        xn = s * (in_w / 2 - NUB_P)
        d, z0, z1 = TAB_GROOVE
        nub = (cq.Workplane("XZ").polyline([(xw, z0), (xn, z0 + NUB_P), (xn, z1 - NUB_P), (xw, z1)]).close()
               .extrude(TAB_W - 1.0).translate((0, a + (TAB_W - 1.0) / 2, 0)))
    else:
        s = 1 if wall == "front" else -1
        for da in (-(TAB_W + TAB_SLIT) / 2, (TAB_W + TAB_SLIT) / 2):
            skirt = skirt.cut(cq.Workplane("XY").workplane(offset=SKIRT_Z0 - 1).center(a + da, s * (in_l / 2 + SKIRT_T / 2)).rect(TAB_SLIT, SKIRT_T + 3).extrude(TAB_H + 1))
        yw, yn = s * (in_l / 2 + 0.5), s * (in_l / 2 - NUB_P)
        d, z0, z1 = TAB_GROOVE
        nub = (cq.Workplane("YZ").polyline([(yw, z0), (yn, z0 + NUB_P), (yn, z1 - NUB_P), (yw, z1)]).close()
               .extrude(TAB_W - 1.0).translate((a - (TAB_W - 1.0) / 2, 0, 0)))
    skirt = skirt.union(nub)

# camera pod: recessed, tilted plate with a window and four M2.5 pilots; the camera itself sits on the flat carrier
pod_w, pod_d, pod_h = CAM_W + 6, POD_D, CAM_H + 1
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
pod = pod.cut(tilted_box(CAM_W + 2, -60, -CAM_T / 2, 0, CAM_H + 20))      # cavity behind the plate (carrier + camera)
pod = pod.cut(tilted_box(POD_WINDOW, -CAM_T, CAM_T, CAM_BOARD_Z + CAM_LENS_DZ - POD_WINDOW / 2, CAM_BOARD_Z + CAM_LENS_DZ + POD_WINDOW / 2))
for hx in (-CARRIER_HOLES[0] / 2, CARRIER_HOLES[0] / 2):
    for hz in (CAM_BOARD_Z - CARRIER_HOLES[1] / 2, CAM_BOARD_Z + CARRIER_HOLES[1] / 2):
        hole = (cq.Workplane("XY").circle(M25_TAP / 2).extrude(CAM_T + 2).rotate((0, 0, 0), (1, 0, 0), 90)
                .translate((hx, CAM_T / 2 + 1, hz)).rotate((0, 0, 0), (1, 0, 0), -CAM_TILT).translate((0, base_y, base_z)))
        pod = pod.cut(hole)
skirt = skirt.union(pod)

for sx in (-1, 1):                                   # SCOPE on both side walls, readable from outside
    wp = cq.Workplane("YZ" if sx > 0 else "ZY").workplane(offset=out_w / 2)
    ty, tz = WALL_TEXT_POS
    txt = wp.center(ty, tz) if sx > 0 else wp.center(tz, ty)
    txt = txt.transformed(rotate=(0, 0, 0 if sx > 0 else -90)).text(WALL_TEXT, WALL_TEXT_SIZE, -ENGRAVE, halign="center", valign="center")
    skirt = skirt.cut(txt)

# ============================================================== CAMERA CARRIER (prints flat, front face down)
# Sits against the back of the pod plate. Camera on its back on four bosses (M1.6 x 4 into pilots), lens through the
# window; four M2.5 x 5 from the back (counterbored) through the carrier into the pod plate's pilots.
car = cq.Workplane("XY").box(CAM_W, CAM_H, CAM_T, centered=(True, True, False)).edges("|Z").fillet(2)
car = car.cut(cq.Workplane("XY").workplane(offset=-1).center(0, CAM_LENS_DZ).rect(CAM_WINDOW, CAM_WINDOW).extrude(CAM_T + 2).edges("|Z").fillet(2))
for hx in (-CAM_HOLES[0] / 2, CAM_HOLES[0] / 2):
    for hz in (-CAM_HOLES[1] / 2, CAM_HOLES[1] / 2):
        car = car.union(cq.Workplane("XY").workplane(offset=CAM_T).center(hx, hz).circle(2.2).extrude(CAM_BOSS_H))
        car = car.cut(cq.Workplane("XY").workplane(offset=-1).center(hx, hz).circle(M16_TAP / 2).extrude(CAM_T + CAM_BOSS_H + 2))
for hx in (-CARRIER_HOLES[0] / 2, CARRIER_HOLES[0] / 2):
    for hz in (-CARRIER_HOLES[1] / 2, CARRIER_HOLES[1] / 2):
        car = car.cut(cq.Workplane("XY").workplane(offset=-1).center(hx, hz).circle(M25_CLEAR / 2).extrude(CAM_T + 2))
        car = car.cut(cq.Workplane("XY").workplane(offset=CAM_T - 1.2).center(hx, hz).circle(2.6).extrude(2))
car = engrave(car, "TOP", (0, CAM_H / 2 - 3), z_top=CAM_T, size=2.5)

# ============================================================== EXPORT
out = "/mnt/user-data/outputs/"
kw = dict(tolerance=0.05, angularTolerance=0.1)
cq.exporters.export(f1, out + "floor1_battery_ubec.stl", **kw)
cq.exporters.export(f2.translate((0, 0, -F2_Z)), out + "floor2_pi_servo2040.stl", **kw)
cq.exporters.export(roof.translate((0, 0, -ROOF_Z)), out + "roof_screen.stl", **kw)
cq.exporters.export(ret.translate((0, 0, -(ret_top - RET_T))), out + "screen_retainer.stl", **kw)
cq.exporters.export(skirt.translate((0, 0, -SKIRT_Z0)), out + "skirt_walls.stl", **kw)
cq.exporters.export(car, out + "camera_carrier.stl", **kw)
cq.exporters.export(f1.union(f2).union(roof).union(skirt), out + "assembly_preview.stl", **kw)
print(f"done. floors {FLOOR_W}x{FLOOR_L}, skirt {out_w:.1f}x{out_l:.1f}x{sk_h:.1f}, roof {ROOF_W:.1f}x{ROOF_L:.1f}, top at z={ROOF_Z + T}")
