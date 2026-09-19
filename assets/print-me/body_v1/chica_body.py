"""
"Scope" – electronics body for the MYP Chica hexapod frame.
Coordinate system = MYP frame.stl: origin at frame centre, Z=0 is the TOP face of the 16 mm frame. Units mm.

Stack, bottom to top:
  floor1_battery_ubec.stl        sits on the frame (M2.5 into the frame's pilot holes). 2S LiPo bay + UBEC pocket.
  floor2_pi_servo2040_relay.stl  on 4 posts, z = 45. Pi 5 (GPIO on the left, ports to the rear), Servo2040, relay.
  roof_screen.stl                on 4 posts, z = 87. 3.5" RPi LCD (F) touch screen from below.
  skirt_walls.stl                one shell hanging from the roof: camera pod, vents, port cut-outs, switch hole.

Fasteners: M2.5 x 6 socket head everywhere structural (heat-set inserts in every post/tab), M2.5 self-tap into the Pi
standoffs, M1.6 x 6 self-tap for the camera board. Edit PARAMETERS and re-run:  python3 chica_body.py
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

# --- fasteners
M25_CLEAR = 2.7                          # M2.5 clearance
M25_TAP = 2.15                           # M2.5 self-tap into PETG(-CF)
M16_TAP = 1.35                           # M1.6 self-tap
HEATSET_D, HEATSET_DEPTH = 3.4, 5.0      # M2.5 heat-set insert (OD 3.5, length 4-5) – 3.4 hole
POST = 12                                # square post
POST_FOOT = 16                           # flared collar at the post base
FOOT_H = 3

# --- frame attachment (measured from frame.stl): 1.5 mm pilot holes on the side rails, x = ±44, y = -40 … 40
FRAME_HOLES = [(sx * 44, y) for sx in (-1, 1) for y in range(-40, 41, 10)]

# --- floor 1 (bottom): battery + UBEC
BAT_W, BAT_L, BAT_H = 48, 140, 27        # HOOVO 2S pack
BAT_CENTER = (-3, -5)                    # bay centre; leads exit the rear end
BAT_RAIL_H, BAT_STOP_H = 6, 8            # side rails; front end-stop ("right angle") so the pack can't slide forward
UBEC_W, UBEC_L = 45, 35                  # Nuofany 2-8S 8A UBEC, 45 x 35 (X x Y)
UBEC_CENTER = (44.5, 5)                  # offset so its zip slots miss the frame screw holes at (44, ±20)
UBEC_PILLAR_H = 4
POSTS_1 = [(48, 72), (-48, 72), (58, -40), (-58, -40)]     # floor1 -> floor2

# --- floor 2 (top): Pi 5 + Servo2040 + relay
PI_W, PI_L = 56, 85
PI_CENTER = (-25, 17.5)                  # long axis along Y, USB/Ethernet to the rear (-Y), GPIO along the left edge
PI_HOLE_DX = 49                          # across the board
PI_HOLES_FROM_PORT_EDGE = (23.5, 81.5)   # Pi 5 drawing: holes 3.5 mm from the SD-card edge, 58 apart -> 23.5 / 81.5 from the port edge
PI_STANDOFF_H, PI_STANDOFF_D = 6, 7
S2040_W, S2040_L = 42, 62                # fits perfectly – don't change
S2040_CENTER = (29, 31)
S2040_PILLAR_H = 5
RELAY_W, RELAY_L = 40, 65                # relay module 65 x 40, long axis along Y
RELAY_CENTER = (29, -40)
RELAY_PILLAR_H = 4
POSTS_2 = [(30, 74), (-30, 74), (58, -60), (-58, -60)]     # floor2 -> roof

# --- roof + screen (Waveshare 3.5inch RPi LCD (F): PCB 61.00 x 92.44, display 49.36 x 73.84), portrait, mounted from below
SCREEN_MODULE = (61.0, 92.44)
SCREEN_POCKET_CLEAR = 0.4
SCREEN_WINDOW = (50.4, 74.8)
SCREEN_WINDOW_DX, SCREEN_WINDOW_DY = 0.0, 0.0    # measure the panel offset on the PCB, then set
SCREEN_CENTER = (0, 18)
SCREEN_LIP_H, SCREEN_LIP_T = 6.0, 1.5
SCREEN_ZIP_DY = (-18, 18)                # zip ties across the short axis; move to miss the connector

# --- skirt
SKIRT_CLEAR, SKIRT_T, SKIRT_Z0 = 1.0, 2.4, 1.0
SKIRT_TABS = [(60, 15), (-60, 15), (60, -25), (-60, -25)]  # heat-set tabs at the top inner edge, on the side walls
PI_PORT_CUTOUT = (-56, 6, 50, 76)        # rear wall, top level: x0, x1, z0, z1 (Ethernet + USB)
XT60_CUTOUT = (-23, 17, 6, 30)           # rear wall, bottom level: pack leads / charging
VENT_Y, VENT_Z = list(range(-20, 61, 7)), (50, 78)   # both side walls, top level (Pi cooler)
SWITCH_HOLE = (20.0, -40, 20)            # right wall: diameter, y, z  (Ø20 rocker / anti-vandal)
WALL_TEXT, WALL_TEXT_SIZE, WALL_TEXT_POS = "SCOPE", 14, (10, 25)

# --- camera pod (Camera Module 3 Wide) in the skirt's front wall, recessed, 15° nose-down
CAM_TILT = 15
CAM_W, CAM_H, CAM_T = 34, 30, 3
CAM_HOLES = (21.0, 12.5)                 # official Camera Module 3 pattern (X x Z)
CAM_HOLE_D = M16_TAP                     # M1.6 x 6 self-tap
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

def pocket_pillars(part, center, w, l, h, z_base, lip=1.5, pillar=6, slot_axis="y"):
    """PCB pocket: 4 corner pillars + outer lips (pocket = exactly w x l), 2 zip-tie slots. Hole-pattern agnostic."""
    cx, cy = center
    for sx in (-1, 1):
        for sy in (-1, 1):
            px, py = cx + sx * (w / 2 - pillar / 2), cy + sy * (l / 2 - pillar / 2)
            part = part.union(cq.Workplane("XY").workplane(offset=z_base).center(px, py).rect(pillar, pillar).extrude(h))
            part = part.union(cq.Workplane("XY").workplane(offset=z_base).center(cx + sx * (w / 2 + lip / 2), py).rect(lip, pillar).extrude(h + 3))
            part = part.union(cq.Workplane("XY").workplane(offset=z_base).center(px, cy + sy * (l / 2 + lip / 2)).rect(pillar + 2 * lip, lip).extrude(h + 3))
    for sgn in (-1, 1):
        if slot_axis == "x":
            s = cq.Workplane("XY").workplane(offset=z_base - T).center(cx + sgn * (w / 2 + lip + 2.5), cy).rect(1.8, 4.0).extrude(T)
        else:
            s = cq.Workplane("XY").workplane(offset=z_base - T).center(cx, cy + sgn * (l / 2 + lip + 2.5)).rect(4.0, 1.8).extrude(T)
        part = part.cut(s)
    return part

def posts(part, xy_list, z0, height, foot=True):
    """square posts with a flared collar at the base and an M2.5 heat-set pocket at the top"""
    for (x, y) in xy_list:
        p = cq.Workplane("XY").workplane(offset=z0).center(x, y).rect(POST, POST).extrude(height)
        if foot:
            f = (cq.Workplane("XY").workplane(offset=z0).center(x, y).rect(POST_FOOT, POST_FOOT).extrude(FOOT_H)
                 .faces(">Z").edges().chamfer((POST_FOOT - POST) / 2 - 0.01))
            p = p.union(f)
        hole = cq.Workplane("XY").workplane(offset=z0 + height - HEATSET_DEPTH).center(x, y).circle(HEATSET_D / 2).extrude(HEATSET_DEPTH)
        part = part.union(p).cut(hole)
    return part

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
f1 = slot(f1, (-34.25, 0), 7.5, 80, 0, r=3)
f1 = slot(f1, (32, 45), 10, 30, 0, r=3)
f1 = slot(f1, (32, -45), 10, 30, 0, r=3)

# UBEC pocket
f1 = pocket_pillars(f1, UBEC_CENTER, UBEC_W, UBEC_L, UBEC_PILLAR_H, T, slot_axis="y")
f1 = engrave(f1, "UBEC 5V", UBEC_CENTER, size=5)
f1 = engrave(f1, "SWITCH >", (52, -45), size=3.5, angle=90)

# posts up to floor 2
f1 = posts(f1, POSTS_1, T, F2_Z - T)
f1 = engrave(f1, "FRONT", (0, 74), size=4)
f1 = clip_to_outline(f1)

# ============================================================== FLOOR 2 – PI 5 + SERVO2040 + RELAY (top floor)
f2 = outline(FLOOR_W, FLOOR_L, T).translate((0, 0, F2_Z))
f2 = through_holes(f2, POSTS_1, M25_CLEAR, F2_Z, T)

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

# Servo2040 pocket (unchanged size)
f2 = pocket_pillars(f2, S2040_CENTER, S2040_W, S2040_L, S2040_PILLAR_H, F2_Z + T, slot_axis="y")
f2 = engrave(f2, "SERVO 2040", (S2040_CENTER[0], S2040_CENTER[1] + 3), z_top=F2_Z + T, size=5)
f2 = engrave(f2, "P00 = SERVO 1", (S2040_CENTER[0], S2040_CENTER[1] - 5), z_top=F2_Z + T, size=3.2)

# relay pocket 65 x 40
f2 = pocket_pillars(f2, RELAY_CENTER, RELAY_W, RELAY_L, RELAY_PILLAR_H, F2_Z + T, slot_axis="x")
f2 = engrave(f2, "RELAY", RELAY_CENTER, z_top=F2_Z + T, size=5, angle=90)

# cable slots: servo leads down beside the Servo2040, UBEC 5 V up to the GPIO
f2 = slot(f2, (57, 20), 4, 40, F2_Z, r=1.5)
f2 = slot(f2, (55, 65), 6, 10, F2_Z, r=2)
f2 = slot(f2, (-57, 40), 4, 20, F2_Z, r=1.5)

# posts up to the roof
f2 = posts(f2, POSTS_2, F2_Z + T, ROOF_Z - (F2_Z + T))
f2 = engrave(f2, "FRONT", (0, 72), z_top=F2_Z + T, size=4)
f2 = clip_to_outline(f2)

# ============================================================== SKIRT dimensions (needed by the roof)
in_w, in_l = FLOOR_W + 2 * SKIRT_CLEAR, FLOOR_L + 2 * SKIRT_CLEAR
out_w, out_l = in_w + 2 * SKIRT_T, in_l + 2 * SKIRT_T
ROOF_W, ROOF_L = out_w + 2, out_l + 2

# ============================================================== ROOF – touch screen from below
roof = outline(ROOF_W, ROOF_L, T, chamfer=CHAMFER + 3).translate((0, 0, ROOF_Z))
roof = through_holes(roof, POSTS_2 + SKIRT_TABS, M25_CLEAR, ROOF_Z, T)
sx_, sy_ = SCREEN_CENTER
pw, pl = SCREEN_MODULE[0] + 2 * SCREEN_POCKET_CLEAR, SCREEN_MODULE[1] + 2 * SCREEN_POCKET_CLEAR
roof = slot(roof, (sx_ + SCREEN_WINDOW_DX, sy_ + SCREEN_WINDOW_DY), *SCREEN_WINDOW, ROOF_Z, r=1.5)
lip = cq.Workplane("XY").workplane(offset=ROOF_Z - SCREEN_LIP_H).center(sx_, sy_).rect(pw + 2 * SCREEN_LIP_T, pl + 2 * SCREEN_LIP_T).extrude(SCREEN_LIP_H)
lip = lip.cut(cq.Workplane("XY").workplane(offset=ROOF_Z - SCREEN_LIP_H).center(sx_, sy_).rect(pw, pl).extrude(SCREEN_LIP_H))
roof = roof.union(lip)
for dy in SCREEN_ZIP_DY:
    for dx in (-1, 1):
        roof = slot(roof, (sx_ + dx * (pw / 2 + SCREEN_LIP_T + 2.0), sy_ + dy), 1.8, 4.0, ROOF_Z)
for y in (-70, -59, -48, -37):                       # vents over the rear (battery side)
    roof = slot(roof, (0, y), 40, 6, ROOF_Z, r=2.5)
roof = engrave(roof, "SCOPE", (-53, 0), z_top=ROOF_Z + T, size=9, angle=90)
roof = engrave(roof, "TANK PRE-SURVEY HEXAPOD", (53, 0), z_top=ROOF_Z + T, size=3.5, angle=90)

# ============================================================== SKIRT
sk_h = ROOF_Z - SKIRT_Z0
skirt = outline(out_w, out_l, sk_h, chamfer=CHAMFER + SKIRT_T * 0.83).translate((0, 0, SKIRT_Z0))
skirt = skirt.cut(outline(in_w, in_l, sk_h + 2, chamfer=CHAMFER + SKIRT_CLEAR * 0.83).translate((0, 0, SKIRT_Z0 - 1)))

for (x, y) in SKIRT_TABS:                            # heat-set tabs at the top inner edge
    tab = cq.Workplane("XY").workplane(offset=ROOF_Z - 8).center(x, y).rect(POST, POST).extrude(8)
    tab = tab.cut(cq.Workplane("XY").workplane(offset=ROOF_Z - HEATSET_DEPTH).center(x, y).circle(HEATSET_D / 2).extrude(HEATSET_DEPTH))
    skirt = skirt.union(tab)

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
d, sy, sz = SWITCH_HOLE                              # Ø20 switch, right wall, bottom level
skirt = skirt.cut(cq.Workplane("YZ").workplane(offset=out_w / 2 - SKIRT_T - 2).center(sy, sz).circle(d / 2).extrude(SKIRT_T + 4))

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
cq.exporters.export(skirt.translate((0, 0, -SKIRT_Z0)), out + "skirt_walls.stl", **kw)
cq.exporters.export(f1.union(f2).union(roof).union(skirt), out + "assembly_preview.stl", **kw)
print(f"done. floors {FLOOR_W}x{FLOOR_L}, skirt {out_w:.1f}x{out_l:.1f}x{sk_h:.0f}, roof {ROOF_W:.0f}x{ROOF_L:.0f}, top at z={ROOF_Z + T}")
