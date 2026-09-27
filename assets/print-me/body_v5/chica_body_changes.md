# Scope body – v3 (26 Sep 2026): big cable slots + clamps, floor-1 brackets, snap-on skirt, camera carrier

Source of truth: `chica_body.py` (CadQuery). Edit the PARAMETERS block, run `python3 chica_body.py`, all STLs regenerate.
Coordinates = MYP frame: origin at the frame centre, z = 0 top of frame, +Y front, +X right.

## Print list
| part | print | notes |
|---|---|---|
| `floor1_battery_ubec.stl` | yes | as exported. The clamp bridges over the slots are 1.2 mm x 10.5 mm bridges – fine on the X1C |
| `floor2_pi_servo2040.stl` | yes | as exported |
| `skirt_walls.stl` | yes | as exported; the 4 snap tabs are cantilevers in the wall – no supports |
| `camera_carrier.stl` | yes, 10 min | flat, bosses up – its accuracy is why it exists |
| `roof_screen.stl` | **only if you don't want to drill** | 3 of the 4 post holes moved. `roof_drill_template_v3.pdf` printed at 100 % marks the 3 new holes on your existing roof; the front-right hole is unchanged |
| `screen_retainer.stl` | no | unchanged |

Fasteners: M3 x 6 → 12 (4 roof, 4 lip, 4 floor 2). M2.5 x 5 → 14 (4 Pi, 2 IMU, 4 camera carrier, 4 spare). M1.6 x 4 → 4 (camera).

## Cable slots and clamps (both floors)
- **One big slot per side**: 10.5 x 84 on floor 1 (x 28…38.5, y ±42 – the whole strip the frame's central opening allows,
  outside the battery walls). Floor 2 right: the same slot, straight above (stops at y −34 for the Servo2040).
  Floor 2 left: x −63…−55, y −42…39, beside the Pi. All twelve leads of a side pass through with their plugs on.
- **Three clamps per slot** at y = 31 / 0 / −26 (L1 L2 L3, R1 R2 R3 – same on both floors), 8 mm tall C-rings that straddle
  the slot: two walls on the rims, two bridges over the slot, a 6 mm opening in the bridge that faces the free part of the
  slot. Push a bundle up through the free section, then slide it along the slot into its ring.
- Right side: vertical from floor 1 to floor 2. Left side: 25 mm outboard over the 34 mm rise (the Pi is in the way at x −53…3).
  1:1 on both sides is impossible with the Pi and Servo2040 on floor 2 – every board orientation was checked.

## Servo2040 – why it sits where it sits
Across the rear, x −7…55, with the **18-way header row on its front edge**, starting just right of the Pi's USB plugs.
That is the only placement where every header is reachable from above with the roof off: anything centred on the rear puts
the row 2 mm behind the Pi's Ethernet/USB plugs. Routing (drawn on `layout_check_floors_v3.png`):
- R1–R3: out of the floor-2 clamps, 3 mm to the right half of the row, drop straight in.
- L1–L3: out of the clamps, aft along x −59 to y −47, across the free rear-left corner (10 mm corridor behind the Ethernet plug,
  or over it), drop into the left half of the row. USB-C and the screw terminal face left into that same corner.
- 6 sensor headers on the rear edge, 4 mm from the wall: plug the foot-sensor leads there once, with tweezers, roof off.

## Floor 1
- Battery bay centred x, pushed aft (y −80…60), 25 mm guide walls with a strap slot (y −20, z 15), front end-stop.
  The pack goes in and out through the skirt's rear door.
- Right wall: relay standing on its long edge, bracket y −3…69; UBEC standing on its 12 mm edge, bracket y −55…−4.
  Two zip-tie pairs each with a 1.2 mm channel on the outer face so the tie sits flush in the 1 mm skirt gap.
- Left wall: switch bracket y −70…−36 (switch at y −53, z 20; 34 wide so the Ø26 nut clears the gussets).
- Posts, one per wall, each flush with it: front (48, 74), left (−59, 63), right (60, −63), rear (−35, −74); grooved
  1.5 deep at z 4.5–9.5 for the skirt's snap tabs.
- The frame screws at x = +44 sit under the relay/UBEC bodies → **screw floor 1 to the frame before strapping those two on.**

## Floor 2
- Pi 5 at (−25, 24.5): ports rear, GPIO left, SD edge front (y 67). Slot under its port edge for the UBEC 5 V + relay control.
- Servo2040 at (24, −56) on 5 mm pillars with lips and 2 zip slots; the front-left lip is omitted (Pi USB plugs pass above it).
- IMU at (20, 55). Front notch 40 wide for the 38 wide camera pod → floor 2 still lifts straight out.
- Roof posts: (30, 76.5) unchanged; (−50, 76.5) moved 20 mm clear of the Pi button; (61, −12) and (−61, 49) off the Servo2040 / slots.

## Skirt
- **Snap retention**: 4 tabs, one per wall (front x 48, left y 63, right y −63, rear x −35): two 1.2 mm slits from the bottom
  edge free a 9 x 20 cantilever; a 45°/45° ridge on its inner face (1.8 mm proud) rides over floor 2's edge and the post face
  with 0.8 mm deflection and clicks into the post groove. Still rests on 8 pads, still captured by the roof lip.
- Rear wall: battery door x ±26, open to the bottom edge up to z 34. Pi port cutout unchanged.
- Left wall: switch U-slot (bezel + 0.5, open to the bottom, so the skirt drops over the switch); Ø12.5 hole at (y 46, z 22)
  for a 12 mm momentary pushbutton wired to the Pi 5's **J2** pads – the proper way to add an external power button.
- Front wall: 14 x 6 slot at (x −33, z 57) in line with the onboard button, assuming it sits 21 mm from the GPIO edge along the
  SD edge – **measure that on your Pi**; the slot covers ±7 mm. Nothing is in front of it any more (the post is at x −58…−42).
- Camera pod 13 deep. Wall text at z 40, SCOPE both sides.

## Camera
The official Camera Module 3 drawing confirms the pattern in the code (21 x 12.5, holes 2.0 / 14.5 from the top edge, Ø2.2);
the sensor housing (10.8 square) is centred 0.5 mm below the hole-pair centre. The printed pod's 1.3 mm horizontal pilots
on a tilted wall are the likely culprit, so the camera now mounts on `camera_carrier.stl` (32 x 30 x 3, printed flat):
four Ø1.45 pilots with 1.5 mm bosses on the back, 15 mm window offset 0.5 mm, four counterbored M2.5 holes (26 x 22) into
pilots in the pod plate. Fit the camera to the carrier first (M1.6 from the back), drop the carrier into the pod from inside,
4 x M2.5 with the roof off.

## Assembly order
1. Floor 1 onto the frame (18 x M2.5) – before the relay/UBEC go on.
2. Switch into its bracket (nut inside); UBEC and relay zip-tied to their brackets; battery in through the rear, strap it.
3. Leg leads up through the floor-1 slots, each bundle into its clamp.
4. Floor 2 with Pi, Servo2040, IMU fitted; bundles up through the floor-2 slots into their clamps, then to the header row;
   4 x M3 down into the floor-1 posts.
5. Skirt: slide down until the four tabs click (bezel into the U-slot, pod window past the notch).
6. Roof with screen + retainer: 4 x M3 into the floor-2 posts, then 4 x M3 lip screws.
Teardown for floor-2 access: 4 roof screws → roof off (skirt stays clicked on) → floor 2 lifts straight out.

## Verified (trimesh, manifold booleans)
All parts watertight. Seated stack: 0 interference. Floor 2 lifts out through the skirt at every height; skirt sliding on shows
only the intended 0.8 mm nub flex. Pi (PCB, SD, GPIO header, Ethernet + USB plugs), Servo2040 (board, 18 plugs, USB-C plug),
IMU, battery (in place and sliding out), UBEC, relay, switch (body, nut, terminals), J2 button, the button-pin path over ±5 mm,
the six Ø7 bundle paths between floors, both floor-2 routes to the header row, and the camera carrier + board in the pod: all clear.

## Measure before trusting
Pi button position along the SD edge (21 mm from the GPIO edge assumed). Servo2040 mounting holes (design is hole-agnostic).
Switch bezel Ø (24 assumed → U-slot 24.5). IMU hole pitch (15.24 assumed; Ø6 bosses give ±1.5 mm). Relay depth (18 assumed; 3 mm spare).
