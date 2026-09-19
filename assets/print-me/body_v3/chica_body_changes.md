# chica_body.py v2 — as built

What changed against the set you printed. Every number below is a named parameter at the top of `chica_body.py`; regenerate with `python3 chica_body.py`.

## Fasteners (no inserts anywhere — everything threads into PLA)

| Screw | Where | Qty | Printed hole |
|---|---|---|---|
| M3 × 6 pozi pan | roof → floor-2 posts, floor 2 → floor-1 posts | 8 | pilot 2.6, 8 deep; plate side 3.3 clearance + Ø6.5 × 1.2 counterbore |
| M2.5 × 5 | Pi 5 standoffs, GY-521 bosses, screen retainer bosses | 10 | pilot 2.1 |
| M1.6 × 4 cap | Camera Module 3 bosses | 4 | pilot 1.3 |
| none | Servo2040, relay, UBEC, Grove hub | – | lipped pockets + zip ties (hole-pattern agnostic) |
| none | **skirt** | – | no fasteners at all: rests on floor-1 pads, captured by the roof lip |

Retainer bosses get 2.5 mm of thread with the 5 mm screws — enough for a clamp-by-flex plate. If you still have any M2.5 × 6, use them there.

## Floor 1 — battery / UBEC

- Rear-right post `(58, −40)` → `(49, −62)`: clear of the switch body and its Ø26 nut.
- Rear-left post `(−58, −40)` → `(−58, −24)`: on floor 2 it sat directly under the relay pocket's corner pillar, so the screw head would have propped the relay up. Now lands in open plate behind the Pi's port edge.
- Slots widened: left 9 × 110 (was 7.5 × 80), right-front 12 × 32, right-rear 12 × 44.
- **Switch bracket** (new): U-bracket at the right edge, face flush with the plate outline, Ø20.2 hole on the skirt's switch axis (y −40, z 20), two 4 mm gussets to the floor. The switch clamps here. The skirt carries no wires and lifts straight off.
- **Skirt landing pads** (new): eight 12 × 3.6 × 2 mm pads outside the plate outline (two per wall). The skirt's bottom edge sits on them at z = 2.

## Floor 2 — Pi / Servo2040 / relay + IMU + hub

- Posts are 7 × 16 rectangles, thin side toward the boards: `(±30, 76.5)`, `(61, −60)`, `(−61, −5)`. Pi PCB clears by ≥ 4.5 mm; the microSD overhang clears the front collars.
- **GY-521** on two Ø6 bosses at `(17, 40)`, holes on the front edge, pitch 15.24, header toward the rear. M2.5 × 5.
- **Grove hub** 20 × 40 pocket at `(52, 45)`, long axis fore-aft, lips + two zip slots. Use the inboard socket row (the outboard row faces the skirt).
- Slots: servo 11 × 52, servo-rear 32 × 10, sensor/Grove 30 × 10, relay 20 × 6, GPIO 6 × 24, and the one you hand-cut along the Pi's port edge is now a proper 55 × 10 slot (`PI_REAR_SLOT`).
- Floor-1 screw positions: 3.3 clearance + counterbore, all four in open plate.
- **Front notch** 43 × 10 between the two front posts: the camera pod lives inside the skirt's front wall at z 49–80, and without the notch the plate hits it on the way up. Verified by sweeping floor 2 up through the skirt in 5 mm steps — zero contact at every height.

## Roof

- Four holes only (the posts), 3.3 + Ø6.5 × 1.2 counterbore. Pan heads sit 1.2 mm in.
- **Lip** (new): 2 mm thick, 5 mm deep, wraps the outside of the skirt's top edge with 0.4 mm clearance. Roof grows to 141.6 × 171.6 (fits the 256 bed). Prints upside-down as before — the lip is just a 5 mm wall standing on the bed.
- `SCOPE` engraving moved 3 mm inboard.

## Skirt

- **Tabs gone.** Nothing protrudes inside the shell, so floor 2 (130 × 160) passes through it in either direction, and the skirt itself comes off upward or downward.
- Height 84.6: bottom on the floor-1 pads at z = 2, top stops 0.4 below the roof so the roof always seats on the posts, never on the skirt.
- Right-wall switch hole is now Ø24.5 clearance for the bezel to pass through.

## Assembly

1. Floor 1 → frame (as before). Battery, UBEC, switch onto its bracket.
2. Floor 2 down onto floor 1's posts — 4 × M3.
3. Skirt drops over the stack onto its pads. Switch bezel passes through the clearance hole. Camera ribbon to the Pi.
4. Roof on — its lip locates the skirt — 4 × M3 into floor 2's posts. Screen jumpers to the GPIO.

**To get at floor 2:** undo the 4 roof screws, lift the roof. The skirt stays standing. Undo floor 2's 4 screws and lift it straight out through the skirt. Only the camera ribbon and the screen jumpers need unplugging.

## Measure before printing (or after — both have drill-out slack)

- `IMU_HOLE_PITCH` = 15.24 assumed. Bosses are Ø6, so ±1.5 mm can be drilled.
- `SWITCH_BEZEL_D` = 24 assumed. If the switch has no bezel wider than its Ø20 body, set it to 20.
- Skirt fit is now clearance-only (0.4 mm at the lip, 1 mm to the floors). If your printer runs fat and the lip binds, `LIP_CLEAR` 0.4 → 0.6.

## Pre-existing, left alone

Identical to your printed set, so they've already been fitted: the UBEC pocket overhangs the plate edge by 2 mm and brushes the battery rail, and the relay's rear-left corner sits over the plate's corner chamfer. If either ever binds, they're one-line changes (`UBEC_CENTER`, `RELAY_CENTER`).

## Print

Unchanged: floors flat side down, roof upside-down (screen pocket walls and lip print as walls on the bed), skirt bottom edge down, retainer flat. 8 walls, 8 top/bottom, 40 % gyroid. Slow-down for small features on — the 7 mm post walls, the 4 mm gussets and the 2 mm pads are the thinnest structural features.

Screw tally: 8 × M3, 10 × M2.5, 4 × M1.6.
