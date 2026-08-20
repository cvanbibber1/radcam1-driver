# Calibration box — 3D-printed fixture for AR1335 bring-up

A single light-tight fixture that holds the camera and every target needed to
calibrate it: field of view, focus at several distances, colour, lens shading
and distortion. The camera's own 5000 K LEDs are the only light source, so
results do not depend on the room.

Everything downstream of the field of view is computed by
`tools/calib-box-spec.py`. **Measure your FOV first** (§2) — the geometry is
not a guess you can skip, and a wrong number makes the fixture the wrong size.

---

## 1. What each test needs, and what that forces

The design falls out of the tests. Listing their requirements first explains
why the box looks the way it does.

| Test | Needs | Consequence for the fixture |
|---|---|---|
| Field of view | A target of known width at a known distance, filling the frame edge to edge | Target plane square to the axis, distance held to ±1 mm |
| Focus | A high-contrast feature, sharp, at several distances | Several target planes; they need **not** fill the frame |
| Colour | Chart lit evenly, no stray light | Enclosure, matte black interior, LEDs the only source |
| Lens shading | A **uniformly lit** white field filling the frame | Diffusing white interior, LED direct beam baffled, camera rotatable |
| Distortion | A grid filling the frame, flat and square to the axis | Full-frame target plane, held flat |
| Veiling glare | Everything non-target black | Flocked interior, no shiny fasteners in view |

Two of those pull in opposite directions:

- Colour and distortion want a **matte black** interior so that light which
  misses the target does not come back as flare. The current bench captures
  measure 22–30 % veiling glare, which is the single largest error in the
  colour fit.
- Lens shading wants a **white diffusing** interior so that the flat field is
  lit uniformly.

They are resolved with a **removable white liner** (§5.4) that drops into the
black shell for shading work and comes out for everything else.

---

## 2. Measuring your FOV first

Every dimension below scales with the lens's real field of view. The AR1335
module here has no published figure, so measure it once:

1. Tape a ruler or a printed scale to a wall, horizontal, marks facing the
   camera.
2. Set the camera on the axis, lens exactly `D` mm from the wall — measure `D`
   from the **front of the lens barrel**, and make `D` at least 200 mm so the
   error in it is small.
3. Capture at full resolution: `rpicam-still -n -t 2000 -o fov.jpg`
4. Read off the scale value at the extreme left and right edges of the frame.
   The difference is the frame width `W` at that distance.
5. Horizontal FOV = `2 * atan(W / (2*D))`, and the diagonal is

   ```
   dfov = 2 * atan( tan(hfov/2) * 5/4 )        # 4:3 frame
   ```

`tools/focus.py --preview` makes step 4 easier — you can see the edges live
while sliding the camera, instead of capturing and checking.

Then generate the dimensions:

```bash
python3 tools/calib-box-spec.py --dfov 120 --stations 60,120,240,400
```

### Why a full-FOV horn does not work at range

The obvious design is a cone whose walls *are* the field-of-view pyramid, so a
flat plate anywhere along it fills the frame exactly. That is a lovely idea and
it collapses immediately on contact with a wide lens:

| Diagonal FOV | Frame at 60 mm | Frame at 400 mm |
|---|---|---|
| 120° | 166 × 125 mm | **1109 × 831 mm** |
| 100° | 114 × 86 mm | 762 × 572 mm |
| 80° | 81 × 60 mm | 538 × 403 mm |

A 400 mm focus station would need a mouth over a metre across. It cannot be
printed, cannot be enclosed cheaply, and is not needed — because **only the
near station has to fill the frame**. Flat field, distortion and FOV all work
at the nearest plane. Focus does not care about frame coverage at all; it needs
a sharp feature on axis, and the tunnel walls may perfectly well appear around
it.

So the body is two zones:

```
   camera                                                     far cap
     |                                                            |
     |<-- flare -->|<---------------- parallel tunnel ----------->|
     |    0-60mm   |                    60-440mm                  |
     |             |                                              |
     +----.        +==============================================+
   throat  ` - _   |                                              |
   20x20        ` -+ 227 x 166 internal                           |
                   |                                              |
     +----'        +==============================================+
              _ - '
       _ -  '
```

The flare follows the FOV (plus margin) out to the plane where the frame
reaches full section; past that the tunnel runs parallel. Walls appear in
frame beyond the flare plane, which is exactly what the far stations tolerate.

---

## 3. Dimensions (worked example: 120° diagonal)

Regenerate for your measured FOV — do not copy these if yours differs.

```
Field of view      120.0° diagonal → 108.4° horizontal, 92.2° vertical
Wall margin        8° per side → walls cut at 124.4° × 108.2°
Flare plane        60 mm
Internal section   227 × 166 mm
Internal length    440 mm  (far station 400 mm + 40 mm behind it)
Throat             20 × 20 mm at the lens (clears the LEDs)
```

| Station | Distance | Frame | Plate (+10 mm) | Resolution at target |
|---|---|---|---|---|
| S1 | 60 mm | 166 × 125 mm | 176 × 135 mm | 24.6 px/mm |
| S2 | 120 mm | 333 × 249 mm | — (partial frame) | 12.3 px/mm |
| S3 | 240 mm | 665 × 499 mm | — (partial frame) | 6.2 px/mm |
| S4 | 400 mm | 1109 × 831 mm | — (partial frame) | 3.7 px/mm |

Only **S1** takes a full-frame plate, at 176 × 135 mm — comfortably inside a
sheet of A4. S2–S4 take small central targets.

A useful bound: at 120° a full-frame plate still fits on A4 out to 101 mm, and
on A3 out to 143 mm. If you want a *second* full-coverage station, put it at
100 mm and print the target on A4; further than that and you are buying A3.

---

## 4. Construction strategy — printed frames, flat panels

At 227 × 166 mm internal, printing the tunnel as solid shells means many hours
and a lot of filament for what is, optically, four flat black walls. Instead:

- **Print the geometry that matters**: the flare, the corner rails, the target
  carriers, the camera mount. These are small and fit any 220 mm bed.
- **Buy the flat bits**: 3 mm black PVC foam board (Foamex/Palight) for the
  four tunnel walls, cut with a knife. It is cheap, rigid, opaque, matte black
  through the thickness, and takes a screw.

This also makes the box repairable and re-sizeable: a different lens means new
rails and new panel cuts, not a reprint of everything.

---

## 5. Modules

### 5.1 Camera mount — rotatable, detented at 0/90/180/270°

```
        ___________
       /           \        A: adapter plate, slotted M2, carries the module
      |   +-----+   |       B: rotor, 60 mm dia, ball detent every 90°
      |   |  A  |   |       C: stator, bolts to the throat flange
      |   +-----+   |       D: 20 x 20 mm throat aperture
       \_____B_____/
      ==============        rotation axis = optical axis, within 0.5 mm
            C
```

- Adapter plate: 40 × 40 mm, 3 mm thick, with **slotted** M2 holes on both
  axes so it fits whatever pattern your module has. Measure your board and cut
  the slots to suit; there is no standard here.
- Rotor/stator: a 60 mm spigot in a 60.4 mm bore, with a 3 mm steel ball and a
  printed leaf spring engaging four detent dimples. Print the rotor at 0.2 mm
  layers and expect to sand the spigot.
- **Why it rotates:** see §7.3. Without it you cannot separate lens shading
  from illumination falloff.
- Engrave the four angles on the rotor face so the captured filename can record
  which one it was.

The FPC exits through a **labyrinth slot** — two offset slots with a 6 mm
overlap so no straight-line path exists — and is sealed with a strip of foam
weatherstrip.

### 5.2 Flare section

- Truncated rectangular pyramid: 20 × 20 mm at the throat, 227 × 166 mm at
  60 mm, walls following the FOV plus 8° per side.
- Print in **four quadrant pieces** (~120 × 90 mm each), joined by 10 mm
  flanges with M3 heat-set inserts. Split lines run along the diagonals so no
  seam sits on the frame's horizontal or vertical edge.
- Interior gets flocked paper (§6). The flare is where flare comes from — it is
  the only surface the lens sees at wide angle.

### 5.3 Tunnel

- Four corner rails, printed in ~190 mm lengths, L-profile with a 3.2 mm slot
  each side for the panels. Rails butt-join with a 20 mm spline and two M3
  screws; stagger the joints between adjacent corners so the box has no
  continuous seam.
- Panels: 3 mm black PVC foam, 233 × 190 mm (sides) and 172 × 190 mm (top and
  bottom) per section, cut to suit your rail design.
- Rails carry a **T-slot on the inner face** at 10 mm pitch so target carriers
  can be positioned anywhere, not only at the four nominal stations.
- Every panel seam gets a bead of black silicone or a strip of foam tape. Test
  for light leaks by putting a bright torch inside in a dark room.

### 5.4 White liner (lens shading only)

- Four sheets of 1 mm matte white styrene (or PTFE sheet, better but pricier)
  cut to slide into the same T-slots, sitting just inside the black panels.
- Plus a **baffle**: a 60 mm black disc on a 20 mm standoff, centred on the
  axis, that blocks the LEDs' *direct* beam from reaching the target plate
  while leaving the lens's view clear at the edges. Every photon reaching the
  target has then bounced at least once off white wall.
- Consequence: the flat field is lit like a crude integrating cavity — dim,
  but genuinely uniform. Expect to raise the exposure by 2–3 stops. That is
  fine; the field is static.

### 5.5 Target carriers

- A U-shaped frame that slides into the rail T-slots and holds a plate by its
  edges, with a 3 mm lip. Two thumbscrews lock it.
- The carrier registers the plate **square to the axis**; print the seat, do
  not rely on the plate being pushed flat.
- Carriers for the far stations are open in the middle, so a small target does
  not block the tunnel when you want the light through it.

### 5.6 Rear cap

- Printed frame + panel, on two printed hinges and a latch, with a foam gasket
  in a groove. Holds the S4 target on its inner face.
- This is the access door: everything else stays assembled.

---

## 6. Bill of materials

| Item | Qty | Note |
|---|---|---|
| PLA or PETG filament, black | ~600 g | PETG if the LEDs will run warm |
| 3 mm black PVC foam board | 1 × A2 sheet | tunnel panels |
| 1 mm matte white styrene | 1 × A2 sheet | removable liner |
| Self-adhesive black flocked paper | 1 × A3 | flare interior — this is what kills veiling glare |
| M3 heat-set inserts | 40 | |
| M3 × 8 and M3 × 12 socket screws, black | 40 | black, not zinc: bright fasteners in frame flare |
| M2 screws + nuts | 8 | camera module |
| 3 mm steel ball | 1 | rotation detent |
| Foam weatherstrip, 3 × 10 mm | 2 m | seams and FPC slot |
| Black silicone sealant | 1 tube | optional, for permanent seams |

Targets are printed on paper (§8) except the colour chart, which is the
physical DKK card.

---

## 7. Print settings

- 0.4 mm nozzle, 0.2 mm layers, 3 perimeters, 20 % infill. Nothing is
  structural except the rails, which get 4 perimeters.
- **Print black, and do not sand the interior smooth.** A slightly rough matte
  surface scatters less specularly than a polished one. The flocking goes on
  top regardless.
- Flanges print flat side down; no supports needed anywhere if the flare
  quadrants are oriented with the split face on the bed.
- Check dimensional accuracy on the rail spline before printing all of them.

---

## 8. Targets

| Plate | Station | Content |
|---|---|---|
| T1 — flat white | S1 | Plain matte white, no print. Lens shading. |
| T2 — distortion grid | S1 | 10 mm grid of 2 mm black dots, printed on matte paper, edge-to-edge |
| T3 — FOV scale | S1 | Horizontal and vertical rules, 5 mm ticks, labelled every 10 mm, origin at centre |
| T4 — colour | S1 | DKK card in a recess, surrounded by black mask so only the card is lit |
| T5 — slanted edge | S1–S4 | 5° slanted black/white edge, 40 × 40 mm. One per station. |
| T6 — focus star | S2–S4 | Siemens star, 40 mm, plus fine text |

Print T2, T3, T5 and T6 on **matte** paper at the highest resolution you have,
and check with a ruler that the printer has not scaled them. Mount on 3 mm
foam board with spray adhesive, rolled flat.

For T4, the mask matters: 22–30 % of the current colour error is light from
outside the chart returning as flare. Mask everything that is not a patch.

---

## 9. Procedures

### 9.1 Field of view

Fit T3 at S1, capture, read the tick values at the frame edges. Compare with
the computed frame width; a mismatch means the station distance or the assumed
FOV is wrong. Feed the corrected FOV back into `calib-box-spec.py`.

### 9.2 Focus

```bash
python3 tools/focus.py --preview --roi 0.4,0.4,0.2,0.2
```

Fit T5 at the station of interest, turn the lens until the number stops
rising, back off to the peak. Repeat at S1 through S4 and record the peak
score for each — the distance with the highest peak is where the lens is
actually focused, and the spread across stations is the depth of field.

### 9.3 Colour

```bash
python3 tools/calibrate-camera.py
```

T4 at S1, liner **out**, baffle **out**. The enclosure is the point: with the
box closed, the flare term the tool currently has to model away should mostly
vanish, and the blue-dominant patches that are presently at the noise floor
should come up out of it.

If the fit is still refused by the quality gate, raise the exposure until the
brightest neutral patch reads about 85 % of full scale, and re-run — the fit
is limited by the blue channel, which the AR1335's raw B/G ≈ 0.27 pushes down
hard.

### 9.4 Lens shading

T1 at S1, liner **in**, baffle **in**. Capture a raw frame at each of the four
rotor detents, recording the angle in the filename.

The reason for the four angles is worth stating, because it is the one part of
this fixture that is not obvious:

> The measured falloff is the product of two things — lens/sensor shading `L`,
> which is fixed **to the sensor** and therefore rotates with the camera, and
> illumination non-uniformity `I`, which the diffusing cavity fixes
> approximately **to the box**. One capture cannot separate them.
>
> Rotate each capture back into sensor coordinates and take the geometric mean
> of all four:
>
> ```
> (L·I₀ · L·I₉₀ · L·I₁₈₀ · L·I₂₇₀)^(1/4)  =  L · (I₀I₉₀I₁₈₀I₂₇₀)^(1/4)
> ```
>
> The second factor is the 4-fold rotational average of a smooth field, which
> is very nearly constant. What remains is `L` alone.

Two angles (0° and 180°) already remove everything odd-symmetric and are worth
doing if the detents are stiff; four removes the two-fold component as well.

### 9.5 Distortion

T2 at S1, liner out. Capture, and fit a radial model to the dot centres. The
detection in `tools/detect_chart.py` already fits a quadratic surface for
exactly this reason and its residual is a direct read on how much distortion
there is.

---

## 10. Known compromises

- **The LEDs are on the camera.** They move with it, so they cannot be used as
  a box-fixed reference. The white cavity plus baffle is what makes the
  illumination approximately box-fixed; without the baffle the direct beam
  dominates and the four-angle trick in §9.4 fails.
- **5000 K is nominal.** The LEDs' actual correlated colour temperature is not
  measured, and the AWB CT labels in the tuning file are still assumed. The box
  makes colour *repeatable*, not absolute. Absolute colour still needs the DKK
  card, which is why T4 exists.
- **The far stations do not fill the frame.** That is deliberate (§2) but it
  does mean the box cannot do a full-frame test at anything except S1. If you
  need one at range, the fixture is the wrong tool — use a wall.
- **Nothing here is space-qualified.** This is ground support equipment. PLA,
  foam board and spray adhesive are all fine on a bench and none of them are
  going anywhere near flight hardware.

---

## 11. Files

| File | Purpose |
|---|---|
| `tools/calib-box-spec.py` | computes every dimension above from a measured FOV |
| `tools/focus.py` | live focus metric, §9.2 |
| `tools/calibrate-camera.py` | colour and white balance, §9.3 |
| `tools/detect_chart.py` | chart location, also the distortion residual in §9.5 |
