# radcam1 — Handoff

**Written 2026-08-18.** For an agent picking this project up cold.

Read this first, then `CLAUDE.md` (stable hardware reference) and
`DEVELOPMENT_STATE.md` (volatile status). This file explains *why* things are
the way they are — the part that is expensive to re-derive and easy to undo by
accident.

---

## 1. What this is

A high-reliability, zero-intervention camera + telemetry payload for a space
application, on a Raspberry Pi 5 (`radcam1`). Custom AR1335 13 MP module on a
fisheye lens, plus a radiation dosimeter, an LED illuminator and an isolated
RS422 downlink. It must run unattended, survive bit flips, and be power-minimal.

**Cameras and Pis are interchangeable.** That single requirement drives most of
the architecture: anything describing a *specific camera* lives on the camera's
own EEPROM, not on the host.

---

## 2. Five-minute orientation

```bash
cd /home/rad/driver-dev

rpicam-hello --list-cameras            # sensor alive?
systemctl is-active radcamd radcam-calibration
python3 -m radcam.cli eeprom status    # calibration intact?
python3 -m unittest discover -s tests -t .   # 51 tests, ~12 s
bash tools/verify-camera.sh            # bottom-up stack check
```

All of the above pass as of this writing. If `eeprom status` exits non-zero,
run `python3 -m radcam.cli eeprom repair` before anything else.

---

## 3. Status at a glance

| Subsystem | State |
|---|---|
| AR1335 driver, both modes, all framerates | ✅ working, 0 kernel errors |
| Colour calibration (AWB + CCM) | ✅ done, dE 22.4 → 6.8, on EEPROM, auto-applied at boot |
| Sensor response (exposure/gain/saturation) | ✅ measured, on EEPROM |
| Lens shading | ⏳ **code done, blocked on a flat-field photo** |
| Lens distortion | ⏳ **code done + validated, blocked on a suitable photo** |
| Dosimeter LTC2485 | ✅ working, 0x24 on i2c-1 |
| LED PWM, capped 10% | ✅ working |
| Flight RS422 | ✅ working — **but see §9.1, baud discrepancy** |
| EXTUART debug via RP1 PIO | ✅ 921600, bidirectional |
| Command protocol | ✅ implemented + tested |
| Test suite | ✅ 51 tests, stdlib only |
| NVMe migration | ⏸️ parked deliberately (partitioned, rootfs copied, 3 commands from done) |

---

## 4. The calibration system — the core recent work

### 4.1 One record, many tools, one place

Every per-camera calibration is a section of a **single JSON record** on the
camera module's 24C64 EEPROM (0x50, on the camera's own I2C bus — i2c-4 for
CAM1, i2c-6 for CAM0).

```
0x0000  copy 0   magic(8) length(2) crc32(4) payload
0x0800  copy 1
0x1000  copy 2
0x1800  spare
```

Three copies, each CRC-32'd, majority-voted on read, self-repairing.
Currently **793 bytes of 2034** per copy (39%).

| Section | Written by | Consumed by |
|---|---|---|
| `awb`, `ccm`, `black_level`, `flare_offset` | `tools/calibrate-camera.py` | `rpi.awb`, `rpi.ccm`, `rpi.black_level` |
| `shading` | `tools/calibrate-shading.py` | `rpi.alsc` |
| `distortion` | `tools/calibrate-distortion.py` | capture-time post-process |
| `response` | `tools/calibrate-response.py` | reference data |

**Use `CameraEEPROM.update(section, value)`, never `store()`.** `store()`
overwrites the whole record. Each tool runs on a different day, and a tool that
called `store()` with only its own results would silently erase the others.
This was a real hazard before `update()` existed.

### 4.2 Models, not grids

libcamera's ALSC wants three 32×32 tables — 3072 doubles, ~18 kB of JSON. The
EEPROM has ~2 kB for *everything*. So shading and distortion store a handful of
coefficients and the grid is regenerated on the host at apply time
(`radcam/shading.py: tables()`).

Twelve numbers replace 3072. Do not "simplify" this by storing the grid.

### 4.3 Boot flow — this is what makes cameras portable

`radcam-calibration.service` (oneshot, `Before=radcamd.service`) runs
`python3 -m radcam.calibration --any-bus`. It reads the EEPROM and writes the
values into the installed libcamera tuning file **before anything opens the
camera**.

Verified by wiping the installed tuning to an identity matrix, restarting the
service, and confirming it rebuilt the full calibration. If you change this
path, re-run that test — it is the only thing proving the portability claim.

An uncalibrated camera is **not** a boot failure. The unit exits 0 regardless;
a module with no calibration produces mediocre colour, not no images.

### 4.4 The tuning file has three copies, and they drift

| Path | Role |
|---|---|
| `/usr/local/share/libcamera/ipa/rpi/pisp/ar1335.json` | what the ISP actually reads |
| `libcamera/ar1335.json` | repo copy |
| `libcamera-src/src/ipa/rpi/pisp/data/ar1335.json` | source tree |

**`ninja install` in `libcamera-src` overwrites the installed file from the
source tree.** This silently reverted a calibration once. After changing the
tuning, sync all three.

---

## 5. Calibration tools

```bash
# Colour — the main one. Captures, finds the chart, solves, stores, applies.
python3 tools/calibrate-camera.py --camera-id ar1335-cam1 --apply
python3 tools/calibrate-camera.py --from-capture f.jpg     # from a saved frame + .dng
python3 tools/calibrate-camera.py --centres f.json         # hand-placed patch centres
python3 tools/calibrate-camera.py --show | --erase

python3 tools/calibrate-shading.py --store --apply         # needs a FLAT FIELD
python3 tools/calibrate-distortion.py --store              # needs STRAIGHT LINES
python3 tools/calibrate-distortion.py --undistort in.jpg out.jpg
python3 tools/calibrate-response.py --store                # works on any static scene

python3 tools/focus.py --preview          # live focus metric over SSH
python3 tools/detect_chart.py f.jpg       # chart location, writes an annotated preview
```

### 5.1 Why the colour solve looks over-engineered

Glare, white balance and the matrix are solved **together**
(`radcam.ccm.calibrate_from_patches`) because they are not independent:

- **Veiling glare is additive.** No gain or matrix downstream removes it. Left
  in, it makes dark saturated patches read as washed-out grey, and it bends the
  white balance and the matrix as they try to absorb it. Measured here at 8% of
  white with room lights in shot.
- The black patch measures glare **only where the black patch sits**. Lens
  shading means that figure is wrong elsewhere — subtracting it wholesale drove
  corner patches through zero. So the black patch sets the *scale of a search*,
  not the value.
- `solve_ccm_shading_invariant()` gives every patch a free brightness, so
  vignetting is not fitted as though it were colour.

**The quality gate judges whether the matrix helps, not a fixed dE.** A flat
`dE < 6` threshold would have discarded the fit that took dE 22 → 7, which is a
large visible win. The gate accepts under 4 outright, accepts under 8 if it is
a ≥35% improvement, and refuses otherwise. That refusal caught a genuinely
unusable capture (see §10.1).

---

## 6. Command protocol (`protocol.md`, `radcam/protocol.py`)

COBS-framed, CRC-16, sequence-numbered, over RS422. Full spec in `protocol.md`.

Recently added:

- **`CAPTURE_REGION (0x33)`** — reads the sensor at full 4096×3072, keeps one
  window, rescales it. *Not* the same as a 1080p capture: asking the sensor for
  1080p bins the detail away first. A 1080p window and a downscaled full frame
  cost the same airtime; only the window carries full angular resolution.
  Out-of-bounds is **refused, not clamped** — a silently moved window returns a
  picture of the wrong thing and nothing downstream can tell.
- **EEPROM access (`0x60`–`0x63`)** — read / write / status / repair, on **raw
  bytes**. If the JSON is what got corrupted, a parsing interface could not
  report it, let alone fix it. Ask `EEPROM_STATUS` first: 12 bytes tells you how
  many copies survived, and `EEPROM_REPAIR` then heals from a good one with no
  uplink at all. `EEPROM_WRITE` is refused unless explicitly unlocked
  (`eeprom_writable`), because a stray write destroys the only state aboard that
  cannot be regenerated. `EEPROM_REPAIR` is deliberately *not* gated — it can
  only copy an already-verified payload, so the worst it can do is nothing.
- **`undistort (0x0E)`** — apply the stored lens model on capture. Off by
  default; costs 0.62 s at 1080p, 3.93 s at 12 MP. For `CAPTURE_REGION` the
  correction runs **before** the window is cut, since the model is defined over
  the whole frame.

Both undistort failure modes degrade to "you still get the picture": no model,
or a malformed model, logs and stores the raw frame. A missing calibration must
not become a missing image.

---

## 7. Tests

```bash
python3 -m unittest discover -s tests -t .     # 51 tests, ~12 s
```

Stdlib `unittest` only — no pytest, no numpy-adjacent CV stack. Deliberate: this
runs on the flight Pi.

| File | Covers |
|---|---|
| `tests/test_eeprom.py` | framing, CRC, majority vote, repair, section merge |
| `tests/test_calibration_math.py` | shading / distortion / colour solvers vs known answers |
| `tests/test_protocol.py` | region bounds, EEPROM commands, write protection, undistort |

`tests/support.py::FakeEEPROM` subclasses `CameraEEPROM` at the **raw
read/write boundary**, so framing, CRC and repair run as the real code. It also
emulates the 24C64's page-wrap, so a driver that stopped chunking writes would
fail rather than pass quietly. Do not replace it with a mock of `load`/`store`
— that would test nothing.

The solver tests all fit models to **constructed data with a known answer**, so
a regression appears as a number that stops matching rather than an image that
looks slightly off. All three solvers failed in exactly that quiet way during
development.

---

## 8. Blocked work — needs a photo, not code

### 8.1 Lens shading

Everything is written and unit-tested (exact round-trip). It needs the camera
pointed at **blank white paper or an evenly lit wall filling the frame**, then:

```bash
python3 tools/calibrate-shading.py --store --apply
```

The validator correctly refuses the current chart scene (31.9% local variation,
93% left/right gradient). Do not use `--force` to get past it: an uneven target
is stored *as if it were lens shading*, and then every future capture is wrong
in a way nothing else will reveal.

### 8.2 Lens distortion

The fitter is validated — on synthetic data it recovers k=[0.345, 0.129] from a
true [0.35, 0.12], and holds up when large deliberately-wrong edges are added.

**It cannot be fitted from `captures/verified.jpg`.** The room has a *vaulted
ceiling*, so the "straight" references curve in reality. The objective bottoms
out at an implied 165° field of view, which is physically impossible for this
lens. No model from that frame is stored, and none should be.

Needs a frame with genuinely straight lines reaching large radius: a
grid/checkerboard poster filling the frame, a doorway, or a brick wall.

**Bonus:** a trustworthy fit also *measures the lens FOV* (f = r/θ), which is
the number `docs/calibration-box.md` says to measure before building the
calibration fixture. That single measurement unblocks the box design too.

---

## 9. Open issues and decisions needed

### 9.1 Flight link baud — 8× discrepancy ⚠️

`protocol.md` states 921600 on both links and derives **88 kB/s** for every
transfer-time figure in the document. The actual flight link runs at
**115200** (`DEFAULT_BAUD` in `radcam/telemetry.py`; `/etc/radcam/config.json`
does not override `flight_baud`).

```
protocol.md assumes :  88000 B/s -> 2.4 MB still in  27 s
actual flight link  :  11000 B/s -> 2.4 MB still in 218 s (3.6 min)
```

Every size/time table in `protocol.md` §1 and §8.1 is **8× optimistic**, and
`LINK_BYTES_PER_S = 88_000` in `radcam/protocol.py` feeds the
`BITRATE_EXCEEDS_LINK` check — so infinite video that the payload accepts as
transmittable is not.

This was **not** changed, because raising the baud requires the other end of the
link to match and that is a mission decision. Resolve it one way or the other
before trusting any transfer estimate. The EXTUART debug port genuinely does run
at 921600, which is probably where the number came from.

### 9.2 Black level: measured 2688, response sweep implies 2939

`tools/calibrate-response.py` extrapolates the exposure sweep back to zero
exposure and finds **+251 DN** left over. A true black level would give zero.

Stored as `response.implied_black_level` but **not applied**: it comes from one
scene with 8% veiling glare. Re-measure against a proper dark frame (lens
capped) before changing `rpi.black_level`. Getting this wrong crushes a channel
— it already happened once (see §10.2).

### 9.3 Colour: three patches carry the remaining error

Cyan (dE 26), purple (dE 26), magenta (dE 11); everything else under 8, and RED
fits at dE 0.1. The camera sees those two magentas as more alike than the eye
does and a 3×3 matrix cannot separate them. Improving this needs lower glare
(the enclosed box in `docs/calibration-box.md`), not a better solver.

### 9.4 Chart detection is good but not bulletproof

`tools/detect_chart.py` fits each row independently (rotation and perspective
tolerated), chooses column windows jointly across rows, and breaks ties using
the grey wedge's monotonicity. It gets 18/18 patches on the calibration frames.

Background blobs landing on a chart row can still shift one row's window by a
column. **The safety net is `check_capture()`**, which verifies the grey wedge
darkens monotonically and refuses the capture otherwise — so the failure mode is
a rejected run, not a silently bad calibration. Keep that check.

### 9.5 Other known-unverified items

- Sensor delays in the libcamera properties are unverified defaults (2/2/2/2).
- AWB ct_curve shape either side of the 5000 K anchor is borrowed, not measured.
- NVMe migration is parked deliberately; BOOT_ORDER and fstab untouched.

---

## 10. Traps — things already fixed, do not reintroduce

### 10.1 Orange Kapton tape over the sensor

Cost a full day. Symptom: greys fitted at dE 0–3 while every saturated colour
failed at dE 40–90.

**That asymmetry is the signature of a spectral filter, not a gain error.** A
uniform cast moves the greys too and white balance absorbs it. Greys fitting
perfectly while colours fail means something is removing colour *information*,
which no matrix can undo. Raw B/G was 0.279 and the BLUE patch read *less blue*
(3035) than the BLACK patch (3232). After removing the tape, B/G went to 0.669
and the matrix fell straight out.

### 10.2 Three driver/tuning bugs between "probe" and "images"

- Mode table written without the 100 ms settle after `0x301A = 0x0219`
  → `Error writing reg 0x30d2: -121`.
- `gen-regs.py` stripped the closing `0x301A = 0x021C`, which clears reset *and*
  sets stream. The read-modify-write then preserved the reset bit and held the
  sensor in reset forever, with D-state processes in the CFE.
- Inherited `black_level` 4096 vs measured 2688 crushed blue (B/G 0.10).

### 10.3 Solver degeneracies

- **Distortion, absolute residual:** minimised by any coefficients that shrink
  the image toward a point. Straightness is measured *relative to each edge's
  own length* for this reason.
- **Distortion, wrong basis:** a polynomial in r has no term shaped like `tan`,
  so fitting a 120° fisheye with one lands on k1 < 0 with a huge k2 — scores
  well, folds the corners back on themselves. Use `MODEL_FISHEYE`.
- **Distortion, the image circle:** the fisheye's illuminated-disc boundary is
  the longest strongest edge in every frame and it is a *circle*. Letting it
  vote drives the fit into its bounds. It is excluded explicitly, and its centre
  is used as the distortion-centre prior (centre and k1 trade off almost
  exactly, so a free centre gives the search a flat direction to wander along).
- **CCM, absolute RGB:** vignetting gets fitted as if it were colour. Hence the
  shading-invariant solver.
- **Chart columns:** a single shared pitch across all three rows cannot fit a
  chart that is both rotated and leaning (measured 419 px pitch on the top row,
  544 px on the bottom). Rows are fitted independently.

### 10.4 Measurement traps

- **Mains flicker.** Exposures that are not a whole number of half-cycles pick
  up a different slice of the flicker each time and it reads as non-linearity.
  120 Hz here (US). `calibrate-response.py --mains` handles it.
- **Noise measurement quantisation.** A percentile of |diff| picks an actual
  sample value, so with a few LSB of noise it snaps to multiples of 1 LSB.
  Sigma-clipped std instead.
- **Clipping measured frame-wide.** With a light fixture in shot, every exposure
  has saturated pixels; the response sweep must judge clipping *inside the
  measured region* or it rejects every point.
- **Benchmarks on identical frames.** Encoding 30 copies of one frame makes
  inter-frame prediction trivial and the bitrate meaningless.

### 10.5 Two defects the test suite found

- `calibrate_from_patches()` dereferenced `None` when several patches sat at
  zero: the glare search rejects offsets that floor more than one patch, and its
  *fallback* obeyed the same rule, leaving no starting point.
- `SyntheticSource.grab()` accepted a `crop` argument and **ignored it**, so
  every region-capture test compared identical frames. Fixing it also took frame
  generation from 11.65 s to 0.80 s by dropping a 12.6 M-iteration Python loop.

### 10.6 Operational

- Never `rmmod`/`modprobe` the AR1335 while `rp1-cfe` is bound — double
  registration oopses in `media_device_register_entity`. Reboot instead.
- Back up `/boot/firmware/config.txt` before editing; a bad edit costs a
  headless recovery.
- ASCII beacon and binary frames must not share a link — the beacon splices onto
  the front of binary frames. Binary on flight, ASCII on the mirror.

---

## 11. File map

| Path | What |
|---|---|
| `CLAUDE.md` | stable hardware/platform reference — read after this |
| `DEVELOPMENT_STATE.md` | volatile status, per-session |
| `protocol.md` | full ground↔payload protocol spec |
| `docs/calibration-box.md` | 3D-printed calibration fixture spec |
| `kernel/ar1335/` | V4L2 subdev driver (`ar1335.c`, `ar1335_modes.h`) |
| `overlays/` | device tree overlays |
| `libcamera/` | CamHelper + tuning JSON |
| `radcam/eeprom.py` | 24C64 access, triple redundancy, `update()` |
| `radcam/calibration.py` | EEPROM → tuning file, boot entry point |
| `radcam/ccm.py` | colour maths, joint glare/WB/CCM solve |
| `radcam/shading.py` | radial shading model ↔ ALSC tables |
| `radcam/distortion.py` | fisheye/poly models, plumb-line fit, undistort |
| `radcam/protocol.py` | message enums, config, dispatcher |
| `radcam/camera.py` | capture pipeline, sources, media store |
| `radcam/daemon.py` | `radcamd`, the always-on loop |
| `radcam/cli.py` | `radcamctl` |
| `tools/detect_chart.py` | chart location (rotation + perspective tolerant) |
| `tests/` | 51 stdlib unittest cases |

---

## 12. Suggested next actions, in order

1. **Resolve the baud discrepancy (§9.1).** Everything about transfer planning
   depends on it and it is currently wrong by 8×.
2. **Get the two blocked calibrations** (§8) — both are one photo each, and the
   distortion one also unblocks the FOV measurement and the calibration box.
3. **Re-measure black level** against a proper dark frame (§9.2).
4. Lens shading, once done, will change the colour numbers — re-run
   `calibrate-camera.py` after it and expect the CCM to improve.
5. NVMe migration, last, per the mission's own sequencing.

Do not start §5 before §2 — a rootfs move while calibration is unfinished just
adds a variable.
