# Development State

**Last updated:** 2026-08-20 — **RS-422 VERIFIED; LRT FILE TRANSFER + FEC ADDED**
**Read `CLAUDE.md` first** for the stable hardware map, `docs/stp/README.md`
for the RS-422 interface, and `HANDOFF.md` for blocked work.

---

## STP / DICE RS-422 (2026-08-18)

The payload now speaks the STP flight protocol as a DICE slave, target ID 1,
921600 baud, DE on GPIO4. Running live under `radcamd`.

```bash
tools/stp-verify.py            # 66 checks: reliability, safety, autonomy
tools/stp-sim.py --self-test   # 19-check conversation, no hardware needed
tools/stp-de-timing.py --throughput
python3 -m unittest discover -s tests -t .      # 125 tests
```

### Verified

| Check | Result |
|---|---|
| Unit tests | **158 pass** |
| Reliability / safety / autonomy | **81/81 pass** |
| Simulator conversation | **21/21 pass** |
| End-to-end through a real tty (PTY) | 49-chunk 61440 B file, bit-exact, CRC match |
| DE assertion excess | **~30 µs flat**, any packet size |
| Sustained HRT | **89.2 kB/s payload, 99.7% wire utilisation** |
| Idle CPU | 0.85% |

### Added 2026-08-20

- **File transfer over LRT** (`radcam/stp/lrtfile.py`). HRT only flows when DICE
  opens the tap, so an HRT-only payload cannot return an image if the master
  never does. The LRT file block carries 512 B per poll — ~18x slower, but it
  needs no permission from anyone.
- **Forward error correction** (`radcam/stp/fec.py`). XOR parity every
  `fec_group_size` chunks on both paths: one lost chunk per group is rebuilt on
  the ground with no retransmission, at 6.25% overhead. Measured with 8% of LRT
  replies dropped, parity rebuilt 5 of 9 missing chunks unaided and resend
  covered the rest; the file came out bit-exact.
- **Wire format verified against the ICD.** Sync transmits as `1A CF FC 1D`,
  confirming big-endian; every packet type was transmitted on the real UART and
  round-tripped through real ttys.

### Three bugs found by testing, all real

- **Duplicate `cmd_seq` executed twice.** The sequence number was recorded by
  the worker *after* execution, so a fast retransmission slipped through the
  window. For CAPTURE_IMAGE that means a duplicate file; for DELETE_MEDIA it
  means deleting whatever took the id next. Now claimed at acceptance, on the
  receive thread.
- **Resend was impossible after a transfer completed** — which is exactly when
  the ground knows which chunks were bad. Completed transfers are now retained,
  with a `reload` hook fetching from the media store beyond that.
- **`tcdrain()` held DE 8–13 ms regardless of packet size**, 140× the wire time
  of an 8-byte ACK, jamming a bus shared with up to five other experiments.
  Replaced with `TIOCSERGETLSR`/`TEMT` polling.

A fourth, found during integration: the dispatcher decodes payloads
little-endian while the ICD envelope is big-endian. Resolved as a documented
convention rather than a 30-site rewrite — see `docs/stp/README.md`.

### Still assumed, not confirmed

1. **Target ID is a placeholder (1).** Needs the real assignment.
2. **Endianness is big-endian**, matching the ICD's `0x1ACF FC1D` written most
   significant byte first — verified on the wire. The CRC parameters remain
   mission-stated rather than confirmed against DICE;
   `tools/stp-crc-solve.py` settles that from one capture.
3. **LRT trailing 2 bytes inferred to be CRC** (the ICD's rows account for only
   1254 of 1256 bytes).
4. **Stop-with-loss (0x86) semantics are a guess** — the ICD names it but does
   not define it. Implemented as stop + rewind one chunk.
5. **Never tested against real DICE hardware.**

---

## Test suite

`python3 -m unittest discover -s tests -t .` — **51 tests, ~12 s**, stdlib only.

Two real defects were found by writing it, both of which had been passing
silently:

- `calibrate_from_patches()` dereferenced `None` when a capture had several
  patches at zero. The glare search rejects any offset that floors more than
  one patch, and its *fallback* was subject to the same rule, so a dark or
  synthetic capture left it with no starting point at all.
- `SyntheticSource.grab()` accepted a `crop` argument and ignored it, so every
  region-capture test was comparing identical frames and proving nothing. Now
  the scene is defined over world coordinates so a window genuinely selects a
  different part of it — and rendering went from **11.65 s to 0.80 s** by
  dropping the per-pixel Python loop.

---

## Calibration status (2026-08-17)

| Calibration | State |
|---|---|
| Colour (AWB + CCM) | ✅ done, dE 22.4 -> 6.8, on EEPROM, applied at boot |
| Sensor response | ✅ done, on EEPROM |
| Lens shading | ⏳ tool ready, **needs a flat field** |
| Lens distortion | ⏳ tool ready, **needs a scene with truly straight lines** |

### Two are blocked on a capture, not on code

**Lens shading** needs the camera pointed at blank white paper or an evenly lit
wall filling the frame, then `tools/calibrate-shading.py --store --apply`. The
guard correctly refuses the chart scene (31.9% local variation). The model and
the tuning expansion are unit-tested with an exact round-trip.

**Lens distortion** cannot be fitted from `captures/verified.jpg` because the
"straight" references in that room are a **vaulted ceiling** - they curve in
reality. The objective bottoms out at an implied 165 deg field of view, which is
physically impossible for this lens. The fitter itself is validated: on
synthetic data it recovers k=[0.345, 0.129] from a true [0.35, 0.12], and holds
up when large deliberately-wrong chains are added. It needs a frame with
genuinely straight lines reaching large radius - a grid or checkerboard poster
filling the frame, a doorway, or a brick wall.

### Sensor response, measured

| Quantity | Value |
|---|---|
| Exposure linearity | 2.1-2.7% (9.6-12.0% including the shortest exposure) |
| Gain linearity | 2.3% over 1.0x to 4.0x |
| Saturation | 65535 DN; usable range 62847 DN above black |
| Zero-exposure offset | **+251 DN, implying black level 2939 not 2688** |

That last row is a genuine finding and is *not* applied automatically: it comes
from one scene with 8% veiling glare, and the black level is worth re-measuring
against a dark frame before changing it.

---

## Colour calibration — done (2026-08-17)

Stored on the camera EEPROM as `ar1335-cam1` and installed into the live
tuning. **mean dE76 22.4 -> 6.8, a 70% reduction.** Re-run any time with:

```bash
python3 tools/calibrate-camera.py --camera-id ar1335-cam1 --apply
```

| Quantity | Value |
|---|---|
| raw R/G, B/G at 5000 K | 0.614, 0.669 |
| AWB gains | R 1.629, B 1.496 |
| CCM diagonal | 1.81, 2.23, 1.79 |
| Veiling glare fitted | 8% of white |

Three independent captures produced matrices agreeing to within 0.06 per
element, so the fit is real rather than noise.

### The thing that was actually wrong

**Orange Kapton tape was left over the image sensor.** It is a long-pass
filter, so it passed red and blocked blue, and no white balance or matrix can
undo a filter that removes the information. The evidence, before removal:

- neutral raw B/G was **0.279** (blue getting a quarter of green)
- the BLUE patch read **less blue (3035) than the BLACK patch (3232)**
- YELLOW and WHITE had identical B/G — blue could not tell them apart
- the greys fitted at dE 0-3 while every saturated colour failed at dE 40-90

That last line is the signature worth remembering: a *uniform* cast moves the
greys too and white balance absorbs it, so greys fitting perfectly while
colours fail means something spectral, not a gain error. After removal, raw
B/G rose 0.279 -> 0.669 and the matrix fell straight out.

### What still limits it

- **Veiling glare, 8% of white.** Ceiling lights and a white desk are in shot.
  This is what `docs/calibration-box.md` exists to fix.
- **Lens shading is uncalibrated** and severe at this FOV - a dark patch in a
  corner once read lower than the black patch near centre.
- **Three patches carry the remaining error**: cyan (dE 26), purple (dE 26),
  magenta (dE 11). The camera sees those two magentas as more alike than the
  eye does, and a 3x3 matrix cannot separate them. Everything else is dE < 8,
  and RED fits at dE 0.1.

### Known limitation in `tools/detect_chart.py`

Chart location is much improved - per-row column fitting, rotation and
perspective tolerated, centres refined onto flat regions, 18/18 patches on the
calibration frames. It is **not yet reliable on every frame**: background blobs
landing on a chart row can still shift one row's column window by one. The
grey-wedge monotonicity check in `check_capture()` catches this and refuses the
capture rather than storing a bad calibration, so the failure mode is a
rejected run, not silent corruption. Re-running usually succeeds.

---

## Status: the AR1335 streams

The camera is on **CAM1** (`i2c-4`, `i2c@80000`). The original fault was a
**faulty FPC cable that was not delivering power** — replaced by the user.
Everything since has been software, and it now works end to end.

| Check | Result |
|---|---|
| Driver probe at boot | ✅ `AR1335 found, model ID 0x0153` at 3.86 s |
| CSI front end | ✅ `Using sensor ar1335 11-0036 for capture` |
| libcamera enumeration | ✅ both modes, correct crops |
| 1920×1080 still | ✅ 0 errors |
| 4096×3072 still | ✅ 0 errors |
| 1080p30 H.264 video | ✅ 135 frames, 314 KB |
| 1080p @ 30 fps | ✅ measured **30.00** |
| 1080p @ 60 fps | ✅ measured **59.97** |
| 4096×3072 @ 25 fps | ✅ measured **25.00** |
| Kernel errors during capture | ✅ **0** |

## The three bugs that stood between probe and working images

### 1. Mode table written without the reset settling

The vendor mode tables open with `0x301A = 0x0219`, a **software reset**, and
the vendor driver sleeps 100 ms immediately after it. The driver wrote all 309
registers back to back, so the sensor was still resetting when the next writes
arrived: `Error writing reg 0x30d2: -121`, a partially programmed mode, and
CSI buffer timeouts. Fixed by writing the reset alone, waiting 100 ms, then
the remainder.

### 2. The stream-enable write left the sensor held in reset

The vendor tables *close* with `0x301A = 0x021C`, which clears the reset bit
**and** sets the stream bit in one write. `tools/gen-regs.py` strips trailing
writes carrying the stream bit, so that closing write was removed — and the
driver's read-modify-write set stream while preserving bit 0. The sensor
stayed in reset, produced no frames, and the CFE blocked forever (processes
stuck in uninterruptible sleep). Fixed: stream enable now clears the reset bit
as well, reproducing 0x0219 → 0x021C exactly.

### 3. Black level guessed, not measured — blue channel crushed

The tuning file inherited `black_level = 4096` from the IMX519. Measured from
real raw AR1335 frames, the pedestal is **2688** (42 LSB at 10 bits). 4096 is
close to the *median* of a normally exposed frame, so the ISP over-subtracted
and destroyed the weakest channel: blue fell from 1846 to 374 and images came
out with essentially no blue (B/G = 0.10). Corrected to 2688 → **B/G = 1.10**.

Confirmed along the way, from the raw Bayer mosaic, that the driver's **GRBG
order is correct**: the two declared green sites differ by 3.7 out of ~8500,
while the R/B pair differ by 4842.

## Notes

* **The Pi 5 has no hardware H.264 encoder** and `rpicam-apps-lite` has no
  software fallback, so `rpicam-vid --codec h264` fails. Video is captured as
  YUV420 and encoded with libx264, which is what `radcam/camera.py` already
  does. Measured at 0.88–0.94× realtime for 1080p30 — see `protocol.md` §1.2.
* Do **not** set `i2c_slow` in config.txt. At 10 kHz the 309-register mode
  table takes over a second to write and the CSI front end times out.
* Connector **IO0 is the module's flash output** — the module drives it. Never
  drive it from the Pi.

## Still to calibrate

Black level is now measured. Remaining tuning work needs targets:
lens shading (`rpi.alsc`, currently neutralised), CCM against a colour chart,
and `rpi.lux` references. See `libcamera/README.md`.

## Remaining work

1. Wire the capture pipeline's `LibcameraSource` in place of `SyntheticSource`
   now that the sensor works, and re-run `tools/bench-compression.py` against
   real frames to replace the synthetic size estimates.
2. NVMe migration — partitioned, formatted and root already copied;
   `BOOT_ORDER` and `fstab` untouched. Three steps from done.
3. Power minimisation.

## Phase 7 — protocol live on the flight link

`radcamd` now serves the binary command protocol from `protocol.md` on
`/dev/ttyAMA0`. Verified end-to-end against the running daemon with real
hardware in the loop (live dosimeter, real PWM):

| Command | Result |
|---|---|
| `PING` | `PONG` with uptime |
| `SET_CONFIG` flash=200 | `CONFIG_ACK`, clamped to 10% on the live daemon |
| `GET_TELEMETRY` | live dose, volts, calibration flag, LED state |
| `CAPTURE_IMAGE` | `ERR_CAMERA_FAULT` — honest, sensor still dead |
| `SET_LED` 50% | capped to 10% by the real PWM channel |
| unknown opcode | `ERR_BAD_TYPE` |
| 100-byte garbage burst | 20 bad frames dropped, still answers the next PING |

### Design flaw found and fixed: two framings cannot share one stream

The first live test dispatched all 7 commands correctly but **no reply ever
decoded** — 20 bad frames dropped. Cause: the ASCII housekeeping beacon and the
binary COBS protocol were both being written to the flight link. The binary
reader treats everything before a `0x00` delimiter as one frame, so each beacon
line was spliced onto the front of the following binary frame and destroyed it.

Fixed by splitting them by purpose, which is what `protocol.md` intended
anyway: **binary protocol on the flight link, human-readable ASCII beacon on
the ground-debug mirror**. Confirmed both in the same test run.

### I2C cross-process lock

`radcamctl` and `radcamd` were interleaving conversions on the LTC2485 and both
getting nonsense — the part has no register map, so a conversion belongs to
whoever waited for it. `radcam/ltc2485.py` now takes an `flock` on
`/run/radcam/i2c-1.lock` across the whole poll-until-ready cycle. Verified with
four concurrent readers plus the daemon: all got clean, distinct conversions.
If the lock file is unavailable it degrades to unlocked rather than failing,
because a flight system must not stop reading its dosimeter over a lock.

## Phase 6 — capture pipeline + measured link budget

`radcam/camera.py` implements grab → flash → encode → store, behind a
`CameraSource` abstraction so the whole pipeline was built and benchmarked
without a working sensor. `LibcameraSource` takes over from `SyntheticSource`
unchanged once the AR1335 responds.

**Two findings that change the design:**

1. **The Pi 5 has no hardware H.264 encoder** (no `/dev/video11`; the Pi 4 had
   one). Video is encoded in software by libx264 at **0.88–0.94× realtime** for
   1080p30 — only 6–12% margin. Continuous recording is therefore **CPU- and
   power-limited, not merely link-limited**, and thermal throttling would push
   it below realtime. Consider 1080p15 or 720p for sustained recording.
2. Measured still-image cost confirms the link is the bottleneck, not the CPU:
   a full-resolution PNG is 17.6 MB and takes **3 min 20 s** to downlink but
   only 5 s to encode.

A benchmark bug was found and fixed in the process: the first video run encoded
30 *identical* frames, which made inter-frame prediction trivial and understated
bitrate badly. With real motion the achieved bitrates now track the configured
targets (563/600, 1577/1500 kbit/s), and the transmit ratios match the original
analysis (0.57×, 0.85×, 2.13×, 5.68×).

Results: `logs/compression-bench.json`. Re-run `tools/bench-compression.py`
against real frames when the sensor works.

## Next actions

**Camera (blocked on hardware):**

1. Do the physical checks above until `tools/scan-camera.sh` reports an ACK.
2. Run `tools/verify-camera.sh` — it walks the whole stack and stops at the first
   failing layer.
3. Then: libcamera CamHelper + tuning JSON (AWB/AGC/CCM — the "post processing"
   requirement), then sweep all modes and framerates.

**Payload (not blocked, in priority order):**

1. Decide the EXTUART pin strategy (phase 2 issue 1) and set `mirror_port`.
2. Add an I2C lock so the CLI and daemon cannot fight over the ADC (issue 3).
3. Investigate the dosimeter drift with a meter (issue 2), then re-calibrate.
4. **NVMe migration** — `nvme0n1` (WD Green SN3000, 500 GB) is still
   unpartitioned and root is on microSD. Not started: it rewrites the boot path,
   so it deserves a deliberate session rather than being slipped in.
5. Power minimisation — disable HDMI / Bluetooth / WiFi if unused.

## Commands needed after reboot

```bash
cd ~/driver-dev
cat DEVELOPMENT_STATE.md
dmesg | grep -iE 'ar1335|rp1-cfe'   # driver probe result
bash tools/scan-camera.sh           # assert enables, scan both camera buses
bash tools/verify-camera.sh         # full stack check, once the sensor ACKs
```

## What has been built

### `kernel/ar1335/ar1335.c` — V4L2 subdev driver
Modern kernel API (regmap/CCI, subdev state, `init_state`), runtime PM with
autosuspend (power-minimised, as the mission requires). Two modes:

| Mode | Frame rate | Link freq | Notes |
|---|---|---|---|
| 4096×3072 | 25 fps | 468 MHz | full resolution, 12 MP |
| 1920×1080 | 30–60 fps | 439.2 MHz | 2×2 binned; rate set via `V4L2_CID_VBLANK` |

Controls: exposure, analogue gain (1/1024 units, 1–24×), vblank, hblank,
pixel rate, link freq, hflip/vflip (with Bayer-order remapping).
RAW10, GRBG, 4 MIPI lanes.

### `kernel/ar1335/ar1335_modes.h` — 928 registers, generated
Produced by `tools/gen-regs.py` from NXP's GPL-2.0 isp-vvcam AR1335 driver
(vendored in `docs/reference/nxp-ar1335/`), which carries the onsemi
recommended-settings sequences. These cannot be guessed; regenerate with
`python3 tools/gen-regs.py`.

### `overlays/ar1335-overlay.dts`
Loads on either connector. Parameters: `cam0`, `addr`, `clock-frequency`,
`rotation`, `orientation`.

### Boot configuration (`/boot/firmware/config.txt`)
```
camera_auto_detect=0
dtparam=i2c_csi_dsi0=on
dtparam=i2c_csi_dsi1=on
dtparam=i2c_arm=on
dtparam=cam0_reg_gpio=35     # enable is IO1, not stock IO0
dtparam=cam1_reg_gpio=48
dtoverlay=ar1335             # CAM1
dtoverlay=ar1335,cam0        # CAM0
```
Backups in `logs/config.txt.bak.*`; tracked copy at `boot/config.txt`.
`/etc/modules-load.d/radcam-i2c.conf` loads `i2c-dev` at boot.

## Known gaps to revisit once the sensor is alive

- **hflip/vflip bit positions** (READ_MODE 0x3040 bits 14/15) are from the
  AR-family register map and have not been confirmed on this part.
- **Link frequencies** (439.2 / 468 MHz) are derived from the PLL dividers in the
  vendor sequences assuming a 24 MHz EXTCLK. If the module's oscillator is not
  24 MHz, the driver rejects it at probe — retune the tables in that case.
- No test-pattern control: the vendor sequences do not touch a test-pattern
  register and the value was not worth guessing.
- Only 2 of the possible modes are implemented; the vendor supplies exactly three
  sequences and two of them are the same mode at different frame rates.

## Recovery notes

- If the Pi fails to boot after a config change, the offending lines in
  `config.txt` are inside the `# --- radcam` banner; backups are in `logs/`.
- `tools/csi-presence.sh` is **inconclusive** for presence detection: the Pi 5
  has board-level pull-ups on both CAM connectors' I2C, so all lines read high
  whether or not a module is attached. Do not read it as proof a camera is there.
