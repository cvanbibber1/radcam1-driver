# Thermal camera protocol — extending the camera chain

**Version 1.1 — 2026-09-01 — proposed, not yet implemented**

The payload is gaining a thermal imager alongside the AR1335 visual sensor.
This specifies how it fits the addressing model already implemented, what a
thermal camera needs that a visual one does not, and which command switches
between them.

> Builds on `radcam/cameras.py` and commit `3fab1eb`, which already implement
> camera selection. Read §2 before designing anything against this: the
> selection model is an electrical interlock, and it constrains what the
> protocol is allowed to express.
>
> Companion to `docs/stp/README.md` for the link itself. Nothing here changes
> the packet envelope, the CRC, the Target ID or the flow-control rules.

---

## 1. One target, many cameras

**`Target_ID` is `0xC7` for the entire payload and never varies.** It
identifies this Experiment on a bus shared with up to five others, sits at a
fixed offset in the ICD envelope, and every packet not carrying it is dropped
before any application code runs. **Adding a camera does not add a target.**

**The camera index is a separate field**, carried inside our own command
payload and invisible to DICE and to every other experiment on the bus.

```
   DICE ─── Target_ID 0xC7 ──▶ radcam1 payload
                                 │
                                 ├── camera 0   AR1335 visual
                                 └── camera 1   thermal
```

Conflating the two would be a protocol error with consequences: a second Target
ID would make the payload answer to an address the mission did not assign it,
and on a shared bus that address belongs to somebody else.

---

## 2. The interlock: exactly one camera at a time

`radcam/cameras.py` already implements this, and the reason is electrical, not
stylistic. Two sensors sharing an I2C address answer together; two driving the
same CSI lanes contend. So the API offers **no "enable camera N"** — only
`select(n)`, which disables every camera and then enables one. **The unsafe
state cannot be expressed**, and a partial failure leaves everything off rather
than two cameras on, because the disables happen first.

Three consequences the thermal camera must respect:

- **There is no per-command camera addressing, and there must not be.** A
  command naming a camera other than the active one would require an implicit
  switch — which powers hardware down mid-operation, exactly what the interlock
  exists to prevent. Commands apply to whichever camera is currently selected.
- **`0xFF` means all cameras off.** That is a legitimate low-power state, not an
  error, and the protocol treats it as one.
- **Switching is destructive to work in progress.** `SELECT_CAMERA` stops a
  running stream and ends a recording first, because both point at hardware
  about to be powered off.

Sixteen cameras is a protocol ceiling from the byte-wide index, not an
electrical one.

---

## 3. The switch command — already implemented

```
SELECT_CAMERA   opcode 0x6D   args: index u8      (0xFF = all off)
CAMERA_LIST     opcode 0x6E   args: none
```

`SELECT_CAMERA` replies with `index u8, count u8, gpio u8`. `CAMERA_LIST`
returns the table, little-endian:

```
u8  count
u8  active            0xFF when all are off
then per entry:
  u8  index
  u8  gpio            enable line
  u8  flags           bit0 active-high, bit1 currently enabled
  i8  i2c_bus
  u8  name_len
  ..  name            ASCII, up to 16 bytes
```

Selection is payload state: it survives commands but not a reset, where
everything returns to off. Telemetry reports the active index continuously, so
the ground never has to remember what it set.

### 3.1 Adding the thermal camera

The camera set is configured, not discovered, so a canned command means the
same thing on every pass. In `/etc/radcam/config.json`:

```json
"cameras": [
  {"index": 0, "gpio": 35, "name": "ar1335",  "i2c_bus": 4, "active_high": true},
  {"index": 1, "gpio": 48, "name": "thermal", "i2c_bus": 6, "active_high": true}
]
```

The AR1335 enable lines are RP1 GPIO 35 (CAM0) and 48 (CAM1) — see `CLAUDE.md`,
noting this board uses connector IO1 rather than the stock IO0. **The thermal
camera's enable line and bus depend on the part and the harness**, and are
among the open questions in §9.

---

## 4. Command scope

With one camera active at a time, scope is simple: a command either concerns
imaging, in which case it acts on the active camera, or it does not.

**Acts on the active camera:** `CAPTURE_IMAGE`, `START_RECORD`, `STOP_RECORD`,
`CAPTURE_REGION`, `SLOT_CAPTURE_IMAGE`, `SLOT_RECORD_START`,
`SLOT_RECORD_STOP`, `STREAM_*`, `EEPROM_*` — calibration lives on each camera
module, so these necessarily follow the selection — and every `THERMAL_*`
command in §5.

**Payload-wide:** `PING`, `GET_TELEMETRY`, `GET_CONFIG`, `SET_CONFIG`,
`GET_LINK_STATS`, `GET_DOSE_LOG`, `SET_LED`, `SET_FEC_GROUP`,
`SET_HRT_IDLE_FILL`, `CLEAR_SAFE_MODE`, `ABORT_TRANSFERS`, and every `SLOT_*`
command that addresses storage rather than capture — `SLOT_LIST`, `SLOT_INFO`,
`SLOT_DOWNLOAD`, `SLOT_DELETE`, `SLOT_DELETE_ALL`, `SLOT_DOWNLOAD_ABORT`.

`SET_LED` is payload-wide because there is one illumination array. It is useful
only to the visual camera, but that is a property of the optics, not the
command — and note that illuminating a scene for the visual camera changes what
the thermal camera sees, since the LED emits heat.

### 4.1 Slots record which camera filled them

The sixteen storage slots remain one pool. **Each records the camera index that
produced it**, so the ground can tell a thermal capture from a visual one
without tracking what it commanded — which matters most exactly when something
has gone wrong and the command history is in doubt.

`Slot` gains a `camera` byte, reported by `SLOT_INFO` and `SLOT_LIST`. The
existing 22-byte `SLOT_LIST` entry has no spare, so entries become 23 bytes and
a 16-slot table grows from 354 to 370 bytes — still inside the 512-byte LRT
response window.

---

## 5. The thermal camera

### 5.1 What is different

A thermal imager is not a visual camera with different optics. Four properties
have no analogue on the AR1335, and each needs protocol support:

- **Every pixel is a measurement.** Raw output is 14- or 16-bit radiometric
  data convertible to temperature, not a picture. Making it viewable means
  mapping it through a palette, which discards the measurement.
- **Accuracy depends on scene parameters the payload cannot infer** — the
  emissivity of the target and the reflected apparent temperature of its
  surroundings.
- **It needs periodic recalibration.** A non-uniformity correction closes an
  internal shutter for roughly half a second, producing no image. This is
  normal and must be announced, or it looks like a fault.
- **Frame rates are low**, often 9 Hz, sometimes deliberately because export
  rules treat higher rates differently.

### 5.2 Bandwidth — what is and is not possible

Radiometric data is large and does not compress without destroying the
measurement it exists to carry. Against the **measured 87.6 kB/s (717 kbit/s)**
of HRT payload:

| Output | Bytes/frame | Time per frame |
|---|---:|---:|
| 640×512 × 16-bit radiometric | 655,360 | **7.3 s** |
| 384×288 × 16-bit radiometric | 221,184 | **2.5 s** |
| 320×256 × 16-bit radiometric | 163,840 | **1.8 s** |
| 640×512 palette JPEG q80 | ~40,000 | 0.45 s |
| 640×512 palette H.264 @ 9 fps, 600 kbit/s | ~8,300 | **streams live**, 84% of link |

This drives the whole design:

- **Radiometric video is impossible — do not offer it.** One frame costs
  seconds; a stream of them cannot exist on this link.
- **Radiometric stills are practical**, and are how thermal science comes down:
  capture to a slot, download over HRT like any other file.
- **Palette-mapped video streams fine**, at rates comparable to the visual
  camera. It is for situational awareness, not measurement.
- **Spot temperatures cost almost nothing** and belong in telemetry (§5.5).

### 5.3 Output mode

```
THERMAL_SET_OUTPUT   opcode 0x74   args: mode u8, palette u8, depth u8
```

| `mode` | Meaning |
|---|---|
| `0` | **Radiometric** — raw 16-bit per pixel, capture only, never streamed |
| `1` | **Palette** — 8-bit mapped, viewable and streamable |
| `2` | **Both** — one capture writes two slots, radiometric and palette |

Palettes: `0` white-hot, `1` black-hot, `2` ironbow, `3` rainbow, `4` arctic.
Palette choice affects only the mapped image and never touches radiometric data.

`STREAM_START` while the active camera is thermal and in radiometric mode is
**refused with `BAD_PARAM`** rather than silently producing something unusable.

### 5.4 Scene parameters and range

```
THERMAL_SET_RANGE       opcode 0x75   args: mode u8, low_cC i16, high_cC i16
THERMAL_SET_EMISSIVITY  opcode 0x76   args: emissivity_milli u16, reflected_cC i16
THERMAL_SET_PALETTE     opcode 0x7E   args: palette u8
```

Temperatures are **centi-degrees Celsius, signed 16-bit** — `2350` is 23.50 °C.
That gives 0.01 °C resolution across −327.68 to +327.67 °C, covers any sensor
worth flying, and needs no floating point at either end.

`mode` `0` is automatic span, `1` uses the supplied `low`/`high`. **Manual range
matters for comparing frames across a pass**: an automatic span rescales
whenever the scene changes, so two images of the same target can be mapped
completely differently and look like different targets.

Emissivity is thousandths, `0`–`1000`; `950` is 0.95. Both parameters affect
reported temperature only, never raw sensor counts — so a capture taken with
the wrong emissivity is still correctable on the ground, **provided the
radiometric data came down**.

### 5.5 Spot temperature — the cheap science

```
THERMAL_SPOT   opcode 0x7D   args: x u16, y u16, w u16, h u16
```

Returns min, max and mean over the box, in the LRT response window. **A few
bytes, on a channel that is polled anyway.**

This is the most valuable thermal command on a bandwidth-limited link: a
continuous temperature record with no bulk transfer at all. The region is
addressed in sensor pixels, the same way the visual camera's crop box is, so
both cameras can be pointed at one target and compared. `w`=`h`=1 reads a single
pixel; a box covering the frame gives whole-scene extremes.

### 5.6 Calibration

```
THERMAL_NUC   opcode 0x7C   args: none
```

Triggers a non-uniformity correction. Acknowledged immediately; the correction
takes roughly half a second, during which no frames are produced.

**The payload must announce this rather than let it look like a fault.** While a
NUC runs, telemetry sets `NUC_ACTIVE`, the stream drops frames as it would under
any other shortfall, and an event is logged. A ground station seeing a gap in
frame numbers with `NUC_ACTIVE` set knows it is a shutter, not a lost link.
Automatic NUCs, on a timer or on temperature drift, are reported identically.

---

## 6. Telemetry

The camera block at LRT offsets 767–772 already reports count, active index,
selection count and selection failures. Three bytes are spare before the event
ring at 776.

A **16-byte thermal block** is needed. Three bytes are free; the remaining
thirteen come from shortening the event ring by two entries, 38 to 36 — still
several seconds of history at any plausible rate, against per-poll thermal
state that has no other route down.

### 6.1 Thermal block

| Offset | Size | Field |
|---:|---:|---|
| +0 | 2 | `spot_mean_cC` i16 — from the last `THERMAL_SPOT` |
| +2 | 2 | `spot_min_cC` i16 |
| +4 | 2 | `spot_max_cC` i16 |
| +6 | 2 | `scene_min_cC` i16 — whole frame |
| +8 | 2 | `scene_max_cC` i16 |
| +10 | 2 | `emissivity_milli` u16 |
| +12 | 1 | `output_mode` |
| +13 | 1 | `palette` |
| +14 | 1 | `flags` — bit0 NUC active, bit1 auto range, bit2 range valid, bit3 over range |
| +15 | 1 | `nuc_count` since boot |

Scene minimum and maximum come free with every thermal frame and cost four
bytes, so they are reported continuously whether or not a spot was requested.

When the active camera is not thermal the block reads zero with `range valid`
clear, so the ground can distinguish "no thermal data" from "0.00 °C".

---

## 7. New events

| Code | Name | Meaning |
|---|---|---|
| `0x0024` | `CAMERA_SELECTED` | `arg` = index |
| `0x0025` | `CAMERA_ABSENT` | a command needed a camera that is not present |
| `0x0080` | `THERMAL_NUC_STARTED` | shutter closed |
| `0x0081` | `THERMAL_NUC_DONE` | `arg` = duration, ms |
| `0x0082` | `THERMAL_RANGE_CHANGED` | automatic span rescaled |
| `0x0083` | `THERMAL_OVER_RANGE` | scene exceeds the configured span |

`THERMAL_OVER_RANGE` matters on a manual span: the image saturates and the
temperatures are wrong. Without it the ground sees a flat white picture and
suspects the camera.

---

## 8. Failure behaviour

| Situation | Result |
|---|---|
| `SELECT_CAMERA` names an absent camera | `CAMERA_FAULT`, everything left off |
| Index ≥ 16 and not `0xFF` | `BAD_PARAM` |
| `THERMAL_*` while the active camera is visual | `BAD_TYPE` |
| `THERMAL_*` with all cameras off | `CAMERA_FAULT` |
| `STREAM_START` on thermal in radiometric mode | `BAD_PARAM` |
| Capture during a NUC | queued, taken when the shutter reopens |
| Thermal camera faults | selection to it fails; the visual camera is unaffected |

**A camera fault must not trip payload safe mode.** An absent or broken camera
fails every command sent to it, and five consecutive failures would otherwise
silence the whole payload — including the telemetry that would explain why.
Camera failures are counted per camera in the existing `camera_select_failures`
and are excluded from the consecutive-failure threshold.

---

## 9. Opcode allocation

All within the ICD's `0x00`–`0x7F` command space, in ranges verified free
against `radcam.protocol.Msg` and `StpOp`.

| Opcode | Command | Status |
|---|---|---|
| `0x6D` | **`SELECT_CAMERA`** | **already implemented** |
| `0x6E` | `CAMERA_LIST` | **already implemented** |
| `0x6F` | `CAMERA_INFO` | proposed |
| `0x74` | `THERMAL_SET_OUTPUT` | proposed |
| `0x75` | `THERMAL_SET_RANGE` | proposed |
| `0x76` | `THERMAL_SET_EMISSIVITY` | proposed |
| `0x7C` | `THERMAL_NUC` | proposed |
| `0x7D` | `THERMAL_SPOT` | proposed |
| `0x7E` | `THERMAL_SET_PALETTE` | proposed |
| `0x7F` | reserved | — |

---

## 10. A typical thermal pass

```
CAMERA_LIST                                    what is present, what is active
SELECT_CAMERA index=1                          switch to thermal; stops any
                                               stream or recording first
THERMAL_SET_EMISSIVITY 950, reflected=2000     0.95, surroundings 20 C
THERMAL_SET_RANGE mode=1 low=-2000 high=8000   manual -20 to +80 C
THERMAL_SPOT x=2104 y=1560 w=16 h=16           returns in the next LRT poll

THERMAL_SET_OUTPUT mode=1 palette=2            ironbow, viewable
STREAM_START                                   live thermal over HRT
   ...
STREAM_STOP

THERMAL_SET_OUTPUT mode=0                      radiometric
SLOT_CAPTURE_IMAGE slot=4
SLOT_DOWNLOAD slot=4                           ~1.8 s at 320x256, ~7.3 s at 640x512
HRT_GO ... HRT_STOP
SLOT_DELETE slot=4

SELECT_CAMERA index=0                          back to visual
```

The ground never changes Target ID, never re-establishes the link, and never
guesses which camera answered: every reply is keyed by `cmd_seq`, every slot
records its source camera, and the active index is in every telemetry packet.

---

## 11. Open questions for the thermal camera vendor

These cannot be specified without the part, and the first one affects the rest
of the payload.

1. **Interface.** CSI-2 competes with the AR1335 for the Pi 5's two camera
   ports — with the interlock that is workable, since only one runs at a time,
   but both still need a port. USB avoids that and costs power. SPI or serial
   is easiest to arbitrate and slowest.
2. **Enable line and I2C address**, which fill in the `cameras` config in §3.1.
   If it shares an address with the AR1335, the interlock is doing real work.
3. **Native resolution and frame rate**, which set the §5.2 figures.
4. **Radiometric format** — 14- or 16-bit, and the counts-to-temperature
   conversion, linear or a lookup.
5. **NUC duration and automatic policy**, so the payload can predict gaps.
6. **Whether the camera maps palettes itself**, or the Pi must — a CPU cost on
   a board already encoding H.264 in software at 25–30% of a core.
7. **Calibration storage** — does it carry an EEPROM like the AR1335, so
   calibration travels with the module?
