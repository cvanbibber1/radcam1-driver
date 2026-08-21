# radcam1 — User Guide

A camera and telemetry payload for a low Earth orbit flight: an AR1335 13 MP
sensor on a Raspberry Pi 5, with a radiation dosimeter, an illumination LED,
and an RS-422 link to the spacecraft.

This guide is for someone who has never touched the project. It goes in the
order you actually need things: check it works, look at the camera, take a
picture, get the picture down the link. Every section shows what you should see
if it is working.

> Deeper references, if you need them: `CLAUDE.md` (hardware map),
> `docs/stp/README.md` (flight link internals), `HANDOFF.md` (open questions),
> `DEVELOPMENT_STATE.md` (current status). A full command list is at the bottom
> of this page.

---

## 1. Is it alive?

```bash
systemctl status radcamd
radcamctl status
```

`radcamd` is the always-on daemon: it samples the dosimeter, keeps the LED
safe, and answers the spacecraft over RS-422. If it is running you will see it
`active (running)`, and `radcamctl status` prints the current dose, temperature
and uptime.

Run the built-in self-test, which exercises the dosimeter, LED, UART and the
redundant-storage layer:

```bash
radcamctl selftest
```

Watch the daemon's log live — the single most useful command when something is
behaving oddly:

```bash
journalctl -u radcamd -f
```

### Restarting after you change anything

The daemon runs from `/opt/radcam`, **not** from this source tree. Editing a
file here changes nothing until you install it:

```bash
sudo bash tools/install-radcam.sh
sudo systemctl restart radcamd
```

That is the single most common cause of "my change did nothing".

---

## 2. Is the camera working?

```bash
bash tools/verify-camera.sh
```

This checks the whole stack from the bottom up and stops at the first layer
that fails: I2C address → kernel driver → media graph → actual capture. Working
output ends with a successful capture.

Quick individual checks:

```bash
i2cdetect -y 4              # CAM1 bus: the sensor answers at 0x36
rpicam-hello --list-cameras # libcamera sees an ar1335
```

Take a picture and look at it:

```bash
rpicam-still -o /tmp/test.jpg
```

If the camera is not detected, `CLAUDE.md` has the GPIO map — the most common
cause is the sensor enable line, which on this board is IO1 (RP1 lines 35/48),
not the stock IO0.

---

## 3. Focus and framing

Focus is manual, so there is a live meter. Over SSH, serve it as MJPEG and open
the URL in a browser:

```bash
python3 tools/focus.py --http 8080
```

Higher numbers are sharper. Turn the lens until the number peaks.

Before a colour calibration, check the chart is framed and sharp enough to be
worth capturing:

```bash
python3 tools/chart-framing.py
```

---

## 4. Calibrating a camera

Calibration lives on **the camera module's own EEPROM**, not on the Pi, because
modules and Pis are interchangeable and the calibration has to travel with the
optics. `radcam-calibration.service` applies it at boot, so a module carries its
calibration to any Pi with no manual step.

Colour is the one that matters most. Point the camera at a colour chart, then:

```bash
python3 tools/calibrate-camera.py --camera-id ar1335-cam1 --apply
```

A good result cuts mean colour error (dE76) by more than half; the reference run
on this hardware went from 22.4 to 6.8. The tool refuses a fit that does not
actually help, which is the outcome you want when the capture was poor.

Inspect or clear what a module is carrying:

```bash
python3 tools/calibrate-camera.py --show
python3 tools/calibrate-camera.py --erase
```

The other calibrations, each needing a particular scene:

| Command | Needs |
|---|---|
| `tools/calibrate-shading.py --store --apply` | a **flat field** — blank white paper or an evenly lit wall filling the frame |
| `tools/calibrate-distortion.py --store` | a scene with **genuinely straight lines** reaching the frame edges |
| `tools/calibrate-response.py --store` | a static scene; measures exposure and gain linearity |
| `tools/find-crop.py --store` | detects the usable frame region |

> A trap worth knowing: a vaulted ceiling is not a straight line. The distortion
> fitter will happily converge on a physically impossible 165° field of view if
> you give it curved references.

---

## 5. Dosimeter and LED

The dosimeter is an LTC2485 24-bit ADC at roughly 2.5 mV/rad, on `i2c-1` at
address 0x24.

```bash
radcamctl dose --watch      # live readings
radcamctl calibrate         # establish the zero point
```

Calibration is stored triple-redundantly in `/var/lib/radcam/` and survives
corruption of any one copy.

The illumination LED is PWM on GPIO18 and is **hard-capped at 10% duty in
software**. Asking for more is clamped, not refused:

```bash
radcamctl led 5             # 5% of full scale
radcamctl led 100           # applies 10%
```

---

## 6. The flight link (RS-422)

In orbit this is the only channel. The payload is an *Experiment* on a bus
mastered by the spacecraft's flight computer (**DICE**), shared with up to four
other experiments. The rule that shapes everything: **the payload never
transmits unless spoken to.**

| | |
|---|---|
| Port | `/dev/ttyAMA0`, 921600 baud, 8N1 |
| Transceiver | ADM2582E, driver enable on **GPIO4** |
| Our address | **Target ID `0xC7`** |
| Byte order | **big-endian**; sync pattern `1A CF FC 1D` |
| CRC | CRC-16/CCITT-FALSE, always the **final two bytes** of every message |

Three things travel over it:

- **Commands** (120 bytes, in) get an 8-byte acknowledgement. That ACK has no
  status field in the spec, so it means *accepted*, never *done* — the result
  comes back in the next telemetry poll, tagged with the sequence number you
  sent.
- **LRT** (low rate telemetry) is **vitals only**: dose, temperature, storage,
  link health, the last command's result, and a ring of recent events. No bulk
  data ever travels on LRT.
- **HRT** (high rate telemetry) carries **live video** and **chunked file
  transfer** of stored media. It only flows between an `HRT Go` and a `Stop`
  from DICE.

### Trying it without a flight computer

The simulator plays DICE, either in-process or over a serial port:

```bash
python3 tools/stp-sim.py --self-test
```

That runs a full conversation — command/ACK, telemetry poll, an HRT file
transfer, live-stream configuration, target filtering, duplicate suppression,
and recovery from corrupted input — and prints PASS or FAIL for each.

Talk to a real payload over a wire (note the target ID):

```bash
python3 tools/stp-sim.py --port /dev/ttyAMA0 --target 0xC7 --lrt
python3 tools/stp-sim.py --port /dev/ttyAMA0 --target 0xC7 --ping
python3 tools/stp-sim.py --port /dev/ttyAMA0 --target 0xC7 --download 1
```

### Live video

This is what HRT is really for. The payload encodes H.264 and streams it as
frames are produced.

```bash
# Aim a 640x480 native-resolution box at the centre of the sensor, and start.
python3 tools/stp-sim.py --port /dev/ttyAMA0 --target 0xC7     --stream-region 2104,1560,640,480     --stream-size 640x480@15:600 --stream-start

# Watch for 20 seconds and save what arrives.
python3 tools/stp-sim.py --port /dev/ttyAMA0 --target 0xC7     --watch-stream 20 --stream-out /tmp/live.h264

python3 tools/stp-sim.py --port /dev/ttyAMA0 --target 0xC7 --stream-stop
ffplay /tmp/live.h264          # or: ffmpeg -i /tmp/live.h264 out.mp4
```

Measured end to end on this hardware at 640x480, 15 fps, 600 kbit/s requested:

| Quantity | Result |
|---|---|
| Frame rate | **15.0 fps** |
| Bitrate | **584 kbit/s** |
| Keyframes | one per second |
| Chunk CRC failures | **0** |
| Decode | 168 frames, 640x480, **no decoder errors** |

**A stream drops, it does not queue.** If the link cannot keep up, or DICE
closes HRT, the oldest frames are discarded rather than buffered. A rising
`stream_frames_dropped` under load is correct behaviour — buffering would mean
a delay that only grows, and ten-minute-old video is worse than none. Live
video takes priority over file transfer on HRT; file chunks use the gaps
between frames, so a transfer alongside a stream still makes progress.

### Pointing the camera without moving it

The sensor is 4208x3120 but the link carries only ~600 kbit/s. Streaming the
whole frame spends nearly all of it on detail the encoder then throws away.
Instead, choose a **box centred on any sensor pixel**:

```bash
# 512x512 at native resolution, centred on sensor pixel (3000, 2000)
--stream-region 3000,2000,512,512 --stream-size 512x512@15:600
```

When the crop equals the output size you get **1:1 sensor pixels** — full
optical detail of just the area you care about. A crop larger than the output
is scaled down, giving a wider view with less detail. Everything is clamped to
the sensor and echoed back in telemetry, so a box asked for near the edge comes
back as the box actually applied.

### Getting a stored file down

Files go over HRT in numbered chunks, each with its own CRC-32:

```bash
python3 tools/stp-sim.py --port /dev/ttyAMA0 --target 0xC7 --download 1
```

Every 16 chunks are followed by an XOR parity chunk, so **a single lost chunk
per group is reconstructed on the ground with nothing asked in return**, at
6.25% bandwidth cost. Losses beyond that fall back to an explicit resend.
Tune or disable it with `SET_FEC_GROUP` (0x77) — larger groups mean less
overhead and less protection, and 0 turns parity off.

Parity deliberately does *not* apply to live video: a video frame that arrives
late is worthless, and H.264 recovers at the next keyframe a second later for
free.

### Checking the link on real hardware

The most important measurement is how long the driver-enable line stays
asserted, because every microsecond past our last stop bit is time another
experiment cannot transmit. `radcamd` holds the port exclusively, so stop it
first:

```bash
sudo systemctl stop radcamd
sudo python3 tools/stp-de-timing.py --throughput
sudo systemctl start radcamd
```

Healthy output is a flat **~25–35 µs excess at every packet size** and about
89 kB/s sustained. If the excess runs to milliseconds or scales with packet
size, the transmitter has fallen back to `tcdrain()` — check the log line
printed when the link opens for which release method is in use.

### If the CRC or byte order ever looks wrong

The values above are mission-confirmed, but the original specification left
them undefined. If the link does not work against the real flight computer,
capture some of its traffic and let the solver identify the real parameters:

```bash
python3 tools/stp-crc-solve.py --bin capture.bin
```

It searches 15 standard CRC-16 variants across both byte orders and three
coverage ranges, and prints the exact config change needed.

## 7. Configuration

`/etc/radcam/config.json`. **Back it up before editing** — a bad edit costs a
headless recovery:

```bash
cp /etc/radcam/config.json /home/rad/driver-dev/logs/config.json.bak.$(date +%Y%m%d-%H%M%S)
```

The keys that matter most:

| Key | Meaning |
|---|---|
| `interval_s` | housekeeping sample period, seconds |
| `led_enabled`, `led_brightness` | illumination; brightness is still capped at 10% |
| `stp.enabled` | turn the flight link on; **replaces** the old debug protocol |
| `stp.target_id` | our address on the bus — **`199` (0xC7)** |
| `stp.baud` | 921600 |
| `stp.de_gpio` | driver enable pin, 4 on this board |
| `stp.big_endian` | `true` |
| `stp.crc_variant` | `"CRC-16/CCITT-FALSE"` |
| `stp.fec_group_size` | chunks per parity chunk on HRT transfers; 0 disables |
| `stream.width`, `stream.height` | stream output size, default 640x480 |
| `stream.fps`, `stream.bitrate` | default 15 fps, 600000 bit/s |
| `stream.centre_x`, `stream.centre_y` | sensor pixel the crop box is centred on |
| `stream.crop_w`, `stream.crop_h` | crop size; equal to output means native pixels |
| `stream.queue_frames` | frames buffered before the oldest is dropped |

> With `stp.enabled` true, the flight port belongs to the RS-422 link and the
> readable ASCII beacon goes only to the debug mirror on GPIO23/24. This is
> deliberate: an unsolicited byte on a shared bus corrupts another experiment's
> reply.

---

## 8. Testing and verification

```bash
python3 -m unittest discover -s tests -t .   # 176 tests, ~17 s
python3 tools/stp-verify.py                  # 85 reliability/safety/autonomy checks
python3 tools/stp-sim.py --self-test         # 21 protocol checks
```

`stp-verify.py` is the adversarial one: it fuzzes the receiver with random
bytes, corrupts every byte position of a command, exhausts queues, forces
transmit failures, simulates bit flips in memory, and drops packets mid-transfer
to confirm the parity actually repairs them, and checks that live video is
discarded rather than queued when the link is closed. Run the suites separately
with
`--suite reliability`, `--suite safety` or `--suite autonomy`.

---

## 9. When something is wrong

| Symptom | Look at |
|---|---|
| Change had no effect | you edited the source but did not `install-radcam.sh` |
| Camera not detected | `bash tools/verify-camera.sh`; check enable GPIO in `CLAUDE.md` |
| `Device or resource busy` on `/dev/ttyAMA0` | `radcamd` holds it; stop the service first |
| Payload silent on the bus | it is *supposed* to be silent until polled; send an LRT request |
| Nothing comes back over HRT | DICE has not sent `HRT Go` — check `stream_gated` in telemetry |
| Stream will not start | no hardware H.264 on a Pi 5; needs `ffmpeg` with libx264 present |
| `stream_frames_dropped` climbing | normal under load — the link or encoder cannot keep up, so stale frames are discarded by design |
| Video decodes as garbage | you started reading mid-stream; begin at a frame flagged as a keyframe |
| Colours wrong, greys fine | something spectral, not gain — check for filters or tape over the sensor |
| `safe_mode` set in telemetry | five commands failed in a row; check events, then `CLEAR_SAFE_MODE` |
| Link works then stops | check `rx_bad_crc` and `rx_resyncs` in telemetry |

Safe mode deliberately keeps answering telemetry while refusing other work: a
payload that has gone quiet is indistinguishable from a dead one.

---

## 10. Every command

### Daily operation

| Command | Does |
|---|---|
| `radcamctl status` | dose, temperature, uptime, subsystem state |
| `radcamctl selftest` | exercises dosimeter, LED, UART, redundant storage |
| `radcamctl dose` | one dosimeter reading |
| `radcamctl dose --watch` | continuous readings |
| `radcamctl calibrate` | establish the dosimeter zero point |
| `radcamctl led <percent>` | set illumination; clamped to 10% |
| `radcamctl eeprom` | inspect the camera calibration EEPROM |
| `systemctl status radcamd` | is the daemon running |
| `sudo systemctl restart radcamd` | restart it |
| `journalctl -u radcamd -f` | follow the log |
| `sudo bash tools/install-radcam.sh` | install source changes to `/opt/radcam` |

### Camera bring-up

| Command | Does |
|---|---|
| `bash tools/verify-camera.sh` | full stack check, bottom-up |
| `bash tools/scan-camera.sh` | assert enables, scan both camera I2C buses |
| `bash tools/sweep-enable.sh` | try every enable-pin combination on both ports |
| `bash tools/ar1335-autodetect.sh` | probe for the sensor's I2C address |
| `bash tools/boot-camera-diag.sh` | post-boot camera diagnostics |
| `bash tools/csi-presence.sh` | I2C pull-up probe (inconclusive by nature) |
| `rpicam-hello --list-cameras` | what libcamera can see |
| `rpicam-still -o out.jpg` | take a picture |
| `media-ctl -p -d /dev/media0` | dump the media graph |
| `i2cdetect -y 4` / `-y 6` | scan CAM1 / CAM0 I2C bus |
| `pinctrl get 35` | read a GPIO's state |

### Calibration

| Command | Does |
|---|---|
| `tools/focus.py --http PORT` | live focus meter over HTTP |
| `tools/focus.py --preview` | live focus meter in the terminal |
| `tools/chart-framing.py` | score chart framing and focus before calibrating |
| `tools/calibrate-camera.py --camera-id X --apply` | colour: glare, white balance and matrix together |
| `tools/calibrate-camera.py --from-capture F.jpg` | solve from a saved frame |
| `tools/calibrate-camera.py --centres F.json` | supply patch centres by hand |
| `tools/calibrate-camera.py --show` | show stored calibration |
| `tools/calibrate-camera.py --erase` | clear stored calibration |
| `tools/calibrate-shading.py --store --apply` | lens shading; needs a flat field |
| `tools/calibrate-distortion.py --store` | radial distortion from straight edges |
| `tools/calibrate-distortion.py --undistort IN OUT` | correct an image |
| `tools/calibrate-response.py --store` | exposure/gain linearity, saturation |
| `tools/find-crop.py --store` | detect and store the usable frame region |
| `tools/measure-awb.py` | raw R/G and B/G from a grey target |
| `tools/fit-awb-curve.py --anchor` | build the AWB colour-temperature curve |
| `tools/measure-noise.py` | sensor noise characterisation |
| `tools/calib-box-spec.py --dfov D` | dimension the calibration fixture |
| `tools/bench-compression.py` | measure encode time and size per profile |

### Flight link

| Command | Does |
|---|---|
| `tools/stp-sim.py --self-test` | full protocol conversation, no hardware |
| `tools/stp-sim.py --port DEV --ping` | liveness check over a serial port |
| `tools/stp-sim.py --port DEV --lrt` | poll telemetry and print it |
| `tools/stp-sim.py --port DEV --capture` | take a picture |
| `tools/stp-sim.py --port DEV --download ID` | fetch a file over HRT |
| `tools/stp-sim.py --stream-start` / `--stream-stop` | start or stop live video |
| `tools/stp-sim.py --stream-size WxH@FPS:KBPS` | e.g. `640x480@15:600` |
| `tools/stp-sim.py --stream-region CX,CY,W,H` | crop centred on a sensor pixel |
| `tools/stp-sim.py --watch-stream N --stream-out F` | receive live video for N s and save it |
| `tools/stp-de-timing.py --throughput` | DE assertion timing and link throughput |
| `tools/stp-crc-solve.py --bin FILE` | recover CRC parameters from captured traffic |
| `tools/stp-crc-solve.py --demo` | prove the solver works |
| `tools/stp-verify.py` | 81 reliability, safety and autonomy checks |
| `tools/stp-verify.py --suite safety` | one suite only |

### Protocol opcodes — what the spacecraft can ask for

Sent in the 105-byte command payload. Arguments are little-endian.

| Opcode | Name | Arguments | Effect |
|---|---|---|---|
| `0x01` | `PING` | — | liveness; returns uptime and version |
| `0x10` | `SET_CONFIG` | key `u8`, value `u32`, repeating | change configuration |
| `0x11` | `GET_CONFIG` | — | read the effective configuration |
| `0x20` | `GET_TELEMETRY` | — | housekeeping snapshot |
| `0x21` | `GET_MEDIA_LIST` | — | stored media: id, type, size, resolution |
| `0x22` | `GET_DOSE_LOG` | start `f64`, end `f64` (optional) | dose history |
| `0x30` | `CAPTURE_IMAGE` | — | take one still |
| `0x31` | `START_RECORD` | — | begin video |
| `0x32` | `STOP_RECORD` | — | end recording |
| `0x33` | `CAPTURE_REGION` | x, y, w, h, out_w, out_h — all `u16` | full-res crop, rescaled |
| `0x40` | `REQUEST_MEDIA` | media id `u32` | queue a file for HRT |
| `0x41` | `RESEND` | media id `u32`, chunk indices `u32…` | re-send HRT chunks |
| `0x42` | `DELETE_MEDIA` | media id `u32` | free storage |
| `0x50` | `SET_LED` | percent `u8` | illumination; clamped to 10% |
| `0x60` | `EEPROM_READ` | offset `u16`, length `u16` | read calibration bytes |
| `0x61` | `EEPROM_WRITE` | offset `u16`, length `u16`, data | write; refused unless unlocked |
| `0x62` | `EEPROM_STATUS` | — | which redundant copies still verify |
| `0x63` | `EEPROM_REPAIR` | — | rewrite all copies from a good one |
| `0x70` | `CLEAR_SAFE_MODE` | — | resume normal operation |
| `0x71` | `ABORT_TRANSFERS` | — | empty the HRT queue |
| `0x72` | `GET_LINK_STATS` | — | receive counters and resync counts |
| `0x73` | `SET_HRT_IDLE_FILL` | enable `u8` | send idle HRT packets when nothing is queued |
| `0x77` | `SET_FEC_GROUP` | group size `u8` | parity group size for HRT transfers; 0 disables |
| `0x78` | `STREAM_START` | optionally w `u16`, h `u16`, fps `u8`, bitrate `u32`, then cx, cy, cw, ch `u16` | start live video |
| `0x79` | `STREAM_STOP` | — | stop live video |
| `0x7A` | `STREAM_SET_REGION` | centre_x, centre_y, crop_w, crop_h — all `u16` | aim the crop box at a sensor pixel |
| `0x7B` | `STREAM_SET_OUTPUT` | width `u16`, height `u16`, fps `u8`, bitrate `u32` | stream resolution and rate |

All four stream commands reply with the **effective** settings after clamping:
width, height, fps, bitrate, centre_x, centre_y, crop_w, crop_h.

### Packet types on the wire

| Type | Direction | Size | Meaning |
|---|---|---:|---|
| `0x10` | DICE → payload | 120 | Command |
| `0x10` | payload → DICE | 8 | Command acknowledge |
| `0x81` | DICE → payload | 14 | LRT request |
| `0x81` | payload → DICE | 1256 | LRT data |
| `0x85` | DICE → payload | 14 | HRT stop |
| `0x86` | DICE → payload | 14 | HRT stop with loss |
| `0x87` | DICE → payload | 14 | HRT go |
| `0x87` | payload → DICE | 1288 | HRT data |

> Packet type alone never identifies a packet — `0x10`, `0x81` and `0x87` each
> mean two different things. Direction and length together disambiguate.

### HRT payload sub-types

The first two bytes of every HRT payload say what it is.

| Sub-type | Name | Meaning |
|---|---|---|
| `0x0000` | `IDLE` | filler, only when idle fill is enabled |
| `0x0001` | `MEDIA_INFO` | file id, size, chunk count, whole-file CRC-32 |
| `0x0002` | `MEDIA_DATA` | one numbered chunk of a file |
| `0x0003` | `MEDIA_END` | file finished, with its CRC-32 |
| `0x0004` | `MEDIA_PARITY` | XOR parity for the group named by `chunk_index` |
| `0x0005` | `STREAM_DATA` | one chunk of a live video frame |

### Error codes

Reported as `last_result` in telemetry. `0` means success.

| Code | Name | Meaning |
|---|---|---|
| 1 | `BAD_CRC` | checksum failed |
| 2 | `BAD_TYPE` | unknown opcode |
| 3 | `BAD_PARAM` | malformed arguments |
| 4 | `BUSY` | queue full, already recording, or in safe mode |
| 5 | `NO_MEDIA` | no such file |
| 6 | `CAMERA_FAULT` | camera unavailable |
| 7 | `STORAGE_FULL` | no space |
| 8 | `BITRATE_EXCEEDS_LINK` | video settings exceed the link budget |
| 9 | `NOT_CALIBRATED` | calibration required first |
| 10 | `EEPROM_FAULT` | calibration memory error |
| 11 | `REGION_INVALID` | crop outside the sensor |
| 12 | `WRITE_PROTECTED` | EEPROM writes are locked |
