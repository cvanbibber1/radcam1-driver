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
  from DICE, and **starts closed** after every reset — nothing bulk is
  transmitted until the master explicitly asks.

The two stops differ only in what happens to a packet already going out:

| | Effect |
|---|---|
| `HRT_STOP` (0x85) | lets the packet in flight finish, then sends no more |
| `HRT_STOP_WITH_LOSS` (0x86) | cuts the transmission short — the receiver sees a truncated frame, fails its CRC and discards it |

Neither buffers anything. After a stop-with-loss, a truncated **file** chunk is
sent again when the tap reopens; a truncated **video** frame is abandoned,
because by then it is stale.

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

**600 kbit/s at 640×480 and 15 fps is close to the ceiling.** Measured link
occupancy is 92% of every frame interval, leaving 8% for telemetry and
commands. Keyframes burst to 231% of one interval — about 2.3 frames' worth —
which the frame ring absorbs and then catches up on, because the mean is below
100%. Asking for 700 kbit/s at that size and rate exceeds the link outright
(105%) and simply produces dropped frames. For more margin, drop to 10 fps at
the same bitrate (84%) or lower the bitrate.

Check any combination before committing to it:

```bash
python3 tools/stp-metrics.py                 # arithmetic, no hardware needed
sudo systemctl stop radcamd
sudo python3 tools/stp-metrics.py --all      # measured link and encoder
sudo systemctl start radcamd
```

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

### Storage slots

Captures do not go to an auto-numbered media store — they go into **numbered
slots**. There are 16 by default, and each holds one image or one video. Slot 3
is slot 3 whatever happened before, which is what makes a canned command
possible: a stored hex string that captures into slot 3 and another that
downloads slot 3 mean the same thing on every pass.

The lifecycle is explicit, and deletion is what frees space:

```
SLOT_CAPTURE_IMAGE slot=3     take a still into slot 3
SLOT_RECORD_START  slot=4     record video into slot 4, held on the Pi
SLOT_RECORD_STOP              finish it
SLOT_LIST                     what is in every slot
SLOT_DOWNLOAD      slot=3     queue it for HRT
SLOT_DELETE        slot=3     free the slot for the next experiment
```

Recording is **not** streaming: `SLOT_RECORD_START` writes video to the Pi's
storage for later transfer, and nothing goes over the link until you download
it. Give it a duration and it stops by itself, so the ground does not have to
be in contact:

```
SLOT_RECORD_START slot=4 seconds=30
```

Every slot carries a CRC-32 taken when it was written and checked when it is
read, so a file that rotted in storage is reported as a fault rather than
discovered after minutes of downlink. Telemetry reports slots used, slots free,
bytes held, which slot is recording and which is downloading.

A recording interrupted by a power loss leaves its slot **free**, not
half-occupied — nothing was finalised, so there is nothing to keep.

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
| `stp.de_control` | **false when DE is tied active in hardware** — releases the pin instead of driving it |
| `stp.big_endian` | `true` |
| `stp.crc_variant` | `"CRC-16/CCITT-FALSE"` |
| `stp.fec_group_size` | chunks per parity chunk on HRT transfers; 0 disables |
| `stream.width`, `stream.height` | stream output size, default 640x480 |
| `stream.fps`, `stream.bitrate` | default 15 fps, 600000 bit/s |
| `stream.centre_x`, `stream.centre_y` | sensor pixel the crop box is centred on |
| `stream.crop_w`, `stream.crop_h` | crop size; equal to output means native pixels |
| `stream.queue_frames` | frames buffered before the oldest is dropped |
| `slot_count` | storage slots, default 16 |
| `slot_dir` | where slot contents live, default `/var/lib/radcam/slots` |

> With `stp.enabled` true, the flight port belongs to the RS-422 link and the
> readable ASCII beacon goes only to the debug mirror on GPIO23/24. This is
> deliberate: an unsolicited byte on a shared bus corrupts another experiment's
> reply.

---

## 8. Testing and verification

```bash
python3 -m unittest discover -s tests -t .   # 217 tests, ~19 s
python3 tools/stp-verify.py                  # 85 reliability/safety/autonomy checks
python3 tools/stp-sim.py --self-test         # 21 protocol checks
python3 tools/stp-command.py --verify        # 39 command hex strings
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
| No free slots | download what you need, then `SLOT_DELETE` — deletion is what frees space |
| Slot download refused with code 10 | the stored file failed its CRC-32; the bytes rotted, recapture |
| A canned command runs only once | it was generated with `--no-force`; regenerate without that flag |
| Link completely silent both ways | check DE wiring first — if the board ties DE active, set `"de_control": false` or software will hold the transmitter off |
| Bytes arrive but are all `0xFF` | **inverted pair.** Swap the two wires of that pair — see `docs/reference/RS422_BRINGUP.md` §7. Confirm first with `tools/stp-pingpong.py --polarity` |
| Bytes arrive but are all `0x00` | inverted pair the other way, or a line stuck low |
| Payload receives nothing, `fe=0` | no signal reaching GPIO15 at all; a baud mismatch or reversed pair gives framing errors, not silence |
| Need to prove the link electrically | `tools/stp-pingpong.py --scope` transmits a continuous square wave to probe |
| Colours wrong, greys fine | something spectral, not gain — check for filters or tape over the sensor |
| `safe_mode` set in telemetry | five commands failed in a row; check events, then `CLEAR_SAFE_MODE` |
| Link works then stops | check `rx_bad_crc` and `rx_resyncs` in telemetry |

Safe mode deliberately keeps answering telemetry while refusing other work: a
payload that has gone quiet is indistinguishable from a dead one.

---

## 10. Every command

### Typical pass

```
LRT_REQUEST                    # vitals: dose, temperature, slots, link health
SLOT_CAPTURE_IMAGE slot=0      # take a picture
SLOT_LIST                      # confirm it landed, see its size
SLOT_DOWNLOAD slot=0           # queue it
HRT_GO                         # open the tap, receive it
HRT_STOP                       # close the tap
SLOT_DELETE slot=0             # free the slot for next time
```

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
| `tools/stp-command.py --list` | every command that exists |
| `tools/stp-command.py --all` | every command as a pasteable hex string |
| `tools/stp-command.py NAME arg=val` | one command with your own arguments |
| `tools/stp-command.py --verify` | prove every generated string is accepted |
| `tools/stp-metrics.py` | link arithmetic, transfer times, stream budget |
| `tools/stp-metrics.py --all` | plus measured UART and encoder performance |
| `tools/stp-rxdiag.py --scan` | why is nothing arriving: silent, garbled, or wrong target |
| `tools/stp-beacon.py --hrt` | transmit LRT/HRT unsolicited to test a host's receiver |
| `tools/stp-pingpong.py --raw-ping` | plain ASCII any terminal can see |
| `tools/stp-pingpong.py --loopback` | prove the payload's whole path with a jumper |
| `tools/stp-pingpong.py --scope` | continuous pattern for oscilloscope probing |
| `tools/stp-pingpong.py --polarity` | alternate 0x00/0xFF to detect an inverted pair |
| `tools/stp-pingpong.py --dc-test` | connectivity without needing correct framing |
| `tools/stp-pingpong.py --tx-sweep` | transmit at every baud in turn, slowest last |
| `tools/stp-verify.py` | 85 reliability, safety and autonomy checks |
| `tools/stp-verify.py --suite safety` | one suite only |

### Command hex strings

Every command is a complete packet — sync, timestamp, type, target, payload and
CRC — as one hex string you can paste straight into the ground configuration.
Regenerate any of them, with your own argument values:

```bash
python3 tools/stp-command.py --list                  # what exists
python3 tools/stp-command.py --all                   # every one, with hex
python3 tools/stp-command.py SLOT_CAPTURE_IMAGE slot=3
python3 tools/stp-command.py --verify                # prove they are accepted
```

Two things about a canned string, both deliberate:

- It carries the **force flag**. The sequence number is baked in, and the
  payload otherwise treats a repeated sequence number as a retransmission and
  suppresses it — so without the flag, pasting the same capture command twice
  would take one picture. With it, ten pastes take ten pictures. Pass
  `--no-force` if your ground station generates its own sequence numbers and
  you want retransmission protection instead.
- Its **timestamp is zero**. Coarse and fine time belong to the master; the
  payload only echoes them back, and nothing it does depends on the value.

`tools/stp-command.py --verify` feeds all 39 strings to a real decoder and
confirms each is accepted and dispatched to the opcode it claims — worth
running after any protocol change, before trusting the table below.

Target ID `0xC7`, CRC-16/CCITT-FALSE, big-endian envelope, `cmd_seq` 1, force flag set.

### Request packets (14 bytes)

| Name | Type | Purpose | Hex |
|---|---|---|---|
| `LRT_REQUEST` | `0x81` | Poll telemetry; the payload replies with one LRT Data packet | `1ACFFC1D00000000000081C7B03C` |
| `HRT_GO` | `0x87` | Open the HRT tap - nothing bulk is transmitted until this arrives | `1ACFFC1D00000000000087C71A9A` |
| `HRT_STOP` | `0x85` | Close the tap, letting the packet in flight finish | `1ACFFC1D00000000000085C77CF8` |
| `HRT_STOP_WITH_LOSS` | `0x86` | Close the tap immediately, truncating any packet in flight | `1ACFFC1D00000000000086C729AB` |

### Imaging commands

| Name | Opcode | Arguments | Purpose |
|---|---|---|---|
| `CAPTURE_IMAGE` | `0x30` | — | Capture into the media store (prefer SLOT_CAPTURE_IMAGE) |
| `START_RECORD` | `0x31` | — | Record into the media store (prefer SLOT_RECORD_START) |
| `STOP_RECORD` | `0x32` | — | End a media-store recording |
| `CAPTURE_REGION` | `0x33` | `x`, `y`, `w`, `h`, `out_w`, `out_h` | Full-resolution capture cropped to a window and rescaled |

<details><summary>Hex strings (defaults shown)</summary>

`CAPTURE_IMAGE` — no arguments

```
1ACFFC1D00000000000010C730000100013AF30000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

`START_RECORD` — no arguments

```
1ACFFC1D00000000000010C7310001000190A20000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

`STOP_RECORD` — no arguments

```
1ACFFC1D00000000000010C732000100017E700000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

`CAPTURE_REGION` — x=1784, y=1320, w=640, h=480, out_w=640, out_h=480

```
1ACFFC1D00000000000010C73300010C01E69FF80628058002E0018002E001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000CCB4
```

</details>

### Slots commands

| Name | Opcode | Arguments | Purpose |
|---|---|---|---|
| `SLOT_LIST` | `0x64` | — | Every slot: kind, size, dimensions, CRC-32, timestamp |
| `SLOT_INFO` | `0x65` | `slot` | One slot in detail |
| `SLOT_CAPTURE_IMAGE` | `0x66` | `slot` | Take a still into a slot, overwriting it |
| `SLOT_RECORD_START` | `0x67` | `slot`, `seconds` | Record video into a slot; held on the Pi, not streamed |
| `SLOT_RECORD_STOP` | `0x68` | — | End the recording and finalise its slot |
| `SLOT_DELETE` | `0x6A` | `slot` | Free one slot for reuse |
| `SLOT_DELETE_ALL` | `0x6B` | — | Free every slot |

<details><summary>Hex strings (defaults shown)</summary>

`SLOT_LIST` — no arguments

```
1ACFFC1D00000000000010C76400010001A6C70000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

`SLOT_INFO` — slot=0

```
1ACFFC1D00000000000010C7650001010160BC000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000AAE4
```

`SLOT_CAPTURE_IMAGE` — slot=0

```
1ACFFC1D00000000000010C76600010101AE5C000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000841D
```

`SLOT_RECORD_START` — slot=1, seconds=30

```
1ACFFC1D00000000000010C76700010301B114011E00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000E526
```

`SLOT_RECORD_STOP` — no arguments

```
1ACFFC1D00000000000010C768000100012DEC0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

`SLOT_DELETE` — slot=0

```
1ACFFC1D00000000000010C76A00010101A5BF0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003FF9
```

`SLOT_DELETE_ALL` — no arguments

```
1ACFFC1D00000000000010C76B00010001C33E0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

</details>

### Stream commands

| Name | Opcode | Arguments | Purpose |
|---|---|---|---|
| `STREAM_START` | `0x78` | `width`, `height`, `fps`, `bitrate`, `centre_x`, `centre_y`, `crop_w`, `crop_h` | Start live video over HRT |
| `STREAM_STOP` | `0x79` | — | Stop live video |
| `STREAM_SET_OUTPUT` | `0x7B` | `width`, `height`, `fps`, `bitrate` | Resolution, frame rate and bitrate |
| `STREAM_SET_REGION` | `0x7A` | `centre_x`, `centre_y`, `crop_w`, `crop_h` | Aim the crop box at a sensor pixel |

<details><summary>Hex strings (defaults shown)</summary>

`STREAM_START` — width=640, height=480, fps=15, bitrate=600000, centre_x=2104, centre_y=1560, crop_w=640, crop_h=480

```
1ACFFC1D00000000000010C7780001110172308002E0010FC0270900380818068002E001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000004CFE
```

`STREAM_STOP` — no arguments

```
1ACFFC1D00000000000010C7790001000183E70000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

`STREAM_SET_OUTPUT` — width=640, height=480, fps=15, bitrate=600000

```
1ACFFC1D00000000000010C77B00010901AB168002E0010FC027090000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000023DA
```

`STREAM_SET_REGION` — centre_x=2104, centre_y=1560, crop_w=640, crop_h=480

```
1ACFFC1D00000000000010C77A00010801139E380818068002E00100000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000D9CE
```

</details>

### Transfer commands

| Name | Opcode | Arguments | Purpose |
|---|---|---|---|
| `SLOT_DOWNLOAD` | `0x69` | `slot` | Queue a slot for HRT transfer |
| `SLOT_DOWNLOAD_ABORT` | `0x6C` | `slot` | Remove a slot from the transfer queue |
| `ABORT_TRANSFERS` | `0x71` | — | Empty the whole HRT transfer queue |
| `RESEND` | `0x41` | `media_id`, `chunk` | Re-send specific chunks of a transfer |
| `GET_MEDIA_LIST` | `0x21` | — | Legacy media store listing |
| `REQUEST_MEDIA` | `0x40` | `media_id` | Queue a legacy media id for HRT |
| `DELETE_MEDIA` | `0x42` | `media_id` | Delete legacy media |

<details><summary>Hex strings (defaults shown)</summary>

`SLOT_DOWNLOAD` — slot=0

```
1ACFFC1D00000000000010C769000101016B5F0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001100
```

`SLOT_DOWNLOAD_ABORT` — slot=0

```
1ACFFC1D00000000000010C76C00010101285E000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000620B
```

`ABORT_TRANSFERS` — no arguments

```
1ACFFC1D00000000000010C7710001000181CA0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

`RESEND` — media_id=1358954496, chunk=0

```
1ACFFC1D00000000000010C74100010801F1FD000000510000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000A3C6
```

`GET_MEDIA_LIST` — no arguments

```
1ACFFC1D00000000000010C7210001000194F80000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

`REQUEST_MEDIA` — media_id=1

```
1ACFFC1D00000000000010C74000010401258E01000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000017D4
```

`DELETE_MEDIA` — media_id=1

```
1ACFFC1D00000000000010C74200010401E3E901000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000041CC
```

</details>

### Link commands

| Name | Opcode | Arguments | Purpose |
|---|---|---|---|
| `CLEAR_SAFE_MODE` | `0x70` | — | Resume normal operation after safe mode |

<details><summary>Hex strings (defaults shown)</summary>

`CLEAR_SAFE_MODE` — no arguments

```
1ACFFC1D00000000000010C770000100012B9B0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

</details>

### Config commands

| Name | Opcode | Arguments | Purpose |
|---|---|---|---|
| `GET_CONFIG` | `0x11` | — | Read the effective configuration |
| `SET_CONFIG` | `0x10` | `key`, `value` | Set one configuration key |
| `SET_LED` | `0x50` | `percent` | Illumination percent; clamped to 10 |
| `SET_FEC_GROUP` | `0x77` | `group` | Parity group size for HRT transfers; 0 disables |
| `SET_HRT_IDLE_FILL` | `0x73` | `enable` | Send idle HRT packets when nothing is queued |

<details><summary>Hex strings (defaults shown)</summary>

`GET_CONFIG` — no arguments

```
1ACFFC1D00000000000010C7110001000198160000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

`SET_CONFIG` — key=6, value=2

```
1ACFFC1D00000000000010C710000105012E96060200000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000BA3E
```

`SET_LED` — percent=5

```
1ACFFC1D00000000000010C750000101015C9405000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000011D4
```

`SET_FEC_GROUP` — group=16

```
1ACFFC1D00000000000010C77700010101E34910000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000032CB
```

`SET_HRT_IDLE_FILL` — enable=0

```
1ACFFC1D00000000000010C77300010101F7D90000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000004247
```

</details>

### Telemetry commands

| Name | Opcode | Arguments | Purpose |
|---|---|---|---|
| `PING` | `0x01` | — | Liveness check; replies with uptime and version |
| `GET_TELEMETRY` | `0x20` | — | Housekeeping snapshot in the LRT response window |
| `GET_LINK_STATS` | `0x72` | — | Receive counters, CRC failures and resync counts |
| `GET_DOSE_LOG` | `0x22` | — | Dose history; zero range means everything |

<details><summary>Hex strings (defaults shown)</summary>

`PING` — no arguments

```
1ACFFC1D00000000000010C701000100019C4C0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

`GET_TELEMETRY` — no arguments

```
1ACFFC1D00000000000010C720000100013EA90000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

`GET_LINK_STATS` — no arguments

```
1ACFFC1D00000000000010C772000100016F180000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

`GET_DOSE_LOG` — no arguments

```
1ACFFC1D00000000000010C722000100017A2A0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

</details>

### Eeprom commands

| Name | Opcode | Arguments | Purpose |
|---|---|---|---|
| `EEPROM_STATUS` | `0x62` | — | Which redundant calibration copies still verify |
| `EEPROM_REPAIR` | `0x63` | — | Rewrite every copy from a surviving one |
| `EEPROM_READ` | `0x60` | `offset`, `length` | Read calibration bytes |

<details><summary>Hex strings (defaults shown)</summary>

`EEPROM_STATUS` — no arguments

```
1ACFFC1D00000000000010C762000100016B420000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

`EEPROM_REPAIR` — no arguments

```
1ACFFC1D00000000000010C76300010001C1130000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003810
```

`EEPROM_READ` — offset=0, length=128

```
1ACFFC1D00000000000010C76000010401EF5E000080000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000D902
```

</details>


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
