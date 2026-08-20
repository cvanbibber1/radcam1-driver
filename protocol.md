# radcam1 Ground ↔ Payload Protocol

Version 1.0 — 2026-08-10

Bidirectional, point-to-point link between the ground station and a single
radcam1 payload. There are no other devices on the bus, so there is no
addressing and no arbitration.

| Property | Value |
|---|---|
| Physical (flight) | RS422 via ADM2582E, GPIO14 TX / GPIO15 RX, `/dev/ttyAMA0` |
| Physical (debug) | **RP1 PIO** hardware-timed UART, GPIO24 TX / GPIO23 RX |
| Baud | **921600** on both links, 8N1, no flow control |
| Framing | COBS, `0x00` as frame delimiter |
| Integrity | CRC-32 per frame |
| Reliability | selective-repeat ARQ (the link is bidirectional, so retransmit beats FEC) |
| Byte order | little-endian |

Both ports carry identical traffic. The debug port is a mirror: the payload
transmits on both and accepts commands from either.

---

## 1. Link budget — read this before choosing compression

Everything downstream depends on one number.

```
921600 baud, 8N1 = 10 bits per byte
    921600 / 10                     =  92,160 bytes/s   raw
    minus COBS (~0.5%), header+CRC
    (12 B per 1024 B payload, 1.2%),
    minus ACK turnaround (~2%)
                                    ≈  88,000 bytes/s   usable
                                    ≈  86 kB/s  =  704 kbit/s
```

> ⚠️ **The payload is not currently running at 921600 on the flight link.**
> `DEFAULT_BAUD` in `radcam/telemetry.py` is 115200 and `/etc/radcam/config.json`
> does not override `flight_baud`, so the real throughput is about 11 kB/s and
> **every size/time figure below is 8x optimistic** — a 2.4 MB still takes 3.6
> minutes, not 27 seconds. `LINK_BYTES_PER_S = 88_000` in `radcam/protocol.py`
> also feeds the `BITRATE_EXCEEDS_LINK` check, so video the payload accepts as
> transmittable currently is not. Raising the baud needs the other end to match,
> which is a mission decision — see HANDOFF.md section 9.1. Resolve before
> trusting any number in this section.

**Use 88 kB/s for all planning.** At 1% packet loss, budget 89 kB/s of offered
load to net 88 kB/s delivered.

### 1.1 Still images — measured

Measured on this Pi 5 with `tools/bench-compression.py`. Sizes come from a
synthetic frame with realistic spatial-frequency content, so treat them as
**indicative**; encode times are properties of this CPU and are directly
usable. Re-run against real AR1335 frames once the sensor responds.

| Resolution | Profile | Size | Encode | Transmit |
|---|---|---:|---:|---:|
| 4096×3072 | `PNG_LOSSLESS` | 17.59 MB | 4.96 s | **3 min 20 s** |
| 4096×3072 | `JPEG_HIGH` (q92) | 4.95 MB | 0.24 s | 56 s |
| 4096×3072 | `JPEG_MED` (q80) | 3.00 MB | 0.17 s | 34 s |
| 4096×3072 | `JPEG_LOW` (q60) | 1.61 MB | 0.13 s | 18 s |
| 4096×3072 | `JPEG_TINY` (q40) | 987 KB | 0.12 s | 11 s |
| 1920×1080 | `PNG_LOSSLESS` | 3.09 MB | 0.70 s | 35 s |
| 1920×1080 | `JPEG_HIGH` (q92) | 822 KB | 0.04 s | 9.3 s |
| 1920×1080 | `JPEG_MED` (q80) | 500 KB | 0.03 s | 5.7 s |
| 1920×1080 | `JPEG_LOW` (q60) | 269 KB | 0.02 s | 3.1 s |
| 1920×1080 | `JPEG_TINY` (q40) | 167 KB | 0.02 s | 1.9 s |

Encode time is negligible next to transmit time in every case except
`PNG_LOSSLESS`, and even there the 5 s encode is dwarfed by a 3 min 20 s
downlink. **The link, not the CPU, is the bottleneck for stills.**

PNG at full resolution costs over three minutes of link time per frame. It
exists for a lossless reference frame, not for routine downlink.

### 1.2 Video — the hard constraint

The link carries **704 kbit/s**. Encoded video must fit inside that, or the
backlog grows without bound.

Measured with `tools/bench-compression.py` on 1080p30 with real inter-frame
motion. Transmit ratio is computed from the *configured* bitrate, not from the
test clip's size: a short synthetic clip never fully exercises the rate
controller, so its size understates sustained real-world output.

| Profile | Bitrate | Achieved on test clip | Encode | Transmit | Verdict |
|---|---:|---:|---:|---:|---|
| `H264_STREAM` | 400 kbit/s | 353 kbit/s | 0.88× realtime | **0.57×** | sustainable continuously |
| `H264_LOW` | 600 kbit/s | 563 kbit/s | 0.94× realtime | **0.85×** | sustainable continuously |
| `H264_MED` | 1.5 Mbit/s | 1577 kbit/s | 0.90× realtime | 2.13× | finite clips only |
| `H264_HIGH` | 4 Mbit/s | 3472 kbit/s | 0.89× realtime | 5.68× | finite clips only |

> **Rule: `video_duration_s = 67` (infinite) is only valid with
> `H264_STREAM` or `H264_LOW`.** Any higher bitrate accumulates backlog
> forever and the payload refuses the combination with
> `ERR_BITRATE_EXCEEDS_LINK`.

#### The Pi 5 has no hardware H.264 encoder

Unlike the Pi 4, the Pi 5 dropped the hardware H.264 encoder — there is no
`/dev/video11`. All video is encoded in **software by libx264**, and the
measurements above show 1080p30 encoding at **0.88–0.94× realtime**.

That is only 6–12% of margin. Consequences worth designing around:

* Continuous 1080p30 recording consumes most of a core continuously, which is
  a meaningful power cost on a payload that is supposed to be power-minimised.
* If the CPU throttles thermally, or another task competes, encoding drops
  below realtime and frames are lost. Continuous recording should be treated
  as **CPU-limited, not just link-limited**.
* If sustained continuous recording matters, consider dropping to 1080p15 or
  720p, which roughly halves the encode load and leaves genuine headroom.

Finite clips at higher bitrates are fine — they simply take longer to downlink
than the recording did. The payload reports the estimate up front in
`MEDIA_INFO` so the ground station knows what it has committed to before the
transfer starts.

---

## 2. Frame format

Every frame is COBS-encoded, then terminated with a single `0x00`. COBS removes
all zero bytes from the body, so `0x00` is an unambiguous frame delimiter and
the receiver resynchronises after any corruption by simply scanning to the next
zero.

Decoded frame layout:

```
 offset  size  field
      0     1  version        always 0x01
      1     1  type           see section 3
      2     2  seq            sender's sequence number, wraps at 0xFFFF
      4     2  payload_len    0 - 1024
      6     N  payload
    6+N     4  crc32          over bytes [0 .. 6+N-1], IEEE 802.3
```

Maximum decoded frame: 1034 bytes. Maximum on-wire after COBS: 1040 bytes.

A frame failing CRC is discarded silently. The sender learns of it through the
absence of an ACK, and retransmits.

### 2.1 Sequence numbers and ACK

- Every command from the ground carries a fresh `seq`.
- Every response echoes the `seq` it answers.
- Unsolicited payload frames (telemetry beacons, `MEDIA_DATA`) use the payload's
  own sequence space.
- Commands are idempotent where possible: re-sending a command with a `seq`
  already seen returns the cached response rather than acting twice. This makes
  a lost ACK harmless.

---

## 3. Message types

Commands are `0x00-0x7F` (ground → payload); responses are `0x80-0xFF`
(payload → ground). A response type is normally the command type with bit 7 set.

| Type | Name | Direction | Meaning |
|---|---|---|---|
| `0x01` | `PING` | → | liveness check |
| `0x81` | `PONG` | ← | uptime, firmware version |
| `0x10` | `SET_CONFIG` | → | apply configuration (section 4) |
| `0x90` | `CONFIG_ACK` | ← | full effective config after clamping |
| `0x11` | `GET_CONFIG` | → | read configuration |
| `0x91` | `CONFIG_REPORT` | ← | full effective config |
| `0x20` | `GET_TELEMETRY` | → | request housekeeping |
| `0xA0` | `TELEMETRY` | ← | dose, LED, temp, storage, errors |
| `0x21` | `GET_MEDIA_LIST` | → | list stored media |
| `0xA1` | `MEDIA_LIST` | ← | id, type, size, resolution, timestamp |
| `0x22` | `GET_DOSE_LOG` | → | dose history, optional time range |
| `0xA2` | `DOSE_LOG` | ← | timestamped dose records |
| `0x30` | `CAPTURE_IMAGE` | → | take one still (fires flash if configured) |
| `0xB0` | `CAPTURE_ACK` | ← | new media id, or error |
| `0x31` | `START_RECORD` | → | begin video per config |
| `0xB1` | `RECORD_ACK` | ← | media id, started/failed |
| `0x32` | `STOP_RECORD` | → | end an in-progress recording |
| `0xB2` | `RECORD_DONE` | ← | media id, duration, size |
| `0x40` | `REQUEST_MEDIA` | → | begin transfer of a media id |
| `0xC0` | `MEDIA_INFO` | ← | size, chunk count, **estimated transfer time** |
| `0xC1` | `MEDIA_DATA` | ← | one chunk (see 5.1) |
| `0xC2` | `MEDIA_END` | ← | whole-file CRC-32 |
| `0x41` | `RESEND` | → | request specific chunks again |
| `0x42` | `DELETE_MEDIA` | → | free storage |
| `0xC3` | `DELETE_ACK` | ← | |
| `0x33` | `CAPTURE_REGION` | → | full-res capture, cropped to a window, rescaled (section 8) |
| `0x50` | `SET_LED` | → | manual LED override (still capped at 10%) |
| `0xD0` | `LED_ACK` | ← | applied value |
| `0x60` | `EEPROM_READ` | → | read raw calibration bytes (section 9) |
| `0xE0` | `EEPROM_DATA` | ← | offset, length, bytes |
| `0x61` | `EEPROM_WRITE` | → | write raw bytes; refused unless unlocked |
| `0xE1` | `EEPROM_WRITE_ACK` | ← | offset, length, **verified** flag |
| `0x62` | `EEPROM_STATUS` | → | which redundant copies still pass CRC |
| `0xE2` | `EEPROM_STATUS_REPORT` | ← | copies total, copies good, per-copy flags |
| `0x63` | `EEPROM_REPAIR` | → | rewrite all copies from a surviving one |
| `0xE3` | `EEPROM_REPAIR_ACK` | ← | whether anything was repaired |
| `0xEE` | `NACK` | ← | error code + the `seq` that failed |

### 3.1 `NACK` error codes

| Code | Name | Meaning |
|---:|---|---|
| 1 | `ERR_BAD_CRC` | frame integrity failed |
| 2 | `ERR_BAD_TYPE` | unknown message type |
| 3 | `ERR_BAD_PARAM` | parameter outside acceptable range |
| 4 | `ERR_BUSY` | recording or transfer already in progress |
| 5 | `ERR_NO_MEDIA` | unknown media id |
| 6 | `ERR_CAMERA_FAULT` | sensor not responding |
| 7 | `ERR_STORAGE_FULL` | no space |
| 8 | `ERR_BITRATE_EXCEEDS_LINK` | infinite video at an untransmittable bitrate |
| 9 | `ERR_NOT_CALIBRATED` | dosimeter baseline missing |
| 10 | `ERR_EEPROM_FAULT` | EEPROM absent or the I2C transfer failed |
| 11 | `ERR_REGION_INVALID` | crop window outside the sensor, or zero-sized |
| 12 | `ERR_WRITE_PROTECTED` | `EEPROM_WRITE` attempted while locked |

---

## 4. Configuration

`SET_CONFIG` carries a sequence of `(key: u8, value: u32)` pairs. Unknown keys
are ignored, and the payload always replies with the **full effective
configuration after clamping**, so the ground station never has to guess what
was actually applied.

| Key | Name | Range | Default | Clamping |
|---:|---|---|---:|---|
| `0x01` | `flash_percent` | 0 – 10 | 0 | **Anything above 10 becomes 10; anything else invalid becomes 0.** Hard cap, see 4.1 |
| `0x02` | `flash_duration_ms` | 0 – 1000 | 50 | clamped to range |
| `0x03` | `image_resolution` | enum | `RES_4096x3072` | invalid → default |
| `0x04` | `video_resolution` | enum | `RES_1920x1080` | invalid → default |
| `0x05` | `video_duration_s` | 1 – 3600, or **67 = infinite** | 10 | see 4.2 |
| `0x06` | `image_compression` | enum | `JPEG_MED` | invalid → default |
| `0x07` | `video_compression` | enum | `H264_LOW` | invalid → default |
| `0x08` | `telemetry_interval_s` | 1 – 3600 | 5 | clamped |
| `0x09` | `chunk_size` | 256 – 1024 | 1024 | clamped |
| `0x0E` | `undistort` | 0 or 1 | 0 | apply the stored lens model to stills, section 8.3 |

Resolution enum: `0 = RES_4096x3072`, `1 = RES_1920x1080`.

Image compression enum: `0 = PNG_LOSSLESS`, `1 = JPEG_HIGH`, `2 = JPEG_MED`,
`3 = JPEG_LOW`, `4 = JPEG_TINY`.

Video compression enum: `0 = H264_HIGH`, `1 = H264_MED`, `2 = H264_LOW`,
`3 = H264_STREAM`.

### 4.1 Flash and the LED cap

The illumination LEDs are **off at all times** except during a capture, and
only then if `flash_percent > 0`. There is no idle illumination and no way to
leave them on by accident: the daemon drives the PWM to zero on boot, after
every capture, and on shutdown.

`flash_percent` is a percentage of **full scale**, and 10 is the ceiling the
mission allows. The cap is enforced in `radcam/led.py`, which is the only code
permitted to touch the PWM channel; the protocol layer cannot bypass it. Per
the requirement, values above 10 clamp to 10 and any other invalid value
becomes 0.

`SET_LED` (`0x50`) exists for ground-commanded diagnostics and is subject to
exactly the same cap. It is intended for checking the LEDs still work, not for
continuous illumination.

### 4.2 `video_duration_s = 67` means infinite

67 is the sentinel for continuous recording, so **an exactly-67-second clip
cannot be requested** — use 66 or 68. Recording then runs until `STOP_RECORD`,
storage fills, or the payload resets.

Infinite recording is rejected with `ERR_BITRATE_EXCEEDS_LINK` unless
`video_compression` is `H264_LOW` or `H264_STREAM`, for the reason given in
section 1.2.

---

## 5. Media transfer

The flow the mission asked for, end to end:

```
  ground                                payload
    │   SET_CONFIG ────────────────────────►│  clamp, store
    │◄──────────────────────── CONFIG_ACK   │  full effective config
    │   GET_TELEMETRY ─────────────────────►│
    │◄──────────────────────── TELEMETRY    │  dose, LED, temp, storage
    │   CAPTURE_IMAGE ─────────────────────►│  flash on → expose → flash off
    │◄──────────────────────── CAPTURE_ACK  │  media id
    │                                       │  encode per image_compression
    │   GET_MEDIA_LIST ────────────────────►│
    │◄──────────────────────── MEDIA_LIST   │  id, size, resolution, time
    │   REQUEST_MEDIA(id) ─────────────────►│
    │◄──────────────────────── MEDIA_INFO   │  size, chunks, ETA
    │◄──────────────────────── MEDIA_DATA   │  ×N chunks
    │◄──────────────────────── MEDIA_END    │  whole-file CRC-32
    │   RESEND(id, [gaps]) ────────────────►│  only if chunks were missed
    │◄──────────────────────── MEDIA_DATA   │  the missing chunks
    │   DELETE_MEDIA(id) ──────────────────►│
```

Media is captured to local storage first (PNG or MP4), compressed according to
the configuration, and only then transmitted. Capture is never blocked on the
link.

### 5.1 `MEDIA_DATA` payload

```
 offset  size  field
      0     4  media_id
      4     4  chunk_index
      8     2  chunk_len
     10     N  data
```

The frame's own CRC-32 protects each chunk. `MEDIA_END` additionally carries a
CRC-32 over the entire reassembled file, so a chunk that was corrupted *and*
happened to pass its frame CRC is still caught at the end.

### 5.2 Error correction strategy

The link is bidirectional, so **ARQ is used rather than FEC**. Forward error
correction would spend 20-50% of a very scarce link on redundancy that is
usually unnecessary; retransmitting only what was actually lost costs the loss
rate itself.

- Chunks are streamed continuously without per-chunk ACKs (stop-and-wait would
  waste most of the link on turnaround latency).
- The receiver tracks which `chunk_index` values arrived and issues a single
  `RESEND` naming the gaps after `MEDIA_END`.
- Repeat until the whole-file CRC matches. Three failed rounds should be
  reported to the operator rather than retried indefinitely.

At 1% chunk loss a transfer costs ~1% extra; at 10% loss, ~11%. Both are far
cheaper than the ~30% a rate-2/3 FEC would cost on every transfer.

If the return path is ever unavailable — receive-only ground pass — a future
`FEC_ENABLE` config key can add Reed-Solomon parity to `MEDIA_DATA`. It is not
implemented, and is deliberately not the default.

---

## 6. Telemetry and dose logging

`TELEMETRY` (`0xA0`) reports dose in rad, raw ADC volts, calibration baseline,
LED brightness, CPU temperature, free storage, uptime and per-subsystem error
counters.

The dosimeter drifts only over weeks to months, so the mission needs a coarse
time series rather than high-rate sampling. The payload keeps a rolling log of
`(unix_timestamp, dose_rad, volts)` records, written to storage and downloadable
via `GET_DOSE_LOG`, optionally bounded by a time range. Timestamps are "rough"
by design — the payload has no RTC battery guarantee, so records also carry
monotonic uptime, which survives clock resets and lets the ground reconstruct
ordering even if wall-clock time jumps.

Dose is one of the fields transmitted with triple redundancy in the
human-readable housekeeping beacon (see `radcam/telemetry.py`), so a single bit
flip in transit is corrected rather than merely detected.

---

## 7. The debug port: 921600 via PIO (bit-banging could not)

`softuart/` implements the bit-banged UART on GPIO24/23 in C, busy-waiting on
`CLOCK_MONOTONIC` under `SCHED_FIFO` with memory locked. Measured on this Pi 5
over 50,000 samples:

| Baud | Bit period | Mean error | Worst error | Verdict |
|---:|---:|---:|---:|---|
| 9600 | 104.2 µs | 57 ns (0.05%) | 12.5 µs (12%) | **usable** |
| 19200 | 52.1 µs | ~60 ns | ~12.5 µs (24%) | marginal |
| 38400 | 26.0 µs | ~60 ns | 28.7 µs (110%) | unusable |
| 115200 | 8.68 µs | 59 ns (0.68%) | 18.8 µs (216%) | unusable |
| 921600 | 1.085 µs | ~60 ns | 20.5 µs (1888%) | unusable |

The *mean* error is excellent — about 60 ns, or 0.05% of a bit at 9600. The
loop itself is precise. What kills it is the **worst case: 12-20 µs of
preemption**, and that figure is a property of the machine, not of the baud
rate. A single late bit corrupts a byte, so the usable baud is set entirely by
worst-case latency:

```
usable baud  ≈  1 / (4 × worst-case latency)
             ≈  1 / (4 × 12.5 µs)  ≈  20 kbaud
```

That capped bit-banging at ~9600 baud, so **bit-banging was abandoned in favour
of PIO**, which solves it completely — see below. `softuart/softuart.c` is kept
as the measurement that justified the decision.

### Solution: RP1 PIO — 921600 achieved, no board change

The RP1's PIO block generates every bit edge in hardware, so Linux scheduling
latency stops mattering: the CPU only has to keep a 4-deep FIFO fed, and it has
~87 µs per byte at 921600 to do it. `softuart/pio_uart.c` implements the classic
8N1 `uart_tx` program (8 PIO cycles per bit, state machine clocked at 8 × baud).

Measured on GPIO24, clk_sys = 200 MHz, accounting for the hardware divider's
integer + 8-bit-fraction quantisation:

| Baud | clkdiv (requested → quantised) | Actual baud | Error |
|---:|---|---:|---:|
| 9600 | 2604.1667 → 2604.1680 | 9599.9 | −0.001% |
| 115200 | 217.0139 → 217.0156 | 115199.0 | −0.001% |
| 230400 | 108.5069 → 108.5078 | 230398.1 | −0.001% |
| 460800 | 54.2535 → 54.2539 | 460796.2 | −0.001% |
| **921600** | 27.1267 → 27.1250 | **921659.0** | **+0.006%** |

A UART tolerates roughly ±2% total error between the two ends, so +0.006% is
three hundred times inside tolerance. GPIO24 muxes to `PIO24` and idles high as
a UART line should.

Both links therefore run at 921600, and the debug port **can** mirror bulk media
in real time.

Remaining work: the PIO **RX** path on GPIO23 (a second state machine running
the standard `uart_rx` program), and wiring the PIO port into
`radcam/telemetry.py` as the mirror device.

---

## 8. Region capture — full resolution, only the part that matters

`CAPTURE_REGION` (`0x33`) reads the sensor at its **full 4096×3072**, keeps one
window of it, and rescales that window to a requested output size.

```
payload:  x u16 | y u16 | w u16 | h u16 | out_w u16 | out_h u16
          └── window in full-resolution sensor pixels ──┘ └ stored size ┘
reply:    CAPTURE_ACK (0xB0) — media id, size
```

This is deliberately **not** the same as configuring a 1080p capture. Asking
the sensor for 1920×1080 bins the whole scene down and throws the detail away
before anything can use it. Reading full resolution and then cropping keeps
every sensor pixel inside the window — so a distant target is imaged at the
sensor's real angular resolution, and only the pixels worth sending are sent.

### 8.1 Why it matters at 88 kB/s

The link is the constraint, and the frame is mostly not interesting:

| What is sent | Pixels | JPEG_MED size | Transfer time |
|---|---:|---:|---:|
| Full frame, 4096×3072 | 12.6 M | ~2.4 MB | **~27 s** |
| Full frame downscaled to 1080p | 2.1 M | ~420 kB | ~4.8 s |
| **1920×1080 window of the full frame** | 2.1 M | ~420 kB | **~4.8 s** |
| 640×480 window | 0.3 M | ~70 kB | ~0.8 s |

Rows 2 and 3 cost the same to transmit. They are not the same picture: row 2 is
the whole scene at 1/6 the angular resolution, row 3 is a sixth of the scene at
full angular resolution. For inspecting a specific feature, row 3 carries the
information and row 2 does not.

### 8.2 Bounds and failure

`x + w` must not exceed 4096 and `y + h` must not exceed 3072; `w`, `h`,
`out_w` and `out_h` must all be non-zero, and the output may not exceed full
resolution. Anything else is `ERR_REGION_INVALID` — the request is rejected
rather than silently clamped, because a silently moved window returns a picture
of the wrong thing and nothing downstream can tell.

The persistent `crop_*` config keys (`0x0A`–`0x0D`) still apply to
`CAPTURE_IMAGE` and to video. `CAPTURE_REGION` ignores them and uses only the
window in its own payload, so a one-off inspection does not disturb the
standing configuration.

### 8.3 `undistort` — correcting the lens on capture

Setting `undistort = 1` applies the distortion model the camera carries on its
EEPROM before the frame is stored, so what comes down the link is rectilinear.

It costs CPU, and on a power-minimised payload that is the deciding factor:

| Frame | Undistort time |
|---|---:|
| 1920×1080 | 0.62 s |
| 4096×3072 | 3.93 s |

Which is why it is **off by default** and per-request rather than permanent. A
12 MP correction costs about four seconds of CPU for a frame that then takes
27 s to transmit, so it is nearly free in wall-clock terms on a full-frame
downlink — but it is not free in energy, and for a small window it can dominate.

Two failure modes both degrade to "you still get the picture":

- The camera carries **no** distortion model — the frame is stored uncorrected.
- The model is present but **malformed** — the failure is logged and the raw
  frame is stored.

Neither returns an error. A missing calibration must not become a missing
image; that trade is the wrong way round for a payload that may only get one
look at something.

For `CAPTURE_REGION` the correction is applied **before** the window is cut,
because the model is defined over the whole frame — applying it to an
already-cropped window would use the wrong radii and bend the picture the wrong
way.

---

## 9. EEPROM access — reading and repairing calibration in flight

Every per-camera calibration lives on the camera module's own 24C64: colour
matrix, white balance, lens shading, distortion, sensor response. It is
measured on the ground and **cannot be recomputed in flight**. That makes it
the one piece of state aboard where a bit flip is unrecoverable without a way
to reach in and fix it — hence these four commands.

They operate on **raw bytes**, not on the parsed record. If what got corrupted
is the JSON itself, an interface that parsed before answering could not report
the problem, let alone repair it.

### 9.1 Layout being addressed

```
0x0000  copy 0   magic(8) length(2) crc32(4) payload
0x0800  copy 1   ...
0x1000  copy 2   ...
0x1800  spare
```

Three independent copies, each with its own CRC-32, majority-voted on read.

### 9.2 Commands

```
EEPROM_STATUS  (0x62)  payload: none
  -> 0xE2   copies u8 | good u8 | flag u8 per copy

EEPROM_READ    (0x60)  payload: offset u16 | length u16      (length ≤ 128)
  -> 0xE0   offset u16 | length u16 | bytes

EEPROM_WRITE   (0x61)  payload: offset u16 | length u16 | bytes
  -> 0xE1   offset u16 | length u16 | verified u8

EEPROM_REPAIR  (0x63)  payload: none
  -> 0xE3   healed u8
```

**Ask `EEPROM_STATUS` first.** It reads three headers, costs almost nothing,
and answers the question that actually matters after a radiation event: how
many copies are still intact. If at least one is good, `EEPROM_REPAIR` rewrites
the others from it and no ground data needs to be uplinked at all — 12 bytes of
traffic instead of a kilobyte.

`EEPROM_WRITE` is the last resort, for when **all three** copies are gone. It
reads the bytes back after writing and reports whether they match, so the
ground confirms the write rather than trusting an acknowledgement. Reads are
capped at 128 bytes per frame so a corrupted reply is cheap to re-request and a
full dump interleaves with telemetry instead of monopolising the link. A whole
8 KB dump is 64 frames, about 1.2 s of airtime.

### 9.3 Write protection

`EEPROM_WRITE` returns `ERR_WRITE_PROTECTED` unless writes have been unlocked
(`Dispatcher(eeprom_writable=True)`). The command exists for a fault that may
never occur, and a stray or corrupted frame that reached it would destroy the
only state aboard that cannot be regenerated. The default is therefore closed,
and opening it is a deliberate mission action.

Note that `EEPROM_REPAIR` is **not** gated this way: it can only ever copy an
already-verified payload over a failing one, so the worst it can do is nothing.


---

## 10. Open items
- Section 1 figures are **measured on this hardware**, but image sizes come
  from a synthetic frame. Re-run `tools/bench-compression.py` against real
  AR1335 frames once the sensor responds and update the tables; encode times
  and the video ratios will not change.
- PIO **RX** on GPIO23 is not yet implemented (TX at 921600 is working).
- The capture pipeline (`radcam/camera.py`) runs against `SyntheticSource`;
  `LibcameraSource` takes over unchanged when the sensor works.
