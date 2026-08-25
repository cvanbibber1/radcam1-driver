# STP / DICE RS-422 — implementation notes

The payload is an **Experiment** on the DICE bus: a slave that may not transmit
unless spoken to. This directory holds the supplied ICD excerpts; this file
records what was built from them, what had to be decided, and what is still
assumed.

| Document | What it fixes |
|---|---|
| `STP_protocol_req.md` | mission context, the ADM2582E and its DE pin, the TMR request |
| `rs422_command_packet_format.md` | the 120-byte command packet, field by field |
| `dice_experiment_rs422_protocol.md` | all six packet classes, CRC coverage, flow control |

## Code map

| Module | Responsibility |
|---|---|
| `radcam/stp/packets.py` | the six packet layouts, offsets taken verbatim from the ICD |
| `radcam/stp/crc.py` | parameterised CRC-16 + a solver for recovering the real parameters |
| `radcam/stp/rx.py` | resynchronising receiver; survives a shared, noisy bus |
| `radcam/stp/link.py` | half-duplex transport and DE timing |
| `radcam/stp/experiment.py` | the slave state machine |
| `radcam/stp/commands.py` | the 105-byte command payload — **ours to define** |
| `radcam/stp/lrt.py` | the 1248-byte housekeeping payload — **ours to define** |
| `radcam/stp/hrt.py` | the 1280-byte bulk payload and transfer manager — **ours to define** |
| `radcam/stp/fec.py` | XOR parity groups: correcting a lost chunk, not just detecting it |
| `radcam/stream.py` | live H.264 over HRT: encoder pipeline, region of interest, drop-not-delay |
| `radcam/slots.py` | numbered storage slots, so a canned command has a fixed address |
| `radcam/stp/catalogue.py` | the command set as data, so hex strings and docs cannot drift |
| `radcam/stp/redundancy.py` | TMR for state in RAM, plus a scrubber |
| `radcam/stp/timebase.py` | GPS-epoch coarse/fine time |

| Tool | Purpose |
|---|---|
| `tools/stp-sim.py --self-test` | drives the whole protocol in-process; 19 checks |
| `tools/stp-de-timing.py --throughput` | measures DE assertion against wire time |
| `tools/stp-crc-solve.py --bin capture.bin` | recovers the real CRC parameters from traffic |

Tests: `tests/test_stp_packets.py` (26), `tests/test_stp_experiment.py` (50),
`tests/test_stp_fec.py` (13) and `tests/test_stp_stream.py` (36).

## The three decisions that shape everything

### 1. The ACK means "accepted", not "done"

The Command Acknowledge packet is 8 bytes with no status field, and a
CAPTURE_IMAGE takes seconds. So a valid command is acknowledged immediately on
the receive thread and executed on a worker; **the result comes back in LRT
telemetry**, keyed by the `cmd_seq` the ground supplied.

Executing before acknowledging would mean either a ground timeout or a receive
loop that is deaf for seconds — and while deaf we would miss the flow-control
packets that tell us when we may transmit.

### 2. Everything egress is pulled

We never initiate. Housekeeping and command results wait for an LRT request;
bulk media waits for `HRT Go`. Responses larger than the 512-byte LRT response
window are additionally queued for HRT under a synthetic media id
(`0xFF000000 | cmd_seq`), so nothing depends on a channel that may never open.

### 3. Failure degrades, it does not stop

Five consecutive command failures trip **safe mode**: HRT stops and non-essential
commands are refused, but **LRT keeps answering**. A payload that has gone quiet
is indistinguishable from a dead one, and the ground needs housekeeping most
when things are going wrong. `StpOp.CLEAR_SAFE_MODE` (0x70) recovers it.

## DE: a hardware question before it is a software one

Before any of the timing below matters, establish how the board wires DE. This
one does it in hardware — DE pulled up, /RE pulled down, permanently enabled in
full duplex — and software driving GPIO4 low between packets held the
transmitter off. A push-pull output overrides a pull-up.

The failure is silent from this end. DE toggles, the UART counts the bytes, and
TEMT says the shift register emptied; none of that observes the differential
driver. What finally showed it was crosstalk: with DE parked low, transmitting
produced no received bytes at all on an unconnected receive pair, and with DE
released every transmit burst produced exactly one — our own driver switching,
coupling in. A silent listen returned zero and a 2000-byte burst returned one,
which distinguishes crosstalk from a wired loopback.

`"de_control": false` makes the link release the pin to an input at startup.
Refraining from asserting DE is not enough; the pin must stop being driven.

## DE timing — the measurement that mattered

The bus may carry up to five other experiments, so every microsecond DE stays
asserted past our last stop bit is time somebody else cannot transmit.

The obvious implementation is `tcdrain()`. On this PL011 it takes **8–13 ms
regardless of packet size** — draining an 8-byte ACK whose wire time is 87 µs
took 12 ms, holding the bus 140× longer than we were using it.

`TIOCSERGETLSR` / `TIOCSER_TEMT` reports when the shift register actually
clears. Measured on this board at 921600 baud:

| Packet | Wire time | DE held | Excess |
|---|---:|---:|---:|
| Command ACK, 8 B | 86.8 µs | 108.6 µs | **21.8 µs** |
| Command-size, 120 B | 1302.1 µs | 1327.1 µs | **25.0 µs** |
| LRT Data, 1256 B | 13628.5 µs | 13662.2 µs | **33.7 µs** |
| HRT Data, 1288 B | 13975.7 µs | 14008.9 µs | **33.2 µs** |

Flat ~30 µs at any size, and sustained HRT runs at **89.2 kB/s of payload,
99.7% wire utilisation**. Re-check any time with `tools/stp-de-timing.py`.

Two subtleties are load-bearing:

* The wait is timed **from before `write()`**, not after. `write()` blocks for
  most of a full packet because the kernel buffer is smaller than 1288 bytes;
  timing from after it double-counts, which measured as 24.5 ms on a 14.0 ms
  packet.
* If `TIOCSERGETLSR` is unavailable the code falls back to `tcdrain` plus a
  guard and **logs a warning**, because the difference matters to everyone else
  on the bus.

## What the ICD leaves undefined, and what we did

The ICD says explicitly not to invent these. None of them are hard-coded; all
live in `Wire` and `/etc/radcam/config.json` under `"stp"`.

| Item | Setting | Value | Confidence |
|---|---|---|---|
| Endianness | `big_endian` | `true` | **mission-confirmed**; verified on the wire as `1A CF FC 1D` |
| CRC-16 variant | `crc_variant` | `CRC-16/CCITT-FALSE` | **mission-confirmed** |
| CRC position | — | final two bytes of **every** message | **mission-confirmed** |
| CRC coverage | `crc_start` | `4` | ICD-stated for HRT, consistent for the rest |
| Target ID | `target_id` | **`0xC7`** | **mission-assigned** |
| Coarse-time epoch | — | GPS, 1980-01-06 | **mission-confirmed** |
| Leap offset | — | 18 s | current; a mission-time leap second needs updating |
| LRT trailing 2 bytes | `lrt_trailer` | `crc` | **mission-confirmed** (was an inference) |
| Baud | `baud` | 921600 | **mission-confirmed** |
| DE pin | `de_gpio` | GPIO4, active high | **mission-confirmed** |
| Initial HRT state | — | disabled | **mission-confirmed** |
| Stop 0x85 | — | finish the packet in flight | **mission-confirmed** |
| Stop-with-loss 0x86 | — | truncate the packet in flight | **mission-confirmed** |

Raw `coarse_time` and `fine_time` are stored verbatim in LRT, so if the epoch
or leap offset is wrong, every past record is still re-derivable.

**If the CRC turns out to be something else**, capture real traffic and run
`tools/stp-crc-solve.py --bin capture.bin`. It searches 15 standard variants ×
both byte orders × three coverage ranges and prints the config change needed.
Verified by construction: given packets built with CRC-16/X-25 little-endian,
it recovers exactly that from four samples.

### The two stops

Both halt HRT the moment they are seen; they differ only in what happens to a
packet already going out.

**0x85 Stop** lets it finish, so the ground gets a whole, valid final packet.

**0x86 Stop with loss** cuts it short. `Rs422Link.abort_tx()` discards what is
still queued in the kernel and drops DE, so the receiver sees a truncated frame,
fails its CRC and discards it — which is exactly what "with loss" names. Up to
a FIFO's worth of bytes, about 32 or 350 µs, may already be past recall; the
guarantee is that the packet does not *finish*, not that it stops on a given
bit.

Detecting the stop mid-transmission needs the transceiver's receiver live while
we transmit. The ADM2582E is full duplex, with separate driver and receiver
pairs, so with /RE tied active this works: `send()` polls an abort check every
millisecond while the packet drains. If /RE is instead tied to DE the payload is
deaf while transmitting, the check never fires, and the abort takes effect at
the next packet boundary. That is a wiring question, not a software one.

After an abort, a truncated **file** chunk is rewound by exactly one so it goes
again — unlike a guess about what the master might have lost, this knows which
chunk did not arrive. A truncated **video** frame is dropped instead: by the
time the tap reopens it is stale, which is the same reason the stream ring
discards rather than queues.

## Storage slots

Captures address **numbered slots**, not auto-incrementing media ids. The
reason is the ground station: it pastes fixed hex strings, so "download the
image I just took" can only exist as a command if the address is fixed in
advance. Slot 3 is slot 3 whatever happened before.

Deletion is explicit and is what frees space, so storage is managed by the
ground rather than filling silently. The slot index is stored triple-redundantly
through `radcam.tmr` — it is the one piece of state that makes the stored bytes
findable, and losing it would leave files on disk that nothing can name. The
payload bytes are not triplicated: far too large, and a corrupted image is
recoverable by taking another.

Each slot carries a CRC-32 written with it and checked on read, so a file that
rotted is reported as a fault rather than discovered after minutes of downlink.
A recording interrupted by power loss leaves its slot free, not half-occupied.

Slot transfers use media ids `0x510000NN`, a namespace that cannot collide with
a legacy media id on the wire.

## Commands as hex strings

The ground pastes complete 120-byte packets as hex. That makes the command set
a table rather than prose, so `radcam/stp/catalogue.py` holds it as data and
both `tools/stp-command.py` and the user guide's reference render from it — a
command cannot be documented wrongly if the documentation is generated from the
thing that builds it. `--verify` feeds every generated string to a real decoder
and confirms it is accepted and dispatched to the opcode it claims.

Canned strings carry the **force flag**, because the sequence number is baked in
and would otherwise be suppressed as a retransmission on the second paste.

## The parts we defined

### 105-byte command payload

```
 0   1  opcode          radcam.protocol.Msg, or StpOp 0x70-0x73
 1   2  cmd_seq         echoed in LRT so a result cannot be misattributed
 3   1  arg_len         0..98
 4   1  flags           bit 0 = force (bypass duplicate suppression)
 5   2  payload_crc16   over bytes[0:5] + args; 0 disables the check
 7  98  args
```

The inner CRC is deliberate: the envelope CRC proves the packet crossed the
wire, but this payload is then copied, queued, and may sit in RAM through a
capture before it is acted on. Note it covers only the header and the
*declared* args — trailing padding is outside it by design.

**Duplicate suppression.** A repeated `cmd_seq` is treated as a retransmission:
acknowledged, but not executed twice. The sequence number is claimed at
*acceptance*, on the receive thread — claiming it after execution left a window
where a fast retransmission ran CAPTURE_IMAGE twice. `FLAG_FORCE` overrides.

### 1248-byte LRT payload

Fixed header (uptime, boot count, last-command result, dose, temperature,
storage, link counters, HRT/transfer state, subsystem health), then a 256-byte
response window, then a 32-byte file header and a 512-byte file chunk, then a
ring of up to 19 events, then a CRC-32 over the whole payload.

```
    0  198  header          housekeeping, counters, last-command result
  198    7  response header opcode, cmd_seq, length, full length
  205  256  response data   command replies that fit inline
  461   32  file header     state, flags, media id, chunk index/total,
                            file size, file CRC-32, data length, FEC group,
                            chunk CRC-32
  493  512  file data       one transfer chunk, or one parity chunk
 1005    2  event count
 1007  228  events          19 x 12 bytes, newest last
 1244    4  payload CRC-32
```

Two things are redundant on purpose:

* **Dose is triplicated in the payload.** The envelope CRC-16 detects
  corruption but cannot correct it, and a dropped LRT costs a whole poll
  interval. Triplication lets a ground station salvage the measurement from a
  CRC-failed packet by majority vote. Verified: flipping a byte of copy 1
  correctly fails the CRC-32 while still returning the exact value.
* **A CRC-32 covers the payload** independently of the envelope CRC-16 — the
  envelope proves it crossed the wire, this proves it was assembled intact.

There is no query in this ICD for "what happened while you were out of
contact", so every LRT carries the newest events unconditionally.

### 1280-byte HRT payload

```
 0   2  sub_type    IDLE / MEDIA_INFO / MEDIA_DATA / MEDIA_END
 2   2  flags       bit 0 last chunk, bit 1 retransmit
 4   4  media_id
 8   4  chunk_index
12   4  chunk_total
16   2  data_len    0..1256
18   2  reserved
20   4  data_crc32
24 1256 data
```

The per-chunk CRC-32 matters: a CRC-16 over 1282 bytes is weak for a file
transfer, and it lets the ground identify *which* chunks are bad and re-request
exactly those, rather than discovering at the end that the whole file is wrong.

A resend almost always arrives *after* the transfer completed — that is when
the ground knows what was corrupted. So completed transfers are retained (last
2) and, beyond that, `TransferManager.reload` fetches the bytes from the media
store again. Without both, resend worked only in the case where it was least
needed.

## What each channel is for

| Channel | Carries | Never carries |
|---|---|---|
| **LRT** | telemetry and vitals: dose, temperature, storage, link health, last-command result, event ring | bulk data of any kind |
| **HRT** | live video, and chunked file transfer of stored media | housekeeping |

An earlier revision reserved 544 bytes of the LRT payload for a contingency
file-transfer path. That is gone: with HRT confirmed as the transfer channel,
the space is better spent on what LRT is actually for. Removing it doubled the
command-response window back to 512 bytes and took the event ring from 19
entries to 40.

## Live video

The stream is the reason HRT exists on this payload. It is not a file transfer
with the end left off, and the difference drives every decision:

* A **file transfer** must not lose a byte, so it queues. Latency does not
  matter; completeness does.
* A **stream** must not fall behind, so it discards. Completeness does not
  matter; latency does. A stream that buffers whatever it cannot send has a
  delay that only grows — after ten minutes of a closed HRT tap you are
  watching ten-minute-old video.

So frames go into a bounded ring and the **oldest is dropped** when it fills,
and closing HRT flushes the ring rather than holding it. A rising drop count
under load is correct operation, not a fault. Dropping requires whole frames,
which is why `radcam/stream.py` parses the encoder's Annex-B output into access
units rather than treating it as opaque bytes.

Live video takes priority over file transfer on HRT, because it is the only
traffic on the link whose value expires. When the encoder is between frames,
that gap goes to file chunks — so a transfer running alongside a stream makes
progress rather than starving.

### The encoder is two processes, and the reason matters

`rpicam-vid --codec h264` **does not work on this board**. The Pi 5 dropped the
Pi 4's hardware H.264 encoder, there is no `/dev/video11`, and this build of
rpicam-apps was compiled without libav. It answers *"Unable to find an
appropriate H.264 codec"* and exits.

So the camera emits raw YUV420 and **ffmpeg/libx264 encodes in software** — the
same encoder `tools/bench-compression.py` measured. x264 runs with
`sliced-threads=0:threads=1`: sliced threading splits each picture into one
slice per core, which at 640x480 buys nothing and during bring-up produced four
VCL NALs per frame, making a naive parser report 55 fps when the true rate was
14. The parser handles multi-slice pictures correctly regardless — it tests
`first_mb_in_slice` — but one slice per frame is simpler and lower latency.

### Measured end to end

Camera → libx264 → HRT packets → real tty → ground reassembly → decode:

| Quantity | Result |
|---|---|
| Frame rate | **15.0 fps**, exactly as configured |
| Bitrate | **584 kbit/s** against a 600 kbit/s target |
| Keyframes | one per second, each carrying SPS+PPS+IDR |
| Chunk CRC failures | **0** |
| Frames delivered | 180 of 181 in the window |
| Decode | 168 frames, 640x480 yuv420p, **no decoder errors** |

Frames are self-describing — frame number, chunk index and count, keyframe flag
— so a ground station can join mid-stream and start decoding at the next
keyframe. There is no MEDIA_INFO for a stream, because a stream has no known
length and no beginning the receiver is guaranteed to have seen.

### Region of interest

The sensor is 4208x3120 and the link carries perhaps 600 kbit/s. Streaming the
whole frame spends almost all of it on detail the encoder then destroys.
Instead the stream takes a **crop of `crop_w` x `crop_h` centred on a chosen
sensor pixel** and scales it to the output size. Setting the crop equal to the
output gives 1:1 sensor pixels — the region of interest at full native
resolution, with nothing spent on the rest of the field.

Everything is clamped rather than refused: a centre near the edge is pulled in
so the box stays on the sensor, dimensions are forced even for YUV420, and the
**effective values are reported in telemetry**, so what was applied is never in
doubt.

## Error correction, not just detection

Every chunk on both paths already carried a CRC-32, which *detects* corruption.
Detection alone means a retransmission, and a retransmission costs a whole poll
interval on LRT, or waiting for the master to reopen the tap on HRT. So both
paths now also carry **XOR parity**: after every `group_size` data chunks, one
parity chunk whose bytes are the XOR of that group. Any single chunk lost or
corrupted per group is reconstructed by the ground with nothing asked in
return.

```
emission order at group size 4:   0 1 2 3 P0  4 5 6 7 P1  8 9 P2  END
```

Parity is emitted **as each group closes**, not saved until the end, so a
transfer that DICE stops part-way still leaves every completed group
repairable.

| Property | Value |
|---|---|
| Default group size | 16 → **6.25% overhead** |
| Corrects | any one chunk per group, no retransmission |
| Does not correct | two or more in a group — falls back to explicit RESEND |
| Disable | `SET_FEC_GROUP` (0x77) with 0 |

Parity applies to HRT **file transfer**. The live stream does not carry parity:
a video frame that arrives late is worthless, so spending 6.25% of the link on
repairing one has the priorities backwards — H.264 recovers at the next
keyframe, one second later, for free.

**Why XOR rather than Reed-Solomon.** RS corrects more, but needs a field
implementation and tables on a machine where the rule is stdlib only, and its
advantage is concentrated in the multi-error case that RESEND already handles.
XOR parity is a few lines, has no failure mode of its own, and covers what
actually dominates on a UART link: a single chunk lost to a burst of noise.

Short final chunks are safe because XOR with zero is the identity — the parity
is computed over zero-padded chunks and the receiver truncates to the advertised
length.

## Integration

`"stp": {"enabled": true}` in `/etc/radcam/config.json` changes the daemon:

* The flight port becomes the STP link. `telemetry.flight` is replaced with
  `NullTelemetryLink` so the ASCII beacon **cannot** reach the DICE bus —
  removing the port is stronger than remembering not to write to it.
* The old COBS command server is disabled; both would read the same port.
* A dedicated `stp-service` thread answers DICE continuously, decoupled from
  the 2 s housekeeping cadence. It blocks in the kernel between packets rather
  than spinning.
* The readable beacon still goes to the PIO debug mirror on GPIO23/24.

The daemon refreshes a housekeeping snapshot each cycle; the LRT reply path
only hands it over, and **never does I/O while DICE is waiting**.

## Still open

1. **No wire test against real DICE.** Verified in-process, through PTY pairs,
   and on the real UART for DE timing, throughput and live video — but the
   flight computer has not been in the loop.

Everything else the ICD left undefined has now been settled by the mission:
byte order, CRC variant and position, target ID, epoch, baud, DE pin, the two
stop semantics, and the initial HRT state.
