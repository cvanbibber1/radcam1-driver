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
| `radcam/stp/lrtfile.py` | file transfer over LRT polls — the downlink that is never blocked |
| `radcam/stp/fec.py` | XOR parity groups: correcting a lost chunk, not just detecting it |
| `radcam/stp/redundancy.py` | TMR for state in RAM, plus a scrubber |
| `radcam/stp/timebase.py` | GPS-epoch coarse/fine time |

| Tool | Purpose |
|---|---|
| `tools/stp-sim.py --self-test` | drives the whole protocol in-process; 19 checks |
| `tools/stp-de-timing.py --throughput` | measures DE assertion against wire time |
| `tools/stp-crc-solve.py --bin capture.bin` | recovers the real CRC parameters from traffic |

Tests: `tests/test_stp_packets.py` (24), `tests/test_stp_experiment.py` (50)
and `tests/test_stp_fec.py` (33).

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
| Endianness | `big_endian` | `true` | Sync `1A CF FC 1D`; **assumed**, unverified |
| CRC-16 variant | `crc_variant` | `CRC-16/CCITT-FALSE` | **mission-confirmed** |
| CRC coverage | `crc_start` | `4` | ICD-stated for HRT, **assumed** for the rest |
| CRC byte order | (in variant) | big-endian | **assumed** |
| Target ID | `target_id` | `1` | **needs assignment** |
| Coarse-time epoch | — | GPS, 1980-01-06 | **mission-confirmed** |
| Leap offset | — | 18 s | current; a mission-time leap second needs updating |
| LRT trailing 2 bytes | `lrt_trailer` | `crc` | **inferred**, consistent with HRT |
| Initial HRT state | — | disabled | **assumed**, conservative |
| Stop-with-loss | — | rewind 1 chunk | **assumed**, see below |
| Baud | `baud` | 921600 | **mission-confirmed** |
| DE pin | `de_gpio` | GPIO4, active high | **mission-confirmed** |

Raw `coarse_time` and `fine_time` are stored verbatim in LRT, so if the epoch
or leap offset is wrong, every past record is still re-derivable.

**If the CRC turns out to be something else**, capture real traffic and run
`tools/stp-crc-solve.py --bin capture.bin`. It searches 15 standard variants ×
both byte orders × three coverage ranges and prints the config change needed.
Verified by construction: given packets built with CRC-16/X-25 little-endian,
it recovers exactly that from four samples.

### Stop-with-loss (0x86)

The ICD names it but does not define it, and says not to invent it. What is
implemented is the smallest thing that could help and cannot corrupt anything:
stop as for 0x85, rewind the send pointer by one chunk so the packet likely in
flight goes again, and log the event. Re-sending a chunk the ground already has
is idempotent at the reassembler. **The authoritative recovery path is an
explicit RESEND command**, which does not depend on this guess.

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

## Two downlink paths, and why both exist

HRT is the fast one. It is also the one that needs permission: it flows only
between an `HRT Go` and a `Stop`, and that decision belongs to DICE. A payload
with an HRT-only downlink cannot return an image if the master never opens the
tap — because HRT is allocated to another experiment that pass, or the schedule
simply does not permit it.

So a file can also be pulled through the **LRT file block**, one chunk per poll,
on the channel that is always polled:

| Path | Per packet | Rate | Needs |
|---|---:|---|---|
| HRT | 1256 B | **89.2 kB/s measured** | `HRT Go` from DICE |
| LRT | 512 B | 512 B × poll rate (~5 kB/s at 10 Hz) | nothing |

A 3 MB image is 35 seconds over HRT and about ten minutes over LRT. That makes
LRT a poor way to move a large image and a perfectly good way to move a
thumbnail, a dose log, a calibration record, or an image that would otherwise
never arrive at all. It is also how an oversized command response gets home:
replies too large for the 256-byte LRT response window are held addressable
under a synthetic id (`0xFF000000 | cmd_seq`) and can be pulled either way.

The file block is **separate from the response window**, not a reuse of it, so
a transfer in progress and a command reply travel in the same LRT packet. If
they shared one window, every command issued mid-transfer would stall it.

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

Measured end to end on the LRT path with 8% of replies dropped: parity rebuilt
5 of 9 missing chunks unaided, RESEND recovered the remaining 4 (whose groups
had lost two chunks, or lost their parity), and the file came out bit-exact
with a matching whole-file CRC-32.

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

1. **Target ID is a placeholder (1).** Needs the real assignment before flight.
2. **Endianness is assumed big.** One `stp-crc-solve.py` run against real
   traffic settles it along with the CRC.
3. **The LRT trailing 2 bytes are inferred to be CRC.** Set
   `"lrt_trailer": "zero"` if the authoritative ICD says otherwise.
4. **Stop-with-loss semantics are a guess.** See above.
5. **No wire test against real DICE.** Verified in-process, through a PTY pair,
   and for DE timing on the real UART — but the flight computer has not been
   in the loop.
