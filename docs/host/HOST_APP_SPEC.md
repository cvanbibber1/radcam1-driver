# radcam-host — Ground Station Application Specification

**Version 1.0 — 2026-08-21**

A Windows application that talks to the radcam1 payload over RS-422, acting as
the flight computer (DICE) does. It sends commands, receives telemetry and bulk
data, shows live video and partially-assembled images as they arrive, records
everything to disk, and can rebuild images and video from a saved capture
offline.

This document is written to be implementable **without access to the payload
source**. Every constant, offset and algorithm needed to interoperate is here.

> Intended as the seed document for a separate repository, `radcam-host`. The
> payload side lives in `driver-dev` and is not a dependency.

---

## Contents

1. [Scope and roles](#1-scope-and-roles)
2. [Hardware and driver setup](#2-hardware-and-driver-setup)
3. [Wire protocol](#3-wire-protocol)
4. [Command payload](#4-command-payload)
5. [LRT: telemetry payload](#5-lrt-telemetry-payload)
6. [HRT: bulk payload](#6-hrt-bulk-payload)
7. [Forward error correction](#7-forward-error-correction)
8. [Live video](#8-live-video)
9. [Storage slots](#9-storage-slots)
10. [Command reference](#10-command-reference)
11. [Codes and enumerations](#11-codes-and-enumerations)
12. [Operating procedure](#12-operating-procedure)
13. [Application architecture](#13-application-architecture)
14. [User interface](#14-user-interface)
15. [On-disk layout](#15-on-disk-layout)
16. [Offline assembly](#16-offline-assembly)
17. [Timing and budgets](#17-timing-and-budgets)
18. [Testing](#18-testing)
19. [Test vectors](#19-test-vectors)
20. [Acceptance criteria](#20-acceptance-criteria)
21. [Traps](#21-traps)

---

## 1. Scope and roles

### 1.1 What the host is

On the real spacecraft the payload is a **slave** on a bus mastered by DICE.
For bench testing, **this application is the master.** It has the same
obligations DICE has, and the payload's behaviour only makes sense in that
light:

- **The payload never transmits unless spoken to.** Silence is correct, not a
  fault. If nothing is arriving, the host is not asking.
- **Telemetry is polled.** No poll, no vitals.
- **Bulk data is gated.** Nothing large moves until the host sends `HRT_GO`,
  and it stops on `HRT_STOP`.
- **The payload answers only to its Target ID**, `0xC7`. Packets addressed
  elsewhere are silently ignored — including by the host, which must ignore
  anything not addressed to it.

### 1.2 Required capabilities

| # | Capability |
|---|---|
| R1 | Connect to an FT232R-based RS-422 adapter on a Windows COM port at 921600 baud |
| R2 | Poll telemetry continuously and display vitals live |
| R3 | Send every command in [§10](#10-command-reference) from UI controls |
| R4 | Correlate command results, which arrive in telemetry, not in the acknowledgement |
| R5 | Receive live H.264 video and display it with low latency |
| R6 | Receive file transfers and show the image **assembling progressively**, chunk by chunk |
| R7 | Record every received byte, plus decoded packets, telemetry and events, to a session folder |
| R8 | Rebuild images and video **offline** from a saved raw capture |
| R9 | Never lose data because the UI is busy |

### 1.3 Non-goals

Flight qualification, multi-experiment bus arbitration, and any attempt to
control the payload's internal scheduling. The host is a test console.

---

## 2. Hardware and driver setup

### 2.1 Signal path

```
  Windows PC ──USB── FT232R ──RS-422 transceiver ──┐
                                                    │  differential pairs
  radcam1 Pi 5 ──uart0──ADM2582E ───────────────────┘
```

The payload's transceiver is an **ADM2582E, which is full duplex**: separate
driver and receiver pairs, not a shared two-wire bus. Wire it accordingly:

| Payload | Direction | Host adapter |
|---|---|---|
| Payload TX pair (Y/Z) | → | Host RX pair (A/B) |
| Payload RX pair (A/B) | ← | Host TX pair (Y/Z) |

Ground must be common between the two ends. Termination of 120 Ω across each
receiving pair is appropriate for anything beyond a short bench lead.

### 2.2 Driver enable

The payload drives its own DE on GPIO4 and releases it within ~30 µs of its
last stop bit. **The host does not need to manage DE**: it is the only master
and, on a full-duplex link with no other talkers on its pair, its driver may
remain permanently enabled. Most FT232R RS-422 adapters wire DE that way
already.

> If the adapter is a **half-duplex RS-485** type instead, it must handle
> direction automatically (auto-DE) or the application must drive RTS. Prefer a
> true full-duplex RS-422 adapter; half duplex will also prevent the payload's
> `HRT_STOP_WITH_LOSS` from being detected mid-packet.

### 2.3 The latency timer — do this first

**The FTDI default latency timer is 16 ms. This must be changed to 1 ms.**

The FT232R buffers received bytes and only forwards them when either its buffer
fills or the latency timer expires. At 16 ms, a 14-byte reply is delayed by up
to 16 ms, telemetry arrives in clumps, and live video stutters regardless of
how well the application is written.

- Windows: *Device Manager → Ports → USB Serial Port → Properties → Port
  Settings → Advanced → Latency Timer (msec) → 1*
- The setting is per-device and persists in the registry.
- `pyserial` cannot change it. The application **must detect and warn**: measure
  the interval between the poll and the reply, and if the median exceeds ~5 ms,
  display a prominent warning naming the latency timer.

### 2.4 Serial parameters

| Parameter | Value |
|---|---|
| Baud | 921600 |
| Data bits | 8 |
| Parity | none |
| Stop bits | 1 |
| Flow control | none |
| Read buffer | ≥ 64 kB |
| Read timeout | 5–20 ms (short; the reader loop polls) |

At 921600 baud the link carries ~92 kB/s. A single HRT packet is 1288 bytes and
occupies the wire for **14.0 ms**.

---

## 3. Wire protocol

### 3.1 Byte order — read this twice

**Two conventions coexist, and confusing them is the most likely cause of a
non-working implementation.**

| Region | Byte order |
|---|---|
| Packet envelope: sync, coarse/fine time, CRC | **big-endian** |
| LRT payload fields | **big-endian** |
| HRT payload header | **big-endian** |
| Command payload header (opcode, seq, len, flags, CRC) | **big-endian** |
| **Command arguments** | **little-endian** |
| **Command response data** (`resp_data`) | **little-endian** |

The arguments and responses are little-endian because they are the payload's
older application format, carried unchanged inside the newer big-endian
envelope. There is no way to tell from the bytes; it must be coded correctly.

### 3.2 Sync pattern

Four bytes, at offset 0 of every packet in both directions:

```
1A CF FC 1D
```

### 3.3 CRC

**CRC-16/CCITT-FALSE**, stored **big-endian**, in the **final two bytes of
every packet**.

| Parameter | Value |
|---|---|
| Width | 16 |
| Polynomial | `0x1021` |
| Initial value | `0xFFFF` |
| Reflect input | no |
| Reflect output | no |
| Final XOR | `0x0000` |
| Check value (`"123456789"`) | `0x29B1` |

Coverage is **from byte 4 to the byte before the CRC**: `crc = CRC16(packet[4:-2])`.
The sync bytes are excluded.

```python
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc
```

### 3.4 Packet types

**Packet type alone does not identify a packet.** Three type values mean
different things in each direction; direction plus length disambiguates.

| Type | Host → payload | Payload → host |
|---|---|---|
| `0x10` | Command, **120 bytes** | Command ACK, **8 bytes** |
| `0x81` | LRT Request, **14 bytes** | LRT Data, **1256 bytes** |
| `0x85` | HRT Stop, 14 bytes | — |
| `0x86` | HRT Stop with loss, 14 bytes | — |
| `0x87` | HRT Go, 14 bytes | HRT Data, **1288 bytes** |

Since the host only ever *receives* payload-side packets, it reads the type
from **offset 4**, not offset 10.

### 3.5 Packets the host sends

**Command — 120 bytes**

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Sync `1A CF FC 1D` |
| 4 | 4 | Coarse time, u32 BE, seconds |
| 8 | 2 | Fine time, u16 BE, 15.3 µs/count |
| 10 | 1 | Packet type = `0x10` |
| 11 | 1 | Target ID = `0xC7` |
| 12 | 105 | Command payload ([§4](#4-command-payload)) |
| 117 | 1 | Spare = `0x00` |
| 118 | 2 | CRC-16 BE over `packet[4:118]` |

**LRT Request / HRT flow control — 14 bytes**

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Sync |
| 4 | 4 | Coarse time, u32 BE |
| 8 | 2 | Fine time, u16 BE |
| 10 | 1 | Type: `0x81`, `0x85`, `0x86` or `0x87` |
| 11 | 1 | Target ID = `0xC7` |
| 12 | 2 | CRC-16 BE over `packet[4:12]` |

**Timestamps.** Coarse time is seconds since the **GPS epoch (1980-01-06
00:00:00 UTC)**, which runs 18 s ahead of UTC. Fine time is the sub-second part
at 15.3 µs per count, wrapping at ~1.003 s. The payload only echoes these back;
nothing it does depends on them, so zeros are valid. Send real values anyway so
recorded telemetry is timestamped.

```python
GPS_EPOCH_UNIX = 315964800.0
GPS_UTC_OFFSET = 18

def to_dice(unix_ts):
    gps = unix_ts - GPS_EPOCH_UNIX + GPS_UTC_OFFSET
    coarse = int(gps)
    return coarse & 0xFFFFFFFF, int(round((gps - coarse) / 15.3e-6)) & 0xFFFF
```

### 3.6 Packets the host receives

**Command ACK — 8 bytes**

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Sync |
| 4 | 1 | Type = `0x10` |
| 5 | 1 | Target ID = `0xC7` |
| 6 | 2 | CRC-16 BE over `packet[4:6]` |

**LRT Data — 1256 bytes**

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Sync |
| 4 | 1 | Type = `0x81` |
| 5 | 1 | Target ID |
| 6 | 1248 | LRT payload ([§5](#5-lrt-telemetry-payload)) |
| 1254 | 2 | CRC-16 BE over `packet[4:1254]` |

**HRT Data — 1288 bytes**

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Sync |
| 4 | 1 | Type = `0x87` |
| 5 | 1 | Target ID |
| 6 | 1280 | HRT payload ([§6](#6-hrt-bulk-payload)) |
| 1286 | 2 | CRC-16 BE over `packet[4:1286]` |

### 3.7 Receive framing

The host must never assume it is aligned to a packet boundary. Required
algorithm:

```
loop:
  find the sync pattern in the buffer
  discard everything before it
  if fewer than 6 bytes are available: wait for more
  read the type byte at offset 4
  length = 8 if type == 0x10, 1256 if 0x81, 1288 if 0x87, else UNKNOWN
  if UNKNOWN:
      count it, advance past this sync by 4 bytes, continue   # do not guess a length
  if fewer than `length` bytes available: wait for more
  verify the CRC
  if the CRC fails:
      count it, advance past this sync by 4 bytes, continue   # do NOT consume `length`
  if target_id != 0xC7: count it and discard
  emit the packet
```

Two rules matter and are easy to get wrong:

- **A CRC failure resynchronises rather than consuming.** The length assumption
  may itself be wrong — a corrupted type byte can make a short packet look
  long — and consuming on that basis destroys the good packet that followed.
- **Unknown types are stepped past, not skipped by length**, because their
  length is unknown.

Bound the buffer (16–64 kB). Past the cap, drop the oldest bytes and count it.

---

## 4. Command payload

The 105 bytes at offset 12 of a Command packet.

| Offset | Size | Endian | Field |
|---:|---:|---|---|
| 0 | 1 | — | `opcode` |
| 1 | 2 | BE | `cmd_seq` — echoed in telemetry to identify the result |
| 3 | 1 | — | `arg_len`, 0–98 |
| 4 | 1 | — | `flags`; bit 0 = force |
| 5 | 2 | BE | `payload_crc16` over `payload[0:5] + args`; `0x0000` disables the check |
| 7 | 98 | LE | `args`, `arg_len` bytes used, remainder zero |

The inner CRC uses the same CRC-16/CCITT-FALSE. It covers only the header and
the **declared** argument bytes — trailing padding is outside it.

### 4.1 Sequence numbers and the force flag

The payload treats a repeated `cmd_seq` as a retransmission and **acknowledges
it without executing it again**. That protects a non-idempotent command such as
a capture or a delete from being run twice by a retry.

**The host application should use an incrementing `cmd_seq` and leave the force
flag clear.** That gives it retransmission protection for free. The force flag
exists for *canned* hex strings pasted from a config, where the sequence number
is fixed and every paste is meant to execute.

Wrap `cmd_seq` at `0xFFFF`. The payload remembers the last 64.

---

## 5. LRT: telemetry payload

1248 bytes, all fields **big-endian**. This is the host's primary data source:
vitals, command results, and everything needed to drive the dashboard.

### 5.1 Header and command result

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 0 | 2 | u16 | `format_version` = `0x0001` |
| 2 | 2 | u16 | `flags` |
| 4 | 4 | u32 | `coarse_time` last seen from the host |
| 8 | 2 | u16 | `fine_time` |
| 10 | 4 | u32 | `uptime_s` |
| 14 | 4 | u32 | `boot_count` |
| 18 | 2 | u16 | `target_id` |
| 20 | 1 | u8 | `fw_major` |
| 21 | 1 | u8 | `fw_minor` |
| 22 | 1 | u8 | `last_opcode` |
| 23 | 2 | u16 | **`last_cmd_seq`** — which command this result belongs to |
| 25 | 1 | u8 | **`last_result`** — 0 = success, else an error code |
| 27 | 4 | u32 | `last_done_uptime` |
| 31 | 4 | u32 | `cmds_received` |
| 35 | 4 | u32 | `cmds_executed` |
| 39 | 4 | u32 | `cmds_rejected` |

### 5.2 Dosimeter — triplicated

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 43 | 12 | f32 ×3 | `dose_rad`, three identical copies |
| 55 | 12 | f32 ×3 | `dose_volts`, three copies |
| 67 | 3 | u8 ×3 | `dose_calibrated`, three copies |
| 70 | 4 | u32 | `dose_samples` |
| 74 | 4 | u32 | `dose_errors` |

These fields are stored three times so a packet that fails its CRC can still
yield a trustworthy reading by **majority vote**. The host should implement the
vote: take the value appearing at least twice; if all three differ, treat it as
unavailable. This is what lets a corrupted telemetry packet still be partially
salvaged instead of costing a whole poll interval.

### 5.3 Housekeeping

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 78 | 4 | f32 | `cpu_temp_c` |
| 82 | 1 | u8 | `led_percent` |
| 83 | 1 | u8 | `camera_available` |
| 84 | 1 | u8 | `recording` |
| 85 | 4 | u32 | `recording_id` |
| 89 | 8 | u64 | `storage_free` bytes |
| 97 | 8 | u64 | `storage_used` bytes |
| 105 | 2 | u16 | `media_count` |

### 5.4 Link health

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 107 | 4 | u32 | `rx_good` |
| 111 | 4 | u32 | `rx_bad_crc` |
| 115 | 4 | u32 | `rx_bad_format` |
| 119 | 4 | u32 | `rx_not_for_us` |
| 123 | 4 | u32 | `rx_unknown_type` |
| 127 | 4 | u32 | `rx_resyncs` |
| 131 | 4 | u32 | `rx_dropped_bytes` |
| 135 | 4 | u32 | `tx_packets` |
| 139 | 4 | u32 | `tx_errors` |
| 143 | 4 | u32 | `lrt_sent` |
| 147 | 4 | u32 | `hrt_sent` |

These are the payload's view of the link. Plotting `rx_bad_crc` and
`rx_resyncs` against time is the fastest way to see a marginal cable.

### 5.5 HRT and transfer state

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 151 | 1 | u8 | `hrt_enabled` |
| 152 | 1 | u8 | `hrt_last_control` — `0x85`/`0x86`/`0x87`, 0 if none yet |
| 153 | 4 | u32 | `xfer_media_id` |
| 157 | 4 | u32 | `xfer_chunk_next` |
| 161 | 4 | u32 | `xfer_chunk_total` |
| 165 | 8 | u64 | `xfer_bytes_total` |
| 173 | 4 | u32 | `xfer_file_crc32` |
| 177 | 1 | u8 | `xfer_state` — 0 idle, 1 active, 2 paused, 3 complete |
| 178 | 2 | u16 | `xfer_queue_depth` |

### 5.6 Subsystem health

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 180 | 1 | u8 | `eeprom_copies` |
| 181 | 1 | u8 | `eeprom_good` |
| 182 | 1 | u8 | `dosimeter_ok` |
| 183 | 1 | u8 | `led_ok` |
| 184 | 1 | u8 | `camera_ok` |
| 185 | 4 | u32 | `tmr_corrections` — memory bit flips corrected |
| 189 | 4 | u32 | `tmr_failures` |
| 193 | 4 | u32 | `watchdog_pets` |
| 197 | 1 | u8 | **`safe_mode`** |

### 5.7 Command response window

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 198 | 1 | u8 | `resp_opcode` — the response type |
| 199 | 2 | u16 | `resp_cmd_seq` — which command it answers |
| 201 | 2 | u16 | `resp_len` — bytes present here, ≤ 512 |
| 203 | 2 | u16 | `resp_full_len` — full length if truncated, else 0 |
| 205 | 512 | bytes | `resp_data` (**little-endian contents**) |

If `resp_full_len` is non-zero the reply was too large for this window and the
payload has also queued the whole thing for HRT under media id
`0xFF000000 | cmd_seq`.

### 5.8 Live stream state

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 717 | 1 | u8 | `stream_state` — 0 off, 1 starting, 2 running, 3 fault |
| 718 | 1 | u8 | `stream_flags` — bit 0 gated, bit 1 encoder late |
| 719 | 2 | u16 | `stream_width` |
| 721 | 2 | u16 | `stream_height` |
| 723 | 1 | u8 | `stream_fps` |
| 724 | 4 | u32 | `stream_bitrate` |
| 728 | 2 | u16 | `stream_centre_x` |
| 730 | 2 | u16 | `stream_centre_y` |
| 732 | 2 | u16 | `stream_crop_w` |
| 734 | 2 | u16 | `stream_crop_h` |
| 736 | 4 | u32 | `stream_frames_sent` |
| 740 | 4 | u32 | **`stream_frames_dropped`** |
| 744 | 8 | u64 | `stream_bytes_sent` |
| 752 | 2 | u16 | `stream_queue_depth` |
| 754 | 1 | u8 | `fec_group_size` |

**`stream_flags` bit 0 (gated)** means the stream is running but HRT is closed,
so frames are being discarded. That is the answer to "why is no video
arriving", and the UI should surface it directly.

### 5.9 Storage slots

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 755 | 1 | u8 | `slot_count` |
| 756 | 1 | u8 | `slots_used` |
| 757 | 1 | u8 | `slot_recording` — `0xFF` = none |
| 758 | 8 | u64 | `slot_bytes_used` |
| 766 | 1 | u8 | `slot_downloading` — `0xFF` = none |

### 5.10 Event ring

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 770 | 2 | u16 | `event_count`, ≤ 39 |
| 772 | 12 × n | — | events, oldest first |

Each event is 12 bytes:

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 0 | 4 | u32 | `uptime_s` |
| 4 | 2 | u16 | `code` ([§11.2](#112-event-codes)) |
| 6 | 4 | u32 | `arg` |
| 10 | 1 | u8 | `severity` — 0 info, 1 warning, 2 error |
| 11 | 1 | — | padding |

There is **no command to query history**, so every telemetry packet carries the
most recent events unconditionally. The ring holds 39; poll often enough not to
miss any, and de-duplicate on `(uptime_s, code, arg)` when appending to the
session log.

### 5.11 Payload CRC-32

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 1244 | 4 | u32 | CRC-32 (IEEE, `zlib.crc32`) over `payload[0:1244]` |

Independent of the envelope CRC-16: the envelope proves the packet crossed the
wire, this proves the payload was assembled intact. Report it, but do not
discard on it — the triplicated dose fields exist precisely so a failed payload
CRC can still be partly salvaged.

---

## 6. HRT: bulk payload

1280 bytes, header **big-endian**.

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 0 | 2 | u16 | `sub_type` |
| 2 | 2 | u16 | `flags` |
| 4 | 4 | u32 | `media_id` — or the **frame number** for stream data |
| 8 | 4 | u32 | `chunk_index` — or the **parity group** for parity |
| 12 | 4 | u32 | `chunk_total` |
| 16 | 2 | u16 | `data_len`, ≤ 1256 |
| 18 | 2 | — | reserved |
| 20 | 4 | u32 | `data_crc32` over `data[0:data_len]` |
| 24 | 1256 | bytes | `data` |

### 6.1 Sub-types

| Value | Name | Meaning |
|---|---|---|
| `0x0000` | `IDLE` | filler; ignore |
| `0x0001` | `MEDIA_INFO` | start of a file transfer |
| `0x0002` | `MEDIA_DATA` | one numbered chunk of a file |
| `0x0003` | `MEDIA_END` | file finished |
| `0x0004` | `MEDIA_PARITY` | XOR parity for the group in `chunk_index` |
| `0x0005` | `STREAM_DATA` | one chunk of a live video frame |

### 6.2 Flags

| Bit | Value | Meaning |
|---|---|---|
| 0 | `0x0001` | last chunk of the file |
| 1 | `0x0002` | retransmission |
| 2 | `0x0004` | parity chunk |
| 3 | `0x0008` | keyframe (stream only) |

### 6.3 `MEDIA_INFO` body

33 bytes, **big-endian**:

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 0 | 4 | u32 | `media_id` |
| 4 | 8 | u64 | `size` in bytes |
| 12 | 4 | u32 | `chunk_total` |
| 16 | 4 | u32 | `file_crc32` |
| 20 | 1 | u8 | `kind` — 0 image, 1 video |
| 21 | 2 | u16 | `width` |
| 23 | 2 | u16 | `height` |
| 25 | 8 | f64 | `created_unix` |

### 6.4 `MEDIA_END` body

12 bytes big-endian: `media_id` u32, `file_crc32` u32, `chunks_sent` u32.

### 6.5 File reassembly

```
on MEDIA_INFO:   allocate a transfer keyed by media_id; record size,
                 chunk_total and file_crc32; start the progressive preview
on MEDIA_DATA:   verify data_crc32; store chunks[chunk_index]
                 (a chunk with the retransmit flag replaces any earlier copy)
on MEDIA_PARITY: verify data_crc32; store parity[chunk_index]
on MEDIA_END:    attempt completion
```

Completion:

1. Repair what is missing using parity ([§7](#7-forward-error-correction)).
2. If chunks are still missing, request them with `RESEND`.
3. Concatenate chunks `0..chunk_total-1`, truncate to `size`.
4. Verify `zlib.crc32(data) == file_crc32`.
5. Write to the session folder; mark the transfer complete.

**A transfer may be interrupted at any point** by an `HRT_STOP`. State must
survive that and resume when the tap reopens — do not discard partial
transfers on a stop.

### 6.6 Progressive preview (R6)

The host must show the image building as chunks land. JPEG is decodable from a
truncated prefix by most decoders, so:

- Maintain the byte array with holes zero-filled.
- Every N chunks (or ~4 Hz, whichever is slower), attempt a decode of the
  contiguous prefix and display whatever comes out.
- Draw a progress overlay: chunks received / total, bytes, elapsed, estimated
  remaining at 87.6 kB/s, and a small map of which chunks are missing.
- Decode failures during assembly are expected and must not be logged as
  errors.

---

## 7. Forward error correction

File transfers carry **XOR parity**: after each group of `fec_group_size`
chunks (default 16, reported at LRT offset 754), the payload sends one
`MEDIA_PARITY` chunk whose bytes are the XOR of that group.

This lets the host rebuild **any one missing or corrupt chunk per group with no
retransmission** — worth doing before falling back to `RESEND`, because a
resend costs another round trip and another HRT window.

```python
def indices_in_group(group, group_size, chunk_total):
    start = group * group_size
    return list(range(start, min(start + group_size, chunk_total)))

def repair(chunks, parities, chunk_total, group_size, chunk_size):
    """Fill single-chunk gaps in place; return the indices still missing."""
    for group, parity in parities.items():
        idx = indices_in_group(group, group_size, chunk_total)
        missing = [i for i in idx if i not in chunks]
        if len(missing) != 1:
            continue                     # 0 = nothing to do, 2+ = beyond parity
        acc = int.from_bytes(parity.ljust(chunk_size, b"\x00"), "big")
        for i in idx:
            if i in chunks:
                acc ^= int.from_bytes(chunks[i].ljust(chunk_size, b"\x00"), "big")
        chunks[missing[0]] = acc.to_bytes(chunk_size, "big")
    return [i for i in range(chunk_total) if i not in chunks]
```

Notes:

- `chunk_size` is 1256. Chunks are zero-padded to full width for the XOR; the
  final chunk of a file is short and is truncated by `size` at the end, so the
  padding is harmless.
- Two losses in one group cannot be repaired — use `RESEND`.
- **Live video carries no parity.** A late frame is worthless, and H.264
  recovers at the next keyframe.

---

## 8. Live video

### 8.1 Format

H.264 Annex-B, 640×480 at 15 fps and 600 kbit/s by default, one keyframe per
second. Each keyframe is preceded by SPS and PPS, so a decoder can start at any
keyframe.

### 8.2 Frame reassembly

`STREAM_DATA` uses `media_id` as the **frame number** and `chunk_total` as the
chunks in that frame. There is no `MEDIA_INFO` and no `MEDIA_END`: a stream has
no known length and no beginning the receiver is guaranteed to have seen, so
frames are self-describing.

```
on STREAM_DATA:
    verify data_crc32; drop the chunk if it fails
    partial[frame][chunk_index] = data
    if len(partial[frame]) == chunk_total:
        frame_bytes = concat chunks in index order
        emit the frame; delete partial[frame]
    expire entries in `partial` older than ~2 s — a frame whose chunks
    stopped arriving is never going to complete
```

### 8.3 Decoding

- **Discard everything before the first frame with the keyframe flag set.**
  Feeding a decoder inter frames with no reference produces garbage, and this
  is the most common "the video looks broken" cause.
- Feed frames in order to the decoder as they complete. Recommended: **PyAV**
  (`av.CodecContext.create("h264", "r")`). Fallback: pipe to an `ffmpeg`
  subprocess.
- Expect gaps. Frames are **deliberately dropped** by the payload when the link
  cannot keep up, so a missing frame number is normal, not an error. Display
  the delivered rate alongside the configured rate.
- Show `stream_frames_dropped` from telemetry; a rate rising faster than
  `stream_frames_sent` means the settings exceed the link.

### 8.4 Recording

Write the raw Annex-B byte stream, starting at the first keyframe, to
`stream/stream-<timestamp>.h264`. Offer a one-click remux to MP4:

```
ffmpeg -framerate <stream_fps> -i stream.h264 -c copy stream.mp4
```

### 8.5 Region of interest

The stream is a crop of `crop_w × crop_h` from the 4208×3120 sensor, centred on
`(centre_x, centre_y)`, scaled to `width × height`. When crop equals output the
pixels are native — full optical detail of that region.

The UI should present this as a **sensor-frame picker**: a 4208×3120 rectangle
with a draggable box, showing whether the current setting is native or scaled.
Values are clamped by the payload to keep the box on the sensor, and the applied
values come back in telemetry — display those, not the requested ones.

Limits: dimensions 64–1920 and even; fps 1–30; bitrate 50 000–8 000 000.

---

## 9. Storage slots

Captures address **numbered slots** — 16 by default — each holding one image or
one video. Slot 3 is slot 3 regardless of history, which is what makes a
repeatable test script possible.

| Kind | Value |
|---|---|
| empty | 0 |
| image | 1 |
| video | 2 |
| recording in progress | 3 |
| error — stored bytes failed their CRC | 4 |

Lifecycle: `SLOT_CAPTURE_IMAGE` or `SLOT_RECORD_START`/`SLOT_RECORD_STOP` →
`SLOT_DOWNLOAD` → `SLOT_DELETE`. **Deletion is what frees space.**

`SLOT_LIST` returns, little-endian:

```
u8  slot_count
u8  slots_used
then slot_count entries of 22 bytes:
    u8  index
    u8  kind
    u32 size
    u16 width
    u16 height
    u32 crc32
    f64 created_unix
```

`SLOT_INFO` returns one 26-byte entry: the same fields plus `f32 duration_s`.

Slot transfers use media ids `0x510000NN` where `NN` is the slot, so a slot
download is always distinguishable from a legacy media transfer.

> Recording is **not** streaming. `SLOT_RECORD_START` writes video to the Pi's
> storage; nothing crosses the link until it is downloaded.

---

## 10. Command reference

Full argument structures. **All arguments are little-endian.**

### 10.1 Telemetry

| Opcode | Name | Arguments | Effect |
|---|---|---|---|
| `0x01` | `PING` | — | liveness; returns uptime and version |
| `0x20` | `GET_TELEMETRY` | — | housekeeping snapshot in `resp_data` |
| `0x72` | `GET_LINK_STATS` | — | 8 × u32 receive counters |
| `0x22` | `GET_DOSE_LOG` | `start` f64, `end` f64 | dose history; zeros = all |

### 10.2 Configuration

| Opcode | Name | Arguments | Effect |
|---|---|---|---|
| `0x11` | `GET_CONFIG` | — | effective configuration |
| `0x10` | `SET_CONFIG` | `key` u8, `value` u32 | set one key |
| `0x50` | `SET_LED` | `percent` u8 | illumination; clamped to 10 |
| `0x77` | `SET_FEC_GROUP` | `group` u8 | parity group size; 0 disables |
| `0x73` | `SET_HRT_IDLE_FILL` | `enable` u8 | idle HRT packets |

### 10.3 Imaging

| Opcode | Name | Arguments | Effect |
|---|---|---|---|
| `0x30` | `CAPTURE_IMAGE` | — | capture to the legacy media store |
| `0x31` | `START_RECORD` | — | record to the legacy media store |
| `0x32` | `STOP_RECORD` | — | end that recording |
| `0x33` | `CAPTURE_REGION` | `x`,`y`,`w`,`h`,`out_w`,`out_h` all u16 | full-res crop, rescaled |

### 10.4 Slots

| Opcode | Name | Arguments | Effect |
|---|---|---|---|
| `0x64` | `SLOT_LIST` | — | the whole slot table |
| `0x65` | `SLOT_INFO` | `slot` u8 | one slot in detail |
| `0x66` | `SLOT_CAPTURE_IMAGE` | `slot` u8 | still into a slot |
| `0x67` | `SLOT_RECORD_START` | `slot` u8, `seconds` u16 | record into a slot; 0 = manual stop |
| `0x68` | `SLOT_RECORD_STOP` | — | finalise the recording |
| `0x6A` | `SLOT_DELETE` | `slot` u8 | free one slot |
| `0x6B` | `SLOT_DELETE_ALL` | — | free every slot |

### 10.5 Transfer

| Opcode | Name | Arguments | Effect |
|---|---|---|---|
| `0x69` | `SLOT_DOWNLOAD` | `slot` u8 | queue a slot for HRT |
| `0x6C` | `SLOT_DOWNLOAD_ABORT` | `slot` u8 | remove it from the queue |
| `0x71` | `ABORT_TRANSFERS` | — | empty the whole queue |
| `0x41` | `RESEND` | `media_id` u32, then `chunk` u32 × n | re-send specific chunks |
| `0x21` | `GET_MEDIA_LIST` | — | legacy media listing |
| `0x40` | `REQUEST_MEDIA` | `media_id` u32 | queue legacy media |
| `0x42` | `DELETE_MEDIA` | `media_id` u32 | delete legacy media |

### 10.6 Stream

| Opcode | Name | Arguments | Effect |
|---|---|---|---|
| `0x78` | `STREAM_START` | `width` u16, `height` u16, `fps` u8, `bitrate` u32, `centre_x` u16, `centre_y` u16, `crop_w` u16, `crop_h` u16 | start live video |
| `0x79` | `STREAM_STOP` | — | stop it |
| `0x7A` | `STREAM_SET_REGION` | `centre_x`, `centre_y`, `crop_w`, `crop_h` u16 | aim the crop box |
| `0x7B` | `STREAM_SET_OUTPUT` | `width` u16, `height` u16, `fps` u8, `bitrate` u32 | resolution and rate |

All four reply with the **effective** settings after clamping:
`width` u16, `height` u16, `fps` u8, `bitrate` u32, `centre_x` u16,
`centre_y` u16, `crop_w` u16, `crop_h` u16.

`STREAM_START` accepts 0, 9 or 17 argument bytes: none keeps current settings,
9 sets output only, 17 sets output and region.

### 10.7 Link and calibration

| Opcode | Name | Arguments | Effect |
|---|---|---|---|
| `0x70` | `CLEAR_SAFE_MODE` | — | resume normal operation |
| `0x62` | `EEPROM_STATUS` | — | which calibration copies verify |
| `0x63` | `EEPROM_REPAIR` | — | rewrite from a good copy |
| `0x60` | `EEPROM_READ` | `offset` u16, `length` u16 | read calibration, ≤128 bytes |

---

## 11. Codes and enumerations

### 11.1 Result codes (`last_result`)

| Code | Name | Meaning |
|---|---|---|
| 0 | OK | success |
| 1 | `BAD_CRC` | checksum failed |
| 2 | `BAD_TYPE` | unknown opcode |
| 3 | `BAD_PARAM` | malformed arguments |
| 4 | `BUSY` | queue full, already recording, or safe mode |
| 5 | `NO_MEDIA` | no such file or slot content |
| 6 | `CAMERA_FAULT` | camera unavailable |
| 7 | `STORAGE_FULL` | no space |
| 8 | `BITRATE_EXCEEDS_LINK` | video settings exceed the link |
| 9 | `NOT_CALIBRATED` | calibration required |
| 10 | `EEPROM_FAULT` | calibration memory error, **or a slot failing its CRC** |
| 11 | `REGION_INVALID` | crop outside the sensor |
| 12 | `WRITE_PROTECTED` | EEPROM writes locked |

### 11.2 Event codes

| Code | Name | | Code | Name |
|---|---|---|---|---|
| `0x0001` | `BOOT` | | `0x0032` | `TRANSFER_ABORTED` |
| `0x0002` | `SAFE_MODE_ENTERED` | | `0x0033` | `HRT_GO` |
| `0x0003` | `SAFE_MODE_CLEARED` | | `0x0034` | `HRT_STOP` |
| `0x0010` | `COMMAND_ACCEPTED` | | `0x0035` | `HRT_STOP_WITH_LOSS` |
| `0x0011` | `COMMAND_REJECTED` | | `0x0040` | `RX_CRC_BURST` |
| `0x0012` | `COMMAND_FAILED` | | `0x0041` | `TX_FAILED` |
| `0x0013` | `COMMAND_DUPLICATE` | | `0x0050` | `TMR_CORRECTED` |
| `0x0020` | `CAPTURE_OK` | | `0x0051` | `TMR_UNRECOVERABLE` |
| `0x0021` | `CAPTURE_FAILED` | | `0x0052` | `EEPROM_DEGRADED` |
| `0x0022` | `RECORD_STARTED` | | `0x0053` | `EEPROM_REPAIRED` |
| `0x0023` | `RECORD_STOPPED` | | `0x0060` | `DOSIMETER_FAULT` |
| `0x0030` | `TRANSFER_STARTED` | | `0x0061` | `CAMERA_FAULT` |
| `0x0031` | `TRANSFER_COMPLETE` | | `0x0062` | `LED_FAULT` |
| | | | `0x0070` | `WATCHDOG_LATE` |

---

## 12. Operating procedure

### 12.1 The acknowledgement means *accepted*, not *done*

The 8-byte Command ACK has **no status field**. It confirms a valid command
addressed to the payload arrived — nothing more. A capture takes seconds, and
its outcome appears later in telemetry.

**Required host behaviour:**

1. Send the command with a fresh `cmd_seq`; mark it *pending*.
2. On ACK, mark it *accepted*. Do **not** show success.
3. Watch `last_cmd_seq` in each telemetry packet. When it matches, read
   `last_result` and `resp_data`, and mark the command *succeeded* or *failed*.
4. If no ACK within ~200 ms, retransmit **the same `cmd_seq`** — the payload
   suppresses duplicates, so this is safe.
5. If no result within a timeout (5 s default, 30 s for capture and record),
   mark it *timed out* and say so.

The UI must show all four states distinctly. Reporting success on the ACK would
be wrong and would hide every failure.

### 12.2 Telemetry polling

Send `LRT_REQUEST` on a timer. 2–5 Hz is a good default: each reply is 1256
bytes and takes 13.7 ms on the wire, so 5 Hz costs about 7% of the link.

**Do not poll while an HRT burst is in flight if latency matters** — a 1288-byte
packet holds the wire for 14 ms and the reply queues behind it. The scheduler
should interleave: send at most a few HRT-window packets' worth of time between
polls.

Raise the rate while a command is pending to shorten result latency; drop it
during a bulk transfer to leave the link free.

### 12.3 HRT flow control

Nothing bulk arrives until `HRT_GO`. **HRT starts closed after every payload
reset**, so the host must send `HRT_GO` again after a reboot — detect this by
watching `boot_count`.

| | Effect |
|---|---|
| `HRT_STOP` (0x85) | lets the packet in flight finish; the last packet is whole and valid |
| `HRT_STOP_WITH_LOSS` (0x86) | truncates the packet in flight; it will fail CRC at the host and must be discarded |

After a stop-with-loss, expect **exactly one malformed packet** — that is the
commanded outcome, not a fault, and should not be counted as a link error. The
payload rewinds the truncated file chunk so it is sent again; a truncated video
frame is abandoned.

Recommended default: use `HRT_STOP`. Use stop-with-loss only when the host
needs the wire immediately.

### 12.4 Safe mode

Five consecutive command failures trip safe mode: HRT stops and most commands
are refused with `BUSY`, but **telemetry keeps answering**. The UI should make
this unmissable and offer `CLEAR_SAFE_MODE`. Commands still accepted in safe
mode: `CLEAR_SAFE_MODE`, `GET_LINK_STATS`, `ABORT_TRANSFERS`, `PING`,
`GET_TELEMETRY`, `GET_CONFIG`.

### 12.5 A typical session

```
1  connect, verify the latency timer
2  LRT_REQUEST at 2 Hz; confirm target 0xC7 and note boot_count
3  SLOT_LIST                       see what is stored
4  SLOT_CAPTURE_IMAGE slot=0       take a picture
5  poll until last_cmd_seq matches and last_result == 0
6  SLOT_DOWNLOAD slot=0            queue it
7  HRT_GO                          receive it; watch it assemble
8  HRT_STOP                        close the tap
9  verify file_crc32; save to the session folder
10 SLOT_DELETE slot=0              free the slot
```

---

## 13. Application architecture

### 13.1 Stack

| Concern | Choice | Why |
|---|---|---|
| Language | Python 3.11+ | matches the payload tooling |
| UI | **PySide6** (Qt 6) | real widgets, fast custom painting, good video surface |
| Serial | `pyserial` | FTDI VCP driver presents a normal COM port |
| Video decode | **PyAV** | in-process H.264, no subprocess plumbing |
| Imaging | `Pillow`, `numpy` | progressive JPEG preview, frame conversion |
| Packaging | `PyInstaller` | single-file .exe for a bench machine |
| Tests | `pytest` | with a payload simulator, no hardware |

### 13.2 Threads

Four, with queues between them. **Nothing that touches the serial port may run
on the UI thread.**

```
  ┌────────────┐   bytes    ┌────────────┐  packets  ┌────────────┐
  │  Reader    │──────────▶│  Parser    │─────────▶│ Dispatcher │
  │  thread    │           │  thread    │          │  thread    │
  └────────────┘           └────────────┘          └─────┬──────┘
        ▲                        │                        │ signals
        │ frames                 │ raw log                ▼
  ┌─────┴──────┐           ┌─────▼──────┐          ┌────────────┐
  │  Writer    │◀──────────│  Recorder  │          │  Qt UI     │
  │  thread    │  commands │            │          │  (main)    │
  └────────────┘           └────────────┘          └────────────┘
```

- **Reader** — blocking `read()` with a short timeout, into a byte queue. Never
  blocks on anything else. Timestamps each read.
- **Parser** — sync hunt, length, CRC, target filter ([§3.7](#37-receive-framing)).
  Emits decoded packets. Maintains resync and error counters.
- **Dispatcher** — routes packets: telemetry to the model, HRT to the transfer
  and stream assemblers, ACKs to the pending-command table. Owns reassembly
  state.
- **Writer** — owns all transmission: command queue, the telemetry poll timer,
  HRT flow control. Single owner means two parts of the UI can never transmit
  at once.
- **Recorder** — appends the raw byte stream and the decoded packet index.
- **UI** — Qt main thread. Receives via signals, never blocks.

Backpressure: bound every queue. If the UI cannot keep up, **drop display
updates, never received data** — the recorder must always win.

### 13.3 Modules

```
radcam_host/
    protocol/
        crc.py            CRC-16/CCITT-FALSE and CRC-32
        packets.py        the six packet types, encode and decode
        commands.py       105-byte payload; the command catalogue
        lrt.py            telemetry decode -> dataclass
        hrt.py            bulk payload decode
        fec.py            parity repair
        timebase.py       GPS epoch conversion
    link/
        serial_link.py    port handling, reader thread, latency check
        framer.py         resynchronising receiver
        session.py        the master state machine, poll timer, flow control
    assembly/
        transfers.py      file reassembly, progressive preview, resend
        stream.py         frame reassembly and decode
    storage/
        recorder.py       raw capture and decoded index
        session_dir.py    folder layout
    offline/
        replay.py         rebuild everything from a saved capture
    ui/
        main_window.py
        dashboard.py      vitals
        commands_panel.py buttons
        video_panel.py
        transfer_panel.py progressive image view
        events_panel.py
        sensor_picker.py  crop-box widget over the 4208x3120 frame
        offline_panel.py
    sim/
        payload_sim.py    a fake payload for development without hardware
```

Keep `protocol/` **pure**: no I/O, no Qt. It is the part worth unit-testing
exhaustively and reusing in the offline tool.

---

## 14. User interface

### 14.1 Layout

```
┌──────────────────────────────────────────────────────────────────────────┐
│ COM4  921600  target 0xC7   ● LINKED   latency 1ms ✓   [Connect]         │
├───────────────────────────┬──────────────────────────────────────────────┤
│ VITALS                    │  LIVE VIDEO                                  │
│  uptime   04:12:33        │  ┌────────────────────────────────────────┐  │
│  boot #   7               │  │                                        │  │
│  dose     0.0731 rad      │  │            640x480 @ 15 fps            │  │
│  temp     51.3 C          │  │                                        │  │
│  camera   OK              │  └────────────────────────────────────────┘  │
│  safe     NO              │  state RUNNING   14.9 fps   587 kbit/s       │
│                           │  sent 1204  dropped 13   [Start][Stop][ROI]  │
│ SLOTS  3/16 used          ├──────────────────────────────────────────────┤
│  0 IMG 2.4MB  [DL][DEL]   │  TRANSFER                                    │
│  1 VID 8.1MB  [DL][DEL]   │  ┌────────────────────────────────────────┐  │
│  2 --                     │  │      (image assembling in place)       │  │
│  3 REC 00:12              │  └────────────────────────────────────────┘  │
│  ...                      │  slot 0   812/1993 chunks   41%   24s left   │
│                           │  missing: 3   CRC ok                         │
│ LINK                      ├──────────────────────────────────────────────┤
│  rx good     18422        │  COMMANDS                                    │
│  bad crc     2            │  [Capture→slot▾] [Record▾] [Stop rec]        │
│  resyncs     0            │  [Download▾] [Delete▾] [Delete all]          │
│  HRT   ● OPEN             │  [Stream start] [Stream stop] [Set output]   │
│  [GO][STOP][STOP+LOSS]    │  [Ping] [Get config] [Clear safe mode]       │
├───────────────────────────┴──────────────────────────────────────────────┤
│ EVENTS   14:31:02 CAPTURE_OK slot=0    14:31:09 TRANSFER_STARTED         │
│ COMMANDS 0x66 seq=41 ✓ 1.2s   0x69 seq=42 ⧗ pending                     │
└──────────────────────────────────────────────────────────────────────────┘
```

### 14.2 Panels

**Connection** — port picker, baud, target ID, connect/disconnect, link state,
and the measured poll-to-reply latency with a warning if the FTDI timer looks
wrong.

**Vitals** — everything from [§5](#5-lrt-telemetry-payload) worth watching
live. Colour by state: safe mode red, camera fault amber, stream gated amber.
Show the age of the last telemetry packet; grey the panel if it exceeds ~2 s.

**Slots** — a row per slot: index, kind, size, dimensions, age, and per-row
Download and Delete buttons. Highlight the recording slot with elapsed time and
the downloading slot with progress. This is the primary work surface.

**Commands** — grouped buttons. Anything taking a slot index or a value opens a
small inline form rather than a modal. Every button shows a tooltip with the
opcode and the exact argument bytes that will be sent.

**Command log** — every command with `cmd_seq`, state (pending / accepted /
succeeded / failed / timed out), round-trip time, and the decoded response.
This is where an operator diagnoses a misbehaving payload.

**Video** — the decoded stream, with delivered fps and bitrate against
configured. A `GATED` badge when the stream is running but HRT is closed. Start
/ Stop, and an ROI editor.

**Sensor picker** — a 4208×3120 rectangle with a draggable, resizable crop box,
snapping to even dimensions, showing "native 1:1" or the scale factor. Sends
`STREAM_SET_REGION` on release, and redraws from the **effective** values in
telemetry, which may be clamped.

**Transfer** — the progressive image, chunk progress, missing-chunk map, a
Resend button for what parity could not fix, and CRC status on completion.

**Events** — the decoded ring, de-duplicated, colour-coded by severity,
filterable, exportable.

**Offline** — pick a capture file, replay it, list what was recovered, export.

### 14.3 Rules

- Every transmission is logged and visible. Nothing is sent invisibly.
- Destructive commands (`SLOT_DELETE`, `SLOT_DELETE_ALL`, `ABORT_TRANSFERS`)
  confirm first.
- Disable controls that cannot work — `SLOT_DOWNLOAD` on an empty slot, stream
  controls with no camera — and say why in the tooltip.
- The UI never blocks on I/O. A command click enqueues and returns.
- Telemetry values older than ~2 s are visually stale, not silently wrong.

---

## 15. On-disk layout

One folder per session, created on connect:

```
sessions/2026-08-21T14-30-00Z/
    session.json           port, baud, target, app version, start/end, counters
    raw/
        capture.bin        every byte received, verbatim — the authority
        rx.jsonl           one record per decoded packet
        tx.jsonl           one record per packet sent
    telemetry/
        lrt.csv            one row per telemetry packet, all fields
        lrt.jsonl          the same, structured
    events/
        events.csv         de-duplicated event ring entries
    images/
        slot00_2026-08-21T14-32-10Z.jpg
        slot00_2026-08-21T14-32-10Z.json      metadata + verification
    video/
        slot01_2026-08-21T14-40-02Z.h264
        slot01_2026-08-21T14-40-02Z.mp4       if remuxed
    stream/
        stream_2026-08-21T14-35-00Z.h264
        stream_2026-08-21T14-35-00Z.mp4
        frames/                                optional PNG dumps
    logs/
        app.log
```

`capture.bin` is written **before any parsing**, so a decoder bug can never
cost the data. It is the input to the offline tool.

`rx.jsonl` records:

```json
{"t": 1755782400.123, "off": 918234, "len": 1288, "type": 135,
 "ok": true, "kind": "HRT_DATA",
 "fields": {"sub_type": 5, "media_id": 4211, "chunk_index": 0,
            "chunk_total": 4, "data_len": 1256, "crc_ok": true}}
```

`off` is the byte offset into `capture.bin`, so any record can be traced back
to its exact bytes.

Every saved image and video gets a sidecar JSON: source slot, media id, size,
`file_crc32` advertised and computed, chunks received, chunks repaired by
parity, chunks resent, and the transfer wall time.

---

## 16. Offline assembly

**Requirement R8.** Point the tool at a saved capture and it rebuilds
everything, with no hardware and no live session.

```
radcam-host replay --input capture.bin --output ./recovered
```

It must:

1. Run the **same parser** as the live path — a separate implementation would
   drift.
2. Reconstruct every file transfer: `MEDIA_INFO` → chunks → parity repair →
   `MEDIA_END` → CRC verification.
3. Reconstruct the video stream, starting at the first keyframe, and remux to
   MP4 if `ffmpeg` is present.
4. Rebuild the telemetry and event series to CSV.
5. Emit a report: packets by type, CRC failures, resyncs, transfers found,
   transfers completed, chunks repaired by parity, chunks still missing.

**Partial recovery is required, not optional.** A capture cut short mid-transfer
must still yield everything complete plus a truncated best-effort image, clearly
labelled. A transfer with holes parity could not fill should be written with the
gaps zero-filled and its sidecar marked incomplete, listing the missing chunk
indices — a partly-recovered image is often still worth looking at.

Also accept a directory of captures and process them in order, so a long test
split across sessions reassembles as one.

---

## 17. Timing and budgets

Measured on the payload hardware; the host should size buffers and timeouts
against these.

| Quantity | Value |
|---|---|
| Character time | 10.85 µs |
| Command ACK, 8 B | 0.11 ms |
| LRT Data, 1256 B | 13.66 ms |
| HRT Data, 1288 B | 14.01 ms |
| HRT packets/s | 71.4 |
| HRT payload throughput | **87.6 kB/s** (717 kbit/s) |
| Line utilisation, sustained | 99.8% |

**File transfer times** (including `MEDIA_INFO`, `MEDIA_END` and parity):

| Size | Chunks | Time |
|---|---:|---:|
| 100 kB | 82 | 1.3 s |
| 1 MB | 835 | 12.5 s |
| 3 MB | 2505 | 37.3 s |
| 5 MB | 4175 | 62.2 s |

**Live video** at 640×480, 15 fps, 600 kbit/s: ~4.4 HRT chunks per frame,
~61 ms transmit per frame against a 66.7 ms interval — **92% link occupancy**.
Keyframes burst to ~231% of one interval and are absorbed by the payload's
frame ring. 700 kbit/s at that size **exceeds the link** and produces continuous
frame drops.

Recommended timeouts: ACK 200 ms with retry; command result 5 s, or 30 s for
capture and record; telemetry stale after 2 s; partial video frames expire
after 2 s.

---

## 18. Testing

### 18.1 Payload simulator

Ship `sim/payload_sim.py`: a fake payload that speaks the protocol over a
virtual COM pair (com0com on Windows, `pty` on Linux). It must reproduce the
behaviours the host has to handle:

- ACK, then a result appearing in telemetry some time later
- HRT gated by go/stop, with **nothing** transmitted while closed
- `HRT_STOP_WITH_LOSS` emitting a deliberately truncated packet
- File transfers with parity, and injectable chunk loss
- A synthetic video stream with keyframes and deliberate frame drops
- Duplicate `cmd_seq` suppression
- Safe mode after repeated failures
- Corrupt packets and random line noise

The whole application must be developable and demonstrable against the
simulator alone.

### 18.2 Unit tests

- CRC against the check value `0x29B1`.
- Encode/decode round trip for all six packet types.
- The framer: leading garbage, split reads, a bad CRC followed by a good
  packet, unknown types, a foreign target ID, buffer overflow.
- Parity repair: one loss per group repairs; two do not; a short final chunk
  reconstructs exactly.
- Telemetry decode against a captured golden packet.
- Endianness: a test that would fail if arguments were encoded big-endian.

### 18.3 Integration

Run against the simulator: full capture → download → verify → delete; a
transfer interrupted by stop and resumed; a stream started and stopped; a
session recorded and then replayed offline to produce byte-identical images.

### 18.4 Hardware acceptance

With a real payload: confirm the latency-timer warning triggers at 16 ms and
clears at 1 ms; sustain a 5-minute stream and confirm delivered fps matches
telemetry; download a multi-megabyte slot and confirm the CRC; pull the cable
mid-transfer and confirm the host recovers without losing the session.

---

## 19. Test vectors

Target ID `0xC7`. Validate an encoder against these before going near hardware.

**CRC-16/CCITT-FALSE** of `"123456789"` = `0x29B1`.

**Request packets, 14 bytes:**

```
LRT_REQUEST         1ACFFC1D00000000000081C7B03C
HRT_GO              1ACFFC1D00000000000087C71A9A
HRT_STOP            1ACFFC1D00000000000085C77CF8
HRT_STOP_WITH_LOSS  1ACFFC1D00000000000086C729AB
```

**Commands, 120 bytes** (`cmd_seq` = 1, force flag set, timestamp zero,
trailing zero padding elided — the full packet is 240 hex characters):

```
PING                1ACFFC1D00000000000010C701000100019C4C 00...00 3810
SLOT_CAPTURE_IMAGE  1ACFFC1D00000000000010C76600010101AE5C 00...00 841D   (slot=0)
SLOT_DOWNLOAD       1ACFFC1D00000000000010C769000101016B5F 00...00 1100   (slot=0)
SLOT_DELETE         1ACFFC1D00000000000010C76A00010101A5BF 00...00 3FF9   (slot=0)
```

Reading `PING` field by field: sync `1ACFFC1D`, coarse `00000000`, fine `0000`,
type `10`, target `C7`, then the 105-byte payload beginning opcode `01`,
`cmd_seq` `0001`, `arg_len` `00`, `flags` `01`, inner CRC `9C4C`, then 98 zero
bytes, spare `00`, envelope CRC `3810`.

**LRT Data**, first 32 bytes of a packet whose payload has
`uptime_s = 0x11223344`, `target_id = 0xC7`, `last_cmd_seq = 7`:

```
1ACFFC1D 81 C7 | 0001 0000 00000000 0000 11223344 00000000 00C7 01 00 00 0007 00
   sync   ty tg | ver  flag coarse   fine uptime   boot     tid  Mj mn op seq  res
```

**HRT Data**, a `MEDIA_DATA` chunk carrying `"RADCAM"` from slot 3
(media id `0x51000003`), chunk 2 of 9, last-chunk flag set:

```
1ACFFC1D 87 C7 | 0002 0001 51000003 00000002 00000009 0006 0000 12B2C63C 52414443414D
   sync   ty tg | sub  flag media_id chunk_ix chunk_tot len  rsvd crc32    "RADCAM"
```

---

## 20. Acceptance criteria

| # | Criterion |
|---|---|
| A1 | Connects at 921600 and warns if the FTDI latency timer is not ~1 ms |
| A2 | Sustains 5 Hz telemetry polling for an hour with zero parser resyncs on a good cable |
| A3 | Every command in [§10](#10-command-reference) is reachable from the UI and shows all four states |
| A4 | Live video plays with under 500 ms of glass-to-glass latency, starting from a keyframe |
| A5 | A slot download shows the image assembling progressively and verifies `file_crc32` |
| A6 | A transfer interrupted by `HRT_STOP` resumes and completes when reopened |
| A7 | Parity repairs single-chunk losses without a resend; the count is displayed |
| A8 | Every byte received is written to `capture.bin` before parsing |
| A9 | Offline replay of a capture reproduces byte-identical images to the live session |
| A10 | A stop-with-loss produces exactly one malformed packet, not counted as a link fault |
| A11 | The UI stays responsive during a sustained transfer and a stream at once |
| A12 | Safe mode is unmissable and clearable from the UI |

---

## 21. Traps

Every one of these has already cost time on the payload side.

1. **The FTDI latency timer defaults to 16 ms.** Set it to 1. Nothing else on
   this list will matter until you do.
2. **Two endiannesses.** Envelope and payload headers big-endian; command
   arguments and response data little-endian.
3. **The ACK does not mean success.** It has no status field.
4. **Silence is normal.** The payload transmits only when asked.
5. **HRT starts closed** after every payload reset. Watch `boot_count`.
6. **Packet type alone does not identify a packet.** Use direction and length.
7. **A CRC failure must resynchronise, not consume a packet's worth of bytes.**
8. **Feed the decoder from a keyframe.** Anything earlier decodes to garbage.
9. **Dropped video frames are by design.** A gap in frame numbers is not a bug.
10. **One malformed packet after a stop-with-loss is the commanded outcome.**
11. **Deleting a slot is what frees space.** Storage does not recycle itself.
12. **A canned hex string with the force flag bypasses duplicate suppression.**
    The application should generate incrementing sequence numbers instead.
13. **Write raw bytes before parsing.** A decoder bug should never cost a test
    run.
