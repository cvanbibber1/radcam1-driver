# radcam-host — Update Specification

**Applies to:** any host built against `docs/host/HOST_APP_SPEC.md` (the 1417-line
base spec). This document is a **delta**, not a replacement: everything the base
spec says still holds except where contradicted here.

**Two changes drive it:**

1. **Multi-camera switching.** The payload can carry more than one camera and
   switches between them under an interlock. The base spec has no concept of
   this at all — and, more urgently, the telemetry layout moved to make room.
2. **CRC and RS-422 settings in the GUI.** The payload's wire settings are now
   fully configurable. The host must be able to match them without a rebuild,
   and must be able to *discover* them when they disagree.

---

## 0. Read this first — there is a breaking change

**The LRT telemetry layout moved.** A host built to the base spec will misparse
every telemetry packet from current firmware, and it will do so **silently** —
the envelope CRC passes, the payload CRC-32 passes, and the event ring decodes
into plausible-looking garbage.

| Field | Base spec | Current firmware |
|---:|---:|---:|
| `camera_count` | — | **767** |
| `camera_active` | — | **768** |
| `camera_selections` | — | **769** (u16) |
| `camera_select_failures` | — | **771** (u16) |
| *(spare)* | — | 773–775 |
| `event_count` | 770 | **776** |
| events | 772 | **778** |
| `MAX_EVENTS` | 39 | **38** |

Offsets 770 and 772 — where the base spec reads `event_count` and the first
event — now sit **inside the camera block**. A host reading `event_count` at 770
reads the low half of `camera_selections` instead, which for a payload that has
never switched cameras is `0`, so the events panel simply goes quiet. That is
the worst possible failure: it looks like a payload with nothing to report.

**Fix this before anything else in this document.** Everything else is a
feature; this is a correctness bug against live firmware.

### 0.1 Version the layout, do not hard-code it

The payload reports its own version, and the host must use it rather than
assuming one layout forever:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 2 | `version` — LRT payload layout version |
| 20 | 1 | `fw_major` |
| 21 | 1 | `fw_minor` |

Restructure `protocol/lrt.py` so offsets come from a **layout table selected by
`version`**, not from module-level constants:

```python
LAYOUTS = {1: LAYOUT_V1, 2: LAYOUT_V2}      # V2 has the camera block

def decode(payload: bytes) -> Telemetry:
    version = int.from_bytes(payload[0:2], "big")
    layout = LAYOUTS.get(version)
    if layout is None:
        raise UnknownLayout(version)         # surface it; do not guess
```

An unknown version must be **reported in the UI, not guessed at**. Decoding
against the wrong layout is how the failure above happens. Decode the fields
that are version-stable (everything below offset 717 is unchanged), show the
rest as unavailable, and say plainly that the host is older than the payload.

This matters beyond today: `docs/stp/MULTI_CAMERA_SPEC.md` §6.1 proposes a
16-byte thermal block that **shortens the event ring again, 38 → 36**. A host
with a version-keyed layout table absorbs that as one new dict entry. A host
with hard-coded constants breaks a second time.

---

## 1. The camera model

### 1.1 One target, many cameras

**The Target ID stays `0xC7` for the entire chain.** Every camera in the
payload answers at that one address; the camera index is a field *inside* our
own payload, invisible to DICE and to the other experiments on the bus. A
second Target ID would make the payload answer to an address the mission
assigned to someone else.

**The host must never expose a per-camera target ID.** There is one connection,
one address, one payload — with a camera selector inside it.

### 1.2 Exactly one camera at a time

Two sensors sharing an I2C address answer together; two driving the same CSI
lanes contend. So the payload does not offer "enable camera N" — only
`SELECT_CAMERA`, which disables everything and then enables one. **This is an
interlock, not a preference.** The UI must present it as a radio group or a
single-select dropdown. Checkboxes would imply a state the payload cannot
enter.

### 1.3 Always-on cameras

A camera's enable line may not be the payload's to switch. On the current board
the AR1335 sits on the CAM1 connector and its enable is held by the kernel as
`cam1_reg`, so it is powered at boot and cannot be turned off — reported by
**flags bit 2** in the camera table.

Consequences the host must implement:

- An always-on camera is reported as **active** even though nothing selected it.
- Selecting **any other** camera while an always-on camera exists is **refused**
  by the payload with `CAMERA_FAULT`, because the always-on one cannot be turned
  off and the result would be exactly the contention the interlock prevents.
- So the host must **disable those options and say why** rather than letting the
  operator send a command that is guaranteed to fail. Tooltip: *"cam0 (AR1335)
  is always on and cannot be disabled; the payload must be reconfigured before
  another camera can be selected."*

For the current single-camera test build this means the camera selector renders
one entry, shown active, with switching disabled. That is correct, not broken,
and the UI should distinguish it from "no cameras".

---

## 2. New commands

### 2.1 `CAMERA_LIST` — `0x6E`

No arguments. Reply arrives in the LRT response window (§5.7 of the base spec),
**little-endian**, variable length:

```
u8  count           number of cameras configured
u8  active          index of the enabled camera, 0xFF for none
then per camera:
u8  index
u8  gpio            enable line
u8  flags           bit0 active-high, bit1 currently enabled,
                    bit2 always-on (not switchable)
i8  i2c_bus         -1 (0xFF) when the camera has no I2C bus of its own
u8  name_len        ≤ 16
..  name            ASCII, name_len bytes
```

Entries are **variable length** — parse by walking, never by fixed stride.

Worked example, the current single-camera build:

```
01 00 | 00 30 07 04 06 41 52 31 33 33 35
 │  │    │  │  │  │  │  └ "AR1335"
 │  │    │  │  │  │  └ name_len 6
 │  │    │  │  │  └ i2c-4
 │  │    │  │  └ flags 0x07 = active-high | enabled | always-on
 │  │    │  └ GPIO 48
 │  │    └ index 0
 │  └ active = camera 0
 └ count = 1
```

And a two-camera build with both switchable:

```
02 00 | 00 05 03 04 06 "AR1335" | 01 06 01 FF 07 "THERMAL"
```

Note `i2c_bus` `0xFF` = −1 (no bus) on the second entry, and flags `0x01` —
active-high, not currently enabled, not always-on.

### 2.2 `SELECT_CAMERA` — `0x6D`

| Argument | Type | Meaning |
|---|---|---|
| `camera` | u8 | camera index 0–15, or `0xFF` to disable all |

Response (little-endian): `u8 selected, u8 count, u8 gpio`.

`0xFF` — all cameras off — is a **legitimate low-power state**, not an error,
and the UI should offer it as such rather than treating it as a failure.

**Side effects the operator must be warned about before the command is sent:**

- A **running stream is stopped**, and the partially-received frame is dropped.
- An **in-progress recording is ended** (not discarded — the slot keeps what was
  recorded up to that point).

Switching the sensor out from under either would leave them pointing at hardware
that is no longer there. The host must therefore **confirm** a camera switch
whenever `stream_state != 0` or `slot_recording != 0xFF`, naming what will stop.

**Failure:** `CAMERA_FAULT`. On failure the payload also posts a `CAMERA_FAULT`
event with the requested index as `arg`. The likely causes are: the index is not
configured, its enable line is held elsewhere, or an always-on camera blocks the
switch — and the host should say which by checking the camera table it already
has, rather than reporting a bare code.

### 2.3 Result and event codes

No new result codes. `CAMERA_FAULT` (result code 6, event `0x0061`)
gains the camera index as its `arg`, so the events panel should render it as
`CAMERA_FAULT cam=1` rather than a raw number.

---

## 3. UI changes for cameras

### 3.1 Camera selector

A new panel, or a strip at the top of the video panel:

```
CAMERA   (•) 0  AR1335   GPIO48  i2c-4   ALWAYS ON
         ( ) 1  THERMAL  GPIO5           [ disabled — cam0 cannot be turned off ]
         ( ) none (all off, low power)
                                              [Refresh]  [Apply]
```

Rules:

- Populate from `CAMERA_LIST`, refresh on connect and after every successful
  `SELECT_CAMERA`.
- The **live active camera comes from telemetry** (`camera_active`, offset 768),
  not from what the host last requested — a selection can fail, and the panel
  must show what is true, not what was asked for.
- Disable entries that cannot be selected and put the reason in the tooltip.
- Confirm when a stream or recording will be stopped.
- Show `camera_select_failures` (offset 771) somewhere in the link/vitals panel:
  a rising count is the signature of a switch that keeps being refused.

### 3.2 Everything downstream of "which camera"

- **Sensor picker.** The crop-box widget is hard-coded to 4208×3120. That is the
  AR1335; a thermal sensor is 640×512. It must take its dimensions from the
  active camera, and it must **redraw when the camera changes**, or the operator
  will be dragging a box over a sensor that does not exist.
- **Slots panel.** Add a camera column. §4.1 of the multi-camera spec proposes
  per-slot camera tagging; until that lands, the host should record the active
  camera at capture time locally, so a session's media is still attributable.
- **Video panel.** Show which camera the stream is coming from, in the same
  place as the fps and bitrate readout.

### 3.3 Session directory

Media from different cameras must not land in one undifferentiated folder — a
16-bit radiometric frame and an 8-bit JPEG are not interchangeable, and telling
them apart afterwards by file size is not a plan.

```
session-2026-09-02T13-40-00/
    cam0-AR1335/
        slots/  stream/  stills/
    cam1-THERMAL/
        slots/  stream/  radiometric/
    telemetry.jsonl
    events.jsonl
    raw.bin
    wire-settings.json        <- see §4.6
```

`telemetry.jsonl` gains `camera_count`, `camera_active`, `camera_selections`
and `camera_select_failures`. The offline replay tool (§16 of the base spec)
must reproduce the same split, which means the **raw capture has to be enough to
reconstruct which camera was active** — it is, since every LRT packet carries
`camera_active`.

---

## 4. Wire settings in the GUI

The payload's CRC, target id, coverage window, byte order and baud are now
configuration rather than code (`tools/rs422-tweak.py`, `docs/rs422tweak.md`).
The host must match them, and the base spec hard-codes all of it.

### 4.1 Why this is worth real UI

**Every one of these settings fails silently and symmetrically.** With the wrong
CRC the payload transmits perfectly formed packets at the right baud with the
right framing, its own logs count every byte out, and the host discards all of
it — while the host's own commands are discarded at the far end for the same
reason. Neither side reports an error. The operator sees "no data" and has no
way to distinguish it from a dead payload, a wrong port, or a wiring fault.

A settings dialog turns a multi-hour bring-up into a dropdown.

### 4.2 The settings panel

```
WIRE SETTINGS                                    [Load profile ▾] [Save]

  Port        [COM4      ▾]     Baud   [921600  ▾]
  Target ID   [0xC7]            Structures  (•) big-endian ( ) little-endian
  CRC variant [CRC-16/CCITT-FALSE ▾]
  ┌ custom parameters ──────────────────────────────────────────────┐
  │ poly [0x1021]  init [0xFFFF]  xor-out [0x0000]                  │
  │ [ ] reflect in   [ ] reflect out    store (•) big ( ) little    │
  └─────────────────────────────────────────────────────────────────┘
  Coverage    from byte [4] to the CRC        LRT trailer [crc ▾]

  Check value CRC16("123456789") = 0x29B1  ✓
                                    [Apply]  [Test against payload]  [Solve…]
```

Mirror `tools/rs422-tweak.py` exactly, so an operator reading `rs422tweak.md`
sees the same names:

| Field | Config key | Notes |
|---|---|---|
| CRC variant | `crc_variant` | the 15 named variants, plus **custom** |
| poly / init / xor-out | `crc_poly` `crc_init` `crc_xor_out` | accept `0x1021`, `1021h`, decimal |
| reflect in / out | `crc_reflect_in` `crc_reflect_out` | |
| store | `crc_store` | **separate from the algorithm** — CCITT-FALSE stored little-endian is still CCITT-FALSE, and confusing the two is a classic first-contact failure |
| coverage start | `crc_start` | default 4 (past the sync word); `0` if the far end covers sync |
| structures | `big_endian` | ICD envelope order. **Command args and response blobs stay little-endian regardless** — that is a deliberate convention, not a setting |
| LRT trailer | `lrt_trailer` | `crc` always; `zero` exists only for testing a non-conforming peer |
| Target ID | `target_id` | `0xC7`, mission-assigned |

Show the live **check value** as the operator edits. `CRC16("123456789")` is
computed instantly and catches most typos before a packet is ever sent.

### 4.3 One setting, both directions

The chosen parameters must apply to **transmit and receive together**. A host
that validated received packets under new settings while still building commands
under the old ones would appear to work — telemetry flows — while no command was
ever accepted. Route everything through one `WireConfig` object owned by the
session, and construct the encoder and the decoder from the same instance.

### 4.4 Detecting a disagreement, and solving it

The host already counts `rx_bad_crc`. Add the inference:

> **If packets are syncing but failing CRC at a high rate, the CRC parameters are
> wrong — not the link.**

Sync found with CRC failing means the framing, baud and wiring are all correct
and only the checksum disagrees. That is a specific, actionable diagnosis and
the UI should state it:

```
⚠ 412 packets synced, 412 CRC failures, 0 accepted.
  The link is good; the CRC parameters do not match the payload.   [Solve…]
```

**Solve** ports `tools/stp-crc-solve.py`: over the last N synced-but-failed
packets, try every catalogue variant × both stored byte orders × coverage starts
`{0, 4, 6}`, and list every parameter set that reproduces the stored value in
**all** of them. Require at least three packets — one sample admits coincidental
matches at roughly 1 in 65536 per candidate, and the catalogue is large enough
that a single packet regularly leaves several survivors. Offer to apply the
winner.

This is the single highest-value feature in this document. It converts the
worst failure mode on the link from an open-ended investigation into a button.

### 4.5 Verifying against the payload

`tools/rs422-tweak.py sample` prints real packets built with the payload's
current settings. The host should accept those three lines pasted in and confirm
it computes the same CRCs — proving agreement **before** a link is even opened:

```
Command ACK      8 bytes  1acffc1d10c7b7d7
LRT Data      1256 bytes  head 1acffc1d81c700000000000000000000 ... crc befb
HRT Data      1288 bytes  head 1acffc1d87c700000000000000000000 ... crc eb2f
```

### 4.6 Profiles, persistence and provenance

- Named, saved profiles; ship "STP flight default" (the table in §3.3 of the
  base spec) as a read-only one that cannot be edited away.
- Write the settings in force into every session directory as
  `wire-settings.json`. **Offline replay must use the settings the capture was
  made under**, not whatever is currently configured — otherwise a capture taken
  during a CRC experiment becomes unreadable later.
- Warn on Apply while connected: it reframes everything in flight, so drop the
  receive buffer and resynchronise rather than pretending continuity.

### 4.7 Canned command hex

Any pre-generated hex string carries **its own CRC baked in**. Changing the CRC
invalidates every canned string in the operator's config.

- The host must **regenerate** canned strings from its own encoder after a
  settings change, not store them as text.
- If it offers a paste box for strings from `tools/stp-command.py`, it must
  **validate the pasted string's CRC under the current settings** and refuse it
  with an explanation rather than transmitting something that will be discarded.
- `tools/stp-command.py` now reads the payload's config for exactly this reason;
  the host's equivalent must do the same against its own `WireConfig`.

### 4.8 Serial settings already in the base spec

Keep §2.3's FTDI latency-timer check — 1 ms, not the 16 ms default — and surface
it next to the port picker. It is unrelated to the CRC but it is the *other*
setting that makes a working link look broken.

---

## 5. Thermal camera — what to build now, what to defer

`docs/stp/MULTI_CAMERA_SPEC.md` specifies the thermal camera. It is **not
implemented**, and §11 lists seven questions that cannot be answered without the
part in hand. The host should therefore build the **general** mechanism now and
defer the thermal specifics:

**Build now** — it is all camera-agnostic:

- version-keyed LRT layout (§0.1)
- camera list, selector and interlock handling (§§1–3)
- sensor dimensions taken from the active camera
- per-camera session directories

**Defer, behind a feature flag keyed on the LRT version:**

- opcodes `0x6F`, `0x74`–`0x76`, `0x7C`–`0x7E` (camera info, output mode, range,
  emissivity, NUC, spot, palette)
- the 16-byte thermal telemetry block, which shortens the event ring to 36
- 16-bit radiometric rendering, palette mapping, and a temperature readout under
  the cursor
- 16-bit PNG or TIFF export — **a radiometric frame must never be saved as
  8-bit**, which would discard the measurement and keep only a picture of it

One number from that spec shapes the UI and is worth stating here: at the
measured 87.6 kB/s of HRT payload, a 640×512 16-bit radiometric frame takes
**7.3 seconds**. Radiometric *video* does not exist on this link. The host should
present radiometric capture as a still-image operation with a progress bar, and
never offer it as a stream mode.

---

## 6. Module-by-module change list

| Module | Change |
|---|---|
| `protocol/lrt.py` | **version-keyed layout table**; camera block; `MAX_EVENTS` from the layout, not a constant |
| `protocol/commands.py` | `SELECT_CAMERA` 0x6D, `CAMERA_LIST` 0x6E; camera-table parser (variable-length entries) |
| `protocol/crc.py` | parameterised CRC (poly/init/reflection/xor/store), the 15-variant catalogue, and `solve()` |
| `protocol/packets.py` | take a `WireConfig` instead of module constants |
| `link/session.py` | own the single `WireConfig`; refresh the camera list on connect and after each switch; buffer-flush on settings change |
| `storage/session_dir.py` | per-camera subdirectories; `wire-settings.json` |
| `offline/replay.py` | read `wire-settings.json`; reproduce the per-camera split |
| `ui/camera_panel.py` | **new** — the selector, with disabled reasons |
| `ui/wire_settings.py` | **new** — §4, including Solve |
| `ui/sensor_picker.py` | dimensions from the active camera; redraw on switch |
| `ui/dashboard.py` | camera fields; the synced-but-failing-CRC diagnosis |
| `sim/payload_sim.py` | serve a camera table, honour `SELECT_CAMERA`, and be able to emit **deliberately wrong CRCs** so the solver and the diagnosis are testable |

`protocol/` stays pure — no I/O, no Qt — so the solver and both layouts remain
unit-testable and reusable by the offline tool.

---

## 7. Test vectors

Current firmware, target `0xC7`, CRC-16/CCITT-FALSE, coverage `[4:crc]`.

```
CRC16("123456789")                = 0x29B1
Command ACK                       = 1ACFFC1D10C7B7D7

SELECT_CAMERA camera=0 (seq 1, force):
1ACFFC1D00000000000010C76D000101016DFE0000...0000785C

CAMERA_LIST (seq 1, force):
1ACFFC1D00000000000010C76E00010001E069000000...00003810

CAMERA_LIST response, one always-on camera:
01 00 00 30 07 04 06 41 52 31 33 33 35

CAMERA_LIST response, two switchable cameras, cam0 active:
02 00 00 05 03 04 06 "AR1335" 01 06 01 FF 07 "THERMAL"
```

Regenerate any of these with `tools/stp-command.py <NAME> [args]` and
`tools/rs422-tweak.py sample`.

---

## 8. Acceptance criteria

Additions to §20 of the base spec.

**Layout**

1. Telemetry from current firmware decodes with the event ring intact — events
   raised on the payload appear in the host within one poll.
2. An LRT `version` the host does not know is reported as such; the host does
   **not** decode the version-specific region against a guess.

**Cameras**

3. `CAMERA_LIST` renders a correct table for both the one-camera and the
   two-camera vectors in §7, including the always-on flag.
4. Selecting an always-on-blocked camera is **prevented in the UI**, with the
   reason visible, and not merely rejected by the payload.
5. A camera switch while streaming or recording prompts for confirmation naming
   what will stop, and the panel afterwards reflects `camera_active` from
   telemetry rather than the request.
6. Media from two cameras in one session lands in separate directories and is
   correctly separated again by offline replay.

**Wire settings**

7. Changing the CRC in the GUI changes both transmit and receive; a command sent
   afterwards is accepted by a payload configured to match.
8. Given ≥3 synced-but-CRC-failing packets from a payload with a deliberately
   changed CRC, **Solve identifies the parameters** and applying them restores
   the link with no restart.
9. The synced-but-failing diagnosis appears within 5 seconds of that condition
   and names the CRC as the cause.
10. A capture taken under non-default settings replays correctly from
    `wire-settings.json` after the live settings have been changed to something
    else.
11. A pasted canned hex string whose CRC does not match the current settings is
    refused with an explanation, not transmitted.

---

## 9. Suggested order of work

1. **§0 — the layout fix.** Everything else is a feature; this is a live bug.
2. **§4.2–4.4 — wire settings and Solve.** This is what makes the RS-422 test
   survive first contact with a flight computer that disagrees with us.
3. **§§1–3 — the camera model.** Needed before a second camera exists, and
   correct for one camera today.
4. **§3.3, §4.6 — session layout and provenance.**
5. **§5 — thermal**, once the vendor questions in the multi-camera spec §11 are
   answered.
