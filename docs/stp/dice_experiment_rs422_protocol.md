# RS-422 DICE ↔ Experiment Protocol

## Purpose

This document consolidates the RS-422 packet formats relevant to an **Experiment** communicating with **DICE**, where:

- **DICE is the flight-computer-side master.**
- **The Experiment is the peripheral/slave endpoint.**
- The Experiment receives commands and control requests from DICE.
- The Experiment sends acknowledgements and telemetry/data back to DICE.
- For Experiment-to-flight-computer output, only **LRT** and **HRT** are considered here.
- **H&S (Health & Safety)** and **FT (File Transfer)** traffic are intentionally ignored.
- Any packet whose `Target_ID` does not match this Experiment must be ignored.

This document is written to be implementation-friendly for firmware, software, and AI code-generation systems.

---

# 1. High-Level Behavior

The Experiment should conceptually behave as follows:

```text
DICE (master)                                  Experiment

Command Packet  ----------------------------->  Parse command
                                                 If Target_ID matches:
                                                   execute/dispatch command
                                                   send Command ACK

                    <-------------------------  Command ACK

LRT Request      ----------------------------->  If Target_ID matches:
                                                 prepare/send LRT data

                    <-------------------------  LRT Data Packet

HRT Flow Control ----------------------------->  If Target_ID matches:
                                                 0x85 = HRT Stop
                                                 0x86 = HRT Stop with loss
                                                 0x87 = HRT Go

                    <-------------------------  HRT Data Packet(s)
                                                 while HRT is enabled
```

The Experiment should **not** process traffic for other Target IDs.

---

# 2. Relevant Packet Types

| Direction | Packet | Packet Type | Size |
|---|---|---:|---:|
| DICE → Experiment | Command Packet | `0x10` | 120 bytes |
| Experiment → DICE | Command Acknowledge Packet | `0x10` | 8 bytes |
| DICE → Experiment | LRT Request | `0x81` | 14 bytes |
| Experiment → DICE | LRT Data Packet | `0x81` | 1256 bytes stated |
| DICE → Experiment | HRT Flow Control: Stop | `0x85` | 14 bytes |
| DICE → Experiment | HRT Flow Control: Stop with loss | `0x86` | 14 bytes |
| DICE → Experiment | HRT Flow Control: Go | `0x87` | 14 bytes |
| Experiment → DICE | HRT Data Packet | `0x87` | 1288 bytes |

## Important discriminator rule

Some packet type values are reused in opposite directions:

- `0x10`: 120-byte Command when received; 8-byte Command ACK when transmitted.
- `0x81`: 14-byte LRT Request when received; 1256-byte LRT Data Packet when transmitted.
- `0x87`: 14-byte HRT Go request when received; 1288-byte HRT Data Packet when transmitted.

Therefore, **do not identify a packet from `Packet Type` alone**. Direction and expected packet size/layout are also required.

---

# 3. Common Protocol Concepts

## 3.1 Sync Bytes

All supplied packet formats use a four-byte synchronization field represented as:

```text
0x1ACF FC1D
```

Conceptually:

```text
Word 1 = 0x1ACF
Word 2 = 0xFC1D
```

If transmitted most-significant byte first, the bytes would be:

```text
1A CF FC 1D
```

However, **wire byte order is not explicitly defined by the supplied tables**, so confirm endianness from the authoritative ICD or known-good traffic before hard-coding it.

## 3.2 Target ID

Packets containing a `Target_ID` identify the intended Experiment.

Receive-side logic should behave as:

```text
if packet.target_id != OUR_TARGET_ID:
    ignore packet
    return
```

Do not execute commands, change HRT state, or respond to LRT requests addressed to another Target ID.

## 3.3 Coarse and Fine Time

Packets that include time fields use:

```text
Coarse Time:
    4 bytes
    1 second/count

Fine Time:
    2 bytes
    approximately 15.3 microseconds/count
```

Conceptually:

```text
timestamp_seconds ≈ coarse_time_raw
                  + fine_time_raw * 15.3e-6
```

The coarse-time epoch is not defined by the supplied tables.

## 3.4 CRC

CRC fields are 16 bits / 2 bytes.

For the HRT Data and HRT Flow Control tables, the source explicitly states that CRC is calculated using all data in the packet **after the last sync byte and before the CRC**.

Conceptually:

```text
CRC16(packet[4 : crc_offset])
```

The exact CRC-16 polynomial, initial value, reflection settings, final XOR, and stored byte order are not defined in the supplied excerpts.

The same CRC coverage may apply to the other packet classes, but that is **not explicitly confirmed by the supplied tables**.

---

# 4. Command Packet — DICE → Experiment

## Purpose

DICE sends a command to the Experiment. If the `Target_ID` matches this Experiment, the Experiment should parse/dispatch the command and return a **Command Acknowledge Packet**.

```text
Direction:   DICE -> Experiment
Packet Type: 0x10
Packet Size: 120 bytes
```

## Layout

| Byte Offset | Length | Word(s) | Field | Description |
|---:|---:|---:|---|---|
| `0` | 4 | 1-2 | Sync Bytes | `0x1ACF FC1D` |
| `4` | 4 | 3-4 | Coarse Time | 1 second resolution |
| `8` | 2 | 5 | Fine Time | ~15.3 µs resolution |
| `10` | 1 | 6 | Packet Type | `0x10` = command |
| `11` | 1 | 6 | Target ID | Destination Experiment |
| `12` | 104 | 7-58 | Command Payload | First 104 payload bytes |
| `116` | 1 | 59 | Command Payload | Final payload byte |
| `117` | 1 | 59 | Spare | `0x00` |
| `118` | 2 | 60 | CRC | 16-bit CRC |

The total command payload is **105 bytes**, spanning offsets `12` through `116` inclusive:

```text
payload = packet[12:117]
```

## Conceptual Representation

```text
CommandPacket {
    sync:            4 bytes
    coarse_time:     uint32
    fine_time:       uint16
    packet_type:     uint8      // 0x10
    target_id:       uint8
    command_payload: byte[105]
    spare:           uint8      // expected 0x00
    crc:             uint16
}
```

## Receive Behavior

```text
receive 120-byte packet
    |
    +-- validate sync
    +-- packet_type == 0x10 ?
    +-- target_id == OUR_TARGET_ID ?
    |      no -> ignore
    +-- validate CRC
    +-- decode/dispatch command payload
    +-- send Command Acknowledge Packet
```

The internal command payload format is not defined by the supplied packet table.

---

# 5. Command Acknowledge Packet — Experiment → DICE

## Purpose

After receiving a valid command addressed to this Experiment, the Experiment returns an acknowledgement to DICE.

```text
Direction:   Experiment -> DICE
Packet Type: 0x10
Packet Size: 8 bytes
```

## Layout

| Byte Offset | Length | Word(s) | Field | Description |
|---:|---:|---:|---|---|
| `0` | 4 | 1-2 | Sync Bytes | `0x1ACF FC1D` |
| `4` | 1 | 3 | Packet Type | `0x10` |
| `5` | 1 | 3 | Target_ID | Target ID of the Experiment acknowledging the command |
| `6` | 2 | 4 | CRC | 16-bit CRC |

## Conceptual Representation

```text
CommandAcknowledgePacket {
    sync:        4 bytes
    packet_type: uint8    // 0x10
    target_id:   uint8    // OUR_TARGET_ID
    crc:         uint16
}
```

The supplied table does not define an acknowledgement status/error code, so this appears to be a simple acknowledgement packet rather than a detailed command-result message.

---

# 6. LRT Request — DICE → Experiment

## Purpose

DICE requests LRT data from the Experiment. The Experiment responds with an LRT Data Packet.

```text
Direction:   DICE -> Experiment
Packet Type: 0x81
Packet Size: 14 bytes
```

## Layout

| Byte Offset | Length | Word(s) | Field | Description |
|---:|---:|---:|---|---|
| `0` | 4 | 1-2 | Sync Bytes | `0x1ACF FC1D` |
| `4` | 4 | 3-4 | Coarse Time | 1 second resolution |
| `8` | 2 | 5 | Fine Time | Fine-time value |
| `10` | 1 | 6 | Packet Type | `0x81` |
| `11` | 1 | 6 | Target_ID | Destination Experiment |
| `12` | 2 | 7 | CRC | 16-bit CRC |

## Conceptual Representation

```text
LRTRequestPacket {
    sync:        4 bytes
    coarse_time: uint32
    fine_time:   uint16
    packet_type: uint8    // 0x81
    target_id:   uint8
    crc:         uint16
}
```

## Receive Behavior

```text
receive 14-byte packet
    |
    +-- packet_type == 0x81 ?
    +-- target_id == OUR_TARGET_ID ?
    |      no -> ignore
    +-- validate CRC
    +-- generate current LRT dataset
    +-- send LRT Data Packet
```

---

# 7. LRT Data Packet — Experiment → DICE

## Purpose

The Experiment returns LRT data in response to an LRT Request.

```text
Direction:   Experiment -> DICE
Packet Type: 0x81
Packet Size: 1256 bytes stated by source table
LRT Data:    fixed at 624 words = 1248 bytes
```

## Visible Source Layout

| Byte Offset | Length | Word(s) | Field | Description |
|---:|---:|---:|---|---|
| `0` | 4 | 1-2 | Sync Bytes | `0x1ACF FC1D` |
| `4` | 1 | 3 | Packet Type | `0x81` |
| `5` | 1 | 3 | Target_ID | Experiment Target ID |
| `6` | 1248 | 4-627 | LRT Data | Fixed at 624 words |

The visible rows account for:

```text
4 + 1 + 1 + 1248 = 1254 bytes
```

but the table header states a total size of:

```text
1256 bytes
```

Therefore, **2 bytes are not described by the visible rows**.

### Likely CRC, but not proven by the supplied image

The 2-byte difference is consistent with a final 16-bit CRC:

```text
offset 1254 : probable CRC start
length      : 2 bytes
```

This would also make LRT structurally consistent with HRT. However, because the supplied Table 13-7 image does not visibly include a CRC row, this must be treated as an **inference requiring confirmation**.

## Known Conceptual Representation

```text
LRTDataPacket {
    sync:        4 bytes
    packet_type: uint8       // 0x81
    target_id:   uint8
    lrt_data:    byte[1248]  // fixed 624 words

    // Stated total size = 1256 bytes.
    // Two trailing bytes are not identified in the visible excerpt.
    // Likely uint16 CRC at offsets 1254-1255; confirm before implementation.
}
```

---

# 8. HRT Data Packet — Experiment → DICE

## Purpose

The Experiment sends HRT data to DICE while HRT transmission is enabled.

```text
Direction:   Experiment -> DICE
Packet Type: 0x87
Packet Size: 1288 bytes
HRT Data:    fixed at 640 words = 1280 bytes
```

## Layout

| Byte Offset | Length | Word(s) | Field | Description |
|---:|---:|---:|---|---|
| `0` | 4 | 1-2 | Sync Bytes | `0x1ACF FC1D` |
| `4` | 1 | 3 | Packet Type | `0x87` |
| `5` | 1 | 3 | Target_ID | Experiment Target ID |
| `6` | 1280 | 4-643 | HRT Data | Fixed at 640 words |
| `1286` | 2 | 644 | CRC | 16-bit CRC |

## CRC Coverage

The source explicitly states:

```text
CRC input = packet[4:1286]
```

This includes:

```text
Packet Type
Target_ID
HRT Data
```

and excludes:

```text
Sync Bytes
CRC itself
```

## Conceptual Representation

```text
HRTDataPacket {
    sync:        4 bytes
    packet_type: uint8       // 0x87
    target_id:   uint8
    hrt_data:    byte[1280]  // fixed 640 words
    crc:         uint16
}
```

---

# 9. HRT Flow Control Requests — DICE → Experiment

## Purpose

DICE controls whether the Experiment may transmit HRT data.

```text
Direction:   DICE -> Experiment
Packet Size: 14 bytes

Packet Types:
    0x85 = HRT Stop
    0x86 = HRT Stop with loss
    0x87 = HRT Go
```

## Layout

| Byte Offset | Length | Word(s) | Field | Description |
|---:|---:|---:|---|---|
| `0` | 4 | 1-2 | Sync Bytes | `0x1ACF FC1D` |
| `4` | 4 | 3-4 | Coarse Time | 1 second resolution |
| `8` | 2 | 5 | Fine Time | Fine-time value |
| `10` | 1 | 6 | Packet Type | `0x85`, `0x86`, or `0x87` |
| `11` | 1 | 6 | Target_ID | Destination Experiment |
| `12` | 2 | 7 | CRC | 16-bit CRC |

## Flow-Control Meanings

```text
0x85 = HRT Stop
0x86 = HRT Stop with loss
0x87 = HRT Go
```

`HRT Stop with loss` clearly indicates different loss/buffering semantics from a normal stop, but the exact behavior is not defined in the supplied table and should not be invented.

## CRC Coverage

The source explicitly states:

```text
CRC input = packet[4:12]
```

This includes:

```text
Coarse Time     offsets 4-7
Fine Time       offsets 8-9
Packet Type     offset 10
Target_ID       offset 11
```

---

# 10. Recommended Experiment-Side Receive Dispatcher

Because DICE is the master, the Experiment receive path primarily needs to handle:

```text
1. Command Packet
2. LRT Request
3. HRT Flow Control
```

Use packet size + packet type + Target ID + CRC rather than packet type alone.

```text
on_rs422_packet(packet):

    if sync_invalid(packet):
        reject_or_resynchronize()
        return

    if packet.length == 120:
        if packet.packet_type == 0x10:
            if packet.target_id != OUR_TARGET_ID:
                return

            if not crc_valid(packet):
                return

            dispatch_command(packet.command_payload)
            send_command_ack(target_id = OUR_TARGET_ID)
            return

    if packet.length == 14:
        packet_type = packet[10]
        target_id   = packet[11]

        if target_id != OUR_TARGET_ID:
            return

        if not crc_valid(packet):
            return

        if packet_type == 0x81:
            send_lrt_data()
            return

        if packet_type == 0x85:
            hrt_enabled = false
            handle_hrt_stop()
            return

        if packet_type == 0x86:
            hrt_enabled = false
            handle_hrt_stop_with_loss()
            return

        if packet_type == 0x87:
            hrt_enabled = true
            handle_hrt_go()
            return

    // H&S, FT, unknown packet classes, and unrelated traffic are ignored.
    ignore_packet()
```

---

# 11. Recommended Experiment-Side Transmit Behavior

The Experiment only needs to generate these packet classes for this implementation:

```text
Command Acknowledge
LRT Data
HRT Data
```

## Command Acknowledge

Triggered by a valid Command Packet addressed to `OUR_TARGET_ID`.

Transmit:

```text
8-byte packet
Packet Type = 0x10
Target_ID   = OUR_TARGET_ID
```

## LRT Data

Triggered by a valid LRT Request addressed to `OUR_TARGET_ID`.

Transmit:

```text
Packet Type = 0x81
Target_ID   = OUR_TARGET_ID
Payload     = exactly 624 words / 1248 bytes
```

The source states a total LRT packet size of 1256 bytes, but the visible excerpt does not identify the final two bytes. Confirm whether they are CRC before final implementation.

## HRT Data

Transmit only while HRT is enabled by DICE:

```text
0x87 HRT Go      -> enable HRT transmission
0x85 HRT Stop    -> disable HRT transmission
0x86 Stop + loss -> disable HRT transmission
```

Outgoing HRT packet:

```text
Packet Type = 0x87
Target_ID   = OUR_TARGET_ID
Payload     = exactly 640 words / 1280 bytes
CRC         = 2 bytes
Total       = 1288 bytes
```

---

# 12. Suggested State Model

A minimal Experiment implementation can maintain:

```text
OUR_TARGET_ID
hrt_enabled
```

Conceptual HRT state machine:

```text
HRT_DISABLED
  |
  | receive valid 0x87 HRT Go
  v
HRT_ENABLED
  |
  +---- receive valid 0x85 HRT Stop ----------> HRT_DISABLED
  |
  +---- receive valid 0x86 HRT Stop with loss -> HRT_DISABLED
```

The initial HRT state is not defined by the supplied tables. A conservative implementation would normally avoid unsolicited HRT transmission until DICE sends `HRT Go`, but this should be checked against the broader ICD.

---

# 13. Fixed Sizes and Offsets

## Command Packet

```text
COMMAND_PACKET_SIZE            = 120
COMMAND_SYNC_OFFSET            = 0
COMMAND_COARSE_TIME_OFFSET     = 4
COMMAND_FINE_TIME_OFFSET       = 8
COMMAND_PACKET_TYPE_OFFSET     = 10
COMMAND_TARGET_ID_OFFSET       = 11
COMMAND_PAYLOAD_OFFSET         = 12
COMMAND_PAYLOAD_LENGTH         = 105
COMMAND_SPARE_OFFSET           = 117
COMMAND_CRC_OFFSET             = 118
```

## Command ACK

```text
COMMAND_ACK_SIZE               = 8
COMMAND_ACK_SYNC_OFFSET        = 0
COMMAND_ACK_PACKET_TYPE_OFFSET = 4
COMMAND_ACK_TARGET_ID_OFFSET   = 5
COMMAND_ACK_CRC_OFFSET         = 6
```

## LRT Request

```text
LRT_REQUEST_SIZE               = 14
LRT_REQ_SYNC_OFFSET            = 0
LRT_REQ_COARSE_TIME_OFFSET     = 4
LRT_REQ_FINE_TIME_OFFSET       = 8
LRT_REQ_PACKET_TYPE_OFFSET     = 10
LRT_REQ_TARGET_ID_OFFSET       = 11
LRT_REQ_CRC_OFFSET             = 12
```

## LRT Data

```text
LRT_DATA_PACKET_SIZE           = 1256  // stated by source table
LRT_DATA_SYNC_OFFSET           = 0
LRT_DATA_PACKET_TYPE_OFFSET    = 4
LRT_DATA_TARGET_ID_OFFSET      = 5
LRT_DATA_OFFSET                = 6
LRT_DATA_LENGTH                = 1248
LRT_DATA_WORDS                 = 624

// 2 bytes remain between end of data and stated packet size.
// Likely CRC at offset 1254, but confirm before implementation.
```

## HRT Flow Control

```text
HRT_CONTROL_SIZE               = 14
HRT_CTRL_SYNC_OFFSET           = 0
HRT_CTRL_COARSE_TIME_OFFSET    = 4
HRT_CTRL_FINE_TIME_OFFSET      = 8
HRT_CTRL_PACKET_TYPE_OFFSET    = 10
HRT_CTRL_TARGET_ID_OFFSET      = 11
HRT_CTRL_CRC_OFFSET            = 12
```

## HRT Data

```text
HRT_DATA_PACKET_SIZE           = 1288
HRT_DATA_SYNC_OFFSET           = 0
HRT_DATA_PACKET_TYPE_OFFSET    = 4
HRT_DATA_TARGET_ID_OFFSET      = 5
HRT_DATA_OFFSET                = 6
HRT_DATA_LENGTH                = 1280
HRT_DATA_WORDS                 = 640
HRT_DATA_CRC_OFFSET            = 1286
```

---

# 14. Packet Type Constants

```text
PACKET_TYPE_COMMAND            = 0x10
PACKET_TYPE_COMMAND_ACK        = 0x10

PACKET_TYPE_LRT_REQUEST        = 0x81
PACKET_TYPE_LRT_DATA           = 0x81

PACKET_TYPE_HRT_STOP           = 0x85
PACKET_TYPE_HRT_STOP_WITH_LOSS = 0x86
PACKET_TYPE_HRT_GO             = 0x87
PACKET_TYPE_HRT_DATA           = 0x87
```

Duplicate values are intentional and must be interpreted from packet direction and structure.

---

# 15. Traffic This Implementation Can Ignore

For the stated Experiment implementation, application support can omit:

```text
H&S / Health and Safety
FT / File Transfer
```

Also ignore/reject:

```text
- packets addressed to another Target_ID
- unsupported packet classes
- malformed packet lengths
- invalid synchronization
- packets that fail CRC validation
```

---

# 16. Minimum Functional Implementation

## Receive

```text
[REQUIRED] 120-byte Command Packet
[REQUIRED] 14-byte LRT Request
[REQUIRED] 14-byte HRT Stop
[REQUIRED] 14-byte HRT Stop with loss
[REQUIRED] 14-byte HRT Go
```

## Transmit

```text
[REQUIRED] 8-byte Command Acknowledge Packet
[REQUIRED] LRT Data Packet
[REQUIRED] 1288-byte HRT Data Packet
```

## Filtering/Validation

```text
[REQUIRED] Target_ID matching
[REQUIRED] Packet length validation
[REQUIRED] Sync validation
[REQUIRED] CRC validation
```

---

# 17. AI / Code-Generation Constraints

Treat the following values as authoritative from the supplied packet tables:

```text
Sync representation                = 0x1ACF FC1D

Command:
    type                           = 0x10
    RX size                        = 120 bytes
    command payload                = 105 bytes
    spare                          = 0x00

Command ACK:
    type                           = 0x10
    TX size                        = 8 bytes

LRT Request:
    type                           = 0x81
    RX size                        = 14 bytes

LRT Data:
    type                           = 0x81
    payload                        = 624 words / 1248 bytes
    stated total size              = 1256 bytes
    unexplained trailing space     = 2 bytes in supplied excerpt

HRT Control:
    stop                           = 0x85
    stop with loss                 = 0x86
    go                             = 0x87
    RX size                        = 14 bytes

HRT Data:
    type                           = 0x87
    payload                        = 640 words / 1280 bytes
    TX size                        = 1288 bytes
    CRC offset                     = 1286

CRC coverage explicitly shown for HRT:
    start                          = byte 4
    end                            = byte immediately before CRC
```

Do **not** invent:

```text
- wire byte order / endianness
- CRC-16 polynomial and parameters
- coarse-time epoch
- Target_ID numeric assignment
- command payload internal structure
- LRT payload semantic structure
- HRT payload semantic structure
- exact HRT Stop-with-loss buffer behavior
- identity of the unexplained final 2 LRT bytes without confirmation
- initial HRT enabled/disabled state unless specified elsewhere
```

---

# 18. Implementation-Oriented Summary

The Experiment is a DICE-controlled RS-422 endpoint.

Receive behavior:

```text
0x10 / 120 bytes -> COMMAND
    if addressed to us:
        process command
        return 0x10 / 8-byte ACK

0x81 / 14 bytes -> LRT REQUEST
    if addressed to us:
        return 0x81 LRT DATA packet

0x85 / 14 bytes -> HRT STOP
    if addressed to us:
        stop HRT output

0x86 / 14 bytes -> HRT STOP WITH LOSS
    if addressed to us:
        stop HRT output
        apply loss semantics once defined

0x87 / 14 bytes -> HRT GO
    if addressed to us:
        enable HRT output
```

Transmit behavior:

```text
Command ACK:
    8 bytes
    type 0x10

LRT Data:
    type 0x81
    624 words / 1248 data bytes
    1256 total bytes stated

HRT Data:
    1288 bytes
    type 0x87
    640 words / 1280 data bytes
```

For this project, **LRT and HRT are the only required Experiment-to-flight-computer data paths**. H&S and FT may be omitted from the implementation.
