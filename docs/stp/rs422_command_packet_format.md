# RS-422 Command Packet Format

## Overview

This document describes the **120-byte command packet sent from DICE to an Experiment over RS-422**.

The packet is fixed-length:

- **Total packet size:** 120 bytes
- **Byte offsets:** Zero-based from the first byte of the packet
- **Word numbering:** The source table numbers the packet as 16-bit words, so one word = 2 bytes
- **Packet type for a command:** `0x10`
- **CRC:** 16-bit CRC stored in the final 2 bytes
- **Command payload capacity:** 105 bytes total, spanning byte offsets 12 through 116 inclusive

> Important: The source table does **not** specify byte order/endianness, the CRC polynomial/initialization/reflection rules, Target ID definitions, or the internal format of the command payload. These must come from the surrounding interface-control document or implementation.

---

## Packet Layout

| Byte Offset | Length | Word(s) | Field | Description |
|---:|---:|---:|---|---|
| `0` | 4 | 1-2 | Sync Bytes | Synchronization pattern specified as `0x1ACF FC1D` |
| `4` | 4 | 3-4 | Coarse Time | Coarse timestamp with 1-second resolution |
| `8` | 2 | 5 | Fine Time | Fine timestamp with approximately 15.3 µs resolution per count |
| `10` | 1 | 6 | Packet Type | Must be `0x10` for a command packet |
| `11` | 1 | 6 | Target ID | Identifies the destination experiment/device |
| `12` | 104 | 7-58 | Command Payload | First 104 bytes of command-specific payload data |
| `116` | 1 | 59 | Command Payload | Final byte of the command payload |
| `117` | 1 | 59 | Spare | Reserved byte; specified value is `0x00` |
| `118` | 2 | 60 | CRC | 16-bit CRC |
|  | **120 total** |  |  |  |

---

## Byte-Level View

```text
Byte offset
0                                                               119
|-----------------------------------------------------------------|
| Sync | Coarse Time | Fine |Type|ID|      Command Payload      |S| CRC |
| 4 B  |     4 B     | 2 B  |1 B |1B|          105 B            |1| 2 B |
|-----------------------------------------------------------------|
  0-3       4-7        8-9   10  11          12-116          117 118-119
```

Where:

- `Type` = Packet Type
- `ID` = Target ID
- `S` = Spare byte, expected to be `0x00`

---

## Field Descriptions

### 1. Sync Bytes

**Offset:** `0`  
**Length:** `4 bytes`  
**Words:** `1-2`

The synchronization field identifies the beginning of a valid packet.

The source table specifies the sync value as:

```text
0x1ACF FC1D
```

This can be interpreted as two 16-bit values:

```text
Word 1 = 0x1ACF
Word 2 = 0xFC1D
```

The exact transmitted byte sequence depends on the protocol's byte order. For example, if the words are transmitted most-significant byte first, the bytes would be:

```text
1A CF FC 1D
```

Do **not** assume this byte order unless it is confirmed elsewhere in the protocol specification.

---

### 2. Coarse Time

**Offset:** `4`  
**Length:** `4 bytes`  
**Words:** `3-4`

The coarse timestamp has a resolution of:

```text
1 second/count
```

Conceptually:

```text
coarse_time_seconds = coarse_time_raw
```

The epoch/reference time is not specified by the source table.

---

### 3. Fine Time

**Offset:** `8`  
**Length:** `2 bytes`  
**Word:** `5`

The fine timestamp provides sub-second timing with approximately:

```text
15.3 microseconds/count
```

Conceptually:

```text
fine_time_seconds ≈ fine_time_raw × 15.3e-6
```

A combined timestamp can therefore be represented as:

```text
timestamp_seconds ≈ coarse_time_raw
                  + fine_time_raw × 15.3e-6
```

The exact fine-time scale should be taken from the authoritative protocol definition if greater timing precision is required.

---

### 4. Packet Type

**Offset:** `10`  
**Length:** `1 byte`  
**Word:** `6`, first byte

For this packet format:

```text
Packet Type = 0x10
```

A receiver can use this field to verify that the packet is a command packet.

---

### 5. Target ID

**Offset:** `11`  
**Length:** `1 byte`  
**Word:** `6`, second byte

The Target ID identifies the intended destination experiment or device.

```text
Target ID = uint8
```

The mapping between numeric Target IDs and physical/logical devices is not defined by the source table.

---

### 6. Command Payload

**Offset:** `12`  
**Length:** `105 bytes total`  
**Words:** `7-59`

The command payload contains command-specific application data.

The source table splits it into two entries because word 59 contains both the last payload byte and the spare byte:

```text
Offsets 12-115 : 104 payload bytes
Offset 116     : final payload byte
```

Therefore:

```text
Command Payload = bytes[12:117]
Payload length  = 105 bytes
```

The meaning of these 105 bytes depends on the command being transmitted.

If a particular command uses fewer than 105 payload bytes, the required padding convention is not specified by this table and must be defined elsewhere.

---

### 7. Spare

**Offset:** `117`  
**Length:** `1 byte`  
**Word:** `59`, second byte

Reserved/spare byte.

Expected value:

```text
0x00
```

A transmitter should set this byte to zero unless a later protocol revision defines another use.

---

### 8. CRC

**Offset:** `118`  
**Length:** `2 bytes`  
**Word:** `60`

The final two bytes contain a:

```text
16-bit CRC
```

The source table does not define:

- CRC polynomial
- Initial value
- Input reflection
- Output reflection
- Final XOR value
- CRC byte order
- Exact byte range included in the CRC calculation

These parameters must be obtained before implementing CRC generation or validation.

A likely implementation pattern is:

```text
crc_received = packet[118:120]
crc_computed = CRC16(packet[0:N])
```

but the exact value of `N` and the CRC algorithm cannot be determined from this table alone.

---

## Canonical Packet Representation

For software or AI-generated code, the packet can be modeled conceptually as:

```text
CommandPacket {
    sync:            4 bytes   // offset 0
    coarse_time:     uint32    // offset 4
    fine_time:       uint16    // offset 8
    packet_type:     uint8     // offset 10, must equal 0x10
    target_id:       uint8     // offset 11
    command_payload: byte[105] // offsets 12-116
    spare:           uint8     // offset 117, expected 0x00
    crc:             uint16    // offsets 118-119
}
```

This representation describes field sizes and positions only. Integer endianness is still undefined.

---

## Parser Requirements

A receiver parsing this protocol should generally perform the following checks:

1. Accumulate exactly **120 bytes** for one packet.
2. Verify the four-byte synchronization pattern.
3. Decode the coarse and fine timestamps using the protocol-defined byte order.
4. Verify that byte `10` is `0x10` for a command packet.
5. Read the Target ID from byte `11`.
6. Treat bytes `12-116` as the **105-byte command payload**.
7. Verify byte `117` is `0x00`, unless another protocol revision says otherwise.
8. Read the CRC from bytes `118-119`.
9. Validate the CRC using the protocol-defined CRC-16 parameters.
10. Pass the command payload to the command-specific decoder.

---

## Offset Constants

Useful constants for an implementation:

```text
PACKET_SIZE            = 120

SYNC_OFFSET            = 0
SYNC_LENGTH            = 4

COARSE_TIME_OFFSET     = 4
COARSE_TIME_LENGTH     = 4

FINE_TIME_OFFSET       = 8
FINE_TIME_LENGTH       = 2

PACKET_TYPE_OFFSET     = 10
PACKET_TYPE_LENGTH     = 1
COMMAND_PACKET_TYPE    = 0x10

TARGET_ID_OFFSET       = 11
TARGET_ID_LENGTH       = 1

COMMAND_PAYLOAD_OFFSET = 12
COMMAND_PAYLOAD_LENGTH = 105

SPARE_OFFSET           = 117
SPARE_LENGTH           = 1
SPARE_EXPECTED_VALUE   = 0x00

CRC_OFFSET             = 118
CRC_LENGTH             = 2
```

---

## AI Implementation Notes

When generating code for this interface, assume only the following facts are authoritative from this packet table:

```text
Packet length = 120 bytes
Sync field = 4 bytes at offset 0, represented in the document as 0x1ACF FC1D
Coarse Time = 4 bytes at offset 4
Fine Time = 2 bytes at offset 8, approximately 15.3 µs/count
Packet Type = 1 byte at offset 10 and equals 0x10 for commands
Target ID = 1 byte at offset 11
Command Payload = 105 bytes at offsets 12-116
Spare = 1 byte at offset 117 and equals 0x00
CRC = 2 bytes at offsets 118-119
```

Do **not** invent any of the following without another source:

```text
- Endianness
- CRC-16 variant
- CRC coverage range
- Timestamp epoch
- Target ID assignments
- Command payload sub-format
- Payload padding rules
```

Those items must be explicitly defined before a fully interoperable encoder/decoder can be implemented.
