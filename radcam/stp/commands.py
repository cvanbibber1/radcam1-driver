"""The 105-byte command payload - the one part of the packet we define.

The ICD fixes the envelope (sync, time, type, target, CRC) and hands us 105
opaque bytes at offsets 12..116. This module defines what goes in them, and
bridges them onto the existing `radcam.protocol.Dispatcher`, whose command set
- capture, configure, transfer, EEPROM repair - is exactly what the mission
needs and is already unit-tested.

Payload layout (big-endian, matching the packet envelope):

     offset  size  field
          0     1  opcode        radcam.protocol.Msg command code
          1     2  cmd_seq       ground's sequence number, echoed in LRT
          3     1  arg_len       0..98
          4     1  flags         bit 0 = force (bypass duplicate suppression)
          5     2  payload_crc16 over bytes [0..4] + args, or 0 to skip
          7    98  args          command-specific, arg_len bytes used
    105 total

**Why a second CRC inside a CRC-protected packet.** The envelope CRC proves the
packet arrived intact. It says nothing about the bytes afterwards, and this
payload is copied, queued, and may sit in RAM for the duration of a capture
before it is acted on. A command that fires the LED, deletes media or writes
the calibration EEPROM is worth re-verifying at the moment of execution rather
than trusting a check performed milliseconds earlier in a different buffer. It
costs 105 bytes of CRC.

Ground stations that do not want to compute it may send 0x0000, which disables
the check. A payload whose real CRC happens to be zero simply loses the extra
check - it does not become invalid - which happens for about one command in
65536 and costs nothing.

**Duplicate suppression.** DICE has no retransmission semantics we can see, so
a repeated `cmd_seq` is assumed to be a retransmission rather than a genuine
second request, and is acknowledged without being executed twice. That matters
most for the commands that are not idempotent: CAPTURE_IMAGE would otherwise
fill storage, and DELETE_MEDIA would remove a second file. Setting bit 0 of
`flags` overrides this when the ground really does mean "do it again".
"""

from __future__ import annotations

import logging
import struct
from dataclasses import dataclass

from .crc import CCITT_FALSE, Crc16Params

log = logging.getLogger(__name__)

__all__ = [
    "CommandRequest", "CommandDecodeError", "decode_command_payload",
    "encode_command_payload", "COMMAND_HEADER_LEN", "MAX_ARGS",
    "FLAG_FORCE", "PAYLOAD_LEN",
]

PAYLOAD_LEN = 105
COMMAND_HEADER_LEN = 7
MAX_ARGS = PAYLOAD_LEN - COMMAND_HEADER_LEN        # 98

FLAG_FORCE = 0x01

_HEADER_FMT = ">BHBBH"


class CommandDecodeError(Exception):
    """The 105 bytes were not a well-formed command."""


@dataclass(frozen=True)
class CommandRequest:
    opcode: int
    cmd_seq: int
    flags: int
    args: bytes

    @property
    def force(self) -> bool:
        return bool(self.flags & FLAG_FORCE)


def encode_command_payload(opcode: int, cmd_seq: int, args: bytes = b"",
                           flags: int = 0, with_crc: bool = True,
                           crc: Crc16Params = CCITT_FALSE) -> bytes:
    """Build the 105-byte payload. Used by the ground and the simulator."""
    if len(args) > MAX_ARGS:
        raise ValueError(f"args {len(args)} exceed {MAX_ARGS} bytes")

    head = struct.pack(_HEADER_FMT, opcode & 0xFF, cmd_seq & 0xFFFF,
                       len(args), flags & 0xFF, 0)
    check = crc.compute(head[:5] + args) if with_crc else 0
    head = struct.pack(_HEADER_FMT, opcode & 0xFF, cmd_seq & 0xFFFF,
                       len(args), flags & 0xFF, check)

    payload = head + args
    return payload + b"\x00" * (PAYLOAD_LEN - len(payload))


def decode_command_payload(payload: bytes,
                           crc: Crc16Params = CCITT_FALSE) -> CommandRequest:
    """Parse and verify the 105-byte payload."""
    if len(payload) != PAYLOAD_LEN:
        raise CommandDecodeError(f"payload must be {PAYLOAD_LEN} bytes, "
                                 f"got {len(payload)}")

    opcode, cmd_seq, arg_len, flags, check = struct.unpack_from(
        _HEADER_FMT, payload, 0)

    if arg_len > MAX_ARGS:
        raise CommandDecodeError(f"arg_len {arg_len} exceeds {MAX_ARGS}")

    args = payload[COMMAND_HEADER_LEN:COMMAND_HEADER_LEN + arg_len]

    if check:
        computed = crc.compute(payload[:5] + args)
        if computed != check:
            raise CommandDecodeError(
                f"command CRC mismatch: computed 0x{computed:04X}, "
                f"payload says 0x{check:04X}")

    return CommandRequest(opcode=opcode, cmd_seq=cmd_seq, flags=flags,
                          args=bytes(args))
