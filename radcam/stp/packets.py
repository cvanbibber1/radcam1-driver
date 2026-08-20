"""STP / DICE RS-422 packet formats.

Every offset and size in this module is lifted verbatim from the supplied ICD
tables (`docs/stp/rs422_command_packet_format.md` and
`docs/stp/dice_experiment_rs422_protocol.md`). Nothing here is inferred except
where explicitly marked, and each inference is a `Wire` field the mission can
change without touching code.

The one structural trap the ICD calls out: **packet type alone does not
identify a packet.** 0x10 is both a 120-byte Command inbound and an 8-byte
Command ACK outbound; 0x81 is both a 14-byte LRT Request and a 1256-byte LRT
Data packet; 0x87 is both a 14-byte HRT Go and a 1288-byte HRT Data packet.
Direction plus length disambiguates, so the decoders here are named by
direction and each asserts its own length.

CRC coverage follows the rule the ICD states explicitly for the HRT classes -
everything after the last sync byte, up to but excluding the CRC:

    crc = CRC16(packet[4 : crc_offset])

The ICD does not confirm that the same coverage applies to the Command and LRT
classes, so `Wire.crc_start` exists to move it.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

from .crc import CCITT_FALSE, Crc16Params

__all__ = [
    "PacketError", "Wire", "PacketType",
    "COMMAND_PACKET_SIZE", "COMMAND_ACK_SIZE", "LRT_REQUEST_SIZE",
    "LRT_DATA_PACKET_SIZE", "HRT_CONTROL_SIZE", "HRT_DATA_PACKET_SIZE",
    "COMMAND_PAYLOAD_LENGTH", "LRT_DATA_LENGTH", "HRT_DATA_LENGTH",
    "CommandPacket", "ShortRequest",
    "decode_command", "decode_short_request",
    "encode_command_ack", "encode_lrt_data", "encode_hrt_data",
    "rx_length_for_type", "RX_LENGTHS",
]


class PacketError(Exception):
    """A packet was malformed. Always recoverable: drop it and resynchronise."""


# --------------------------------------------------------------- constants

class PacketType:
    COMMAND = 0x10
    COMMAND_ACK = 0x10
    LRT_REQUEST = 0x81
    LRT_DATA = 0x81
    HRT_STOP = 0x85
    HRT_STOP_WITH_LOSS = 0x86
    HRT_GO = 0x87
    HRT_DATA = 0x87


SYNC_WORD_1 = 0x1ACF
SYNC_WORD_2 = 0xFC1D

COMMAND_PACKET_SIZE = 120
COMMAND_SYNC_OFFSET = 0
COMMAND_COARSE_TIME_OFFSET = 4
COMMAND_FINE_TIME_OFFSET = 8
COMMAND_PACKET_TYPE_OFFSET = 10
COMMAND_TARGET_ID_OFFSET = 11
COMMAND_PAYLOAD_OFFSET = 12
COMMAND_PAYLOAD_LENGTH = 105
COMMAND_SPARE_OFFSET = 117
COMMAND_CRC_OFFSET = 118

COMMAND_ACK_SIZE = 8
COMMAND_ACK_PACKET_TYPE_OFFSET = 4
COMMAND_ACK_TARGET_ID_OFFSET = 5
COMMAND_ACK_CRC_OFFSET = 6

#: The LRT Request and all three HRT flow-control packets share one 14-byte
#: layout, differing only in the type byte. One decoder serves all four.
SHORT_REQUEST_SIZE = 14
LRT_REQUEST_SIZE = 14
HRT_CONTROL_SIZE = 14
SHORT_COARSE_TIME_OFFSET = 4
SHORT_FINE_TIME_OFFSET = 8
SHORT_PACKET_TYPE_OFFSET = 10
SHORT_TARGET_ID_OFFSET = 11
SHORT_CRC_OFFSET = 12

LRT_DATA_PACKET_SIZE = 1256
LRT_DATA_PACKET_TYPE_OFFSET = 4
LRT_DATA_TARGET_ID_OFFSET = 5
LRT_DATA_OFFSET = 6
LRT_DATA_LENGTH = 1248             # 624 words, fixed by the ICD
LRT_DATA_CRC_OFFSET = 1254         # inferred; see Wire.lrt_trailer

HRT_DATA_PACKET_SIZE = 1288
HRT_DATA_PACKET_TYPE_OFFSET = 4
HRT_DATA_TARGET_ID_OFFSET = 5
HRT_DATA_OFFSET = 6
HRT_DATA_LENGTH = 1280             # 640 words, fixed by the ICD
HRT_DATA_CRC_OFFSET = 1286

#: Inbound packet type -> total length. Used by the receiver to decide how many
#: bytes to accumulate once it has locked onto a sync pattern. Types absent
#: from this map (H&S, File Transfer, anything unknown) have lengths this
#: implementation does not know, so they are resynchronised past rather than
#: consumed.
RX_LENGTHS: dict[int, int] = {
    PacketType.COMMAND: COMMAND_PACKET_SIZE,
    PacketType.LRT_REQUEST: SHORT_REQUEST_SIZE,
    PacketType.HRT_STOP: SHORT_REQUEST_SIZE,
    PacketType.HRT_STOP_WITH_LOSS: SHORT_REQUEST_SIZE,
    PacketType.HRT_GO: SHORT_REQUEST_SIZE,
}


def rx_length_for_type(packet_type: int) -> int | None:
    return RX_LENGTHS.get(packet_type)


# -------------------------------------------------------------------- wire

@dataclass(frozen=True)
class Wire:
    """Everything the ICD leaves undefined, in one place.

    `big_endian` governs multi-byte integers *and* the sync pattern. The ICD
    writes the sync as two 16-bit words, 0x1ACF then 0xFC1D, so transmitting
    most-significant byte first gives `1A CF FC 1D`; a little-endian wire would
    put each word out low byte first, giving `CF 1A 1D FC`. Both are generated
    from the same word constants rather than being hard-coded byte strings, so
    flipping this flag cannot leave the sync and the integers disagreeing.
    """

    big_endian: bool = True
    crc: Crc16Params = field(default_factory=lambda: CCITT_FALSE)
    target_id: int = 0x01
    #: First byte the CRC covers. The ICD states 4 (just past sync) for the HRT
    #: classes and does not confirm it for the others.
    crc_start: int = 4
    #: What occupies the final two bytes of an LRT Data packet. The ICD's
    #: visible rows account for only 1254 of the stated 1256 bytes; a trailing
    #: CRC is the reading consistent with HRT, but it is an inference.
    lrt_trailer: str = "crc"          # "crc" or "zero"

    # -- helpers ---------------------------------------------------------

    @property
    def endian(self) -> str:
        return ">" if self.big_endian else "<"

    @property
    def sync_bytes(self) -> bytes:
        return struct.pack(self.endian + "HH", SYNC_WORD_1, SYNC_WORD_2)

    def u32(self, value: int) -> bytes:
        return struct.pack(self.endian + "I", value & 0xFFFFFFFF)

    def u16(self, value: int) -> bytes:
        return struct.pack(self.endian + "H", value & 0xFFFF)

    def read_u32(self, buf: bytes, off: int) -> int:
        return struct.unpack_from(self.endian + "I", buf, off)[0]

    def read_u16(self, buf: bytes, off: int) -> int:
        return struct.unpack_from(self.endian + "H", buf, off)[0]

    def compute_crc(self, packet: bytes, crc_offset: int) -> int:
        return self.crc.compute(packet[self.crc_start:crc_offset])

    def check_crc(self, packet: bytes, crc_offset: int) -> bool:
        stored = self.crc.unpack(packet[crc_offset:crc_offset + 2])
        return self.compute_crc(packet, crc_offset) == stored

    def seal(self, packet: bytearray, crc_offset: int) -> bytes:
        """Write the CRC into an otherwise complete packet."""
        value = self.compute_crc(bytes(packet), crc_offset)
        packet[crc_offset:crc_offset + 2] = self.crc.pack(value)
        return bytes(packet)


DEFAULT_WIRE = Wire()


# ----------------------------------------------------------------- decoded

@dataclass(frozen=True)
class CommandPacket:
    """A 120-byte Command, DICE -> Experiment."""

    coarse_time: int
    fine_time: int
    target_id: int
    payload: bytes            # exactly 105 bytes
    spare: int
    raw: bytes = b""

    @property
    def timestamp_s(self) -> float:
        return self.coarse_time + self.fine_time * 15.3e-6


@dataclass(frozen=True)
class ShortRequest:
    """A 14-byte LRT Request or HRT flow-control packet, DICE -> Experiment."""

    packet_type: int
    coarse_time: int
    fine_time: int
    target_id: int
    raw: bytes = b""

    @property
    def timestamp_s(self) -> float:
        return self.coarse_time + self.fine_time * 15.3e-6

    @property
    def is_lrt_request(self) -> bool:
        return self.packet_type == PacketType.LRT_REQUEST

    @property
    def is_hrt_control(self) -> bool:
        return self.packet_type in (PacketType.HRT_STOP,
                                    PacketType.HRT_STOP_WITH_LOSS,
                                    PacketType.HRT_GO)


# ---------------------------------------------------------------- decoders

def _require_sync(buf: bytes, wire: Wire) -> None:
    if buf[0:4] != wire.sync_bytes:
        raise PacketError(f"bad sync {buf[0:4].hex()}, "
                          f"expected {wire.sync_bytes.hex()}")


def decode_command(buf: bytes, wire: Wire = DEFAULT_WIRE) -> CommandPacket:
    """Decode a 120-byte Command packet. Raises PacketError if it is not one.

    The spare byte is checked but a non-zero value is *not* fatal: the ICD says
    a transmitter should send zero, and a later revision may define a use for
    it. Rejecting the whole command over a byte we are told to ignore would
    turn a forward-compatible change into a comms outage.
    """
    if len(buf) != COMMAND_PACKET_SIZE:
        raise PacketError(f"command must be {COMMAND_PACKET_SIZE} bytes, "
                          f"got {len(buf)}")
    _require_sync(buf, wire)

    packet_type = buf[COMMAND_PACKET_TYPE_OFFSET]
    if packet_type != PacketType.COMMAND:
        raise PacketError(f"packet type 0x{packet_type:02X} is not a command")

    if not wire.check_crc(buf, COMMAND_CRC_OFFSET):
        raise PacketError("command CRC mismatch")

    return CommandPacket(
        coarse_time=wire.read_u32(buf, COMMAND_COARSE_TIME_OFFSET),
        fine_time=wire.read_u16(buf, COMMAND_FINE_TIME_OFFSET),
        target_id=buf[COMMAND_TARGET_ID_OFFSET],
        payload=bytes(buf[COMMAND_PAYLOAD_OFFSET:
                          COMMAND_PAYLOAD_OFFSET + COMMAND_PAYLOAD_LENGTH]),
        spare=buf[COMMAND_SPARE_OFFSET],
        raw=bytes(buf),
    )


def decode_short_request(buf: bytes, wire: Wire = DEFAULT_WIRE) -> ShortRequest:
    """Decode a 14-byte LRT Request or HRT flow-control packet."""
    if len(buf) != SHORT_REQUEST_SIZE:
        raise PacketError(f"short request must be {SHORT_REQUEST_SIZE} bytes, "
                          f"got {len(buf)}")
    _require_sync(buf, wire)

    packet_type = buf[SHORT_PACKET_TYPE_OFFSET]
    if packet_type not in (PacketType.LRT_REQUEST, PacketType.HRT_STOP,
                           PacketType.HRT_STOP_WITH_LOSS, PacketType.HRT_GO):
        raise PacketError(f"packet type 0x{packet_type:02X} is not a "
                          "14-byte request this implementation handles")

    if not wire.check_crc(buf, SHORT_CRC_OFFSET):
        raise PacketError("short request CRC mismatch")

    return ShortRequest(
        packet_type=packet_type,
        coarse_time=wire.read_u32(buf, SHORT_COARSE_TIME_OFFSET),
        fine_time=wire.read_u16(buf, SHORT_FINE_TIME_OFFSET),
        target_id=buf[SHORT_TARGET_ID_OFFSET],
        raw=bytes(buf),
    )


# ---------------------------------------------------------------- encoders

def encode_command_ack(wire: Wire = DEFAULT_WIRE,
                       target_id: int | None = None) -> bytes:
    """Build the 8-byte Command Acknowledge packet.

    The ICD gives this packet no status or error field, so it can only mean
    "a valid command addressed to me arrived" - never "the command succeeded".
    Command *results* are reported through LRT telemetry instead; see
    `radcam/stp/lrt.py`.
    """
    tid = wire.target_id if target_id is None else target_id
    packet = bytearray(COMMAND_ACK_SIZE)
    packet[0:4] = wire.sync_bytes
    packet[COMMAND_ACK_PACKET_TYPE_OFFSET] = PacketType.COMMAND_ACK
    packet[COMMAND_ACK_TARGET_ID_OFFSET] = tid & 0xFF
    return wire.seal(packet, COMMAND_ACK_CRC_OFFSET)


def encode_lrt_data(payload: bytes, wire: Wire = DEFAULT_WIRE,
                    target_id: int | None = None) -> bytes:
    """Build a 1256-byte LRT Data packet around a 1248-byte payload.

    Payloads shorter than 1248 bytes are zero-padded: the ICD fixes the LRT
    data field at 624 words, so the packet length is not negotiable.
    """
    if len(payload) > LRT_DATA_LENGTH:
        raise PacketError(f"LRT payload {len(payload)} exceeds "
                          f"{LRT_DATA_LENGTH} bytes")
    tid = wire.target_id if target_id is None else target_id

    packet = bytearray(LRT_DATA_PACKET_SIZE)
    packet[0:4] = wire.sync_bytes
    packet[LRT_DATA_PACKET_TYPE_OFFSET] = PacketType.LRT_DATA
    packet[LRT_DATA_TARGET_ID_OFFSET] = tid & 0xFF
    packet[LRT_DATA_OFFSET:LRT_DATA_OFFSET + len(payload)] = payload

    if wire.lrt_trailer == "crc":
        return wire.seal(packet, LRT_DATA_CRC_OFFSET)
    return bytes(packet)          # trailer left as zeros


def encode_hrt_data(payload: bytes, wire: Wire = DEFAULT_WIRE,
                    target_id: int | None = None) -> bytes:
    """Build a 1288-byte HRT Data packet around a 1280-byte payload."""
    if len(payload) > HRT_DATA_LENGTH:
        raise PacketError(f"HRT payload {len(payload)} exceeds "
                          f"{HRT_DATA_LENGTH} bytes")
    tid = wire.target_id if target_id is None else target_id

    packet = bytearray(HRT_DATA_PACKET_SIZE)
    packet[0:4] = wire.sync_bytes
    packet[HRT_DATA_PACKET_TYPE_OFFSET] = PacketType.HRT_DATA
    packet[HRT_DATA_TARGET_ID_OFFSET] = tid & 0xFF
    packet[HRT_DATA_OFFSET:HRT_DATA_OFFSET + len(payload)] = payload
    return wire.seal(packet, HRT_DATA_CRC_OFFSET)


# ------------------------------------------------ encoders for the DICE side
# Only the simulator and the test suite send these; the flight payload never
# transmits a command or a request. They live here so that one module owns
# every byte layout and the two directions cannot drift apart.

def encode_command(payload: bytes, coarse_time: int = 0, fine_time: int = 0,
                   wire: Wire = DEFAULT_WIRE,
                   target_id: int | None = None, spare: int = 0x00) -> bytes:
    """Build a 120-byte Command packet (DICE side)."""
    if len(payload) > COMMAND_PAYLOAD_LENGTH:
        raise PacketError(f"command payload {len(payload)} exceeds "
                          f"{COMMAND_PAYLOAD_LENGTH} bytes")
    tid = wire.target_id if target_id is None else target_id

    packet = bytearray(COMMAND_PACKET_SIZE)
    packet[0:4] = wire.sync_bytes
    packet[COMMAND_COARSE_TIME_OFFSET:COMMAND_COARSE_TIME_OFFSET + 4] = \
        wire.u32(coarse_time)
    packet[COMMAND_FINE_TIME_OFFSET:COMMAND_FINE_TIME_OFFSET + 2] = \
        wire.u16(fine_time)
    packet[COMMAND_PACKET_TYPE_OFFSET] = PacketType.COMMAND
    packet[COMMAND_TARGET_ID_OFFSET] = tid & 0xFF
    packet[COMMAND_PAYLOAD_OFFSET:COMMAND_PAYLOAD_OFFSET + len(payload)] = payload
    packet[COMMAND_SPARE_OFFSET] = spare & 0xFF
    return wire.seal(packet, COMMAND_CRC_OFFSET)


def encode_short_request(packet_type: int, coarse_time: int = 0,
                         fine_time: int = 0, wire: Wire = DEFAULT_WIRE,
                         target_id: int | None = None) -> bytes:
    """Build a 14-byte LRT Request or HRT flow-control packet (DICE side)."""
    tid = wire.target_id if target_id is None else target_id

    packet = bytearray(SHORT_REQUEST_SIZE)
    packet[0:4] = wire.sync_bytes
    packet[SHORT_COARSE_TIME_OFFSET:SHORT_COARSE_TIME_OFFSET + 4] = \
        wire.u32(coarse_time)
    packet[SHORT_FINE_TIME_OFFSET:SHORT_FINE_TIME_OFFSET + 2] = \
        wire.u16(fine_time)
    packet[SHORT_PACKET_TYPE_OFFSET] = packet_type & 0xFF
    packet[SHORT_TARGET_ID_OFFSET] = tid & 0xFF
    return wire.seal(packet, SHORT_CRC_OFFSET)
