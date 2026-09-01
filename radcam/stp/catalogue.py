"""Every command the ground can send, as data rather than as prose.

The ground station for this mission pastes a command in as a **single hex
string**: the complete 120-byte packet, sync through CRC, ready to go on the
wire. That makes the command set a table, not documentation - so it lives here,
once, and both `tools/stp-command.py` and the generated reference in the user
guide render from it. A command cannot be documented wrongly if the
documentation is generated from the thing that builds it.

Two consequences worth knowing before pasting anything:

**Canned commands carry the force flag.** A stored hex string has a fixed
`cmd_seq` baked into it, and the payload suppresses a repeated sequence number
as a retransmission - so without the force flag, pasting the same string twice
would run it once. Every string generated here sets the flag, which means
"execute this even if you have seen the sequence number", and pasting it ten
times captures ten images. A ground station that generates its own sequence
numbers should turn it off and get retransmission protection instead.

**The timestamp is zero.** Coarse and fine time are the master's to fill in;
the payload only echoes what it receives, and nothing it does depends on the
value. A canned string with a zero timestamp is valid.

Argument fields are little-endian, matching the `radcam.protocol` payload
format that the dispatcher decodes. The packet envelope around them is
big-endian, per the ICD. See `docs/stp/README.md` for why the two differ.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

from ..protocol import Msg
from .commands import FLAG_FORCE, encode_command_payload
from .experiment import StpOp
from .packets import (DEFAULT_WIRE, PacketType, Wire, encode_command,
                      encode_short_request)

__all__ = ["Field", "CommandSpec", "CATALOGUE", "SHORT_PACKETS", "find",
           "build_command_hex", "build_short_hex", "GROUPS"]


@dataclass(frozen=True)
class Field:
    name: str
    fmt: str                 # struct format, little-endian
    default: int = 0
    help: str = ""

    @property
    def size(self) -> int:
        return struct.calcsize("<" + self.fmt)


@dataclass(frozen=True)
class CommandSpec:
    name: str
    opcode: int
    summary: str
    group: str = "general"
    fields: tuple = ()

    def pack_args(self, values: dict | None = None) -> bytes:
        values = values or {}
        out = b""
        for f in self.fields:
            out += struct.pack("<" + f.fmt, values.get(f.name, f.default))
        return out

    def signature(self) -> str:
        if not self.fields:
            return "(none)"
        return ", ".join(f"{f.name}:{f.fmt}" for f in self.fields)


GROUPS = ("imaging", "slots", "stream", "transfer", "link", "config",
          "telemetry", "eeprom")

_U8 = "B"
_U16 = "H"
_U32 = "I"

CATALOGUE: tuple[CommandSpec, ...] = (
    # -- telemetry and liveness ------------------------------------------
    CommandSpec("PING", Msg.PING, "Liveness check; replies with uptime and version",
                "telemetry"),
    CommandSpec("GET_TELEMETRY", Msg.GET_TELEMETRY,
                "Housekeeping snapshot in the LRT response window", "telemetry"),
    CommandSpec("GET_LINK_STATS", StpOp.GET_LINK_STATS,
                "Receive counters, CRC failures and resync counts", "telemetry"),
    CommandSpec("GET_DOSE_LOG", Msg.GET_DOSE_LOG,
                "Dose history; zero range means everything", "telemetry"),

    # -- configuration ----------------------------------------------------
    CommandSpec("GET_CONFIG", Msg.GET_CONFIG, "Read the effective configuration",
                "config"),
    CommandSpec("SET_CONFIG", Msg.SET_CONFIG, "Set one configuration key",
                "config",
                (Field("key", _U8, 0x06, "config key, see Cfg"),
                 Field("value", _U32, 2, "new value"))),
    CommandSpec("SET_LED", Msg.SET_LED,
                "Illumination percent; clamped to 10", "config",
                (Field("percent", _U8, 5, "0-10"),)),
    CommandSpec("SET_FEC_GROUP", StpOp.SET_FEC_GROUP,
                "Parity group size for HRT transfers; 0 disables", "config",
                (Field("group", _U8, 16, "chunks per parity chunk"),)),
    CommandSpec("SET_HRT_IDLE_FILL", StpOp.SET_HRT_IDLE_FILL,
                "Send idle HRT packets when nothing is queued", "config",
                (Field("enable", _U8, 0, "0 or 1"),)),

    # -- imaging ----------------------------------------------------------
    CommandSpec("CAPTURE_IMAGE", Msg.CAPTURE_IMAGE,
                "Capture into the media store (prefer SLOT_CAPTURE_IMAGE)",
                "imaging"),
    CommandSpec("START_RECORD", Msg.START_RECORD,
                "Record into the media store (prefer SLOT_RECORD_START)",
                "imaging"),
    CommandSpec("STOP_RECORD", Msg.STOP_RECORD, "End a media-store recording",
                "imaging"),
    CommandSpec("CAPTURE_REGION", Msg.CAPTURE_REGION,
                "Full-resolution capture cropped to a window and rescaled",
                "imaging",
                (Field("x", _U16, 1784, "crop left, sensor pixels"),
                 Field("y", _U16, 1320, "crop top"),
                 Field("w", _U16, 640, "crop width"),
                 Field("h", _U16, 480, "crop height"),
                 Field("out_w", _U16, 640, "output width"),
                 Field("out_h", _U16, 480, "output height"))),

    # -- storage slots ----------------------------------------------------
    CommandSpec("SLOT_LIST", StpOp.SLOT_LIST,
                "Every slot: kind, size, dimensions, CRC-32, timestamp", "slots"),
    CommandSpec("SLOT_INFO", StpOp.SLOT_INFO, "One slot in detail", "slots",
                (Field("slot", _U8, 0, "slot index"),)),
    CommandSpec("SLOT_CAPTURE_IMAGE", StpOp.SLOT_CAPTURE_IMAGE,
                "Take a still into a slot, overwriting it", "slots",
                (Field("slot", _U8, 0, "slot index"),)),
    CommandSpec("SLOT_RECORD_START", StpOp.SLOT_RECORD_START,
                "Record video into a slot; held on the Pi, not streamed", "slots",
                (Field("slot", _U8, 1, "slot index"),
                 Field("seconds", _U16, 30, "auto-stop after N s; 0 = manual"))),
    CommandSpec("SLOT_RECORD_STOP", StpOp.SLOT_RECORD_STOP,
                "End the recording and finalise its slot", "slots"),
    CommandSpec("SLOT_DELETE", StpOp.SLOT_DELETE,
                "Free one slot for reuse", "slots",
                (Field("slot", _U8, 0, "slot index"),)),
    CommandSpec("SLOT_DELETE_ALL", StpOp.SLOT_DELETE_ALL,
                "Free every slot", "slots"),

    # -- camera selection -------------------------------------------------
    CommandSpec("CAMERA_LIST", StpOp.CAMERA_LIST,
                "Every camera: index, enable GPIO, which one is on", "imaging"),
    CommandSpec("SELECT_CAMERA", StpOp.SELECT_CAMERA,
                "Enable one camera and disable all others; 0xFF disables all",
                "imaging",
                (Field("camera", _U8, 0, "camera index 0-15, or 255 for none"),)),

    # -- transfer ---------------------------------------------------------
    CommandSpec("SLOT_DOWNLOAD", StpOp.SLOT_DOWNLOAD,
                "Queue a slot for HRT transfer", "transfer",
                (Field("slot", _U8, 0, "slot index"),)),
    CommandSpec("SLOT_DOWNLOAD_ABORT", StpOp.SLOT_DOWNLOAD_ABORT,
                "Remove a slot from the transfer queue", "transfer",
                (Field("slot", _U8, 0, "slot index"),)),
    CommandSpec("ABORT_TRANSFERS", StpOp.ABORT_TRANSFERS,
                "Empty the whole HRT transfer queue", "transfer"),
    CommandSpec("RESEND", Msg.RESEND,
                "Re-send specific chunks of a transfer", "transfer",
                (Field("media_id", _U32, 0x51000000, "transfer id"),
                 Field("chunk", _U32, 0, "chunk index"))),
    CommandSpec("GET_MEDIA_LIST", Msg.GET_MEDIA_LIST,
                "Legacy media store listing", "transfer"),
    CommandSpec("REQUEST_MEDIA", Msg.REQUEST_MEDIA,
                "Queue a legacy media id for HRT", "transfer",
                (Field("media_id", _U32, 1, "media id"),)),
    CommandSpec("DELETE_MEDIA", Msg.DELETE_MEDIA, "Delete legacy media",
                "transfer", (Field("media_id", _U32, 1, "media id"),)),

    # -- live stream ------------------------------------------------------
    CommandSpec("STREAM_START", StpOp.STREAM_START,
                "Start live video over HRT", "stream",
                (Field("width", _U16, 640, "output width"),
                 Field("height", _U16, 480, "output height"),
                 Field("fps", _U8, 15, "frames per second"),
                 Field("bitrate", _U32, 600000, "bits per second"),
                 Field("centre_x", _U16, 2104, "sensor pixel the crop centres on"),
                 Field("centre_y", _U16, 1560, "sensor pixel the crop centres on"),
                 Field("crop_w", _U16, 640, "crop width; equal to output = native"),
                 Field("crop_h", _U16, 480, "crop height"))),
    CommandSpec("STREAM_STOP", StpOp.STREAM_STOP, "Stop live video", "stream"),
    CommandSpec("STREAM_SET_OUTPUT", StpOp.STREAM_SET_OUTPUT,
                "Resolution, frame rate and bitrate", "stream",
                (Field("width", _U16, 640, "output width"),
                 Field("height", _U16, 480, "output height"),
                 Field("fps", _U8, 15, "frames per second"),
                 Field("bitrate", _U32, 600000, "bits per second"))),
    CommandSpec("STREAM_SET_REGION", StpOp.STREAM_SET_REGION,
                "Aim the crop box at a sensor pixel", "stream",
                (Field("centre_x", _U16, 2104, "sensor pixel, 0-4207"),
                 Field("centre_y", _U16, 1560, "sensor pixel, 0-3119"),
                 Field("crop_w", _U16, 640, "crop width"),
                 Field("crop_h", _U16, 480, "crop height"))),

    # -- link management --------------------------------------------------
    CommandSpec("CLEAR_SAFE_MODE", StpOp.CLEAR_SAFE_MODE,
                "Resume normal operation after safe mode", "link"),

    # -- calibration EEPROM ----------------------------------------------
    CommandSpec("EEPROM_STATUS", Msg.EEPROM_STATUS,
                "Which redundant calibration copies still verify", "eeprom"),
    CommandSpec("EEPROM_REPAIR", Msg.EEPROM_REPAIR,
                "Rewrite every copy from a surviving one", "eeprom"),
    CommandSpec("EEPROM_READ", Msg.EEPROM_READ, "Read calibration bytes",
                "eeprom",
                (Field("offset", _U16, 0, "byte offset"),
                 Field("length", _U16, 128, "bytes, max 128"))),
)

#: The 14-byte packets DICE sends that are not commands. They are hex strings
#: the ground pastes exactly like commands, so they belong in the same table.
SHORT_PACKETS = (
    ("LRT_REQUEST", PacketType.LRT_REQUEST,
     "Poll telemetry; the payload replies with one LRT Data packet"),
    ("HRT_GO", PacketType.HRT_GO,
     "Open the HRT tap - nothing bulk is transmitted until this arrives"),
    ("HRT_STOP", PacketType.HRT_STOP,
     "Close the tap, letting the packet in flight finish"),
    ("HRT_STOP_WITH_LOSS", PacketType.HRT_STOP_WITH_LOSS,
     "Close the tap immediately, truncating any packet in flight"),
)


def find(name: str) -> CommandSpec | None:
    upper = name.upper()
    return next((c for c in CATALOGUE if c.name == upper), None)


def build_command_hex(spec: CommandSpec, values: dict | None = None,
                      wire: Wire = DEFAULT_WIRE, cmd_seq: int = 1,
                      force: bool = True) -> str:
    """The complete 120-byte command packet, as a hex string."""
    payload = encode_command_payload(
        spec.opcode, cmd_seq, spec.pack_args(values),
        FLAG_FORCE if force else 0, crc=wire.crc)
    return encode_command(payload, 0, 0, wire).hex().upper()


def build_short_hex(packet_type: int, wire: Wire = DEFAULT_WIRE) -> str:
    """The complete 14-byte request packet, as a hex string."""
    return encode_short_request(packet_type, 0, 0, wire).hex().upper()
