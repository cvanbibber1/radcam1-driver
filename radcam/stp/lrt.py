"""The 1248-byte LRT payload - housekeeping, and where command results land.

The ICD's Command Acknowledge packet is 8 bytes with no status field, so it can
only report "a valid command addressed to me arrived". It cannot say whether
the command worked, and a capture that takes seconds cannot be made to fit
inside an acknowledgement anyway. **LRT is therefore the only channel by which
the ground learns what a command actually did**, which drives the layout below:
the last-command block sits near the front, and carries the sequence number the
ground sent so a result can never be misattributed to the wrong command.

Layout is big-endian throughout, matching the packet envelope, and every offset
is a named constant so the ground decoder and this builder cannot drift.

Two things are deliberately redundant:

* **The dose fields are triplicated in the payload itself.** The envelope CRC
  detects corruption but cannot correct it, and a failed CRC costs a whole poll
  interval before the ground sees a fresh number. Triplication lets a ground
  station that chooses to salvage a CRC-failed packet still recover the
  measurement by majority vote. This mirrors what `radcam/telemetry.py` already
  does on the debug link, for the same reason.

* **A CRC-32 covers the payload independently of the envelope CRC-16.** The
  envelope proves the packet crossed the wire intact; this proves the payload
  was assembled intact, which is a different claim on a machine taking hits.
"""

from __future__ import annotations

import logging
import struct
import time
import zlib
from collections import deque
from dataclasses import dataclass, field

log = logging.getLogger(__name__)

__all__ = [
    "LRT_PAYLOAD_LEN", "LRT_FORMAT_VERSION", "EventLog", "Event", "EventCode",
    "build_lrt_payload", "decode_lrt_payload", "MAX_EVENTS",
    "RESP_DATA_MAX", "FILE_DATA_MAX", "FILE_IDLE", "FILE_ACTIVE",
    "FILE_COMPLETE", "FILE_ERROR", "FILE_FLAG_PARITY",
    "FILE_FLAG_LAST_DATA", "FILE_FLAG_RETRANSMIT",
]

LRT_PAYLOAD_LEN = 1248
LRT_FORMAT_VERSION = 0x0001

# ---- fixed header offsets -------------------------------------------------
OFF_VERSION = 0
OFF_FLAGS = 2
OFF_COARSE_TIME = 4
OFF_FINE_TIME = 8
OFF_UPTIME = 10
OFF_BOOT_COUNT = 14
OFF_TARGET_ID = 18
OFF_FW_MAJOR = 20
OFF_FW_MINOR = 21

# ---- last command result --------------------------------------------------
OFF_LAST_OPCODE = 22
OFF_LAST_CMD_SEQ = 23
OFF_LAST_RESULT = 25
OFF_LAST_DONE_UPTIME = 27
OFF_CMDS_RECEIVED = 31
OFF_CMDS_EXECUTED = 35
OFF_CMDS_REJECTED = 39

# ---- dosimeter, triplicated ----------------------------------------------
OFF_DOSE_RAD = 43          # float32 x3
OFF_DOSE_VOLTS = 55        # float32 x3
OFF_DOSE_CAL = 67          # uint8 x3
OFF_DOSE_SAMPLES = 70
OFF_DOSE_ERRORS = 74

# ---- housekeeping ---------------------------------------------------------
OFF_CPU_TEMP = 78
OFF_LED_PERCENT = 82
OFF_CAMERA_AVAILABLE = 83
OFF_RECORDING = 84
OFF_RECORDING_ID = 85
OFF_STORAGE_FREE = 89
OFF_STORAGE_USED = 97
OFF_MEDIA_COUNT = 105

# ---- link health ----------------------------------------------------------
OFF_RX_GOOD = 107
OFF_RX_BAD_CRC = 111
OFF_RX_BAD_FORMAT = 115
OFF_RX_NOT_FOR_US = 119
OFF_RX_UNKNOWN_TYPE = 123
OFF_RX_RESYNCS = 127
OFF_RX_DROPPED = 131
OFF_TX_PACKETS = 135
OFF_TX_ERRORS = 139
OFF_LRT_SENT = 143
OFF_HRT_SENT = 147

# ---- HRT and transfer state ----------------------------------------------
OFF_HRT_ENABLED = 151
OFF_HRT_LAST_CONTROL = 152
OFF_XFER_MEDIA_ID = 153
OFF_XFER_CHUNK_NEXT = 157
OFF_XFER_CHUNK_TOTAL = 161
OFF_XFER_BYTES_TOTAL = 165
OFF_XFER_FILE_CRC32 = 173
OFF_XFER_STATE = 177
OFF_XFER_QUEUE_DEPTH = 178

# ---- subsystem health -----------------------------------------------------
OFF_EEPROM_COPIES = 180
OFF_EEPROM_GOOD = 181
OFF_DOSIMETER_OK = 182
OFF_LED_OK = 183
OFF_CAMERA_OK = 184
OFF_TMR_CORRECTIONS = 185
OFF_TMR_FAILURES = 189
OFF_WATCHDOG_PETS = 193
OFF_SAFE_MODE = 197

# ---- last command response ------------------------------------------------
# Several commands return data rather than a bare result code - GET_CONFIG,
# EEPROM_READ, PONG, the media list. HRT would be the natural channel for
# those, but HRT only flows when DICE opens it, so a reply routed there could
# sit undelivered indefinitely. A response area in LRT means every small reply
# comes back on the channel that is always polled, and only genuinely large
# ones need the tap open.
OFF_RESP_OPCODE = 198
OFF_RESP_CMD_SEQ = 199
OFF_RESP_LEN = 201
#: Full length when the reply did not fit; 0 when `resp_data` holds all of it.
#: A non-zero value means the complete reply was also queued for HRT.
OFF_RESP_FULL_LEN = 203
OFF_RESP_DATA = 205
RESP_DATA_MAX = 256

# ---- LRT file transfer block ----------------------------------------------
# HRT is the fast path for bulk data, but it only flows when DICE opens the
# tap. If the master never sends HRT Go - because HRT is allocated to another
# experiment, or the schedule does not permit it - then without this block
# there is no way to get an image down at all. LRT is the one channel that is
# always polled, so a file can also be pulled through it, one chunk per poll:
# far slower, but never blocked on somebody else's permission.
#
# It is a *separate* block rather than a reuse of the response window so that a
# transfer in progress and a command reply can occupy the same LRT packet. If
# they shared the window, every command issued mid-transfer would stall it.
OFF_FILE_STATE = 461
OFF_FILE_FLAGS = 463
OFF_FILE_MEDIA_ID = 465
OFF_FILE_CHUNK_INDEX = 469
OFF_FILE_CHUNK_TOTAL = 473
OFF_FILE_SIZE = 477
OFF_FILE_CRC32 = 481
OFF_FILE_DATA_LEN = 485
OFF_FILE_FEC_GROUP = 487
OFF_FILE_DATA_CRC32 = 489
OFF_FILE_DATA = 493
FILE_DATA_MAX = 512
FILE_HEADER_LEN = OFF_FILE_DATA - OFF_FILE_STATE                 # 32

#: LRT file transfer states, reported in OFF_FILE_STATE.
FILE_IDLE, FILE_ACTIVE, FILE_COMPLETE, FILE_ERROR = 0, 1, 2, 3

#: This chunk is XOR parity for the group named by chunk_index, not file data.
FILE_FLAG_PARITY = 0x0001
#: This is the last *data* chunk of the file.
FILE_FLAG_LAST_DATA = 0x0002
#: This chunk is being sent again after an explicit resend request.
FILE_FLAG_RETRANSMIT = 0x0004

# ---- event ring -----------------------------------------------------------
OFF_EVENT_COUNT = OFF_FILE_DATA + FILE_DATA_MAX                  # 1005
OFF_EVENTS = OFF_EVENT_COUNT + 2                                 # 1007
EVENT_SIZE = 12
OFF_PAYLOAD_CRC32 = 1244
MAX_EVENTS = (OFF_PAYLOAD_CRC32 - OFF_EVENTS) // EVENT_SIZE      # 19

#: Transfer states reported in OFF_XFER_STATE.
XFER_IDLE, XFER_ACTIVE, XFER_PAUSED, XFER_COMPLETE = 0, 1, 2, 3


class EventCode:
    """Codes for the event ring. Stable numbers - the ground decodes these."""

    BOOT = 0x0001
    SAFE_MODE_ENTERED = 0x0002
    SAFE_MODE_CLEARED = 0x0003
    COMMAND_ACCEPTED = 0x0010
    COMMAND_REJECTED = 0x0011
    COMMAND_FAILED = 0x0012
    COMMAND_DUPLICATE = 0x0013
    CAPTURE_OK = 0x0020
    CAPTURE_FAILED = 0x0021
    RECORD_STARTED = 0x0022
    RECORD_STOPPED = 0x0023
    TRANSFER_STARTED = 0x0030
    TRANSFER_COMPLETE = 0x0031
    TRANSFER_ABORTED = 0x0032
    HRT_GO = 0x0033
    HRT_STOP = 0x0034
    HRT_STOP_WITH_LOSS = 0x0035
    RX_CRC_BURST = 0x0040
    TX_FAILED = 0x0041
    TMR_CORRECTED = 0x0050
    TMR_UNRECOVERABLE = 0x0051
    EEPROM_DEGRADED = 0x0052
    EEPROM_REPAIRED = 0x0053
    DOSIMETER_FAULT = 0x0060
    CAMERA_FAULT = 0x0061
    LED_FAULT = 0x0062
    WATCHDOG_LATE = 0x0070


SEV_INFO, SEV_WARN, SEV_ERROR = 0, 1, 2


@dataclass(frozen=True)
class Event:
    uptime_s: int
    code: int
    arg: int = 0
    severity: int = SEV_INFO

    def pack(self) -> bytes:
        return struct.pack(">IHIBB", self.uptime_s & 0xFFFFFFFF,
                           self.code & 0xFFFF, self.arg & 0xFFFFFFFF,
                           self.severity & 0xFF, 0)

    @classmethod
    def unpack(cls, raw: bytes) -> "Event":
        uptime, code, arg, sev, _ = struct.unpack(">IHIBB", raw)
        return cls(uptime, code, arg, sev)


class EventLog:
    """A bounded ring of recent events, drained into each LRT payload.

    The ground cannot ask "what happened while you were out of contact?" over
    this ICD - there is no query for it. So every LRT carries the most recent
    events unconditionally, and the ring is sized to the space available. It is
    lossy by construction: under a storm of events the oldest are overwritten,
    which is the right failure, because the newest events are the ones that
    explain the current state.
    """

    def __init__(self, capacity: int = MAX_EVENTS):
        self.capacity = min(capacity, MAX_EVENTS)
        self._events: deque[Event] = deque(maxlen=self.capacity)
        self.total = 0

    def add(self, code: int, arg: int = 0, severity: int = SEV_INFO,
            uptime_s: int | None = None) -> None:
        if uptime_s is None:
            uptime_s = int(time.monotonic())
        self._events.append(Event(uptime_s, code, arg, severity))
        self.total += 1

    def recent(self, limit: int | None = None) -> list[Event]:
        events = list(self._events)
        if limit is not None:
            events = events[-limit:]
        return events

    def __len__(self) -> int:
        return len(self._events)


def _pack_into(buf: bytearray, offset: int, fmt: str, *values) -> None:
    struct.pack_into(">" + fmt, buf, offset, *values)


def _triplicate_f32(buf: bytearray, offset: int, value: float) -> None:
    raw = struct.pack(">f", value)
    buf[offset:offset + 12] = raw * 3


def _triplicate_u8(buf: bytearray, offset: int, value: int) -> None:
    buf[offset:offset + 3] = bytes([value & 0xFF]) * 3


def build_lrt_payload(state: dict, events: list[Event] | None = None) -> bytes:
    """Assemble the 1248-byte LRT payload from a flat state dictionary.

    Missing keys default to zero rather than raising: a telemetry frame that
    cannot be built is worse than one carrying a zero, because it takes the
    whole housekeeping channel down with it. The caller is expected to be
    generous with what it supplies and this function to be forgiving.
    """
    buf = bytearray(LRT_PAYLOAD_LEN)
    g = state.get

    _pack_into(buf, OFF_VERSION, "H", LRT_FORMAT_VERSION)
    _pack_into(buf, OFF_FLAGS, "H", int(g("flags", 0)))
    _pack_into(buf, OFF_COARSE_TIME, "I", int(g("coarse_time", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_FINE_TIME, "H", int(g("fine_time", 0)) & 0xFFFF)
    _pack_into(buf, OFF_UPTIME, "I", int(g("uptime_s", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_BOOT_COUNT, "I", int(g("boot_count", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_TARGET_ID, "H", int(g("target_id", 0)) & 0xFFFF)
    buf[OFF_FW_MAJOR] = int(g("fw_major", 1)) & 0xFF
    buf[OFF_FW_MINOR] = int(g("fw_minor", 0)) & 0xFF

    buf[OFF_LAST_OPCODE] = int(g("last_opcode", 0)) & 0xFF
    _pack_into(buf, OFF_LAST_CMD_SEQ, "H", int(g("last_cmd_seq", 0)) & 0xFFFF)
    buf[OFF_LAST_RESULT] = int(g("last_result", 0)) & 0xFF
    _pack_into(buf, OFF_LAST_DONE_UPTIME, "I",
               int(g("last_done_uptime", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_CMDS_RECEIVED, "I", int(g("cmds_received", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_CMDS_EXECUTED, "I", int(g("cmds_executed", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_CMDS_REJECTED, "I", int(g("cmds_rejected", 0)) & 0xFFFFFFFF)

    _triplicate_f32(buf, OFF_DOSE_RAD, float(g("dose_rad", 0.0)))
    _triplicate_f32(buf, OFF_DOSE_VOLTS, float(g("dose_volts", 0.0)))
    _triplicate_u8(buf, OFF_DOSE_CAL, 1 if g("dose_calibrated") else 0)
    _pack_into(buf, OFF_DOSE_SAMPLES, "I", int(g("dose_samples", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_DOSE_ERRORS, "I", int(g("dose_errors", 0)) & 0xFFFFFFFF)

    _pack_into(buf, OFF_CPU_TEMP, "f", float(g("cpu_temp_c", 0.0)))
    buf[OFF_LED_PERCENT] = int(g("led_percent", 0)) & 0xFF
    buf[OFF_CAMERA_AVAILABLE] = 1 if g("camera_available") else 0
    buf[OFF_RECORDING] = 1 if g("recording") else 0
    _pack_into(buf, OFF_RECORDING_ID, "I", int(g("recording_id", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_STORAGE_FREE, "Q", int(g("storage_free", 0)) & (2**64 - 1))
    _pack_into(buf, OFF_STORAGE_USED, "Q", int(g("storage_used", 0)) & (2**64 - 1))
    _pack_into(buf, OFF_MEDIA_COUNT, "H", int(g("media_count", 0)) & 0xFFFF)

    for offset, key in ((OFF_RX_GOOD, "rx_good"),
                        (OFF_RX_BAD_CRC, "rx_bad_crc"),
                        (OFF_RX_BAD_FORMAT, "rx_bad_format"),
                        (OFF_RX_NOT_FOR_US, "rx_not_for_us"),
                        (OFF_RX_UNKNOWN_TYPE, "rx_unknown_type"),
                        (OFF_RX_RESYNCS, "rx_resyncs"),
                        (OFF_RX_DROPPED, "rx_dropped_bytes"),
                        (OFF_TX_PACKETS, "tx_packets"),
                        (OFF_TX_ERRORS, "tx_errors"),
                        (OFF_LRT_SENT, "lrt_sent"),
                        (OFF_HRT_SENT, "hrt_sent")):
        _pack_into(buf, offset, "I", int(g(key, 0)) & 0xFFFFFFFF)

    buf[OFF_HRT_ENABLED] = 1 if g("hrt_enabled") else 0
    buf[OFF_HRT_LAST_CONTROL] = int(g("hrt_last_control", 0)) & 0xFF
    _pack_into(buf, OFF_XFER_MEDIA_ID, "I", int(g("xfer_media_id", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_XFER_CHUNK_NEXT, "I", int(g("xfer_chunk_next", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_XFER_CHUNK_TOTAL, "I", int(g("xfer_chunk_total", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_XFER_BYTES_TOTAL, "Q", int(g("xfer_bytes_total", 0)) & (2**64 - 1))
    _pack_into(buf, OFF_XFER_FILE_CRC32, "I", int(g("xfer_file_crc32", 0)) & 0xFFFFFFFF)
    buf[OFF_XFER_STATE] = int(g("xfer_state", XFER_IDLE)) & 0xFF
    _pack_into(buf, OFF_XFER_QUEUE_DEPTH, "H", int(g("xfer_queue_depth", 0)) & 0xFFFF)

    buf[OFF_EEPROM_COPIES] = int(g("eeprom_copies", 0)) & 0xFF
    buf[OFF_EEPROM_GOOD] = int(g("eeprom_good", 0)) & 0xFF
    buf[OFF_DOSIMETER_OK] = 1 if g("dosimeter_ok") else 0
    buf[OFF_LED_OK] = 1 if g("led_ok") else 0
    buf[OFF_CAMERA_OK] = 1 if g("camera_ok") else 0
    _pack_into(buf, OFF_TMR_CORRECTIONS, "I", int(g("tmr_corrections", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_TMR_FAILURES, "I", int(g("tmr_failures", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_WATCHDOG_PETS, "I", int(g("watchdog_pets", 0)) & 0xFFFFFFFF)
    buf[OFF_SAFE_MODE] = 1 if g("safe_mode") else 0

    resp = g("resp_data", b"") or b""
    buf[OFF_RESP_OPCODE] = int(g("resp_opcode", 0)) & 0xFF
    _pack_into(buf, OFF_RESP_CMD_SEQ, "H", int(g("resp_cmd_seq", 0)) & 0xFFFF)
    fitted = resp[:RESP_DATA_MAX]
    _pack_into(buf, OFF_RESP_LEN, "H", len(fitted))
    _pack_into(buf, OFF_RESP_FULL_LEN, "H",
               len(resp) & 0xFFFF if len(resp) > RESP_DATA_MAX else 0)
    buf[OFF_RESP_DATA:OFF_RESP_DATA + len(fitted)] = fitted

    chunk = g("file_data", b"") or b""
    _pack_into(buf, OFF_FILE_STATE, "H", int(g("file_state", FILE_IDLE)) & 0xFFFF)
    _pack_into(buf, OFF_FILE_FLAGS, "H", int(g("file_flags", 0)) & 0xFFFF)
    _pack_into(buf, OFF_FILE_MEDIA_ID, "I", int(g("file_media_id", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_FILE_CHUNK_INDEX, "I",
               int(g("file_chunk_index", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_FILE_CHUNK_TOTAL, "I",
               int(g("file_chunk_total", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_FILE_SIZE, "I", int(g("file_size", 0)) & 0xFFFFFFFF)
    _pack_into(buf, OFF_FILE_CRC32, "I", int(g("file_crc32", 0)) & 0xFFFFFFFF)
    fitted_chunk = chunk[:FILE_DATA_MAX]
    _pack_into(buf, OFF_FILE_DATA_LEN, "H", len(fitted_chunk))
    _pack_into(buf, OFF_FILE_FEC_GROUP, "H", int(g("file_fec_group", 0)) & 0xFFFF)
    _pack_into(buf, OFF_FILE_DATA_CRC32, "I",
               zlib.crc32(fitted_chunk) & 0xFFFFFFFF)
    buf[OFF_FILE_DATA:OFF_FILE_DATA + len(fitted_chunk)] = fitted_chunk

    events = events or []
    n = min(len(events), MAX_EVENTS)
    _pack_into(buf, OFF_EVENT_COUNT, "H", n)
    # Newest events last, and the newest are the ones worth keeping if the
    # ring holds more than fits.
    for i, event in enumerate(events[-n:] if n else []):
        off = OFF_EVENTS + i * EVENT_SIZE
        buf[off:off + EVENT_SIZE] = event.pack()

    crc = zlib.crc32(bytes(buf[:OFF_PAYLOAD_CRC32])) & 0xFFFFFFFF
    _pack_into(buf, OFF_PAYLOAD_CRC32, "I", crc)
    return bytes(buf)


def _vote_f32(buf: bytes, offset: int) -> float:
    """Majority-vote three float32 copies; fall back to the first."""
    copies = [buf[offset + i * 4:offset + i * 4 + 4] for i in range(3)]
    for candidate in copies:
        if copies.count(candidate) >= 2:
            return struct.unpack(">f", candidate)[0]
    return struct.unpack(">f", copies[0])[0]


def decode_lrt_payload(payload: bytes) -> dict:
    """Decode an LRT payload. Used by ground tooling, the simulator and tests.

    `payload_crc_ok` is reported rather than enforced: a payload whose CRC-32
    fails may still carry a recoverable dose reading through the triplicated
    fields, and it is the ground's decision whether to salvage it.
    """
    if len(payload) != LRT_PAYLOAD_LEN:
        raise ValueError(f"LRT payload must be {LRT_PAYLOAD_LEN} bytes, "
                         f"got {len(payload)}")

    u = lambda fmt, off: struct.unpack_from(">" + fmt, payload, off)[0]  # noqa: E731

    out = {
        "format_version": u("H", OFF_VERSION),
        "flags": u("H", OFF_FLAGS),
        "coarse_time": u("I", OFF_COARSE_TIME),
        "fine_time": u("H", OFF_FINE_TIME),
        "uptime_s": u("I", OFF_UPTIME),
        "boot_count": u("I", OFF_BOOT_COUNT),
        "target_id": u("H", OFF_TARGET_ID),
        "fw_major": payload[OFF_FW_MAJOR],
        "fw_minor": payload[OFF_FW_MINOR],
        "last_opcode": payload[OFF_LAST_OPCODE],
        "last_cmd_seq": u("H", OFF_LAST_CMD_SEQ),
        "last_result": payload[OFF_LAST_RESULT],
        "last_done_uptime": u("I", OFF_LAST_DONE_UPTIME),
        "cmds_received": u("I", OFF_CMDS_RECEIVED),
        "cmds_executed": u("I", OFF_CMDS_EXECUTED),
        "cmds_rejected": u("I", OFF_CMDS_REJECTED),
        "dose_rad": _vote_f32(payload, OFF_DOSE_RAD),
        "dose_volts": _vote_f32(payload, OFF_DOSE_VOLTS),
        "dose_calibrated": bool(max(set(payload[OFF_DOSE_CAL:OFF_DOSE_CAL + 3]),
                                    key=list(payload[OFF_DOSE_CAL:OFF_DOSE_CAL + 3]).count)),
        "dose_samples": u("I", OFF_DOSE_SAMPLES),
        "dose_errors": u("I", OFF_DOSE_ERRORS),
        "cpu_temp_c": u("f", OFF_CPU_TEMP),
        "led_percent": payload[OFF_LED_PERCENT],
        "camera_available": bool(payload[OFF_CAMERA_AVAILABLE]),
        "recording": bool(payload[OFF_RECORDING]),
        "recording_id": u("I", OFF_RECORDING_ID),
        "storage_free": u("Q", OFF_STORAGE_FREE),
        "storage_used": u("Q", OFF_STORAGE_USED),
        "media_count": u("H", OFF_MEDIA_COUNT),
        "hrt_enabled": bool(payload[OFF_HRT_ENABLED]),
        "hrt_last_control": payload[OFF_HRT_LAST_CONTROL],
        "xfer_media_id": u("I", OFF_XFER_MEDIA_ID),
        "xfer_chunk_next": u("I", OFF_XFER_CHUNK_NEXT),
        "xfer_chunk_total": u("I", OFF_XFER_CHUNK_TOTAL),
        "xfer_bytes_total": u("Q", OFF_XFER_BYTES_TOTAL),
        "xfer_file_crc32": u("I", OFF_XFER_FILE_CRC32),
        "xfer_state": payload[OFF_XFER_STATE],
        "xfer_queue_depth": u("H", OFF_XFER_QUEUE_DEPTH),
        "eeprom_copies": payload[OFF_EEPROM_COPIES],
        "eeprom_good": payload[OFF_EEPROM_GOOD],
        "dosimeter_ok": bool(payload[OFF_DOSIMETER_OK]),
        "led_ok": bool(payload[OFF_LED_OK]),
        "camera_ok": bool(payload[OFF_CAMERA_OK]),
        "tmr_corrections": u("I", OFF_TMR_CORRECTIONS),
        "tmr_failures": u("I", OFF_TMR_FAILURES),
        "watchdog_pets": u("I", OFF_WATCHDOG_PETS),
        "safe_mode": bool(payload[OFF_SAFE_MODE]),
        "resp_opcode": payload[OFF_RESP_OPCODE],
        "resp_cmd_seq": u("H", OFF_RESP_CMD_SEQ),
        "resp_full_len": u("H", OFF_RESP_FULL_LEN),
    }
    resp_len = min(u("H", OFF_RESP_LEN), RESP_DATA_MAX)
    out["resp_data"] = payload[OFF_RESP_DATA:OFF_RESP_DATA + resp_len]
    #: True when the reply was too big for LRT and also went out over HRT.
    out["resp_truncated"] = out["resp_full_len"] > 0

    file_len = min(u("H", OFF_FILE_DATA_LEN), FILE_DATA_MAX)
    file_data = payload[OFF_FILE_DATA:OFF_FILE_DATA + file_len]
    file_flags = u("H", OFF_FILE_FLAGS)
    out.update({
        "file_state": u("H", OFF_FILE_STATE),
        "file_flags": file_flags,
        "file_media_id": u("I", OFF_FILE_MEDIA_ID),
        "file_chunk_index": u("I", OFF_FILE_CHUNK_INDEX),
        "file_chunk_total": u("I", OFF_FILE_CHUNK_TOTAL),
        "file_size": u("I", OFF_FILE_SIZE),
        "file_crc32": u("I", OFF_FILE_CRC32),
        "file_fec_group": u("H", OFF_FILE_FEC_GROUP),
        "file_data": file_data,
        "file_data_crc32": u("I", OFF_FILE_DATA_CRC32),
        # Reported, never enforced: a chunk failing this is exactly what the
        # parity groups exist to reconstruct.
        "file_data_crc_ok": (zlib.crc32(file_data) & 0xFFFFFFFF)
                            == u("I", OFF_FILE_DATA_CRC32),
        "file_is_parity": bool(file_flags & FILE_FLAG_PARITY),
        "file_last_data": bool(file_flags & FILE_FLAG_LAST_DATA),
        "file_retransmit": bool(file_flags & FILE_FLAG_RETRANSMIT),
    })

    for offset, key in ((OFF_RX_GOOD, "rx_good"),
                        (OFF_RX_BAD_CRC, "rx_bad_crc"),
                        (OFF_RX_BAD_FORMAT, "rx_bad_format"),
                        (OFF_RX_NOT_FOR_US, "rx_not_for_us"),
                        (OFF_RX_UNKNOWN_TYPE, "rx_unknown_type"),
                        (OFF_RX_RESYNCS, "rx_resyncs"),
                        (OFF_RX_DROPPED, "rx_dropped_bytes"),
                        (OFF_TX_PACKETS, "tx_packets"),
                        (OFF_TX_ERRORS, "tx_errors"),
                        (OFF_LRT_SENT, "lrt_sent"),
                        (OFF_HRT_SENT, "hrt_sent")):
        out[key] = u("I", offset)

    count = min(u("H", OFF_EVENT_COUNT), MAX_EVENTS)
    out["events"] = [
        Event.unpack(payload[OFF_EVENTS + i * EVENT_SIZE:
                             OFF_EVENTS + (i + 1) * EVENT_SIZE])
        for i in range(count)
    ]

    stored = u("I", OFF_PAYLOAD_CRC32)
    out["payload_crc32"] = stored
    out["payload_crc_ok"] = (zlib.crc32(payload[:OFF_PAYLOAD_CRC32]) & 0xFFFFFFFF) == stored
    return out
