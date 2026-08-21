"""The DICE-side experiment state machine.

This is the module that enforces the one rule the rest of the payload has never
had to obey: **we are a slave on a shared bus and may not speak unless spoken
to.** Every transmission originates in `service()`, on one thread, in direct
response to a packet from DICE or inside an HRT window DICE has opened. Nothing
else in the payload is allowed to touch the link.

Three design decisions are worth stating up front, because each is a departure
from how `radcam/protocol.py` worked on the old point-to-point link:

**The ACK means "accepted", not "done".** The ICD's Command Acknowledge is 8
bytes with no status field, and commands like CAPTURE_IMAGE take seconds. So a
valid command is acknowledged immediately and executed on a worker thread; the
result appears in LRT telemetry, keyed by the `cmd_seq` the ground supplied.
Blocking the receive loop until a capture finished would drop every packet that
arrived meanwhile, including commands for other experiments we are obliged to
ignore politely rather than miss.

**Responses are pulled, never pushed.** A reply is stashed and travels out on
the next LRT poll, with anything over 512 bytes additionally queued for HRT
under a synthetic media id. Nothing waits for a channel that may never open.

**Failure degrades rather than stops.** Repeated command failures trip safe
mode, which stops HRT and holds capture, but leaves LRT answering - because a
payload that has gone quiet is indistinguishable from a dead one, and the
ground needs the housekeeping most exactly when things are going wrong.

### Byte order

Two conventions meet in this module, and mixing them up is a silent bug that
looks like a malformed command:

* **ICD-defined structures are big-endian** - the packet envelope, the 105-byte
  command header, and the LRT and HRT payloads. Those are built in
  `packets.py`, `commands.py`, `lrt.py` and `hrt.py`.
* **Command arguments and response blobs are little-endian**, because they are
  the existing `radcam.protocol` wire format, which this module dispatches to
  unchanged. Every `struct` call *in this file* is therefore little-endian.

Rewriting the dispatcher to big-endian was the alternative; it would have
touched thirty call sites of already-tested code and changed the format the
`radcamctl` CLI speaks, for no benefit to the link. The ground station needs
one sentence of documentation instead.
"""

from __future__ import annotations

import logging
import queue
import struct
import threading
import time
import zlib
from collections import OrderedDict
from dataclasses import dataclass, field, replace

from ..framing import Frame
from ..protocol import Err, Msg
from . import lrt as L
from .commands import CommandDecodeError, decode_command_payload
from .fec import DEFAULT_GROUP_SIZE
from .hrt import (FLAG_KEYFRAME, FLAG_LAST_CHUNK, HRT_CHUNK_DATA, SubType,
                  TransferManager, build_hrt_payload)
from .packets import (
    DEFAULT_WIRE, PacketType, Wire, encode_command_ack, encode_hrt_data,
    encode_lrt_data,
)
from .redundancy import Scrubber, TMRBool, TMRInt
from .rx import PacketReader

log = logging.getLogger(__name__)

__all__ = ["Experiment", "ExperimentConfig", "StpOp"]


class StpOp:
    """Link-level opcodes handled here rather than by the media dispatcher.

    These exist so the link itself can be managed from the ground without a
    reboot: an experiment that can only be recovered by power-cycling is not
    autonomous. They occupy the command opcode space above the media commands.
    """

    CLEAR_SAFE_MODE = 0x70
    ABORT_TRANSFERS = 0x71
    GET_LINK_STATS = 0x72
    SET_HRT_IDLE_FILL = 0x73
    #: Parity group size for HRT transfers; 0 disables forward error
    #: correction, larger is less overhead and less protection.
    SET_FEC_GROUP = 0x77
    #: Live video over HRT. The stream shares the channel with file transfer
    #: and takes priority while it is running, because it is the only traffic
    #: on the link whose value expires.
    STREAM_START = 0x78
    STREAM_STOP = 0x79
    STREAM_SET_REGION = 0x7A
    STREAM_SET_OUTPUT = 0x7B


#: Synthetic media ids for command responses too large for LRT. The high byte
#: is reserved so these can never collide with a real capture's id.
_RESPONSE_ID_BASE = 0xFF000000

#: Distinguishes "never seen this cmd_seq" from "seen, still running" (None).
_UNSEEN = object()



@dataclass
class ExperimentConfig:
    target_id: int = 0xC7
    version: str = "1.0"
    #: HRT payloads emitted per `service()` call. Bounds how long one pass can
    #: hold the bus, so a command arriving mid-transfer is not starved.
    hrt_packets_per_service: int = 8
    #: Commands accepted while one is executing. Beyond this, ERR_BUSY.
    max_command_queue: int = 4
    #: Consecutive command failures before safe mode trips.
    safe_mode_threshold: int = 5
    #: Emit an idle HRT packet when the tap is open but nothing is queued.
    #: Off by default: the ICD does not require it and it wastes bus time.
    hrt_idle_fill: bool = False
    #: Data chunks per XOR parity chunk, on both the HRT and LRT paths.
    #: 6.25% overhead at the default of 16; 0 disables it.
    fec_group_size: int = DEFAULT_GROUP_SIZE
    #: Remembered `cmd_seq` values, for duplicate suppression.
    dedup_depth: int = 64
    scrub_interval_s: float = 30.0
    boot_count: int = 0
    #: How long a service pass waits for the first byte. Bounds the latency
    #: added to an LRT reply, and the idle wake-up rate.
    poll_timeout_s: float = 0.005


class Experiment:
    def __init__(self, link, wire: Wire = DEFAULT_WIRE,
                 dispatcher=None, store=None, state_provider=None,
                 config: ExperimentConfig | None = None,
                 events: L.EventLog | None = None, stream=None):
        self.link = link
        self.wire = wire
        self.dispatcher = dispatcher
        self.store = store
        #: Must be cheap - it is called on the LRT response path, where DICE is
        #: waiting. The daemon refreshes a cached dict on its own cadence and
        #: this just hands it over; it must never do I/O.
        self.state_provider = state_provider or (lambda: {})
        self.cfg = config or ExperimentConfig()
        self.events = events or L.EventLog()

        self.reader = PacketReader(wire)
        self.transfers = TransferManager(
            reload=(store.read if store is not None else None),
            group_size=self.cfg.fec_group_size)
        #: Live video. Constructed lazily on STREAM_START so that a payload
        #: which never streams pays nothing for the capability.
        self.stream = stream
        self._stream_frame: bytes | None = None
        self._stream_chunk = 0
        self._stream_chunks = 0
        self._stream_index = 0
        self._stream_keyframe = False

        # REQUEST_MEDIA and RESEND are served from `self.store`, while
        # DELETE_MEDIA, GET_MEDIA_LIST and GET_DOSE_LOG are served from the
        # dispatcher's. In the daemon they are the same object; if a caller
        # wires only one, adopt it for both rather than letting half the media
        # commands fail with a misleading BAD_PARAM.
        if (self.dispatcher is not None and store is not None
                and getattr(self.dispatcher, "store", None) is None):
            self.dispatcher.store = store

        # -- state that must survive a bit flip ---------------------------
        self._hrt_enabled = TMRBool(False, "hrt_enabled")
        self._safe_mode = TMRBool(False, "safe_mode")
        self._cmds_received = TMRInt(0, 4, "cmds_received")
        self._cmds_executed = TMRInt(0, 4, "cmds_executed")
        self._cmds_rejected = TMRInt(0, 4, "cmds_rejected")
        self._lrt_sent = TMRInt(0, 4, "lrt_sent")
        self._hrt_sent = TMRInt(0, 4, "hrt_sent")
        self.scrubber = Scrubber(self.cfg.scrub_interval_s)
        self.scrubber.register(self._hrt_enabled, self._safe_mode,
                               self._cmds_received, self._cmds_executed,
                               self._cmds_rejected, self._lrt_sent,
                               self._hrt_sent)

        # -- plain state --------------------------------------------------
        self.started = time.monotonic()
        self.hrt_last_control = 0
        self.coarse_time = 0
        self.fine_time = 0
        self.consecutive_failures = 0
        self.poll_timeout_s = self.cfg.poll_timeout_s

        self._result_lock = threading.Lock()
        self.last_opcode = 0
        self.last_cmd_seq = 0
        self.last_result = 0
        self.last_done_uptime = 0
        self.resp_opcode = 0
        self.resp_cmd_seq = 0
        self.resp_data = b""

        self._seen: OrderedDict[int, int] = OrderedDict()
        self._queue: queue.Queue = queue.Queue(maxsize=self.cfg.max_command_queue)
        self._worker: threading.Thread | None = None
        self._running = threading.Event()

    # ------------------------------------------------------------ lifecycle

    def start(self) -> None:
        self._running.set()
        self._worker = threading.Thread(target=self._work, name="stp-commands",
                                        daemon=True)
        self._worker.start()
        self.scrubber.start()
        self.events.add(L.EventCode.BOOT, arg=self.cfg.boot_count)
        log.info("STP experiment up as target 0x%02X", self.cfg.target_id)

    def stop(self) -> None:
        self._running.clear()
        self.scrubber.stop()
        if self._worker is not None:
            # Unblock the worker's get() so it can notice _running cleared.
            try:
                self._queue.put_nowait(None)
            except queue.Full:
                pass
            self._worker.join(timeout=3.0)
            self._worker = None

    @property
    def uptime_s(self) -> int:
        return int(time.monotonic() - self.started)

    # -------------------------------------------------------------- service

    def service(self) -> int:
        """One pass: drain the port, act on packets, pump HRT.

        Returns the number of packets handled. Never raises: an exception here
        would take down the only channel the mission has.
        """
        handled = 0
        try:
            # Prefer the blocking read when the transport offers one: it lets
            # the service thread sleep in the kernel between packets instead
            # of spinning, which matters on a power-minimised payload.
            if hasattr(self.link, "read_wait"):
                data = self.link.read_wait(self.poll_timeout_s)
            else:
                data = self.link.read()
            for packet in self.reader.feed(data):
                try:
                    self._on_packet(packet)
                    handled += 1
                except Exception as exc:               # noqa: BLE001
                    log.exception("handling packet failed: %s", exc)
            self._pump_hrt()
        except Exception as exc:                       # noqa: BLE001
            log.exception("service pass failed: %s", exc)
        return handled

    def _on_packet(self, packet) -> None:
        # Track DICE's clock whatever the packet was; it is the only time
        # reference aboard and every packet class except the ACK carries it.
        coarse = getattr(packet, "coarse_time", None)
        if coarse is not None:
            self.coarse_time = coarse
            self.fine_time = packet.fine_time

        if hasattr(packet, "payload"):                 # 120-byte Command
            self._on_command(packet)
            return

        ptype = packet.packet_type
        if ptype == PacketType.LRT_REQUEST:
            self._send_lrt()
        elif ptype in (PacketType.HRT_STOP, PacketType.HRT_STOP_WITH_LOSS,
                       PacketType.HRT_GO):
            self._on_hrt_control(ptype)

    # ------------------------------------------------------------- commands

    def _on_command(self, packet) -> None:
        """Acknowledge, then queue for execution.

        The ACK goes out for any command whose *envelope* was valid and
        addressed to us, even if the 105-byte payload turns out to be
        malformed. The ICD ties the acknowledgement to the packet, not to our
        private payload format, and a ground station that gets silence cannot
        tell a rejected command from a dead payload.
        """
        self._cmds_received.add(1)
        self._transmit(encode_command_ack(self.wire, self.cfg.target_id))

        try:
            request = decode_command_payload(packet.payload, self.wire.crc)
        except CommandDecodeError as exc:
            log.warning("malformed command payload: %s", exc)
            self._cmds_rejected.add(1)
            self._record_result(0, 0, int(Err.BAD_PARAM))
            self.events.add(L.EventCode.COMMAND_REJECTED,
                            severity=L.SEV_WARN)
            return

        if not request.force:
            with self._result_lock:
                previous = self._seen.get(request.cmd_seq, _UNSEEN)
            if previous is not _UNSEEN:
                # A repeat of a sequence number we have already accepted.
                # Treated as a retransmission: acknowledged above, but not
                # executed twice. `previous is None` means the first copy is
                # still running, which is reported as BUSY rather than as a
                # result we do not have yet.
                log.info("duplicate cmd_seq %d (opcode 0x%02X), not "
                         "re-executing", request.cmd_seq, request.opcode)
                self._record_result(
                    request.opcode, request.cmd_seq,
                    int(Err.BUSY) if previous is None else previous)
                self.events.add(L.EventCode.COMMAND_DUPLICATE,
                                arg=request.cmd_seq)
                return

        if self._safe_mode.value() and request.opcode not in (
                StpOp.CLEAR_SAFE_MODE, StpOp.GET_LINK_STATS,
                StpOp.ABORT_TRANSFERS, Msg.PING, Msg.GET_TELEMETRY,
                Msg.GET_CONFIG):
            log.warning("safe mode: refusing opcode 0x%02X", request.opcode)
            self._cmds_rejected.add(1)
            self._record_result(request.opcode, request.cmd_seq, int(Err.BUSY))
            self.events.add(L.EventCode.COMMAND_REJECTED,
                            arg=request.cmd_seq, severity=L.SEV_WARN)
            return

        try:
            self._queue.put_nowait(request)
            self._reserve_seq(request.cmd_seq)
            self.events.add(L.EventCode.COMMAND_ACCEPTED, arg=request.cmd_seq)
        except queue.Full:
            log.warning("command queue full, rejecting cmd_seq %d",
                        request.cmd_seq)
            self._cmds_rejected.add(1)
            self._record_result(request.opcode, request.cmd_seq, int(Err.BUSY))
            self.events.add(L.EventCode.COMMAND_REJECTED,
                            arg=request.cmd_seq, severity=L.SEV_WARN)

    def _work(self) -> None:
        while self._running.is_set():
            try:
                request = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if request is None:
                continue
            try:
                self._execute(request)
            except Exception as exc:                   # noqa: BLE001
                log.exception("command 0x%02X failed: %s",
                              request.opcode, exc)
                self._fail(request, Err.BAD_PARAM)

    def _execute(self, request) -> None:
        opcode = request.opcode

        if opcode == StpOp.CLEAR_SAFE_MODE:
            self._safe_mode.store(False)
            self.consecutive_failures = 0
            self.events.add(L.EventCode.SAFE_MODE_CLEARED)
            return self._succeed(request, b"")

        if opcode == StpOp.ABORT_TRANSFERS:
            n = self.transfers.clear()
            self.events.add(L.EventCode.TRANSFER_ABORTED, arg=n)
            return self._succeed(request, struct.pack("<I", n))

        if opcode == StpOp.GET_LINK_STATS:
            stats = self.reader.stats
            return self._succeed(request, struct.pack(
                ">8I", stats.good, stats.bad_crc, stats.bad_format,
                stats.not_for_us, stats.unknown_type, stats.resyncs,
                stats.dropped_bytes, self._hrt_sent.value()))

        if opcode == StpOp.SET_FEC_GROUP:
            if not request.args:
                return self._fail(request, Err.BAD_PARAM)
            group = request.args[0]
            self.cfg.fec_group_size = group
            self.transfers.group_size = group
            log.info("FEC parity group size set to %d", group)
            return self._succeed(request, bytes([group]))

        if opcode in (StpOp.STREAM_START, StpOp.STREAM_STOP,
                      StpOp.STREAM_SET_REGION, StpOp.STREAM_SET_OUTPUT):
            return self._stream_command(request)

        if opcode == StpOp.SET_HRT_IDLE_FILL:
            self.cfg.hrt_idle_fill = bool(request.args and request.args[0])
            return self._succeed(request, bytes([1 if self.cfg.hrt_idle_fill
                                                 else 0]))

        # Media transfer is intercepted: the dispatcher would return the whole
        # file as protocol frames, but on this link bulk data belongs to HRT
        # and only HRT.
        if opcode == Msg.REQUEST_MEDIA:
            return self._request_media(request)
        if opcode == Msg.RESEND:
            return self._resend(request)

        if self.dispatcher is None:
            return self._fail(request, Err.BAD_TYPE)

        responses = self.dispatcher.handle(
            Frame(opcode, request.cmd_seq, request.args))
        if not responses:
            return self._fail(request, Err.BAD_TYPE)

        head = responses[0]
        if head.type == Msg.NACK:
            code = head.payload[2] if len(head.payload) >= 3 else int(Err.BAD_PARAM)
            return self._fail(request, code)

        if opcode == Msg.DELETE_MEDIA and len(request.args) >= 4:
            # Keep the HRT queue consistent with storage.
            self.transfers.abort(struct.unpack("<I", request.args[:4])[0])

        return self._succeed(request, head.payload, head.type)

    # -- media ------------------------------------------------------------

    def _request_media(self, request) -> None:
        if self.store is None or len(request.args) < 4:
            return self._fail(request, Err.BAD_PARAM)
        media_id = struct.unpack("<I", request.args[:4])[0]

        data = self.store.read(media_id)
        if data is None:
            return self._fail(request, Err.NO_MEDIA)

        meta = self._media_meta(media_id)
        if not self.transfers.enqueue(media_id, data, **meta):
            return self._fail(request, Err.BUSY)

        self.events.add(L.EventCode.TRANSFER_STARTED, arg=media_id)
        chunks = (len(data) + 1255) // 1256
        return self._succeed(request, struct.pack(
            ">IQII", media_id, len(data), chunks,
            zlib.crc32(data) & 0xFFFFFFFF))

    def _media_meta(self, media_id: int) -> dict:
        try:
            for record in (self.store.list() or []):
                if record.media_id == media_id:
                    return {"kind": 0 if record.kind == "image" else 1,
                            "width": record.width, "height": record.height,
                            "created_unix": record.created_unix}
        except Exception as exc:                       # noqa: BLE001
            log.debug("media metadata lookup failed: %s", exc)
        return {}

    def _resend(self, request) -> None:
        if len(request.args) < 4:
            return self._fail(request, Err.BAD_PARAM)
        media_id = struct.unpack("<I", request.args[:4])[0]
        rest = request.args[4:]
        chunks = [struct.unpack("<I", rest[i:i + 4])[0]
                  for i in range(0, len(rest) - 3, 4)]
        if not chunks:
            return self._fail(request, Err.BAD_PARAM)

        accepted = self.transfers.request_resend(media_id, chunks)
        if not accepted:
            return self._fail(request, Err.NO_MEDIA)
        return self._succeed(request, struct.pack("<II", media_id, accepted))

    # -- live stream ------------------------------------------------------

    def _ensure_stream(self):
        """Build the stream object on first use, not at construction.

        Importing and constructing it eagerly would make every payload pay for
        a capability most runs never use, and would make `radcam.stp` depend on
        the camera stack even in tests that have no camera.
        """
        if self.stream is None:
            from ..stream import VideoStream
            self.stream = VideoStream()
        return self.stream

    def _stream_command(self, request) -> None:
        from ..stream import StreamConfig

        try:
            stream = self._ensure_stream()
        except Exception as exc:                       # noqa: BLE001
            log.error("stream unavailable: %s", exc)
            return self._fail(request, Err.CAMERA_FAULT)

        opcode = request.opcode
        current = stream.config

        if opcode == StpOp.STREAM_STOP:
            stopped = stream.stop()
            self._drop_stream_frame()
            self.events.add(L.EventCode.RECORD_STOPPED)
            return self._succeed(request, bytes([1 if stopped else 0]))

        if opcode == StpOp.STREAM_SET_REGION:
            if len(request.args) < 8:
                return self._fail(request, Err.BAD_PARAM)
            cx, cy, cw, ch = struct.unpack("<4H", request.args[:8])
            wanted = replace(current, centre_x=cx, centre_y=cy,
                             crop_w=cw, crop_h=ch)
        elif opcode == StpOp.STREAM_SET_OUTPUT:
            if len(request.args) < 9:
                return self._fail(request, Err.BAD_PARAM)
            width, height, fps, bitrate = struct.unpack("<HHBI", request.args[:9])
            wanted = replace(current, width=width, height=height,
                             fps=fps, bitrate=bitrate)
        else:                                          # STREAM_START
            wanted = current
            if len(request.args) >= 9:
                width, height, fps, bitrate = struct.unpack(
                    "<HHBI", request.args[:9])
                wanted = replace(wanted, width=width, height=height,
                                 fps=fps, bitrate=bitrate)
            if len(request.args) >= 17:
                cx, cy, cw, ch = struct.unpack("<4H", request.args[9:17])
                wanted = replace(wanted, centre_x=cx, centre_y=cy,
                                 crop_w=cw, crop_h=ch)

        effective = wanted.sanitised()
        was_running = stream.running

        # A settings change restarts the encoder, because resolution, frame
        # rate and region are all fixed at process start. Restarting only when
        # something actually changed keeps a redundant command from putting a
        # gap in a running stream.
        if opcode == StpOp.STREAM_START or was_running:
            if opcode == StpOp.STREAM_START or effective != current:
                self._drop_stream_frame()
                if not stream.start(effective):
                    self.events.add(L.EventCode.CAPTURE_FAILED,
                                    severity=L.SEV_ERROR)
                    return self._fail(request, Err.CAMERA_FAULT)
                self.events.add(L.EventCode.RECORD_STARTED,
                                arg=effective.width)
        else:
            stream.config = effective

        applied = stream.config
        # Report what was actually applied, not what was asked for: the values
        # are clamped to the sensor and to sane encoder limits, and the ground
        # should never have to guess which.
        return self._succeed(request, struct.pack(
            "<HHBI4H", applied.width, applied.height, applied.fps,
            applied.bitrate, applied.centre_x, applied.centre_y,
            applied.crop_w, applied.crop_h))

    def _drop_stream_frame(self) -> None:
        self._stream_frame = None
        self._stream_chunk = 0
        self._stream_chunks = 0

    def _stream_state(self) -> dict:
        if self.stream is None:
            return {"stream_state": L.STREAM_OFF}

        stream = self.stream
        state = dict(stream.status())
        if stream.fault:
            state["stream_state"] = L.STREAM_FAULT
        elif stream.running:
            state["stream_state"] = (L.STREAM_RUNNING if stream.frames_encoded
                                     else L.STREAM_STARTING)
        else:
            state["stream_state"] = L.STREAM_OFF

        flags = 0
        if stream.running and not self._hrt_enabled.value():
            flags |= L.STREAM_FLAG_GATED
        if stream.encoder_late():
            flags |= L.STREAM_FLAG_ENCODER_LATE
        state["stream_flags"] = flags
        return state

    def _next_stream_payload(self) -> bytes | None:
        """One HRT payload of live video, or None if no frame is ready."""
        stream = self.stream
        if stream is None or not stream.running:
            return None

        if self._stream_frame is None:
            frame = stream.take()
            if frame is None:
                return None
            self._stream_frame = frame.data
            self._stream_index = frame.index
            self._stream_keyframe = frame.keyframe
            self._stream_chunk = 0
            self._stream_chunks = max(
                1, (len(frame.data) + HRT_CHUNK_DATA - 1) // HRT_CHUNK_DATA)

        index = self._stream_chunk
        start = index * HRT_CHUNK_DATA
        piece = self._stream_frame[start:start + HRT_CHUNK_DATA]
        self._stream_chunk += 1

        # Read these before the frame state is cleared below: resetting first
        # published chunk_total=0 on the final chunk of every frame, which is
        # the one chunk a reassembler most needs it from.
        frame_index, chunk_total = self._stream_index, self._stream_chunks

        flags = FLAG_KEYFRAME if self._stream_keyframe else 0
        if self._stream_chunk >= chunk_total:
            flags |= FLAG_LAST_CHUNK
            self._drop_stream_frame()

        return build_hrt_payload(SubType.STREAM_DATA, frame_index,
                                 index, chunk_total, piece, flags)

    # -- results ----------------------------------------------------------

    def _succeed(self, request, payload: bytes, resp_opcode: int | None = None) -> None:
        self._cmds_executed.add(1)
        self.consecutive_failures = 0
        self._record_result(request.opcode, request.cmd_seq, 0,
                            payload, resp_opcode)
        if len(payload) > L.RESP_DATA_MAX:
            # Too big for the LRT response window, so it goes over HRT under a
            # synthetic id the ground can recognise by its high byte.
            self.transfers.enqueue(_RESPONSE_ID_BASE | (request.cmd_seq & 0xFFFF),
                                   payload)

    def _fail(self, request, code) -> None:
        self._cmds_rejected.add(1)
        self.consecutive_failures += 1
        self._record_result(request.opcode, request.cmd_seq, int(code))
        self.events.add(L.EventCode.COMMAND_FAILED, arg=request.cmd_seq,
                        severity=L.SEV_ERROR)

        if (self.consecutive_failures >= self.cfg.safe_mode_threshold
                and not self._safe_mode.value()):
            self._safe_mode.store(True)
            self._hrt_enabled.store(False)
            self.events.add(L.EventCode.SAFE_MODE_ENTERED,
                            arg=self.consecutive_failures,
                            severity=L.SEV_ERROR)
            log.error("safe mode: %d consecutive command failures",
                      self.consecutive_failures)

    def _reserve_seq(self, cmd_seq: int) -> None:
        """Claim a sequence number the moment the command is queued.

        Recording it only after execution left a window - a retransmission
        arriving while the first copy was still running would be executed
        again, which for CAPTURE_IMAGE means a duplicate file and for
        DELETE_MEDIA means deleting whatever took the id next.
        """
        with self._result_lock:
            self._seen[cmd_seq] = None
            self._trim_seen()

    def _trim_seen(self) -> None:
        while len(self._seen) > self.cfg.dedup_depth:
            self._seen.popitem(last=False)

    def _record_result(self, opcode: int, cmd_seq: int, result: int,
                       payload: bytes = b"", resp_opcode: int | None = None) -> None:
        with self._result_lock:
            self.last_opcode = opcode
            self.last_cmd_seq = cmd_seq
            self.last_result = result
            self.last_done_uptime = self.uptime_s
            if payload or resp_opcode is not None:
                self.resp_opcode = resp_opcode if resp_opcode is not None else opcode
                self.resp_cmd_seq = cmd_seq
                self.resp_data = payload

            self._seen[cmd_seq] = result
            self._trim_seen()

    # ------------------------------------------------------------------ LRT

    def _send_lrt(self) -> None:
        payload = L.build_lrt_payload(self.build_state(),
                                      self.events.recent(L.MAX_EVENTS))
        if self._transmit(encode_lrt_data(payload, self.wire,
                                          self.cfg.target_id)):
            self._lrt_sent.add(1)

    def build_state(self) -> dict:
        """Merge the daemon's housekeeping with everything owned here."""
        state = {}
        try:
            state.update(self.state_provider() or {})
        except Exception as exc:                       # noqa: BLE001
            log.error("state provider failed: %s", exc)

        with self._result_lock:
            state.update({
                "last_opcode": self.last_opcode,
                "last_cmd_seq": self.last_cmd_seq,
                "last_result": self.last_result,
                "last_done_uptime": self.last_done_uptime,
                "resp_opcode": self.resp_opcode,
                "resp_cmd_seq": self.resp_cmd_seq,
                "resp_data": self.resp_data,
            })

        stats = self.reader.stats
        state.update({
            "uptime_s": self.uptime_s,
            "boot_count": self.cfg.boot_count,
            "target_id": self.cfg.target_id,
            "coarse_time": self.coarse_time,
            "fine_time": self.fine_time,
            "cmds_received": self._cmds_received.value(),
            "cmds_executed": self._cmds_executed.value(),
            "cmds_rejected": self._cmds_rejected.value(),
            "rx_good": stats.good, "rx_bad_crc": stats.bad_crc,
            "rx_bad_format": stats.bad_format,
            "rx_not_for_us": stats.not_for_us,
            "rx_unknown_type": stats.unknown_type,
            "rx_resyncs": stats.resyncs,
            "rx_dropped_bytes": stats.dropped_bytes,
            "tx_packets": getattr(self.link, "tx_packets", 0),
            "tx_errors": getattr(self.link, "tx_errors", 0),
            "lrt_sent": self._lrt_sent.value(),
            "hrt_sent": self._hrt_sent.value(),
            "hrt_enabled": self._hrt_enabled.value(),
            "hrt_last_control": self.hrt_last_control,
            "safe_mode": self._safe_mode.value(),
            "tmr_corrections": self.scrubber.repairs,
            "xfer_queue_depth": self.transfers.pending,
        })
        state.update(self.transfers.status())
        state["fec_group_size"] = self.cfg.fec_group_size
        state.update(self._stream_state())
        if self._safe_mode.value():
            state["xfer_state"] = L.XFER_PAUSED
        elif not self._hrt_enabled.value() and self.transfers.pending:
            state["xfer_state"] = L.XFER_PAUSED
        return state

    # ------------------------------------------------------------------ HRT

    def _on_hrt_control(self, ptype: int) -> None:
        self.hrt_last_control = ptype

        if ptype == PacketType.HRT_GO:
            if self._safe_mode.value():
                log.warning("HRT Go refused: safe mode")
                return
            self._hrt_enabled.store(True)
            self.events.add(L.EventCode.HRT_GO)
            log.info("HRT enabled by DICE")
            return

        self._hrt_enabled.store(False)
        if ptype == PacketType.HRT_STOP_WITH_LOSS:
            self.transfers.on_stop_with_loss()
            self.events.add(L.EventCode.HRT_STOP_WITH_LOSS,
                            severity=L.SEV_WARN)
        else:
            self.events.add(L.EventCode.HRT_STOP)
        log.info("HRT disabled by DICE (0x%02X)", ptype)

    def _pump_hrt(self) -> None:
        if not self._hrt_enabled.value() or self._safe_mode.value():
            # The tap is shut. Anything the encoder has already produced is
            # only going to get staler, so throw it away rather than send
            # history when the tap reopens.
            if self.stream is not None and self.stream.running:
                self._drop_stream_frame()
                self.stream.flush()
            return

        for _ in range(self.cfg.hrt_packets_per_service):
            # Live video first. It is the only traffic on this link whose value
            # expires, and a file transfer waiting a few seconds loses nothing.
            # When no frame is ready the encoder is between frames, and that
            # gap is exactly when a file chunk should use the link.
            payload = self._next_stream_payload()
            if payload is None:
                payload = self.transfers.next_payload()
            if payload is None:
                if self.cfg.hrt_idle_fill:
                    payload = b"\x00" * 1280
                else:
                    return
            if not self._transmit(encode_hrt_data(payload, self.wire,
                                                  self.cfg.target_id)):
                return
            self._hrt_sent.add(1)
            # DICE can revoke the tap between packets; re-check every time
            # rather than committing to a whole burst.
            if not self._hrt_enabled.value():
                return

    # ------------------------------------------------------------ transmit

    def _transmit(self, packet: bytes) -> bool:
        try:
            ok = self.link.send(packet)
        except Exception as exc:                       # noqa: BLE001
            log.error("transmit failed: %s", exc)
            ok = False
        if not ok:
            self.events.add(L.EventCode.TX_FAILED, severity=L.SEV_ERROR)
        return ok
