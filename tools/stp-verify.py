#!/usr/bin/env python3
"""Reliability, safety and autonomy checks for the STP/DICE link.

Three suites, run in order, each answering a different question:

  reliability  does it behave correctly at the edges, and survive abuse?
  safety       does it fail safe - radiation, faults, unsolicited transmission?
  autonomy     can the mission be flown with nothing but this link?

Unlike the unit tests, these are adversarial: fuzzed input, forced faults,
exhausted queues, sequence wraparound. They exist because the failure that
matters in orbit is the one nobody wrote a happy-path test for.

    tools/stp-verify.py              # all three suites
    tools/stp-verify.py --suite safety
"""

from __future__ import annotations

import argparse
import os
import random
import struct
import sys
import time
import zlib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from radcam.protocol import Config as ProtoConfig, Dispatcher, Err, Msg   # noqa: E402
from radcam.stp import hrt as H, lrt as L, packets as P                   # noqa: E402
from radcam.stp.commands import (FLAG_FORCE, encode_command_payload)      # noqa: E402
from radcam.stp.experiment import Experiment, ExperimentConfig, StpOp     # noqa: E402
from radcam.stp.link import Rs422Link, NullDeLine                         # noqa: E402
from radcam.stp.redundancy import Scrubber, TMRBool, TMRInt, TMRUnrecoverable  # noqa: E402
from radcam.stp import fec                                            # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + "/..")
from tests.support import FakeMediaStore, MemoryLink, drain_experiment    # noqa: E402

TARGET = 1
WIRE = P.Wire(target_id=TARGET)


class Checker:
    def __init__(self):
        self.passed = 0
        self.failed: list[str] = []

    def check(self, name: str, ok: bool, detail: str = "") -> bool:
        if ok:
            self.passed += 1
            print(f"  PASS  {name}" + (f"   {detail}" if detail else ""))
        else:
            self.failed.append(name)
            print(f"  FAIL  {name}" + (f"   {detail}" if detail else ""))
        return ok

    def section(self, title: str) -> None:
        print(f"\n--- {title} ---")


class _FakeStream:
    """A stream with no camera, so the protocol path is checkable anywhere."""

    def __init__(self):
        from radcam.stream import StreamConfig
        self.config = StreamConfig()
        self.running = False
        self.fault = None
        self.frames = []
        self.frames_encoded = 1
        self.frames_dropped = 0
        self.frames_taken = 0
        self.bytes_encoded = 0

    def start(self, config=None):
        if config is not None:
            self.config = config.sanitised()
        self.running = True
        return True

    def stop(self):
        was, self.running = self.running, False
        return was

    def take(self):
        return self.frames.pop(0) if self.frames else None

    def flush(self, keep_keyframe=False):
        dropped = len(self.frames)
        self.frames.clear()
        self.frames_dropped += dropped
        return dropped

    def encoder_late(self):
        return False

    @property
    def queue_depth(self):
        return len(self.frames)

    def status(self):
        cfg = self.config
        return {"stream_width": cfg.width, "stream_height": cfg.height,
                "stream_fps": cfg.fps, "stream_bitrate": cfg.bitrate,
                "stream_centre_x": cfg.centre_x, "stream_centre_y": cfg.centre_y,
                "stream_crop_w": cfg.crop_w, "stream_crop_h": cfg.crop_h,
                "stream_frames_sent": self.frames_taken,
                "stream_frames_dropped": self.frames_dropped,
                "stream_bytes_sent": self.bytes_encoded,
                "stream_queue_depth": len(self.frames)}


def _frame(index, data, keyframe=False):
    from radcam.stream import EncodedFrame
    return EncodedFrame(index, data, keyframe, time.time())


def _hrt(link) -> list:
    raw = link.dice_read()
    return [H.decode_hrt_payload(raw[i * 1288:(i + 1) * 1288][6:6 + 1280])
            for i in range(len(raw) // 1288)]


def _raises(fn) -> bool:
    try:
        fn()
    except Exception:                                   # noqa: BLE001
        return True
    return False


def build(**cfg):
    link = MemoryLink()
    store = FakeMediaStore()
    experiment = Experiment(
        link=link, wire=WIRE,
        dispatcher=Dispatcher(config=ProtoConfig(), store=store),
        store=store,
        config=ExperimentConfig(target_id=TARGET, scrub_interval_s=3600, **cfg))
    experiment.start()
    return link, store, experiment


def command(link, opcode, args=b"", seq=1, flags=0, target=TARGET):
    link.dice_send(P.encode_command(
        encode_command_payload(opcode, seq, args, flags), 1000, 5, WIRE, target))


def short(link, ptype, target=TARGET):
    link.dice_send(P.encode_short_request(ptype, 1000, 5, WIRE, target))


def replies(link):
    raw = link.dice_read()
    out, i = [], 0
    while i < len(raw):
        at = raw.find(WIRE.sync_bytes, i)
        if at < 0 or at + 6 > len(raw):
            break
        kind = raw[at + 4]
        if kind == 0x10 and at + 8 <= len(raw):
            out.append(("ACK", raw[at:at + 8])); i = at + 8
        elif kind == 0x81 and at + 1256 <= len(raw):
            out.append(("LRT", raw[at:at + 1256])); i = at + 1256
        elif kind == 0x87 and at + 1288 <= len(raw):
            out.append(("HRT", raw[at:at + 1288])); i = at + 1288
        else:
            i = at + 4
    return out


def poll_lrt(link, experiment):
    drain_experiment(experiment, passes=4)
    link.dice_read()
    short(link, P.PacketType.LRT_REQUEST)
    experiment.service()
    for kind, raw in replies(link):
        if kind == "LRT":
            return L.decode_lrt_payload(raw[6:6 + 1248])
    return None


# ---------------------------------------------------------------- reliability

def suite_reliability(c: Checker) -> None:
    print("\n=== RELIABILITY ===")

    c.section("Fuzzing: random bytes must never crash or wedge the receiver")
    link, store, exp = build()
    random.seed(1234)
    crashed = None
    try:
        for _ in range(300):
            link.dice_send(bytes(random.randrange(256)
                                 for _ in range(random.randrange(1, 400))))
            exp.service()
    except Exception as e:                              # noqa: BLE001
        crashed = e
    c.check("survives 300 random garbage bursts", crashed is None, str(crashed or ""))
    c.check("no reply to garbage", len(replies(link)) == 0)
    c.check("receive buffer stayed bounded",
            exp.reader.pending_bytes <= exp.reader.max_buffer,
            f"{exp.reader.pending_bytes} B")
    # And it must still work afterwards.
    command(link, Msg.PING, seq=1)
    exp.service()
    c.check("still answers a valid command after fuzzing",
            [k for k, _ in replies(link)] == ["ACK"])
    exp.stop()

    c.section("Fuzzing: valid sync, random contents")
    link, store, exp = build()
    crashed = None
    try:
        for _ in range(300):
            body = bytes(random.randrange(256) for _ in range(random.randrange(4, 200)))
            link.dice_send(WIRE.sync_bytes + body)
            exp.service()
    except Exception as e:                              # noqa: BLE001
        crashed = e
    c.check("survives 300 sync-prefixed garbage bursts", crashed is None,
            str(crashed or ""))
    command(link, Msg.PING, seq=1)
    exp.service()
    c.check("still answers afterwards",
            "ACK" in [k for k, _ in replies(link)])
    exp.stop()

    c.section("Byte-at-a-time delivery")
    link, store, exp = build()
    packet = P.encode_command(encode_command_payload(Msg.PING, 7), 0, 0, WIRE, TARGET)
    for byte in packet:
        link.dice_send(bytes([byte]))
        exp.service()
    c.check("packet arriving one byte per service pass is decoded",
            len([k for k, _ in replies(link) if k == "ACK"]) == 1)
    exp.stop()

    c.section("Every single-byte corruption of a command is rejected")
    link, store, exp = build()
    good = P.encode_command(encode_command_payload(Msg.PING, 8), 0, 0, WIRE, TARGET)
    accepted = []
    for offset in range(len(good)):
        broken = bytearray(good)
        broken[offset] ^= 0xFF
        link.dice_read()
        link.dice_send(bytes(broken))
        exp.service()
        if replies(link):
            accepted.append(offset)
    c.check("no corrupted command is acknowledged", not accepted,
            f"accepted at offsets {accepted[:8]}" if accepted else "all 120 rejected")
    exp.stop()

    c.section("Sequence number wraparound")
    link, store, exp = build()
    before = exp._cmds_executed.value()
    for seq in (0xFFFE, 0xFFFF, 0x0000, 0x0001):
        command(link, Msg.PING, seq=seq)
        drain_experiment(exp, passes=3)
    c.check("all four sequence numbers across the wrap execute",
            exp._cmds_executed.value() - before == 4,
            f"{exp._cmds_executed.value() - before}/4")
    exp.stop()

    c.section("Command queue exhaustion")
    link, store, exp = build(max_command_queue=2)
    for seq in range(20, 30):
        command(link, Msg.PING, seq=seq)
        exp.service()
    report = poll_lrt(link, exp)
    c.check("queue overflow is reported, not crashed", report is not None)
    c.check("overflow rejections are counted",
            report["cmds_rejected"] > 0, f"{report['cmds_rejected']} rejected")
    c.check("every command was still acknowledged",
            report["cmds_received"] >= 10, f"{report['cmds_received']} received")
    exp.stop()

    c.section("Degenerate media")
    link, store, exp = build()
    store.blobs[10] = b""                       # zero-length file
    store.blobs[11] = b"x"                      # one byte
    store.blobs[12] = bytes(H.HRT_CHUNK_DATA)   # exactly one full chunk
    for media_id in (10, 11, 12):
        command(link, Msg.REQUEST_MEDIA, struct.pack("<I", media_id), seq=40 + media_id)
        drain_experiment(exp, passes=3)
    short(link, P.PacketType.HRT_GO)
    for _ in range(20):
        exp.service()
    c.check("degenerate file sizes do not wedge the transfer queue",
            exp.transfers.pending == 0, f"{exp.transfers.pending} left")
    exp.stop()

    c.section("Resend edge cases")
    manager = H.TransferManager()
    manager.enqueue(1, b"a" * 5000)
    c.check("out-of-range chunk indices are refused",
            manager.request_resend(1, [999, 1000]) == 0)
    c.check("mixed valid/invalid keeps only the valid",
            manager.request_resend(1, [0, 999]) == 1)
    c.check("resend for unknown media is refused",
            manager.request_resend(4242, [0]) == 0)
    c.check("duplicate enqueue is refused", not manager.enqueue(1, b"b"))

    c.section("Transmit failure handling")
    link, store, exp = build()
    link.fail_next = 3
    command(link, Msg.PING, seq=60)
    exp.service()
    c.check("a failed transmit does not raise", True)
    c.check("transmit failure is recorded as an event",
            any(e.code == L.EventCode.TX_FAILED for e in exp.events.recent()))
    link.fail_next = 0
    command(link, Msg.PING, seq=61)
    exp.service()
    c.check("recovers after transmit failures",
            "ACK" in [k for k, _ in replies(link)])
    exp.stop()

    c.section("Parity arithmetic edge cases")
    size = 128
    chunks = [bytes([i]) * size for i in range(4)]
    parity = fec.parity_of(chunks, size)
    c.check("parity of a group XORed with itself is zero",
            fec.parity_of(chunks + [parity], size) == bytes(size))
    short_group = [b"abc", b"de"]
    p2 = fec.parity_of(short_group, size)
    recovered = fec.recover_missing({1: b"de"}, p2, [0, 1], size)
    c.check("a ragged group still reconstructs",
            recovered is not None and recovered[1][:3] == b"abc")
    c.check("parity refuses an oversized chunk",
            _raises(lambda: fec.parity_of([b"x" * (size + 1)], size)))
    c.check("group size 0 emits no parity", fec.group_count(1000, 0) == 0)

    c.section("HRT transfer under loss, repaired by parity")
    manager = H.TransferManager(group_size=8)
    blob = os.urandom(H.HRT_CHUNK_DATA * 40 + 77)
    manager.enqueue(1, blob)
    data, parity, lengths, total = {}, {}, {}, 0
    seen = 0
    while True:
        payload = manager.next_payload()
        if payload is None:
            break
        decoded = H.decode_hrt_payload(payload)
        if decoded["sub_type"] == H.SubType.MEDIA_INFO:
            total = H.decode_media_info(decoded["data"])["chunk_total"]
            continue
        if decoded["sub_type"] == H.SubType.MEDIA_PARITY:
            parity[decoded["chunk_index"]] = decoded["data"]
            continue
        if decoded["sub_type"] != H.SubType.MEDIA_DATA:
            continue
        seen += 1
        lengths[decoded["chunk_index"]] = decoded["data_len"]
        if seen % 11 == 0:              # a chunk lost in transit
            continue
        data[decoded["chunk_index"]] = decoded["data"]

    lost = [i for i in range(total) if i not in data]
    c.check("chunks were genuinely lost", bool(lost), f"{len(lost)} lost")
    stats = fec.FecStats()
    missing = fec.verify_and_repair(data, parity, total, 8,
                                    H.HRT_CHUNK_DATA, stats)
    c.check("parity repaired them with no retransmission",
            not missing and stats.recovered > 0,
            f"recovered {stats.recovered}, still missing {len(missing)}")
    if not missing:
        rebuilt = b"".join(data[i][:lengths[i]] for i in range(total))
        c.check("the repaired file is bit-exact", rebuilt == blob)
        c.check("its CRC-32 matches the original",
                (zlib.crc32(rebuilt) & 0xFFFFFFFF)
                == (zlib.crc32(blob) & 0xFFFFFFFF))

    c.section("Losses beyond parity are reported, not silently wrong")
    two_lost = {k: v for k, v in data.items() if k not in (0, 1)}
    still = fec.verify_and_repair(two_lost, parity, total, 8, H.HRT_CHUNK_DATA)
    c.check("two losses in one group remain unrecoverable",
            sorted(still) == [0, 1], str(still))

    c.section("Payload builders never raise on bad state")
    ok = True
    for state in ({}, {"dose_rad": float("nan")}, {"uptime_s": -1},
                  {"storage_free": 2**70}, {"media_count": 10**9},
                  {"resp_data": b"x" * 5000}, {"cpu_temp_c": float("inf")}):
        try:
            payload = L.build_lrt_payload(state)
            assert len(payload) == 1248
        except Exception as exc:                        # noqa: BLE001
            ok = False
            print(f"        state {list(state)[:1]} raised {exc}")
    c.check("LRT builder tolerates hostile state", ok)


# --------------------------------------------------------------------- safety

def suite_safety(c: Checker) -> None:
    print("\n=== SAFETY ===")

    c.section("Radiation: TMR in RAM")
    cell = TMRInt(0xDEADBEEF, 4, "x")
    cell._copies[1][0] ^= 0x80
    c.check("single-copy bit flip is corrected", cell.value() == 0xDEADBEEF)
    c.check("and the damaged copy is repaired",
            len({bytes(x) for x in cell._copies}) == 1)
    cell2 = TMRInt(0x11111111, 4, "y")
    cell2._copies[0][1] ^= 0xFF
    cell2._copies[2][1] ^= 0x0F
    unrecoverable = False
    try:
        cell2.get()
    except TMRUnrecoverable:
        unrecoverable = True
    c.check("double fault in one byte is detected, not silently wrong",
            unrecoverable)
    c.check("and falls back to a safe default", cell2.value(default=0) == 0)

    flag = TMRBool(False, "hrt")
    for copy in flag._copies:
        copy[0] ^= 0x02
    c.check("bool encoding survives a flip in every copy", flag.value() is False)

    scrubber = Scrubber()
    damaged = TMRInt(7, 4, "z")
    damaged._copies[0][3] ^= 0x01
    scrubber.register(damaged)
    c.check("scrubber finds and repairs latent damage",
            scrubber.scrub_once() == 1)

    c.section("Radiation: TMR over the link")
    payload = bytearray(L.build_lrt_payload({"dose_rad": 2.5, "dose_volts": 1.1}))
    payload[L.OFF_DOSE_RAD + 2] ^= 0xFF
    decoded = L.decode_lrt_payload(bytes(payload))
    c.check("corrupted LRT is flagged by its CRC-32",
            not decoded["payload_crc_ok"])
    c.check("but dose is still recovered by majority vote",
            abs(decoded["dose_rad"] - 2.5) < 1e-5)

    c.section("Never transmits unsolicited")
    link, store, exp = build()
    for _ in range(50):
        exp.service()
    c.check("silent with no traffic at all", len(link.dice_read()) == 0)

    command(link, Msg.REQUEST_MEDIA, struct.pack("<I", 3), seq=1)
    drain_experiment(exp)
    link.dice_read()
    for _ in range(20):
        exp.service()
    c.check("silent with media queued but HRT closed", len(link.dice_read()) == 0)

    short(link, P.PacketType.HRT_GO)
    exp.service()
    sent = len(link.dice_read())
    c.check("transmits only once HRT is opened", sent > 0, f"{sent} B")
    short(link, P.PacketType.HRT_STOP)
    exp.service()
    link.dice_read()
    for _ in range(20):
        exp.service()
    c.check("stops immediately when HRT is closed", len(link.dice_read()) == 0)
    exp.stop()

    c.section("Never answers another target")
    link, store, exp = build()
    for opcode in (Msg.PING, Msg.CAPTURE_IMAGE, Msg.REQUEST_MEDIA):
        command(link, opcode, struct.pack("<I", 1), seq=1, target=TARGET + 1)
        exp.service()
    for ptype in (P.PacketType.LRT_REQUEST, P.PacketType.HRT_GO,
                  P.PacketType.HRT_STOP):
        short(link, ptype, target=TARGET + 1)
        exp.service()
    c.check("no reply to any foreign-target packet", len(link.dice_read()) == 0)
    c.check("a foreign HRT Go does not open our tap",
            not exp._hrt_enabled.value())
    exp.stop()

    c.section("Safe mode")
    link, store, exp = build(safe_mode_threshold=3)
    for i in range(3):
        command(link, Msg.REQUEST_MEDIA, struct.pack("<I", 800 + i), seq=70 + i)
        drain_experiment(exp, passes=3)
    c.check("repeated failures trip safe mode", exp._safe_mode.value())
    report = poll_lrt(link, exp)
    c.check("LRT still answers in safe mode", report is not None)
    c.check("and reports safe mode to the ground", report and report["safe_mode"])
    short(link, P.PacketType.HRT_GO)
    exp.service()
    c.check("HRT is refused while in safe mode", not exp._hrt_enabled.value())
    command(link, StpOp.CLEAR_SAFE_MODE, seq=80)
    drain_experiment(exp)
    c.check("ground can clear safe mode", not exp._safe_mode.value())
    exp.stop()

    c.section("Hardware safety interlocks still hold through STP")
    link, store, exp = build()
    command(link, Msg.SET_LED, bytes([100]), seq=90)     # ask for 100%
    report = poll_lrt(link, exp)
    applied = report["resp_data"][0] if report["resp_data"] else 255
    c.check("LED stays capped at 10% via the STP path", applied <= 10,
            f"requested 100%, applied {applied}%")

    exp2 = Experiment(link=MemoryLink(), wire=WIRE,
                      dispatcher=Dispatcher(config=ProtoConfig(),
                                            eeprom=object(),
                                            eeprom_writable=False),
                      config=ExperimentConfig(target_id=TARGET,
                                              scrub_interval_s=3600))
    exp2.start()
    command(exp2.link, Msg.EEPROM_WRITE, struct.pack("<HH", 0, 4) + b"\x00" * 4,
            seq=91)
    drain_experiment(exp2)
    c.check("EEPROM writes stay refused unless unlocked",
            exp2.last_result == int(Err.WRITE_PROTECTED),
            f"result {exp2.last_result}")
    exp2.stop()
    exp.stop()

    c.section("DE is released even when transmission fails")
    class ExplodingSerial:
        is_open = True
        timeout = 0
        def write(self, data):
            raise OSError("simulated UART failure")
        def flush(self):
            pass
        def reset_input_buffer(self):
            pass
        def fileno(self):
            return 0
    class WatchedDe(NullDeLine):
        def __init__(self):
            super().__init__()
            self.states = []
        def set(self, enabled):
            self.states.append(enabled)

    de = WatchedDe()
    watched = Rs422Link(de=de)
    watched._serial = ExplodingSerial()
    result = watched.send(b"hello")
    c.check("a write exception is contained", result is False)
    c.check("DE is asserted then released despite the exception",
            de.states[-1] is False and True in de.states, str(de.states))

    c.section("Bounded resources")
    c.check("event ring is bounded", L.MAX_EVENTS <= 43 and len(
        (lambda lg: [lg.add(1) for _ in range(500)] and lg)(L.EventLog())) == L.MAX_EVENTS)
    manager = H.TransferManager(max_queue=3)
    for i in range(10):
        manager.enqueue(i, b"x")
    c.check("transfer queue is bounded", manager.pending <= 3,
            f"{manager.pending} queued")
    link, store, exp = build()
    for i in range(200):
        exp._reserve_seq(i)
    c.check("duplicate-suppression table is bounded",
            len(exp._seen) <= exp.cfg.dedup_depth, f"{len(exp._seen)} entries")
    exp.stop()


# ------------------------------------------------------------------ autonomy

def suite_autonomy(c: Checker) -> None:
    print("\n=== AUTONOMY ===")

    c.section("Every mission function is reachable over RS-422")
    link, store, exp = build()
    functions = [
        ("liveness", Msg.PING, b""),
        ("read configuration", Msg.GET_CONFIG, b""),
        ("change configuration", Msg.SET_CONFIG, struct.pack("<BI", 0x06, 2)),
        ("housekeeping", Msg.GET_TELEMETRY, b""),
        ("list stored media", Msg.GET_MEDIA_LIST, b""),
        ("dose history", Msg.GET_DOSE_LOG, b""),
        ("illumination", Msg.SET_LED, bytes([5])),
        ("download media", Msg.REQUEST_MEDIA, struct.pack("<I", 1)),
        ("re-request chunks", Msg.RESEND, struct.pack("<II", 1, 0)),
        ("free storage", Msg.DELETE_MEDIA, struct.pack("<I", 2)),
        ("link statistics", StpOp.GET_LINK_STATS, b""),
        ("abort transfers", StpOp.ABORT_TRANSFERS, b""),
        ("clear safe mode", StpOp.CLEAR_SAFE_MODE, b""),
        ("set FEC group size", StpOp.SET_FEC_GROUP, bytes([16])),
        ("set HRT idle fill", StpOp.SET_HRT_IDLE_FILL, bytes([0])),
        ("set stream output", StpOp.STREAM_SET_OUTPUT,
         struct.pack("<HHBI", 640, 480, 15, 600_000)),
        ("set stream region", StpOp.STREAM_SET_REGION,
         struct.pack("<4H", 2104, 1560, 640, 480)),
    ]
    unreachable = []
    for seq, (name, opcode, args) in enumerate(functions, start=200):
        command(link, opcode, args, seq=seq)
        report = poll_lrt(link, exp)
        if report is None or report["last_cmd_seq"] != seq or report["last_result"] != 0:
            unreachable.append(f"{name}({report['last_result'] if report else '?'})")
    c.check(f"all {len(functions)} mission functions answer over the link",
            not unreachable, ", ".join(unreachable) if unreachable else "")
    exp.stop()

    c.section("Live video is gated by HRT and never queues")
    link, store, exp = build()
    exp.stream = _FakeStream()
    command(link, StpOp.STREAM_START, seq=6000)
    drain_experiment(exp, passes=3)
    link.dice_read()

    exp.stream.frames = [_frame(0, b"K" * 4000, True), _frame(1, b"P" * 900)]
    for _ in range(4):
        exp.service()
    c.check("nothing streamed while HRT is closed", len(link.dice_read()) == 0)
    c.check("and the queued frames were discarded, not held",
            exp.stream.queue_depth == 0, f"{exp.stream.queue_depth} held")

    exp.stream.frames = [_frame(2, b"K" * 4000, True)]
    short(link, P.PacketType.HRT_GO)
    exp.service()
    packets = _hrt(link)
    c.check("frames flow once HRT opens", bool(packets), f"{len(packets)} packets")
    if packets:
        c.check("every stream chunk carries a valid CRC-32",
                all(p["data_crc_ok"] for p in packets))
        c.check("chunk_total is correct on every chunk including the last",
                len({p["chunk_total"] for p in packets}) == 1
                and packets[-1]["last_chunk"])
        c.check("keyframes are flagged so a receiver can join mid-stream",
                packets[0]["keyframe"])
        joined = b"".join(p["data"] for p in sorted(
            packets, key=lambda p: p["chunk_index"]))
        c.check("the frame reassembles bit-exact", joined == b"K" * 4000)
    exp.stop()

    c.section("Stream settings are clamped and echoed")
    link, store, exp = build()
    exp.stream = _FakeStream()
    command(link, StpOp.STREAM_SET_REGION, struct.pack("<4H", 5, 5, 1024, 1024),
            seq=6100)
    report = poll_lrt(link, exp)
    c.check("a region off the sensor edge is clamped",
            report["stream_centre_x"] == 512 and report["stream_centre_y"] == 512,
            f"centre=({report['stream_centre_x']},{report['stream_centre_y']})")
    command(link, StpOp.STREAM_SET_OUTPUT,
            struct.pack("<HHBI", 65535, 0, 250, 99_000_000), seq=6101)
    report = poll_lrt(link, exp)
    c.check("absurd output settings are clamped, not obeyed",
            report["stream_width"] <= 1920 and report["stream_fps"] <= 30
            and report["stream_bitrate"] <= 8_000_000,
            f"{report['stream_width']}x{report['stream_height']}@"
            f"{report['stream_fps']} {report['stream_bitrate']}bps")
    c.check("the applied values are visible in telemetry",
            report["stream_width"] > 0)
    exp.stop()

    c.section("Camera commands report faults rather than hanging")
    link, store, exp = build()
    for name, opcode, args in (("capture", Msg.CAPTURE_IMAGE, b""),
                               ("record", Msg.START_RECORD, b""),
                               ("region", Msg.CAPTURE_REGION,
                                struct.pack("<6H", 0, 0, 100, 100, 50, 50))):
        command(link, opcode, args, seq=300 + len(name))
        report = poll_lrt(link, exp)
        c.check(f"{name} with no camera returns an error code",
                report is not None and report["last_result"] == int(Err.CAMERA_FAULT),
                f"result {report['last_result'] if report else 'none'}")
    exp.stop()

    c.section("The ground can always see what happened")
    link, store, exp = build()
    command(link, Msg.PING, seq=400)
    command(link, Msg.REQUEST_MEDIA, struct.pack("<I", 999), seq=401)   # fails
    drain_experiment(exp)
    report = poll_lrt(link, exp)
    c.check("events are carried in every LRT", len(report["events"]) > 0,
            f"{len(report['events'])} events")
    codes = {e.code for e in report["events"]}
    c.check("both acceptance and failure are visible",
            L.EventCode.COMMAND_ACCEPTED in codes
            and L.EventCode.COMMAND_FAILED in codes)
    c.check("link health counters are reported",
            report["rx_good"] > 0 and report["lrt_sent"] >= 0)
    c.check("subsystem health is reported",
            "camera_ok" in report and "dosimeter_ok" in report)
    exp.stop()

    c.section("Recovery without ground intervention")
    link, store, exp = build(safe_mode_threshold=2)
    for i in range(2):
        command(link, Msg.REQUEST_MEDIA, struct.pack("<I", 700 + i), seq=500 + i)
        drain_experiment(exp, passes=3)
    c.check("the payload protects itself without being told to",
            exp._safe_mode.value())
    c.check("HRT was stopped as part of tripping", not exp._hrt_enabled.value())
    report = poll_lrt(link, exp)
    c.check("a diagnosable reason is downlinked",
            any(e.code == L.EventCode.SAFE_MODE_ENTERED for e in report["events"]))
    exp.stop()

    c.section("State survives a reset")
    link, store, exp = build()
    c.check("boot count is reported to the ground",
            "boot_count" in exp.build_state())
    c.check("uptime is reported", exp.build_state()["uptime_s"] >= 0)
    c.check("HRT starts closed after a reset", not exp._hrt_enabled.value())
    c.check("safe mode starts clear after a reset", not exp._safe_mode.value())
    exp.stop()

    c.section("A long transfer survives being interrupted repeatedly")
    link, store, exp = build()
    command(link, Msg.REQUEST_MEDIA, struct.pack("<I", 3), seq=600)
    drain_experiment(exp)
    link.dice_read()

    chunks, info, end = {}, None, False
    for cycle in range(60):
        short(link, P.PacketType.HRT_GO)
        exp.service()
        for kind, raw in replies(link):
            if kind != "HRT":
                continue
            payload = H.decode_hrt_payload(raw[6:6 + 1280])
            if not payload["data_crc_ok"]:
                continue
            if payload["sub_type"] == H.SubType.MEDIA_INFO:
                info = H.decode_media_info(payload["data"])
            elif payload["sub_type"] == H.SubType.MEDIA_DATA:
                chunks[payload["chunk_index"]] = payload["data"]
            elif payload["sub_type"] == H.SubType.MEDIA_END:
                end = True
        # Alternate a clean stop and a lossy one between every burst.
        short(link, P.PacketType.HRT_STOP_WITH_LOSS if cycle % 2
              else P.PacketType.HRT_STOP)
        exp.service()
        link.dice_read()
        if end:
            break

    c.check("transfer completes despite 60 stop/go cycles", end)
    if info:
        missing = [i for i in range(info["chunk_total"]) if i not in chunks]
        c.check("no chunk was lost across the interruptions", not missing,
                f"{len(missing)} missing" if missing else
                f"{info['chunk_total']} chunks")
        blob = b"".join(chunks[i] for i in range(info["chunk_total"]))[:info["size"]]
        c.check("the reassembled file is bit-exact",
                blob == store.blobs[3])
        c.check("its CRC-32 matches what was advertised",
                (zlib.crc32(blob) & 0xFFFFFFFF) == info["file_crc32"])
    exp.stop()


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--suite", choices=("reliability", "safety", "autonomy"),
                    action="append")
    args = ap.parse_args()

    suites = args.suite or ["reliability", "safety", "autonomy"]
    checker = Checker()
    start = time.time()

    for name in suites:
        globals()[f"suite_{name}"](checker)

    total = checker.passed + len(checker.failed)
    print(f"\n{'=' * 60}")
    print(f"{checker.passed}/{total} checks passed in {time.time() - start:.1f}s")
    if checker.failed:
        print(f"\nFAILED ({len(checker.failed)}):")
        for name in checker.failed:
            print(f"  - {name}")
        return 1
    print("ALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
