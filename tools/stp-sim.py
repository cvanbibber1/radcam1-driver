#!/usr/bin/env python3
"""DICE master simulator - drives the experiment without a flight computer.

This is the bench counterpart to `radcam/stp/experiment.py`: it sends the three
packet classes DICE sends and decodes the three the experiment sends back. It
exists because every interesting behaviour of the protocol - flow control,
resend, duplicate suppression, safe mode - is a conversation, and none of it
can be tested by inspecting one side alone.

Two transports:

  --port DEV     talk to a real experiment over a serial port. On a bench with
                 two boards this is the real thing; with TX looped to RX it at
                 least proves the framing survives a UART.
  (default)      run the experiment in-process over an in-memory link, which
                 needs no hardware at all and is what the test suite uses.

Examples:

    tools/stp-sim.py --self-test
    tools/stp-sim.py --ping --target 1
    tools/stp-sim.py --port /dev/ttyAMA0 --baud 921600 --lrt --target 1
    tools/stp-sim.py --capture --then-download
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time
import zlib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from radcam.stp import hrt as H                        # noqa: E402
from radcam.stp import lrt as L                        # noqa: E402
from radcam.stp import packets as P                    # noqa: E402
from radcam.stp.commands import encode_command_payload  # noqa: E402
from radcam.stp.crc import CATALOG                     # noqa: E402
from radcam.stp.timebase import unix_to_dice           # noqa: E402
from radcam.stp import fec                            # noqa: E402
from radcam.stp.experiment import StpOp               # noqa: E402
from radcam.protocol import Err, Msg                   # noqa: E402


class MemoryBus:
    """An in-memory two-ended link, so the simulator needs no hardware."""

    def __init__(self):
        self.to_experiment = bytearray()
        self.to_dice = bytearray()
        self.tx_packets = 0
        self.tx_errors = 0

    # -- experiment side (matches Rs422Link) -----------------------------
    def read(self, size: int = 8192) -> bytes:
        data = bytes(self.to_experiment)
        self.to_experiment.clear()
        return data

    def read_wait(self, timeout_s: float = 0.005, size: int = 8192) -> bytes:
        if not self.to_experiment:
            time.sleep(min(timeout_s, 0.001))
        return self.read(size)

    def send(self, data: bytes, abort_check=None) -> bool:
        if abort_check is not None:
            try:
                if abort_check():
                    return False
            except Exception:                          # noqa: BLE001
                pass
        self.to_dice += data
        self.tx_packets += 1
        return True

    # -- DICE side --------------------------------------------------------
    def dice_send(self, data: bytes) -> None:
        self.to_experiment += data

    def dice_read(self) -> bytes:
        data = bytes(self.to_dice)
        self.to_dice.clear()
        return data


class SerialBus:
    """DICE side of a real serial port."""

    def __init__(self, port: str, baud: int):
        import serial
        self.port = serial.Serial(port, baud, timeout=0.2)
        self.buf = bytearray()

    def dice_send(self, data: bytes) -> None:
        self.port.write(data)
        self.port.flush()

    def dice_read(self) -> bytes:
        time.sleep(0.05)
        n = self.port.in_waiting
        return self.port.read(n) if n else b""


class Dice:
    """The master side of the protocol."""

    def __init__(self, bus, wire: P.Wire, verbose: bool = True):
        self.bus = bus
        self.wire = wire
        self.verbose = verbose
        self.cmd_seq = 1

    def _now(self):
        return unix_to_dice(time.time())

    def say(self, *args):
        if self.verbose:
            print(*args)

    # -- transmit ---------------------------------------------------------

    def command(self, opcode: int, args: bytes = b"", flags: int = 0) -> int:
        seq = self.cmd_seq
        self.cmd_seq = (self.cmd_seq + 1) & 0xFFFF
        coarse, fine = self._now()
        payload = encode_command_payload(opcode, seq, args, flags)
        self.bus.dice_send(P.encode_command(payload, coarse, fine, self.wire))
        self.say(f"-> COMMAND opcode=0x{opcode:02X} seq={seq} "
                 f"args={len(args)}B")
        return seq

    def short(self, packet_type: int) -> None:
        coarse, fine = self._now()
        self.bus.dice_send(
            P.encode_short_request(packet_type, coarse, fine, self.wire))
        names = {0x81: "LRT REQUEST", 0x85: "HRT STOP",
                 0x86: "HRT STOP WITH LOSS", 0x87: "HRT GO"}
        self.say(f"-> {names.get(packet_type, hex(packet_type))}")

    # -- receive ----------------------------------------------------------

    def collect(self, settle: float = 0.15) -> list[dict]:
        """Read whatever came back and classify it by length and type."""
        time.sleep(settle)
        raw = bytes(self.bus.dice_read())
        out, i = [], 0
        sync = self.wire.sync_bytes

        while i < len(raw):
            at = raw.find(sync, i)
            if at < 0:
                break
            if at + 6 > len(raw):
                break
            ptype = raw[at + 4]
            if ptype == P.PacketType.COMMAND_ACK and at + 8 <= len(raw):
                out.append({"kind": "ACK", "raw": raw[at:at + 8]})
                i = at + 8
            elif ptype == P.PacketType.LRT_DATA and at + 1256 <= len(raw):
                packet = raw[at:at + 1256]
                out.append({"kind": "LRT", "raw": packet,
                            "data": L.decode_lrt_payload(packet[6:6 + 1248])})
                i = at + 1256
            elif ptype == P.PacketType.HRT_DATA and at + 1288 <= len(raw):
                packet = raw[at:at + 1288]
                out.append({"kind": "HRT", "raw": packet,
                            "data": H.decode_hrt_payload(packet[6:6 + 1280])})
                i = at + 1288
            else:
                i = at + 4
        return out

    def crc_ok(self, packet: bytes) -> bool:
        return self.wire.check_crc(packet, len(packet) - 2)

    # -- conversations -----------------------------------------------------

    def poll_lrt(self) -> dict | None:
        self.short(P.PacketType.LRT_REQUEST)
        for item in self.collect():
            if item["kind"] == "LRT":
                return item["data"]
        return None

    def watch_stream(self, seconds: float = 10.0, out_path: str | None = None):
        """Open HRT, reassemble live video frames, optionally save them.

        Frames are self-describing - frame number, chunk index and count, and a
        keyframe flag - so this can start mid-stream. It waits for the first
        keyframe before writing anything, because a decoder handed inter frames
        with no reference produces garbage.
        """
        self.short(P.PacketType.HRT_GO)
        partial: dict[int, dict[int, bytes]] = {}
        totals: dict[int, int] = {}
        keyframes: set[int] = set()
        complete: list[tuple[int, bytes]] = []
        bad_chunks = 0
        started = time.time()

        while time.time() - started < seconds:
            for item in self.collect(settle=0.02):
                if item["kind"] != "HRT":
                    continue
                payload = item["data"]
                if payload["sub_type"] != H.SubType.STREAM_DATA:
                    continue
                if not payload["data_crc_ok"]:
                    bad_chunks += 1
                    continue
                frame = payload["media_id"]
                partial.setdefault(frame, {})[payload["chunk_index"]] = payload["data"]
                totals[frame] = payload["chunk_total"]
                if payload["keyframe"]:
                    keyframes.add(frame)
                if len(partial[frame]) == totals[frame]:
                    data = b"".join(partial[frame][i] for i in range(totals[frame]))
                    complete.append((frame, data))
                    del partial[frame]

        self.short(P.PacketType.HRT_STOP)
        self.collect()

        complete.sort()
        self.say(f"<- {len(complete)} complete frames, {len(partial)} partial, "
                 f"{bad_chunks} chunks failed CRC")
        if complete:
            span = complete[-1][0] - complete[0][0] + 1
            self.say(f"   frames {complete[0][0]}..{complete[-1][0]}, "
                     f"{len(complete)}/{span} arrived, "
                     f"{len(keyframes)} keyframes")
            self.say(f"   {sum(len(d) for _, d in complete)} bytes in "
                     f"{seconds:.0f}s = "
                     f"{sum(len(d) for _, d in complete) * 8 / seconds / 1000:.0f} kbit/s")

        if out_path and complete:
            first_key = next((i for i, (n, _) in enumerate(complete)
                              if n in keyframes), None)
            if first_key is None:
                self.say("   no keyframe seen; nothing written")
                return complete
            with open(out_path, "wb") as handle:
                for _, data in complete[first_key:]:
                    handle.write(data)
            self.say(f"   wrote {out_path} from the first keyframe onward")
        return complete

    def download(self, media_id: int, max_rounds: int = 4000) -> bytes | None:
        """Full media transfer: request, open the tap, reassemble, verify."""
        self.command(Msg.REQUEST_MEDIA, struct.pack("<I", media_id))
        self.collect()
        self.short(P.PacketType.HRT_GO)

        chunks: dict[int, bytes] = {}
        info = end = None
        bad = []

        for _ in range(max_rounds):
            items = self.collect(settle=0.02)
            if not items:
                if end is not None:
                    break
                continue
            for item in items:
                if item["kind"] != "HRT":
                    continue
                d = item["data"]
                if not d["data_crc_ok"]:
                    bad.append(d["chunk_index"])
                    continue
                if d["sub_type"] == H.SubType.MEDIA_INFO:
                    info = H.decode_media_info(d["data"])
                    self.say(f"<- MEDIA_INFO id={info['media_id']} "
                             f"size={info['size']} chunks={info['chunk_total']}")
                elif d["sub_type"] == H.SubType.MEDIA_DATA:
                    chunks[d["chunk_index"]] = d["data"]
                elif d["sub_type"] == H.SubType.MEDIA_END:
                    end = struct.unpack(">III", d["data"][:12])
                    self.say(f"<- MEDIA_END crc=0x{end[1]:08X}")
            if end is not None and info is not None and \
                    len(chunks) >= info["chunk_total"]:
                break

        self.short(P.PacketType.HRT_STOP)
        self.collect()

        if info is None:
            self.say("!! no MEDIA_INFO - transfer never started")
            return None

        missing = [i for i in range(info["chunk_total"]) if i not in chunks]
        if missing or bad:
            self.say(f"!! missing {len(missing)} chunk(s), {len(bad)} bad CRC; "
                     f"requesting resend")
            wanted = sorted(set(missing + bad))[:24]
            self.command(Msg.RESEND, struct.pack("<I", media_id)
                         + b"".join(struct.pack("<I", c) for c in wanted))
            self.collect()
            self.short(P.PacketType.HRT_GO)
            for _ in range(200):
                items = self.collect(settle=0.02)
                if not items:
                    break
                for item in items:
                    if item["kind"] == "HRT" and item["data"]["data_crc_ok"] \
                            and item["data"]["sub_type"] == H.SubType.MEDIA_DATA:
                        chunks[item["data"]["chunk_index"]] = item["data"]["data"]
            self.short(P.PacketType.HRT_STOP)
            self.collect()

        missing = [i for i in range(info["chunk_total"]) if i not in chunks]
        if missing:
            self.say(f"!! still missing {len(missing)} chunk(s)")
            return None

        blob = b"".join(chunks[i] for i in range(info["chunk_total"]))
        blob = blob[:info["size"]]
        ok = (zlib.crc32(blob) & 0xFFFFFFFF) == info["file_crc32"]
        self.say(f"<- reassembled {len(blob)} bytes, CRC {'OK' if ok else 'BAD'}")
        return blob if ok else None


# ------------------------------------------------------------------ helpers

def describe_lrt(d: dict) -> None:
    print(f"   uptime={d['uptime_s']}s boot={d['boot_count']} "
          f"safe_mode={d['safe_mode']}")
    print(f"   last cmd: opcode=0x{d['last_opcode']:02X} "
          f"seq={d['last_cmd_seq']} result={d['last_result']} "
          f"({'OK' if d['last_result'] == 0 else Err(d['last_result']).name if d['last_result'] in [e.value for e in Err] else '?'})")
    print(f"   dose={d['dose_rad']:.4f} rad  volts={d['dose_volts']:.6f}  "
          f"cal={d['dose_calibrated']}  temp={d['cpu_temp_c']:.1f}C")
    print(f"   camera={d['camera_available']} media={d['media_count']} "
          f"free={d['storage_free']}B")
    print(f"   rx good={d['rx_good']} badcrc={d['rx_bad_crc']} "
          f"notus={d['rx_not_for_us']} resync={d['rx_resyncs']}")
    print(f"   hrt={d['hrt_enabled']} xfer state={d['xfer_state']} "
          f"{d['xfer_chunk_next']}/{d['xfer_chunk_total']}")
    if d["resp_data"]:
        print(f"   response: opcode=0x{d['resp_opcode']:02X} "
              f"{len(d['resp_data'])}B{' (truncated)' if d['resp_truncated'] else ''}")
    if d["events"]:
        print(f"   events ({len(d['events'])}): "
              + ", ".join(f"0x{e.code:04X}/{e.arg}" for e in d["events"][-6:]))
    print(f"   payload CRC32 {'ok' if d['payload_crc_ok'] else 'BAD'}")


def build_in_process(target_id: int, wire: P.Wire):
    """An experiment running in this process, over a MemoryBus."""
    from dataclasses import dataclass
    from radcam.protocol import Config as ProtoConfig, Dispatcher
    from radcam.stp.experiment import Experiment, ExperimentConfig

    @dataclass
    class Record:
        media_id: int
        kind: str
        size: int
        width: int
        height: int
        created_unix: float

    class FakeStore:
        def __init__(self):
            self.blobs = {1: os.urandom(40000), 2: os.urandom(3000)}

        def read(self, media_id):
            return self.blobs.get(media_id)

        def list(self):
            return [Record(k, "image", len(v), 1920, 1080, time.time())
                    for k, v in self.blobs.items()]

        def delete(self, media_id):
            return self.blobs.pop(media_id, None) is not None

    bus = MemoryBus()
    store = FakeStore()
    experiment = Experiment(
        link=bus, wire=wire, dispatcher=Dispatcher(config=ProtoConfig()),
        store=store, config=ExperimentConfig(target_id=target_id))
    experiment.start()
    return bus, experiment, store


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial device; omit for in-process")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--target", type=lambda v: int(v, 0), default=0xC7,
                    help="Target ID of the experiment (default 0xC7)")
    ap.add_argument("--little-endian", action="store_true")
    ap.add_argument("--crc", default="CRC-16/CCITT-FALSE")
    ap.add_argument("--ping", action="store_true")
    ap.add_argument("--lrt", action="store_true")
    ap.add_argument("--capture", action="store_true")
    ap.add_argument("--download", type=int, metavar="MEDIA_ID",
                    help="download over HRT (needs the master to open the tap)")
    ap.add_argument("--lrt-download", type=int, metavar="MEDIA_ID",
                    help="download over LRT polls; works with HRT closed")
    ap.add_argument("--fec-group", type=int, default=None,
                    help="parity group size for an LRT download; 0 disables")
    ap.add_argument("--hrt-go", action="store_true")
    ap.add_argument("--hrt-stop", action="store_true")
    ap.add_argument("--self-test", action="store_true",
                    help="run the full conversation in-process")
    args = ap.parse_args()

    crc = next((c for c in CATALOG if c.name.upper() == args.crc.upper()),
               CATALOG[0])
    wire = P.Wire(big_endian=not args.little_endian, crc=crc,
                  target_id=args.target)

    experiment = None
    if args.port:
        bus = SerialBus(args.port, args.baud)
    else:
        bus, experiment, _store = build_in_process(args.target, wire)

    dice = Dice(bus, wire)

    def pump():
        """In-process only: give the experiment a chance to run."""
        if experiment is not None:
            for _ in range(6):
                experiment.service()
                time.sleep(0.01)

    try:
        if args.self_test:
            return self_test(dice, experiment, bus)

        if args.ping:
            dice.command(Msg.PING)
            pump()
            print(f"<- {len(dice.collect())} packet(s)")
        if args.capture:
            dice.command(Msg.CAPTURE_IMAGE)
            pump()
        if args.hrt_go:
            dice.short(P.PacketType.HRT_GO)
            pump()
        if args.hrt_stop:
            dice.short(P.PacketType.HRT_STOP)
            pump()
        if args.stream_size:
            spec, _, kbps = args.stream_size.partition(":")
            dims, _, fps = spec.partition("@")
            width, _, height = dims.partition("x")
            dice.command(StpOp.STREAM_SET_OUTPUT,
                         struct.pack("<HHBI", int(width), int(height),
                                     int(fps or 15), int(kbps or 600) * 1000))
            pump()
            dice.collect()
        if args.stream_region:
            cx, cy, w, h = (int(v) for v in args.stream_region.split(","))
            dice.command(StpOp.STREAM_SET_REGION,
                         struct.pack("<4H", cx, cy, w, h))
            pump()
            dice.collect()
        if args.stream_start:
            dice.command(StpOp.STREAM_START)
            pump()
            dice.collect()
        if args.stream_stop:
            dice.command(StpOp.STREAM_STOP)
            pump()
            dice.collect()
        if args.watch_stream is not None:
            dice.watch_stream(args.watch_stream, args.stream_out)
        if args.download is not None:
            if experiment is not None:
                print("!! --download needs --port; in-process use --self-test")
            else:
                blob = dice.download(args.download)
                print("downloaded", len(blob) if blob else 0, "bytes")
        if args.lrt or not any((args.ping, args.capture, args.download,
                                args.hrt_go, args.hrt_stop, args.stream_start,
                                args.stream_stop, args.stream_size,
                                args.stream_region,
                                args.watch_stream is not None)):
            dice.short(P.PacketType.LRT_REQUEST)
            pump()
            for item in dice.collect():
                if item["kind"] == "LRT":
                    print("<- LRT DATA")
                    describe_lrt(item["data"])
    finally:
        if experiment is not None:
            experiment.stop()
    return 0


def self_test(dice: "Dice", experiment, bus) -> int:
    """Exercise the whole protocol in-process and report pass/fail."""
    failures = []

    def check(name, condition, detail=""):
        print(f"  {'PASS' if condition else 'FAIL'}  {name}"
              + (f"   {detail}" if detail else ""))
        if not condition:
            failures.append(name)

    def pump(n=8):
        for _ in range(n):
            experiment.service()
            time.sleep(0.02)

    dice.verbose = False
    print("STP self-test (in-process)\n")

    print("Command / ACK")
    seq = dice.command(Msg.PING)
    pump()
    items = dice.collect()
    acks = [i for i in items if i["kind"] == "ACK"]
    check("command is acknowledged", len(acks) == 1)
    check("ACK is 8 bytes", acks and len(acks[0]["raw"]) == 8)
    check("ACK CRC valid", acks and dice.crc_ok(acks[0]["raw"]))
    check("ACK carries our target id", acks and acks[0]["raw"][5] == dice.wire.target_id)

    print("\nLRT")
    d = None
    dice.short(P.PacketType.LRT_REQUEST)
    pump()
    for item in dice.collect():
        if item["kind"] == "LRT":
            d = item["data"]
    check("LRT returned", d is not None)
    if d:
        check("LRT payload CRC32 valid", d["payload_crc_ok"])
        check("command result reported", d["last_cmd_seq"] == seq
              and d["last_result"] == 0,
              f"seq={d['last_cmd_seq']} result={d['last_result']}")

    print("\nTarget filtering")
    foreign = P.Wire(big_endian=dice.wire.big_endian, crc=dice.wire.crc,
                     target_id=(dice.wire.target_id + 1) & 0xFF)
    bus.dice_send(P.encode_command(encode_command_payload(Msg.PING, 999),
                                   0, 0, foreign))
    pump()
    check("foreign target ignored", len(dice.collect()) == 0)

    print("\nHRT flow control")
    dice.command(Msg.REQUEST_MEDIA, struct.pack("<I", 1))
    pump()
    dice.collect()
    pump(2)
    check("nothing sent before HRT Go", len([
        i for i in dice.collect() if i["kind"] == "HRT"]) == 0)

    dice.short(P.PacketType.HRT_GO)
    chunks, info, end = {}, None, None
    for _ in range(60):
        pump(2)
        for item in dice.collect(settle=0.0):
            if item["kind"] != "HRT":
                continue
            payload = item["data"]
            if not payload["data_crc_ok"]:
                continue
            if payload["sub_type"] == H.SubType.MEDIA_INFO:
                info = H.decode_media_info(payload["data"])
            elif payload["sub_type"] == H.SubType.MEDIA_DATA:
                chunks[payload["chunk_index"]] = payload["data"]
            elif payload["sub_type"] == H.SubType.MEDIA_END:
                end = payload
        if end is not None:
            break
    check("MEDIA_INFO received", info is not None)
    check("MEDIA_END received", end is not None)
    if info:
        blob = b"".join(chunks[i] for i in sorted(chunks))[:info["size"]]
        check("all chunks arrived", len(chunks) == info["chunk_total"],
              f"{len(chunks)}/{info['chunk_total']}")
        check("file CRC matches", (zlib.crc32(blob) & 0xFFFFFFFF)
              == info["file_crc32"])

    dice.short(P.PacketType.HRT_STOP)
    pump()
    dice.collect()
    pump(3)
    check("HRT Stop silences the link", len(dice.collect()) == 0)

    print("\nDuplicate suppression")
    dice.collect()
    before = experiment._cmds_executed.value()
    fixed = dice.cmd_seq
    for _ in range(3):
        dice.cmd_seq = fixed
        dice.command(Msg.PING)
        experiment.service()
    pump()
    check("repeated cmd_seq executes once",
          experiment._cmds_executed.value() - before == 1,
          f"executed {experiment._cmds_executed.value() - before}")

    print("\nLive stream configuration")
    dice.collect()
    dice.command(StpOp.STREAM_SET_REGION, struct.pack("<4H", 3000, 2000, 512, 512))
    pump()
    dice.collect()
    report = None
    dice.short(P.PacketType.LRT_REQUEST)
    pump(1)
    for item in dice.collect(settle=0.0):
        if item["kind"] == "LRT":
            report = item["data"]
    check("region is applied and echoed in telemetry",
          report is not None and report["stream_centre_x"] == 3000
          and report["stream_crop_w"] == 512,
          f"centre={report['stream_centre_x'] if report else '?'}")
    dice.command(StpOp.STREAM_SET_REGION, struct.pack("<4H", 5, 5, 1024, 1024))
    pump()
    dice.collect()
    dice.short(P.PacketType.LRT_REQUEST)
    pump(1)
    for item in dice.collect(settle=0.0):
        if item["kind"] == "LRT":
            report = item["data"]
    check("a region off the sensor edge is clamped, and the clamp reported",
          report is not None and report["stream_centre_x"] == 512,
          f"centre={report['stream_centre_x'] if report else '?'}")

    print("\nCorruption")
    pump()
    dice.collect()          # drop replies owed to earlier phases
    good = P.encode_command(encode_command_payload(Msg.PING, 4321), 0, 0,
                            dice.wire)
    broken = bytearray(good)
    broken[40] ^= 0xFF
    bus.dice_send(bytes(broken))
    pump()
    check("corrupted command not acknowledged", len(dice.collect()) == 0)
    bus.dice_send(b"\x00\x11\x22" + good)
    pump()
    check("resyncs after garbage", len([
        i for i in dice.collect() if i["kind"] == "ACK"]) == 1)

    print(f"\n{'ALL PASS' if not failures else str(len(failures)) + ' FAILURE(S): ' + ', '.join(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
