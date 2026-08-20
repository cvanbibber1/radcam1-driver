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

    def send(self, data: bytes) -> bool:
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

    def download_lrt(self, media_id: int, max_polls: int = 4000,
                     group_size: int | None = None) -> bytes | None:
        """Pull a file through LRT polls, repairing losses from parity.

        This is the downlink that needs no permission from the master. It is
        much slower than HRT, and it is the one that always works.
        """
        args = struct.pack("<I", media_id)
        if group_size is not None:
            args += struct.pack("<HH", 0, group_size)
        self.command(StpOp.LRT_FILE_START, args)
        self.collect()

        data: dict[int, bytes] = {}
        parity: dict[int, bytes] = {}
        final = None

        for _ in range(max_polls):
            report = self.poll_lrt()
            if report is None:
                continue
            state = report["file_state"]
            if state == L.FILE_COMPLETE:
                final = report
                break
            if state != L.FILE_ACTIVE:
                continue
            if not report["file_data_crc_ok"]:
                self.say(f"   chunk {report['file_chunk_index']} failed CRC")
                continue
            if report["file_is_parity"]:
                parity[report["file_chunk_index"]] = report["file_data"]
            else:
                data[report["file_chunk_index"]] = report["file_data"]

        if final is None:
            self.say("!! LRT transfer did not complete")
            return None

        total = final["file_chunk_total"]
        group = final["file_fec_group"]
        self.say(f"<- LRT transfer: {len(data)}/{total} chunks, "
                 f"{len(parity)} parity, {final['file_size']} bytes")

        stats = fec.FecStats()
        missing = fec.verify_and_repair(data, parity, total, group,
                                        L.FILE_DATA_MAX, stats)
        if stats.recovered:
            self.say(f"   parity rebuilt {stats.recovered} chunk(s) with no "
                     f"retransmission")

        if missing:
            self.say(f"   requesting resend of {len(missing)} chunk(s)")
            self.command(StpOp.LRT_FILE_RESEND, struct.pack("<I", media_id)
                         + b"".join(struct.pack("<I", i) for i in missing[:24]))
            self.collect()
            for _ in range(len(missing) + 8):
                report = self.poll_lrt()
                if report and report["file_state"] == L.FILE_ACTIVE \
                        and report["file_data_crc_ok"] \
                        and not report["file_is_parity"]:
                    data[report["file_chunk_index"]] = report["file_data"]
            missing = [i for i in range(total) if i not in data]

        if missing:
            self.say(f"!! still missing {len(missing)} chunk(s)")
            return None

        blob = b"".join(data[i] for i in range(total))[:final["file_size"]]
        ok = (zlib.crc32(blob) & 0xFFFFFFFF) == final["file_crc32"]
        self.say(f"<- reassembled {len(blob)} bytes, CRC {'OK' if ok else 'BAD'}")
        return blob if ok else None

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
    ap.add_argument("--target", type=int, default=1)
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
        if args.lrt_download is not None:
            blob = dice.download_lrt(args.lrt_download,
                                     group_size=args.fec_group)
            print("downloaded", len(blob) if blob else 0, "bytes over LRT")
        if args.download is not None:
            if experiment is not None:
                print("!! --download needs --port; in-process use --self-test")
            else:
                blob = dice.download(args.download)
                print("downloaded", len(blob) if blob else 0, "bytes")
        if args.lrt or not any((args.ping, args.capture, args.download,
                                args.lrt_download, args.hrt_go, args.hrt_stop)):
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

    print("\nLRT file transfer (no HRT permission)")
    dice.collect()
    dice.command(StpOp.LRT_FILE_START, struct.pack("<I", 2))
    pump()
    dice.collect()
    lrt_data, lrt_parity, lrt_final = {}, {}, None
    for _ in range(80):
        pump(2)
        dice.short(P.PacketType.LRT_REQUEST)
        pump(1)
        for item in dice.collect(settle=0.0):
            if item["kind"] != "LRT":
                continue
            report = item["data"]
            if report["file_state"] == L.FILE_COMPLETE:
                lrt_final = report
            elif report["file_state"] == L.FILE_ACTIVE and report["file_data_crc_ok"]:
                if report["file_is_parity"]:
                    lrt_parity[report["file_chunk_index"]] = report["file_data"]
                else:
                    lrt_data[report["file_chunk_index"]] = report["file_data"]
        if lrt_final:
            break
    check("LRT transfer completes with HRT closed", lrt_final is not None)
    check("HRT was never opened", not experiment._hrt_enabled.value())
    if lrt_final:
        blob = b"".join(lrt_data[i] for i in sorted(lrt_data))[:lrt_final["file_size"]]
        check("LRT file CRC matches",
              (zlib.crc32(blob) & 0xFFFFFFFF) == lrt_final["file_crc32"])
        check("parity chunks were emitted", bool(lrt_parity),
              f"{len(lrt_parity)} parity chunk(s)")

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
