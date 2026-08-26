#!/usr/bin/env python3
"""Transmit LRT and HRT packets unsolicited, to test a host's receive path.

**This is a bench tool and must never run in flight.** The payload is a slave:
it transmits only when polled, and a beacon on a shared bus would corrupt every
other experiment's traffic. It exists for exactly one situation - bringing up a
ground station, where you need to know whether the host can see and decode the
payload's packets before the command path works.

That matters because a silent link is ambiguous. If the host sends requests and
sees nothing back, the fault could be in either direction. Running this removes
one variable: whatever the host makes of these packets says something about the
host's receive path alone.

    sudo systemctl stop radcamd
    sudo tools/stp-beacon.py --seconds 60
    sudo systemctl start radcamd

The packets are real: correct sync, target, CRC and payload structure. A host
that decodes these will decode live telemetry.
"""

from __future__ import annotations

import argparse
import os
import struct
import zlib
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from radcam.stp import hrt as H                        # noqa: E402
from radcam.stp import lrt as L                        # noqa: E402
from radcam.stp import packets as P                    # noqa: E402
from radcam.stp.link import DeLine, NullDeLine, Rs422Link   # noqa: E402
from radcam.stp.timebase import unix_to_dice           # noqa: E402


def build_lrt(wire: P.Wire, target: int, count: int, started: float) -> bytes:
    """A telemetry packet with plausible, moving values."""
    coarse, fine = unix_to_dice(time.time())
    payload = L.build_lrt_payload({
        "uptime_s": int(time.time() - started),
        "boot_count": 1,
        "target_id": target,
        "coarse_time": coarse,
        "fine_time": fine,
        "fw_major": 1, "fw_minor": 0,
        "last_cmd_seq": count & 0xFFFF,
        "last_result": 0,
        "cmds_received": count,
        "cmds_executed": count,
        # Moving so a host plot visibly updates rather than looking frozen.
        "dose_rad": 0.0700 + 0.0001 * (count % 100),
        "dose_volts": 1.1136 + 0.00001 * (count % 100),
        "dose_calibrated": True,
        "cpu_temp_c": 50.0 + (count % 20) * 0.1,
        "camera_available": True, "camera_ok": True, "dosimeter_ok": True,
        "storage_free": 12_000_000_000, "storage_used": 3_000_000_000,
        "slot_count": 16, "slots_used": 2, "slot_recording": -1,
        "slot_downloading": -1, "slot_bytes_used": 4_500_000,
        "rx_good": count, "lrt_sent": count,
        "fec_group_size": 16,
        "stream_state": L.STREAM_OFF,
    }, [L.Event(int(time.time() - started), L.EventCode.BOOT, 1, L.SEV_INFO)])
    return P.encode_lrt_data(payload, wire, target)


def build_hrt(wire: P.Wire, target: int, index: int, total: int) -> bytes:
    """A recognisable file chunk: a counting pattern the host can verify."""
    body = bytes(((index + i) & 0xFF) for i in range(H.HRT_CHUNK_DATA))
    flags = H.FLAG_LAST_CHUNK if index == total - 1 else 0
    payload = H.build_hrt_payload(H.SubType.MEDIA_DATA, 0x51000000, index,
                                  total, body, flags)
    return P.encode_hrt_data(payload, wire, target)


def transmit_file(link, wire, target, path: str, group_size: int,
                  repeat: bool = True, seconds: float = 60.0) -> int:
    """Send a real file over HRT as a complete, conformant transfer.

    Emits the same sequence a commanded download does - MEDIA_INFO, numbered
    MEDIA_DATA chunks, XOR parity every `group_size`, then MEDIA_END with the
    whole-file CRC-32 - so a ground station that reassembles this correctly
    will reassemble a real capture correctly. Repeating it means a receiver can
    join at any point and still get a whole file from the next MEDIA_INFO.
    """
    from radcam.stp.fec import group_count, indices_in_group, parity_of

    data = open(path, "rb").read()
    file_crc = zlib.crc32(data) & 0xFFFFFFFF
    chunk_total = max(1, -(-len(data) // H.HRT_CHUNK_DATA))

    def chunk(i):
        return data[i * H.HRT_CHUNK_DATA:(i + 1) * H.HRT_CHUNK_DATA]

    print(f"  file      : {path}")
    print(f"  size      : {len(data)} bytes, CRC-32 0x{file_crc:08X}")
    print(f"  chunks    : {chunk_total} of {H.HRT_CHUNK_DATA} B"
          f" + {group_count(chunk_total, group_size)} parity")
    print()

    passes = 0
    start = time.time()
    while time.time() - start < seconds:
        passes += 1
        link.send(P.encode_hrt_data(H.build_hrt_payload(
            H.SubType.MEDIA_INFO, 0x51000000, 0, chunk_total,
            H.encode_media_info(0x51000000, len(data), chunk_total, file_crc,
                                0, 640, 480, time.time())), wire, target))

        for index in range(chunk_total):
            flags = H.FLAG_LAST_CHUNK if index == chunk_total - 1 else 0
            link.send(P.encode_hrt_data(H.build_hrt_payload(
                H.SubType.MEDIA_DATA, 0x51000000, index, chunk_total,
                chunk(index), flags), wire, target))

            # Parity as each group closes, so an interrupted transfer still
            # leaves completed groups repairable.
            if group_size and (index + 1) % group_size == 0:
                group = index // group_size
                link.send(P.encode_hrt_data(H.build_hrt_payload(
                    H.SubType.MEDIA_PARITY, 0x51000000, group, chunk_total,
                    parity_of([chunk(i) for i in
                               indices_in_group(group, group_size, chunk_total)],
                              H.HRT_CHUNK_DATA), H.FLAG_PARITY), wire, target))

        if group_size:
            last = group_count(chunk_total, group_size) - 1
            if last >= 0 and chunk_total % group_size:
                link.send(P.encode_hrt_data(H.build_hrt_payload(
                    H.SubType.MEDIA_PARITY, 0x51000000, last, chunk_total,
                    parity_of([chunk(i) for i in
                               indices_in_group(last, group_size, chunk_total)],
                              H.HRT_CHUNK_DATA), H.FLAG_PARITY), wire, target))

        link.send(P.encode_hrt_data(H.build_hrt_payload(
            H.SubType.MEDIA_END, 0x51000000, chunk_total, chunk_total,
            struct.pack(">III", 0x51000000, file_crc, chunk_total)),
            wire, target))

        print(f"\r  pass {passes:4d} complete   {link.tx_bytes:9d} bytes sent",
              end="", flush=True)
        if not repeat:
            break
        time.sleep(1.0)
    print()
    return passes


def stream_forever(link, wire, target, args) -> int:
    """Live video over HRT, with telemetry interleaved, until stopped.

    The payload never does this unsolicited in flight - HRT flows only between
    an HRT Go and a Stop. This exists for a bench where the return path is not
    yet working, so the ground can develop against a real stream rather than a
    recording.

    Two rules from the flight code carry over because they are what make a
    stream a stream. Frames are taken newest-available and stale ones are
    dropped rather than queued, since on a fixed-rate link a queue converts a
    bandwidth shortfall into latency that only grows. And telemetry is
    interleaved between whole frames, never inside one, so a frame is never
    split around 13.7 ms of LRT.
    """
    from radcam.stream import StreamConfig, VideoStream

    cfg = StreamConfig(width=args.width, height=args.height, fps=args.fps,
                       bitrate=args.bitrate, crop_w=args.crop_w or args.width,
                       crop_h=args.crop_h or args.height,
                       centre_x=args.centre_x, centre_y=args.centre_y)
    stream = VideoStream(cfg, queue_frames=args.queue)
    if not stream.available:
        print(f"  cannot stream, missing: {', '.join(stream.missing())}")
        return 2
    if not stream.start():
        print(f"  encoder failed to start: {stream.fault}")
        return 2

    applied = stream.config
    hrt_ms = (P.HRT_DATA_PACKET_SIZE * 10 / args.baud + 30e-6) * 1000
    print(f"  stream    : {applied.describe()}")
    print(f"  HRT packet: {hrt_ms:.2f} ms   budget {1000/applied.fps:.1f} ms/frame")
    print(f"  telemetry : every {args.lrt_period:.0f} s, between frames")
    print()

    frame_no = 0
    chunks_sent = 0
    lrt_sent = 0
    dropped_seen = 0
    next_lrt = time.time() + args.lrt_period
    started = time.time()
    last_report = started

    try:
        while True:
            frame = stream.take()
            if frame is None:
                # Between frames is exactly when telemetry should go out.
                if time.time() >= next_lrt:
                    next_lrt = time.time() + args.lrt_period
                    link.send(build_lrt(wire, target, lrt_sent, started))
                    lrt_sent += 1
                else:
                    time.sleep(0.002)
                continue

            data = frame.data
            total = max(1, -(-len(data) // H.HRT_CHUNK_DATA))
            for index in range(total):
                piece = data[index * H.HRT_CHUNK_DATA:
                             (index + 1) * H.HRT_CHUNK_DATA]
                flags = H.FLAG_KEYFRAME if frame.keyframe else 0
                if index == total - 1:
                    flags |= H.FLAG_LAST_CHUNK
                link.send(P.encode_hrt_data(H.build_hrt_payload(
                    H.SubType.STREAM_DATA, frame.index, index, total,
                    piece, flags), wire, target))
                chunks_sent += 1
            frame_no += 1

            now = time.time()
            if now - last_report >= 2.0:
                last_report = now
                elapsed = now - started
                dropped_seen = stream.frames_dropped
                print(f"\r  {elapsed:7.0f}s  frames {frame_no:6d} "
                      f"({frame_no/elapsed:5.2f} fps)  chunks {chunks_sent:8d}  "
                      f"LRT {lrt_sent:5d}  dropped {dropped_seen:5d}  "
                      f"{link.tx_bytes*8/elapsed/1000:6.0f} kbit/s",
                      end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        stream.stop()
        link.close()
    print()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--target", type=lambda v: int(v, 0), default=0xC7)
    ap.add_argument("--de-gpio", type=int, default=4)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--lrt-hz", type=float, default=1.0,
                    help="telemetry packets per second")
    ap.add_argument("--hrt", action="store_true",
                    help="also send a repeating file transfer over HRT")
    ap.add_argument("--hrt-chunks", type=int, default=8)
    ap.add_argument("--image", metavar="FILE",
                    help="transmit a real file over HRT as a full transfer")
    ap.add_argument("--fec-group", type=int, default=16,
                    help="parity group size for --image")
    ap.add_argument("--stream", action="store_true",
                    help="live video over HRT, with telemetry interleaved")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--bitrate", type=int, default=600000)
    ap.add_argument("--crop-w", type=int, default=0)
    ap.add_argument("--crop-h", type=int, default=0)
    ap.add_argument("--centre-x", type=int, default=2104)
    ap.add_argument("--centre-y", type=int, default=1560)
    ap.add_argument("--queue", type=int, default=8)
    ap.add_argument("--lrt-period", type=float, default=2.0,
                    help="seconds between telemetry packets in --stream")
    ap.add_argument("--ack-only", action="store_true",
                    help="send only 8-byte ACKs - the smallest valid packet")
    args = ap.parse_args()

    wire = P.Wire(target_id=args.target)
    de = NullDeLine() if args.de_gpio < 0 else DeLine(gpio=args.de_gpio)
    link = Rs422Link(port=args.port, baud=args.baud, de=de)

    try:
        link.open()
    except Exception as exc:                            # noqa: BLE001
        print(f"cannot open {args.port}: {exc}")
        print("(radcamd holds the port; stop it first)")
        return 2

    print(f"\nBeaconing on {args.port} at {args.baud} baud as target "
          f"0x{args.target:02X} for {args.seconds:.0f} s.")
    print("This is a bench tool. The payload does not do this in normal use.\n")

    if args.stream:
        return stream_forever(link, wire, args.target, args)

    if args.image:
        try:
            transmit_file(link, wire, args.target, args.image,
                          args.fec_group, True, args.seconds)
        finally:
            link.close()
        return 0

    started = time.time()
    interval = 1.0 / max(args.lrt_hz, 0.01)
    next_lrt = time.time()
    counts = {"LRT": 0, "HRT": 0, "ACK": 0}
    chunk = 0

    try:
        while args.seconds <= 0 or time.time() - started < args.seconds:
            now = time.time()
            if now >= next_lrt:
                next_lrt += interval
                if args.ack_only:
                    link.send(P.encode_command_ack(wire, args.target))
                    counts["ACK"] += 1
                else:
                    link.send(build_lrt(wire, args.target, counts["LRT"],
                                        started))
                    counts["LRT"] += 1

                if args.hrt:
                    link.send(build_hrt(wire, args.target, chunk,
                                        args.hrt_chunks))
                    counts["HRT"] += 1
                    chunk = (chunk + 1) % args.hrt_chunks

                elapsed = now - started
                print(f"\r  t+{elapsed:5.1f}s  "
                      + "  ".join(f"{k} {v}" for k, v in counts.items()
                                  if v or k == "LRT")
                      + f"   {link.tx_bytes} bytes sent", end="", flush=True)
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()

    print(f"\n\nsent: " + ", ".join(f"{k}={v}" for k, v in counts.items()))
    print(f"link stats: {link.stats()}")
    print("\nIf the host saw none of this, the fault is in the")
    print("payload-to-host direction or the host's receiver.")
    print("If the host saw it, that direction is fine and the problem is")
    print("in the host-to-payload direction only.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
