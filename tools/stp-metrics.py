#!/usr/bin/env python3
"""Timing and throughput metrics for the DICE link and the camera pipeline.

Everything here is either measured on this hardware or derived from a measured
constant. Nothing is a vendor figure. The point is to answer the questions that
actually come up when planning a pass:

  * how long does one video frame take to encode, and to transmit?
  * how many frames per second can the link carry at a given size?
  * how long will a 3 MB image take to come down?
  * how much of the link does a stream leave for anything else?

    tools/stp-metrics.py                 # link arithmetic, no hardware needed
    tools/stp-metrics.py --link          # measure the UART and DE overhead
    tools/stp-metrics.py --stream 12     # measure the encoder for 12 seconds
    tools/stp-metrics.py --all

Measuring the link needs the port, which `radcamd` holds:

    sudo systemctl stop radcamd && sudo tools/stp-metrics.py --all
    sudo systemctl start radcamd
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from radcam.stp import packets as P                    # noqa: E402
from radcam.stp.hrt import HRT_CHUNK_DATA              # noqa: E402
from radcam.stp.link import DeLine, NullDeLine, Rs422Link   # noqa: E402

BAUD = 921600
HRT_PACKET = P.HRT_DATA_PACKET_SIZE          # 1288
LRT_PACKET = P.LRT_DATA_PACKET_SIZE          # 1256
ACK_PACKET = P.COMMAND_ACK_SIZE              # 8

#: Measured, not assumed: DE is released this long after the last stop bit.
#: Re-measure with tools/stp-de-timing.py if the transmit path changes.
DE_OVERHEAD_S = 30e-6


def char_time(baud: int = BAUD) -> float:
    return 10.0 / baud


def packet_time(size: int, baud: int = BAUD,
                overhead: float = DE_OVERHEAD_S) -> float:
    return size * char_time(baud) + overhead


def section(title: str) -> None:
    print(f"\n{title}\n{'-' * len(title)}")


def link_arithmetic(baud: int) -> dict:
    section(f"Link arithmetic at {baud} baud")
    ct = char_time(baud)
    hrt = packet_time(HRT_PACKET, baud)
    lrt = packet_time(LRT_PACKET, baud)
    ack = packet_time(ACK_PACKET, baud)

    rate = 1.0 / hrt
    payload_rate = rate * HRT_CHUNK_DATA

    print(f"  character time            {ct * 1e6:8.2f} us")
    print(f"  DE overhead per packet    {DE_OVERHEAD_S * 1e6:8.2f} us  (measured)")
    print()
    print(f"  Command ACK    {ACK_PACKET:5d} B  {ack * 1e3:8.3f} ms")
    print(f"  LRT Data       {LRT_PACKET:5d} B  {lrt * 1e3:8.3f} ms")
    print(f"  HRT Data       {HRT_PACKET:5d} B  {hrt * 1e3:8.3f} ms")
    print()
    print(f"  HRT packets per second    {rate:8.1f}")
    print(f"  HRT payload throughput    {payload_rate / 1024:8.1f} kB/s"
          f"   ({payload_rate * 8 / 1000:.0f} kbit/s)")
    print(f"  framing efficiency        {100 * HRT_CHUNK_DATA / HRT_PACKET:8.1f} %")
    return {"hrt": hrt, "lrt": lrt, "payload_rate": payload_rate}


def transfer_table(payload_rate: float) -> None:
    section("Time to downlink a stored file")
    print(f"  {'size':>10}  {'chunks':>8}  {'packets':>8}  {'time':>12}")
    for size, label in ((100 * 1024, "100 kB"), (500 * 1024, "500 kB"),
                        (1024 * 1024, "1 MB"), (3 * 1024 * 1024, "3 MB"),
                        (5 * 1024 * 1024, "5 MB"), (17 * 1024 * 1024, "17 MB")):
        chunks = -(-size // HRT_CHUNK_DATA)
        # info + data + parity + end
        packets = chunks + 2 + -(-chunks // 16)
        seconds = packets * packet_time(HRT_PACKET)
        shown = (f"{seconds:.1f} s" if seconds < 90
                 else f"{seconds / 60:.1f} min")
        print(f"  {label:>10}  {chunks:8d}  {packets:8d}  {shown:>12}")
    print("\n  Includes MEDIA_INFO, MEDIA_END and one parity chunk per 16.")


def stream_budget(payload_rate: float) -> None:
    section("Live video budget")
    print(f"  {'resolution':>12} {'fps':>4} {'kbit/s':>8} {'kB/frame':>9}"
          f" {'chunks':>7} {'ms/frame TX':>12} {'link':>7}")
    for width, height, fps, bitrate in (
            (320, 240, 15, 400_000), (640, 480, 10, 600_000),
            (640, 480, 15, 600_000), (640, 480, 15, 700_000),
            (1280, 720, 10, 600_000), (1920, 1080, 5, 600_000)):
        frame_bytes = bitrate / 8 / fps
        chunks = max(1, -(-int(frame_bytes) // HRT_CHUNK_DATA))
        tx_per_frame = chunks * packet_time(HRT_PACKET)
        budget = 1.0 / fps
        use = 100 * tx_per_frame / budget
        flag = "" if use < 90 else "  OVER" if use > 100 else "  tight"
        print(f"  {width:5d}x{height:<6d} {fps:4d} {bitrate // 1000:8d}"
              f" {frame_bytes / 1024:9.1f} {chunks:7d} {tx_per_frame * 1e3:12.1f}"
              f" {use:6.0f}%{flag}")
    print("\n  'link' is the share of each frame interval spent transmitting.")
    print("  Above 100% the encoder outruns the link and frames are dropped -")
    print("  which is correct behaviour, but it means the configured bitrate")
    print("  is not actually being delivered.")


def latency_model() -> None:
    section("Frame latency, capture to last byte on the wire")
    hrt = packet_time(HRT_PACKET)
    for label, frame_bytes in (("small inter frame", 2_800),
                               ("average frame", 5_300),
                               ("keyframe", 10_700)):
        chunks = max(1, -(-frame_bytes // HRT_CHUNK_DATA))
        tx = chunks * hrt
        print(f"  {label:<20} {frame_bytes:6d} B  {chunks} chunk(s)  "
              f"transmit {tx * 1e3:6.1f} ms")
    print("\n  Frame sizes are measured from a real 640x480 15 fps 600 kbit/s")
    print("  encode. Add one frame interval (66.7 ms at 15 fps) for the encoder")
    print("  to produce the frame, plus queueing if the ring is not empty.")


def measure_link(baud: int, target: int, de_gpio: int, port: str) -> None:
    section("Measured on the real UART")
    de = NullDeLine() if de_gpio < 0 else DeLine(gpio=de_gpio)
    link = Rs422Link(port=port, baud=baud, de=de)
    try:
        link.open()
    except Exception as exc:                            # noqa: BLE001
        print(f"  cannot open {port}: {exc}")
        print("  (radcamd holds the port; stop it first)")
        return

    wire = P.Wire(target_id=target)
    try:
        for label, packet in (
                ("Command ACK", P.encode_command_ack(wire, target)),
                ("LRT Data", P.encode_lrt_data(b"\x00" * 1248, wire, target)),
                ("HRT Data", P.encode_hrt_data(b"\x00" * 1280, wire, target))):
            samples = []
            for _ in range(10):
                start = time.perf_counter()
                link.send(packet)
                samples.append(time.perf_counter() - start)
            wire_s = len(packet) * char_time(baud)
            median = statistics.median(samples)
            print(f"  {label:<14} {len(packet):5d} B   wire {wire_s * 1e3:7.3f} ms"
                  f"   DE held {median * 1e3:7.3f} ms"
                  f"   overhead {(median - wire_s) * 1e6:6.1f} us")

        packet = P.encode_hrt_data(b"\x5a" * 1280, wire, target)
        count = 60
        start = time.perf_counter()
        for _ in range(count):
            link.send(packet)
        elapsed = time.perf_counter() - start
        print(f"\n  sustained HRT   {count / elapsed:6.1f} packets/s"
              f"   {count * HRT_CHUNK_DATA / elapsed / 1024:6.1f} kB/s payload"
              f"   {100 * count * len(packet) * 10 / elapsed / baud:5.1f}% of line rate")
    finally:
        link.close()


def measure_stream(seconds: float) -> None:
    section(f"Measured encoder output over {seconds:.0f} s")
    from radcam.stream import StreamConfig, VideoStream

    stream = VideoStream(StreamConfig())
    if not stream.available:
        print(f"  unavailable, missing: {', '.join(stream.missing())}")
        return
    print(f"  {stream.config.describe()}")
    if not stream.start():
        print(f"  failed to start: {stream.fault}")
        return

    frames, start = [], time.time()
    while time.time() - start < seconds:
        frame = stream.take()
        if frame is None:
            time.sleep(0.005)
            continue
        frames.append(frame)
    late, dropped = stream.encoder_late(), stream.frames_dropped
    stream.stop()

    if not frames:
        print("  no frames produced")
        return

    sizes = [len(f.data) for f in frames]
    span = frames[-1].produced - frames[0].produced
    keyframes = [f for f in frames if f.keyframe]
    hrt = packet_time(HRT_PACKET)
    chunks = [max(1, -(-s // HRT_CHUNK_DATA)) for s in sizes]

    print(f"\n  frames                {len(frames):8d}")
    print(f"  measured rate         {(len(frames) - 1) / span:8.2f} fps"
          f"   (configured {stream.config.fps})")
    print(f"  mean interval         {span / max(1, len(frames) - 1) * 1e3:8.2f} ms")
    print(f"  bitrate               {sum(sizes) * 8 / span / 1000:8.0f} kbit/s"
          f"   (configured {stream.config.bitrate // 1000})")
    print(f"  frame size  min/mean/max  {min(sizes)}/{sum(sizes) // len(sizes)}"
          f"/{max(sizes)} bytes")
    print(f"  keyframes             {len(keyframes):8d}"
          f"   one per {span / max(1, len(keyframes)):.2f} s")
    print(f"  dropped               {dropped:8d}")
    print(f"  encoder keeping up    {'no' if late else 'yes':>8}")
    print()
    print(f"  HRT chunks per frame  {statistics.mean(chunks):8.2f} mean, "
          f"{max(chunks)} worst")
    print(f"  transmit per frame    {statistics.mean(chunks) * hrt * 1e3:8.2f} ms"
          f" mean, {max(chunks) * hrt * 1e3:.2f} ms worst")
    budget = 1.0 / stream.config.fps
    mean_use = 100 * statistics.mean(chunks) * hrt / budget
    peak_use = 100 * max(chunks) * hrt / budget
    print(f"  frame interval        {budget * 1e3:8.2f} ms")
    print(f"  link occupancy        {mean_use:8.1f} %"
          f"   worst frame {peak_use:.1f} %")

    print()
    if mean_use < 100:
        print(f"  VERDICT: sustainable. Mean occupancy {mean_use:.0f}% leaves"
              f" {100 - mean_use:.0f}% for telemetry and commands.")
        print(f"  Keyframes burst to {peak_use:.0f}% of a frame interval -"
              f" roughly {max(chunks) * hrt / budget:.1f} frames' worth - which")
        print("  the frame ring absorbs and then catches up on, because the")
        print("  mean is below 100%. That is what the ring is for.")
        if mean_use > 90:
            print(f"\n  Margin is thin at {100 - mean_use:.0f}%. For more"
                  " headroom, drop to 10 fps (~84% at the same bitrate) or")
            print("  reduce the bitrate; both leave the picture usable.")
    else:
        print(f"  VERDICT: NOT sustainable. Mean occupancy {mean_use:.0f}%"
              " exceeds the link, so frames will be dropped continuously")
        print("  and the delivered bitrate will be below the configured one.")
        print("  Reduce the bitrate or the frame rate.")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--target", type=lambda v: int(v, 0), default=0xC7)
    ap.add_argument("--de-gpio", type=int, default=4)
    ap.add_argument("--link", action="store_true", help="measure the UART")
    ap.add_argument("--stream", type=float, nargs="?", const=10.0,
                    metavar="SECONDS", help="measure the encoder")
    ap.add_argument("--all", action="store_true")
    args = ap.parse_args()

    numbers = link_arithmetic(args.baud)
    transfer_table(numbers["payload_rate"])
    stream_budget(numbers["payload_rate"])
    latency_model()

    if args.link or args.all:
        measure_link(args.baud, args.target, args.de_gpio, args.port)
    if args.stream is not None or args.all:
        measure_stream(args.stream if args.stream is not None else 10.0)
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
