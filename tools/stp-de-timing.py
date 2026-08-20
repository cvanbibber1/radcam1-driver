#!/usr/bin/env python3
"""Measure how long DE is asserted, against how long the packet actually takes.

This is the check that matters most for bus courtesy. The DICE bus may carry up
to five other experiments, so every microsecond DE stays asserted past our last
stop bit is a microsecond somebody else cannot transmit. The number is not
guessable from the code - it depends on the UART driver - so it has to be
measured on the hardware it will fly on.

What good looks like on this board (Pi 5, PL011, 921600 baud):

    excess is flat at roughly 25-30 us regardless of packet size

If excess scales with packet size, or runs to milliseconds, the transmit path
has fallen back to `tcdrain()`. That is correct but coarse: it measured 8-13 ms
per packet here whatever the length, which is 140x the wire time of an 8-byte
ACK. Check the log line emitted at open for which release method is in use.

    sudo systemctl stop radcamd
    sudo tools/stp-de-timing.py --throughput
    sudo systemctl start radcamd

`radcamd` holds the flight port exclusively while it is running, so this will
report "Device or resource busy" until the service is stopped. That exclusivity
is deliberate: two processes writing the same UART would interleave packets on
a shared bus.
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from radcam.stp import packets as P                     # noqa: E402
from radcam.stp.link import DeLine, NullDeLine, Rs422Link   # noqa: E402

#: Worst tolerable overshoot past the end of transmission, in microseconds.
EXCESS_LIMIT_US = 250.0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--de-gpio", type=int, default=4)
    ap.add_argument("--no-de", action="store_true",
                    help="do not drive DE (only safe if nothing shares the bus)")
    ap.add_argument("-n", "--repeats", type=int, default=10)
    ap.add_argument("--throughput", action="store_true",
                    help="also measure sustained HRT rate")
    args = ap.parse_args()

    wire = P.Wire(target_id=1)
    de = NullDeLine() if args.no_de else DeLine(gpio=args.de_gpio)
    link = Rs422Link(port=args.port, baud=args.baud, de=de)

    try:
        link.open()
    except Exception as exc:                            # noqa: BLE001
        print(f"cannot open {args.port}: {exc}", file=sys.stderr)
        return 2

    packets = [
        ("Command ACK", P.encode_command_ack(wire, 1)),
        ("Command-size", b"\x00" * P.COMMAND_PACKET_SIZE),
        ("LRT Data", P.encode_lrt_data(b"\x00" * 1248, wire, 1)),
        ("HRT Data", P.encode_hrt_data(b"\x00" * 1280, wire, 1)),
    ]

    print(f"\n{args.port} at {args.baud} baud, "
          f"char time {link.char_time_s * 1e6:.2f} us, "
          f"DE on GPIO{args.de_gpio if not args.no_de else '-'}\n")
    print(f"  {'packet':14s} {'bytes':>6s} {'wire':>11s} "
          f"{'DE held':>11s} {'excess':>10s} {'verdict':>8s}")

    worst = 0.0
    try:
        for name, packet in packets:
            samples = []
            for _ in range(args.repeats):
                start = time.perf_counter()
                link.send(packet)
                samples.append(time.perf_counter() - start)
            wire_s = len(packet) * link.char_time_s
            median = statistics.median(samples)
            excess_us = (median - wire_s) * 1e6
            worst = max(worst, excess_us)
            ok = excess_us <= EXCESS_LIMIT_US
            print(f"  {name:14s} {len(packet):6d} {wire_s * 1e6:10.1f}us "
                  f"{median * 1e6:10.1f}us {excess_us:9.1f}us "
                  f"{'ok' if ok else 'HIGH':>8s}")

        if args.throughput:
            packet = P.encode_hrt_data(b"\x5a" * 1280, wire, 1)
            count = 40
            start = time.perf_counter()
            for _ in range(count):
                link.send(packet)
            elapsed = time.perf_counter() - start
            payload_rate = count * 1280 / elapsed
            wire_rate = count * len(packet) * 10 / elapsed
            print(f"\n  sustained HRT: {count / elapsed:.1f} packets/s, "
                  f"{payload_rate / 1024:.1f} kB/s of payload, "
                  f"{wire_rate / 1000:.0f} kbit/s on the wire "
                  f"({100 * wire_rate / args.baud:.1f}% utilisation)")

        stats = link.stats()
        if stats.get("tx_drain_timeouts"):
            print(f"\n  WARNING: {stats['tx_drain_timeouts']} transmission(s) "
                  f"did not report transmitter-empty in time")
    finally:
        link.close()

    print(f"\n  worst excess {worst:.1f} us "
          f"(limit {EXCESS_LIMIT_US:.0f} us) -> "
          f"{'PASS' if worst <= EXCESS_LIMIT_US else 'FAIL'}\n")
    return 0 if worst <= EXCESS_LIMIT_US else 1


if __name__ == "__main__":
    raise SystemExit(main())
