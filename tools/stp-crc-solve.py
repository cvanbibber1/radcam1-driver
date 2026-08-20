#!/usr/bin/env python3
"""Recover the DICE CRC-16 parameters from captured traffic.

The ICD excerpts do not define the CRC polynomial, initial value, reflection,
final XOR, coverage range or stored byte order. The mission has stated
CRC-16/CCITT-FALSE, and that is the default everywhere in this codebase - but
"stated" is not "verified against the flight computer", and the cost of being
wrong is that nothing on the link works at all.

Point this at a capture of real DICE packets and it will report every parameter
set consistent with them. If exactly one survives several packets, that is the
answer. If CCITT-FALSE is among the survivors, the default is safe.

Input formats:

    --hex FILE      whitespace/newline separated hex bytes, one packet per line
    --bin FILE      raw bytes; packets are found by hunting the sync pattern
    --demo          generate packets with a deliberately non-default CRC and
                    prove the solver recovers it

Example:

    tools/stp-crc-solve.py --bin capture.bin
    tools/stp-crc-solve.py --hex packets.txt --crc-offset-from-end 2
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from radcam.stp.crc import CATALOG, Crc16Params, solve       # noqa: E402
from radcam.stp import packets as P                          # noqa: E402

#: Lengths this implementation knows. Used to carve packets out of a raw dump.
KNOWN_LENGTHS = (8, 14, 120, 1256, 1288)


def load_hex(path: str) -> list[bytes]:
    packets = []
    for line in open(path):
        line = line.strip().replace(",", " ").replace("0x", "")
        if not line or line.startswith("#"):
            continue
        try:
            packets.append(bytes.fromhex(line.replace(" ", "")))
        except ValueError:
            print(f"skipping unparseable line: {line[:40]}", file=sys.stderr)
    return packets


def carve(raw: bytes, sync: bytes) -> list[bytes]:
    """Split a raw dump into packets by hunting sync and trying known lengths."""
    packets, i = [], 0
    while True:
        at = raw.find(sync, i)
        if at < 0:
            break
        nxt = raw.find(sync, at + 4)
        span = (nxt - at) if nxt > 0 else (len(raw) - at)
        # Prefer an exact known length; otherwise take the gap to the next sync.
        length = next((n for n in KNOWN_LENGTHS if n == span), None)
        if length is None:
            length = max((n for n in KNOWN_LENGTHS if n <= span), default=0)
        if length:
            packets.append(raw[at:at + length])
        i = at + 4
    return packets


def demo() -> list[bytes]:
    """Packets built with CRC-16/X-25, little-endian store - not the default."""
    truth = next(c for c in CATALOG if c.name == "CRC-16/X-25")
    truth = Crc16Params(truth.name, truth.poly, truth.init, truth.reflect_in,
                        truth.reflect_out, truth.xor_out,
                        big_endian_store=False)
    wire = P.Wire(crc=truth, target_id=2)
    out = [
        P.encode_command(bytes(range(105)), 1_000_000, 123, wire, 2),
        P.encode_command(b"\xa5" * 105, 1_000_001, 456, wire, 2),
        P.encode_short_request(P.PacketType.LRT_REQUEST, 1_000_002, 789, wire, 2),
        P.encode_short_request(P.PacketType.HRT_GO, 1_000_003, 1011, wire, 2),
    ]
    print(f"demo: packets built with {truth.name}, "
          f"{'big' if truth.big_endian_store else 'little'}-endian store\n")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hex")
    ap.add_argument("--bin")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--crc-offset-from-end", type=int, default=2,
                    help="where the CRC sits relative to the packet end")
    ap.add_argument("--little-endian-sync", action="store_true",
                    help="hunt for the little-endian form of the sync pattern")
    args = ap.parse_args()

    if args.demo:
        packets = demo()
    elif args.hex:
        packets = load_hex(args.hex)
    elif args.bin:
        wire = P.Wire(big_endian=not args.little_endian_sync)
        packets = carve(open(args.bin, "rb").read(), wire.sync_bytes)
    else:
        ap.error("supply --hex, --bin or --demo")

    packets = [p for p in packets if len(p) > args.crc_offset_from_end + 4]
    if not packets:
        print("no usable packets found", file=sys.stderr)
        return 2

    print(f"{len(packets)} packet(s): "
          + ", ".join(str(len(p)) for p in packets[:8])
          + (" ..." if len(packets) > 8 else "") + " bytes\n")

    if len(packets) < 3:
        print("WARNING: fewer than 3 packets. A single sample matches a\n"
              "         wrong candidate about once in 65536 tries, and the\n"
              "         catalogue is large enough that coincidences happen.\n")

    candidates = solve(packets, crc_offset_from_end=args.crc_offset_from_end)

    if not candidates:
        print("No standard CRC-16 variant reproduces these packets.")
        print("Things worth trying:")
        print("  * a different --crc-offset-from-end (is the CRC really last?)")
        print("  * --little-endian-sync, if the words go out low byte first")
        print("  * packets may be misaligned; check the carve with a hex dump")
        return 1

    print(f"{len(candidates)} candidate(s) consistent with every packet:\n")
    for candidate in candidates:
        print("  " + candidate.describe())

    default_ok = any(c.params.name == "CRC-16/CCITT-FALSE"
                     and c.params.big_endian_store and c.start == 4
                     for c in candidates)
    print()
    if default_ok:
        print("The configured default (CCITT-FALSE, big-endian, from byte 4)\n"
              "is among the survivors - no config change needed.")
    else:
        best = candidates[0]
        print("The configured default is NOT consistent with this capture.\n"
              "Set in /etc/radcam/config.json under \"stp\":\n")
        print(f'    "crc_variant": "{best.params.name}",')
        print(f'    "crc_start": {best.start},')
        print(f'    "big_endian": {str(best.params.big_endian_store).lower()}')
    if len(candidates) > 1:
        print("\nSeveral survived: capture more packets, ideally with varied\n"
              "payload content, to separate them.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
