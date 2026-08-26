#!/usr/bin/env python3
"""Receive-only monitor: log everything that arrives, with timestamps.

Built for the case where the two ends cannot be made to transmit and listen at
the same moment. It listens continuously, transmits nothing at all, and records
the instant anything appears - so the far end can send whenever it likes and
the evidence is waiting afterwards.

Every byte is written to a raw capture before any parsing, and each burst is
logged with its arrival time, size, and a decode attempt. A burst that is not
valid STP is still recorded in full, because "12 bytes arrived that were not a
packet" and "nothing arrived" call for completely different next steps.

    sudo systemctl stop radcamd
    sudo tools/stp-rxmonitor.py --minutes 30 --out /var/log/radcam/rx.bin
"""

from __future__ import annotations

import argparse
import collections
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import serial                                          # noqa: E402

from radcam.stp import packets as P                    # noqa: E402

RX_LENGTHS = {0x10: 120, 0x81: 14, 0x85: 14, 0x86: 14, 0x87: 14}
NAMES = {0x10: "COMMAND", 0x81: "LRT_REQUEST", 0x85: "HRT_STOP",
         0x86: "HRT_STOP_WITH_LOSS", 0x87: "HRT_GO"}


def classify(burst: bytes, wire: P.Wire) -> str:
    """Say what a burst is, in one line."""
    sync = wire.sync_bytes
    if sync not in burst:
        counts = collections.Counter(burst)
        common = counts.most_common(1)[0]
        return (f"no sync; {len(set(burst))} distinct values, "
                f"most common 0x{common[0]:02X} x{common[1]}")

    out = []
    index = 0
    while len(out) < 6:
        at = burst.find(sync, index)
        if at < 0 or at + 12 > len(burst):
            break
        index = at + 4
        ptype, target = burst[at + 10], burst[at + 11]
        length = RX_LENGTHS.get(ptype)
        crc = "?"
        if length and at + length <= len(burst):
            crc = "ok" if wire.check_crc(burst[at:at + length], length - 2) \
                else "BAD"
        out.append(f"{NAMES.get(ptype, hex(ptype))} target=0x{target:02X} CRC={crc}")
    return "; ".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--target", type=lambda v: int(v, 0), default=0xC7)
    ap.add_argument("--minutes", type=float, default=30.0)
    ap.add_argument("--out", default="")
    ap.add_argument("--gap", type=float, default=0.05,
                    help="idle seconds that end one burst")
    args = ap.parse_args()

    wire = P.Wire(target_id=args.target)
    handle = serial.Serial(args.port, args.baud, timeout=0.02)
    handle.reset_input_buffer()
    capture = open(args.out, "ab", buffering=0) if args.out else None

    print(f"\nReceive-only on {args.port} at {args.baud} 8N1 for "
          f"{args.minutes:.0f} min. Transmitting nothing.")
    print(f"Target for decode: 0x{args.target:02X}"
          + (f"   raw capture -> {args.out}" if capture else ""))
    print("Send from the host at any point; it will be recorded.\n", flush=True)

    end = time.time() + args.minutes * 60
    total = bursts = 0
    pending = bytearray()
    last_rx = 0.0
    started = time.time()

    try:
        while time.time() < end:
            chunk = handle.read(8192)
            now = time.time()
            if chunk:
                if capture:
                    capture.write(chunk)
                pending += chunk
                total += len(chunk)
                last_rx = now
            elif pending and now - last_rx >= args.gap:
                bursts += 1
                burst = bytes(pending)
                pending.clear()
                print(f"[t+{now-started:8.2f}s] BURST {bursts}: "
                      f"{len(burst)} bytes", flush=True)
                print(f"    hex : {burst[:48].hex(' ').upper()}"
                      + (" ..." if len(burst) > 48 else ""), flush=True)
                print(f"    what: {classify(burst, wire)}\n", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if pending:
            print(f"[final] {len(pending)} bytes: "
                  f"{bytes(pending[:48]).hex(' ').upper()}")
            print(f"    what: {classify(bytes(pending), wire)}")
        handle.close()
        if capture:
            capture.close()

    print(f"\ntotal: {total} bytes in {bursts} bursts over "
          f"{(time.time()-started)/60:.1f} min")
    if total == 0:
        print("Nothing arrived at any point in the window.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
