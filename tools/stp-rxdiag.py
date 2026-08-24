#!/usr/bin/env python3
"""Diagnose a silent RS-422 link, from the payload end.

When nothing is arriving there are only a few possibilities, and they are
distinguishable without an oscilloscope:

  1. Nothing is reaching the receiver at all      -> wiring, power, host not sending
  2. Bytes arrive but never form a valid packet   -> baud mismatch or inverted pair
  3. Packets arrive addressed to someone else     -> Target ID mismatch
  4. Packets arrive and are valid                 -> the link is fine; look higher up

This walks those in order and says which one it found. `radcamd` holds the port
exclusively, so stop it first:

    sudo systemctl stop radcamd
    sudo tools/stp-rxdiag.py --seconds 30
    sudo systemctl start radcamd

The baud scan is the useful part when the line is not silent: it listens at
each candidate rate and reports which one yields the sync pattern. A mismatch
produces bytes that look like noise at the wrong rate and clean packets at the
right one.
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

SYNC = bytes.fromhex("1acffc1d")
#: The little-endian-per-word form, which is what a host that got the byte
#: order wrong would emit.
SYNC_LE = bytes.fromhex("cf1a1dfc")

CANDIDATE_BAUDS = (921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600)

RX_LENGTHS = {0x10: 120, 0x81: 14, 0x85: 14, 0x86: 14, 0x87: 14}


def listen(port: str, baud: int, seconds: float) -> bytes:
    try:
        handle = serial.Serial(port, baud, timeout=0.05)
    except Exception as exc:                            # noqa: BLE001
        print(f"  cannot open {port} at {baud}: {exc}")
        return b""
    buf = bytearray()
    handle.reset_input_buffer()
    end = time.time() + seconds
    try:
        while time.time() < end:
            chunk = handle.read(8192)
            if chunk:
                buf += chunk
    finally:
        handle.close()
    return bytes(buf)


def describe(buf: bytes, target: int) -> dict:
    """Classify what arrived."""
    result = {"bytes": len(buf), "sync": buf.count(SYNC),
              "sync_le": buf.count(SYNC_LE), "packets": [], "for_us": 0,
              "for_others": 0, "bad_crc": 0}
    wire = P.Wire(target_id=target)

    index = 0
    while True:
        at = buf.find(SYNC, index)
        if at < 0:
            break
        index = at + 4
        if at + 12 > len(buf):
            break
        ptype = buf[at + 10]
        length = RX_LENGTHS.get(ptype)
        entry = {"offset": at, "type": ptype, "target": buf[at + 11],
                 "length": length}
        if length and at + length <= len(buf):
            packet = buf[at:at + length]
            entry["crc_ok"] = wire.check_crc(packet, length - 2)
            if not entry["crc_ok"]:
                result["bad_crc"] += 1
        if buf[at + 11] == target:
            result["for_us"] += 1
        else:
            result["for_others"] += 1
        result["packets"].append(entry)
    return result


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--target", type=lambda v: int(v, 0), default=0xC7)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--scan", action="store_true",
                    help="if silent or garbled, try other baud rates")
    ap.add_argument("--de-gpio", type=int, default=4,
                    help="hold DE inactive while listening; -1 to leave alone")
    args = ap.parse_args()

    print(f"\nListening on {args.port} at {args.baud} baud for "
          f"{args.seconds:.0f} s, expecting Target ID 0x{args.target:02X}.")

    # Hold DE inactive for the whole listen. If the transceiver shares a pair
    # with the host, leaving DE asserted would jam exactly what we are trying
    # to hear - and an unclaimed line's pull decides the level otherwise.
    de = None
    if args.de_gpio >= 0:
        try:
            import gpiod
            from gpiod.line import Direction, Value
            de = gpiod.request_lines(
                "/dev/gpiochip0", consumer="stp-rxdiag",
                config={args.de_gpio: gpiod.LineSettings(
                    direction=Direction.OUTPUT, output_value=Value.INACTIVE)})
            print(f"holding DE (GPIO{args.de_gpio}) inactive while listening.")
        except Exception as exc:                        # noqa: BLE001
            print(f"could not claim DE: {exc}")

    try:
        buf = listen(args.port, args.baud, args.seconds)
    finally:
        if de is not None:
            de.release()

    print(f"\nbytes received: {len(buf)}")

    if not buf:
        print("\n  DIAGNOSIS: nothing is reaching the receiver.")
        print("  Not a protocol problem - no bytes at all arrived.\n")
        print("  Check, in order:")
        print("    * the host is actually transmitting (scope or loopback at its end)")
        print("    * the host's TX pair reaches the payload's RX pair (A/B), not its TX")
        print("    * A and B are not swapped - a reversed pair reads as a stuck line")
        print("    * common ground between the two ends")
        print("    * the adapter is powered and its driver is enabled")
        if args.scan:
            print("\n  (a baud scan cannot help when no bytes arrive at all)")
        return 1

    info = describe(buf, args.target)
    print(f"  sync patterns found : {info['sync']}")
    hist = collections.Counter(buf)
    print(f"  distinct byte values: {len(hist)}")
    print(f"  most common bytes   : {hist.most_common(4)}")
    print(f"  first 48 bytes      : {buf[:48].hex(' ').upper()}")

    if info["sync"] == 0:
        print("\n  DIAGNOSIS: bytes are arriving, but no valid sync pattern.")
        if info["sync_le"]:
            print(f"  Found {info['sync_le']} occurrences of CF 1A 1D FC - the")
            print("  little-endian form. The host has the byte order wrong;")
            print("  the sync must go out as 1A CF FC 1D.")
            return 1
        if len(hist) < 8:
            print("  Very few distinct byte values: the line may be stuck or")
            print("  the pair inverted rather than carrying real data.")
        print("  Most likely a baud mismatch or an inverted differential pair.")
        if args.scan:
            print("\n  scanning other baud rates...")
            for baud in CANDIDATE_BAUDS:
                if baud == args.baud:
                    continue
                sample = listen(args.port, baud, min(4.0, args.seconds / 4))
                found = sample.count(SYNC)
                print(f"    {baud:>7} baud: {len(sample):6d} bytes, "
                      f"{found} sync" + ("   <-- MATCH" if found else ""))
                if found:
                    print(f"\n  The host is transmitting at {baud}, not "
                          f"{args.baud}. Fix one end.")
                    return 1
        else:
            print("  Re-run with --scan to try other rates.")
        return 1

    print(f"\n  packets addressed to 0x{args.target:02X} : {info['for_us']}")
    print(f"  packets for other targets     : {info['for_others']}")
    print(f"  packets failing CRC           : {info['bad_crc']}")

    for entry in info["packets"][:8]:
        name = {0x10: "COMMAND", 0x81: "LRT_REQUEST", 0x85: "HRT_STOP",
                0x86: "HRT_STOP_WITH_LOSS", 0x87: "HRT_GO"}.get(
                    entry["type"], f"unknown 0x{entry['type']:02X}")
        print(f"    @{entry['offset']:6d}  {name:<20} target 0x{entry['target']:02X}"
              + (f"  CRC {'ok' if entry.get('crc_ok') else 'BAD'}"
                 if "crc_ok" in entry else ""))

    if info["for_us"] == 0 and info["for_others"]:
        others = {e["target"] for e in info["packets"]}
        print(f"\n  DIAGNOSIS: traffic is arriving, but addressed to "
              f"{', '.join(f'0x{t:02X}' for t in sorted(others))}, not "
              f"0x{args.target:02X}.")
        print("  Either the host is using the wrong Target ID, or this payload's")
        print('  "target_id" in /etc/radcam/config.json is wrong.')
        return 1

    if info["bad_crc"]:
        print("\n  DIAGNOSIS: packets arrive but fail CRC. Check the host is")
        print("  using CRC-16/CCITT-FALSE over packet[4:-2], stored big-endian.")
        return 1

    print("\n  DIAGNOSIS: the link is working. Valid packets addressed to this")
    print("  payload are arriving and passing CRC. If the payload still seems")
    print("  silent, remember it only answers what it is asked:")
    print("    * LRT Data comes only in reply to an LRT Request (0x81)")
    print("    * HRT Data flows only between HRT Go (0x87) and a Stop")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
