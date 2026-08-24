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

    started = time.time()
    interval = 1.0 / max(args.lrt_hz, 0.01)
    next_lrt = time.time()
    counts = {"LRT": 0, "HRT": 0, "ACK": 0}
    chunk = 0

    try:
        while time.time() - started < args.seconds:
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
