#!/usr/bin/env python3
"""Get the RS-422 link working, before any protocol is involved.

When a link is dead, the protocol is the wrong place to start looking. This
sends and receives the simplest thing that can travel down a wire, so the
question becomes "do bytes cross?" rather than "is my CRC right?".

Modes, roughly in the order you want them:

  --raw-ping     send plain ASCII lines. Open PuTTY or RealTerm on the host at
                 921600 8N1 and you should see readable text. No decoder, no
                 framing, no endianness - if this does not appear, nothing else
                 will.
  --raw-listen   dump whatever arrives, as hex and as text. Type into a
                 terminal on the host and watch it land here.
  --echo         echo every received byte straight back. Lets the host prove a
                 round trip on its own: type a character, see it return.
  --loopback     transmit a pattern and check it comes back. Needs the payload's
                 own driver pair jumpered to its own receiver pair (Y->A, Z->B).
                 This is the one test that proves the entire payload path -
                 UART, transceiver, DE, wiring - with no host involved at all.
  --ping         the same thing but as real STP packets, once ASCII works.

DE handling is the important option. With DE pulled up in hardware and no other
devices on the bus, the driver should simply stay enabled; software gating it
only adds a way to fail. `--de-gpio -1` (the default here) leaves the pin alone,
and `--de-release` actively turns it into an input so a hardware pull-up wins.

    sudo systemctl stop radcamd
    sudo tools/stp-pingpong.py --raw-ping --de-release
    sudo systemctl start radcamd
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import serial                                          # noqa: E402

from radcam.stp import packets as P                    # noqa: E402


def release_de(gpio: int) -> None:
    """Make the DE pin an input so a hardware pull-up decides the level.

    The pin is left as a driven output by normal operation, which overrides the
    pull-up. Turning it into an input hands control back to the hardware.
    """
    if gpio < 0:
        return
    os.system(f"pinctrl set {gpio} ip pu >/dev/null 2>&1")
    state = os.popen(f"pinctrl get {gpio}").read().strip()
    print(f"  DE released to input: {state}")


def hold_de(gpio: int, active_high: bool = True) -> None:
    """Drive the DE pin permanently enabled."""
    if gpio < 0:
        return
    os.system(f"pinctrl set {gpio} op {'dh' if active_high else 'dl'} >/dev/null 2>&1")
    print(f"  DE held enabled: {os.popen(f'pinctrl get {gpio}').read().strip()}")


def open_port(args) -> serial.Serial:
    handle = serial.Serial(args.port, args.baud, timeout=0.05)
    handle.reset_input_buffer()
    handle.reset_output_buffer()
    return handle


def printable(chunk: bytes) -> str:
    return "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)


def raw_ping(args) -> int:
    handle = open_port(args)
    print(f"\nSending plain ASCII on {args.port} at {args.baud} baud.")
    print("Open a terminal on the host at the same settings; you should see:\n")
    print("    RADCAM PING 00000001 ...\n")
    count = 0
    seen = bytearray()
    try:
        while args.seconds <= 0 or count < args.seconds / args.interval:
            count += 1
            line = f"RADCAM PING {count:08d} T={time.time():.3f}\r\n".encode()
            handle.write(line)
            handle.flush()
            back = handle.read(4096)
            if back:
                seen += back
            print(f"\r  sent {count:6d} lines, {count * len(line):8d} bytes"
                  + (f"   |  received {len(seen)} bytes back" if seen else ""),
                  end="", flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        handle.close()
    print()
    if seen:
        print(f"\n  Received {len(seen)} bytes while sending:")
        print(f"    hex : {bytes(seen[:64]).hex(' ').upper()}")
        print(f"    text: {printable(seen[:64])}")
        print("\n  Bytes came back. If they match what was sent, the pair is")
        print("  looped; if they are different, the host is replying.")
    else:
        print("\n  Nothing received while sending.")
    return 0


def raw_listen(args) -> int:
    handle = open_port(args)
    print(f"\nListening on {args.port} at {args.baud} baud. Send anything from")
    print("the host - typing in a terminal is enough.\n")
    total = bytearray()
    last = time.time()
    try:
        while args.seconds <= 0 or time.time() - last < args.seconds:
            chunk = handle.read(4096)
            if chunk:
                total += chunk
                print(f"  +{len(chunk):4d} B  {chunk[:32].hex(' ').upper()}"
                      f"   |{printable(chunk[:32])}|")
                last = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        handle.close()
    print(f"\n  total received: {len(total)} bytes")
    return 0 if total else 1


def echo(args) -> int:
    handle = open_port(args)
    print(f"\nEchoing every received byte back, on {args.port} at {args.baud}.")
    print("Type in a terminal on the host: each character should come back.\n")
    count = 0
    try:
        while True:
            chunk = handle.read(4096)
            if chunk:
                handle.write(chunk)
                handle.flush()
                count += len(chunk)
                print(f"  echoed {len(chunk):4d} B ({count} total)  "
                      f"|{printable(chunk[:48])}|")
    except KeyboardInterrupt:
        pass
    finally:
        handle.close()
    return 0


def loopback(args) -> int:
    """Prove the payload's own path, with its TX pair jumpered to its RX pair."""
    handle = open_port(args)
    print(f"\nLoopback test on {args.port} at {args.baud} baud.")
    print("Expects the payload's driver pair jumpered to its own receiver pair")
    print("(Y->A, Z->B). Nothing else is involved - no host, no protocol.\n")

    trials = passed = 0
    try:
        for size in (1, 8, 64, 256, 1024):
            for _ in range(args.repeat):
                trials += 1
                pattern = bytes((i * 7 + size) & 0xFF for i in range(size))
                handle.reset_input_buffer()
                handle.write(pattern)
                handle.flush()

                deadline = time.time() + 0.5 + size * 10.0 / args.baud
                back = bytearray()
                while time.time() < deadline and len(back) < size:
                    back += handle.read(size - len(back))

                ok = bytes(back) == pattern
                passed += ok
                if not ok:
                    print(f"  {size:5d} B  FAIL  got {len(back)} of {size}"
                          + (f", first mismatch at "
                             f"{next((i for i, (a, b) in enumerate(zip(back, pattern)) if a != b), len(back))}"
                             if back else ""))
                else:
                    print(f"  {size:5d} B  ok")
    except KeyboardInterrupt:
        pass
    finally:
        handle.close()

    print(f"\n  {passed}/{trials} loopback trials passed")
    if passed == trials:
        print("\n  The payload's entire transmit and receive path works:")
        print("  UART, transceiver, DE, and the wiring to the connector.")
        print("  Any remaining fault is between here and the host.")
    elif passed == 0:
        print("\n  Nothing came back. Either the jumper is not fitted, or the")
        print("  transceiver is not driving, or DE is disabled - try")
        print("  --de-release, or --de-hold to force it enabled.")
    return 0 if passed == trials else 1


def stp_ping(args) -> int:
    """The same idea, but as real protocol packets."""
    handle = open_port(args)
    wire = P.Wire(target_id=args.target)
    ack = P.encode_command_ack(wire, args.target)
    request = P.encode_short_request(P.PacketType.LRT_REQUEST, 0, 0, wire,
                                     args.target)
    print(f"\nSending STP packets as target 0x{args.target:02X}:")
    print(f"  8-byte ACK      {ack.hex().upper()}")
    print(f"  14-byte request {request.hex().upper()}\n")

    count = 0
    seen = bytearray()
    try:
        while args.seconds <= 0 or count < args.seconds / args.interval:
            count += 1
            handle.write(ack)
            handle.write(request)
            handle.flush()
            back = handle.read(4096)
            if back:
                seen += back
            print(f"\r  sent {count:6d} pairs"
                  + (f"   received {len(seen)} bytes" if seen else ""),
                  end="", flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        handle.close()
    print()
    if seen:
        sync = seen.count(bytes.fromhex("1acffc1d"))
        print(f"\n  received {len(seen)} bytes, {sync} sync patterns")
        print(f"    {bytes(seen[:48]).hex(' ').upper()}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--target", type=lambda v: int(v, 0), default=0xC7)
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--seconds", type=float, default=0,
                    help="0 = run until interrupted")
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--de-gpio", type=int, default=4)
    ap.add_argument("--de-release", action="store_true",
                    help="make DE an input so a hardware pull-up enables it")
    ap.add_argument("--de-hold", action="store_true",
                    help="drive DE permanently enabled")

    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--raw-ping", action="store_true")
    mode.add_argument("--raw-listen", action="store_true")
    mode.add_argument("--echo", action="store_true")
    mode.add_argument("--loopback", action="store_true")
    mode.add_argument("--ping", action="store_true")
    args = ap.parse_args()

    if args.de_release:
        release_de(args.de_gpio)
    elif args.de_hold:
        hold_de(args.de_gpio)

    if args.raw_ping:
        return raw_ping(args)
    if args.raw_listen:
        return raw_listen(args)
    if args.echo:
        return echo(args)
    if args.loopback:
        return loopback(args)
    return stp_ping(args)


if __name__ == "__main__":
    raise SystemExit(main())
