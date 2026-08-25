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


#: Patterns chosen for what they look like on an oscilloscope, not for meaning.
#: Descending, because the question is how far down you must go before the
#: link works. 9600 forgives almost any wiring fault that still connects.
SWEEP_BAUDS = (921600, 460800, 230400, 115200, 57600, 19200, 9600)

SCOPE_PATTERNS = {
    # 8N1 sends LSB first, so 0x55 = 0101 0101 alternates every bit and gives a
    # clean square wave at half the bit rate. The easiest way to measure baud.
    "55": (0x55, "square wave at half the bit rate; period = 2 bit times"),
    "AA": (0xAA, "as 0x55 but inverted phase"),
    # Start bit plus eight zeros = nine consecutive low bit times, then the
    # stop bit. The widest low pulse the format can produce.
    "00": (0x00, "9 bit times low, then 1 high; widest low pulse possible"),
    # Only the start bit is low, so a narrow negative-going pulse per byte.
    "FF": (0xFF, "1 bit time low per byte; narrow pulses on an idle-high line"),
}


def scope(args) -> int:
    """Transmit a continuous pattern for probing with an oscilloscope.

    This is for the case where the link is dead and nothing electronic on
    either end can tell you why. A repeating byte gives a signal whose timing
    and amplitude can be measured directly, so the questions become concrete:
    is the driver switching at all, at what rate, and with what swing?
    """
    handle = open_port(args)
    byte, description = SCOPE_PATTERNS[args.pattern.upper().replace("0X", "")]
    bit_us = 1e6 / args.baud
    block = bytes([byte]) * 256

    print(f"\nTransmitting 0x{byte:02X} continuously on {args.port} "
          f"at {args.baud} baud.")
    print(f"  {description}")
    print()
    print("  What to expect on a scope:")
    print(f"    bit period            {bit_us:.3f} us")
    print(f"    byte period (10 bits) {10 * bit_us:.3f} us")
    if byte == 0x55:
        print(f"    square wave period    {2 * bit_us:.3f} us "
              f"({args.baud / 2 / 1000:.1f} kHz)")
    print()
    print("  Probe points, in order of what they rule out:")
    print("    1. GPIO14 (pin 8) - the Pi's TXD. Activity here proves the UART")
    print("       is transmitting and the pin mux is right.")
    print("    2. The transceiver's DI input - proves the signal reaches it.")
    print("    3. Y and Z, single-ended to ground - proves the driver switches.")
    print("    4. Y minus Z, differential - should swing at least +/-2 V into")
    print("       120 ohms. Much less means the driver is not enabled, or the")
    print("       isolated supply is not running.")
    print("    5. DE - should sit high the whole time, not pulse.")
    print()
    print("  Ctrl-C to stop.\n")

    # No flush between blocks. tcdrain waits for the FIFO to empty, which puts
    # an idle gap between every block and turns a continuous carrier into a
    # burst - measured at a third of line rate. Writing without draining keeps
    # the transmitter saturated, which is what makes a clean scope trace.
    sent = 0
    start = time.time()
    try:
        while args.seconds <= 0 or time.time() - start < args.seconds:
            handle.write(block)
            sent += len(block)
            elapsed = time.time() - start
            if sent % (256 * 40) == 0:
                print(f"\r  {elapsed:7.1f}s   {sent:10d} bytes   "
                      f"{sent * 10 / max(elapsed, 1e-9) / 1000:7.1f} kbit/s",
                      end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        handle.close()
    print(f"\n\n  stopped after {sent} bytes")
    return 0


def pingpong(args) -> int:
    """Transmit continuously and report anything that comes back.

    Sends three things each cycle so that whatever the far end is prepared to
    understand, something lands: a plain ASCII line a dumb terminal will show,
    an 8-byte Command ACK, and a full 1256-byte LRT Data packet. Everything
    received is classified rather than merely counted, because "12 bytes
    arrived" and "12 bytes arrived that are a valid LRT request for 0xC7" call
    for completely different next steps.
    """
    from radcam.stp import lrt as L

    handle = open_port(args)
    wire = P.Wire(target_id=args.target)
    ack = P.encode_command_ack(wire, args.target)
    lrt = P.encode_lrt_data(
        L.build_lrt_payload({"target_id": args.target, "fw_major": 1,
                             "dose_rad": 0.0731, "slot_count": 16}),
        wire, args.target)

    print(f"\nBidirectional ping on {args.port} at {args.baud} baud, "
          f"target 0x{args.target:02X}.")
    print("Each cycle sends: ASCII line, 8-byte ACK, 1256-byte LRT Data.")
    print("Anything received is decoded and reported.\n")

    cycles, reads = 0, 0
    rx = bytearray()
    start = time.time()
    try:
        while args.seconds <= 0 or time.time() - start < args.seconds:
            cycles += 1
            handle.write(f"RADCAM PING {cycles:06d}\r\n".encode())
            handle.write(ack)
            handle.write(lrt)
            handle.flush()

            deadline = time.time() + args.interval
            while time.time() < deadline:
                chunk = handle.read(4096)
                if chunk:
                    rx += chunk
                    reads += 1
                else:
                    time.sleep(0.005)
            print(f"  {time.time()-start:6.1f}s  TX {cycles:5d} cycles   "
                  f"RX {len(rx):7d} B in {reads} reads", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        handle.close()

    print()
    return _report_rx(bytes(rx), wire, bursts=cycles * 3)


def _report_rx(rx: bytes, wire, bursts: int = 0) -> int:
    """Say what arrived, and what it means."""
    if not rx:
        print("  Nothing was received.")
        print("\n  If the host sees the ping, transmit is proven and the")
        print("  return path - host TX to payload RX - carries nothing.")
        return 1

    print(f"  Received {len(rx)} bytes.")
    print(f"    first 64 hex : {rx[:64].hex(' ').upper()}")
    print(f"    as text      : {printable(rx[:64])}")
    sync = rx.count(bytes.fromhex("1acffc1d"))
    print(f"    sync patterns: {sync}")

    if b"RADCAM PING" in rx:
        print("\n  This contains our OWN ascii ping: the transmit pair is")
        print("  looped into the receive pair. That proves the entire payload")
        print("  path - UART, transceiver, DE, wiring - but it is not the host.")
        return 0

    if sync:
        lengths = {0x10: 120, 0x81: 14, 0x85: 14, 0x86: 14, 0x87: 14}
        names = {0x10: "COMMAND", 0x81: "LRT_REQUEST", 0x85: "HRT_STOP",
                 0x86: "HRT_STOP_LOSS", 0x87: "HRT_GO"}
        index = shown = 0
        while shown < 6:
            at = rx.find(bytes.fromhex("1acffc1d"), index)
            if at < 0 or at + 12 > len(rx):
                break
            index, shown = at + 4, shown + 1
            ptype, target = rx[at + 10], rx[at + 11]
            size = lengths.get(ptype)
            crc = ""
            if size and at + size <= len(rx):
                crc = ("  CRC ok" if wire.check_crc(rx[at:at + size], size - 2)
                       else "  CRC BAD")
            print(f"      @{at:6d}  {names.get(ptype, hex(ptype)):<14} "
                  f"target 0x{target:02X}{crc}")
        print("\n  Valid STP framing received: the link works both ways.")
        return 0

    unique = len(set(rx))
    print(f"    distinct byte values: {unique}")

    # The discriminator that matters is not how many distinct values arrived,
    # but whether the count tracks the number of transmit *bursts* rather than
    # the number of bytes sent. One received byte per burst, whatever the burst
    # size, is the driver's switching edge coupling into the receive pair - it
    # cannot be data, because data would scale with length.
    if bursts and abs(len(rx) - bursts) <= max(3, bursts * 0.25):
        print(f"\n  {len(rx)} bytes received for {bursts} transmit bursts -")
        print("  almost exactly one per burst, independent of burst size.")
        print("  That is our own driver switching, coupling into the receive")
        print("  pair. It is not data, and it confirms two things: the")
        print("  transmitter is driving the line, and the receive pair carries")
        print("  nothing from the host.")
    elif unique <= 3:
        print("\n  Very few distinct values - likely coupling rather than data.")
    else:
        print("\n  Bytes arrive but do not frame as STP: suspect a baud")
        print("  mismatch or an inverted pair on the return path.")
    return 1


def tx_sweep(args) -> int:
    """Transmit an identifiable pattern at each baud rate in turn.

    A link that fails at 921600 may work perfectly at 9600. The bit period goes
    from 1.085 us to 104 us, which forgives an enormous amount: unterminated
    stubs, reflections, marginal connections, slew-rate limits, a transceiver
    running out of drive. When a link will not come up at speed, the fastest
    way to learn whether it works *at all* is to slow it down until it does.

    Each rate announces itself in ASCII at that rate, so a host parked on one
    rate sees readable text only while the sweep is passing through it.
    """
    print(f"\nSweeping transmit baud rates on {args.port}, "
          f"{args.dwell:.0f} s at each.\n")
    print("  Set the host to ONE rate and watch. When readable text appears")
    print("  naming that rate, the link works there.\n")

    for baud in SWEEP_BAUDS:
        try:
            handle = serial.Serial(args.port, baud, timeout=0.05)
        except Exception as exc:                        # noqa: BLE001
            print(f"  {baud:>7} baud: cannot open ({exc})")
            continue

        bit_us = 1e6 / baud
        print(f"  {baud:>7} baud  (bit {bit_us:7.2f} us) ... ",
              end="", flush=True)

        line = (f"=== RADCAM AT {baud} BAUD === "
                f"bit={bit_us:.2f}us ===\r\n").encode()
        sent = 0
        received = bytearray()
        end = time.time() + args.dwell
        try:
            while time.time() < end:
                handle.write(line)
                sent += len(line)
                chunk = handle.read(1024)
                if chunk:
                    received += chunk
                time.sleep(0.05)
            handle.flush()
        finally:
            handle.close()
        print(f"sent {sent:6d} B, received {len(received):5d} B"
              + (f"  <-- {printable(bytes(received[:24]))}" if received else ""))

    print("\n  If one rate produced readable text on the host, use it:")
    print("  change the stp \"baud\" in /etc/radcam/config.json and set the")
    print("  host to match. A working link at 9600 is worth more than a broken")
    print("  one at 921600; speed can come afterwards.")
    return 0


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
    mode.add_argument("--scope", action="store_true",
                      help="continuous pattern for oscilloscope probing")
    mode.add_argument("--pingpong", action="store_true",
                      help="transmit continuously and decode anything received")
    mode.add_argument("--tx-sweep", action="store_true",
                      help="transmit at each baud rate in turn, slowest last")
    ap.add_argument("--dwell", type=float, default=10.0,
                    help="seconds at each rate in --tx-sweep")
    ap.add_argument("--pattern", default="55",
                    choices=["55", "AA", "00", "FF"],
                    help="byte to repeat in --scope mode")
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
    if args.scope:
        return scope(args)
    if args.pingpong:
        return pingpong(args)
    if args.tx_sweep:
        return tx_sweep(args)
    return stp_ping(args)


if __name__ == "__main__":
    raise SystemExit(main())
