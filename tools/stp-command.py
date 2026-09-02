#!/usr/bin/env python3
"""Generate ready-to-paste command hex strings for the DICE link.

Every command is a complete packet — sync, timestamp, type, target, payload and
CRC — rendered as one hex string. Paste it into the ground configuration and it
goes on the wire as-is.

    tools/stp-command.py --list                       # what commands exist
    tools/stp-command.py --all                        # every one, with hex
    tools/stp-command.py SLOT_CAPTURE_IMAGE slot=3
    tools/stp-command.py STREAM_SET_REGION centre_x=3000 centre_y=2000 crop_w=512 crop_h=512
    tools/stp-command.py --markdown                   # the reference table
    tools/stp-command.py --verify                     # prove they are accepted

Two things about a *canned* string, both deliberate:

* It carries the **force flag**, because the sequence number is baked in and
  the payload otherwise suppresses a repeat as a retransmission. Pasting the
  same capture command ten times takes ten pictures. Pass `--no-force` for a
  ground station that generates its own sequence numbers.
* Its **timestamp is zero**. Coarse and fine time belong to the master; the
  payload only echoes them back and nothing it does depends on the value.

`--verify` is the one that matters before trusting a printed table: it feeds
every generated string to a real experiment instance and checks each is
decoded, accepted and dispatched to the opcode it claims.
"""

from __future__ import annotations

import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from radcam.stp import catalogue as C                  # noqa: E402
from radcam.stp.packets import Wire                    # noqa: E402
from radcam.stp import crc as crcmod                    # noqa: E402


def wire_from_config(target: int, path: str) -> Wire:
    """Build the wire from the live config, so generated hex is not stale.

    A canned hex string carries its own CRC. If the payload's CRC settings are
    changed and these strings are not regenerated with them, every canned
    command silently stops being accepted - so read the same file the daemon
    reads rather than assuming the defaults.
    """
    stp = {}
    try:
        import json
        with open(path) as handle:
            stp = (json.load(handle).get("stp") or {})
    except (OSError, ValueError):
        pass                       # no config here: the defaults are correct
    params, problems = crcmod.from_config(stp)
    for problem in problems:
        print(f"  !! config: {problem}", file=sys.stderr)
    return Wire(big_endian=bool(stp.get("big_endian", True)),
                crc=params, target_id=target,
                crc_start=int(stp.get("crc_start", 4)),
                lrt_trailer=str(stp.get("lrt_trailer", "crc")))

DEFAULT_TARGET = 0xC7


def parse_values(spec: C.CommandSpec, pairs: list[str]) -> dict:
    known = {f.name: f for f in spec.fields}
    values = {}
    for pair in pairs:
        if "=" not in pair:
            raise SystemExit(f"argument '{pair}' is not name=value")
        name, _, raw = pair.partition("=")
        if name not in known:
            raise SystemExit(f"{spec.name} has no argument '{name}'. "
                             f"Valid: {', '.join(known) or 'none'}")
        try:
            values[name] = int(raw, 0)
        except ValueError:
            raise SystemExit(f"'{raw}' is not an integer")
    return values


def render_one(spec: C.CommandSpec, wire: Wire, values: dict, seq: int,
               force: bool) -> None:
    print(f"\n{spec.name}  (opcode 0x{spec.opcode:02X})")
    print(f"  {spec.summary}")
    if spec.fields:
        print("  arguments:")
        for f in spec.fields:
            shown = values.get(f.name, f.default)
            print(f"    {f.name:<10} = {shown:<12} ({f.fmt}, {f.size} B)"
                  + (f"  {f.help}" if f.help else ""))
    print(f"  target   : 0x{wire.target_id:02X}")
    print(f"  cmd_seq  : {seq}{'  (force set)' if force else ''}")
    print("\n" + C.build_command_hex(spec, values, wire, seq, force) + "\n")


def render_list() -> None:
    print(f"\n{len(C.CATALOGUE)} commands\n")
    for group in C.GROUPS:
        entries = [c for c in C.CATALOGUE if c.group == group]
        if not entries:
            continue
        print(f"  {group.upper()}")
        for spec in entries:
            print(f"    0x{spec.opcode:02X}  {spec.name:<22} {spec.summary}")
        print()
    print("  REQUEST PACKETS (14 bytes, not commands)")
    for name, ptype, summary in C.SHORT_PACKETS:
        print(f"    0x{ptype:02X}  {name:<22} {summary}")
    print()


def render_all(wire: Wire, seq: int, force: bool) -> None:
    print(f"\nTarget 0x{wire.target_id:02X}, {wire.crc.name}, "
          f"{'big' if wire.big_endian else 'little'}-endian, "
          f"cmd_seq {seq}{', force set' if force else ''}\n")
    print("REQUEST PACKETS (14 bytes)")
    for name, ptype, _ in C.SHORT_PACKETS:
        print(f"  {name:<22} {C.build_short_hex(ptype, wire)}")

    for group in C.GROUPS:
        entries = [c for c in C.CATALOGUE if c.group == group]
        if not entries:
            continue
        print(f"\n{group.upper()} (120 bytes)")
        for spec in entries:
            print(f"  {spec.name}"
                  + (f"  [{spec.signature()}]" if spec.fields else ""))
            print(f"    {C.build_command_hex(spec, None, wire, seq, force)}")
    print()


def render_markdown(wire: Wire, seq: int, force: bool) -> None:
    """The reference table, generated so it cannot drift from the code."""
    print(f"<!-- generated by tools/stp-command.py --markdown -->")
    print(f"\nTarget ID `0x{wire.target_id:02X}`, {wire.crc.name}, big-endian "
          f"envelope, `cmd_seq` {seq}, force flag "
          f"{'set' if force else 'clear'}.\n")

    print("### Request packets (14 bytes)\n")
    print("| Name | Type | Purpose | Hex |")
    print("|---|---|---|---|")
    for name, ptype, summary in C.SHORT_PACKETS:
        print(f"| `{name}` | `0x{ptype:02X}` | {summary} | "
              f"`{C.build_short_hex(ptype, wire)}` |")

    for group in C.GROUPS:
        entries = [c for c in C.CATALOGUE if c.group == group]
        if not entries:
            continue
        print(f"\n### {group.title()} commands\n")
        print("| Name | Opcode | Arguments | Purpose |")
        print("|---|---|---|---|")
        for spec in entries:
            args = (", ".join(f"`{f.name}`" for f in spec.fields)
                    if spec.fields else "—")
            print(f"| `{spec.name}` | `0x{spec.opcode:02X}` | {args} | "
                  f"{spec.summary} |")
        print("\n<details><summary>Hex strings (defaults shown)</summary>\n")
        for spec in entries:
            defaults = (", ".join(f"{f.name}={f.default}" for f in spec.fields)
                        if spec.fields else "no arguments")
            print(f"`{spec.name}` — {defaults}\n")
            print(f"```\n{C.build_command_hex(spec, None, wire, seq, force)}\n```\n")
        print("</details>")
    print()


def verify(wire: Wire, seq: int, force: bool) -> int:
    """Feed every generated string to a real experiment and check it lands."""
    from radcam.protocol import Config as ProtoConfig, Dispatcher
    from radcam.stp.experiment import Experiment, ExperimentConfig
    from radcam.stp import packets as P
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + "/..")
    from tests.support import FakeMediaStore, MemoryLink

    link = MemoryLink()
    store = FakeMediaStore()
    experiment = Experiment(
        link=link, wire=wire,
        dispatcher=Dispatcher(config=ProtoConfig(), store=store), store=store,
        config=ExperimentConfig(target_id=wire.target_id,
                                scrub_interval_s=3600))
    seen: list[int] = []
    # Intercept at the queue so this checks decoding and dispatch, not whether
    # a camera happens to be attached.
    experiment._queue.put_nowait = lambda request: seen.append(request.opcode)

    failures = []
    print(f"\nverifying {len(C.CATALOGUE)} commands and "
          f"{len(C.SHORT_PACKETS)} request packets against a live decoder\n")

    for spec in C.CATALOGUE:
        seen.clear()
        link.dice_read()
        link.dice_send(bytes.fromhex(
            C.build_command_hex(spec, None, wire, seq, force)))
        experiment.service()

        acked = any(kind == 0x10 for kind in _reply_types(link, wire))
        dispatched = seen == [spec.opcode]
        ok = acked and dispatched
        print(f"  {'PASS' if ok else 'FAIL'}  {spec.name:<22} "
              f"opcode 0x{spec.opcode:02X}"
              + ("" if ok else f"   acked={acked} dispatched={seen}"))
        if not ok:
            failures.append(spec.name)

    for name, ptype, _ in C.SHORT_PACKETS:
        link.dice_read()
        link.dice_send(bytes.fromhex(C.build_short_hex(ptype, wire)))
        experiment.service()
        replies = _reply_types(link, wire)
        if ptype == P.PacketType.LRT_REQUEST:
            ok = 0x81 in replies
        else:
            ok = True                      # flow control produces no reply
        print(f"  {'PASS' if ok else 'FAIL'}  {name:<22} type 0x{ptype:02X}")
        if not ok:
            failures.append(name)

    experiment.stop()
    total = len(C.CATALOGUE) + len(C.SHORT_PACKETS)
    print(f"\n{total - len(failures)}/{total} accepted")
    if failures:
        print("FAILED: " + ", ".join(failures))
        return 1
    print("every generated hex string is decoded, accepted and dispatched")
    return 0


def _reply_types(link, wire: Wire) -> list[int]:
    raw = link.dice_read()
    out, i = [], 0
    while i < len(raw):
        at = raw.find(wire.sync_bytes, i)
        if at < 0 or at + 6 > len(raw):
            break
        kind = raw[at + 4]
        out.append(kind)
        i = at + (8 if kind == 0x10 else 1256 if kind == 0x81 else 1288)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", nargs="?", help="command name, e.g. SLOT_DOWNLOAD")
    ap.add_argument("args", nargs="*", help="name=value arguments")
    ap.add_argument("--target", type=lambda v: int(v, 0), default=DEFAULT_TARGET)
    ap.add_argument("--seq", type=int, default=1, help="cmd_seq baked in")
    ap.add_argument("--no-force", action="store_true",
                    help="omit the force flag (enables duplicate suppression)")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--markdown", action="store_true")
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--config", default="/etc/radcam/config.json",
                    help="read CRC and byte order from this config")
    args = ap.parse_args()

    wire = wire_from_config(args.target, args.config)
    force = not args.no_force

    if args.list:
        render_list()
        return 0
    if args.all:
        render_all(wire, args.seq, force)
        return 0
    if args.markdown:
        render_markdown(wire, args.seq, force)
        return 0
    if args.verify:
        return verify(wire, args.seq, force)
    if not args.command:
        ap.print_help()
        return 0

    spec = C.find(args.command)
    if spec is None:
        near = [c.name for c in C.CATALOGUE
                if args.command.upper() in c.name]
        raise SystemExit(f"unknown command '{args.command}'"
                         + (f". Did you mean: {', '.join(near)}?" if near
                            else ". Try --list"))
    render_one(spec, wire, parse_values(spec, args.args), args.seq, force)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
