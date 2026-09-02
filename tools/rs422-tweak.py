#!/usr/bin/env python3
"""Inspect and change the RS-422 wire settings without editing JSON by hand.

The CRC, the target id, the baud rate and the CRC coverage window are the
settings that decide whether the other end accepts a packet at all. Getting one
wrong produces no error here: we transmit perfectly and the flight computer
discards everything. So they need to be quick to change, quick to read back,
and hard to typo - which is what this tool is for.

    tools/rs422-tweak.py show                 # what is configured now
    tools/rs422-tweak.py list-crc             # every CRC variant by name
    tools/rs422-tweak.py set crc=CRC-16/XMODEM --restart
    tools/rs422-tweak.py set crc=custom crc_poly=0x8005 crc_init=0 --restart
    tools/rs422-tweak.py crc 01 02 03         # CRC of some bytes, as configured
    tools/rs422-tweak.py sample               # real packets under these settings
    tools/rs422-tweak.py diff                 # what differs from the defaults

Every write backs the file up first and validates before restarting anything.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from radcam.stp import crc as crcmod                    # noqa: E402
from radcam.stp import packets as P                     # noqa: E402

CONFIG = "/etc/radcam/config.json"
BACKUPS = "/home/rad/driver-dev/logs"

#: Setting name as typed -> key in the `stp` config block. The short names on
#: the left are what an operator says out loud; the long ones on the right are
#: what the daemon reads.
KEYS = {
    "crc": "crc_variant",
    "crc_variant": "crc_variant",
    "crc_poly": "crc_poly",
    "crc_init": "crc_init",
    "crc_reflect_in": "crc_reflect_in",
    "crc_reflect_out": "crc_reflect_out",
    "crc_xor_out": "crc_xor_out",
    "crc_store": "crc_store",
    "crc_start": "crc_start",
    "target": "target_id",
    "target_id": "target_id",
    "baud": "baud",
    "port": "port",
    "big_endian": "big_endian",
    "lrt_trailer": "lrt_trailer",
    "de_control": "de_control",
    "de_gpio": "de_gpio",
    "de_active_high": "de_active_high",
    "hrt_packets_per_service": "hrt_packets_per_service",
    "log_rx": "log_rx",
}

_BOOL = {"crc_reflect_in", "crc_reflect_out", "big_endian", "de_control",
         "de_active_high", "log_rx"}
_INT = {"crc_poly", "crc_init", "crc_xor_out", "crc_start", "target_id",
        "baud", "de_gpio", "hrt_packets_per_service"}
_ENUM = {"crc_store": ("big", "little"), "lrt_trailer": ("crc", "zero")}


def load(path: str) -> dict:
    with open(path) as handle:
        return json.load(handle)


def stp_block(cfg: dict) -> dict:
    return cfg.setdefault("stp", {})


def wire_from(cfg: dict) -> tuple[P.Wire, list[str]]:
    stp = cfg.get("stp") or {}
    params, problems = crcmod.from_config(stp)
    wire = P.Wire(big_endian=bool(stp.get("big_endian", True)),
                  crc=params,
                  target_id=int(stp.get("target_id", 0xC7)),
                  crc_start=int(stp.get("crc_start", 4)),
                  lrt_trailer=str(stp.get("lrt_trailer", "crc")))
    return wire, problems


# ------------------------------------------------------------------ show

def cmd_show(args) -> int:
    cfg = load(args.config)
    stp = cfg.get("stp") or {}
    wire, problems = wire_from(cfg)

    print(f"\n{args.config}\n")
    print(f"  enabled          {stp.get('enabled', False)}")
    print(f"  port             {stp.get('port', '/dev/ttyAMA0')}")
    print(f"  baud             {stp.get('baud', 921600)}")
    print(f"  target_id        0x{wire.target_id:02X}")
    print(f"  structure order  {'big' if wire.big_endian else 'little'}-endian")
    print(f"  lrt_trailer      {wire.lrt_trailer}")
    de = "released to hardware (de_control false)" if not stp.get("de_control", True) \
        else f"driven on GPIO{stp.get('de_gpio', 4)}"
    print(f"  DE               {de}")
    print(f"\n  CRC              {crcmod.describe(wire.crc, wire.crc_start)}")
    for problem in problems:
        print(f"  !! {problem}")

    cams = cfg.get("cameras") or (stp.get("cameras") or [])
    print(f"\n  cameras          {len(cams)} configured, "
          f"default {cfg.get('default_camera', stp.get('default_camera', 'none'))}")
    for cam in cams:
        flags = " always-on" if cam.get("always_on") else ""
        print(f"    [{cam.get('index')}] {cam.get('name', '?')} "
              f"GPIO{cam.get('gpio')} i2c-{cam.get('i2c_bus', -1)}{flags}")
    print()
    return 0


def cmd_list_crc(args) -> int:
    print("\nCRC variants (use the name, or any unique tail of it):\n")
    for params in crcmod.CATALOG:
        print(f"  {params.name:<22} poly=0x{params.poly:04X} "
              f"init=0x{params.init:04X} refin={int(params.reflect_in)} "
              f"refout={int(params.reflect_out)} "
              f"xorout=0x{params.xor_out:04X}")
    print("\n  custom                 base CCITT-FALSE, then override any of "
          "crc_poly crc_init\n                         crc_reflect_in "
          "crc_reflect_out crc_xor_out crc_store\n")
    return 0


def cmd_diff(args) -> int:
    """What differs from the shipped defaults - the short list worth checking."""
    cfg = load(args.config)
    stp = cfg.get("stp") or {}
    wire, _ = wire_from(cfg)
    base = P.Wire()
    rows = []
    if wire.target_id != base.target_id:
        rows.append(f"target_id 0x{wire.target_id:02X} (default 0x{base.target_id:02X})")
    if wire.crc_start != base.crc_start:
        rows.append(f"crc_start {wire.crc_start} (default {base.crc_start})")
    if wire.big_endian != base.big_endian:
        rows.append(f"big_endian {wire.big_endian} (default {base.big_endian})")
    if wire.lrt_trailer != base.lrt_trailer:
        rows.append(f"lrt_trailer {wire.lrt_trailer} (default {base.lrt_trailer})")
    for name in ("poly", "init", "reflect_in", "reflect_out", "xor_out",
                 "big_endian_store"):
        mine, theirs = getattr(wire.crc, name), getattr(base.crc, name)
        if mine != theirs:
            rows.append(f"crc {name} {mine!r} (default {theirs!r})")
    if int(stp.get("baud", 921600)) != 921600:
        rows.append(f"baud {stp.get('baud')} (default 921600)")
    if not rows:
        print("\n  nothing differs from the defaults; the wire is as shipped\n")
    else:
        print()
        for row in rows:
            print(f"  {row}")
        print()
    return 0


# ------------------------------------------------------------------- set

def parse_value(key: str, raw: str):
    if key in _BOOL:
        low = raw.strip().lower()
        if low in ("1", "true", "yes", "on"):
            return True
        if low in ("0", "false", "no", "off"):
            return False
        raise ValueError(f"{key}: expected true or false, got {raw!r}")
    if key in _INT:
        return crcmod._as_int(raw)
    if key in _ENUM:
        if raw.lower() not in _ENUM[key]:
            raise ValueError(f"{key}: expected one of {_ENUM[key]}, got {raw!r}")
        return raw.lower()
    return raw


def cmd_set(args) -> int:
    cfg = load(args.config)
    stp = stp_block(cfg)
    changes = {}
    for assignment in args.assignment:
        if "=" not in assignment:
            print(f"error: {assignment!r} is not key=value", file=sys.stderr)
            return 2
        name, raw = assignment.split("=", 1)
        name = name.strip().lower()
        if name not in KEYS:
            print(f"error: unknown setting {name!r}. Known: "
                  f"{', '.join(sorted(set(KEYS)))}", file=sys.stderr)
            return 2
        key = KEYS[name]
        try:
            changes[key] = parse_value(key, raw)
        except ValueError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2

    # Validate the *result*, not the edit: a CRC name is only wrong in
    # combination with the overrides that accompany it.
    trial = dict(stp)
    trial.update(changes)
    if changes.get("crc_variant", "").upper() != "CUSTOM" and \
            "crc_variant" in changes:
        # Naming a standard variant abandons any leftover custom overrides,
        # which would otherwise silently keep applying.
        for key in crcmod._PARAM_KEYS:
            trial.pop(key, None)
            changes.setdefault("__drop__", []).append(key)
    dropped = changes.pop("__drop__", [])
    params, problems = crcmod.from_config(trial)
    if problems:
        for problem in problems:
            print(f"error: {problem}", file=sys.stderr)
        return 2

    for key, value in changes.items():
        stp[key] = value
    for key in dropped:
        stp.pop(key, None)

    print("\n  changes:")
    for key, value in changes.items():
        print(f"    {key} = {value!r}")
    for key in dropped:
        print(f"    {key} removed (superseded by the named variant)")
    print(f"\n  resulting CRC: "
          f"{crcmod.describe(params, int(stp.get('crc_start', 4)))}")

    if args.dry_run:
        print("\n  --dry-run: nothing written\n")
        return 0

    stamp = time.strftime("%Y%m%d-%H%M%S")
    backup = os.path.join(BACKUPS, f"config.json.bak.{stamp}")
    try:
        os.makedirs(BACKUPS, exist_ok=True)
        shutil.copy2(args.config, backup)
    except OSError as exc:
        print(f"error: could not back up {args.config}: {exc}", file=sys.stderr)
        return 1

    tmp = args.config + ".new"
    with open(tmp, "w") as handle:
        json.dump(cfg, handle, indent=2)
        handle.write("\n")
    json.load(open(tmp))                 # parses, or we do not install it
    os.replace(tmp, args.config)
    print(f"\n  written; previous file kept at {backup}")

    if args.restart:
        print("  restarting radcamd...")
        subprocess.run(["systemctl", "restart", "radcamd"], check=False)
        time.sleep(3)
        subprocess.run(["journalctl", "-u", "radcamd", "--since", "-15s",
                        "--no-pager", "-o", "cat"], check=False)
    else:
        print("  run with --restart, or: sudo systemctl restart radcamd")
    print()
    return 0


# ------------------------------------------------------------------ bytes

def cmd_crc(args) -> int:
    cfg = load(args.config)
    wire, _ = wire_from(cfg)
    text = "".join(args.bytes).replace(",", " ").replace("0x", " ")
    data = bytes.fromhex("".join(text.split()))
    value = wire.crc.compute(data)
    print(f"\n  {crcmod.describe(wire.crc, wire.crc_start)}")
    print(f"  over {len(data)} bytes -> 0x{value:04X} "
          f"stored as {wire.crc.pack(value).hex()}\n")
    return 0


def cmd_sample(args) -> int:
    """Real packets built with the configured settings, for the ground to check.

    If the flight computer rejects these, the disagreement is in this output
    and can be found by inspection instead of by guesswork on a live link.
    """
    cfg = load(args.config)
    wire, problems = wire_from(cfg)
    for problem in problems:
        print(f"  !! {problem}")
    print(f"\n  {crcmod.describe(wire.crc, wire.crc_start)}")
    print(f"  target 0x{wire.target_id:02X}\n")

    ack = P.encode_command_ack(wire)
    print(f"  Command ACK   {len(ack):>4} bytes  {ack.hex()}")
    print(f"                     CRC over [{wire.crc_start}:{len(ack) - 2}] "
          f"= {ack[-2:].hex()}")

    lrt = P.encode_lrt_data(b"\x00" * 1248, wire)
    hrt = P.encode_hrt_data(b"\x00" * 1280, wire)
    for name, pkt in (("LRT Data", lrt), ("HRT Data", hrt)):
        print(f"  {name:<13} {len(pkt):>4} bytes  "
              f"head {pkt[:16].hex()} ... crc {pkt[-2:].hex()}")
    print()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=CONFIG)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("show", help="current wire settings").set_defaults(fn=cmd_show)
    sub.add_parser("list-crc", help="every CRC variant").set_defaults(fn=cmd_list_crc)
    sub.add_parser("diff", help="what differs from the defaults").set_defaults(fn=cmd_diff)
    sub.add_parser("sample", help="real packets under these settings").set_defaults(fn=cmd_sample)

    s = sub.add_parser("set", help="change settings (key=value ...)")
    s.add_argument("assignment", nargs="+")
    s.add_argument("--restart", action="store_true",
                   help="restart radcamd and show its startup log")
    s.add_argument("--dry-run", action="store_true")
    s.set_defaults(fn=cmd_set)

    s = sub.add_parser("crc", help="CRC of some hex bytes, as configured")
    s.add_argument("bytes", nargs="+")
    s.set_defaults(fn=cmd_crc)

    args = ap.parse_args()
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
