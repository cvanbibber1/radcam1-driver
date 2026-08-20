"""radcamctl - operator command line for the radcam1 payload.

    radcamctl status                 one-shot health snapshot
    radcamctl dose [--watch]         dosimeter reading(s)
    radcamctl calibrate [--force]    capture the unirradiated baseline
    radcamctl led <percent>          set illumination (hard-capped at 10%)
    radcamctl led off
    radcamctl selftest               exercise every subsystem and report
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time

from .daemon import CONFIG_PATH, cpu_temp_c, load_config
from .dosimeter import Dosimeter
from .led import MAX_DUTY, LED, LEDError
from .ltc2485 import LTC2485
from .telemetry import PortConfig, TelemetryLink

log = logging.getLogger("radcamctl")


def _dosimeter(cfg: dict) -> Dosimeter:
    adc = LTC2485(bus=cfg["i2c_bus"], address=cfg["dosimeter_address"],
                  vref=cfg["vref"])
    return Dosimeter(adc, store_path=cfg["calibration_store"])


def cmd_status(args, cfg) -> int:
    out: dict = {"cpu_temp_c": cpu_temp_c()}

    try:
        with _dosimeter(cfg) as d:
            r = d.read()
            out["dosimeter"] = r.to_dict()
            out["calibration"] = (d.calibration.to_dict()
                                  if d.calibration else None)
    except Exception as exc:
        out["dosimeter_error"] = str(exc)

    try:
        led = LED()
        led.open()
        out["led"] = led.status()
        led.close()
    except LEDError as exc:
        out["led_error"] = str(exc)

    for key, port in (("flight", cfg["flight_port"]),
                      ("mirror", cfg.get("mirror_port"))):
        if not port:
            out[f"{key}_port"] = None
            continue
        link = TelemetryLink(PortConfig(port, name=key))
        out[f"{key}_port"] = {"device": port, "open": link.open()}
        link.close()

    print(json.dumps(out, indent=2, default=str))
    return 0


def cmd_dose(args, cfg) -> int:
    with _dosimeter(cfg) as d:
        if not args.no_settle:
            print("settling...", file=sys.stderr)
            d.settle()
        while True:
            r = d.read()
            dose = "uncalibrated" if r.dose_rad is None else f"{r.dose_rad:+9.3f} rad"
            print(f"{r.timestamp_utc}  code={r.code:>9d}  "
                  f"V={r.volts:+.6f}  {dose}")
            if not args.watch:
                return 0
            time.sleep(args.interval)


def cmd_calibrate(args, cfg) -> int:
    with _dosimeter(cfg) as d:
        if d.calibration is not None and not args.force:
            print("Calibration already exists - refusing to overwrite.\n"
                  "Re-zeroing discards accumulated dose history. Use --force "
                  "only if you are certain the sensor is unirradiated.",
                  file=sys.stderr)
            print(json.dumps(d.calibration.to_dict(), indent=2))
            return 1
        cal = d.calibrate(samples=args.samples, force=args.force,
                          note=args.note or "")
        print(json.dumps(cal.to_dict(), indent=2))
    return 0


def cmd_led(args, cfg) -> int:
    try:
        with LED() as led:
            if args.value.lower() in ("off", "0"):
                led.off()
            elif args.value.lower() == "max":
                led.full()
            else:
                led.set_percent(float(args.value))
            st = led.status()
            print(f"brightness {st['brightness'] * 100:.1f}% of full scale "
                  f"(cap {MAX_DUTY * 100:.0f}%), duty "
                  f"{st['duty_cycle_ns']}/{st['period_ns']} ns")
            # Hold the setting until told otherwise rather than releasing it.
            if args.hold:
                print("holding; Ctrl-C to release")
                while True:
                    time.sleep(1)
    except LEDError as exc:
        print(f"LED error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        pass
    return 0


def cmd_selftest(args, cfg) -> int:
    failures = 0

    def check(name: str, fn) -> None:
        nonlocal failures
        try:
            detail = fn()
            print(f"  PASS  {name}: {detail}")
        except Exception as exc:
            failures += 1
            print(f"  FAIL  {name}: {exc}")

    print("radcam self-test")

    def dosimeter_check():
        with _dosimeter(cfg) as d:
            r = d.read()
            return f"code={r.code}, {r.volts:+.4f} V, calibrated={r.calibrated}"

    def led_check():
        with LED() as led:
            led.set_brightness(0.01)
            st = led.status()
            led.off()
            return f"duty {st['duty_cycle_ns']} ns at 1%"

    def flight_check():
        link = TelemetryLink(PortConfig(cfg["flight_port"], name="flight"))
        if not link.open():
            raise RuntimeError(f"cannot open {cfg['flight_port']}")
        link.close()
        return cfg["flight_port"]

    def tmr_check():
        from .tmr import TMRStore
        import tempfile
        import os
        with tempfile.TemporaryDirectory() as tmp:
            store = TMRStore(os.path.join(tmp, "t.json"))
            store.write({"a": 1})
            p = store._paths()[0]
            b = bytearray(p.read_bytes())
            b[-5] ^= 0x02
            p.write_bytes(bytes(b))
            if store.read() != {"a": 1}:
                raise RuntimeError("majority vote did not recover the value")
        return "bit flip injected and corrected"

    check("dosimeter", dosimeter_check)
    check("led pwm", led_check)
    check("flight uart", flight_check)
    check("tmr store", tmr_check)

    print(f"\n{'all subsystems nominal' if not failures else f'{failures} failure(s)'}")
    return 1 if failures else 0


def cmd_eeprom(args, cfg) -> int:
    """Inspect, dump and repair the camera module's calibration EEPROM.

    The local equivalent of the RS422 commands in protocol.md section 9, so the
    same operations can be rehearsed on the bench before anyone needs them over
    a link with minutes of latency.
    """
    from .eeprom import COPY_OFFSETS, CameraEEPROM

    bus = getattr(args, "bus", None) or cfg.get("camera_i2c_bus", 4)
    ee = CameraEEPROM(bus=bus)
    if not ee.present():
        print(f"no EEPROM answering at 0x50 on i2c-{bus}")
        return 1

    if args.action == "status":
        ok = ee.copy_status()
        for off, good in zip(COPY_OFFSETS, ok):
            print(f"  copy at 0x{off:04X}: {'ok' if good else 'FAILED CRC'}")
        print(f"  {sum(ok)}/{len(ok)} copies intact")
        rec = ee.load()
        if rec:
            print(f"  camera {rec.get('camera_id', '?')}, sections: "
                  f"{', '.join(sorted(k for k in rec if isinstance(rec[k], dict)))}")
        # A single surviving copy still reads correctly but is one event from
        # losing the calibration entirely, so it is worth a non-zero exit.
        return 0 if all(ok) else 2

    if args.action == "repair":
        if ee.repair():
            print("repaired; all copies now match")
        else:
            print("nothing to repair (or nothing left to repair from)")
        return 0

    if args.action == "dump":
        if args.raw:
            data = ee.read(args.offset, args.length)
            for i in range(0, len(data), 16):
                row = data[i:i + 16]
                hexs = " ".join(f"{b:02x}" for b in row)
                text = "".join(chr(b) if 32 <= b < 127 else "." for b in row)
                print(f"  {args.offset + i:04x}  {hexs:<47}  {text}")
        else:
            rec = ee.load()
            if rec is None:
                print("no valid calibration record")
                return 1
            print(json.dumps(rec, indent=2, sort_keys=True))
        return 0

    if args.action == "erase":
        if not args.yes:
            print("refusing to erase without --yes")
            return 1
        ee.erase()
        print("erased; this camera now carries no calibration")
        return 0
    return 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="radcamctl",
                                 description="radcam1 payload control")
    ap.add_argument("-c", "--config", default=CONFIG_PATH)
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status").set_defaults(fn=cmd_status)

    p = sub.add_parser("dose")
    p.add_argument("--watch", action="store_true")
    p.add_argument("--interval", type=float, default=2.0)
    p.add_argument("--no-settle", action="store_true")
    p.set_defaults(fn=cmd_dose)

    p = sub.add_parser("calibrate")
    p.add_argument("--force", action="store_true")
    p.add_argument("--samples", type=int, default=32)
    p.add_argument("--note", default="")
    p.set_defaults(fn=cmd_calibrate)

    p = sub.add_parser("led")
    p.add_argument("value", help="percent of full scale, 'off', or 'max'")
    p.add_argument("--hold", action="store_true")
    p.set_defaults(fn=cmd_led)

    p = sub.add_parser("eeprom", help="camera calibration EEPROM")
    p.add_argument("action",
                   choices=("status", "dump", "repair", "erase"))
    p.add_argument("--bus", type=int, default=None,
                   help="camera I2C bus (4 = CAM1, 6 = CAM0)")
    p.add_argument("--raw", action="store_true",
                   help="hex dump the device instead of the parsed record")
    p.add_argument("--offset", type=lambda v: int(v, 0), default=0)
    p.add_argument("--length", type=lambda v: int(v, 0), default=256)
    p.add_argument("--yes", action="store_true", help="confirm erase")
    p.set_defaults(fn=cmd_eeprom)

    sub.add_parser("selftest").set_defaults(fn=cmd_selftest)

    args = ap.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s %(message)s")

    return args.fn(args, load_config(args.config))


if __name__ == "__main__":
    raise SystemExit(main())
