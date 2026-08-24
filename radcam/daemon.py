"""radcamd - the always-on telemetry and housekeeping loop.

Responsibilities, in priority order:

1. Never exit. A crash on a spacecraft is an outage nobody can clear, so every
   subsystem is wrapped: a failed dosimeter read, a dead serial port or a
   missing PWM channel degrades that one field and the loop carries on.
2. Sample the dosimeter and downlink dose telemetry over RS422, mirrored to the
   ground debug port.
3. Pet the systemd watchdog, so that if the loop *does* wedge, systemd restarts
   the unit without anyone asking.

Configuration lives at /etc/radcam/config.json; every key is optional and falls
back to the defaults below.
"""

from __future__ import annotations

import json
import logging
import os
import signal
import socket
import sys
import threading
import time
from pathlib import Path

from .eeprom import CameraEEPROM
from .camera import Camera, LibcameraSource, MediaStore
from .doselog import DEFAULT_PATH as DOSELOG_PATH, DoseLog
from .framing import FrameReader, encode_frame
from .framing import Frame
from .protocol import Config as ProtoConfig, Dispatcher, Msg
from .dosimeter import DEFAULT_STORE, Dosimeter
from .led import LED, LEDError
from .piolink import PioLink
from .ltc2485 import LTC2485
from .telemetry import (DEFAULT_BAUD, FLIGHT_PORT, NullTelemetryLink,
                         PortConfig, Telemetry)
from .stp.crc import CATALOG as CRC_CATALOG, CCITT_FALSE
from .stp.experiment import Experiment, ExperimentConfig
from .stp.link import DeLine, NullDeLine, Rs422Link
from .stp.lrt import EventCode, EventLog
from .stp.packets import Wire
from .slots import DEFAULT_SLOTS, SlotStore
from .stream import StreamConfig, VideoStream

log = logging.getLogger("radcamd")

CONFIG_PATH = "/etc/radcam/config.json"

DEFAULTS = {
    "interval_s": 5.0,
    "warmup_s": 15.0,
    "i2c_bus": 1,
    "dosimeter_address": 0x24,
    "vref": 5.0,
    "calibration_store": DEFAULT_STORE,
    "dose_log": DOSELOG_PATH,
    # The sensor drifts over weeks, so a record a minute is ample.
    "dose_log_interval_s": 60.0,
    "flight_port": FLIGHT_PORT,
    "flight_baud": DEFAULT_BAUD,
    # The mirror is disabled by default: on a Pi 5 the requested GPIO23/24 pins
    # have no hardware UART function, so the operator must choose a real port.
    # See DEVELOPMENT_STATE.md.
    "mirror_port": None,
    # EXTUART via the RP1 PIO block on GPIO24 (TX) / GPIO23 (RX).
    "mirror_pio": True,
    "mirror_tx_pin": 24,
    "mirror_rx_pin": 23,
    "mirror_baud": 921600,
    "led_enabled": True,
    "led_brightness": 0.0,
    # Fields worth triplicating on the downlink.
    "tmr_fields": ["dose_rad", "cal_zero_v"],
    # Serve the binary command protocol (protocol.md) on the flight link.
    "command_server": True,
    # Camera module EEPROM (calibration): CAM1 is i2c-4, CAM0 is i2c-6.
    "camera_i2c_bus": 4,
    # EEPROM_WRITE stays refused unless this is turned on. See protocol.md 9.3:
    # a stray write destroys the only state aboard that cannot be regenerated.
    "eeprom_writable": False,
}


def load_config(path: str = CONFIG_PATH) -> dict:
    cfg = dict(DEFAULTS)
    p = Path(path)
    if p.exists():
        try:
            cfg.update(json.loads(p.read_text()))
            log.info("loaded config from %s", p)
        except Exception as exc:
            log.error("bad config %s (%s); using defaults", p, exc)
    else:
        log.info("no config at %s; using defaults", p)
    return cfg


class Watchdog:
    """Minimal sd_notify, so we do not need python-systemd installed."""

    def __init__(self):
        self.addr = os.environ.get("NOTIFY_SOCKET")
        self.sock = None
        usec = os.environ.get("WATCHDOG_USEC")
        # Pet at half the configured timeout, as systemd recommends.
        self.interval = (int(usec) / 2_000_000) if usec else None
        if self.addr:
            if self.addr.startswith("@"):        # abstract namespace
                self.addr = "\0" + self.addr[1:]
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._last = 0.0

    def _send(self, msg: str) -> None:
        if self.sock and self.addr:
            try:
                self.sock.sendto(msg.encode(), self.addr)
            except OSError as exc:
                log.debug("sd_notify failed: %s", exc)

    def ready(self) -> None:
        self._send("READY=1")

    def status(self, text: str) -> None:
        self._send(f"STATUS={text}")

    def pet(self) -> None:
        if self.interval is None:
            return
        now = time.monotonic()
        if now - self._last >= self.interval:
            self._send("WATCHDOG=1")
            self._last = now


def cpu_temp_c() -> float | None:
    try:
        raw = Path("/sys/class/thermal/thermal_zone0/temp").read_text().strip()
        return int(raw) / 1000.0
    except (OSError, ValueError):
        return None


class _BeaconDone(Exception):
    """Control-flow marker: the beacon path is finished for this cycle."""


class Daemon:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.running = True
        self.watchdog = Watchdog()

        adc = LTC2485(bus=cfg["i2c_bus"], address=cfg["dosimeter_address"],
                      vref=cfg["vref"])
        self.dosimeter = Dosimeter(adc, store_path=cfg["calibration_store"])

        mirror = None
        if cfg.get("mirror_port"):
            mirror = PortConfig(cfg["mirror_port"], cfg["mirror_baud"],
                                "ground-debug")
        self.telemetry = Telemetry(
            flight=PortConfig(cfg["flight_port"], cfg["flight_baud"],
                              "flight-rs422"),
            mirror=mirror,
            tmr_keys=cfg.get("tmr_fields", []),
        )

        # EXTUART on GPIO24/23 is driven by PIO, not a kernel tty, so it is
        # substituted in rather than configured as a device path.
        if cfg.get("mirror_pio"):
            self.telemetry.mirror = PioLink(
                baud=int(cfg.get("mirror_baud", 921600)),
                tx_pin=int(cfg.get("mirror_tx_pin", 24)),
                rx_pin=int(cfg.get("mirror_rx_pin", 23)))
        self.led: LED | None = None
        self.doselog = DoseLog(cfg.get('dose_log', DOSELOG_PATH))
        self._last_dose_log = 0.0

        # Binary command protocol on the flight link. The camera source is the
        # real one; it reports unavailable until the sensor answers, and the
        # dispatcher then returns ERR_CAMERA_FAULT rather than pretending.
        self.reader = FrameReader()
        self.media = MediaStore()
        # The camera EEPROM, so the ground can read, verify and repair the
        # calibration over the link. Constructed even if absent: the protocol
        # answers ERR_EEPROM_FAULT rather than the daemon failing to start.
        try:
            self.camera_eeprom = CameraEEPROM(bus=cfg.get("camera_i2c_bus", 4))
        except Exception as exc:                       # noqa: BLE001
            log.warning("camera EEPROM unavailable: %s", exc)
            self.camera_eeprom = None
        # Hand the capture path the distortion model, if the module carries
        # one, so CAPTURE with undistort=1 can correct without touching I2C.
        dmodel = None
        if self.camera_eeprom is not None:
            try:
                dmodel = (self.camera_eeprom.load() or {}).get("distortion")
            except Exception as exc:                   # noqa: BLE001
                log.warning("cannot read distortion model: %s", exc)
        self.camera = Camera(source=LibcameraSource(), store=self.media,
                             led=None, distortion_model=dmodel)
        self.dispatcher = Dispatcher(config=ProtoConfig(), camera=self.camera,
                                     dosimeter=self.dosimeter, led=None,
                                     store=self.media,
                                     eeprom=self.camera_eeprom,
                                     eeprom_writable=bool(
                                         cfg.get("eeprom_writable", False)))
        self.commands_served = 0

        # Counters that go out in the telemetry so the ground can see health.
        self.errors = {"dosimeter": 0, "telemetry": 0, "led": 0}
        self.watchdog_pets = 0

        # -- STP / DICE RS-422 -------------------------------------------
        # When this is on, the flight port belongs to the STP link and nothing
        # else may write to it: we are a slave on a bus shared with other
        # experiments, and an unsolicited byte corrupts somebody else's reply.
        self.stp: Experiment | None = None
        self.stp_link: Rs422Link | None = None
        self.stream: VideoStream | None = None
        self.slots: SlotStore | None = None
        self.stp_events = EventLog()
        self._stp_thread = None
        self._stp_state: dict = {}
        self._build_stp()

    def _build_stp(self) -> None:
        cfg = self.cfg.get("stp") or {}
        if not cfg.get("enabled"):
            return

        crc = CCITT_FALSE
        wanted = str(cfg.get("crc_variant", "CRC-16/CCITT-FALSE")).upper()
        for candidate in CRC_CATALOG:
            if candidate.name.upper() == wanted or \
                    candidate.name.upper().endswith(wanted):
                crc = candidate
                break
        else:
            log.warning("unknown crc_variant %r; using %s",
                        cfg.get("crc_variant"), crc.name)

        wire = Wire(
            big_endian=bool(cfg.get("big_endian", True)),
            crc=crc,
            target_id=int(cfg.get("target_id", 0xC7)),
            crc_start=int(cfg.get("crc_start", 4)),
            lrt_trailer=str(cfg.get("lrt_trailer", "crc")),
        )

        de_gpio = cfg.get("de_gpio", 4)
        chip = str(cfg.get("de_chip", "/dev/gpiochip0"))
        if de_gpio in (None, -1):
            de = NullDeLine(chip=chip)
        elif not cfg.get("de_control", True):
            # DE is tied active in hardware. Release the pin rather than
            # parking it low, which would hold the transmitter disabled.
            de = NullDeLine(release_gpio=int(de_gpio), chip=chip)
        else:
            de = DeLine(gpio=int(de_gpio), chip=chip,
                        active_high=bool(cfg.get("de_active_high", True)))

        self.stp_link = Rs422Link(
            port=str(cfg.get("port", self.cfg.get("flight_port", FLIGHT_PORT))),
            baud=int(cfg.get("baud", 921600)),
            de=de,
            guard_chars=float(cfg.get("de_guard_chars", 2.0)),
            setup_us=float(cfg.get("de_setup_us", 10.0)),
            discard_echo=bool(cfg.get("discard_echo", True)))

        # Live video is constructed but not started: the encoder is a real
        # cost in CPU and power, and nothing should be running until the
        # ground asks for it.
        stream_cfg = cfg.get("stream") or {}
        self.stream = VideoStream(
            StreamConfig(
                width=int(stream_cfg.get("width", 640)),
                height=int(stream_cfg.get("height", 480)),
                fps=int(stream_cfg.get("fps", 15)),
                bitrate=int(stream_cfg.get("bitrate", 600_000)),
                centre_x=int(stream_cfg.get("centre_x", 4208 // 2)),
                centre_y=int(stream_cfg.get("centre_y", 3120 // 2)),
                crop_w=int(stream_cfg.get("crop_w", 640)),
                crop_h=int(stream_cfg.get("crop_h", 480))),
            queue_frames=int(stream_cfg.get("queue_frames", 8)))

        # Numbered slots, so a canned command from the ground addresses the
        # same place every time regardless of capture history.
        self.slots = SlotStore(
            directory=cfg.get("slot_dir", "/var/lib/radcam/slots"),
            count=int(cfg.get("slot_count", DEFAULT_SLOTS)))

        self.stp = Experiment(
            link=self.stp_link, wire=wire, dispatcher=self.dispatcher,
            store=self.media, state_provider=lambda: self._stp_state,
            events=self.stp_events, stream=self.stream, slots=self.slots,
            config=ExperimentConfig(
                target_id=wire.target_id,
                version=str(cfg.get("version", "1.0")),
                hrt_packets_per_service=int(cfg.get("hrt_packets_per_service", 8)),
                max_command_queue=int(cfg.get("max_command_queue", 4)),
                safe_mode_threshold=int(cfg.get("safe_mode_threshold", 5)),
                hrt_idle_fill=bool(cfg.get("hrt_idle_fill", False)),
                scrub_interval_s=float(cfg.get("scrub_interval_s", 30.0)),
                boot_count=self._boot_count()))

        # The ASCII beacon must never reach the DICE bus. Removing the port
        # entirely is stronger than remembering not to write to it.
        self.telemetry.flight = NullTelemetryLink()
        log.info("STP enabled: target 0x%02X, %s at %d baud, DE on GPIO%s",
                 wire.target_id, self.stp_link.port, self.stp_link.baud,
                 de_gpio)
        log.info("live stream configured (not started): %s",
                 self.stream.config.describe())
        summary = self.slots.summary()
        log.info("storage: %d slots, %d used, %d free",
                 summary["slot_count"], summary["slots_used"],
                 summary["slots_free"])
        if not self.stream.available:
            log.warning("live stream unavailable, missing: %s",
                        ", ".join(self.stream.missing()))

    def _boot_count(self) -> int:
        """Persisted across reboots so the ground can see resets happening."""
        path = Path(self.cfg.get("boot_count_path",
                                 "/var/lib/radcam/boot-count"))
        try:
            count = int(path.read_text().strip()) + 1 if path.exists() else 1
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(str(count))
            return count
        except Exception as exc:                       # noqa: BLE001
            log.warning("cannot track boot count: %s", exc)
            return 0

    # -- startup / shutdown ---------------------------------------------

    def start(self) -> None:
        # Only the main thread may install handlers; tolerate being embedded.
        try:
            signal.signal(signal.SIGTERM, self._stop)
            signal.signal(signal.SIGINT, self._stop)
        except ValueError:
            log.debug("not on the main thread; skipping signal handlers")

        try:
            self.dosimeter.open()
            # The input settles along an RC curve after the bus has been idle,
            # so readings taken immediately after startup are not meaningful.
            # Warm up before the first frame goes out, and calibrate only once
            # the reading is stable.
            self.dosimeter.settle(seconds=float(self.cfg.get("warmup_s", 15.0)))
            self.dosimeter.ensure_calibrated()
        except Exception as exc:
            self.errors["dosimeter"] += 1
            log.error("dosimeter startup failed: %s", exc)

        self.telemetry.open()

        if self.cfg.get("led_enabled"):
            try:
                self.led = LED().open()
                # The illumination array is off except during a capture with
                # flash configured. Never honour a non-zero brightness here.
                self.led.off()
                self.dispatcher.led = self.led
                self.camera.led = self.led
            except LEDError as exc:
                self.errors["led"] += 1
                log.error("LED init failed: %s", exc)
                self.led = None

        if self.stp is not None:
            try:
                self.stp_link.open()
                self.stp.start()
                self._stp_state = self._stp_snapshot({})
                self._stp_thread = threading.Thread(
                    target=self._stp_serve, name="stp-service", daemon=True)
                self._stp_thread.start()
            except Exception as exc:                   # noqa: BLE001
                self.errors["telemetry"] += 1
                log.error("STP link failed to start: %s", exc)
                self.stp = None

        self.watchdog.ready()
        log.info("radcamd running, interval %.1fs", self.cfg["interval_s"])

    # -- STP service -----------------------------------------------------

    def _stp_serve(self) -> None:
        """Answer DICE continuously, independently of the housekeeping cycle.

        This runs in its own thread because the two cadences are unrelated: the
        dosimeter is sampled every couple of seconds, while an LRT request has
        to be answered in milliseconds. Driving both from one loop would make
        the reply latency hostage to the sample interval.
        """
        while self.running:
            try:
                self.stp.service()
            except Exception as exc:                   # noqa: BLE001
                log.exception("STP service pass failed: %s", exc)
                time.sleep(0.1)

    def _stp_snapshot(self, fields: dict) -> dict:
        """Flatten the housekeeping into the keys the LRT builder expects.

        Built here, on the daemon thread, and handed over as a finished dict:
        the LRT reply path must not do I/O while DICE is waiting for it.
        """
        def _f(key, default=0.0):
            value = fields.get(key, default)
            try:
                return float(value)
            except (TypeError, ValueError):
                return default

        media = []
        try:
            media = self.media.list() or []
        except Exception:                              # noqa: BLE001
            pass

        free = used = 0
        try:
            stat = os.statvfs(str(getattr(self.media, "root", "/")))
            free = stat.f_bavail * stat.f_frsize
            used = (stat.f_blocks - stat.f_bfree) * stat.f_frsize
        except Exception:                              # noqa: BLE001
            pass

        eeprom_total = eeprom_good = 0
        if self.camera_eeprom is not None:
            try:
                status = self.camera_eeprom.copy_status()
                eeprom_total = len(status)
                eeprom_good = sum(1 for ok in status if ok)
            except Exception:                          # noqa: BLE001
                pass

        camera_ok = False
        try:
            camera_ok = bool(self.camera.available())
        except Exception:                              # noqa: BLE001
            pass

        return {
            "dose_rad": _f("dose_rad"),
            "dose_volts": _f("volts"),
            "dose_calibrated": bool(fields.get("cal")),
            "dose_errors": self.errors["dosimeter"],
            "cpu_temp_c": _f("tempC"),
            "led_percent": int(round(_f("led") * 100)),
            "led_ok": self.led is not None,
            "camera_available": camera_ok,
            "camera_ok": camera_ok,
            "recording": self.dispatcher.recording is not None,
            "recording_id": self.dispatcher.recording or 0,
            "storage_free": free,
            "storage_used": used,
            "media_count": len(media),
            "eeprom_copies": eeprom_total,
            "eeprom_good": eeprom_good,
            "dosimeter_ok": self.errors["dosimeter"] == 0,
            "watchdog_pets": self.watchdog_pets,
            "fw_major": 1,
            "fw_minor": 0,
        }

    def _stop(self, *_args) -> None:
        log.info("shutdown requested")
        self.running = False

    def shutdown(self) -> None:
        if self.stream is not None:
            # Two child processes; leaving them behind would hold the camera
            # against the next start.
            try:
                self.stream.stop()
            except Exception as exc:                   # noqa: BLE001
                log.error("stopping the stream failed: %s", exc)
        if self.stp is not None:
            try:
                self.stp.stop()
            except Exception as exc:                   # noqa: BLE001
                log.error("STP shutdown failed: %s", exc)
        if self.stp_link is not None:
            # Releases DE, so the transceiver stops driving the shared bus.
            try:
                self.stp_link.close()
            except Exception as exc:                   # noqa: BLE001
                log.error("closing RS-422 link failed: %s", exc)
        if self.led is not None:
            try:
                self.led.close()
            except Exception:
                pass
        self.telemetry.close()
        self.dosimeter.close()
        log.info("radcamd stopped")

    # -- main loop -------------------------------------------------------

    def sample(self) -> dict:
        fields: dict = {}

        try:
            reading = self.dosimeter.read()
            fields["code"] = reading.code
            fields["volts"] = round(reading.volts, 6)
            fields["dose_rad"] = (round(reading.dose_rad, 3)
                                  if reading.dose_rad is not None else "")
            fields["cal"] = reading.calibrated
            cal = self.dosimeter.calibration
            if cal is not None:
                fields["cal_zero_v"] = round(cal.zero_volts, 6)
        except Exception as exc:
            self.errors["dosimeter"] += 1
            log.error("dosimeter read failed: %s", exc)
            fields["dose_rad"] = ""
            fields["cal"] = False

        if self.led is not None:
            fields["led"] = round(self.led.brightness, 4)

        temp = cpu_temp_c()
        if temp is not None:
            fields["tempC"] = round(temp, 1)

        # Persist a coarse dose history for later download.
        interval = float(self.cfg.get("dose_log_interval_s", 60.0))
        now = time.monotonic()
        if (fields.get("cal") and fields.get("dose_rad") != ""
                and now - self._last_dose_log >= interval):
            try:
                self.doselog.append(float(fields["dose_rad"]),
                                    float(fields["volts"]))
                self._last_dose_log = now
            except Exception as exc:            # noqa: BLE001
                log.error("dose log append failed: %s", exc)

        fields["up_s"] = int(time.monotonic())
        fields["cmds"] = self.commands_served
        fields["bad"] = self.reader.bad_frames
        fields["err"] = "{}/{}/{}".format(self.errors["dosimeter"],
                                          self.errors["telemetry"],
                                          self.errors["led"])
        return fields

    def serve_commands(self) -> None:
        """Drain the flight link, dispatch any complete frames, reply.

        Wrapped whole: a malformed command must never take down the loop.
        """
        try:
            data = self.telemetry.flight.read_bytes()
            if self.telemetry.mirror is not None:
                data += self.telemetry.mirror.read_bytes()
            if not data:
                return

            for frame in self.reader.feed(data):
                for response in self.dispatcher.handle(frame):
                    wire = encode_frame(response)
                    self.telemetry.flight.write(wire)
                    if self.telemetry.mirror is not None:
                        self.telemetry.mirror.write(wire)
                self.commands_served += 1
        except Exception as exc:                      # noqa: BLE001
            log.error("command server error: %s", exc)

    def run(self) -> int:
        self.start()
        interval = float(self.cfg["interval_s"])
        try:
            while self.running:
                cycle_start = time.monotonic()

                fields = self.sample()

                if self.stp is not None:
                    # Refresh what the LRT reply path will hand to DICE.
                    self._stp_state = self._stp_snapshot(fields)

                serving = bool(self.cfg.get("command_server", True))
                try:
                    # When the flight link carries the binary protocol, the
                    # beacon goes out as a binary TELEMETRY frame and the
                    # readable ASCII form is restricted to the debug mirror.
                    # Two framings on one stream corrupt each other.
                    # With STP running, the flight port is the DICE bus and
                    # carries nothing unsolicited; the readable beacon goes to
                    # the debug mirror only.
                    if self.stp is not None:
                        self.telemetry.send(fields, flight=False)
                        raise _BeaconDone
                    sent = self.telemetry.send(fields, flight=not serving)
                    if serving:
                        beacon = Frame(Msg.TELEMETRY, self.telemetry.seq & 0xFFFF,
                                       self.dispatcher._telemetry_payload())
                        if not self.telemetry.flight.write(encode_frame(beacon)):
                            self.errors["telemetry"] += 1
                    elif not sent["flight"]:
                        self.errors["telemetry"] += 1
                except _BeaconDone:
                    pass
                except Exception as exc:
                    self.errors["telemetry"] += 1
                    log.error("telemetry send failed: %s", exc)

                # The old COBS command server and the STP link are mutually
                # exclusive: both would read the same port.
                if self.stp is None and self.cfg.get("command_server", True):
                    self.serve_commands()

                self.watchdog_pets += 1
                self.watchdog.pet()
                self.watchdog.status(
                    f"dose={fields.get('dose_rad', '?')} rad "
                    f"errs={fields.get('err')}")

                # Sleep the remainder of the cycle, in short slices so SIGTERM
                # is acted on promptly.
                while self.running:
                    elapsed = time.monotonic() - cycle_start
                    if elapsed >= interval:
                        break
                    time.sleep(min(0.25, interval - elapsed))
        finally:
            self.shutdown()
        return 0


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(
        level=os.environ.get("RADCAM_LOGLEVEL", "INFO"),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    argv = argv if argv is not None else sys.argv[1:]
    path = argv[0] if argv else CONFIG_PATH
    return Daemon(load_config(path)).run()


if __name__ == "__main__":
    raise SystemExit(main())
