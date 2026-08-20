"""Illumination LED control - TPS922051 constant-current driver on GPIO18.

The LED array is very bright and the mission caps it at 10% duty. That cap is
enforced here, in the one class that is allowed to touch the PWM channel, and
it cannot be raised through the public API: `MAX_DUTY` is applied to every
write, and brightness is always expressed as a fraction of *full scale* so the
limit stays visible rather than being hidden behind a rescaled range.

Hardware path: GPIO18 -> PWM0_CHAN2 on the RP1, enabled by
overlays/radcam-led-overlay.dts, exported by the kernel as channel 2 of
/sys/class/pwm/pwmchip0.
"""

from __future__ import annotations

import logging
import time
from contextlib import contextmanager
from pathlib import Path

log = logging.getLogger(__name__)

PWMCHIP = "/sys/class/pwm/pwmchip0"
LED_CHANNEL = 2                 # GPIO18 = PWM0_CHAN2

# 10 kHz: well above anything visible or audible, and comfortably inside the
# TPS922051's PWM dimming range. At the RP1's 50 MHz PWM clock this leaves
# ~5000 steps of resolution across the period.
DEFAULT_PERIOD_NS = 100_000

#: Absolute ceiling on duty cycle, as a fraction of full scale.
MAX_DUTY = 0.10


class LEDError(Exception):
    pass


class LED:
    def __init__(self, chip: str = PWMCHIP, channel: int = LED_CHANNEL,
                 period_ns: int = DEFAULT_PERIOD_NS):
        self.chip = Path(chip)
        self.channel = channel
        self.period_ns = period_ns
        self.path = self.chip / f"pwm{channel}"
        self._brightness = 0.0

    # -- sysfs helpers ---------------------------------------------------

    def _write(self, node: str, value: str | int) -> None:
        try:
            (self.path / node).write_text(f"{value}\n")
        except OSError as exc:
            raise LEDError(f"writing {node}={value}: {exc}") from exc

    def _read(self, node: str) -> str:
        return (self.path / node).read_text().strip()

    # -- lifecycle -------------------------------------------------------

    def open(self) -> "LED":
        if not self.chip.exists():
            raise LEDError(
                f"{self.chip} missing - is dtoverlay=radcam-led loaded?")

        if not self.path.exists():
            try:
                (self.chip / "export").write_text(f"{self.channel}\n")
            except OSError as exc:
                raise LEDError(f"exporting channel {self.channel}: {exc}") from exc
            # udev needs a moment to create the attributes.
            for _ in range(50):
                if (self.path / "period").exists():
                    break
                time.sleep(0.02)

        # Duty must never exceed period, so zero it before changing period.
        self._write("duty_cycle", 0)
        self._write("period", self.period_ns)
        self._write("enable", 1)
        self._brightness = 0.0
        log.info("LED PWM ready: %s, period %d ns, cap %.0f%%",
                 self.path, self.period_ns, MAX_DUTY * 100)
        return self

    def close(self) -> None:
        """Turn the LEDs off and release the channel."""
        try:
            self.off()
            self._write("enable", 0)
        except LEDError:
            pass
        try:
            (self.chip / "unexport").write_text(f"{self.channel}\n")
        except OSError:
            pass

    def __enter__(self) -> "LED":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    # -- brightness ------------------------------------------------------

    @property
    def brightness(self) -> float:
        """Current duty cycle as a fraction of full scale."""
        return self._brightness

    def set_brightness(self, fraction: float) -> float:
        """Set duty cycle as a fraction of full scale (0.0 - 1.0).

        Values above MAX_DUTY are clamped, not rejected, so a caller asking for
        full brightness gets the brightest the mission allows rather than an
        exception mid-flight. The clamp is logged. Returns what was applied.
        """
        if fraction != fraction:            # NaN
            raise LEDError("brightness must be a number")
        requested = float(fraction)
        value = max(0.0, min(requested, MAX_DUTY))

        if requested > MAX_DUTY:
            log.warning("brightness %.3f requested, capped to %.3f",
                        requested, MAX_DUTY)
        elif requested < 0.0:
            log.warning("negative brightness %.3f requested, using 0", requested)

        duty_ns = int(round(self.period_ns * value))
        self._write("duty_cycle", duty_ns)
        self._brightness = value
        return value

    def set_percent(self, percent: float) -> float:
        """Set brightness as a percentage of full scale (0 - 100)."""
        return self.set_brightness(percent / 100.0)

    def off(self) -> None:
        self.set_brightness(0.0)

    def full(self) -> float:
        """Brightest setting the cap permits."""
        return self.set_brightness(MAX_DUTY)

    @contextmanager
    def flash(self, percent: float, duration_s: float | None = None):
        """Illuminate for the duration of a capture, then guarantee darkness.

        The LEDs are off at all times except inside this block. The `finally`
        is what matters: if the capture raises, the caller is killed, or the
        exposure hangs, the LEDs still go out. Nothing else in the system is
        allowed to leave them lit.

        `percent` of 0 (or anything invalid) means no flash at all, and the
        10% ceiling still applies - see set_brightness().
        """
        applied = 0.0
        try:
            if percent and percent > 0:
                applied = self.set_percent(percent)
                if duration_s:
                    time.sleep(duration_s)
            yield applied
        finally:
            try:
                self.set_brightness(0.0)
            except LEDError as exc:
                # Losing the PWM channel mid-capture must be loud: this is the
                # one failure that could leave the array lit.
                log.error("FAILED TO EXTINGUISH LEDs: %s", exc)

    def status(self) -> dict:
        return {
            "path": str(self.path),
            "period_ns": int(self._read("period")),
            "duty_cycle_ns": int(self._read("duty_cycle")),
            "enabled": self._read("enable") == "1",
            "brightness": self._brightness,
            "max_duty": MAX_DUTY,
        }
