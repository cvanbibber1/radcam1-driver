"""EXTUART over the RP1 PIO block, presented as an ordinary telemetry link.

The debug port is fixed at GPIO24 (TX) / GPIO23 (RX) by a finalised board, and
neither pin has a UART alt-function on a Pi 5. PIO drives them at the full
921600 baud, but a PIO state machine is not a kernel tty - there is no device
node for pyserial to open.

`softuart/pio_uart_bridge` closes that gap: stdin goes out on the wire and
whatever arrives comes back on stdout. This class runs it as a subprocess and
exposes the same `write()` / `read_bytes()` interface as
`telemetry.TelemetryLink`, so `Telemetry` can use it as the mirror without
knowing the difference.

Failures are contained the same way as a serial port: a dead bridge degrades
the mirror to silence and is restarted on the next write, and never takes the
flight link with it.
"""

from __future__ import annotations

import fcntl
import logging
import os
import shutil
import subprocess
import time
from pathlib import Path

log = logging.getLogger(__name__)

DEFAULT_BRIDGE = "/home/rad/driver-dev/softuart/pio_uart_bridge"
DEFAULT_BAUD = 921600

#: Restart no faster than this, so a permanently broken bridge cannot become a
#: fork bomb on a system that is supposed to run unattended for months.
RESTART_INTERVAL_S = 5.0


class PioLink:
    """A PIO-driven UART with the TelemetryLink interface."""

    def __init__(self, bridge: str = DEFAULT_BRIDGE, baud: int = DEFAULT_BAUD,
                 tx_pin: int = 24, rx_pin: int = 23, name: str = "extuart-pio"):
        self.bridge = bridge
        self.baud = baud
        self.tx_pin = tx_pin
        self.rx_pin = rx_pin
        self.name = name
        self.failures = 0
        self._proc: subprocess.Popen | None = None
        self._last_start = 0.0

    # -- lifecycle -------------------------------------------------------

    def open(self) -> bool:
        if self.is_open:
            return True

        if not Path(self.bridge).is_file() or not os.access(self.bridge, os.X_OK):
            log.error("PIO bridge %s missing or not executable", self.bridge)
            return False

        if time.monotonic() - self._last_start < RESTART_INTERVAL_S:
            return False
        self._last_start = time.monotonic()

        cmd = [self.bridge, "--baud", str(self.baud),
               "--tx-pin", str(self.tx_pin), "--rx-pin", str(self.rx_pin)]
        if os.geteuid() != 0 and shutil.which("sudo"):
            cmd = ["sudo", "-n", *cmd]

        try:
            self._proc = subprocess.Popen(
                cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL, bufsize=0)
        except OSError as exc:
            log.error("cannot start PIO bridge: %s", exc)
            self._proc = None
            return False

        # Non-blocking reads: the daemon polls, it must never stall here.
        flags = fcntl.fcntl(self._proc.stdout, fcntl.F_GETFL)
        fcntl.fcntl(self._proc.stdout, fcntl.F_SETFL, flags | os.O_NONBLOCK)

        # Give the state machines a moment to come up before the first write.
        time.sleep(0.1)
        if self._proc.poll() is not None:
            log.error("PIO bridge exited immediately (rc=%s)", self._proc.returncode)
            self._proc = None
            return False

        log.info("PIO EXTUART up: TX GPIO%d, RX GPIO%d, %d baud",
                 self.tx_pin, self.rx_pin, self.baud)
        return True

    def close(self) -> None:
        if self._proc is None:
            return
        try:
            self._proc.terminate()
            self._proc.wait(timeout=2)
        except Exception:
            try:
                self._proc.kill()
            except Exception:
                pass
        self._proc = None

    @property
    def is_open(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    # -- transfer --------------------------------------------------------

    def write(self, data: bytes) -> bool:
        if not self.is_open and not self.open():
            return False
        try:
            self._proc.stdin.write(data)      # type: ignore[union-attr]
            self._proc.stdin.flush()          # type: ignore[union-attr]
            return True
        except (BrokenPipeError, OSError, ValueError) as exc:
            self.failures += 1
            log.error("PIO bridge write failed (%d failures): %s",
                      self.failures, exc)
            self.close()
            return False

    def read_bytes(self, size: int = 4096) -> bytes:
        if not self.is_open:
            return b""
        try:
            data = self._proc.stdout.read(size)   # type: ignore[union-attr]
            return data or b""
        except (BlockingIOError, InterruptedError):
            return b""
        except (OSError, ValueError) as exc:
            log.error("PIO bridge read failed: %s", exc)
            self.close()
            return b""

    def read_line(self) -> str | None:
        data = self.read_bytes()
        return data.decode("ascii", "replace") if data else None
