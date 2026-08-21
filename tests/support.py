"""Shared test helpers: an in-memory EEPROM and paths."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from radcam.eeprom import CameraEEPROM, EEPROM_SIZE, PAGE_SIZE   # noqa: E402


class FakeEEPROM(CameraEEPROM):
    """CameraEEPROM backed by a bytearray instead of I2C.

    Subclassing at the raw read/write boundary keeps every layer above it -
    framing, CRC, majority vote, repair, merge - under test as the real code,
    which is where the bugs would be. A mock of `load`/`store` would test
    nothing.
    """

    def __init__(self, fill: int = 0xFF):
        super().__init__(bus=-1)
        self.mem = bytearray([fill]) * EEPROM_SIZE
        self.writes = 0

    def read(self, offset: int, length: int) -> bytes:
        return bytes(self.mem[offset:offset + length])

    def write(self, offset: int, data: bytes) -> None:
        # Emulate the page-wrap behaviour of a real 24C64: a write that runs
        # past a page boundary wraps to the start of the same page rather than
        # carrying into the next one. If the driver ever stops chunking, the
        # tests should see corruption rather than silently passing.
        i = 0
        while i < len(data):
            page = (offset + i) // PAGE_SIZE
            pos = (offset + i) % PAGE_SIZE
            space = PAGE_SIZE - pos
            chunk = data[i:i + space]
            base = page * PAGE_SIZE
            for j, b in enumerate(chunk):
                self.mem[base + ((pos + j) % PAGE_SIZE)] = b
            i += len(chunk)
        self.writes += 1

    def present(self) -> bool:
        return True

    def corrupt(self, offset: int, count: int = 4) -> None:
        for i in range(count):
            self.mem[offset + i] ^= 0xFF


# --------------------------------------------------------------- STP helpers

import os          # noqa: E402
import time        # noqa: E402
from dataclasses import dataclass    # noqa: E402


class MemoryLink:
    """A two-ended in-memory stand-in for `radcam.stp.link.Rs422Link`.

    Presents the experiment-facing half of the interface (`read`, `read_wait`,
    `send`, `tx_packets`) while letting a test play DICE through `dice_send`
    and `dice_read`. Nothing here simulates timing: the point is to exercise
    the protocol logic deterministically, not the UART.
    """

    def __init__(self):
        self.to_experiment = bytearray()
        self.to_dice = bytearray()
        self.tx_packets = 0
        self.tx_errors = 0
        self.tx_aborted = 0
        self.fail_next = 0

    # experiment side
    def read(self, size: int = 8192) -> bytes:
        data = bytes(self.to_experiment)
        self.to_experiment.clear()
        return data

    def read_wait(self, timeout_s: float = 0.005, size: int = 8192) -> bytes:
        return self.read(size)

    def send(self, data: bytes, abort_check=None) -> bool:
        """Mirrors `Rs422Link.send`, including the mid-packet abort check.

        The abort path is polled once here rather than repeatedly: there is no
        real transmission to interrupt, so one check is enough to exercise the
        decision without inventing a fake drain.
        """
        if self.fail_next:
            self.fail_next -= 1
            self.tx_errors += 1
            return False
        if abort_check is not None:
            try:
                if abort_check():
                    self.tx_aborted += 1
                    return False
            except Exception:                          # noqa: BLE001
                pass
        self.to_dice += data
        self.tx_packets += 1
        return True

    # DICE side
    def dice_send(self, data: bytes) -> None:
        self.to_experiment += data

    def dice_read(self) -> bytes:
        data = bytes(self.to_dice)
        self.to_dice.clear()
        return data


@dataclass
class MediaRecordStub:
    media_id: int
    kind: str
    size: int
    width: int
    height: int
    created_unix: float


class FakeMediaStore:
    """Media store with deterministic contents, for transfer tests."""

    def __init__(self, blobs: dict | None = None):
        self.blobs = blobs if blobs is not None else {
            1: bytes(range(256)) * 20,        # 5120 bytes, 5 chunks
            2: b"\xa5" * 100,                 # single chunk
            # Large enough that it cannot complete in one service pass, which
            # is what mid-transfer tests (flow control, rewind) need.
            3: bytes(range(256)) * 240,       # 61440 bytes, 49 chunks
        }

    def read(self, media_id: int):
        return self.blobs.get(media_id)

    def list(self):
        return [MediaRecordStub(k, "image", len(v), 1920, 1080, 0.0)
                for k, v in sorted(self.blobs.items())]

    def delete(self, media_id: int) -> bool:
        return self.blobs.pop(media_id, None) is not None


def drain_experiment(experiment, passes: int = 12, settle: float = 0.02):
    """Run service passes, giving the command worker time to finish.

    Commands execute on a worker thread by design (the ACK means "accepted",
    not "done"), so a test that checks a result immediately after one service
    pass is racing the worker.
    """
    for _ in range(passes):
        experiment.service()
        time.sleep(settle)
