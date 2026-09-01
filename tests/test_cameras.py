"""Camera selection: the interlock, not the convenience.

More than one camera on the same CSI/I2C fabric is only safe if exactly one is
enabled. Two sensors sharing an I2C address answer together; two driving the
same lanes contend. Most of what is tested here is that the unsafe state cannot
be reached - not by a bad command, not by a partial failure, not by whatever
the pins happened to be holding at reset.
"""

import struct
import unittest

from tests.support import FakeMediaStore, MemoryLink, drain_experiment

from radcam.cameras import (MAX_CAMERAS, NO_CAMERA, CameraEntry,
                            CameraSelector, entries_from_config)
from radcam.protocol import Config as ProtoConfig, Dispatcher, Err
from radcam.stp import lrt as L
from radcam.stp import packets as P
from radcam.stp.commands import encode_command_payload
from radcam.stp.experiment import Experiment, ExperimentConfig, StpOp

TARGET = 0xC7


class FakeRequest:
    """Stands in for a gpiod line request, recording every level set."""

    def __init__(self):
        self.levels = {}
        self.released = False
        self.fail_on = set()

    def set_value(self, gpio, value):
        if gpio in self.fail_on:
            raise OSError(f"simulated failure on GPIO{gpio}")
        self.levels[gpio] = value

    def release(self):
        self.released = True


def selector(count=4, active_high=True):
    entries = [CameraEntry(i, 30 + i, f"CAM{i}", active_high=active_high,
                           i2c_bus=4 + i) for i in range(count)]
    sel = CameraSelector(entries)
    sel._requests = {i: FakeRequest() for i in range(count)}
    shared = FakeRequest()
    sel._requests = {i: shared for i in range(count)}
    from gpiod.line import Value
    for entry in entries:
        sel._requests[entry.index].levels[entry.gpio] = (Value.INACTIVE if active_high
                                           else Value.ACTIVE)
    return sel


def enabled(sel):
    """Which camera indices are currently driven on."""
    from gpiod.line import Value
    out = []
    for index, entry in sel.entries.items():
        level = sel._requests[index].levels.get(entry.gpio) if index in sel._requests else None
        on = (level == Value.ACTIVE) if entry.active_high \
            else (level == Value.INACTIVE)
        if on:
            out.append(index)
    return sorted(out)


class TestSelector(unittest.TestCase):
    def test_ceiling_is_sixteen(self):
        self.assertEqual(MAX_CAMERAS, 16)

    def test_indices_outside_the_range_are_refused_at_construction(self):
        sel = CameraSelector([CameraEntry(0, 30), CameraEntry(16, 31),
                              CameraEntry(-1, 32)])
        self.assertEqual(sorted(sel.entries), [0])

    def test_everything_starts_disabled(self):
        self.assertEqual(enabled(selector()), [])

    def test_select_enables_exactly_one(self):
        sel = selector()
        self.assertTrue(sel.select(2, settle_s=0))
        self.assertEqual(enabled(sel), [2])
        self.assertEqual(sel.active, 2)

    def test_selecting_another_disables_the_first(self):
        sel = selector()
        sel.select(1, settle_s=0)
        sel.select(3, settle_s=0)
        self.assertEqual(enabled(sel), [3], "two cameras were left enabled")

    def test_every_camera_in_turn_leaves_only_one_on(self):
        sel = selector(count=8)
        for index in range(8):
            sel.select(index, settle_s=0)
            self.assertEqual(enabled(sel), [index])

    def test_no_camera_disables_everything(self):
        sel = selector()
        sel.select(1, settle_s=0)
        self.assertTrue(sel.select(NO_CAMERA, settle_s=0))
        self.assertEqual(enabled(sel), [])
        self.assertEqual(sel.active, NO_CAMERA)

    def test_unknown_index_is_refused_and_changes_nothing(self):
        sel = selector()
        sel.select(1, settle_s=0)
        self.assertFalse(sel.select(9, settle_s=0))
        self.assertEqual(enabled(sel), [1])
        self.assertEqual(sel.failures, 1)

    def test_a_failed_enable_leaves_everything_off(self):
        """Disables happen first, so a partial failure is quiet, not contended."""
        sel = selector()
        sel.select(0, settle_s=0)
        sel._requests[2].fail_on = {sel.entries[2].gpio}
        self.assertFalse(sel.select(2, settle_s=0))
        self.assertEqual(enabled(sel), [],
                         "a failed switch must not leave the old camera on")

    def test_active_low_enables_are_honoured(self):
        sel = selector(active_high=False)
        sel.select(1, settle_s=0)
        self.assertEqual(enabled(sel), [1])

    def test_close_disables_everything(self):
        sel = selector()
        sel.select(1, settle_s=0)
        sel.close()
        self.assertTrue(sel.is_open is False)

    def test_summary_reports_state(self):
        sel = selector()
        sel.select(2, settle_s=0)
        summary = sel.summary()
        self.assertEqual(summary["camera_count"], 4)
        self.assertEqual(summary["camera_active"], 2)
        self.assertEqual(summary["camera_selections"], 1)

    def test_packed_table_round_trips(self):
        sel = selector(count=3)
        sel.select(1, settle_s=0)
        blob = sel.pack_table()
        count, active = struct.unpack_from("<BB", blob, 0)
        self.assertEqual(count, 3)
        self.assertEqual(active, 1)
        offset = 2
        seen = []
        for _ in range(count):
            index, gpio, flags, bus, namelen = struct.unpack_from(
                "<BBBbB", blob, offset)
            offset += 5
            name = blob[offset:offset + namelen].decode()
            offset += namelen
            seen.append((index, gpio, bool(flags & 0x02), name))
        self.assertEqual([s[0] for s in seen], [0, 1, 2])
        self.assertEqual([s[2] for s in seen], [False, True, False])
        self.assertEqual(seen[1][3], "CAM1")

    def test_table_fits_the_lrt_response_window(self):
        sel = selector(count=MAX_CAMERAS)
        self.assertLessEqual(len(sel.pack_table()), L.RESP_DATA_MAX)

    def test_config_parsing_skips_bad_entries(self):
        entries = entries_from_config([
            {"index": 0, "gpio": 35, "name": "CAM0"},
            {"index": 1},                                  # missing gpio
            {"index": 2, "gpio": 48, "active_high": False},
        ])
        self.assertEqual([e.index for e in entries], [0, 2])
        self.assertFalse(entries[1].active_high)


class TestCameraCommands(unittest.TestCase):
    def setUp(self):
        self.wire = P.Wire(target_id=TARGET)
        self.link = MemoryLink()
        self.sel = selector(count=4)
        self.experiment = Experiment(
            link=self.link, wire=self.wire,
            dispatcher=Dispatcher(config=ProtoConfig()),
            store=FakeMediaStore(), cameras=self.sel,
            config=ExperimentConfig(target_id=TARGET, scrub_interval_s=3600))
        self.experiment.start()
        self.seq = 0

    def tearDown(self):
        self.experiment.stop()

    def command(self, opcode, args=b""):
        self.seq += 1
        self.link.dice_send(P.encode_command(
            encode_command_payload(opcode, self.seq, args), 0, 0,
            self.wire, TARGET))
        drain_experiment(self.experiment, passes=3)
        self.link.dice_read()

    def poll(self):
        self.link.dice_read()
        self.link.dice_send(P.encode_short_request(
            P.PacketType.LRT_REQUEST, 0, 0, self.wire, TARGET))
        self.experiment.service()
        raw = self.link.dice_read()
        at = raw.find(self.wire.sync_bytes)
        return L.decode_lrt_payload(raw[at + 6:at + 6 + 1248]) if at >= 0 else None

    def test_select_enables_only_that_camera(self):
        self.command(StpOp.SELECT_CAMERA, bytes([2]))
        self.assertEqual(self.experiment.last_result, 0)
        self.assertEqual(enabled(self.sel), [2])

    def test_switching_disables_the_previous(self):
        self.command(StpOp.SELECT_CAMERA, bytes([0]))
        self.command(StpOp.SELECT_CAMERA, bytes([3]))
        self.assertEqual(enabled(self.sel), [3])

    def test_disable_all(self):
        self.command(StpOp.SELECT_CAMERA, bytes([1]))
        self.command(StpOp.SELECT_CAMERA, bytes([NO_CAMERA]))
        self.assertEqual(enabled(self.sel), [])

    def test_index_beyond_the_ceiling_is_refused(self):
        self.command(StpOp.SELECT_CAMERA, bytes([MAX_CAMERAS]))
        self.assertEqual(self.experiment.last_result, int(Err.BAD_PARAM))

    def test_unconfigured_index_reports_a_camera_fault(self):
        self.command(StpOp.SELECT_CAMERA, bytes([9]))
        self.assertEqual(self.experiment.last_result, int(Err.CAMERA_FAULT))

    def test_missing_argument_is_refused(self):
        self.command(StpOp.SELECT_CAMERA)
        self.assertEqual(self.experiment.last_result, int(Err.BAD_PARAM))

    def test_camera_list_returns_the_table(self):
        self.command(StpOp.SELECT_CAMERA, bytes([1]))
        self.command(StpOp.CAMERA_LIST)
        report = self.poll()
        self.assertEqual(report["last_result"], 0)
        self.assertEqual(report["resp_data"][0], 4)     # four cameras
        self.assertEqual(report["resp_data"][1], 1)     # camera 1 active

    def test_telemetry_reports_the_active_camera(self):
        self.command(StpOp.SELECT_CAMERA, bytes([3]))
        report = self.poll()
        self.assertEqual(report["camera_count"], 4)
        self.assertEqual(report["camera_active"], 3)
        self.assertGreaterEqual(report["camera_selections"], 1)

    def test_telemetry_shows_none_active_before_selection(self):
        self.assertEqual(self.poll()["camera_active"], NO_CAMERA)

    def test_switching_stops_a_running_stream(self):
        """The stream points at hardware that is about to be switched off."""
        class FakeStream:
            running = True
            fault = None
            frames_encoded = 1
            frames_dropped = 0
            frames_taken = 0
            bytes_encoded = 0
            stopped = False

            def stop(self):
                self.stopped = True
                self.running = False
                return True

            def take(self):
                return None

            def flush(self, keep_keyframe=False):
                return 0

            def encoder_late(self):
                return False

            @property
            def queue_depth(self):
                return 0

            def status(self):
                return {}

        fake = FakeStream()
        self.experiment.stream = fake
        self.command(StpOp.SELECT_CAMERA, bytes([1]))
        self.assertTrue(fake.stopped, "stream kept running across a switch")

    def test_no_selector_reports_a_camera_fault(self):
        experiment = Experiment(
            link=MemoryLink(), wire=self.wire,
            config=ExperimentConfig(target_id=TARGET, scrub_interval_s=3600))
        experiment.start()
        try:
            experiment.link.dice_send(P.encode_command(
                encode_command_payload(StpOp.SELECT_CAMERA, 1, bytes([0])),
                0, 0, self.wire, TARGET))
            drain_experiment(experiment, passes=3)
            self.assertEqual(experiment.last_result, int(Err.CAMERA_FAULT))
        finally:
            experiment.stop()


if __name__ == "__main__":
    unittest.main()
