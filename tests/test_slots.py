"""Numbered storage slots, and the commands that drive them.

Slots exist so a canned hex string means the same thing every time it is
pasted. Most of what is tested here is that property holding under the things
that would break it: reboots, overwrites, corruption, and a recording that was
interrupted by power loss.
"""

import shutil
import struct
import tempfile
import unittest
import zlib

from tests.support import FakeMediaStore, MemoryLink, drain_experiment

from radcam.protocol import Config as ProtoConfig, Dispatcher, Err
from radcam.slots import (SLOT_EMPTY, SLOT_ERROR, SLOT_IMAGE, SLOT_MEDIA_BASE,
                          SLOT_RECORDING, SLOT_VIDEO, Slot, SlotStore)
from radcam.stp import lrt as L
from radcam.stp import packets as P
from radcam.stp.commands import encode_command_payload
from radcam.stp.experiment import Experiment, ExperimentConfig, StpOp

TARGET = 0xC7


class TestSlotStore(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.store = SlotStore(self.dir, count=8)

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def test_starts_empty(self):
        self.assertEqual(len(self.store.free_slots()), 8)
        self.assertEqual(self.store.summary()["slots_used"], 0)

    def test_store_and_read_round_trip(self):
        data = bytes(range(256)) * 4
        slot = self.store.store(2, data, SLOT_IMAGE, 640, 480)
        self.assertEqual(slot.size, len(data))
        self.assertEqual(slot.crc32, zlib.crc32(data) & 0xFFFFFFFF)
        self.assertEqual(self.store.read(2), data)

    def test_addresses_are_stable_across_captures(self):
        """The whole point: slot 3 is slot 3 regardless of history."""
        for _ in range(5):
            self.store.store(3, b"x" * 10, SLOT_IMAGE)
        self.assertEqual(self.store.get(3).index, 3)
        self.assertEqual(self.store.summary()["slots_used"], 1)

    def test_index_survives_a_restart(self):
        self.store.store(1, b"a" * 100, SLOT_IMAGE, 640, 480)
        self.store.store(4, b"b" * 50, SLOT_VIDEO, 1920, 1080, duration_s=7.5)
        reopened = SlotStore(self.dir, count=8)
        self.assertEqual(reopened.summary()["slots_used"], 2)
        self.assertEqual(reopened.get(4).duration_s, 7.5)
        self.assertEqual(reopened.read(1), b"a" * 100)

    def test_an_interrupted_recording_does_not_survive_a_restart(self):
        """Nothing was finalised, so the slot is free, not half-occupied."""
        self.store.mark_recording(6)
        self.assertEqual(self.store.get(6).kind, SLOT_RECORDING)
        reopened = SlotStore(self.dir, count=8)
        self.assertEqual(reopened.get(6).kind, SLOT_EMPTY)
        self.assertIn(6, reopened.free_slots())

    def test_corrupted_bytes_are_caught_before_they_are_downlinked(self):
        data = b"y" * 500
        self.store.store(0, data, SLOT_IMAGE)
        with open(f"{self.dir}/slot-00.bin", "r+b") as handle:
            handle.write(b"ZZZZ")
        self.assertIsNone(self.store.read(0))
        self.assertEqual(self.store.get(0).kind, SLOT_ERROR)

    def test_delete_frees_the_slot(self):
        self.store.store(5, b"z" * 10, SLOT_IMAGE)
        self.assertTrue(self.store.delete(5))
        self.assertIn(5, self.store.free_slots())
        self.assertIsNone(self.store.read(5))

    def test_deleting_an_empty_slot_reports_it(self):
        self.assertFalse(self.store.delete(5))

    def test_delete_all(self):
        for i in range(4):
            self.store.store(i, b"q", SLOT_IMAGE)
        self.assertEqual(self.store.delete_all(), 4)
        self.assertEqual(self.store.summary()["slots_used"], 0)

    def test_out_of_range_indices_are_refused(self):
        self.assertFalse(self.store.valid(8))
        self.assertFalse(self.store.valid(-1))
        self.assertIsNone(self.store.store(99, b"x", SLOT_IMAGE))
        self.assertFalse(self.store.delete(99))

    def test_media_id_namespace_is_distinct(self):
        self.store.store(7, b"w" * 10, SLOT_IMAGE)
        media_id = self.store.get(7).media_id
        self.assertEqual(media_id, SLOT_MEDIA_BASE | 7)
        self.assertEqual(self.store.read_by_media_id(media_id), b"w" * 10)
        # A legacy media id must not resolve to a slot.
        self.assertIsNone(self.store.read_by_media_id(7))

    def test_a_recording_with_no_limit_never_expires(self):
        self.store.mark_recording(2, limit_s=0.0)
        self.assertFalse(self.store.recording_expired())

    def test_a_negative_limit_is_treated_as_no_limit(self):
        """A corrupted duration must not end a recording instantly."""
        self.store.mark_recording(2, limit_s=-5.0)
        self.assertFalse(self.store.recording_expired())

    def test_a_recording_expires_once_its_limit_elapses(self):
        import time
        self.store.mark_recording(2, limit_s=0.05)
        self.assertFalse(self.store.recording_expired())
        time.sleep(0.08)
        self.assertTrue(self.store.recording_expired())
        self.assertGreater(self.store.recording_elapsed_s, 0.05)

    def test_nothing_expires_when_nothing_is_recording(self):
        self.assertFalse(self.store.recording_expired())
        self.assertEqual(self.store.recording_elapsed_s, 0.0)

    def test_packed_table_fits_the_lrt_response_window(self):
        store = SlotStore(self.dir, count=16)
        self.assertLessEqual(len(store.pack_table()), L.RESP_DATA_MAX)

    def test_bytes_used_tracks_content(self):
        self.store.store(0, b"a" * 100, SLOT_IMAGE)
        self.store.store(1, b"b" * 250, SLOT_VIDEO)
        self.assertEqual(self.store.bytes_used(), 350)


class TestSlotCommands(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.wire = P.Wire(target_id=TARGET)
        self.link = MemoryLink()
        self.media = FakeMediaStore()
        self.slots = SlotStore(self.dir, count=8)
        self.experiment = Experiment(
            link=self.link, wire=self.wire,
            dispatcher=Dispatcher(config=ProtoConfig(), store=self.media),
            store=self.media, slots=self.slots,
            config=ExperimentConfig(target_id=TARGET, scrub_interval_s=3600))
        self.experiment.start()
        self.seq = 0

    def tearDown(self):
        self.experiment.stop()
        shutil.rmtree(self.dir, ignore_errors=True)

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

    def test_slot_list_returns_the_table(self):
        self.command(StpOp.SLOT_LIST)
        report = self.poll()
        self.assertEqual(report["last_result"], 0)
        self.assertEqual(report["resp_data"][0], 8)      # slot count

    def test_slot_info_for_an_empty_slot(self):
        self.command(StpOp.SLOT_INFO, bytes([3]))
        report = self.poll()
        self.assertEqual(report["last_result"], 0)
        index, kind = struct.unpack_from("<BB", report["resp_data"], 0)
        self.assertEqual(index, 3)
        self.assertEqual(kind, SLOT_EMPTY)

    def test_out_of_range_slot_is_refused(self):
        self.command(StpOp.SLOT_INFO, bytes([99]))
        self.assertEqual(self.experiment.last_result, int(Err.BAD_PARAM))

    def test_download_of_an_empty_slot_is_refused(self):
        self.command(StpOp.SLOT_DOWNLOAD, bytes([0]))
        self.assertEqual(self.experiment.last_result, int(Err.NO_MEDIA))

    def test_download_queues_the_slot_for_hrt(self):
        self.slots.store(2, b"P" * 5000, SLOT_IMAGE, 640, 480)
        self.command(StpOp.SLOT_DOWNLOAD, bytes([2]))
        self.assertEqual(self.experiment.last_result, 0)
        self.assertEqual(self.experiment.transfers.pending, 1)
        status = self.experiment.transfers.status()
        self.assertEqual(status["xfer_media_id"], SLOT_MEDIA_BASE | 2)

    def test_telemetry_reports_which_slot_is_downloading(self):
        self.slots.store(4, b"Q" * 3000, SLOT_IMAGE)
        self.command(StpOp.SLOT_DOWNLOAD, bytes([4]))
        self.assertEqual(self.poll()["slot_downloading"], 4)

    def test_download_abort_clears_the_queue(self):
        self.slots.store(2, b"P" * 5000, SLOT_IMAGE)
        self.command(StpOp.SLOT_DOWNLOAD, bytes([2]))
        self.command(StpOp.SLOT_DOWNLOAD_ABORT, bytes([2]))
        self.assertEqual(self.experiment.transfers.pending, 0)

    def test_delete_frees_the_slot_and_cancels_its_transfer(self):
        self.slots.store(1, b"R" * 4000, SLOT_IMAGE)
        self.command(StpOp.SLOT_DOWNLOAD, bytes([1]))
        self.command(StpOp.SLOT_DELETE, bytes([1]))
        self.assertEqual(self.experiment.last_result, 0)
        self.assertIn(1, self.slots.free_slots())
        self.assertEqual(self.experiment.transfers.pending, 0)

    def test_delete_all(self):
        for i in range(3):
            self.slots.store(i, b"S" * 10, SLOT_IMAGE)
        self.command(StpOp.SLOT_DELETE_ALL)
        self.assertEqual(self.slots.summary()["slots_used"], 0)

    def test_corrupted_slot_reports_a_fault_rather_than_downlinking_it(self):
        self.slots.store(0, b"T" * 900, SLOT_IMAGE)
        with open(f"{self.dir}/slot-00.bin", "r+b") as handle:
            handle.write(b"XXXX")
        self.command(StpOp.SLOT_DOWNLOAD, bytes([0]))
        self.assertEqual(self.experiment.last_result, int(Err.EEPROM_FAULT))
        self.assertEqual(self.experiment.transfers.pending, 0)

    def test_capture_without_a_camera_reports_a_fault(self):
        self.command(StpOp.SLOT_CAPTURE_IMAGE, bytes([0]))
        self.assertEqual(self.experiment.last_result, int(Err.CAMERA_FAULT))

    def test_record_stop_with_nothing_recording_is_refused(self):
        self.command(StpOp.SLOT_RECORD_STOP)
        self.assertEqual(self.experiment.last_result, int(Err.BAD_PARAM))

    def test_slot_usage_is_downlinked(self):
        self.slots.store(0, b"U" * 1000, SLOT_IMAGE)
        self.slots.store(1, b"V" * 2000, SLOT_VIDEO)
        report = self.poll()
        self.assertEqual(report["slot_count"], 8)
        self.assertEqual(report["slots_used"], 2)
        self.assertEqual(report["slots_free"], 6)
        self.assertEqual(report["slot_bytes_used"], 3000)
        self.assertEqual(report["slot_recording"], -1)

    def test_a_slot_transfer_reassembles_over_hrt(self):
        data = bytes(range(256)) * 30
        self.slots.store(6, data, SLOT_IMAGE, 640, 480)
        self.command(StpOp.SLOT_DOWNLOAD, bytes([6]))
        self.link.dice_send(P.encode_short_request(
            P.PacketType.HRT_GO, 0, 0, self.wire, TARGET))

        from radcam.stp import hrt as H
        chunks, info = {}, None
        for _ in range(20):
            self.experiment.service()
            raw = self.link.dice_read()
            for i in range(len(raw) // 1288):
                payload = H.decode_hrt_payload(
                    raw[i * 1288:(i + 1) * 1288][6:6 + 1280])
                self.assertTrue(payload["data_crc_ok"])
                if payload["sub_type"] == H.SubType.MEDIA_INFO:
                    info = H.decode_media_info(payload["data"])
                elif payload["sub_type"] == H.SubType.MEDIA_DATA:
                    chunks[payload["chunk_index"]] = payload["data"]

        self.assertIsNotNone(info)
        self.assertEqual(info["media_id"], SLOT_MEDIA_BASE | 6)
        rebuilt = b"".join(chunks[i] for i in range(info["chunk_total"]))
        self.assertEqual(rebuilt[:info["size"]], data)


class TestCommandCatalogue(unittest.TestCase):
    """The generated hex strings are the ground's actual interface."""

    def test_every_command_has_a_unique_opcode(self):
        from radcam.stp.catalogue import CATALOGUE
        opcodes = [c.opcode for c in CATALOGUE]
        self.assertEqual(len(opcodes), len(set(opcodes)))

    def test_every_command_has_a_unique_name(self):
        from radcam.stp.catalogue import CATALOGUE
        names = [c.name for c in CATALOGUE]
        self.assertEqual(len(names), len(set(names)))

    def test_generated_hex_decodes_to_the_command_it_claims(self):
        from radcam.stp.catalogue import CATALOGUE, build_command_hex
        from radcam.stp.commands import decode_command_payload
        wire = P.Wire(target_id=TARGET)
        for spec in CATALOGUE:
            with self.subTest(spec.name):
                packet = bytes.fromhex(build_command_hex(spec, wire=wire))
                self.assertEqual(len(packet), 120)
                decoded = P.decode_command(packet, wire)
                self.assertEqual(decoded.target_id, TARGET)
                request = decode_command_payload(decoded.payload, wire.crc)
                self.assertEqual(request.opcode, spec.opcode)
                self.assertEqual(len(request.args), sum(f.size
                                                        for f in spec.fields))

    def test_canned_commands_carry_the_force_flag(self):
        """Otherwise pasting the same string twice would run it once."""
        from radcam.stp.catalogue import CATALOGUE, build_command_hex
        from radcam.stp.commands import decode_command_payload
        wire = P.Wire(target_id=TARGET)
        spec = CATALOGUE[0]
        packet = bytes.fromhex(build_command_hex(spec, wire=wire))
        request = decode_command_payload(
            P.decode_command(packet, wire).payload, wire.crc)
        self.assertTrue(request.force)

    def test_argument_values_reach_the_payload(self):
        from radcam.stp.catalogue import build_command_hex, find
        from radcam.stp.commands import decode_command_payload
        wire = P.Wire(target_id=TARGET)
        spec = find("SLOT_DOWNLOAD")
        packet = bytes.fromhex(build_command_hex(spec, {"slot": 5}, wire=wire))
        request = decode_command_payload(
            P.decode_command(packet, wire).payload, wire.crc)
        self.assertEqual(request.args, bytes([5]))

    def test_short_packets_are_fourteen_bytes_and_valid(self):
        from radcam.stp.catalogue import SHORT_PACKETS, build_short_hex
        wire = P.Wire(target_id=TARGET)
        for name, ptype, _ in SHORT_PACKETS:
            with self.subTest(name):
                packet = bytes.fromhex(build_short_hex(ptype, wire))
                self.assertEqual(len(packet), 14)
                self.assertEqual(
                    P.decode_short_request(packet, wire).packet_type, ptype)


if __name__ == "__main__":
    unittest.main()
