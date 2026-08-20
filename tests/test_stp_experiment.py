"""The experiment state machine, LRT/HRT payloads, and TMR state.

These tests are about behaviour on a shared bus, so most of them assert what
the payload does *not* do: it does not answer another target, does not transmit
outside an HRT window, does not execute a retransmitted command twice, and does
not go silent when it enters safe mode.
"""

import struct
import time
import unittest
import zlib

from tests.support import FakeMediaStore, MemoryLink, drain_experiment

from radcam.protocol import Config as ProtoConfig, Dispatcher, Err, Msg
from radcam.stp import hrt as H
from radcam.stp import lrt as L
from radcam.stp import packets as P
from radcam.stp.commands import (FLAG_FORCE, CommandDecodeError,
                                 decode_command_payload,
                                 encode_command_payload)
from radcam.stp.experiment import Experiment, ExperimentConfig, StpOp
from radcam.stp.redundancy import (Scrubber, TMRBool, TMRInt,
                                   TMRUnrecoverable)
from radcam.stp.timebase import dice_to_unix, unix_to_dice

TARGET = 3


class ExperimentFixture(unittest.TestCase):
    def setUp(self):
        self.wire = P.Wire(target_id=TARGET)
        self.link = MemoryLink()
        self.store = FakeMediaStore()
        self.experiment = Experiment(
            link=self.link, wire=self.wire,
            dispatcher=Dispatcher(config=ProtoConfig(), store=self.store),
            store=self.store,
            config=ExperimentConfig(target_id=TARGET, scrub_interval_s=3600))
        self.experiment.start()
        self.seq = 0

    def tearDown(self):
        self.experiment.stop()

    # -- DICE side helpers ------------------------------------------------

    def send_command(self, opcode, args=b"", flags=0, seq=None, target=TARGET):
        self.seq = (self.seq + 1) if seq is None else seq
        payload = encode_command_payload(opcode, self.seq, args, flags)
        self.link.dice_send(P.encode_command(payload, 1000, 5, self.wire,
                                             target))
        return self.seq

    def send_short(self, ptype, target=TARGET):
        self.link.dice_send(
            P.encode_short_request(ptype, 1000, 5, self.wire, target))

    def replies(self):
        raw = self.link.dice_read()
        out, i = [], 0
        while i < len(raw):
            at = raw.find(self.wire.sync_bytes, i)
            if at < 0 or at + 6 > len(raw):
                break
            ptype = raw[at + 4]
            if ptype == P.PacketType.COMMAND_ACK and at + 8 <= len(raw):
                out.append(("ACK", raw[at:at + 8]))
                i = at + 8
            elif ptype == P.PacketType.LRT_DATA and at + 1256 <= len(raw):
                out.append(("LRT", raw[at:at + 1256]))
                i = at + 1256
            elif ptype == P.PacketType.HRT_DATA and at + 1288 <= len(raw):
                out.append(("HRT", raw[at:at + 1288]))
                i = at + 1288
            else:
                i = at + 4
        return out

    def poll_lrt(self):
        drain_experiment(self.experiment, passes=4)
        self.link.dice_read()
        self.send_short(P.PacketType.LRT_REQUEST)
        self.experiment.service()
        for kind, raw in self.replies():
            if kind == "LRT":
                return L.decode_lrt_payload(raw[6:6 + 1248])
        return None


class TestCommandAcknowledge(ExperimentFixture):
    def test_valid_command_is_acknowledged_immediately(self):
        self.send_command(Msg.PING)
        self.experiment.service()
        kinds = [k for k, _ in self.replies()]
        self.assertEqual(kinds, ["ACK"])

    def test_ack_is_well_formed(self):
        self.send_command(Msg.PING)
        self.experiment.service()
        _, ack = self.replies()[0]
        self.assertEqual(len(ack), 8)
        self.assertEqual(ack[0:4], self.wire.sync_bytes)
        self.assertEqual(ack[4], P.PacketType.COMMAND_ACK)
        self.assertEqual(ack[5], TARGET)
        self.assertTrue(self.wire.check_crc(ack, 6))

    def test_command_for_another_target_is_silently_ignored(self):
        self.send_command(Msg.PING, target=TARGET + 1)
        self.experiment.service()
        self.assertEqual(self.replies(), [])

    def test_malformed_inner_payload_is_still_acknowledged(self):
        """The ICD ties the ACK to the envelope, not to our payload format."""
        bad = bytearray(encode_command_payload(Msg.PING, 5))
        # The inner CRC covers payload[0:5] plus the declared args, so the
        # trailing padding is deliberately outside it. Corrupt cmd_seq.
        bad[1] ^= 0xFF
        self.link.dice_send(P.encode_command(bytes(bad), 0, 0, self.wire,
                                             TARGET))
        self.experiment.service()
        self.assertEqual([k for k, _ in self.replies()], ["ACK"])
        report = self.poll_lrt()
        self.assertEqual(report["last_result"], int(Err.BAD_PARAM))

    def test_result_appears_in_lrt_keyed_by_cmd_seq(self):
        seq = self.send_command(Msg.PING)
        report = self.poll_lrt()
        self.assertEqual(report["last_cmd_seq"], seq)
        self.assertEqual(report["last_result"], 0)

    def test_response_data_rides_back_in_lrt(self):
        self.send_command(Msg.GET_CONFIG)
        report = self.poll_lrt()
        self.assertEqual(report["resp_opcode"], int(Msg.CONFIG_REPORT))
        self.assertTrue(len(report["resp_data"]) > 0)
        self.assertFalse(report["resp_truncated"])

    def test_unknown_opcode_reports_an_error_not_a_crash(self):
        self.send_command(0x7E)
        report = self.poll_lrt()
        self.assertNotEqual(report["last_result"], 0)


class TestDuplicateSuppression(ExperimentFixture):
    def test_retransmitted_command_executes_once(self):
        before = self.experiment._cmds_executed.value()
        for _ in range(3):
            self.send_command(Msg.PING, seq=42)
            self.experiment.service()
        drain_experiment(self.experiment)
        self.assertEqual(self.experiment._cmds_executed.value() - before, 1)

    def test_every_copy_is_still_acknowledged(self):
        for _ in range(3):
            self.send_command(Msg.PING, seq=43)
            self.experiment.service()
        self.assertEqual(len([k for k, _ in self.replies() if k == "ACK"]), 3)

    def test_force_flag_re_executes(self):
        before = self.experiment._cmds_executed.value()
        for _ in range(2):
            self.send_command(Msg.PING, seq=44, flags=FLAG_FORCE)
            drain_experiment(self.experiment, passes=3)
        self.assertEqual(self.experiment._cmds_executed.value() - before, 2)

    def test_distinct_sequence_numbers_both_execute(self):
        before = self.experiment._cmds_executed.value()
        self.send_command(Msg.PING, seq=50)
        self.send_command(Msg.PING, seq=51)
        drain_experiment(self.experiment)
        self.assertEqual(self.experiment._cmds_executed.value() - before, 2)


class TestHrtFlowControl(ExperimentFixture):
    def request_media(self, media_id=1):
        self.send_command(Msg.REQUEST_MEDIA, struct.pack("<I", media_id))
        drain_experiment(self.experiment)
        self.link.dice_read()

    def test_nothing_is_transmitted_before_hrt_go(self):
        self.request_media()
        for _ in range(5):
            self.experiment.service()
        self.assertEqual([k for k, _ in self.replies() if k == "HRT"], [])

    def test_go_then_stop_bounds_the_transmission(self):
        self.request_media()
        self.send_short(P.PacketType.HRT_GO)
        self.experiment.service()
        self.assertGreater(len([k for k, _ in self.replies() if k == "HRT"]), 0)

        self.send_short(P.PacketType.HRT_STOP)
        self.experiment.service()
        self.link.dice_read()
        for _ in range(5):
            self.experiment.service()
        self.assertEqual(self.replies(), [])

    def test_full_file_reassembles_and_verifies(self):
        self.request_media(1)
        self.send_short(P.PacketType.HRT_GO)

        chunks, info, saw_end = {}, None, False
        for _ in range(40):
            self.experiment.service()
            for kind, raw in self.replies():
                if kind != "HRT":
                    continue
                payload = H.decode_hrt_payload(raw[6:6 + 1280])
                self.assertTrue(payload["data_crc_ok"])
                if payload["sub_type"] == H.SubType.MEDIA_INFO:
                    info = H.decode_media_info(payload["data"])
                elif payload["sub_type"] == H.SubType.MEDIA_DATA:
                    chunks[payload["chunk_index"]] = payload["data"]
                elif payload["sub_type"] == H.SubType.MEDIA_END:
                    saw_end = True
            if saw_end:
                break

        self.assertIsNotNone(info)
        self.assertTrue(saw_end)
        self.assertEqual(len(chunks), info["chunk_total"])
        blob = b"".join(chunks[i] for i in range(info["chunk_total"]))
        self.assertEqual(blob[:info["size"]], self.store.blobs[1])
        self.assertEqual(zlib.crc32(blob[:info["size"]]) & 0xFFFFFFFF,
                         info["file_crc32"])

    def test_stop_with_loss_rewinds_the_send_pointer(self):
        self.request_media(3)          # too large to finish in one pass
        self.send_short(P.PacketType.HRT_GO)
        self.experiment.service()
        advanced = self.experiment.transfers.status()["xfer_chunk_next"]
        self.assertGreater(advanced, 0)

        self.send_short(P.PacketType.HRT_STOP_WITH_LOSS)
        self.experiment.service()
        self.assertEqual(self.experiment.transfers.status()["xfer_chunk_next"],
                         advanced - 1)
        self.assertEqual(self.experiment.transfers.loss_events, 1)

    def test_resend_after_completion_is_honoured(self):
        self.request_media(2)                    # single-chunk file
        self.send_short(P.PacketType.HRT_GO)
        for _ in range(10):
            self.experiment.service()
        # Close the tap first: with HRT still open the resent chunks would go
        # out during the drain below and never be observed.
        self.send_short(P.PacketType.HRT_STOP)
        self.experiment.service()
        self.link.dice_read()

        self.send_command(Msg.RESEND,
                          struct.pack("<II", 2, 0))
        drain_experiment(self.experiment)
        self.link.dice_read()
        self.send_short(P.PacketType.HRT_GO)
        self.experiment.service()

        resent = [H.decode_hrt_payload(raw[6:6 + 1280])
                  for kind, raw in self.replies() if kind == "HRT"]
        data = [p for p in resent if p["sub_type"] == H.SubType.MEDIA_DATA]
        self.assertTrue(data)
        self.assertTrue(data[0]["retransmit"])

    def test_request_for_missing_media_is_refused(self):
        self.send_command(Msg.REQUEST_MEDIA, struct.pack("<I", 999))
        report = self.poll_lrt()
        self.assertEqual(report["last_result"], int(Err.NO_MEDIA))


class TestSafeMode(ExperimentFixture):
    def trip(self):
        for i in range(self.experiment.cfg.safe_mode_threshold):
            self.send_command(Msg.REQUEST_MEDIA,
                              struct.pack("<I", 900 + i))
            drain_experiment(self.experiment, passes=3)

    def test_repeated_failures_trip_safe_mode(self):
        self.assertFalse(self.experiment._safe_mode.value())
        self.trip()
        self.assertTrue(self.experiment._safe_mode.value())

    def test_lrt_keeps_answering_in_safe_mode(self):
        self.trip()
        report = self.poll_lrt()
        self.assertIsNotNone(report)
        self.assertTrue(report["safe_mode"])

    def test_hrt_go_is_refused_in_safe_mode(self):
        self.trip()
        self.send_short(P.PacketType.HRT_GO)
        self.experiment.service()
        self.assertFalse(self.experiment._hrt_enabled.value())

    def test_ground_can_clear_safe_mode(self):
        self.trip()
        self.send_command(StpOp.CLEAR_SAFE_MODE)
        drain_experiment(self.experiment)
        self.assertFalse(self.experiment._safe_mode.value())

    def test_link_stats_are_readable_while_in_safe_mode(self):
        self.trip()
        self.send_command(StpOp.GET_LINK_STATS)
        report = self.poll_lrt()
        self.assertEqual(report["last_result"], 0)
        self.assertEqual(len(report["resp_data"]), 32)


class TestLrtPayload(unittest.TestCase):
    def test_payload_is_exactly_the_icd_length(self):
        self.assertEqual(len(L.build_lrt_payload({})), 1248)

    def test_event_ring_fits_inside_the_payload(self):
        self.assertLessEqual(L.OFF_EVENTS + L.MAX_EVENTS * L.EVENT_SIZE,
                             L.OFF_PAYLOAD_CRC32)

    def test_round_trip(self):
        state = {"uptime_s": 99, "dose_rad": 1.25, "cpu_temp_c": 42.5,
                 "storage_free": 2**40, "media_count": 7, "hrt_enabled": True,
                 "last_cmd_seq": 1234, "safe_mode": True}
        decoded = L.decode_lrt_payload(L.build_lrt_payload(state))
        self.assertTrue(decoded["payload_crc_ok"])
        self.assertEqual(decoded["uptime_s"], 99)
        self.assertAlmostEqual(decoded["dose_rad"], 1.25, places=5)
        self.assertEqual(decoded["storage_free"], 2**40)
        self.assertTrue(decoded["hrt_enabled"])
        self.assertTrue(decoded["safe_mode"])

    def test_missing_keys_do_not_break_the_frame(self):
        """A telemetry frame that cannot be built takes housekeeping down."""
        decoded = L.decode_lrt_payload(L.build_lrt_payload({}))
        self.assertTrue(decoded["payload_crc_ok"])
        self.assertEqual(decoded["uptime_s"], 0)

    def test_triplicated_dose_survives_a_single_bit_flip(self):
        payload = bytearray(L.build_lrt_payload({"dose_rad": 3.5}))
        payload[L.OFF_DOSE_RAD + 1] ^= 0xFF
        decoded = L.decode_lrt_payload(bytes(payload))
        self.assertFalse(decoded["payload_crc_ok"])     # damage is visible
        self.assertAlmostEqual(decoded["dose_rad"], 3.5, places=5)

    def test_oversized_response_is_flagged_and_truncated(self):
        decoded = L.decode_lrt_payload(
            L.build_lrt_payload({"resp_data": b"x" * 900}))
        self.assertTrue(decoded["resp_truncated"])
        self.assertEqual(decoded["resp_full_len"], 900)
        self.assertEqual(len(decoded["resp_data"]), L.RESP_DATA_MAX)

    def test_event_ring_keeps_the_newest(self):
        log = L.EventLog()
        for i in range(L.MAX_EVENTS + 30):
            log.add(L.EventCode.CAPTURE_OK, arg=i)
        decoded = L.decode_lrt_payload(
            L.build_lrt_payload({}, log.recent(L.MAX_EVENTS)))
        self.assertEqual(len(decoded["events"]), L.MAX_EVENTS)
        self.assertEqual(decoded["events"][-1].arg, L.MAX_EVENTS + 29)


class TestHrtPayload(unittest.TestCase):
    def test_payload_is_exactly_the_icd_length(self):
        self.assertEqual(len(H.build_hrt_payload(H.SubType.IDLE)), 1280)

    def test_chunk_capacity(self):
        self.assertEqual(H.HRT_CHUNK_DATA, 1256)

    def test_round_trip_and_crc(self):
        decoded = H.decode_hrt_payload(
            H.build_hrt_payload(H.SubType.MEDIA_DATA, 5, 2, 9, b"hello"))
        self.assertEqual(decoded["media_id"], 5)
        self.assertEqual(decoded["chunk_index"], 2)
        self.assertEqual(decoded["data"], b"hello")
        self.assertTrue(decoded["data_crc_ok"])

    def test_corrupted_chunk_is_detected(self):
        payload = bytearray(
            H.build_hrt_payload(H.SubType.MEDIA_DATA, 1, 0, 1, b"abcdef"))
        payload[H.HRT_HEADER_LEN] ^= 0xFF
        self.assertFalse(H.decode_hrt_payload(bytes(payload))["data_crc_ok"])

    def test_oversized_chunk_is_refused(self):
        with self.assertRaises(ValueError):
            H.build_hrt_payload(H.SubType.MEDIA_DATA,
                                data=b"x" * (H.HRT_CHUNK_DATA + 1))

    def test_reload_hook_revives_a_forgotten_transfer(self):
        blob = bytes(range(256)) * 8
        manager = H.TransferManager(retain=0, reload=lambda mid: blob)
        self.assertEqual(manager.request_resend(77, [0, 1]), 2)
        decoded = H.decode_hrt_payload(manager.next_payload())
        self.assertEqual(decoded["sub_type"], H.SubType.MEDIA_DATA)
        self.assertTrue(decoded["retransmit"])

    def test_queue_is_bounded(self):
        manager = H.TransferManager(max_queue=2)
        self.assertTrue(manager.enqueue(1, b"a"))
        self.assertTrue(manager.enqueue(2, b"b"))
        self.assertFalse(manager.enqueue(3, b"c"))


class TestCommandPayload(unittest.TestCase):
    def test_payload_is_exactly_105_bytes(self):
        self.assertEqual(len(encode_command_payload(1, 2, b"abc")), 105)

    def test_round_trip(self):
        request = decode_command_payload(
            encode_command_payload(0x30, 4242, b"\x01\x02", FLAG_FORCE))
        self.assertEqual(request.opcode, 0x30)
        self.assertEqual(request.cmd_seq, 4242)
        self.assertEqual(request.args, b"\x01\x02")
        self.assertTrue(request.force)

    def test_inner_crc_catches_corruption(self):
        payload = bytearray(encode_command_payload(0x30, 1, b"abcdef"))
        payload[10] ^= 0xFF
        with self.assertRaises(CommandDecodeError):
            decode_command_payload(bytes(payload))

    def test_zero_crc_disables_the_check(self):
        payload = encode_command_payload(0x30, 1, b"abc", with_crc=False)
        self.assertEqual(decode_command_payload(payload).args, b"abc")

    def test_args_beyond_capacity_are_refused(self):
        with self.assertRaises(ValueError):
            encode_command_payload(1, 2, b"x" * 99)

    def test_declared_length_beyond_capacity_is_refused(self):
        payload = bytearray(encode_command_payload(1, 2, b"", with_crc=False))
        payload[3] = 200                               # arg_len out of range
        with self.assertRaises(CommandDecodeError):
            decode_command_payload(bytes(payload))


class TestRedundancy(unittest.TestCase):
    def test_single_bit_flip_is_corrected_and_repaired(self):
        cell = TMRInt(0x12345678, 4, "t")
        cell._copies[1][2] ^= 0x40
        self.assertEqual(cell.value(), 0x12345678)
        self.assertEqual(cell.corrections, 1)
        self.assertEqual(len({bytes(c) for c in cell._copies}), 1)

    def test_two_copies_damaged_in_one_byte_is_unrecoverable(self):
        cell = TMRInt(0x11223344, 4, "t")
        cell._copies[0][0] ^= 0xFF
        cell._copies[1][0] ^= 0x0F
        with self.assertRaises(TMRUnrecoverable):
            cell.get()
        self.assertEqual(cell.value(default=7), 7)

    def test_bool_encoding_resists_a_single_flip(self):
        flag = TMRBool(True, "f")
        for copy in flag._copies:
            copy[0] ^= 0x01
        self.assertTrue(flag.value())

    def test_counter_increments_atomically(self):
        counter = TMRInt(0, 4, "c")
        for _ in range(100):
            counter.add()
        self.assertEqual(counter.value(), 100)

    def test_scrubber_repairs_registered_cells(self):
        cell = TMRInt(5, 4, "c")
        cell._copies[2][3] ^= 0x01
        scrubber = Scrubber()
        scrubber.register(cell)
        self.assertEqual(scrubber.scrub_once(), 1)
        self.assertEqual(len({bytes(c) for c in cell._copies}), 1)


class TestTimebase(unittest.TestCase):
    def test_round_trip(self):
        for unix_ts in (1_000_000_000.0, 1_700_000_000.5, 315_964_800.0 + 20):
            coarse, fine = unix_to_dice(unix_ts)
            self.assertAlmostEqual(dice_to_unix(coarse, fine), unix_ts,
                                   places=4)

    def test_fine_field_spans_about_one_second(self):
        from radcam.stp.timebase import FINE_TICK_S
        self.assertAlmostEqual(65536 * FINE_TICK_S, 1.0, places=1)

    def test_leap_offset_is_a_parameter(self):
        a = dice_to_unix(1_000_000, 0, leap_offset_s=18)
        b = dice_to_unix(1_000_000, 0, leap_offset_s=19)
        self.assertAlmostEqual(a - b, 1.0, places=6)


if __name__ == "__main__":
    unittest.main()
