"""Forward error correction, and file transfer over the LRT path.

The theme is what happens when chunks do not arrive. Detection was already
covered by the CRC tests; these are about *correction* - reconstructing a lost
chunk from parity without asking for it again - and about the LRT downlink,
which exists precisely for the case where HRT is never opened.
"""

import os
import struct
import unittest
import zlib

from tests.support import FakeMediaStore, MemoryLink, drain_experiment

from radcam.protocol import Config as ProtoConfig, Dispatcher, Err
from radcam.stp import fec
from radcam.stp import hrt as H
from radcam.stp import lrt as L
from radcam.stp import packets as P
from radcam.stp.commands import encode_command_payload
from radcam.stp.experiment import Experiment, ExperimentConfig, StpOp
from radcam.stp.lrtfile import LrtFileManager, PARITY_REQUEST_BIT


class TestParityMath(unittest.TestCase):
    SIZE = 64

    def chunks(self, count, tail=None):
        data = {i: bytes([i]) * self.SIZE for i in range(count)}
        if tail is not None:
            data[count - 1] = bytes([count - 1]) * tail
        return data

    def test_parity_of_nothing_is_zeros(self):
        self.assertEqual(fec.parity_of([], self.SIZE), bytes(self.SIZE))

    def test_parity_is_its_own_inverse(self):
        chunks = list(self.chunks(4).values())
        parity = fec.parity_of(chunks, self.SIZE)
        self.assertEqual(fec.parity_of(chunks + [parity], self.SIZE),
                         bytes(self.SIZE))

    def test_recovers_any_single_chunk(self):
        chunks = self.chunks(4)
        parity = fec.parity_of(list(chunks.values()), self.SIZE)
        for lost in range(4):
            with self.subTest(lost=lost):
                present = {k: v for k, v in chunks.items() if k != lost}
                found = fec.recover_missing(present, parity, [0, 1, 2, 3],
                                            self.SIZE)
                self.assertIsNotNone(found)
                index, data = found
                self.assertEqual(index, lost)
                self.assertEqual(data, chunks[lost])

    def test_short_final_chunk_reconstructs_exactly(self):
        """Zero padding is what makes a ragged last chunk safe."""
        chunks = self.chunks(4, tail=17)
        parity = fec.parity_of(list(chunks.values()), self.SIZE)
        present = {k: v for k, v in chunks.items() if k != 3}
        index, data = fec.recover_missing(present, parity, [0, 1, 2, 3], self.SIZE)
        self.assertEqual(index, 3)
        self.assertEqual(data[:17], chunks[3])
        self.assertEqual(data[17:], bytes(self.SIZE - 17))

    def test_two_losses_in_a_group_are_not_recoverable(self):
        chunks = self.chunks(4)
        parity = fec.parity_of(list(chunks.values()), self.SIZE)
        present = {2: chunks[2], 3: chunks[3]}
        self.assertIsNone(
            fec.recover_missing(present, parity, [0, 1, 2, 3], self.SIZE))

    def test_no_loss_returns_nothing_to_do(self):
        chunks = self.chunks(4)
        parity = fec.parity_of(list(chunks.values()), self.SIZE)
        self.assertIsNone(
            fec.recover_missing(chunks, parity, [0, 1, 2, 3], self.SIZE))

    def test_group_arithmetic(self):
        self.assertEqual(fec.group_of(0, 4), 0)
        self.assertEqual(fec.group_of(7, 4), 1)
        self.assertEqual(fec.indices_in_group(0, 4, 10), [0, 1, 2, 3])
        self.assertEqual(fec.indices_in_group(2, 4, 10), [8, 9])
        self.assertEqual(fec.group_count(10, 4), 3)
        self.assertEqual(fec.group_count(0, 4), 0)

    def test_group_size_zero_disables_parity(self):
        self.assertEqual(fec.group_count(100, 0), 0)
        self.assertEqual(fec.indices_in_group(0, 0, 100), [])

    def test_repair_reports_what_it_could_not_fix(self):
        chunks = self.chunks(10)
        parities = {g: fec.parity_of(
            [chunks[i] for i in fec.indices_in_group(g, 4, 10)], self.SIZE)
            for g in range(fec.group_count(10, 4))}
        damaged = {k: v for k, v in chunks.items() if k not in (0, 1, 5)}
        stats = fec.FecStats()
        missing = fec.verify_and_repair(damaged, parities, 10, 4, self.SIZE,
                                        stats)
        self.assertEqual(sorted(missing), [0, 1])   # two lost in group 0
        self.assertEqual(stats.recovered, 1)        # group 1 repaired
        self.assertEqual(stats.unrecoverable, 2)


class TestHrtParity(unittest.TestCase):
    def test_parity_follows_each_completed_group(self):
        manager = H.TransferManager(group_size=4)
        manager.enqueue(1, os.urandom(H.HRT_CHUNK_DATA * 9 + 10))
        order = []
        while True:
            payload = manager.next_payload()
            if payload is None:
                break
            decoded = H.decode_hrt_payload(payload)
            if decoded["sub_type"] == H.SubType.MEDIA_PARITY:
                order.append(f"P{decoded['chunk_index']}")
            elif decoded["sub_type"] == H.SubType.MEDIA_DATA:
                order.append(str(decoded["chunk_index"]))
        self.assertEqual(" ".join(order), "0 1 2 3 P0 4 5 6 7 P1 8 9 P2")

    def test_parity_payloads_are_flagged_both_ways(self):
        manager = H.TransferManager(group_size=2)
        manager.enqueue(1, os.urandom(H.HRT_CHUNK_DATA * 2))
        parity = None
        while parity is None:
            decoded = H.decode_hrt_payload(manager.next_payload())
            if decoded["sub_type"] == H.SubType.MEDIA_PARITY:
                parity = decoded
        self.assertTrue(parity["parity"])
        self.assertTrue(parity["flags"] & H.FLAG_PARITY)

    def test_a_lost_chunk_is_rebuilt_from_parity(self):
        blob = os.urandom(H.HRT_CHUNK_DATA * 7 + 55)
        manager = H.TransferManager(group_size=4)
        manager.enqueue(1, blob)
        data, parity, info = {}, {}, None
        while True:
            payload = manager.next_payload()
            if payload is None:
                break
            decoded = H.decode_hrt_payload(payload)
            if decoded["sub_type"] == H.SubType.MEDIA_INFO:
                info = H.decode_media_info(decoded["data"])
            elif decoded["sub_type"] == H.SubType.MEDIA_PARITY:
                parity[decoded["chunk_index"]] = decoded["data"]
            elif decoded["sub_type"] == H.SubType.MEDIA_DATA:
                data[decoded["chunk_index"]] = decoded["data"]

        lengths = {i: len(v) for i, v in data.items()}
        del data[5]
        missing = fec.verify_and_repair(data, parity, info["chunk_total"], 4,
                                        H.HRT_CHUNK_DATA)
        self.assertEqual(missing, [])
        rebuilt = b"".join(data[i][:lengths[i]]
                           for i in range(info["chunk_total"]))
        self.assertEqual(rebuilt, blob)

    def test_group_size_zero_emits_no_parity(self):
        manager = H.TransferManager(group_size=0)
        manager.enqueue(1, os.urandom(H.HRT_CHUNK_DATA * 5))
        kinds = []
        while True:
            payload = manager.next_payload()
            if payload is None:
                break
            kinds.append(H.decode_hrt_payload(payload)["sub_type"])
        self.assertNotIn(H.SubType.MEDIA_PARITY, kinds)


class TestLrtFileBlock(unittest.TestCase):
    def test_layout_fits_the_payload(self):
        self.assertEqual(L.OFF_FILE_DATA + L.FILE_DATA_MAX, L.OFF_EVENT_COUNT)
        self.assertLessEqual(L.OFF_EVENTS + L.MAX_EVENTS * L.EVENT_SIZE,
                             L.OFF_PAYLOAD_CRC32)
        self.assertEqual(L.FILE_HEADER_LEN, 32)

    def test_round_trip(self):
        chunk = bytes(range(256)) * 2
        decoded = L.decode_lrt_payload(L.build_lrt_payload({
            "file_state": L.FILE_ACTIVE, "file_media_id": 9,
            "file_chunk_index": 4, "file_chunk_total": 77,
            "file_size": 12345, "file_crc32": 0xAABBCCDD,
            "file_data": chunk, "file_fec_group": 16,
            "file_flags": L.FILE_FLAG_LAST_DATA}))
        self.assertEqual(decoded["file_state"], L.FILE_ACTIVE)
        self.assertEqual(decoded["file_media_id"], 9)
        self.assertEqual(decoded["file_chunk_index"], 4)
        self.assertEqual(decoded["file_size"], 12345)
        self.assertEqual(decoded["file_crc32"], 0xAABBCCDD)
        self.assertEqual(decoded["file_data"], chunk)
        self.assertTrue(decoded["file_data_crc_ok"])
        self.assertTrue(decoded["file_last_data"])
        self.assertFalse(decoded["file_is_parity"])

    def test_chunk_corruption_is_detected_not_enforced(self):
        payload = bytearray(L.build_lrt_payload(
            {"file_state": L.FILE_ACTIVE, "file_data": b"abcdef"}))
        payload[L.OFF_FILE_DATA] ^= 0xFF
        decoded = L.decode_lrt_payload(bytes(payload))
        self.assertFalse(decoded["file_data_crc_ok"])
        self.assertEqual(len(decoded["file_data"]), 6)

    def test_file_block_and_command_response_coexist(self):
        """A command mid-transfer must not stall it, nor be stalled by it."""
        decoded = L.decode_lrt_payload(L.build_lrt_payload({
            "file_state": L.FILE_ACTIVE, "file_data": b"F" * 512,
            "resp_opcode": 0x91, "resp_data": b"R" * 100}))
        self.assertEqual(len(decoded["file_data"]), 512)
        self.assertEqual(decoded["resp_data"], b"R" * 100)


class TestLrtFileManager(unittest.TestCase):
    def setUp(self):
        self.blob = os.urandom(L.FILE_DATA_MAX * 9 + 200)
        self.manager = LrtFileManager(resolve=lambda mid: self.blob
                                      if mid == 5 else None,
                                      default_group_size=4)

    def drain(self, limit=80):
        data, parity = {}, {}
        for _ in range(limit):
            block = self.manager.next_block()
            if block["file_state"] != L.FILE_ACTIVE:
                if block["file_state"] == L.FILE_COMPLETE:
                    return data, parity, block
                continue
            if block["file_flags"] & L.FILE_FLAG_PARITY:
                parity[block["file_chunk_index"]] = block["file_data"]
            else:
                data[block["file_chunk_index"]] = block["file_data"]
        return data, parity, None

    def test_unknown_media_will_not_start(self):
        self.assertFalse(self.manager.start(999))

    def test_full_transfer_reassembles(self):
        self.assertTrue(self.manager.start(5))
        data, parity, final = self.drain()
        self.assertIsNotNone(final)
        rebuilt = b"".join(data[i] for i in range(len(data)))
        self.assertEqual(rebuilt, self.blob)
        self.assertEqual(final["file_crc32"],
                         zlib.crc32(self.blob) & 0xFFFFFFFF)
        self.assertEqual(len(parity), fec.group_count(len(data), 4))

    def test_idle_when_nothing_started(self):
        self.assertEqual(self.manager.next_block()["file_state"], L.FILE_IDLE)

    def test_completion_state_persists_for_the_ground_to_see(self):
        self.manager.start(5)
        self.drain()
        for _ in range(3):
            self.assertEqual(self.manager.next_block()["file_state"],
                             L.FILE_COMPLETE)

    def test_resend_reopens_a_finished_transfer(self):
        self.manager.start(5)
        data, _parity, _final = self.drain()
        self.assertEqual(self.manager.request_resend(5, [2, 3]), 2)
        block = self.manager.next_block()
        self.assertEqual(block["file_state"], L.FILE_ACTIVE)
        self.assertEqual(block["file_chunk_index"], 2)
        self.assertTrue(block["file_flags"] & L.FILE_FLAG_RETRANSMIT)
        self.assertEqual(block["file_data"], data[2])

    def test_parity_chunks_can_be_re_requested(self):
        self.manager.start(5)
        self.drain()
        self.assertEqual(
            self.manager.request_resend(5, [PARITY_REQUEST_BIT | 1]), 1)
        block = self.manager.next_block()
        self.assertTrue(block["file_flags"] & L.FILE_FLAG_PARITY)
        self.assertEqual(block["file_chunk_index"], 1)

    def test_out_of_range_resend_is_refused(self):
        self.manager.start(5)
        self.assertEqual(self.manager.request_resend(5, [9999]), 0)
        self.assertEqual(
            self.manager.request_resend(5, [PARITY_REQUEST_BIT | 9999]), 0)

    def test_resend_for_the_wrong_media_is_refused(self):
        self.manager.start(5)
        self.assertEqual(self.manager.request_resend(6, [0]), 0)

    def test_interrupted_transfer_still_leaves_repairable_groups(self):
        """Parity is emitted per group, not saved until the end."""
        self.manager.start(5)
        data, parity = {}, {}
        for _ in range(7):          # stop part-way through
            block = self.manager.next_block()
            if block["file_flags"] & L.FILE_FLAG_PARITY:
                parity[block["file_chunk_index"]] = block["file_data"]
            else:
                data[block["file_chunk_index"]] = block["file_data"]
        self.assertTrue(parity, "no parity emitted before the interruption")
        lost = dict(data)
        del lost[1]
        found = fec.recover_missing(lost, parity[0], [0, 1, 2, 3],
                                    L.FILE_DATA_MAX)
        self.assertIsNotNone(found)
        self.assertEqual(found[1], data[1])


class TestLrtTransferOverTheLink(unittest.TestCase):
    """The whole path: command in, chunks out through LRT replies."""

    def setUp(self):
        self.wire = P.Wire(target_id=1)
        self.link = MemoryLink()
        self.store = FakeMediaStore()
        self.experiment = Experiment(
            link=self.link, wire=self.wire,
            dispatcher=Dispatcher(config=ProtoConfig(), store=self.store),
            store=self.store,
            config=ExperimentConfig(target_id=1, scrub_interval_s=3600,
                                    fec_group_size=8))
        self.experiment.start()
        self.seq = 0

    def tearDown(self):
        self.experiment.stop()

    def command(self, opcode, args=b""):
        self.seq += 1
        self.link.dice_send(P.encode_command(
            encode_command_payload(opcode, self.seq, args), 0, 0, self.wire, 1))
        drain_experiment(self.experiment, passes=3)
        self.link.dice_read()

    def poll(self):
        self.link.dice_read()
        self.link.dice_send(P.encode_short_request(
            P.PacketType.LRT_REQUEST, 0, 0, self.wire, 1))
        self.experiment.service()
        raw = self.link.dice_read()
        at = raw.find(self.wire.sync_bytes)
        return L.decode_lrt_payload(raw[at + 6:at + 6 + 1248]) if at >= 0 else None

    def test_transfer_needs_no_hrt_permission(self):
        self.command(StpOp.LRT_FILE_START, struct.pack("<I", 1))
        self.assertFalse(self.experiment._hrt_enabled.value())
        report = self.poll()
        self.assertEqual(report["file_state"], L.FILE_ACTIVE)
        self.assertEqual(report["file_media_id"], 1)

    def test_full_file_arrives_through_lrt_polls(self):
        self.command(StpOp.LRT_FILE_START, struct.pack("<I", 1))
        data, parity, final = {}, {}, None
        for _ in range(120):
            report = self.poll()
            if report is None:
                continue
            if report["file_state"] == L.FILE_COMPLETE:
                final = report
                break
            if report["file_state"] != L.FILE_ACTIVE:
                continue
            self.assertTrue(report["file_data_crc_ok"])
            if report["file_is_parity"]:
                parity[report["file_chunk_index"]] = report["file_data"]
            else:
                data[report["file_chunk_index"]] = report["file_data"]

        self.assertIsNotNone(final)
        rebuilt = b"".join(data[i] for i in range(final["file_chunk_total"]))
        rebuilt = rebuilt[:final["file_size"]]
        self.assertEqual(rebuilt, self.store.blobs[1])
        self.assertEqual(zlib.crc32(rebuilt) & 0xFFFFFFFF, final["file_crc32"])

    def test_lost_chunks_are_repaired_by_parity(self):
        # Media 3 is 120 chunks. A 10-chunk file is too small for a periodic
        # drop to reliably land on data rather than parity.
        self.command(StpOp.LRT_FILE_START, struct.pack("<I", 3))
        data, parity, final, poll_index = {}, {}, None, 0
        for _ in range(300):
            report = self.poll()
            if report is None:
                continue
            if report["file_state"] == L.FILE_COMPLETE:
                final = report
                break
            if report["file_state"] != L.FILE_ACTIVE:
                continue
            poll_index += 1
            # The emission cycle is 9 blocks (8 data + 1 parity), so a drop
            # interval of 9 would hit only parity chunks and never test the
            # repair path at all. 11 is longer than the group, which also
            # guarantees at most one loss per group - exactly what a single
            # parity chunk can fix.
            if poll_index % 11 == 0:
                continue
            if report["file_is_parity"]:
                parity[report["file_chunk_index"]] = report["file_data"]
            else:
                data[report["file_chunk_index"]] = report["file_data"]

        total = final["file_chunk_total"]
        self.assertLess(len(data), total, "no chunks were actually dropped")
        stats = fec.FecStats()
        missing = fec.verify_and_repair(data, parity, total, 8,
                                        L.FILE_DATA_MAX, stats)
        self.assertGreater(stats.recovered, 0)
        self.assertEqual(missing, [], "parity should have covered 1-in-9 loss")
        rebuilt = b"".join(data[i] for i in range(total))[:final["file_size"]]
        self.assertEqual(rebuilt, self.store.blobs[3])

    def test_stop_ends_the_transfer(self):
        self.command(StpOp.LRT_FILE_START, struct.pack("<I", 1))
        self.poll()
        self.command(StpOp.LRT_FILE_STOP)
        self.assertEqual(self.poll()["file_state"], L.FILE_IDLE)

    def test_start_for_missing_media_reports_an_error(self):
        self.command(StpOp.LRT_FILE_START, struct.pack("<I", 999))
        self.assertEqual(self.experiment.last_result, int(Err.NO_MEDIA))

    def test_fec_group_size_is_settable_from_the_ground(self):
        self.command(StpOp.SET_FEC_GROUP, bytes([0]))
        self.assertEqual(self.experiment.cfg.fec_group_size, 0)
        self.assertEqual(self.experiment.transfers.group_size, 0)
        self.command(StpOp.LRT_FILE_START, struct.pack("<I", 2))
        seen_parity = False
        for _ in range(20):
            report = self.poll()
            if report and report["file_state"] == L.FILE_ACTIVE \
                    and report["file_is_parity"]:
                seen_parity = True
        self.assertFalse(seen_parity, "parity emitted with FEC disabled")

    def test_oversized_command_response_is_pullable_over_lrt(self):
        """A reply too big for the LRT window must not need HRT to arrive."""
        self.command(0x72)                      # GET_LINK_STATS, small
        big = b"Z" * (L.RESP_DATA_MAX + 500)
        blob_id = 0xFF000000 | 0x1234
        self.experiment._response_blobs[blob_id] = big
        self.command(StpOp.LRT_FILE_START, struct.pack("<I", blob_id))
        report = self.poll()
        self.assertEqual(report["file_state"], L.FILE_ACTIVE)
        self.assertEqual(report["file_size"], len(big))


if __name__ == "__main__":
    unittest.main()
