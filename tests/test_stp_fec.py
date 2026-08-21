"""Forward error correction on the HRT transfer path.

The theme is what happens when chunks do not arrive. Detection was already
covered by the CRC tests; these are about *correction* - reconstructing a lost
chunk from parity without having to ask for it again.
"""

import os
import unittest

from radcam.stp import fec
from radcam.stp import hrt as H


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


if __name__ == "__main__":
    unittest.main()
