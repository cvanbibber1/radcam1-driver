"""Packet layer: sizes, offsets, CRC coverage and the resynchronising reader.

Every size assertion here is taken from the ICD tables rather than from the
implementation, so a change to the code that breaks interoperability fails the
test instead of quietly redefining the protocol.
"""

import unittest

from tests.support import ROOT      # noqa: F401  (puts the repo on sys.path)

from radcam.stp import packets as P
from radcam.stp.crc import ARC, CATALOG, CCITT_FALSE, Crc16Params, solve
from radcam.stp.rx import PacketReader


class TestCrc(unittest.TestCase):
    #: Published check values for the string "123456789".
    CHECKS = {"CRC-16/CCITT-FALSE": 0x29B1, "CRC-16/XMODEM": 0x31C3,
              "CRC-16/ARC": 0xBB3D, "CRC-16/MODBUS": 0x4B37,
              "CRC-16/KERMIT": 0x2189, "CRC-16/X-25": 0x906E,
              "CRC-16/GENIBUS": 0xD64E, "CRC-16/MAXIM": 0x44C2,
              "CRC-16/USB": 0xB4C8, "CRC-16/MCRF4XX": 0x6F91,
              "CRC-16/CDMA2000": 0x4C06, "CRC-16/DDS-110": 0x9ECF,
              "CRC-16/EN-13757": 0xC2B7, "CRC-16/T10-DIF": 0xD0DB,
              "CRC-16/DECT-R": 0x007E}

    def test_catalog_matches_published_check_values(self):
        for params in CATALOG:
            with self.subTest(params.name):
                self.assertEqual(params.compute(b"123456789"),
                                 self.CHECKS[params.name])

    def test_store_byte_order_round_trips(self):
        big = Crc16Params(big_endian_store=True)
        little = Crc16Params(big_endian_store=False)
        self.assertEqual(big.pack(0x1234), b"\x12\x34")
        self.assertEqual(little.pack(0x1234), b"\x34\x12")
        self.assertEqual(big.unpack(big.pack(0xBEEF)), 0xBEEF)

    def test_solver_recovers_non_default_parameters(self):
        truth = Crc16Params("CRC-16/ARC", ARC.poly, ARC.init, True, True,
                            0x0000, big_endian_store=False)
        wire = P.Wire(crc=truth, target_id=2)
        packets = [
            P.encode_command(bytes(range(105)), 5, 6, wire, 2),
            P.encode_command(b"\x00" * 105, 7, 8, wire, 2),
            P.encode_short_request(P.PacketType.HRT_GO, 9, 10, wire, 2),
        ]
        found = solve(packets)
        self.assertTrue(found)
        self.assertTrue(any(c.params.poly == ARC.poly
                            and not c.params.big_endian_store
                            and c.start == 4 for c in found))


class TestPacketSizes(unittest.TestCase):
    """Sizes stated by the ICD, asserted independently of the code."""

    def setUp(self):
        self.wire = P.Wire(target_id=1)

    def test_icd_stated_sizes(self):
        self.assertEqual(P.COMMAND_PACKET_SIZE, 120)
        self.assertEqual(P.COMMAND_ACK_SIZE, 8)
        self.assertEqual(P.LRT_REQUEST_SIZE, 14)
        self.assertEqual(P.HRT_CONTROL_SIZE, 14)
        self.assertEqual(P.LRT_DATA_PACKET_SIZE, 1256)
        self.assertEqual(P.HRT_DATA_PACKET_SIZE, 1288)
        self.assertEqual(P.COMMAND_PAYLOAD_LENGTH, 105)
        self.assertEqual(P.LRT_DATA_LENGTH, 1248)      # 624 words
        self.assertEqual(P.HRT_DATA_LENGTH, 1280)      # 640 words

    def test_encoders_emit_stated_sizes(self):
        self.assertEqual(len(P.encode_command_ack(self.wire)), 8)
        self.assertEqual(len(P.encode_lrt_data(b"", self.wire)), 1256)
        self.assertEqual(len(P.encode_hrt_data(b"", self.wire)), 1288)
        self.assertEqual(len(P.encode_command(b"", 0, 0, self.wire)), 120)
        self.assertEqual(
            len(P.encode_short_request(P.PacketType.LRT_REQUEST, 0, 0,
                                       self.wire)), 14)

    def test_sync_pattern_is_derived_from_the_word_constants(self):
        self.assertEqual(P.Wire(big_endian=True).sync_bytes,
                         bytes.fromhex("1acffc1d"))
        # Little-endian sends each 16-bit word low byte first.
        self.assertEqual(P.Wire(big_endian=False).sync_bytes,
                         bytes.fromhex("cf1a1dfc"))

    def test_field_offsets_match_the_icd(self):
        packet = P.encode_command(b"Z" * 105, 0x11223344, 0x5566,
                                  self.wire, 0x42, spare=0x00)
        self.assertEqual(packet[0:4], self.wire.sync_bytes)
        self.assertEqual(packet[4:8], b"\x11\x22\x33\x44")
        self.assertEqual(packet[8:10], b"\x55\x66")
        self.assertEqual(packet[10], 0x10)
        self.assertEqual(packet[11], 0x42)
        self.assertEqual(packet[12:117], b"Z" * 105)
        self.assertEqual(packet[117], 0x00)

    def test_hrt_crc_covers_exactly_what_the_icd_states(self):
        """The ICD is explicit: CRC input is packet[4:1286]."""
        packet = P.encode_hrt_data(b"\x5a" * 1280, self.wire, 1)
        stored = self.wire.crc.unpack(packet[1286:1288])
        self.assertEqual(self.wire.crc.compute(packet[4:1286]), stored)

    def test_hrt_control_crc_covers_packet_4_to_12(self):
        packet = P.encode_short_request(P.PacketType.HRT_STOP, 1, 2,
                                        self.wire, 1)
        stored = self.wire.crc.unpack(packet[12:14])
        self.assertEqual(self.wire.crc.compute(packet[4:12]), stored)

    def test_crc_is_always_the_final_two_bytes(self):
        """Mission-confirmed rule: the last two bytes of every message are CRC.

        This was an inference for LRT Data, whose supplied ICD rows accounted
        for only 1254 of its 1256 bytes. It is now a stated rule, so it is
        asserted here for every message this implementation emits rather than
        left implicit in six separate offset constants.
        """
        messages = [
            ("Command", P.encode_command(b"x" * 105, 1, 2, self.wire, 0xC7)),
            ("Command ACK", P.encode_command_ack(self.wire, 0xC7)),
            ("LRT Request", P.encode_short_request(
                P.PacketType.LRT_REQUEST, 1, 2, self.wire, 0xC7)),
            ("HRT Go", P.encode_short_request(
                P.PacketType.HRT_GO, 1, 2, self.wire, 0xC7)),
            ("LRT Data", P.encode_lrt_data(b"y" * 1248, self.wire, 0xC7)),
            ("HRT Data", P.encode_hrt_data(b"z" * 1280, self.wire, 0xC7)),
        ]
        for name, packet in messages:
            with self.subTest(name):
                stored = self.wire.crc.unpack(packet[-2:])
                computed = self.wire.crc.compute(packet[4:-2])
                self.assertEqual(computed, stored,
                                 f"{name}: CRC is not the final two bytes")

    def test_assigned_target_id_is_the_default(self):
        """0xC7 is this experiment's mission-assigned address, not a placeholder."""
        self.assertEqual(P.Wire().target_id, 0xC7)

    def test_lrt_trailer_can_be_zeros_instead_of_crc(self):
        zeroed = P.Wire(target_id=1, lrt_trailer="zero")
        packet = P.encode_lrt_data(b"x" * 1248, zeroed, 1)
        self.assertEqual(len(packet), 1256)
        self.assertEqual(packet[1254:1256], b"\x00\x00")

    def test_oversized_payloads_are_refused(self):
        with self.assertRaises(P.PacketError):
            P.encode_hrt_data(b"x" * 1281, self.wire)
        with self.assertRaises(P.PacketError):
            P.encode_lrt_data(b"x" * 1249, self.wire)
        with self.assertRaises(P.PacketError):
            P.encode_command(b"x" * 106, 0, 0, self.wire)


class TestDecoders(unittest.TestCase):
    def setUp(self):
        self.wire = P.Wire(target_id=7)

    def test_command_round_trip(self):
        packet = P.encode_command(b"payload", 1234, 56, self.wire, 7)
        decoded = P.decode_command(packet, self.wire)
        self.assertEqual(decoded.coarse_time, 1234)
        self.assertEqual(decoded.fine_time, 56)
        self.assertEqual(decoded.target_id, 7)
        self.assertEqual(decoded.payload[:7], b"payload")
        self.assertEqual(len(decoded.payload), 105)

    def test_timestamp_combines_coarse_and_fine(self):
        packet = P.encode_command(b"", 100, 65535, self.wire, 7)
        decoded = P.decode_command(packet, self.wire)
        self.assertAlmostEqual(decoded.timestamp_s, 100 + 65535 * 15.3e-6,
                               places=6)

    def test_every_short_request_type_decodes(self):
        for ptype in (P.PacketType.LRT_REQUEST, P.PacketType.HRT_STOP,
                      P.PacketType.HRT_STOP_WITH_LOSS, P.PacketType.HRT_GO):
            with self.subTest(hex(ptype)):
                packet = P.encode_short_request(ptype, 1, 2, self.wire, 7)
                decoded = P.decode_short_request(packet, self.wire)
                self.assertEqual(decoded.packet_type, ptype)
                self.assertEqual(decoded.target_id, 7)

    def test_corruption_anywhere_in_the_covered_range_is_caught(self):
        packet = bytearray(P.encode_command(b"abc", 1, 2, self.wire, 7))
        for offset in (4, 10, 11, 12, 116, 117):
            with self.subTest(offset=offset):
                broken = bytearray(packet)
                broken[offset] ^= 0xFF
                with self.assertRaises(P.PacketError):
                    P.decode_command(bytes(broken), self.wire)

    def test_wrong_length_is_refused(self):
        packet = P.encode_command(b"", 0, 0, self.wire, 7)
        with self.assertRaises(P.PacketError):
            P.decode_command(packet[:119], self.wire)

    def test_non_zero_spare_is_tolerated(self):
        """The ICD says a later revision may define the spare byte."""
        packet = P.encode_command(b"", 0, 0, self.wire, 7, spare=0x99)
        self.assertEqual(P.decode_command(packet, self.wire).spare, 0x99)


class TestPacketReader(unittest.TestCase):
    def setUp(self):
        self.wire = P.Wire(target_id=3)
        self.reader = PacketReader(self.wire)

    def cmd(self, target=3):
        return P.encode_command(b"c", 1, 1, self.wire, target)

    def req(self, ptype=P.PacketType.LRT_REQUEST, target=3):
        return P.encode_short_request(ptype, 1, 1, self.wire, target)

    def test_leading_garbage_is_discarded(self):
        got = self.reader.feed(b"\xff\xee\xdd" + self.cmd())
        self.assertEqual(len(got), 1)
        self.assertEqual(self.reader.stats.dropped_bytes, 3)

    def test_packets_split_across_reads_reassemble(self):
        stream = self.cmd() + self.req()
        got = []
        for i in range(0, len(stream), 5):
            got += self.reader.feed(stream[i:i + 5])
        self.assertEqual(len(got), 2)

    def test_other_targets_are_counted_and_dropped(self):
        got = self.reader.feed(self.cmd(target=9) + self.cmd(target=3))
        self.assertEqual(len(got), 1)
        self.assertEqual(self.reader.stats.not_for_us, 1)

    def test_bad_crc_does_not_consume_the_following_packet(self):
        broken = bytearray(self.cmd())
        broken[60] ^= 0xFF
        got = self.reader.feed(bytes(broken) + self.req())
        self.assertEqual(len(got), 1, "the good packet after a bad one is lost")
        self.assertEqual(self.reader.stats.bad_crc, 1)

    def test_unknown_packet_type_is_skipped_not_consumed(self):
        """H&S and File Transfer share the sync but have unknown lengths."""
        foreign = bytearray(self.req())
        foreign[10] = 0x40                      # a class we do not handle
        got = self.reader.feed(bytes(foreign) + self.cmd())
        self.assertEqual(len(got), 1)
        self.assertGreaterEqual(self.reader.stats.unknown_type, 1)

    def test_buffer_is_bounded(self):
        reader = PacketReader(self.wire, max_buffer=512)
        reader.feed(b"\x01" * 4096)
        self.assertLessEqual(reader.pending_bytes, 512)
        self.assertGreater(reader.stats.dropped_bytes, 0)

    def test_sync_straddling_two_reads_is_found(self):
        packet = self.cmd()
        got = self.reader.feed(packet[:2])
        got += self.reader.feed(packet[2:])
        self.assertEqual(len(got), 1)


if __name__ == "__main__":
    unittest.main()
