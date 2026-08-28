"""The flight computer's own CRC-16, as a golden reference.

The DICE side computes packet checksums with a precomputed CRC-16/CCITT table
and a fixed coverage range. That implementation is the authority: if ours ever
disagrees with it, every packet this payload sends is rejected and every packet
it receives is discarded, and nothing above the physical layer would explain
why.

So the flight table is transcribed here verbatim, together with the loop that
uses it, and the tests below assert that `radcam.stp.crc` produces identical
results over the exact ranges the ICD defines for each packet class. This is
not testing our CRC against itself in a different form - it is testing it
against the other end of the link.

Reference implementation, transcribed from the flight source:

    uint16_t crc16_ccitt(const etl::vector<uint8_t, 1288>& data, size_t length)
    {
        uint16_t crc = 0xFFFF;
        for (size_t i = 4; i < length; i++)
            crc = crc16_lookup_table[((crc >> 8) ^ data[i]) & 0xff] ^ (crc << 8);
        return crc;
    }

Two properties of it matter and are asserted separately:

* the loop starts at index 4, so the four sync bytes are excluded
* it runs to `length`, which the caller sets to the CRC field's offset, so the
  checksum never covers itself
"""

import random
import unittest

from radcam.stp import packets as P
from radcam.stp.crc import CCITT_FALSE

#: Transcribed verbatim from the flight computer source. Do not regenerate:
#: the point is that this is *their* table, not one we derived.
FLIGHT_TABLE = [
0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,0x8108,
0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,0x1231,0x0210,
0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,0x9339,0x8318,0xB37B,
0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,0x2462,0x3443,0x0420,0x1401,
0x64E6,0x74C7,0x44A4,0x5485,0xA56A,0xB54B,0x8528,0x9509,0xE5EE,
0xF5CF,0xC5AC,0xD58D,0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,
0x5695,0x46B4,0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,
0xC7BC,0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,
0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,0x5AF5,
0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,0xDBFD,0xCBDC,
0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,0x6CA6,0x7C87,0x4CE4,
0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,0xEDAE,0xFD8F,0xCDEC,0xDDCD,
0xAD2A,0xBD0B,0x8D68,0x9D49,0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,
0x2E32,0x1E51,0x0E70,0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,
0x9F59,0x8F78,0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,
0xE16F,0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,0x02B1,
0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,0xB5EA,0xA5CB,
0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,0x34E2,0x24C3,0x14A0,
0x0481,0x7466,0x6447,0x5424,0x4405,0xA7DB,0xB7FA,0x8799,0x97B8,
0xE75F,0xF77E,0xC71D,0xD73C,0x26D3,0x36F2,0x0691,0x16B0,0x6657,
0x7676,0x4615,0x5634,0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,
0xB98A,0xA9AB,0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,
0x28A3,0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,0xFD2E,
0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,0x7C26,0x6C07,
0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,0xEF1F,0xFF3E,0xCF5D,
0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,0x6E17,0x7E36,0x4E55,0x5E74,
0x2E93,0x3EB2,0x0ED1,0x1EF0]


def flight_crc16(data: bytes, length: int) -> int:
    """The flight computer's function, transcribed. Covers data[4:length]."""
    crc = 0xFFFF
    for i in range(4, length):
        crc = (FLIGHT_TABLE[((crc >> 8) ^ data[i]) & 0xFF]
               ^ ((crc << 8) & 0xFFFF)) & 0xFFFF
    return crc


#: (packet size, offset of the CRC field, name) for every class on the link.
PACKET_SHAPES = (
    (P.COMMAND_ACK_SIZE, P.COMMAND_ACK_CRC_OFFSET, "Command ACK"),
    (P.SHORT_REQUEST_SIZE, P.SHORT_CRC_OFFSET, "LRT request / HRT control"),
    (P.COMMAND_PACKET_SIZE, P.COMMAND_CRC_OFFSET, "Command"),
    (P.LRT_DATA_PACKET_SIZE, P.LRT_DATA_CRC_OFFSET, "LRT Data"),
    (P.HRT_DATA_PACKET_SIZE, P.HRT_DATA_CRC_OFFSET, "HRT Data"),
)


class TestFlightTable(unittest.TestCase):
    def test_table_is_256_entries(self):
        self.assertEqual(len(FLIGHT_TABLE), 256)

    def test_table_is_the_standard_ccitt_polynomial(self):
        """Generated from 0x1021, MSB-first, which is what CCITT-FALSE uses."""
        generated = []
        for i in range(256):
            value = i << 8
            for _ in range(8):
                value = ((value << 1) ^ 0x1021) & 0xFFFF if value & 0x8000 \
                    else (value << 1) & 0xFFFF
            generated.append(value)
        self.assertEqual(generated, list(FLIGHT_TABLE))

    def test_published_check_value(self):
        """CRC-16/CCITT-FALSE of '123456789' is 0x29B1."""
        crc = 0xFFFF
        for byte in b"123456789":
            crc = (FLIGHT_TABLE[((crc >> 8) ^ byte) & 0xFF]
                   ^ ((crc << 8) & 0xFFFF)) & 0xFFFF
        self.assertEqual(crc, 0x29B1)
        self.assertEqual(CCITT_FALSE.compute(b"123456789"), 0x29B1)


class TestAgreementWithFlight(unittest.TestCase):
    """Ours must equal theirs over the ICD range, for every packet class."""

    def test_agrees_on_random_packets(self):
        random.seed(20260828)
        wire = P.Wire(target_id=0xC7)
        for size, crc_offset, name in PACKET_SHAPES:
            with self.subTest(name):
                for _ in range(150):
                    packet = bytearray(random.randbytes(size))
                    packet[0:4] = wire.sync_bytes
                    self.assertEqual(
                        flight_crc16(bytes(packet), crc_offset),
                        CCITT_FALSE.compute(bytes(packet[4:crc_offset])),
                        f"{name}: disagreement at size {size}")

    def test_agrees_on_degenerate_content(self):
        """All-zero and all-ones payloads are where a bad init value shows."""
        wire = P.Wire(target_id=0xC7)
        for size, crc_offset, name in PACKET_SHAPES:
            for fill in (0x00, 0xFF, 0x55, 0xAA):
                with self.subTest(f"{name} fill {fill:#04x}"):
                    packet = bytearray([fill]) * size
                    packet[0:4] = wire.sync_bytes
                    self.assertEqual(
                        flight_crc16(bytes(packet), crc_offset),
                        CCITT_FALSE.compute(bytes(packet[4:crc_offset])))

    def test_sync_bytes_are_excluded(self):
        """Changing the sync must not change the checksum."""
        packet = bytearray(random.Random(1).randbytes(120))
        a = flight_crc16(bytes(packet), 118)
        packet[0:4] = b"\xde\xad\xbe\xef"
        self.assertEqual(flight_crc16(bytes(packet), 118), a)

    def test_the_crc_field_is_not_covered(self):
        """Coverage stops at the CRC offset, so the field cannot cover itself."""
        packet = bytearray(random.Random(2).randbytes(1288))
        a = flight_crc16(bytes(packet), P.HRT_DATA_CRC_OFFSET)
        packet[P.HRT_DATA_CRC_OFFSET:] = b"\x00\x00"
        self.assertEqual(flight_crc16(bytes(packet), P.HRT_DATA_CRC_OFFSET), a)


class TestPacketsWeEmit(unittest.TestCase):
    """The packets the payload actually builds must satisfy the flight CRC."""

    def setUp(self):
        self.wire = P.Wire(target_id=0xC7)

    def check(self, packet: bytes, crc_offset: int, name: str):
        stored = self.wire.crc.unpack(packet[crc_offset:crc_offset + 2])
        self.assertEqual(flight_crc16(packet, crc_offset), stored,
                         f"{name} would be rejected by the flight computer")

    def test_command_ack(self):
        self.check(P.encode_command_ack(self.wire, 0xC7),
                   P.COMMAND_ACK_CRC_OFFSET, "Command ACK")

    def test_lrt_data(self):
        from radcam.stp import lrt as L
        packet = P.encode_lrt_data(
            L.build_lrt_payload({"target_id": 0xC7, "uptime_s": 4242}),
            self.wire, 0xC7)
        self.check(packet, P.LRT_DATA_CRC_OFFSET, "LRT Data")

    def test_hrt_data(self):
        from radcam.stp import hrt as H
        packet = P.encode_hrt_data(
            H.build_hrt_payload(H.SubType.MEDIA_DATA, 1, 0, 1, b"radcam"),
            self.wire, 0xC7)
        self.check(packet, P.HRT_DATA_CRC_OFFSET, "HRT Data")

    def test_every_catalogue_command(self):
        """All 35 canned command strings must pass the flight check."""
        from radcam.stp.catalogue import CATALOGUE, build_command_hex
        for spec in CATALOGUE:
            with self.subTest(spec.name):
                packet = bytes.fromhex(build_command_hex(spec, wire=self.wire))
                self.check(packet, P.COMMAND_CRC_OFFSET, spec.name)

    def test_every_catalogue_request(self):
        from radcam.stp.catalogue import SHORT_PACKETS, build_short_hex
        for name, ptype, _ in SHORT_PACKETS:
            with self.subTest(name):
                packet = bytes.fromhex(build_short_hex(ptype, self.wire))
                self.check(packet, P.SHORT_CRC_OFFSET, name)


if __name__ == "__main__":
    unittest.main()
