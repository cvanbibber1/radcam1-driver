"""EEPROM record framing, redundancy and merge behaviour.

Calibration is the one thing aboard that cannot be recomputed in flight, so
these tests care much less about the happy path than about what happens when
copies disagree, when a bit flips, and when two calibration tools write to the
same record on different days.
"""

import unittest

from tests.support import FakeEEPROM
from radcam.eeprom import COPY_OFFSETS, EEPROMError, MAX_PAYLOAD


class TestFraming(unittest.TestCase):
    def test_round_trip(self):
        ee = FakeEEPROM()
        rec = {"schema": 1, "camera_id": "x", "ccm": [[1, 0, 0]]}
        ee.store(rec)
        self.assertEqual(ee.load(), rec)

    def test_blank_device_reads_as_no_calibration(self):
        self.assertIsNone(FakeEEPROM().load())

    def test_oversized_record_is_refused_not_truncated(self):
        # Silently truncating would store a record that fails CRC on every
        # later read, which looks like radiation damage rather than a bug.
        ee = FakeEEPROM()
        with self.assertRaises(EEPROMError):
            ee.store({"pad": "x" * (MAX_PAYLOAD + 100)})

    def test_write_is_verified(self):
        ee = FakeEEPROM()

        class Sabotage(FakeEEPROM):
            def write(self, offset, data):
                super().write(offset, bytes(len(data)))

        with self.assertRaises(EEPROMError):
            Sabotage().store({"a": 1})


class TestRedundancy(unittest.TestCase):
    def setUp(self):
        self.ee = FakeEEPROM()
        self.rec = {"schema": 1, "ccm": [[1.5, -0.4, -0.1]]}
        self.ee.store(self.rec)

    def test_survives_two_destroyed_copies(self):
        self.ee.corrupt(COPY_OFFSETS[0], 32)
        self.ee.corrupt(COPY_OFFSETS[1], 32)
        self.assertEqual(self.ee.load(), self.rec)

    def test_all_copies_destroyed_returns_none(self):
        for off in COPY_OFFSETS:
            self.ee.corrupt(off, 64)
        self.assertIsNone(self.ee.load())

    def test_copy_status_reports_the_damaged_one(self):
        self.ee.corrupt(COPY_OFFSETS[1], 16)
        self.assertEqual(self.ee.copy_status(), [True, False, True])

    def test_repair_heals_and_reports(self):
        self.ee.corrupt(COPY_OFFSETS[2], 16)
        self.assertTrue(self.ee.repair())
        self.assertEqual(self.ee.copy_status(), [True, True, True])
        # Nothing left to do the second time; ground needs a truthful answer,
        # not an unconditional "yes".
        self.assertFalse(self.ee.repair())

    def test_repair_with_nothing_intact_does_not_invent_data(self):
        for off in COPY_OFFSETS:
            self.ee.corrupt(off, 64)
        self.assertFalse(self.ee.repair())

    def test_load_repairs_as_a_side_effect(self):
        self.ee.corrupt(COPY_OFFSETS[0], 16)
        self.assertEqual(self.ee.load(), self.rec)
        self.assertEqual(self.ee.copy_status(), [True, True, True])


class TestMerge(unittest.TestCase):
    """Each calibration tool writes its own section and must not clobber others."""

    def test_sections_accumulate(self):
        ee = FakeEEPROM()
        ee.update("awb", {"r_over_g": 0.61}, camera_id="cam1")
        ee.update("shading", {"centre": [0.5, 0.5]})
        ee.update("distortion", {"k": [0.3]})
        ee.update("response", {"saturation_dn": 65535})
        rec = ee.load()
        self.assertEqual(sorted(rec),
                         ["awb", "camera_id", "distortion", "response",
                          "schema", "sensor", "shading"])
        self.assertEqual(rec["awb"]["r_over_g"], 0.61)
        self.assertEqual(rec["camera_id"], "cam1")

    def test_rewriting_one_section_leaves_the_others(self):
        ee = FakeEEPROM()
        ee.update("awb", {"r_over_g": 0.61})
        ee.update("shading", {"centre": [0.5, 0.5]})
        ee.update("awb", {"r_over_g": 0.75})
        rec = ee.load()
        self.assertEqual(rec["awb"]["r_over_g"], 0.75)
        self.assertIn("shading", rec)

    def test_update_on_a_blank_device_creates_a_valid_record(self):
        ee = FakeEEPROM()
        ee.update("response", {"saturation_dn": 1})
        self.assertEqual(ee.load()["schema"], 1)


if __name__ == "__main__":
    unittest.main()
