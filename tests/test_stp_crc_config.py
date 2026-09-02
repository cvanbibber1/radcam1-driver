"""The CRC and camera settings as configuration.

The CRC is the one setting whose being wrong is invisible from this end: we
transmit correctly formed packets and the other end silently discards them. So
the mapping from configuration to parameters is worth testing directly, and in
particular the failure modes - a typo must degrade to a known CRC and a logged
complaint, never to an exception during startup that leaves the payload silent.
"""

import unittest

from radcam.stp import crc as crcmod
from radcam.cameras import CameraSelector, CameraEntry, NO_CAMERA, \
    entries_from_config


class CrcFromConfig(unittest.TestCase):

    def test_default_is_ccitt_false(self):
        params, problems = crcmod.from_config({})
        self.assertEqual(params, crcmod.CCITT_FALSE)
        self.assertEqual(problems, [])

    def test_check_value(self):
        # The Rocksoft check value. If this moves, the engine is wrong and
        # every other test in this file is measuring the wrong thing.
        self.assertEqual(crcmod.crc16(b"123456789"), 0x29B1)

    def test_names_are_tolerant(self):
        for name in ("CRC-16/XMODEM", "xmodem", "XMODEM", "crc-16/xmodem"):
            params, problems = crcmod.from_config({"crc_variant": name})
            self.assertEqual(params.name, "CRC-16/XMODEM", name)
            self.assertEqual(problems, [])

    def test_unknown_name_degrades_and_complains(self):
        params, problems = crcmod.from_config({"crc_variant": "nope"})
        self.assertEqual(params, crcmod.CCITT_FALSE)
        self.assertEqual(len(problems), 1)
        self.assertIn("nope", problems[0])

    def test_custom_parameters(self):
        params, problems = crcmod.from_config({
            "crc_variant": "custom", "crc_poly": "0x8005", "crc_init": 0,
            "crc_reflect_in": True, "crc_reflect_out": True})
        self.assertEqual(problems, [])
        self.assertEqual((params.poly, params.init), (0x8005, 0x0000))
        self.assertTrue(params.reflect_in and params.reflect_out)
        # ARC's check value, reached by parameters rather than by name.
        self.assertEqual(params.compute(b"123456789"), 0xBB3D)

    def test_hex_accepted_in_several_spellings(self):
        for spelling in (0x1021, "0x1021", "1021h", 4129):
            params, problems = crcmod.from_config({"crc_poly": spelling})
            self.assertEqual(problems, [])
            self.assertEqual(params.poly, 0x1021, spelling)

    def test_bad_number_keeps_the_base_value(self):
        params, problems = crcmod.from_config({"crc_poly": "zzz"})
        self.assertEqual(params.poly, 0x1021)
        self.assertEqual(len(problems), 1)

    def test_store_order_is_separate_from_the_algorithm(self):
        params, _ = crcmod.from_config({"crc_store": "little"})
        self.assertFalse(params.big_endian_store)
        self.assertEqual(params.pack(0x1234), b"\x34\x12")

    def test_bad_store_order_complains(self):
        params, problems = crcmod.from_config({"crc_store": "middle"})
        self.assertTrue(params.big_endian_store)
        self.assertEqual(len(problems), 1)

    def test_override_on_a_named_variant_uses_it_as_the_base(self):
        params, _ = crcmod.from_config({"crc_variant": "xmodem",
                                        "crc_init": "0xFFFF"})
        self.assertEqual((params.poly, params.init), (0x1021, 0xFFFF))

    def test_custom_that_matches_a_standard_is_named(self):
        params, _ = crcmod.from_config({"crc_poly": 0x1021, "crc_init": 0})
        self.assertIn("XMODEM", params.name)


class AlwaysOnCamera(unittest.TestCase):
    """A camera whose supply is not ours to switch is still the active one."""

    def selector(self):
        sel = CameraSelector([CameraEntry(index=0, gpio=48, name="AR1335",
                                          i2c_bus=4, always_on=True)])
        # open() needs libgpiod and a real chip; the always-on path claims no
        # lines, so drive the state the same way open() would.
        sel._active = sel._always_on_index()
        return sel

    def test_always_on_is_active_without_claiming_a_line(self):
        sel = self.selector()
        self.assertEqual(sel.active, 0)
        self.assertEqual(sel._requests, {})

    def test_selecting_it_succeeds(self):
        sel = self.selector()
        self.assertTrue(sel.select(0))
        self.assertEqual(sel.active, 0)

    def test_disable_all_cannot_turn_it_off_and_says_so(self):
        sel = self.selector()
        sel.disable_all()
        self.assertEqual(sel.active, 0)

    def test_selecting_another_camera_is_refused(self):
        sel = CameraSelector([
            CameraEntry(index=0, gpio=48, name="AR1335", always_on=True),
            CameraEntry(index=1, gpio=5, name="THERMAL")])
        sel._active = sel._always_on_index()
        # Enabling the thermal camera would put two on the fabric at once,
        # because the first cannot be switched off. Refuse, do not half-do it.
        self.assertFalse(sel.select(1))
        self.assertEqual(sel.active, 0)
        self.assertEqual(sel.failures, 1)

    def test_table_flags_always_on(self):
        sel = self.selector()
        table = sel.pack_table()
        self.assertEqual(table[0], 1)          # one camera
        self.assertEqual(table[1], 0)          # active index 0
        flags = table[4]
        self.assertTrue(flags & 0x01)          # active high
        self.assertTrue(flags & 0x02)          # currently enabled
        self.assertTrue(flags & 0x04)          # always on

    def test_config_round_trip(self):
        entries = entries_from_config([
            {"index": 0, "gpio": 48, "name": "AR1335", "i2c_bus": 4,
             "always_on": True}])
        self.assertEqual(len(entries), 1)
        self.assertTrue(entries[0].always_on)
        self.assertIn("always-on", entries[0].describe())

    def test_no_always_on_camera_reports_none(self):
        sel = CameraSelector([CameraEntry(index=1, gpio=5, name="THERMAL")])
        self.assertEqual(sel._always_on_index(), NO_CAMERA)


if __name__ == "__main__":
    unittest.main()
