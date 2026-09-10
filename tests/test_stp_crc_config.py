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


class AlwaysOnCameraOverTheLink(unittest.TestCase):
    """The camera commands, exercised through the real state machine.

    Testing `is_open` directly would not have caught the bug this class exists
    for: the property was self-consistent, and only the dispatcher's use of it
    as a gate made an always-on payload refuse to describe its own camera. So
    these go in through a command packet and read the response window, the way
    the ground does.
    """

    def setUp(self):
        from tests.support import FakeMediaStore, MemoryLink
        from radcam.protocol import Config as ProtoConfig, Dispatcher
        from radcam.stp import packets as P
        from radcam.stp.experiment import Experiment, ExperimentConfig

        self.P = P
        self.wire = P.Wire(target_id=0xC7)
        self.link = MemoryLink()
        self.cameras = CameraSelector(
            [CameraEntry(0, 48, "AR1335", i2c_bus=4, always_on=True)])
        self.cameras.open()
        store = FakeMediaStore()
        self.experiment = Experiment(
            link=self.link, wire=self.wire,
            dispatcher=Dispatcher(config=ProtoConfig(), store=store),
            store=store, cameras=self.cameras,
            config=ExperimentConfig(target_id=0xC7, scrub_interval_s=3600))
        self.experiment.start()

    def tearDown(self):
        self.experiment.stop()

    def response_to(self, opcode, args=b""):
        from radcam.stp import lrt as L
        from radcam.stp.commands import encode_command_payload
        from tests.support import drain_experiment

        payload = encode_command_payload(opcode, 1, args, 0)
        self.link.dice_send(
            self.P.encode_command(payload, 1000, 5, self.wire, 0xC7))
        self.experiment.service()
        drain_experiment(self.experiment, passes=4)
        self.link.dice_read()
        self.link.dice_send(
            self.P.encode_short_request(self.P.PacketType.LRT_REQUEST,
                                        1000, 5, self.wire, 0xC7))
        self.experiment.service()
        raw = self.link.dice_read()
        at = raw.find(self.wire.sync_bytes)
        return L.decode_lrt_payload(raw[at + 6:at + 6 + 1248])

    def test_camera_list_is_answered(self):
        from radcam.stp.experiment import StpOp
        telemetry = self.response_to(StpOp.CAMERA_LIST)
        self.assertEqual(telemetry["last_result"], 0)
        table = telemetry["resp_data"]
        self.assertEqual(table[0], 1)                 # one camera
        self.assertEqual(table[1], 0)                 # camera 0 active
        self.assertTrue(table[4] & 0x04)              # always-on flag
        self.assertIn(b"AR1335", table)

    def test_selecting_the_always_on_camera_succeeds(self):
        from radcam.stp.experiment import StpOp
        telemetry = self.response_to(StpOp.SELECT_CAMERA, b"\x00")
        self.assertEqual(telemetry["last_result"], 0)
        self.assertEqual(telemetry["camera_active"], 0)

    def test_telemetry_reports_the_camera_without_being_asked(self):
        from radcam.protocol import Msg
        telemetry = self.response_to(Msg.PING)
        self.assertEqual(telemetry["camera_count"], 1)
        self.assertEqual(telemetry["camera_active"], 0)
