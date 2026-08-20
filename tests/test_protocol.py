"""Command protocol: framing, region capture, EEPROM access and refusals.

The refusals matter as much as the successes. A command that silently clamps a
bad parameter returns a picture of the wrong thing, or writes to the wrong
address, and nothing downstream can tell it happened.
"""

import struct
import tempfile
import unittest

from tests.support import FakeEEPROM
from radcam.camera import Camera, MediaStore, SyntheticSource
from radcam.protocol import (Config, Dispatcher, Err, Msg, RESOLUTIONS,
                             apply_config_value, Cfg, FLASH_PERCENT_MAX)
from radcam.framing import Frame


def make(eeprom=None, writable=False):
    cam = Camera(SyntheticSource(), MediaStore(tempfile.mkdtemp()), led=None)
    d = Dispatcher(Config(), camera=cam, store=cam.store, eeprom=eeprom,
                   eeprom_writable=writable)
    return d, cam


def one(d, msg, payload=b"", seq=1):
    replies = d.handle(Frame(msg, seq, payload))
    return replies[0]


class TestRegionCapture(unittest.TestCase):
    def setUp(self):
        self.d, self.cam = make()

    def _region(self, x, y, w, h, ow, oh):
        return one(self.d, Msg.CAPTURE_REGION,
                   struct.pack("<6H", x, y, w, h, ow, oh))

    def test_stores_at_the_requested_output_size(self):
        r = self._region(1000, 800, 1920, 1080, 1920, 1080)
        self.assertEqual(r.type, Msg.CAPTURE_ACK)
        media_id, size = struct.unpack("<IQ", r.payload)
        self.assertGreater(size, 0)
        rec = [m for m in self.cam.store.list() if m.media_id == media_id][0]
        self.assertEqual((rec.width, rec.height), (1920, 1080))

    def test_downscales_a_large_window(self):
        r = self._region(0, 0, 4096, 3072, 640, 480)
        media_id, _ = struct.unpack("<IQ", r.payload)
        rec = [m for m in self.cam.store.list() if m.media_id == media_id][0]
        self.assertEqual((rec.width, rec.height), (640, 480))

    def test_window_past_the_sensor_is_refused_not_clamped(self):
        full_w, full_h = RESOLUTIONS[0]
        for args in ((full_w - 100, 0, 1920, 1080, 1920, 1080),
                     (0, full_h - 100, 1920, 1080, 1920, 1080),
                     (0, 0, 0, 1080, 1920, 1080),
                     (0, 0, 1920, 0, 1920, 1080),
                     (0, 0, 1920, 1080, 0, 1080),
                     (0, 0, 1920, 1080, 9000, 1080)):
            r = self._region(*args)
            self.assertEqual(r.type, Msg.NACK, args)
            self.assertEqual(r.payload[2], Err.REGION_INVALID, args)

    def test_smaller_window_produces_a_smaller_file(self):
        """The whole point is bandwidth, so this is the property to protect."""
        big = struct.unpack("<IQ", self._region(0, 0, 3840, 2160,
                                                1920, 1080).payload)[1]
        small = struct.unpack("<IQ", self._region(1800, 1300, 640, 480,
                                                  640, 480).payload)[1]
        self.assertLess(small, big)

    def test_different_windows_return_different_pictures(self):
        """Guards against the crop being accepted and then ignored.

        The synthetic source used to take a crop argument and drop it, so every
        window produced an identical frame and these tests passed while
        proving nothing about the region path.
        """
        a = struct.unpack("<IQ", self._region(0, 0, 1024, 768,
                                              256, 192).payload)[0]
        b = struct.unpack("<IQ", self._region(3000, 2200, 1024, 768,
                                              256, 192).payload)[0]
        self.assertNotEqual(self.cam.store.read(a), self.cam.store.read(b))

    def test_short_payload_is_a_bad_param(self):
        r = one(self.d, Msg.CAPTURE_REGION, struct.pack("<3H", 1, 2, 3))
        self.assertEqual(r.type, Msg.NACK)
        self.assertEqual(r.payload[2], Err.BAD_PARAM)


class TestEepromCommands(unittest.TestCase):
    def setUp(self):
        self.ee = FakeEEPROM()
        self.ee.store({"schema": 1, "camera_id": "cam1"})
        self.d, _ = make(eeprom=self.ee)

    def test_status_counts_good_copies(self):
        r = one(self.d, Msg.EEPROM_STATUS)
        self.assertEqual(r.type, Msg.EEPROM_STATUS_REPORT)
        total, good = r.payload[0], r.payload[1]
        self.assertEqual((total, good), (3, 3))
        self.assertEqual(list(r.payload[2:]), [1, 1, 1])

    def test_status_after_damage(self):
        from radcam.eeprom import COPY_OFFSETS
        self.ee.corrupt(COPY_OFFSETS[1], 16)
        r = one(self.d, Msg.EEPROM_STATUS)
        self.assertEqual(r.payload[1], 2)
        self.assertEqual(list(r.payload[2:]), [1, 0, 1])

    def test_repair_then_status_is_clean(self):
        from radcam.eeprom import COPY_OFFSETS
        self.ee.corrupt(COPY_OFFSETS[0], 16)
        r = one(self.d, Msg.EEPROM_REPAIR)
        self.assertEqual(r.type, Msg.EEPROM_REPAIR_ACK)
        self.assertEqual(r.payload[0], 1)
        self.assertEqual(list(one(self.d, Msg.EEPROM_STATUS).payload[2:]),
                         [1, 1, 1])

    def test_read_returns_the_bytes_at_the_offset(self):
        r = one(self.d, Msg.EEPROM_READ, struct.pack("<HH", 0, 8))
        off, n = struct.unpack("<HH", r.payload[:4])
        self.assertEqual((off, n), (0, 8))
        self.assertEqual(r.payload[4:4 + n], self.ee.read(0, 8))

    def test_read_is_capped_to_one_chunk(self):
        r = one(self.d, Msg.EEPROM_READ, struct.pack("<HH", 0, 4096))
        _, n = struct.unpack("<HH", r.payload[:4])
        self.assertEqual(n, Dispatcher.EEPROM_CHUNK)

    def test_write_refused_while_locked(self):
        r = one(self.d, Msg.EEPROM_WRITE,
                struct.pack("<HH", 0, 4) + b"\xde\xad\xbe\xef")
        self.assertEqual(r.type, Msg.NACK)
        self.assertEqual(r.payload[2], Err.WRITE_PROTECTED)
        # And nothing may have reached the device.
        self.assertNotEqual(self.ee.read(0, 4), b"\xde\xad\xbe\xef")

    def test_write_when_unlocked_reports_verified(self):
        self.d.eeprom_writable = True
        r = one(self.d, Msg.EEPROM_WRITE,
                struct.pack("<HH", 0, 4) + b"\xde\xad\xbe\xef")
        off, n, ok = struct.unpack("<HHB", r.payload)
        self.assertEqual((off, n, ok), (0, 4, 1))
        self.assertEqual(self.ee.read(0, 4), b"\xde\xad\xbe\xef")

    def test_write_reports_unverified_when_the_device_lies(self):
        class Deaf(FakeEEPROM):
            def write(self, offset, data):
                pass

        d, _ = make(eeprom=Deaf(), writable=True)
        r = one(d, Msg.EEPROM_WRITE, struct.pack("<HH", 0, 4) + b"\x01\x02\x03\x04")
        self.assertEqual(struct.unpack("<HHB", r.payload)[2], 0)

    def test_write_length_mismatch_is_rejected(self):
        self.d.eeprom_writable = True
        r = one(self.d, Msg.EEPROM_WRITE, struct.pack("<HH", 0, 8) + b"\x01\x02")
        self.assertEqual(r.type, Msg.NACK)
        self.assertEqual(r.payload[2], Err.BAD_PARAM)

    def test_no_eeprom_reports_a_fault_rather_than_raising(self):
        d, _ = make(eeprom=None)
        for msg in (Msg.EEPROM_STATUS, Msg.EEPROM_REPAIR,
                    Msg.EEPROM_READ, Msg.EEPROM_WRITE):
            r = one(d, msg, struct.pack("<HH", 0, 4))
            self.assertEqual(r.type, Msg.NACK, msg)
            self.assertEqual(r.payload[2], Err.EEPROM_FAULT, msg)

    def test_i2c_failure_becomes_a_nack(self):
        class Broken(FakeEEPROM):
            def read(self, offset, length):
                raise OSError("i2c timeout")

        d, _ = make(eeprom=Broken())
        r = one(d, Msg.EEPROM_READ, struct.pack("<HH", 0, 4))
        self.assertEqual(r.type, Msg.NACK)
        self.assertEqual(r.payload[2], Err.EEPROM_FAULT)


class TestConfigClamping(unittest.TestCase):
    def test_flash_is_capped_at_ten_percent(self):
        cfg = Config()
        for requested in (11, 50, 100, 255):
            apply_config_value(cfg, Cfg.FLASH_PERCENT, requested)
            self.assertEqual(cfg.flash_percent, FLASH_PERCENT_MAX, requested)

    def test_led_command_never_exceeds_the_cap(self):
        d, _ = make()
        r = one(d, Msg.SET_LED, bytes([90]))
        self.assertEqual(r.type, Msg.LED_ACK)
        self.assertLessEqual(r.payload[0], FLASH_PERCENT_MAX)


if __name__ == "__main__":
    unittest.main()


class TestUndistortOnCapture(unittest.TestCase):
    """The stored model must reach the capture path, and must fail safe."""

    MODEL = {"model": "radial_poly", "centre": [0.5, 0.5], "k": [0.3, 0.1],
             "r_valid": 0.86}

    def _cam(self, model):
        return Camera(SyntheticSource(), MediaStore(tempfile.mkdtemp()),
                      led=None, distortion_model=model)

    def test_disabled_by_default(self):
        cam = self._cam(self.MODEL)
        cfg = Config()
        cfg.image_resolution = 1
        a = cam.capture_image(cfg)
        cfg.undistort = 1
        b = cam.capture_image(cfg)
        self.assertNotEqual(cam.store.read(a.media_id),
                            cam.store.read(b.media_id))

    def test_no_model_still_returns_an_image(self):
        """An uncalibrated camera must not lose the picture."""
        cam = self._cam(None)
        cfg = Config()
        cfg.image_resolution = 1
        cfg.undistort = 1
        rec = cam.capture_image(cfg)
        self.assertGreater(rec.size, 0)

    def test_a_broken_model_falls_back_to_the_raw_frame(self):
        cam = self._cam({"model": "radial_poly"})       # missing centre and k
        cfg = Config()
        cfg.image_resolution = 1
        cfg.undistort = 1
        rec = cam.capture_image(cfg)
        self.assertGreater(rec.size, 0)

    def test_config_key_round_trips(self):
        cfg = Config()
        apply_config_value(cfg, Cfg.UNDISTORT, 1)
        self.assertEqual(cfg.undistort, 1)
        apply_config_value(cfg, Cfg.UNDISTORT, 0)
        self.assertEqual(cfg.undistort, 0)
        apply_config_value(cfg, Cfg.UNDISTORT, 99)
        self.assertEqual(cfg.undistort, 1)
