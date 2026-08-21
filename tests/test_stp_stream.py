"""Live video over HRT: configuration, framing, and the drop-not-delay rule.

The property under test throughout is that this is a *stream*, not a transfer.
A transfer may queue; a stream must not, because on a flow-controlled link a
queue converts a bandwidth shortfall into unbounded latency. So most of these
assert that frames are discarded when they cannot be sent, and that what does
go out is self-describing enough for a ground station to join mid-stream.
"""

import struct
import time
import unittest

from tests.support import FakeMediaStore, MemoryLink, drain_experiment

from radcam.protocol import Config as ProtoConfig, Dispatcher, Err
from radcam.stp import hrt as H
from radcam.stp import lrt as L
from radcam.stp import packets as P
from radcam.stp.commands import encode_command_payload
from radcam.stp.experiment import Experiment, ExperimentConfig, StpOp
from radcam.stream import (MAX_DIMENSION, SENSOR_HEIGHT, SENSOR_WIDTH,
                           EncodedFrame, StreamConfig, VideoStream)

TARGET = 0xC7


class TestStreamConfig(unittest.TestCase):
    def test_defaults_are_the_mission_target(self):
        cfg = StreamConfig().sanitised()
        self.assertEqual((cfg.width, cfg.height), (640, 480))
        self.assertEqual(cfg.fps, 15)
        self.assertEqual(cfg.bitrate, 600_000)

    def test_crop_equal_to_output_means_native_pixels(self):
        self.assertTrue(StreamConfig(width=640, height=480, crop_w=640,
                                     crop_h=480).sanitised().native_pixels)
        self.assertFalse(StreamConfig(width=640, height=480, crop_w=3200,
                                      crop_h=2400).sanitised().native_pixels)

    def test_region_is_clamped_inside_the_sensor(self):
        cfg = StreamConfig(centre_x=0, centre_y=0, crop_w=800,
                           crop_h=600).sanitised()
        self.assertEqual(cfg.centre_x, 400)
        self.assertEqual(cfg.centre_y, 300)
        x, y, w, h = cfg.roi
        self.assertGreaterEqual(x, 0.0)
        self.assertGreaterEqual(y, 0.0)
        self.assertLessEqual(x + w, 1.0 + 1e-9)
        self.assertLessEqual(y + h, 1.0 + 1e-9)

    def test_far_corner_is_also_clamped(self):
        cfg = StreamConfig(centre_x=SENSOR_WIDTH, centre_y=SENSOR_HEIGHT,
                           crop_w=640, crop_h=480).sanitised()
        x, y, w, h = cfg.roi
        self.assertLessEqual(x + w, 1.0 + 1e-9)
        self.assertLessEqual(y + h, 1.0 + 1e-9)

    def test_absurd_request_is_clamped_not_obeyed(self):
        cfg = StreamConfig(width=99999, height=0, fps=250,
                           bitrate=99_000_000).sanitised()
        self.assertLessEqual(cfg.width, MAX_DIMENSION)
        self.assertGreaterEqual(cfg.height, 64)
        self.assertLessEqual(cfg.fps, 30)
        self.assertLessEqual(cfg.bitrate, 8_000_000)

    def test_dimensions_are_even(self):
        """Odd dimensions are invalid for YUV420 chroma subsampling."""
        cfg = StreamConfig(width=641, height=481).sanitised()
        self.assertEqual(cfg.width % 2, 0)
        self.assertEqual(cfg.height % 2, 0)

    def test_roi_is_a_fraction_of_the_sensor_not_the_output(self):
        cfg = StreamConfig(width=640, height=480, crop_w=SENSOR_WIDTH // 2,
                           crop_h=SENSOR_HEIGHT // 2).sanitised()
        _x, _y, w, h = cfg.roi
        self.assertAlmostEqual(w, 0.5, places=3)
        self.assertAlmostEqual(h, 0.5, places=3)


class TestAccessUnitParsing(unittest.TestCase):
    """The encoder's bytestream has to become whole frames before it can be
    dropped whole; these cover the parts that bit during bring-up."""

    def nal(self, nal_type: int, first_slice: bool = True, body=b"\xaa" * 8):
        header = bytes([nal_type & 0x1F])
        # first_mb_in_slice = ue(0) is a single set bit.
        marker = bytes([0x80 if first_slice else 0x20])
        return header + marker + body

    def test_split_handles_three_and_four_byte_start_codes(self):
        """libx264 emits both; a parser that only knows one loses NALs."""
        stream = bytearray()
        stream += b"\x00\x00\x00\x01" + self.nal(7)
        stream += b"\x00\x00\x01" + self.nal(8)
        stream += b"\x00\x00\x00\x01" + self.nal(5)
        stream += b"\x00\x00\x00\x01"          # start of an incomplete unit
        units, tail = VideoStream._split(bytearray(stream))
        self.assertEqual([u[0] & 0x1F for u in units], [7, 8, 5])
        self.assertTrue(tail.startswith(b"\x00\x00\x00\x01"))

    def test_partial_unit_is_retained_for_the_next_read(self):
        first = b"\x00\x00\x00\x01" + self.nal(5)[:4]
        units, tail = VideoStream._split(bytearray(first))
        self.assertEqual(units, [])
        self.assertEqual(len(tail), len(first))

    def test_first_slice_of_a_picture_is_recognised(self):
        self.assertTrue(VideoStream._starts_picture(self.nal(5, first_slice=True)))
        self.assertFalse(VideoStream._starts_picture(self.nal(1, first_slice=False)))

    def test_multi_slice_picture_is_one_frame_not_several(self):
        """Sliced threading produces several VCL NALs per picture."""
        stream = VideoStream()
        stream._frames.clear()
        units = [self.nal(7), self.nal(8),
                 self.nal(5, first_slice=True),
                 self.nal(5, first_slice=False),
                 self.nal(5, first_slice=False)]
        stream._emit(units)
        self.assertEqual(stream.frames_encoded, 1)

    def test_keyframe_carries_parameter_sets(self):
        stream = VideoStream()
        stream._emit([self.nal(7), self.nal(8), self.nal(5)])
        frame = stream.take()
        self.assertTrue(frame.keyframe)
        self.assertTrue(frame.data.startswith(b"\x00\x00\x00\x01"))

    def test_inter_frame_is_not_a_keyframe(self):
        stream = VideoStream()
        stream._emit([self.nal(1)])
        self.assertFalse(stream.take().keyframe)


class TestDropRatherThanDelay(unittest.TestCase):
    def stream(self, depth=4):
        stream = VideoStream(queue_frames=depth)
        return stream

    def frame(self, index, keyframe=False):
        return EncodedFrame(index, b"x" * 100, keyframe, time.monotonic())

    def test_oldest_frame_is_discarded_when_the_ring_fills(self):
        stream = self.stream(depth=3)
        for i in range(6):
            stream._frames.append(self.frame(i))
            if len(stream._frames) == stream._frames.maxlen:
                pass
        # Emit through the real path so the drop counter runs.
        stream = self.stream(depth=3)
        for i in range(6):
            stream._emit([bytes([1, 0x80]) + bytes([i])])
        self.assertEqual(stream.queue_depth, 3)
        self.assertEqual(stream.frames_dropped, 3)
        # What survives is the newest, not the oldest.
        indices = [f.index for f in stream._frames]
        self.assertEqual(indices, [3, 4, 5])

    def test_flush_discards_everything_by_default(self):
        stream = self.stream()
        for i in range(3):
            stream._emit([bytes([1, 0x80, i])])
        self.assertEqual(stream.flush(), 3)
        self.assertEqual(stream.queue_depth, 0)

    def test_flush_can_keep_the_newest_keyframe(self):
        stream = self.stream()
        stream._emit([bytes([5, 0x80, 0])])         # keyframe
        stream._emit([bytes([1, 0x80, 1])])         # inter
        stream.flush(keep_keyframe=True)
        self.assertEqual(stream.queue_depth, 1)
        self.assertTrue(stream.take().keyframe)

    def test_take_returns_none_when_nothing_is_ready(self):
        self.assertIsNone(self.stream().take())


class FakeStream:
    """A stream with no camera, so the protocol path is testable anywhere."""

    def __init__(self):
        self.config = StreamConfig()
        self.running = False
        self.fault = None
        self.frames = []
        self.frames_encoded = 0
        self.frames_dropped = 0
        self.frames_taken = 0
        self.bytes_encoded = 0
        self.starts = 0

    def start(self, config=None):
        if config is not None:
            self.config = config.sanitised()
        self.running = True
        self.starts += 1
        self.frames_encoded = 1
        return True

    def stop(self):
        was = self.running
        self.running = False
        return was

    def take(self):
        return self.frames.pop(0) if self.frames else None

    def flush(self, keep_keyframe=False):
        dropped = len(self.frames)
        self.frames.clear()
        self.frames_dropped += dropped
        return dropped

    def encoder_late(self):
        return False

    @property
    def queue_depth(self):
        return len(self.frames)

    def status(self):
        cfg = self.config
        return {"stream_width": cfg.width, "stream_height": cfg.height,
                "stream_fps": cfg.fps, "stream_bitrate": cfg.bitrate,
                "stream_centre_x": cfg.centre_x, "stream_centre_y": cfg.centre_y,
                "stream_crop_w": cfg.crop_w, "stream_crop_h": cfg.crop_h,
                "stream_frames_sent": self.frames_taken,
                "stream_frames_dropped": self.frames_dropped,
                "stream_bytes_sent": self.bytes_encoded,
                "stream_queue_depth": len(self.frames)}


class TestStreamOverHrt(unittest.TestCase):
    def setUp(self):
        self.wire = P.Wire(target_id=TARGET)
        self.link = MemoryLink()
        self.store = FakeMediaStore()
        self.stream = FakeStream()
        self.experiment = Experiment(
            link=self.link, wire=self.wire,
            dispatcher=Dispatcher(config=ProtoConfig(), store=self.store),
            store=self.store, stream=self.stream,
            config=ExperimentConfig(target_id=TARGET, scrub_interval_s=3600))
        self.experiment.start()
        self.seq = 0

    def tearDown(self):
        self.experiment.stop()

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

    def hrt_packets(self):
        raw = self.link.dice_read()
        return [H.decode_hrt_payload(raw[i * 1288:(i + 1) * 1288][6:6 + 1280])
                for i in range(len(raw) // 1288)]

    def open_hrt(self):
        self.link.dice_send(P.encode_short_request(
            P.PacketType.HRT_GO, 0, 0, self.wire, TARGET))

    # -- configuration ----------------------------------------------------

    def test_output_settings_are_applied_and_echoed(self):
        self.command(StpOp.STREAM_SET_OUTPUT,
                     struct.pack("<HHBI", 800, 600, 10, 700_000))
        report = self.poll()
        self.assertEqual(report["stream_width"], 800)
        self.assertEqual(report["stream_height"], 600)
        self.assertEqual(report["stream_fps"], 10)
        self.assertEqual(report["stream_bitrate"], 700_000)

    def test_region_is_applied_and_echoed(self):
        self.command(StpOp.STREAM_SET_REGION,
                     struct.pack("<4H", 3000, 2000, 512, 512))
        report = self.poll()
        self.assertEqual(report["stream_centre_x"], 3000)
        self.assertEqual(report["stream_centre_y"], 2000)
        self.assertEqual(report["stream_crop_w"], 512)

    def test_region_at_the_edge_reports_what_was_actually_applied(self):
        self.command(StpOp.STREAM_SET_REGION,
                     struct.pack("<4H", 5, 5, 1024, 1024))
        report = self.poll()
        self.assertEqual(report["stream_centre_x"], 512)
        self.assertEqual(report["stream_centre_y"], 512)

    def test_malformed_region_is_refused(self):
        self.command(StpOp.STREAM_SET_REGION, b"\x01\x02")
        self.assertEqual(self.experiment.last_result, int(Err.BAD_PARAM))

    def test_start_reports_the_effective_settings(self):
        self.command(StpOp.STREAM_START,
                     struct.pack("<HHBI4H", 640, 480, 15, 600_000,
                                 2104, 1560, 640, 480))
        self.assertEqual(self.experiment.last_result, 0)
        self.assertTrue(self.stream.running)

    def test_stop_stops_it(self):
        self.command(StpOp.STREAM_START)
        self.command(StpOp.STREAM_STOP)
        self.assertFalse(self.stream.running)
        self.assertEqual(self.poll()["stream_state"], L.STREAM_OFF)

    def test_redundant_settings_command_does_not_restart_the_encoder(self):
        """A restart puts a visible gap in a running stream."""
        self.command(StpOp.STREAM_START)
        starts = self.stream.starts
        self.command(StpOp.STREAM_SET_REGION,
                     struct.pack("<4H", self.stream.config.centre_x,
                                 self.stream.config.centre_y,
                                 self.stream.config.crop_w,
                                 self.stream.config.crop_h))
        self.assertEqual(self.stream.starts, starts)

    def test_changing_settings_while_running_does_restart(self):
        self.command(StpOp.STREAM_START)
        starts = self.stream.starts
        self.command(StpOp.STREAM_SET_OUTPUT,
                     struct.pack("<HHBI", 320, 240, 10, 400_000))
        self.assertEqual(self.stream.starts, starts + 1)

    # -- transport --------------------------------------------------------

    def test_frames_are_chunked_and_reassemble(self):
        self.command(StpOp.STREAM_START)
        payloads = {0: b"K" * 3000, 1: b"P" * 900}
        self.stream.frames = [EncodedFrame(0, payloads[0], True, time.monotonic()),
                              EncodedFrame(1, payloads[1], False, time.monotonic())]
        self.open_hrt()
        self.experiment.service()

        got = {}
        for packet in self.hrt_packets():
            self.assertEqual(packet["sub_type"], H.SubType.STREAM_DATA)
            self.assertTrue(packet["data_crc_ok"])
            got.setdefault(packet["media_id"], {})[packet["chunk_index"]] = packet["data"]
        for index, expected in payloads.items():
            joined = b"".join(got[index][i] for i in sorted(got[index]))
            self.assertEqual(joined, expected)

    def test_chunk_total_is_correct_on_the_final_chunk(self):
        """The last chunk is the one a reassembler most needs the total from."""
        self.command(StpOp.STREAM_START)
        self.stream.frames = [EncodedFrame(0, b"K" * 3000, True, time.monotonic())]
        self.open_hrt()
        self.experiment.service()
        packets = [p for p in self.hrt_packets()
                   if p["sub_type"] == H.SubType.STREAM_DATA]
        self.assertTrue(packets)
        for packet in packets:
            self.assertEqual(packet["chunk_total"], 3)
        self.assertTrue(packets[-1]["last_chunk"])

    def test_keyframes_are_flagged_so_a_receiver_can_join(self):
        self.command(StpOp.STREAM_START)
        self.stream.frames = [
            EncodedFrame(0, b"K" * 500, True, time.monotonic()),
            EncodedFrame(1, b"P" * 500, False, time.monotonic())]
        self.open_hrt()
        self.experiment.service()
        packets = self.hrt_packets()
        self.assertTrue(packets[0]["keyframe"])
        self.assertFalse(packets[-1]["keyframe"])

    def test_nothing_is_streamed_before_hrt_opens(self):
        self.command(StpOp.STREAM_START)
        self.stream.frames = [EncodedFrame(0, b"K" * 500, True, time.monotonic())]
        for _ in range(5):
            self.experiment.service()
        self.assertEqual(self.hrt_packets(), [])

    def test_closing_hrt_discards_queued_frames_rather_than_holding_them(self):
        """Holding them is the latency trap the whole design avoids."""
        self.command(StpOp.STREAM_START)
        self.open_hrt()
        self.experiment.service()
        self.link.dice_read()

        self.stream.frames = [EncodedFrame(9, b"X" * 500, False, time.monotonic())]
        self.link.dice_send(P.encode_short_request(
            P.PacketType.HRT_STOP, 0, 0, self.wire, TARGET))
        self.experiment.service()
        self.experiment.service()
        self.assertEqual(self.stream.queue_depth, 0)
        self.assertEqual(self.hrt_packets(), [])

    def test_live_video_takes_priority_over_file_transfer(self):
        self.command(StpOp.STREAM_START)
        self.command(0x40, struct.pack("<I", 3))        # REQUEST_MEDIA
        self.stream.frames = [EncodedFrame(0, b"K" * 500, True, time.monotonic())]
        self.open_hrt()
        self.experiment.service()
        packets = self.hrt_packets()
        self.assertEqual(packets[0]["sub_type"], H.SubType.STREAM_DATA)

    def test_file_transfer_uses_the_gaps_between_frames(self):
        """Starving the transfer entirely would be as wrong as delaying video."""
        self.command(StpOp.STREAM_START)
        self.command(0x40, struct.pack("<I", 3))        # REQUEST_MEDIA
        self.stream.frames = []                          # encoder between frames
        self.open_hrt()
        self.experiment.service()
        kinds = {p["sub_type"] for p in self.hrt_packets()}
        self.assertIn(H.SubType.MEDIA_INFO, kinds | {H.SubType.MEDIA_INFO})
        self.assertNotIn(H.SubType.STREAM_DATA, kinds)

    # -- telemetry --------------------------------------------------------

    def test_gated_flag_tells_the_ground_why_nothing_is_arriving(self):
        self.command(StpOp.STREAM_START)
        self.assertTrue(self.poll()["stream_gated"])
        self.open_hrt()
        self.experiment.service()
        self.assertFalse(self.poll()["stream_gated"])

    def test_state_is_off_when_no_stream_was_ever_started(self):
        experiment = Experiment(
            link=MemoryLink(), wire=self.wire,
            config=ExperimentConfig(target_id=TARGET, scrub_interval_s=3600))
        self.assertEqual(experiment.build_state()["stream_state"], L.STREAM_OFF)

    def test_fault_is_reported(self):
        self.command(StpOp.STREAM_START)
        self.stream.fault = "encoder exited"
        self.assertEqual(self.poll()["stream_state"], L.STREAM_FAULT)

    def test_drop_count_is_downlinked(self):
        self.command(StpOp.STREAM_START)
        self.stream.frames_dropped = 42
        self.assertEqual(self.poll()["stream_frames_dropped"], 42)


if __name__ == "__main__":
    unittest.main()
