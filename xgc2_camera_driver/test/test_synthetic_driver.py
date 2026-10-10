#!/usr/bin/env python3

import math
import unittest

import cv2
import numpy as np
import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from xgc_camera_msgs.msg import FrameTiming, StreamInfo


class SyntheticCameraContractTest(unittest.TestCase):
    def test_compressed_image_is_a_real_jpeg(self):
        message = rospy.wait_for_message(
            "/test_camera/image_raw/compressed", CompressedImage, timeout=15.0
        )
        self.assertEqual(message.format, "jpeg")
        self.assertEqual(message.header.frame_id, "usb_cam_optical_frame")
        self.assertGreater(message.header.stamp.to_nsec(), 0)
        decoded = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.shape, (240, 320, 3))

    def test_raw_image_is_decoded_on_demand(self):
        image = rospy.wait_for_message("/test_camera/image_raw", Image, timeout=15.0)
        self.assertEqual((image.width, image.height), (320, 240))
        self.assertEqual(image.encoding, "bgr8")
        self.assertEqual(image.header.frame_id, "usb_cam_optical_frame")
        self.assertGreater(image.header.stamp.to_nsec(), 0)

    def test_frame_timing_follows_the_jpeg_stream(self):
        first = rospy.wait_for_message("/test_camera/frame_timing", FrameTiming, timeout=15.0)
        second = rospy.wait_for_message("/test_camera/frame_timing", FrameTiming, timeout=15.0)
        self.assertEqual(first.stream_id, "xgc2_synthetic_ci")
        self.assertEqual(second.epoch, first.epoch)
        self.assertGreater(second.frame_sequence, first.frame_sequence)
        self.assertTrue(first.keyframe)
        self.assertTrue(first.source_time_valid)
        self.assertGreater(first.encoded_size_bytes, 0)
        self.assertEqual(first.frame_id, "usb_cam_optical_frame")

    def test_stream_info_is_latched_for_late_subscribers(self):
        info = rospy.wait_for_message("/test_camera/stream_info", StreamInfo, timeout=15.0)
        self.assertEqual(info.stream_id, "xgc2_synthetic_ci")
        self.assertEqual((info.width, info.height), (320, 240))
        self.assertEqual(info.codec, StreamInfo.CODEC_MJPEG)
        self.assertEqual(info.bitstream_format, StreamInfo.BITSTREAM_FORMAT_JPEG)
        self.assertEqual(info.transport_mask, StreamInfo.TRANSPORT_ROS_COMPRESSED_IMAGE)
        self.assertEqual(info.nominal_frame_rate, 10.0)

    def test_camera_info_is_the_default_pinhole(self):
        info = rospy.wait_for_message("/test_camera/camera_info", CameraInfo, timeout=15.0)
        self.assertEqual((info.width, info.height), (320, 240))
        self.assertEqual(info.header.frame_id, "usb_cam_optical_frame")
        focal = 320.0 / (2.0 * math.tan(math.radians(110.0) / 2.0))
        self.assertEqual(info.distortion_model, "plumb_bob")
        self.assertEqual(list(info.D), [0.0] * 5)
        self.assertAlmostEqual(info.K[0], focal, places=9)
        self.assertEqual(info.K[1], 0.0)
        self.assertAlmostEqual(info.K[2], 159.5, places=9)
        self.assertEqual(info.K[3], 0.0)
        self.assertAlmostEqual(info.K[4], focal, places=9)
        self.assertAlmostEqual(info.K[5], 119.5, places=9)
        self.assertEqual(list(info.K[6:8]), [0.0, 0.0])
        self.assertEqual(info.K[8], 1.0)

    def test_diagnostics_report_the_uncalibrated_camera(self):
        deadline = rospy.Time.now() + rospy.Duration(15.0)
        camera_status = None
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            diagnostics = rospy.wait_for_message("/diagnostics", DiagnosticArray, timeout=10.0)
            for status in diagnostics.status:
                values = {item.key: item.value for item in status.values}
                if status.name.endswith("camera stream") and values.get(
                    "source_kind"
                ) == "synthetic":
                    camera_status = status
                    break
            if camera_status is not None:
                break
        self.assertIsNotNone(camera_status)
        self.assertEqual(camera_status.level, DiagnosticStatus.WARN)
        self.assertIn("CameraInfo is not calibrated", camera_status.message)
        values = {item.key: item.value for item in camera_status.values}
        self.assertEqual(values["camera_info_calibrated"], "False")
        self.assertGreater(int(values["published_frames"]), 0)


if __name__ == "__main__":
    rospy.init_node("synthetic_camera_contract_test")
    rostest.rosrun("xgc2_camera_driver", "synthetic_camera_contract", SyntheticCameraContractTest)
