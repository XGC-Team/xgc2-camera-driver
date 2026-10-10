#!/usr/bin/env python3
"""A still scene is published once, latched, with static TF and no stream timing."""

import math
import threading
import unittest

import cv2
import numpy as np
import rospy
import rostest
import tf2_ros
from sensor_msgs.msg import CameraInfo, CompressedImage
from xgc_camera_msgs.msg import StreamInfo


NAMESPACE = "/still_camera"


def collect(topic, message_type, seconds):
    """Subscribe now and return every message received during `seconds`."""
    messages = []
    lock = threading.Lock()

    def callback(message):
        with lock:
            messages.append(message)

    subscriber = rospy.Subscriber(topic, message_type, callback, queue_size=10)
    rospy.sleep(seconds)
    subscriber.unregister()
    with lock:
        return list(messages)


class StillCameraContractTest(unittest.TestCase):
    def test_jpeg_is_latched_and_decodes_to_the_sealed_scene(self):
        message = rospy.wait_for_message(
            NAMESPACE + "/image_raw/compressed", CompressedImage, timeout=20.0
        )
        self.assertEqual(message.format, "jpeg")
        self.assertEqual(message.header.frame_id, "usb_cam_optical_frame")
        self.assertGreater(message.header.stamp.to_nsec(), 0)
        decoded = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, (48, 64, 3))
        # Quadrant colours (BGR) survive a q95 JPEG round trip within a small margin.
        expected = {
            (10, 10): (0, 0, 220),
            (10, 50): (0, 200, 0),
            (40, 10): (220, 0, 0),
            (40, 50): (200, 200, 200),
        }
        for (row, column), colour in expected.items():
            for channel in range(3):
                self.assertLessEqual(
                    abs(int(decoded[row, column, channel]) - colour[channel]), 12
                )

    def test_every_late_subscriber_gets_the_same_single_publication(self):
        first = collect(NAMESPACE + "/image_raw/compressed", CompressedImage, 2.0)
        second = collect(NAMESPACE + "/image_raw/compressed", CompressedImage, 2.0)
        # Each late subscriber receives exactly the latched message; nothing is
        # re-sent periodically.
        self.assertEqual(len(first), 1)
        self.assertEqual(len(second), 1)
        self.assertEqual(first[0].header.stamp, second[0].header.stamp)
        self.assertEqual(first[0].data, second[0].data)

    def test_camera_info_is_latched_and_matches_the_intrinsics(self):
        image = rospy.wait_for_message(
            NAMESPACE + "/image_raw/compressed", CompressedImage, timeout=20.0
        )
        info = rospy.wait_for_message(NAMESPACE + "/camera_info", CameraInfo, timeout=20.0)
        self.assertEqual((info.width, info.height), (64, 48))
        self.assertEqual(info.header.frame_id, "usb_cam_optical_frame")
        self.assertEqual(info.header.stamp, image.header.stamp)
        self.assertEqual(info.distortion_model, "plumb_bob")
        self.assertEqual(list(info.D), [0.1, -0.05, 0.001, 0.002, 0.0])
        self.assertEqual(list(info.K), [50.0, 0.0, 31.5, 0.0, 51.0, 23.5, 0.0, 0.0, 1.0])
        self.assertEqual(list(info.R), [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.assertEqual(
            list(info.P), [50.0, 0.0, 31.5, 0.0, 0.0, 51.0, 23.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        )
        again = collect(NAMESPACE + "/camera_info", CameraInfo, 1.5)
        self.assertEqual(len(again), 1)

    def test_stream_info_describes_a_single_latched_picture(self):
        info = rospy.wait_for_message(NAMESPACE + "/stream_info", StreamInfo, timeout=20.0)
        self.assertEqual(info.stream_id, "xgc2_still_ci")
        self.assertEqual(info.frame_id, "usb_cam_optical_frame")
        self.assertEqual((info.width, info.height), (64, 48))
        self.assertEqual(info.codec, StreamInfo.CODEC_MJPEG)
        self.assertEqual(info.nominal_frame_rate, 0.0)
        self.assertNotEqual(info.epoch, 0)

    def test_static_tf_chain_reproduces_the_extrinsic(self):
        buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(buffer)
        link = buffer.lookup_transform(
            "world", "usb_cam_link", rospy.Time(0), rospy.Duration(20.0)
        )
        optical = buffer.lookup_transform(
            "world", "usb_cam_optical_frame", rospy.Time(0), rospy.Duration(20.0)
        )
        # The stored translation is the camera position; the link frame sits there too.
        for transform in (link, optical):
            self.assertAlmostEqual(transform.transform.translation.x, 1.0, places=9)
            self.assertAlmostEqual(transform.transform.translation.y, 2.0, places=9)
            self.assertAlmostEqual(transform.transform.translation.z, 3.0, places=9)
        half = math.sqrt(0.5)
        q = optical.transform.rotation
        expected = (0.0, 0.0, half, half)
        actual = (q.x, q.y, q.z, q.w)
        sign = 1.0 if sum(a * e for a, e in zip(actual, expected)) >= 0 else -1.0
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, sign * e, places=9)

    def test_no_frame_timing_or_raw_stream_exists(self):
        published = dict(rospy.get_published_topics())
        self.assertIn(NAMESPACE + "/image_raw/compressed", published)
        self.assertNotIn(NAMESPACE + "/frame_timing", published)
        self.assertNotIn(NAMESPACE + "/image_raw", published)


if __name__ == "__main__":
    rospy.init_node("still_camera_contract_test")
    rostest.rosrun("xgc2_camera_driver", "still_camera_contract", StillCameraContractTest)
