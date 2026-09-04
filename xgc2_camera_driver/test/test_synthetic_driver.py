#!/usr/bin/env python3

import math
import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import CameraInfo, Image


class SyntheticCameraContractTest(unittest.TestCase):
    def test_image_and_camera_info_share_the_contract(self):
        image = rospy.wait_for_message("/test_camera/image_raw", Image, timeout=10.0)
        info = rospy.wait_for_message("/test_camera/camera_info", CameraInfo, timeout=10.0)
        self.assertEqual((image.width, image.height), (320, 240))
        self.assertEqual((info.width, info.height), (320, 240))
        self.assertEqual(image.encoding, "bgr8")
        self.assertEqual(image.header.frame_id, "usb_cam_optical_frame")
        self.assertEqual(info.header.frame_id, image.header.frame_id)
        self.assertGreater(image.header.stamp.to_nsec(), 0)
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

        deadline = rospy.Time.now() + rospy.Duration(10.0)
        camera_status = None
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            diagnostics = rospy.wait_for_message("/diagnostics", DiagnosticArray, timeout=10.0)
            for status in diagnostics.status:
                values = {item.key: item.value for item in status.values}
                if status.name.endswith("camera stream") and values.get(
                    "backend"
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
