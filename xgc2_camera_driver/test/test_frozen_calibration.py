#!/usr/bin/env python3
"""Frozen physical calibration on the native process, using synthetic pixels."""
import json
import time
import threading
import unittest
import rospy
import rostest
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from foxglove_msgs.msg import CompressedVideo
from xgc_camera_msgs.msg import StreamInfo, FrameTiming
from tf2_msgs.msg import TFMessage
from test_media_driver import CameraProcess, RtpReceiver


class FrozenCalibrationTest(unittest.TestCase):
    def test_live_apply_updates_tf_and_every_frame_contract_without_reopening(self):
        rtp = RtpReceiver()
        camera = None
        subscriptions = []
        seen = {}
        lock = threading.Lock()
        def record(key):
            def save(message):
                with lock: seen[key] = message
            return save
        def wait(predicate, seconds=10):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                with lock:
                    if predicate(): return
                time.sleep(.03)
            self.fail('live camera contract did not settle: ' + ','.join(seen))
        try:
            camera = CameraProcess('live_extrinsic_camera', {
                'source_kind':'synthetic','camera_name':'world','frame_id':'world_camera_optical_frame',
                'width':640,'height':360,'framerate':15,'preview_max_width':320,'preview_max_height':180},rtp.port)
            description = camera.wait_for_socket()
            instance = camera.instance
            for key, topic, kind in [('info','camera_info',CameraInfo),('image','image_raw/compressed',CompressedImage),
                                      ('preview','video_h264',CompressedVideo),('stream','stream_info',StreamInfo),('timing','frame_timing',FrameTiming)]:
                subscriptions.append(rospy.Subscriber('/live_extrinsic_camera/'+topic,kind,record(key),queue_size=1))
            subscriptions.append(rospy.Subscriber('/tf_static',TFMessage,record('tf'),queue_size=1))
            wait(lambda:'info' in seen and 'image' in seen and 'preview' in seen)
            before_k = list(seen['info'].K)
            resolved = {'schemaVersion':1,'resolutionId':'live-apply-one','cameraName':'world','choiceMode':'asset','status':'calibrated',
                'sourceMode':'phy','asset':{'resourceId':'asset-one','commitId':'commit-one'},'targetCoordinates':{'frame':'world'},
                'sourceOpticalFrame':'usb_cam_optical_frame','resolvedOpticalPose':{'translation':[3,4,5],'quaternionXyzw':[0,0,0,1]}}
            receipt = camera.json('POST','/v1/media/sources/front/extrinsic',resolved)
            self.assertEqual(camera.instance,instance)
            self.assertEqual(receipt['resolvedExtrinsic'],resolved)
            self.assertEqual(receipt['asset'],resolved['asset'])
            self.assertEqual(receipt['frame_id'],'usb_cam_optical_frame')
            self.assertGreater(int(receipt['appliedAtUnixNs']),0)
            def updated():
                return all(key in seen and (seen[key].header.frame_id if key in ('info','image') else seen[key].frame_id)=='usb_cam_optical_frame'
                           for key in ('info','image','preview','stream','timing')) and 'tf' in seen and any(t.child_frame_id=='usb_cam_link' and t.transform.translation.x==3 for t in seen['tf'].transforms)
            wait(updated)
            self.assertEqual(list(seen['info'].K),before_k)
            self.assertEqual(camera.json('POST','/v1/media/sources/front/extrinsic',resolved),receipt)
            self.assertEqual(camera.json('GET','/v1/media/sources/front/status')['appliedExtrinsic'],receipt)
            self.assertIsNone(camera.process.poll())
            capture=camera.json('GET','/v1/media/sources/front/describe')
            self.assertEqual(capture['frameId'],'usb_cam_optical_frame')
        finally:
            for subscription in subscriptions: subscription.unregister()
            if camera is not None: camera.stop()
            rtp.close()

    def test_frozen_camera_info_and_transform_use_the_resolved_optical_frame(self):
        intrinsic = {"image_width": 640, "image_height": 360,
            "camera_matrix": {"rows": 3, "cols": 3, "data": [500, 0, 320, 0, 501, 180, 0, 0, 1]},
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {"rows": 1, "cols": 5, "data": [0.1, 0, 0, 0, 0]},
            "rectification_matrix": {"rows": 3, "cols": 3, "data": [1, 0, 0, 0, 1, 0, 0, 0, 1]},
            "projection_matrix": {"rows": 3, "cols": 4, "data": [500, 0, 320, 0, 0, 501, 180, 0, 0, 0, 1, 0]}}
        extrinsic = {"schemaVersion": 1, "cameraName": "world", "status": "calibrated",
            "sourceMode": "phy", "targetCoordinates": {"frame": "world"},
            "sourceOpticalFrame": "usb_cam_optical_frame",
            "resolvedOpticalPose": {"translation": [3, 4, 5], "quaternionXyzw": [0, 0, 0, 1]}}
        rtp = RtpReceiver()
        camera = None
        try:
            camera = CameraProcess("frozen_camera", {
                "source_kind": "synthetic", "camera_name": "world", "width": 640, "height": 360,
                "framerate": 10, "intrinsic_json": json.dumps(intrinsic),
                "resolved_extrinsic_json": json.dumps(extrinsic)}, rtp.port)
            camera.wait_for_socket()
            info = rospy.wait_for_message("/frozen_camera/camera_info", CameraInfo, timeout=10)
            self.assertEqual(info.header.frame_id, "usb_cam_optical_frame")
            self.assertEqual((info.width, info.height), (640, 360))
            self.assertEqual(list(info.K), intrinsic["camera_matrix"]["data"])
            self.assertEqual(list(info.D), intrinsic["distortion_coefficients"]["data"])
            transforms = rospy.wait_for_message("/tf_static", TFMessage, timeout=10).transforms
            parent = next(t for t in transforms if t.child_frame_id == "usb_cam_link")
            optical = next(t for t in transforms if t.child_frame_id == "usb_cam_optical_frame")
            self.assertEqual(parent.header.frame_id, "world")
            self.assertEqual([parent.transform.translation.x, parent.transform.translation.y,
                              parent.transform.translation.z], [3, 4, 5])
            self.assertEqual(optical.header.frame_id, "usb_cam_link")
            self.assertEqual(camera.json("GET", "/v1/describe")["facts"]["calibrated"], True)
        finally:
            if camera is not None:
                camera.stop()
            rtp.close()


if __name__ == "__main__":
    rospy.init_node("frozen_calibration_test")
    rostest.rosrun("xgc2_camera_driver", "frozen_calibration", FrozenCalibrationTest)
