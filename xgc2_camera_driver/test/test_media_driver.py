#!/usr/bin/env python3
"""The camera process end to end: ROS topics, the media-edge camera-source
service on a Unix socket, RTP on loopback and the 1080p preview topic.

The test starts the node itself so that it owns the private runtime directory
the service socket lives in, exactly as the process supervisor does.
"""

import http.client
import json
import os
import shutil
import signal
import socket
import subprocess
import tempfile
import threading
import time
import unittest
import uuid

import cv2
import numpy as np
import rospy
import rostest
from foxglove_msgs.msg import CompressedVideo
from sensor_msgs.msg import CompressedImage

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


class UnixConnection(http.client.HTTPConnection):
    def __init__(self, path, timeout=10.0):
        super().__init__("localhost", timeout=timeout)
        self._path = path

    def connect(self):
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        sock.connect(self._path)
        self.sock = sock


def call(path, method, target, body=None, instance=None, timeout_ms=8000):
    connection = UnixConnection(path, timeout=timeout_ms / 1000.0 + 2.0)
    headers = {"X-Request-ID": uuid.uuid4().hex, "X-Xrpc-Timeout-Ms": str(timeout_ms)}
    if instance:
        headers["X-Xrpc-Instance-ID"] = instance
    payload = None
    if body is not None:
        headers["Content-Type"] = "application/json"
        payload = json.dumps(body)
    connection.request(method, target, body=payload, headers=headers)
    response = connection.getresponse()
    data = response.read()
    connection.close()
    return response.status, dict(response.getheaders()), data


class RtpReceiver:
    """Receives RTP on an ephemeral loopback port and records the datagrams."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.1)
        self.port = self.sock.getsockname()[1]
        self.packets = []
        self._stop = False
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop:
            try:
                data = self.sock.recv(2048)
            except socket.timeout:
                continue
            with self._lock:
                self.packets.append((time.time(), data))

    def snapshot(self):
        with self._lock:
            return list(self.packets)

    def close(self):
        self._stop = True
        self._thread.join()
        self.sock.close()


def access_units(packets):
    """Groups RTP packets into access units by marker bit; returns NAL types per unit."""
    units, current = [], []
    for _, packet in packets:
        assert packet[0] >> 6 == 2, "RTP version"
        assert packet[1] & 0x7F == 96, "payload type"
        payload = packet[12:]
        nal_type = payload[0] & 0x1F
        if nal_type == 28:  # FU-A: the type is in the fragment header
            if payload[1] & 0x80:
                current.append(payload[1] & 0x1F)
        else:
            current.append(nal_type)
        if packet[1] & 0x80:
            units.append(current)
            current = []
    return units


def is_idr(unit):
    """SPS and PPS first, an IDR slice last; an SEI may sit in between."""
    return unit[:2] == [7, 8] and unit[-1] == 5 and set(unit) <= {5, 6, 7, 8}


class CameraProcess:
    def __init__(self, namespace, parameters, rtp_port):
        self.directory = tempfile.mkdtemp(prefix="xgc2_camera_rostest_")
        os.chmod(self.directory, 0o700)
        self.socket = os.path.join(self.directory, "camera.sock")
        self.namespace = namespace
        arguments = [
            "rosrun", "xgc2_camera_driver", "xgc2_camera_driver_node",
            "__name:=camera", "__ns:=/" + namespace,
            "_control_socket:=" + self.socket,
            "_rtp_port:=%d" % rtp_port,
            "_h264_encoder:=libx264",
            "_source_id:=front",
        ] + ["_%s:=%s" % item for item in parameters.items()]
        self.process = subprocess.Popen(arguments, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.log = []
        self._reader = threading.Thread(target=self._drain, daemon=True)
        self._reader.start()
        self.instance = None

    def _drain(self):
        for line in self.process.stdout:
            self.log.append(line.decode("utf-8", "replace"))

    def wait_for_socket(self, timeout=20.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.process.poll() is not None:
                raise AssertionError("camera exited early:\n" + "".join(self.log[-30:]))
            if os.path.exists(self.socket):
                try:
                    status, _, data = call(self.socket, "GET", "/v1/describe")
                except OSError:
                    time.sleep(0.05)
                    continue
                if status == 200:
                    self.instance = json.loads(data)["instance_id"]
                    return json.loads(data)
            time.sleep(0.05)
        raise AssertionError("camera service did not appear:\n" + "".join(self.log[-30:]))

    def request(self, method, target, body=None, timeout_ms=8000):
        status, headers, data = call(
            self.socket, method, target, body, instance=self.instance, timeout_ms=timeout_ms
        )
        return status, headers, data

    def json(self, method, target, body=None, expect=200):
        status, _, data = self.request(method, target, body)
        assert status == expect, "%s %s -> %d %r" % (method, target, status, data[:300])
        return json.loads(data)

    def stop(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        shutil.rmtree(self.directory, ignore_errors=True)


SOURCE = "/v1/media/sources/front"


class SyntheticCameraMediaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rtp = RtpReceiver()
        cls.camera = CameraProcess(
            "media_camera",
            {
                "source_kind": "synthetic",
                "width": 640,
                "height": 360,
                "framerate": 30.0,
                "camera_name": "media_ci",
                "preview_max_width": 320,
                "preview_max_height": 180,
                "synthetic_frames": 8,
            },
            cls.rtp.port,
        )
        cls.camera.wait_for_socket()

    @classmethod
    def tearDownClass(cls):
        cls.camera.stop()
        cls.rtp.close()

    def test_01_describe_becomes_ready_with_the_first_picture(self):
        status, _, data = self.camera.request(
            "GET", "/v1/describe?wait_ready_ms=15000", timeout_ms=20000
        )
        self.assertEqual(status, 200, data)
        description = json.loads(data)
        self.assertTrue(description["ready"])
        self.assertEqual(description["service"], "camera-source")
        self.assertEqual(description["instance_id"], self.camera.instance)
        self.assertEqual(description["facts"]["source_kind"], "synthetic")
        self.assertTrue(description["facts"]["frames_flowing"])
        self.assertEqual(description["service_ref"]["endpoint"]["address"], self.camera.socket)
        source = self.camera.json("GET", SOURCE + "/describe")
        self.assertEqual((source["width"], source["height"]), (640, 360))
        self.assertEqual(source["rtpPort"], self.rtp.port)
        self.assertEqual(source["codec"], "H264")

    def test_02_nothing_is_encoded_until_a_lease_is_held(self):
        self.camera.json("GET", "/v1/describe?wait_ready_ms=15000")
        time.sleep(0.5)
        self.assertEqual(self.rtp.snapshot(), [])
        self.assertEqual(self.camera.json("GET", SOURCE + "/status")["state"], "idle")

    def test_03_lease_streams_rtp_that_starts_with_an_idr_and_stops_cleanly(self):
        self.camera.json("GET", "/v1/describe?wait_ready_ms=15000")
        before = len(self.rtp.snapshot())
        receipt = self.camera.json("POST", SOURCE + "/start", {})
        self.assertEqual(receipt["completion"], "applied")
        self.assertEqual(receipt["state"], "active")
        time.sleep(1.5)
        packets = self.rtp.snapshot()[before:]
        self.assertGreater(len(packets), 20)
        units = access_units(packets)
        self.assertGreaterEqual(len(units), 15)
        self.assertTrue(is_idr(units[0]), units[0])
        # request-keyframe forces another IDR on demand
        mark = len(self.rtp.snapshot())
        self.camera.json("POST", SOURCE + "/request-keyframe", {})
        time.sleep(0.6)
        later = access_units(self.rtp.snapshot()[mark:])
        self.assertTrue(any(is_idr(unit) for unit in later[:4]), later[:4])
        stopped = self.camera.json("POST", SOURCE + "/stop", {})
        self.assertEqual(stopped["state"], "idle")
        time.sleep(0.3)
        settled = len(self.rtp.snapshot())
        time.sleep(0.5)
        self.assertEqual(len(self.rtp.snapshot()), settled, "no RTP after the lease ended")

    def test_04_capture_returns_the_jpeg_the_ros_topic_carries(self):
        self.camera.json("GET", "/v1/describe?wait_ready_ms=15000")
        status, headers, data = self.camera.request(
            "POST", SOURCE + "/capture", {"snapshotId": "ci", "includeRgb": False}
        )
        self.assertEqual(status, 200)
        content_type = headers.get("Content-Type", "")
        self.assertTrue(content_type.startswith("multipart/mixed; boundary="), content_type)
        boundary = ("--" + content_type.split("boundary=")[1]).encode()
        pieces = data.split(boundary)
        self.assertGreaterEqual(len(pieces), 4)
        metadata = json.loads(pieces[1].split(b"\r\n\r\n", 1)[1].rsplit(b"\r\n", 1)[0])
        self.assertTrue(metadata["ok"])
        self.assertEqual(metadata["snapshotId"], "ci")
        self.assertGreater(metadata["frameSequence"], 0)
        self.assertEqual((metadata["width"], metadata["height"]), (640, 360))
        jpeg = pieces[2].split(b"\r\n\r\n", 1)[1].rsplit(b"\r\n", 1)[0]
        self.assertEqual(len(jpeg), metadata["jpegBytes"])
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, (360, 640, 3))

    def test_05_preview_is_encoded_only_while_the_topic_has_subscribers(self):
        self.camera.json("GET", "/v1/describe?wait_ready_ms=15000")
        self.assertFalse(self.camera.json("GET", SOURCE + "/status")["native"]["preview_active"])
        received = []
        subscriber = rospy.Subscriber(
            "/media_camera/video_h264", CompressedVideo, received.append, queue_size=50
        )
        deadline = time.time() + 15
        while len(received) < 10 and time.time() < deadline:
            time.sleep(0.05)
        self.assertGreaterEqual(len(received), 10)
        first = received[0]
        self.assertEqual(first.format, "h264")
        self.assertEqual(first.frame_id, "usb_cam_optical_frame")
        data = bytes(first.data)
        self.assertTrue(data.startswith(b"\x00\x00\x00\x01") or data.startswith(b"\x00\x00\x01"))
        self.assertIn(7, [b & 0x1F for b in self._nal_headers(data)], "the first unit carries an SPS")
        self.assertTrue(self.camera.json("GET", SOURCE + "/status")["native"]["preview_active"])
        subscriber.unregister()
        deadline = time.time() + 10
        while time.time() < deadline:
            if not self.camera.json("GET", SOURCE + "/status")["native"]["preview_active"]:
                break
            time.sleep(0.1)
        self.assertFalse(self.camera.json("GET", SOURCE + "/status")["native"]["preview_active"])

    @staticmethod
    def _nal_headers(data):
        headers = []
        index = 0
        while index + 3 < len(data):
            if data[index:index + 3] == b"\x00\x00\x01":
                headers.append(data[index + 3])
                index += 3
            else:
                index += 1
        return headers

    def test_06_the_service_refuses_a_stale_instance_and_bad_requests(self):
        status, _, _ = call(self.camera.socket, "GET", SOURCE + "/status", instance="00" * 16)
        self.assertEqual(status, 409)
        status, _, _ = call(self.camera.socket, "GET", SOURCE + "/status")
        self.assertEqual(status, 409, "only discovery is unfenced")
        status, _, _ = self.camera.request("POST", SOURCE + "/start", {"unexpected": 1})
        self.assertEqual(status, 400)
        status, _, _ = self.camera.request("GET", "/v1/media/sources/other/status")
        self.assertEqual(status, 404)


class StillCameraMediaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rtp = RtpReceiver()
        cls.camera = CameraProcess(
            "still_media",
            {
                "source_kind": "still",
                "camera_name": "still_ci",
                "still_image": os.path.join(DATA, "still_scene.png"),
                "still_intrinsic_file": os.path.join(DATA, "still_intrinsic.yaml"),
                "still_extrinsic_file": os.path.join(DATA, "still_extrinsic.yaml"),
                "still_fps": 4.0,
            },
            cls.rtp.port,
        )
        cls.camera.wait_for_socket()

    @classmethod
    def tearDownClass(cls):
        cls.camera.stop()
        cls.rtp.close()

    def test_a_still_streams_one_idr_then_skip_pictures_at_its_own_rate(self):
        description = self.camera.json("GET", "/v1/describe?wait_ready_ms=15000")
        self.assertTrue(description["ready"])
        self.assertEqual(description["facts"]["source_kind"], "still")
        source = self.camera.json("GET", SOURCE + "/describe")
        self.assertEqual((source["width"], source["height"]), (64, 48))
        self.assertEqual(source["fps"], 4.0)
        self.assertEqual(source["frameId"], "usb_cam_optical_frame")
        self.camera.json("POST", SOURCE + "/start", {})
        time.sleep(2.2)
        packets = self.rtp.snapshot()
        units = access_units(packets)
        self.assertGreaterEqual(len(units), 7)
        self.assertLessEqual(len(units), 14)
        self.assertTrue(is_idr(units[0]), units[0])
        for unit in units[1:]:
            self.assertEqual(unit, [1], "every later picture is a single P slice")
        total = sum(len(packet) for _, packet in packets)
        self.assertLess(total, 40000)
        self.camera.json("POST", SOURCE + "/stop", {})

    def test_the_capture_is_the_latched_jpeg(self):
        latched = rospy.wait_for_message(
            "/still_media/image_raw/compressed", CompressedImage, timeout=20
        )
        status, headers, data = self.camera.request(
            "POST", SOURCE + "/capture", {"snapshotId": "s", "includeRgb": False, "requireFresh": True}
        )
        self.assertEqual(status, 200)
        boundary = ("--" + headers["Content-Type"].split("boundary=")[1]).encode()
        pieces = data.split(boundary)
        jpeg = pieces[2].split(b"\r\n\r\n", 1)[1].rsplit(b"\r\n", 1)[0]
        self.assertEqual(jpeg, bytes(latched.data))
        metadata = json.loads(pieces[1].split(b"\r\n\r\n", 1)[1].rsplit(b"\r\n", 1)[0])
        self.assertEqual(metadata["calibrationState"], "available")
        self.assertEqual(metadata["cameraMatrix"][0], 50.0)
        self.assertEqual(metadata["frameId"], "usb_cam_optical_frame")


if __name__ == "__main__":
    import sys

    cases = {"synthetic": SyntheticCameraMediaTest, "still": StillCameraMediaTest}
    name = next(argument for argument in sys.argv[1:] if argument in cases)
    rospy.init_node("media_camera_contract_test_" + name)
    rostest.rosrun("xgc2_camera_driver", "media_camera_contract_" + name, cases[name])
