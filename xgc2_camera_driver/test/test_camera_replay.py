#!/usr/bin/env python3
"""Pure-python contract tests for the scene-replay camera provider.

cv2 and rospy are faked or never imported; these tests run without ROS.
"""

import importlib.machinery
import importlib.util
import math
import os
import subprocess
import sys
import tempfile
import types
import unittest


PACKAGE_DIRECTORY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT_PATH = os.path.join(PACKAGE_DIRECTORY, "scripts", "xgc_camera_replay")


def load_script(module_name, filename):
    path = os.path.join(PACKAGE_DIRECTORY, "scripts", filename)
    loader = importlib.machinery.SourceFileLoader(module_name, path)
    spec = importlib.util.spec_from_loader(module_name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


replay = load_script("xgc_camera_replay_test", "xgc_camera_replay")


def write_yaml(directory, name, document):
    import yaml

    path = os.path.join(directory, name)
    with open(path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(document, stream, sort_keys=False)
    return path


def intrinsic_document():
    return {
        "schema": "xgc2.camera.intrinsic.v1",
        "image_width": 3840,
        "image_height": 2160,
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [2000.0, 0.0, 1920.5, 0.0, 2001.0, 1080.5, 0.0, 0.0, 1.0],
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": 5,
            "data": [0.1, -0.05, 0.001, 0.002, 0.0],
        },
    }


def extrinsic_document():
    return {
        "schema": "xgc2.camera.extrinsic.v1",
        "frame_convention": "parent_T_camera_optical",
        "parent_frame": "world",
        "child_frame": "usb_cam_optical_frame",
        "translation": {"x": 1.0, "y": 2.0, "z": 3.0},
        "quaternion_xyzw": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }


class FakeCapture(object):
    def __init__(self, frames, opened=True):
        self._frames = list(frames)
        self._opened = opened
        self.released = False

    def isOpened(self):
        return self._opened

    def read(self):
        if not self._frames:
            return False, None
        return True, self._frames.pop(0)

    def release(self):
        self.released = True


class FakeCv2(types.ModuleType):
    def __init__(self, still_frame=None, captures=None):
        super().__init__("cv2")
        self._still_frame = still_frame
        self._captures = list(captures or [])
        self.created_captures = []
        self.IMWRITE_JPEG_QUALITY = 1

    def imread(self, path):
        return self._still_frame

    def VideoCapture(self, path):
        capture = self._captures.pop(0)
        self.created_captures.append(capture)
        return capture


class ExtrinsicMathTest(unittest.TestCase):
    def assertQuaternionAlmostEqual(self, actual, expected):
        # q and -q encode the same rotation; align signs before comparing.
        if sum(a * e for a, e in zip(actual, expected)) < 0:
            expected = tuple(-value for value in expected)
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, e, places=12)

    def test_identity_parent_to_optical_recovers_inverse_standard_rotation(self):
        translation, quaternion = replay.derive_parent_to_link(
            (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0)
        )
        self.assertEqual(translation, (1.0, 2.0, 3.0))
        # parent->link must be the inverse of the standard link->optical rotation.
        self.assertQuaternionAlmostEqual(quaternion, (0.5, -0.5, 0.5, 0.5))

    def test_round_trip_with_rotation_is_identity(self):
        half = math.sqrt(0.5)
        parent_to_optical = (0.0, 0.0, half, half)  # 90 deg about z
        translation = (-0.25, 0.5, 1.75)
        link_translation, parent_to_link = replay.derive_parent_to_link(
            translation, parent_to_optical
        )
        self.assertEqual(link_translation, translation)
        # Recompose: T_parent_link * T_link_optical must equal T_parent_optical.
        recomposed = replay.quaternion_multiply(
            parent_to_link, replay.STANDARD_LINK_TO_OPTICAL_QUATERNION
        )
        self.assertQuaternionAlmostEqual(recomposed, parent_to_optical)

    def test_custom_link_to_optical_quaternion_is_respected(self):
        half = math.sqrt(0.5)
        custom = (0.0, half, 0.0, half)  # 90 deg about y
        translation, quaternion = replay.derive_parent_to_link(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), custom
        )
        self.assertEqual(translation, (0.0, 0.0, 0.0))
        self.assertQuaternionAlmostEqual(quaternion, (0.0, -half, 0.0, half))

    def test_derive_link_frame(self):
        self.assertEqual(
            replay.derive_link_frame("usb_cam_optical_frame"), "usb_cam_link"
        )
        with self.assertRaises(replay.ReplayError):
            replay.derive_link_frame("usb_cam")
        with self.assertRaises(replay.ReplayError):
            replay.derive_link_frame("_optical_frame")

    def test_static_chain_matches_physical_usb_contract(self):
        half = math.sqrt(0.5)
        parent_to_optical = (0.0, 0.0, half, half)
        translation = (2.798, 7.207, 1.906)
        parent_to_link, link_to_optical = replay.compose_static_camera_transforms(
            {
                "parent_frame": "world",
                "child_frame": "usb_cam_optical_frame",
                "translation": translation,
                "quaternion": parent_to_optical,
            }
        )
        self.assertEqual(parent_to_link["parent_frame"], "world")
        self.assertEqual(parent_to_link["child_frame"], "usb_cam_link")
        self.assertEqual(parent_to_link["translation"], translation)
        self.assertEqual(link_to_optical["parent_frame"], "usb_cam_link")
        self.assertEqual(link_to_optical["child_frame"], "usb_cam_optical_frame")
        self.assertEqual(link_to_optical["translation"], (0.0, 0.0, 0.0))
        self.assertEqual(
            link_to_optical["quaternion"], replay.STANDARD_LINK_TO_OPTICAL_QUATERNION
        )
        recomposed = replay.quaternion_multiply(
            parent_to_link["quaternion"], link_to_optical["quaternion"]
        )
        self.assertQuaternionAlmostEqual(recomposed, parent_to_optical)


class AssetYamlTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="xgc-camera-replay-test.")
        self.addCleanup(self._tmp.cleanup)
        self.directory = self._tmp.name

    def test_intrinsic_maps_to_camera_info_fields(self):
        path = write_yaml(self.directory, "camera-info.yaml", intrinsic_document())
        intrinsic = replay.load_intrinsic(path)
        fields = replay.build_camera_info_fields(intrinsic)
        self.assertEqual(fields["width"], 3840)
        self.assertEqual(fields["height"], 2160)
        self.assertEqual(fields["distortion_model"], "plumb_bob")
        self.assertEqual(fields["D"], [0.1, -0.05, 0.001, 0.002, 0.0])
        self.assertEqual(
            fields["K"],
            [2000.0, 0.0, 1920.5, 0.0, 2001.0, 1080.5, 0.0, 0.0, 1.0],
        )
        self.assertEqual(
            fields["R"], [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )
        self.assertEqual(
            fields["P"],
            [2000.0, 0.0, 1920.5, 0.0, 0.0, 2001.0, 1080.5, 0.0, 0.0, 0.0, 1.0, 0.0],
        )

    def test_intrinsic_rejects_bad_schema(self):
        document = intrinsic_document()
        document["schema"] = "xgc2.camera.intrinsic.v0"
        path = write_yaml(self.directory, "camera-info.yaml", document)
        with self.assertRaises(replay.ReplayError):
            replay.load_intrinsic(path)

    def test_intrinsic_rejects_bad_matrix_shape(self):
        document = intrinsic_document()
        document["camera_matrix"]["rows"] = 2
        path = write_yaml(self.directory, "camera-info.yaml", document)
        with self.assertRaises(replay.ReplayError):
            replay.load_intrinsic(path)

    def test_intrinsic_rejects_coefficient_length_mismatch(self):
        document = intrinsic_document()
        document["distortion_coefficients"]["cols"] = 4
        path = write_yaml(self.directory, "camera-info.yaml", document)
        with self.assertRaises(replay.ReplayError):
            replay.load_intrinsic(path)

    def test_missing_file_is_a_replay_error(self):
        with self.assertRaises(replay.ReplayError):
            replay.load_intrinsic(os.path.join(self.directory, "absent.yaml"))

    def test_extrinsic_loads_normalized_quaternion(self):
        document = extrinsic_document()
        document["quaternion_xyzw"] = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 2.0}
        path = write_yaml(self.directory, "extrinsic.yaml", document)
        extrinsic = replay.load_extrinsic(path)
        self.assertEqual(extrinsic["parent_frame"], "world")
        self.assertEqual(extrinsic["child_frame"], "usb_cam_optical_frame")
        self.assertEqual(extrinsic["translation"], (1.0, 2.0, 3.0))
        self.assertEqual(extrinsic["quaternion"], (0.0, 0.0, 0.0, 1.0))

    def test_extrinsic_rejects_wrong_frame_convention(self):
        document = extrinsic_document()
        document["frame_convention"] = "camera_T_parent"
        path = write_yaml(self.directory, "extrinsic.yaml", document)
        with self.assertRaises(replay.ReplayError):
            replay.load_extrinsic(path)

    def test_extrinsic_rejects_zero_quaternion(self):
        document = extrinsic_document()
        document["quaternion_xyzw"] = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}
        path = write_yaml(self.directory, "extrinsic.yaml", document)
        with self.assertRaises(replay.ReplayError):
            replay.load_extrinsic(path)

    def test_extrinsic_rejects_bad_schema(self):
        document = extrinsic_document()
        document["schema"] = "xgc2.camera.extrinsic.v9"
        path = write_yaml(self.directory, "extrinsic.yaml", document)
        with self.assertRaises(replay.ReplayError):
            replay.load_extrinsic(path)


class MediaSourceTest(unittest.TestCase):
    def test_still_frame_reads_forever(self):
        frame = object()
        cv2 = FakeCv2(still_frame=frame)
        source = replay.open_media_source(cv2, "/asset/frame.png", loop=True)
        self.assertIs(source.read(), frame)
        self.assertIs(source.read(), frame)

    def test_still_frame_undecodable_is_rejected(self):
        cv2 = FakeCv2(still_frame=None)
        with self.assertRaises(replay.ReplayError):
            replay.open_media_source(cv2, "/asset/frame.png", loop=True)

    def test_video_reopens_at_end_when_looping(self):
        first = FakeCapture(frames=["f1", "f2"])
        second = FakeCapture(frames=["f3"])
        cv2 = FakeCv2(captures=[first, second])
        source = replay.open_media_source(cv2, "/asset/media.mp4", loop=True)
        self.assertEqual(source.read(), "f1")
        self.assertEqual(source.read(), "f2")
        self.assertEqual(source.read(), "f3")
        self.assertTrue(first.released)
        self.assertIs(cv2.created_captures[1], second)

    def test_video_no_loop_returns_none_at_end(self):
        capture = FakeCapture(frames=["f1"])
        cv2 = FakeCv2(captures=[capture])
        source = replay.open_media_source(cv2, "/asset/media.mp4", loop=False)
        self.assertEqual(source.read(), "f1")
        self.assertIsNone(source.read())

    def test_video_that_never_decodes_is_rejected(self):
        cv2 = FakeCv2(captures=[FakeCapture(frames=[]), FakeCapture(frames=[])])
        source = replay.open_media_source(cv2, "/asset/media.mp4", loop=True)
        with self.assertRaises(replay.ReplayError):
            source.read()

    def test_unopenable_video_is_rejected(self):
        cv2 = FakeCv2(captures=[FakeCapture(frames=[], opened=False)])
        with self.assertRaises(replay.ReplayError):
            replay.open_media_source(cv2, "/asset/media.mp4", loop=True)

    def test_unsupported_extension_is_rejected(self):
        with self.assertRaises(replay.ReplayError):
            replay.open_media_source(FakeCv2(), "/asset/media.mkv", loop=True)


class ArgumentValidationTest(unittest.TestCase):
    def parse(self, extra):
        return replay.build_parser().parse_args(
            [
                "--media-file", "media.mp4",
                "--camera-info-file", "camera-info.yaml",
                "--extrinsic-file", "extrinsic.yaml",
            ]
            + extra
        )

    def test_defaults(self):
        args = self.parse([])
        self.assertEqual(args.rate, 30.0)
        self.assertTrue(args.loop)
        self.assertEqual(args.jpeg_quality, 90)
        self.assertEqual(
            args.link_to_optical_quaternion, (-0.5, 0.5, -0.5, 0.5)
        )
        self.assertEqual(args.node_name, "xgc_camera_replay")

    def test_loop_value_and_no_loop_flag(self):
        self.assertFalse(self.parse(["--loop", "false"]).loop)
        self.assertTrue(self.parse(["--loop", "true"]).loop)
        self.assertTrue(self.parse(["--loop"]).loop)
        self.assertFalse(self.parse(["--no-loop"]).loop)

    def test_bad_rate_rejected(self):
        for value in ("0", "-3", "abc", "nan"):
            with self.assertRaises(SystemExit):
                self.parse(["--rate", value])

    def test_bad_jpeg_quality_rejected(self):
        for value in ("0", "101", "1.5", "x"):
            with self.assertRaises(SystemExit):
                self.parse(["--jpeg-quality", value])

    def test_bad_quaternion_rejected(self):
        for value in ("1,2,3", "0,0,0,0", "a,b,c,d"):
            with self.assertRaises(SystemExit):
                self.parse(["--link-to-optical-quaternion", value])

    def test_missing_required_files_rejected(self):
        with self.assertRaises(SystemExit):
            replay.build_parser().parse_args(["--media-file", "media.mp4"])

    def test_ros_remap_args_are_split_out(self):
        ros_args, node_args = replay.split_ros_args(
            [
                "--media-file", "media.mp4",
                "__name:=camera_replay",
                "__ns:=/usb_cam",
                "image_raw/compressed:=/usb_cam/image_raw/compressed",
                "--rate", "15",
            ]
        )
        self.assertEqual(node_args, ["--media-file", "media.mp4", "--rate", "15"])
        self.assertEqual(
            ros_args,
            [
                "__name:=camera_replay",
                "__ns:=/usb_cam",
                "image_raw/compressed:=/usb_cam/image_raw/compressed",
            ],
        )

    def test_main_rejects_missing_media_file_without_ros(self):
        info_path = write_yaml(
            tempfile.mkdtemp(prefix="xgc-camera-replay-main."), "camera-info.yaml",
            intrinsic_document(),
        )
        directory = os.path.dirname(info_path)
        extrinsic_path = write_yaml(directory, "extrinsic.yaml", extrinsic_document())
        status = replay.main(
            [
                "--media-file", os.path.join(directory, "absent.mp4"),
                "--camera-info-file", info_path,
                "--extrinsic-file", extrinsic_path,
            ]
        )
        self.assertEqual(status, 1)


class ImportContractTest(unittest.TestCase):
    def test_top_level_import_needs_no_ros_or_cv2_or_yaml(self):
        code = (
            "import importlib.machinery, importlib.util, sys\n"
            "loader = importlib.machinery.SourceFileLoader('m', sys.argv[1])\n"
            "spec = importlib.util.spec_from_loader('m', loader)\n"
            "module = importlib.util.module_from_spec(spec)\n"
            "loader.exec_module(module)\n"
            "forbidden = {'rospy', 'rosbag', 'cv2', 'sensor_msgs', 'geometry_msgs',"
            " 'tf2_ros', 'yaml'}\n"
            "loaded = {name.split('.')[0] for name in sys.modules}\n"
            "sys.exit(1 if loaded & forbidden else 0)\n"
        )
        result = subprocess.run(
            [sys.executable, "-c", code, SCRIPT_PATH],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
