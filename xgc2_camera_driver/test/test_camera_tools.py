#!/usr/bin/env python3

import argparse
import csv
import hashlib
import importlib.machinery
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock


PACKAGE_DIRECTORY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_script(module_name, filename):
    path = os.path.join(PACKAGE_DIRECTORY, "scripts", filename)
    loader = importlib.machinery.SourceFileLoader(module_name, path)
    spec = importlib.util.spec_from_loader(module_name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


bag_export = load_script("xgc_camera_bag_export_test", "xgc_camera_bag_export")
alignment = load_script("xgc_camera_alignment_test", "xgc_camera_alignment")
apply_alignment = load_script(
    "xgc_camera_apply_alignment_test", "xgc_camera_apply_alignment"
)


class FakeTime:
    __slots__ = ("secs", "nsecs")

    def __init__(self, secs=0, nsecs=0):
        self.secs = secs
        self.nsecs = nsecs

    def to_nsec(self):
        return self.secs * 1_000_000_000 + self.nsecs


class FakeHeader:
    __slots__ = ("stamp",)
    _slot_types = ("time",)

    def __init__(self, stamp):
        self.stamp = stamp


class FakeSceneMessage:
    __slots__ = ("header", "label")
    _slot_types = ("std_msgs/Header", "string")

    def __init__(self, stamp, label="scene"):
        self.header = FakeHeader(stamp)
        self.label = label


class FakeBagEntry:
    def __init__(
        self, topic, message, timestamp, connection_header=None
    ):
        self.topic = topic
        self.message = message
        self.timestamp = timestamp
        self.connection_header = connection_header

    def __iter__(self):
        return iter(
            (
                self.topic,
                self.message,
                self.timestamp,
                self.connection_header,
            )
        )


class FakeRosbagModule(types.ModuleType):
    def __init__(self, records):
        super().__init__("rosbag")
        self.records = records
        self.writes = []
        module = self

        class Bag:
            def __init__(self, path, mode):
                self.path = os.path.realpath(path)
                self.mode = mode

            def __enter__(self):
                return self

            def __exit__(self, exception_type, exception, traceback):
                if self.mode == "w" and exception is None:
                    with open(self.path, "wb") as stream:
                        stream.write(
                            ("derived:" + str(len(module.writes))).encode()
                        )
                return False

            def read_messages(
                self,
                topics=None,
                return_connection_header=False,
            ):
                selected = set(topics or [])
                for entry in module.records.get(self.path, []):
                    topic = (
                        entry.topic
                        if hasattr(entry, "topic")
                        else entry[0]
                    )
                    if selected and topic not in selected:
                        continue
                    if return_connection_header:
                        if hasattr(entry, "topic"):
                            yield entry
                        else:
                            yield FakeBagEntry(
                                entry[0],
                                entry[1],
                                entry[2],
                                entry[3] if len(entry) > 3 else None,
                            )
                    else:
                        if hasattr(entry, "topic"):
                            yield entry.topic, entry.message, entry.timestamp
                        else:
                            yield entry[:3]

            def write(
                self,
                topic,
                message,
                t,
                connection_header=None,
            ):
                module.writes.append(
                    FakeBagEntry(
                        topic, message, t, connection_header
                    )
                )

        self.Bag = Bag


def message(**values):
    return types.SimpleNamespace(**values)


def sha256(path):
    with open(path, "rb") as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def provenance_for(paths):
    return {
        "bags": [
            {
                # Deliberately not the live path: provenance matching is based
                # on immutable content and size so a session may be relocated.
                "path": "/recorded/original/" + os.path.basename(path),
                "sizeBytes": os.path.getsize(path),
                "sha256": sha256(path),
            }
            for path in paths
        ]
    }


class AlignmentModelTest(unittest.TestCase):
    def test_affine_model_retains_exact_nanosecond_scale(self):
        image0 = 1_700_000_000_000_000_001
        scene0 = 1_700_000_000_100_000_003
        image1 = image0 + 3_000_000_007
        scene1 = scene0 + 3_000_003_007
        model = alignment._alignment_model(
            None, [(image0, scene0), (image1, scene1)]
        )

        self.assertEqual(model["scaleNumerator"], 3_000_003_007)
        self.assertEqual(model["scaleDenominator"], 3_000_000_007)
        self.assertEqual(
            apply_alignment._scene_to_image_time(model, scene0), image0
        )
        self.assertEqual(
            apply_alignment._scene_to_image_time(model, scene1), image1
        )

    def test_delay_direction_and_piecewise_bounds_are_explicit(self):
        model = alignment._alignment_model(120.0, [])
        self.assertEqual(model["offsetNs"], -120_000_000)
        self.assertEqual(
            apply_alignment._scene_to_image_time(model, 1_000_000_000),
            1_120_000_000,
        )
        piecewise = alignment._alignment_model(
            None, [(100, 90), (200, 190), (300, 290)]
        )
        with self.assertRaisesRegex(RuntimeError, "outside"):
            apply_alignment._scene_to_image_time(piecewise, 50)
        self.assertEqual(
            apply_alignment._scene_to_image_time(
                piecewise, 50, allow_extrapolation=True
            ),
            60,
        )

    def test_split_bags_use_natural_order(self):
        with tempfile.TemporaryDirectory() as directory:
            split10 = os.path.join(directory, "session_10.bag")
            split2 = os.path.join(directory, "session_2.bag")
            for path in (split10, split2):
                with open(path, "wb") as stream:
                    stream.write(path.encode())
            self.assertEqual(
                apply_alignment._resolve_bags(
                    [os.path.join(directory, "session_*.bag")]
                ),
                [split2, split10],
            )


class ExportToolTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = self.temporary.name
        self.bag1 = os.path.join(self.directory, "session_1.bag")
        self.bag2 = os.path.join(self.directory, "session_2.bag")
        with open(self.bag1, "wb") as stream:
            stream.write(b"bag-one")
        with open(self.bag2, "wb") as stream:
            stream.write(b"bag-two")
        self.source_hashes = {
            self.bag1: sha256(self.bag1),
            self.bag2: sha256(self.bag2),
        }
        self.source_stats = {
            path: os.stat(path) for path in self.source_hashes
        }

    def tearDown(self):
        self.temporary.cleanup()

    def _arguments(self, output):
        return argparse.Namespace(
            bags=[self.bag1, self.bag2],
            output=output,
            video_topic="/camera/video",
            timing_topic="/camera/frame_timing",
            stream_info_topic="/camera/stream_info",
            fps=None,
            ffmpeg="/fake/ffmpeg",
            overwrite=False,
        )

    @staticmethod
    def _fake_ffmpeg(command, check):
        del check
        source = command[command.index("-i") + 1]
        shutil.copyfile(source, command[-1])
        return subprocess.CompletedProcess(command, 0)

    def test_cross_split_duplicate_timestamps_match_by_signature(self):
        timestamp = FakeTime(10, 25)
        bootstrap = bytes.fromhex(
            "000000016742001f00000168ce06000001658884"
        )
        p_frame = bytes.fromhex("000001419a22")
        timing1 = message(
            source_time=timestamp,
            source_time_valid=True,
            epoch=7,
            frame_sequence=10,
            source_sequence=100,
            native_source_time_ns=9,
            discontinuity=1,
            dropped_frames_before=0,
            encoded_size_bytes=len(bootstrap),
            keyframe=True,
        )
        timing2 = message(
            source_time=timestamp,
            source_time_valid=True,
            epoch=7,
            frame_sequence=11,
            source_sequence=101,
            native_source_time_ns=10,
            discontinuity=0,
            dropped_frames_before=0,
            encoded_size_bytes=len(p_frame),
            keyframe=False,
        )
        records = {
            self.bag1: [
                (
                    "/camera/stream_info",
                    message(nominal_frame_rate=30.0),
                    FakeTime(1),
                ),
                ("/camera/frame_timing", timing2, FakeTime(11)),
                ("/camera/frame_timing", timing1, FakeTime(12)),
            ],
            self.bag2: [
                (
                    "/camera/video",
                    message(
                        format="h264", data=bootstrap, timestamp=timestamp
                    ),
                    FakeTime(13),
                ),
                (
                    "/camera/video",
                    message(
                        format="h264", data=p_frame, timestamp=timestamp
                    ),
                    FakeTime(14),
                ),
            ],
        }
        fake_rosbag = FakeRosbagModule(records)
        output = os.path.join(self.directory, "camera.mkv")
        with mock.patch.dict(sys.modules, {"rosbag": fake_rosbag}), mock.patch.object(
            bag_export.subprocess, "run", self._fake_ffmpeg
        ):
            result = bag_export.export(self._arguments(output))

        self.assertEqual(result["frames"], 2)
        with open(output + ".timing.csv", newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(
            [row["frame_sequence"] for row in rows], ["10", "11"]
        )
        with open(output + ".manifest.json", encoding="utf-8") as stream:
            manifest = json.load(stream)
        self.assertEqual(manifest["matchedTimingFrames"], 2)
        self.assertEqual(manifest["unmatchedVideoFrames"], 0)
        self.assertEqual(
            manifest["derivatives"]["video"]["sha256"], sha256(output)
        )
        for path in self.source_hashes:
            self.assertEqual(sha256(path), self.source_hashes[path])
            after = os.stat(path)
            self.assertEqual(after.st_mtime_ns, self.source_stats[path].st_mtime_ns)

    def test_failed_mux_leaves_no_derivative_or_staging_files(self):
        timestamp = FakeTime(10)
        records = {
            self.bag1: [
                (
                    "/camera/stream_info",
                    message(nominal_frame_rate=30.0),
                    FakeTime(1),
                )
            ],
            self.bag2: [
                (
                    "/camera/video",
                    message(
                        format="h264",
                        data=bytes.fromhex(
                            "000000016742001f00000168ce06000001658884"
                        ),
                        timestamp=timestamp,
                    ),
                    FakeTime(2),
                )
            ],
        }
        fake_rosbag = FakeRosbagModule(records)
        output = os.path.join(self.directory, "failed.mkv")
        with mock.patch.dict(sys.modules, {"rosbag": fake_rosbag}), mock.patch.object(
            bag_export.subprocess,
            "run",
            side_effect=subprocess.CalledProcessError(1, "ffmpeg"),
        ):
            with self.assertRaises(subprocess.CalledProcessError):
                bag_export.export(self._arguments(output))
        self.assertFalse(os.path.exists(output))
        self.assertFalse(os.path.exists(output + ".timing.csv"))
        self.assertFalse(os.path.exists(output + ".manifest.json"))
        self.assertFalse(
            any(name.endswith(".part") for name in os.listdir(self.directory))
        )


class AlignmentArtifactTest(unittest.TestCase):
    def test_creation_is_atomic_and_does_not_touch_source(self):
        with tempfile.TemporaryDirectory() as directory:
            bag = os.path.join(directory, "source.bag")
            with open(bag, "wb") as stream:
                stream.write(b"immutable")
            before = (sha256(bag), os.stat(bag).st_mtime_ns)
            timing = message(
                source_time=FakeTime(2, 10),
                source_time_valid=True,
                native_source_time_ns=123,
                source_to_ros_offset_ns=5,
                mapping_uncertainty_ns=7,
                epoch=3,
                frame_sequence=4,
                stream_id="usb",
            )
            fake_rosbag = FakeRosbagModule(
                {
                    bag: [
                        ("/camera/frame_timing", timing, FakeTime(3))
                    ]
                }
            )
            output = os.path.join(directory, "alignment.yaml")
            args = argparse.Namespace(
                bags=[bag],
                timing_topic="/camera/frame_timing",
                output=output,
                image_delay_ms=120.0,
                anchor=[],
                method="manual",
                note="test",
                overwrite=False,
            )
            with mock.patch.dict(sys.modules, {"rosbag": fake_rosbag}):
                artifact = alignment.create_alignment(args)
            self.assertEqual(
                artifact["model"]["offsetNs"], -120_000_000
            )
            self.assertEqual(before, (sha256(bag), os.stat(bag).st_mtime_ns))
            self.assertFalse(
                any(name.endswith(".part") for name in os.listdir(directory))
            )


class ApplyAlignmentTest(unittest.TestCase):
    def test_writes_only_explicit_aligned_scene_topic(self):
        with tempfile.TemporaryDirectory() as directory:
            bag = os.path.join(directory, "source.bag")
            with open(bag, "wb") as stream:
                stream.write(b"source")
            source_message = FakeSceneMessage(FakeTime(1))
            fake_rosbag = FakeRosbagModule(
                {
                    bag: [
                        FakeBagEntry(
                            "/tf",
                            source_message,
                            FakeTime(1),
                            {"topic": "/tf", "latching": "0"},
                        )
                    ]
                }
            )
            alignment_path = os.path.join(directory, "alignment.yaml")
            with open(alignment_path, "w", encoding="utf-8") as stream:
                json.dump(
                    {
                        "schema": "xgc.camera.alignment/v1",
                        "model": {
                            "kind": "constant-offset",
                            "offsetNs": -120_000_000,
                        },
                        "provenance": provenance_for([bag]),
                    },
                    stream,
                )
            output = os.path.join(directory, "aligned-scene.bag")
            args = argparse.Namespace(
                bags=[bag],
                alignment=alignment_path,
                output=output,
                topic_map=[("/tf", "/xgc/aligned/tf")],
                allow_extrapolation=False,
                allow_large_topics=False,
                overwrite=False,
            )
            before = (sha256(bag), os.stat(bag).st_mtime_ns)
            with mock.patch.dict(sys.modules, {"rosbag": fake_rosbag}):
                manifest = apply_alignment.apply_alignment(args)

            self.assertTrue(os.path.isfile(output))
            self.assertTrue(os.path.isfile(output + ".manifest.json"))
            self.assertEqual(len(fake_rosbag.writes), 1)
            written = fake_rosbag.writes[0]
            self.assertEqual(written.topic, "/xgc/aligned/tf")
            self.assertEqual(written.timestamp.to_nsec(), 1_120_000_000)
            self.assertEqual(
                written.message.header.stamp.to_nsec(), 1_120_000_000
            )
            self.assertEqual(source_message.header.stamp.to_nsec(), 1_000_000_000)
            self.assertEqual(before, (sha256(bag), os.stat(bag).st_mtime_ns))
            self.assertEqual(manifest["statistics"]["messagesCopied"], 1)
            self.assertFalse(manifest["allowLargeTopics"])

    def test_all_model_kinds_materialize_with_verifiable_hashes(self):
        models = {
            "constant": alignment._alignment_model(100.0, []),
            "affine": alignment._alignment_model(
                None,
                [
                    (1_000_000_000, 900_000_000),
                    (2_100_000_000, 1_900_000_000),
                ],
            ),
            "piecewise": alignment._alignment_model(
                None,
                [
                    (1_000_000_000, 900_000_000),
                    (2_100_000_000, 1_900_000_000),
                    (3_300_000_000, 2_900_000_000),
                ],
            ),
        }
        expected = {
            "constant": 1_500_000_000,
            "affine": 1_550_000_000,
            "piecewise": 1_550_000_000,
        }
        for name, model in models.items():
            with self.subTest(model=name), tempfile.TemporaryDirectory() as directory:
                bag = os.path.join(directory, "source.bag")
                with open(bag, "wb") as stream:
                    stream.write(("source-" + name).encode())
                fake_rosbag = FakeRosbagModule(
                    {
                        bag: [
                            FakeBagEntry(
                                "/scene",
                                FakeSceneMessage(
                                    FakeTime(1, 400_000_000)
                                ),
                                FakeTime(1, 400_000_000),
                            )
                        ]
                    }
                )
                alignment_path = os.path.join(
                    directory, "alignment.yaml"
                )
                with open(
                    alignment_path, "w", encoding="utf-8"
                ) as stream:
                    json.dump(
                        {
                            "schema": "xgc.camera.alignment/v1",
                            "model": model,
                            "provenance": provenance_for([bag]),
                        },
                        stream,
                    )
                output = os.path.join(
                    directory, "aligned-" + name + ".bag"
                )
                arguments = argparse.Namespace(
                    bags=[bag],
                    alignment=alignment_path,
                    output=output,
                    topic_map=[("/scene", "/aligned/scene")],
                    allow_extrapolation=False,
                    allow_large_topics=False,
                    overwrite=False,
                )
                source_hash = sha256(bag)
                with mock.patch.dict(
                    sys.modules, {"rosbag": fake_rosbag}
                ):
                    manifest = apply_alignment.apply_alignment(
                        arguments
                    )

                self.assertEqual(
                    fake_rosbag.writes[0].timestamp.to_nsec(),
                    expected[name],
                )
                self.assertEqual(
                    fake_rosbag.writes[0].message.header.stamp.to_nsec(),
                    expected[name],
                )
                self.assertEqual(sha256(bag), source_hash)
                self.assertEqual(
                    manifest["sourceBags"][0]["sha256"], source_hash
                )
                self.assertEqual(
                    manifest["alignment"]["sha256"],
                    sha256(alignment_path),
                )
                self.assertEqual(
                    manifest["derivedBag"]["sha256"], sha256(output)
                )
                self.assertTrue(
                    manifest["alignmentSourceSetVerified"]
                )

    def test_mismatched_alignment_provenance_fails_before_output(self):
        with tempfile.TemporaryDirectory() as directory:
            bag = os.path.join(directory, "source.bag")
            with open(bag, "wb") as stream:
                stream.write(b"right-session")
            alignment_path = os.path.join(directory, "alignment.yaml")
            with open(alignment_path, "w", encoding="utf-8") as stream:
                json.dump(
                    {
                        "schema": "xgc.camera.alignment/v1",
                        "model": {
                            "kind": "constant-offset",
                            "offsetNs": -10,
                        },
                        "provenance": {
                            "bags": [
                                {
                                    "path": "/another/session.bag",
                                    "sizeBytes": len(b"wrong-session"),
                                    "sha256": hashlib.sha256(
                                        b"wrong-session"
                                    ).hexdigest(),
                                }
                            ]
                        },
                    },
                    stream,
                )
            fake_rosbag = FakeRosbagModule(
                {
                    bag: [
                        FakeBagEntry(
                            "/scene",
                            FakeSceneMessage(FakeTime(1)),
                            FakeTime(1),
                        )
                    ]
                }
            )
            output = os.path.join(directory, "must-not-exist.bag")
            arguments = argparse.Namespace(
                bags=[bag],
                alignment=alignment_path,
                output=output,
                topic_map=[("/scene", "/aligned/scene")],
                allow_extrapolation=False,
                allow_large_topics=False,
                overwrite=False,
            )
            with mock.patch.dict(sys.modules, {"rosbag": fake_rosbag}):
                with self.assertRaisesRegex(
                    RuntimeError, "complete input bag set"
                ):
                    apply_alignment.apply_alignment(arguments)
            self.assertFalse(os.path.exists(output))
            self.assertFalse(
                os.path.exists(output + ".manifest.json")
            )
            self.assertEqual(fake_rosbag.writes, [])

    def test_large_camera_topics_require_second_explicit_opt_in(self):
        with self.assertRaisesRegex(RuntimeError, "blocked"):
            apply_alignment._validate_topic_maps(
                [("/camera/video", "/aligned/camera/video")],
                allow_large_topics=False,
            )


if __name__ == "__main__":
    unittest.main()
