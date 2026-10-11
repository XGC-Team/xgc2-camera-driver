#!/usr/bin/env python3
"""The shipped process definitions describe the node that is actually installed.

Static checks catch typos between a definition and the node's parameters; the
rendered command lines then start the real node, the way the supervisor would.
"""

import json
import os
import re
import shutil
import signal
import subprocess
import tempfile
import threading
import time
import unittest

import rostest
from xrpc_unix import call

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFINITIONS = os.path.join(PACKAGE, "process-definitions")
DATA = os.path.join(PACKAGE, "test", "data")


def load(name):
    with open(os.path.join(DEFINITIONS, name + ".json"), encoding="utf-8") as handle:
        document = json.load(handle)
    assert document["apiVersion"] == "xgc.execution.process/v1"
    assert len(document["definitions"]) == 1
    return document["definitions"][0]


def render_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float) and value.is_integer():
        return str(int(value))
    return str(value)


def render(template, values):
    return re.sub(r"\$\{(\w+)\}", lambda match: render_value(values[match.group(1)]), template)


def parameter_values(definition, overrides):
    values = {}
    for name, schema in definition["parameters"]["properties"].items():
        if "default" in schema:
            values[name] = schema["default"]
    values.update(overrides)
    missing = [name for name in definition["parameters"].get("required", []) if name not in values]
    assert not missing, missing
    return values


def node_parameter_names():
    """Every private parameter name the node and its media code read."""
    names = set()
    for filename in ("camera_driver_node.cpp", "media_outputs.cpp"):
        with open(os.path.join(PACKAGE, "src", filename), encoding="utf-8") as handle:
            source = handle.read()
        names |= set(re.findall(r'(?:private_nh_|nh|private_nh)\.param(?:<[^>]*>)?\(\s*"(\w+)"', source))
    return names


class Rendered:
    """Starts the node from a rendered definition inside a private runtime directory."""

    def __init__(self, name, overrides):
        self.definition = load(name)
        self.directory = tempfile.mkdtemp(prefix="xgc2_camera_definition_")
        os.chmod(self.directory, 0o700)
        self.socket = os.path.join(self.directory, "camera.sock")
        values = parameter_values(
            self.definition,
            dict(overrides, controlSocket=self.socket, rosMasterUri=os.environ["ROS_MASTER_URI"]),
        )
        arguments = [render(argument, values) for argument in self.definition["command"]["args"]]
        environment = dict(os.environ)
        for key, template in self.definition["command"]["env"].items():
            if key.startswith("ROS_") and key in ("ROS_MASTER_URI", "ROS_IP", "ROS_HOSTNAME"):
                rendered = render(template, values)
                if rendered:
                    environment[key] = rendered
        self.process = subprocess.Popen(
            ["rosrun", "xgc2_camera_driver", "xgc2_camera_driver_node"] + arguments,
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        self.log = []
        threading.Thread(target=self._drain, daemon=True).start()
        self.instance = None

    def _drain(self):
        for line in self.process.stdout:
            self.log.append(line.decode("utf-8", "replace"))

    def call(self, target, timeout_ms=8000):
        status, _, data = call(self.socket, "GET", target, instance=self.instance, timeout_ms=timeout_ms)
        return status, json.loads(data)

    def wait_for_service(self, timeout=20.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.assertAlive()
            if os.path.exists(self.socket):
                try:
                    status, _, data = call(self.socket, "GET", "/v1/describe")
                    if status == 200:
                        self.instance = json.loads(data)["instance_id"]
                        return json.loads(data)
                except OSError:
                    pass
            time.sleep(0.05)
        raise AssertionError("the service did not appear:\n" + "".join(self.log[-20:]))

    def assertAlive(self):
        if self.process.poll() is not None:
            raise AssertionError("the node exited early:\n" + "".join(self.log[-30:]))

    def stop(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
        shutil.rmtree(self.directory, ignore_errors=True)


class ProcessDefinitionTest(unittest.TestCase):
    NAMES = ("xgc2-camera-v4l2", "xgc2-camera-still")

    def test_shape_and_identity(self):
        for name in self.NAMES:
            definition = load(name)
            self.assertEqual(definition["id"], name)
            self.assertEqual(definition["drivers"], ["host"])
            self.assertEqual(definition["readiness"]["kind"], "describe")
            self.assertEqual(definition["restart"]["mode"], "never")
            self.assertEqual(definition["command"]["executable"],
                             "/opt/ros/noetic/lib/xgc2_camera_driver/xgc2_camera_driver_node")
            self.assertFalse(definition["parameters"]["additionalProperties"])
            self.assertNotIn("beforeStart", definition, "no hidden side effects before the start")

    def test_the_service_is_the_owned_unix_endpoint(self):
        for name in self.NAMES:
            definition = load(name)
            (service,) = definition["services"]
            self.assertEqual(service["service"], "camera-source")
            self.assertEqual(service["api_version"], "v1")
            self.assertEqual(service["profile"], "http.v1")
            self.assertEqual(service["describePath"], "/v1/describe")
            endpoint = definition["parameters"]["properties"][service["endpointParameter"]]
            self.assertEqual(endpoint["ownedEndpoint"], "unix-socket")
            self.assertTrue(endpoint["fixedOnly"])
            self.assertEqual(endpoint["type"], "string")
            self.assertIn("_control_socket:=${%s}" % service["endpointParameter"], definition["command"]["args"])

    def test_every_placeholder_is_declared_and_every_parameter_is_used(self):
        for name in self.NAMES:
            definition = load(name)
            declared = set(definition["parameters"]["properties"])
            used = set()
            texts = definition["command"]["args"] + list(definition["command"]["env"].values())
            for text in texts:
                used |= set(re.findall(r"\$\{(\w+)\}", text))
            self.assertEqual(used - declared, set(), name)
            self.assertEqual(declared - used, set(), name)
            grouped = [item for group in definition["parameters"]["groups"] for item in group["parameters"]]
            self.assertEqual(len(grouped), len(set(grouped)))
            self.assertLessEqual(set(grouped), declared)

    def test_every_node_parameter_the_definition_passes_exists(self):
        known = node_parameter_names()
        self.assertIn("control_socket", known)
        self.assertIn("still_image", known)
        for name in self.NAMES:
            passed = set()
            for argument in load(name)["command"]["args"]:
                match = re.match(r"_(\w+):=", argument)
                if match and not argument.startswith("__"):  # __name and __ns are ROS remappings
                    passed.add(match.group(1))
            self.assertEqual(passed - known, set(), name)

    def test_defaults_satisfy_their_own_constraints(self):
        for name in self.NAMES:
            for parameter, schema in load(name)["parameters"]["properties"].items():
                if "default" not in schema:
                    continue
                default = schema["default"]
                if "enum" in schema:
                    self.assertIn(default, schema["enum"], parameter)
                if "minimum" in schema:
                    self.assertGreaterEqual(default, schema["minimum"], parameter)
                if "maximum" in schema:
                    self.assertLessEqual(default, schema["maximum"], parameter)

    def test_the_still_variant_requires_its_assets(self):
        definition = load("xgc2-camera-still")
        self.assertEqual(
            sorted(definition["parameters"]["required"]),
            ["stillExtrinsicFile", "stillImage", "stillIntrinsicFile"],
        )
        self.assertNotIn("videoDevice", definition["parameters"]["properties"])
        self.assertNotIn("frameId", definition["parameters"]["properties"])

    # The rendered command lines start the real node.
    def test_the_still_definition_starts_a_ready_camera(self):
        rendered = Rendered("xgc2-camera-still", {
            "stillImage": os.path.join(DATA, "still_scene.png"),
            "stillIntrinsicFile": os.path.join(DATA, "still_intrinsic.yaml"),
            "stillExtrinsicFile": os.path.join(DATA, "still_extrinsic.yaml"),
            "cameraNamespace": "/definition_still",
            "rtpPort": 5999,
        })
        try:
            rendered.wait_for_service()
            status, description = rendered.call("/v1/describe?wait_ready_ms=15000")
            self.assertEqual(status, 200)
            self.assertTrue(description["ready"], description)
            self.assertEqual(description["facts"]["source_kind"], "still")
            self.assertEqual(description["facts"]["frame_id"], "usb_cam_optical_frame")
            self.assertEqual(description["facts"]["rtp"]["port"], 5999)
        finally:
            rendered.stop()

    def test_the_v4l2_definition_waits_for_an_absent_camera_without_dying(self):
        rendered = Rendered("xgc2-camera-v4l2", {
            "videoDevice": "/dev/v4l/by-id/xgc2-camera-that-is-not-plugged-in",
            "cameraNamespace": "/definition_v4l2",
            "width": 640,
            "height": 480,
            "framerate": 30.0,
        })
        try:
            description = rendered.wait_for_service()
            self.assertFalse(description["ready"])
            self.assertEqual(description["facts"]["source_kind"], "v4l2")
            started = time.time()
            status, held = rendered.call("/v1/describe?wait_ready_ms=700")
            self.assertEqual(status, 200)
            self.assertFalse(held["ready"])
            self.assertGreaterEqual(time.time() - started, 0.6)
            self.assertIn("reason", held["facts"])
            time.sleep(2.5)
            rendered.assertAlive()
            self.assertEqual(rendered.call("/v1/describe")[1]["instance_id"], rendered.instance,
                             "the service keeps its identity while the camera is absent")
        finally:
            rendered.stop()


if __name__ == "__main__":
    rostest.rosrun("xgc2_camera_driver", "process_definitions", ProcessDefinitionTest)
