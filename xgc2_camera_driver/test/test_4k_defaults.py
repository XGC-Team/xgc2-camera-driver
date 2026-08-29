#!/usr/bin/env python3
"""Lock the station USB camera defaults to the verified native 4K profile."""

from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
DEVICE = "/dev/v4l/by-id/usb-LRCP_imx415_LRCP_imx415_01.00.00-video-index0"


class FourKDefaultsTest(unittest.TestCase):
    def test_yaml_profile_is_native_4k20_mjpeg(self):
        values = {}
        for line in (PACKAGE / "config/usb_cam.yaml").read_text(encoding="utf-8").splitlines():
            match = re.fullmatch(r"([a-z_]+):\s*(.*?)\s*", line)
            if match:
                values[match.group(1)] = match.group(2)

        self.assertEqual(values["video_device"], DEVICE)
        self.assertEqual(values["width"], "3840")
        self.assertEqual(values["height"], "2160")
        self.assertEqual(values["framerate"], "30.0")
        self.assertEqual(values["pixel_format"], "mjpeg")
        self.assertEqual(values["publish_encoded"], "true")
        self.assertEqual(values["unknown_timestamp_clock"], "assume_monotonic")

    def test_launch_defaults_cannot_silently_fall_back(self):
        for relative in ("launch/camera.launch", "launch/usb_camera_4k.launch"):
            root = ET.parse(PACKAGE / relative).getroot()
            defaults = {
                node.attrib["name"]: node.attrib.get("default")
                for node in root.findall("arg")
            }
            self.assertEqual(defaults["video_device"], DEVICE, relative)
            self.assertEqual(defaults["width"], "3840", relative)
            self.assertEqual(defaults["height"], "2160", relative)
            self.assertEqual(defaults["framerate"], "30.0", relative)
            self.assertEqual(defaults["pixel_format"], "mjpeg", relative)
            if relative == "launch/camera.launch":
                self.assertEqual(defaults["unknown_timestamp_clock"], "assume_monotonic")


if __name__ == "__main__":
    unittest.main()
