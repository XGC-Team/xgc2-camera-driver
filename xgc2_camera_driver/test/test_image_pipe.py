#!/usr/bin/env python3
"""xgc2_ros_image_pipe: ROS images in, raw frames on stdout, exit when the reader leaves."""

import subprocess
import threading
import time
import unittest

import rospy
import rostest
from sensor_msgs.msg import Image

TOPIC = "/pipe_test/image"
WIDTH, HEIGHT = 64, 48


class Publisher:
    """Publishes frames whose every byte equals the frame counter."""

    def __init__(self, encoding="bgr8", width=WIDTH, height=HEIGHT, rate=40.0, bytes_per_pixel=3):
        self.publisher = rospy.Publisher(TOPIC, Image, queue_size=1)
        self.encoding, self.width, self.height = encoding, width, height
        self.bytes_per_pixel = bytes_per_pixel
        self.rate = rate
        self.counter = 0
        self.stop = False
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def message(self):
        message = Image()
        message.header.stamp = rospy.Time.now()
        message.encoding = self.encoding
        message.width, message.height = self.width, self.height
        message.step = self.width * self.bytes_per_pixel
        message.data = bytes([self.counter % 250 + 1]) * (message.step * self.height)
        return message

    def run(self):
        period = 1.0 / self.rate
        while not self.stop and not rospy.is_shutdown():
            self.counter += 1
            self.publisher.publish(self.message())
            time.sleep(period)

    def close(self):
        self.stop = True
        self.thread.join()
        self.publisher.unregister()


def start_tool(*arguments):
    return subprocess.Popen(
        ["rosrun", "xgc2_camera_driver", "xgc2_ros_image_pipe", "--topic", TOPIC] + list(arguments),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def read_exactly(stream, count):
    data = b""
    while len(data) < count:
        chunk = stream.read(count - len(data))
        if not chunk:
            break
        data += chunk
    return data


class ImagePipeTest(unittest.TestCase):
    def test_frames_arrive_whole_and_unmixed_with_text(self):
        publisher = Publisher()
        tool = start_tool("--format", "bgr8", "--width", str(WIDTH), "--height", str(HEIGHT))
        try:
            frame_size = WIDTH * HEIGHT * 3
            for _ in range(5):
                frame = read_exactly(tool.stdout, frame_size)
                self.assertEqual(len(frame), frame_size)
                self.assertEqual(len(set(frame)), 1, "one frame carries one publication")
                self.assertGreaterEqual(frame[0], 1)
        finally:
            publisher.close()
            tool.terminate()
            rest = tool.stdout.read()
            tool.wait(timeout=10)
        self.assertEqual(len(rest) % (WIDTH * HEIGHT * 3), 0, "stdout never carries anything but frames")
        text = tool.stderr.read().decode()
        self.assertIn("64x48 bgr8 -> bgr8", text)

    def test_the_tool_exits_when_the_reader_closes_the_pipe(self):
        publisher = Publisher()
        tool = start_tool("--width", str(WIDTH), "--height", str(HEIGHT))
        try:
            self.assertEqual(len(read_exactly(tool.stdout, WIDTH * HEIGHT * 3)), WIDTH * HEIGHT * 3)
            tool.stdout.close()
            self.assertEqual(tool.wait(timeout=10), 0)
        finally:
            publisher.close()
            if tool.poll() is None:
                tool.kill()

    def test_encodings_are_converted_to_the_requested_format(self):
        publisher = Publisher(encoding="rgb8")
        tool = start_tool("--format", "mono8")
        try:
            frame = read_exactly(tool.stdout, WIDTH * HEIGHT)
            self.assertEqual(len(frame), WIDTH * HEIGHT)
        finally:
            publisher.close()
            tool.terminate()
            tool.wait(timeout=10)

    def test_gray_images_become_gray_colour_frames(self):
        publisher = Publisher(encoding="mono8", bytes_per_pixel=1)
        tool = start_tool("--format", "bgr8")
        try:
            frame = read_exactly(tool.stdout, WIDTH * HEIGHT * 3)
            self.assertEqual(len(frame), WIDTH * HEIGHT * 3)
            self.assertEqual(frame[0], frame[1])
            self.assertEqual(frame[1], frame[2])
        finally:
            publisher.close()
            tool.terminate()
            tool.wait(timeout=10)

    def test_depth_images_are_scaled_to_eight_bit_gray(self):
        publisher = Publisher(encoding="16UC1", bytes_per_pixel=2)
        tool = start_tool("--format", "mono8", "--depth-max", "65535")
        try:
            frame = read_exactly(tool.stdout, WIDTH * HEIGHT)
            self.assertEqual(len(frame), WIDTH * HEIGHT)
            # Every 16 bit pixel is 0xNNNN, so 65535 mm maps it to NN.
            self.assertEqual(len(set(frame)), 1)
            self.assertGreaterEqual(frame[0], 1)
        finally:
            publisher.close()
            tool.terminate()
            tool.wait(timeout=10)

    def test_max_fps_drops_the_surplus(self):
        publisher = Publisher(rate=40.0)
        tool = start_tool("--max-fps", "5")
        try:
            frame_size = WIDTH * HEIGHT * 3
            read_exactly(tool.stdout, frame_size)
            started = time.time()
            count = 0
            while time.time() - started < 2.0:
                read_exactly(tool.stdout, frame_size)
                count += 1
            self.assertGreaterEqual(count, 6)
            self.assertLessEqual(count, 12)
        finally:
            publisher.close()
            tool.terminate()
            tool.wait(timeout=10)

    def test_a_frame_of_the_wrong_size_is_fatal(self):
        publisher = Publisher()
        tool = start_tool("--width", "32", "--height", "24")
        try:
            self.assertEqual(tool.wait(timeout=15), 1)
            self.assertIn("image size changed", tool.stderr.read().decode())
            self.assertEqual(tool.stdout.read(), b"")
        finally:
            publisher.close()
            if tool.poll() is None:
                tool.kill()

    def test_an_unsupported_encoding_is_fatal(self):
        publisher = Publisher(encoding="32FC1", bytes_per_pixel=4)
        tool = start_tool()
        try:
            self.assertEqual(tool.wait(timeout=15), 1)
            self.assertIn("unsupported image encoding", tool.stderr.read().decode())
        finally:
            publisher.close()
            if tool.poll() is None:
                tool.kill()

    def test_missing_arguments_print_usage(self):
        tool = subprocess.run(
            ["rosrun", "xgc2_camera_driver", "xgc2_ros_image_pipe"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=20,
        )
        self.assertEqual(tool.returncode, 2)
        self.assertIn(b"usage", tool.stderr)
        self.assertEqual(tool.stdout, b"")


if __name__ == "__main__":
    rospy.init_node("image_pipe_contract_test")
    rostest.rosrun("xgc2_camera_driver", "image_pipe_contract", ImagePipeTest)
