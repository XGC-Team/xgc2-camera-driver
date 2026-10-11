#!/usr/bin/env python3
"""Private-master camera and image-pipe lifecycle tests; no hardware required.

Fast: test_peer_lifecycle.py --bin-dir BUILD/lib/xgc2_camera_driver
Soaks: add --long (60 seconds each for absent master and absent publisher).
Every child, including all roscore children, is terminated and waited for.
"""

import argparse
import json
import os
from pathlib import Path
import random
import select
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest
import xmlrpc.client

from xrpc_unix import call

PACKAGE = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT = 64, 48
FRAME_BYTES = WIDTH * HEIGHT * 3


def publisher(topic, value):
    import rospy
    from sensor_msgs.msg import Image
    rospy.init_node("pipe_restart_publisher", anonymous=True)
    output = rospy.Publisher(topic, Image, queue_size=1)
    message = Image(width=WIDTH, height=HEIGHT, encoding="bgr8", step=WIDTH * 3,
                    data=bytes([value]) * FRAME_BYTES)
    while not rospy.is_shutdown():
        message.header.stamp = rospy.Time.now()
        output.publish(message)
        time.sleep(0.025)


def resources(pid):
    stat = Path("/proc/{}/stat".format(pid)).read_text().rsplit(")", 1)[1].split()
    cpu = (int(stat[11]) + int(stat[12])) / os.sysconf("SC_CLK_TCK")
    fds = len(list(Path("/proc/{}/fd".format(pid)).iterdir()))
    threads = len(list(Path("/proc/{}/task".format(pid)).iterdir()))
    return cpu, fds, threads


@unittest.skipUnless(__name__ == "__main__", "run test_peer_lifecycle.py with --bin-dir")
class PeerLifecycleTest(unittest.TestCase):
    def setUp(self):
        self.directory = Path(tempfile.mkdtemp(prefix="camera-peers-"))
        self.addCleanup(shutil.rmtree, str(self.directory))
        self.children = []
        self.environment = dict(os.environ)
        for _ in range(100):
            port = random.SystemRandom().randint(20000, 40000)
            with socket.socket() as candidate:
                try:
                    candidate.bind(("127.0.0.1", port))
                    break
                except OSError:
                    continue
        else:
            self.fail("no private ROS master port available")
        self.port = port
        self.environment.update(ROS_MASTER_URI="http://127.0.0.1:{}/".format(port),
                                ROS_HOME=str(self.directory / "ros-home"),
                                ROS_LOG_DIR=str(self.directory / "ros-log"))
        self.counter_library = self.directory / "connect_counter.so"
        subprocess.run(["cc", "-std=c11", "-shared", "-fPIC", "-O2",
                        str(PACKAGE / "test/connect_counter.c"), "-ldl", "-o", str(self.counter_library)],
                       check=True, timeout=30)
        self.addCleanup(self.stop_children)

    def start(self, name, command, frames=False, instrument=False):
        environment = dict(self.environment)
        counter = self.directory / (name + ".connections")
        if instrument:
            counter.write_bytes(bytes(8))
            environment.update(LD_PRELOAD=str(self.counter_library), XGC_TEST_CONNECT_COUNTER=str(counter))
        log = open(str(self.directory / (name + ".log")), "wb")
        self.addCleanup(log.close)
        process = subprocess.Popen(command, env=environment, start_new_session=True,
                                   stdout=subprocess.PIPE if frames else log, stderr=log)
        self.children.append(process)
        process.counter_path = counter
        process.log_path = Path(log.name)
        if frames:
            self.addCleanup(process.stdout.close)
        return process

    def stop(self, process):
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=10)
        else:
            process.wait()

    def stop_children(self):
        for process in reversed(self.children):
            self.stop(process)

    def wait(self, predicate, timeout=35):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.05)
        logs = "\n".join(process.log_path.read_text(errors="replace")[-3000:] for process in self.children)
        self.fail("condition did not converge:\n" + logs)

    def master(self):
        master = self.start("master-{}".format(len(self.children)), ["roscore", "-p", str(self.port)])
        client = xmlrpc.client.ServerProxy(self.environment["ROS_MASTER_URI"])
        self.addCleanup(client.__exit__, None, None, None)

        def connected():
            try:
                return client.getPid("/peer_lifecycle_test")[0] == 1
            except (OSError, xmlrpc.client.Error):
                return False
        self.wait(connected, timeout=15)
        return master, client

    def describe(self, path):
        try:
            status, _, body = call(str(path), "GET", "/v1/describe", timeout_ms=1000)
            return json.loads(body) if status == 200 else {}
        except (OSError, ValueError):
            return {}

    def connections(self, process):
        return struct.unpack("Q", process.counter_path.read_bytes())[0]

    def outage(self, process, label):
        duration = 60.0 if OPTIONS.long else 0.5
        before_cpu, before_fds, before_threads = resources(process.pid)
        attempts = self.connections(process)
        lines = len(process.log_path.read_text(errors="replace").splitlines())
        time.sleep(duration)
        self.assertIsNone(process.poll(), label + " exited while its peer was down")
        after_cpu, after_fds, after_threads = resources(process.pid)
        attempts = self.connections(process) - attempts
        lines = len(process.log_path.read_text(errors="replace").splitlines()) - lines
        if OPTIONS.long:
            self.assertLess(after_cpu - before_cpu, 1.2, label + " consumed CPU while idle")
            self.assertLessEqual(attempts, 15, label + " caused a connection storm")
            self.assertLessEqual(lines, 7, label + " caused a log storm")
        self.assertLessEqual(after_fds, before_fds)
        self.assertLessEqual(after_threads, before_threads)
        print("{}: {:.1f}s CPU={:.3f}s connects={} logs={} fds={}->{} threads={}->{}".format(
            label, duration, after_cpu - before_cpu, attempts, lines,
            before_fds, after_fds, before_threads, after_threads), flush=True)
        return before_fds, before_threads

    def camera(self, name):
        path = self.directory / (name + ".sock")
        process = self.start(name, [str(Path(OPTIONS.bin_dir) / "xgc2_camera_driver_node"),
                            "__name:=" + name, "_source_kind:=still", "_control_socket:=" + str(path),
                            "_source_id:=front", "_h264_encoder:=libx264", "_preview_enabled:=false",
                            "_still_image:=" + str(PACKAGE / "test/data/still_scene.png"),
                            "_still_intrinsic_file:=" + str(PACKAGE / "test/data/still_intrinsic.yaml"),
                            "_still_extrinsic_file:=" + str(PACKAGE / "test/data/still_extrinsic.yaml")], instrument=True)
        return process, path

    def test_camera_waits_for_its_master_and_ends_on_replacement(self):
        camera, path = self.camera("late_master_camera")
        self.wait(lambda: bool(self.describe(path)), timeout=10)
        waiting = self.describe(path)
        self.assertFalse(waiting["ready"])
        self.assertIn("ROS master", waiting["facts"]["reason"])
        instance = waiting["instance_id"]
        self.outage(camera, "ROS master down")
        master, _ = self.master()
        self.wait(lambda: self.describe(path).get("ready", False))
        ready = self.describe(path)
        self.assertEqual(ready["instance_id"], instance)
        self.assertTrue(ready["facts"]["source_open"])
        self.assertTrue(ready["facts"]["frames_flowing"])
        self.assertGreater(ready["facts"]["ros_master_pid"], 0)
        self.assertFalse(ready["facts"]["active"], "source readiness must not need a media lease")
        self.stop(master)
        self.master()
        self.assertEqual(camera.wait(timeout=15), 1)

    def test_still_readiness_does_not_wait_for_a_world_clock(self):
        _, master = self.master()
        master.setParam("/peer_lifecycle_test", "/use_sim_time", True)
        camera, path = self.camera("no_clock_camera")
        self.wait(lambda: self.describe(path).get("ready", False), timeout=10)
        self.assertIsNone(camera.poll())
        self.assertTrue(self.describe(path)["facts"]["source_open"])

    def read_frame(self, process):
        data = b""
        deadline = time.monotonic() + 10
        while len(data) < FRAME_BYTES and time.monotonic() < deadline:
            if select.select([process.stdout], [], [], 0.2)[0]:
                chunk = os.read(process.stdout.fileno(), FRAME_BYTES - len(data))
                if not chunk:
                    break
                data += chunk
        self.assertEqual(len(data), FRAME_BYTES, process.log_path.read_text(errors="replace"))
        return data

    def stop_publisher(self, publisher, pipe):
        self.stop(publisher)
        time.sleep(0.25)
        # Drain complete buffered frames and allow the final blocked write to
        # finish. Dropping arbitrary byte chunks would destroy frame boundaries.
        drain_until = time.monotonic() + 3
        while select.select([pipe.stdout], [], [], 0.25)[0]:
            self.assertLess(time.monotonic(), drain_until, "publisher backlog was unbounded")
            self.assertEqual(set(self.read_frame(pipe)), {17})

    def test_image_pipe_waits_for_a_late_topic_and_resumes_after_publisher_restart(self):
        self.master()
        topic = "/peer_lifecycle/image"
        pipe = self.start("pipe", [str(Path(OPTIONS.bin_dir) / "xgc2_ros_image_pipe"), "--topic", topic,
                                   "--width", str(WIDTH), "--height", str(HEIGHT)], frames=True, instrument=True)
        time.sleep(0.5)
        self.assertFalse(select.select([pipe.stdout], [], [], 0)[0], "absent topic emitted output")
        idle_fds, idle_threads = self.outage(pipe, "ROS topic starts late")
        # The first disconnect creates roscpp's one shared retry-timer thread.
        # The master's three notification workers also establish reusable
        # XMLRPC connections. Warm these fixed resources before comparing
        # connected states across the measured publisher outage.
        for cycle in range(4):
            first = self.start("publisher_{}".format(cycle),
                               [sys.executable, __file__, "--publisher", topic, "17"])
            self.assertEqual(set(self.read_frame(pipe)), {17})
            time.sleep(0.25)
            _, fds, threads = resources(pipe.pid)
            self.assertLessEqual(fds, idle_fds + 4)
            self.assertLessEqual(threads, idle_threads + 1)
            if cycle < 3:
                self.stop_publisher(first, pipe)
        _, connected_fds, connected_threads = resources(pipe.pid)
        self.stop_publisher(first, pipe)
        self.outage(pipe, "ROS publisher down")
        second = self.start("publisher_second", [sys.executable, __file__, "--publisher", topic, "93"])
        self.assertEqual(set(self.read_frame(pipe)), {93})
        self.assertIsNone(pipe.poll())
        self.wait(lambda: resources(pipe.pid)[1] <= connected_fds and
                  resources(pipe.pid)[2] <= connected_threads)
        _, restored_fds, restored_threads = resources(pipe.pid)
        print("ROS publisher restored: fds={}->{} threads={}->{}".format(
            connected_fds, restored_fds, connected_threads, restored_threads), flush=True)
        self.stop(second)


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "--publisher":
        publisher(sys.argv[2], int(sys.argv[3]))
    else:
        parser = argparse.ArgumentParser()
        parser.add_argument("--bin-dir", required=True)
        parser.add_argument("--long", action="store_true")
        OPTIONS, rest = parser.parse_known_args()
        unittest.main(argv=[sys.argv[0]] + rest, verbosity=2)
