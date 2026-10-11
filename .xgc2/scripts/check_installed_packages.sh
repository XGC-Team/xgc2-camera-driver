#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"
DRIVER="${PREFIX}/lib/xgc2_camera_driver/xgc2_camera_driver_node"

source "${PREFIX}/setup.bash"
dpkg -s "ros-${ROS_DISTRO}-xgc2-camera-driver" >/dev/null
test "$(rospack find xgc2_camera_driver)" = "${PREFIX}/share/xgc2_camera_driver"
test -x "${DRIVER}"
test -x "${PREFIX}/lib/xgc2_camera_driver/xgc2_ros_image_pipe"
if [[ "${ROS_DISTRO}" == "noetic" ]]; then
  for definition in xgc2-camera-v4l2 xgc2-camera-still; do
    python3 - "/usr/share/xgc2/process-definitions/${definition}.json" "${definition}" <<'PY'
import json
import sys

document = json.load(open(sys.argv[1], encoding="utf-8"))
assert document["apiVersion"] == "xgc.execution.process/v1"
assert document["definitions"][0]["id"] == sys.argv[2]
PY
  done
fi
roslaunch --files xgc2_camera_driver camera.launch >/dev/null
roslaunch --files xgc2_camera_driver usb_camera_4k.launch >/dev/null
if [[ "${ROS_DISTRO}" == "melodic" ]]; then
  test "$(rospack find foxglove_msgs)" = "${PREFIX}/share/foxglove_msgs"
fi
if ldd "${DRIVER}" | grep -q 'not found'; then
  ldd "${DRIVER}" >&2
  exit 1
fi

RUNTIME="$(mktemp -d)"
ROSCORE_PID=""
DRIVER_PID=""
cleanup() {
  if [[ -n "${DRIVER_PID}" ]]; then kill "${DRIVER_PID}" 2>/dev/null || true; fi
  if [[ -n "${ROSCORE_PID}" ]]; then kill "${ROSCORE_PID}" 2>/dev/null || true; fi
  wait "${DRIVER_PID}" 2>/dev/null || true
  wait "${ROSCORE_PID}" 2>/dev/null || true
  rm -rf "${RUNTIME}"
}
trap cleanup EXIT
export ROS_MASTER_URI="http://127.0.0.1:11359"
export ROS_HOME="${RUNTIME}/ros-home"
export ROS_LOG_DIR="${RUNTIME}/ros-log"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"
roscore -p 11359 >"${RUNTIME}/roscore.log" 2>&1 &
ROSCORE_PID="$!"
for _ in $(seq 1 50); do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep 0.1
done
rosparam list >/dev/null
SOCKET_ARGS=()
if [[ "${ROS_DISTRO}" == "noetic" ]]; then
  # RUNTIME is a private 0700 directory, as the supervisor provides one.
  SOCKET_ARGS=("_control_socket:=${RUNTIME}/camera.sock" "_h264_encoder:=libx264")
fi
"${DRIVER}" __name:=driver __ns:=/usb_cam \
  _source_kind:=synthetic \
  _width:=320 _height:=240 _framerate:=10 \
  _output_encoding:=bgr8 _frame_id:=usb_cam_optical_frame \
  "${SOCKET_ARGS[@]}" \
  >"${RUNTIME}/driver.log" 2>&1 &
DRIVER_PID="$!"
timeout 15 rostopic echo -n 1 /usb_cam/image_raw/compressed/header/stamp >"${RUNTIME}/stamp.yaml"
timeout 15 rostopic echo -n 1 /usb_cam/camera_info/header/stamp >/dev/null
test "$(rostopic type /usb_cam/image_raw/compressed)" = "sensor_msgs/CompressedImage"
test "$(rostopic type /usb_cam/camera_info)" = "sensor_msgs/CameraInfo"
grep -q '^secs:' "${RUNTIME}/stamp.yaml"

if [[ "${ROS_DISTRO}" == "noetic" ]]; then
  # The camera-source service answers discovery and reports the camera ready.
  python3 - "${RUNTIME}/camera.sock" <<'PY'
import http.client
import json
import socket
import sys
import uuid


class Connection(http.client.HTTPConnection):
    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.connect(sys.argv[1])


def get(target, instance=None):
    headers = {"X-Request-ID": uuid.uuid4().hex, "X-Xrpc-Timeout-Ms": "20000"}
    if instance:
        headers["X-Xrpc-Instance-ID"] = instance
    connection = Connection("localhost", timeout=25)
    connection.request("GET", target, headers=headers)
    response = connection.getresponse()
    return response.status, json.loads(response.read())


status, discovery = get("/v1/describe")
assert status == 200 and discovery["service"] == "camera-source", discovery
status, ready = get("/v1/describe?wait_ready_ms=15000", discovery["instance_id"])
assert status == 200 and ready["ready"] is True, ready
status, source = get("/v1/media/sources/usb_cam/describe", discovery["instance_id"])
assert status == 200 and source["codec"] == "H264" and source["width"] == 320, source
PY
fi

echo "Installed ROS1 camera driver and synthetic topic contract passed"
