#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
INSTALL_ROOT=""
OUTPUT_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root) INSTALL_ROOT="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

VERSION="${PACKAGE_VERSION:-$(awk -F': *' '/^version:/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml")}"
if [[ -z "${VERSION}" ]]; then
  echo "package version is missing" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
BUILD_ROOT="$(mktemp -d)"
trap 'rm -rf "${BUILD_ROOT}"' EXIT
mkdir -p "${OUTPUT_DIR}"

copy_path() {
  local source="$1"
  local package_root="$2"
  if [[ ! -e "${source}" ]]; then
    return
  fi
  local relative="${source#${INSTALL_ROOT}}"
  mkdir -p "${package_root}$(dirname "${relative}")"
  cp -a "${source}" "${package_root}${relative}"
}

copy_ros_package() {
  local ros_package="$1"
  local package_root="$2"
  copy_path "${INSTALL_ROOT}${PREFIX}/share/${ros_package}" "${package_root}"
  copy_path "${INSTALL_ROOT}${PREFIX}/lib/${ros_package}" "${package_root}"
  copy_path "${INSTALL_ROOT}${PREFIX}/include/${ros_package}" "${package_root}"
  copy_path "${INSTALL_ROOT}${PREFIX}/lib/python3/dist-packages/${ros_package}" "${package_root}"
  copy_path "${INSTALL_ROOT}${PREFIX}/lib/python2.7/dist-packages/${ros_package}" "${package_root}"
}

write_control() {
  local package_root="$1"
  local package_name="$2"
  local dependencies="$3"
  local description="$4"
  mkdir -p "${package_root}/DEBIAN" "${package_root}/usr/share/doc/${package_name}"
  cat >"${package_root}/DEBIAN/control" <<EOF
Package: ${package_name}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <dev@xiaokang.ink>
Depends: ${dependencies}
Description: ${description}
EOF
  install -m 0644 "${REPO_ROOT}/LICENSE" "${package_root}/usr/share/doc/${package_name}/copyright"
  chmod 0755 "${package_root}/DEBIAN"
}

driver_depends() {
  case "${ROS_DISTRO}" in
    melodic)
      printf '%s' "ffmpeg, libavcodec57, libavutil55, libopencv-core3.2, libopencv-imgcodecs3.2, libopencv-imgproc3.2, libswscale4, libxgc2-camera-dev (>= 0.1.0-10~bionic), ros-melodic-camera-info-manager, ros-melodic-cv-bridge, ros-melodic-diagnostic-msgs, ros-melodic-diagnostic-updater, ros-melodic-image-transport, ros-melodic-rosbag, ros-melodic-roscpp, ros-melodic-roslaunch, ros-melodic-rostopic, ros-melodic-sensor-msgs, ros-melodic-xgc2-camera-msgs (>= 1.2.0-8)"
      ;;
    noetic)
      printf '%s' "ffmpeg, libavcodec58, libavutil56, libopencv-core4.2, libopencv-imgcodecs4.2, libopencv-imgproc4.2, libswscale5, libxgc2-camera-dev (>= 0.1.0-10~focal), ros-noetic-camera-info-manager, ros-noetic-cv-bridge, ros-noetic-diagnostic-msgs, ros-noetic-diagnostic-updater, ros-noetic-foxglove-msgs, ros-noetic-image-transport, ros-noetic-rosbag, ros-noetic-roscpp, ros-noetic-roslaunch, ros-noetic-rostopic, ros-noetic-sensor-msgs, ros-noetic-xgc2-camera-msgs (>= 1.2.0-8)"
      ;;
    *)
      echo "unsupported ROS_DISTRO: ${ROS_DISTRO}" >&2
      exit 1
      ;;
  esac
}

build_driver() {
  local package_name="ros-${ROS_DISTRO}-xgc2-camera-driver"
  local package_root="${BUILD_ROOT}/${package_name}"
  mkdir -p "${package_root}"
  copy_ros_package xgc2_camera_driver "${package_root}"
  if [[ "${ROS_DISTRO}" == "melodic" ]]; then
    copy_ros_package foxglove_msgs "${package_root}"
  fi
  write_control "${package_root}" "${package_name}" \
    "$(driver_depends)" \
    "XGC2 ROS ${ROS_DISTRO} adapter for the independent Linux camera core"
  test -x "${package_root}${PREFIX}/lib/xgc2_camera_driver/xgc2_camera_driver_node"
  if [[ "${ROS_DISTRO}" == "melodic" ]]; then
    test -d "${package_root}${PREFIX}/share/foxglove_msgs"
  fi
  find "${package_root}" -type d -exec chmod 0755 {} +
  find "${package_root}" -type f -exec chmod 0644 {} +
  chmod 0755 "${package_root}${PREFIX}/lib/xgc2_camera_driver/xgc2_camera_driver_node"
  for bin in xgc_native_v4l2_rtp xgc_ros_image_rtp \
    xgc_camera_apply_alignment xgc_camera_alignment xgc_camera_bag_export
  do
    if [[ -e "${package_root}${PREFIX}/lib/xgc2_camera_driver/${bin}" ]]; then
      chmod 0755 "${package_root}${PREFIX}/lib/xgc2_camera_driver/${bin}"
    fi
  done
  strip --strip-unneeded "${package_root}${PREFIX}/lib/xgc2_camera_driver/xgc2_camera_driver_node" 2>/dev/null || true
  fakeroot dpkg-deb --build "${package_root}" "${OUTPUT_DIR}/${package_name}_${VERSION}_${ARCH}.deb" >/dev/null
}

build_driver
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
