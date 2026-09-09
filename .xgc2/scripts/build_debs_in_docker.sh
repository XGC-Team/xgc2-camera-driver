#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DOCKER_IMAGE="${DOCKER_IMAGE:-ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic:1.0.0}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) DOCKER_IMAGE="$2"; shift 2 ;;
    --work-dir) WORK_DIR="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --skip-install-check) INSTALL_CHECK=false; shift ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"
docker pull "${DOCKER_IMAGE}"
docker run --rm \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -e PACKAGE_VERSION="${PACKAGE_VERSION:-}" \
  -e XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}" \
  -v "${REPO_ROOT}:/workspace/repo:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" bash -lc '
    set -euo pipefail
    : "${ROS_DISTRO:?ROS_DISTRO must be set in the image}"
    # shellcheck disable=SC1091
    . /etc/os-release
    suite="${VERSION_CODENAME:-}"
    [[ -n "${suite}" ]] || { echo "VERSION_CODENAME missing" >&2; exit 1; }

    for pkg in \
      cmake \
      dpkg-dev \
      fakeroot \
      file \
      libavcodec-dev \
      libavutil-dev \
      libswscale-dev \
      pkg-config \
      rsync \
      "ros-${ROS_DISTRO}-camera-info-manager" \
      "ros-${ROS_DISTRO}-cv-bridge" \
      "ros-${ROS_DISTRO}-diagnostic-updater" \
      "ros-${ROS_DISTRO}-image-transport" \
      "ros-${ROS_DISTRO}-roscpp" \
      "ros-${ROS_DISTRO}-rostest"
    do
      if ! dpkg -s "${pkg}" >/dev/null 2>&1; then
        echo "image is missing ${pkg}; add it to xgc2-images, do not apt in product CI" >&2
        exit 1
      fi
    done
    if [[ "${ROS_DISTRO}" == "noetic" ]]; then
      dpkg -s ros-noetic-foxglove-msgs >/dev/null 2>&1 || {
        echo "image is missing ros-noetic-foxglove-msgs" >&2
        exit 1
      }
    fi

    echo "deb [trusted=yes arch=$(dpkg --print-architecture)] https://xgc2.apt.xiaokang.ink ${suite} main" >/etc/apt/sources.list.d/xgc2.list
    if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
      sed "s#https://xgc2.apt.xiaokang.ink#${XGC2_APT_OVERLAY_URL%/}#g" /etc/apt/sources.list.d/xgc2.list >/etc/apt/sources.list.d/00-xgc2-release-train.list
    fi
    apt-get update
    apt-get install -y --no-install-recommends \
      libxgc2-camera-dev \
      "ros-${ROS_DISTRO}-xgc2-camera-msgs"
    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src/xgc2_camera_driver
    rsync -a --delete /workspace/repo/xgc2_camera_driver/ /workspace/work/src/xgc2_camera_driver/
    if [[ "${ROS_DISTRO}" == "melodic" ]]; then
      mkdir -p /workspace/work/src/foxglove_msgs
      rsync -a --delete /workspace/repo/foxglove_msgs/ /workspace/work/src/foxglove_msgs/
    fi
    cd /workspace/work
    set +u
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
    ROS_HOME=/workspace/work/ros-home ROS_LOG_DIR=/workspace/work/ros-log catkin_make run_tests
    catkin_test_results --verbose
    DESTDIR=/workspace/work/install-root catkin_make install \
      -DCMAKE_INSTALL_PREFIX="/opt/ros/${ROS_DISTRO}" \
      -DCATKIN_ENABLE_TESTING=OFF
    /workspace/repo/.xgc2/scripts/package_debs.sh \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out
    if [[ "${INSTALL_CHECK}" == true ]]; then
      apt-get install -y /workspace/out/ros-${ROS_DISTRO}-xgc2-camera-driver_*.deb
      /workspace/repo/.xgc2/scripts/check_installed_packages.sh
    fi
  '

find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
