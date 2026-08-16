#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

for script in .xgc2/scripts/*.sh; do
  bash -n "${script}"
done

PYTHONPYCACHEPREFIX="${TMPDIR:-/tmp}/xgc2-camera-pycache" python3 -m py_compile \
  xgc2_camera_driver/test/*.py \
  .xgc2/scripts/xgc2_artifact_manifest.py

MANIFEST_TEST_ROOT="$(mktemp -d)"
trap 'rm -rf "${MANIFEST_TEST_ROOT}"' EXIT
MANIFEST_TEST_ARCH="$(dpkg --print-architecture)"
mkdir -p \
  "${MANIFEST_TEST_ROOT}/package/DEBIAN" \
  "${MANIFEST_TEST_ROOT}/debs" \
  "${MANIFEST_TEST_ROOT}/manifests"
printf '%s\n' \
  'Package: xgc2-camera-manifest-contract' \
  'Version: 0.3.0-2' \
  'Section: misc' \
  'Priority: optional' \
  "Architecture: ${MANIFEST_TEST_ARCH}" \
  'Maintainer: XGC2 <dev@xiaokang.ink>' \
  'Description: XGC2 camera build-manifest contract test' \
  >"${MANIFEST_TEST_ROOT}/package/DEBIAN/control"
dpkg-deb --build \
  "${MANIFEST_TEST_ROOT}/package" \
  "${MANIFEST_TEST_ROOT}/debs/xgc2-camera-manifest-contract_0.3.0-2_${MANIFEST_TEST_ARCH}.deb" \
  >/dev/null
python3 .xgc2/scripts/xgc2_artifact_manifest.py build \
  --deb-dir "${MANIFEST_TEST_ROOT}/debs" \
  --output-dir "${MANIFEST_TEST_ROOT}/manifests" \
  --product xgc2-camera-ros1 \
  --product-version 0.3.0-2 \
  --distribution focal \
  --architecture "${MANIFEST_TEST_ARCH}" \
  --source-sha 0000000000000000000000000000000000000000 \
  --ci-run-id compliance \
  --ci-workflow ci \
  --ci-workflow-ref refs/heads/main

MANIFEST_TEST_ROOT="${MANIFEST_TEST_ROOT}" python3 - <<'PY'
import json
import os
import pathlib
import xml.etree.ElementTree as ET

root = pathlib.Path(".")
for path in sorted(root.glob("xgc2_camera_*/package.xml")):
    ET.parse(path)
for path in sorted(root.glob("xgc2_camera_*/*/*.launch")):
    ET.parse(path)

manifest_paths = list(
    (pathlib.Path(os.environ["MANIFEST_TEST_ROOT"]) / "manifests").glob("*.json")
)
assert len(manifest_paths) == 1
manifest = json.loads(manifest_paths[0].read_text(encoding="utf-8"))
assert set(manifest) == {
    "schema", "product", "source_sha", "version", "distribution",
    "architecture", "ci", "created_at", "debs",
}
assert manifest["schema"] == "xgc2.build-artifact.v1"
assert manifest["product"] == "xgc2-camera-ros1"
assert manifest["version"] == "0.3.0-2"
assert set(manifest["ci"]) == {"run_id", "workflow", "workflow_ref"}
assert len(manifest["debs"]) == 1
deb = manifest["debs"][0]
assert set(deb) == {
    "file", "package", "version", "architecture", "sha256", "size",
}
assert deb["package"] == "xgc2-camera-manifest-contract"
assert deb["version"] == "0.3.0-2"
assert len(deb["sha256"]) == 64
assert deb["size"] > 0
PY

grep -q '^id: xgc2-camera-ros1$' .xgc2/product.yml
grep -q '^version: 0.3.0-10$' .xgc2/product.yml
grep -q '^    focal: 0.3.0-10$' .xgc2/product.yml
if grep -q '^    focal: .*~focal' .xgc2/product.yml; then
  echo "single-distribution ROS1 package version must not retain a focal suffix" >&2
  exit 1
fi
grep -q 'xgc2::camera' xgc2_camera_driver/CMakeLists.txt
grep -q 'ffmpeg, .*ros-noetic-xgc2-camera-msgs (>= 1.2.0-8)' \
  .xgc2/scripts/package_debs.sh
grep -q 'libxgc2-camera-dev (>= 0.1.0-7~focal)' \
  .xgc2/scripts/package_debs.sh
grep -q 'ros-noetic-image-transport, ros-noetic-rosbag, ros-noetic-roscpp' \
  .xgc2/scripts/package_debs.sh
grep -q 'xgc2-build-focal-full-noetic:1.0.0' \
  .xgc2/scripts/build_debs_in_docker.sh
grep -q 'libxgc2-camera-dev' \
  .xgc2/scripts/build_debs_in_docker.sh
grep -q 'ros-noetic-xgc2-camera-msgs' \
  .xgc2/scripts/build_debs_in_docker.sh
if grep -Eq 'ros-noetic-(foxglove-msgs|image-transport|sensor-msgs)' \
  .xgc2/scripts/build_debs_in_docker.sh; then
  echo "third-party ROS build dependencies must come from the XGC2 image" >&2
  exit 1
fi
if grep -R -E 'xgc_camera_calibration|xgc2-camera-(intrinsic|extrinsic)|xgc2-camera-calibration-ros1.json' \
  .xgc2/scripts/build_debs_in_docker.sh .xgc2/scripts/package_debs.sh \
  .xgc2/scripts/check_installed_packages.sh >/dev/null; then
  echo "calibration implementation leaked into the ROS camera driver product" >&2
  exit 1
fi

echo "ROS1 camera product compliance passed"
