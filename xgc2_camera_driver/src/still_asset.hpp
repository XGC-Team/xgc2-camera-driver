#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace xgc2_camera_driver {

// Intrinsics of a sealed scene image (schema xgc2.camera.intrinsic.v1).
struct Intrinsics {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::array<double, 9> camera_matrix{};
  std::string distortion_model;
  std::vector<double> distortion;
};

// Pose of the camera optical frame in its parent frame (schema
// xgc2.camera.extrinsic.v1, frame convention parent_T_camera_optical).
struct Extrinsic {
  std::string parent_frame;
  std::string child_frame;
  std::array<double, 3> translation{};
  std::array<double, 4> quaternion{};  // x, y, z, w; normalized
};

struct StaticTransform {
  std::string parent_frame;
  std::string child_frame;
  std::array<double, 3> translation{};
  std::array<double, 4> quaternion{};  // x, y, z, w; normalized
};

// The static chain of the physical USB camera contract:
// parent -> <camera>_link -> <camera>_optical_frame.
struct CameraStaticTransforms {
  StaticTransform parent_to_link;
  StaticTransform link_to_optical;
};

// Standard ROS optical rotation (camera link -> optical frame), xyzw.
extern const std::array<double, 4> kStandardLinkToOpticalQuaternion;

using Quaternion = std::array<double, 4>;

Quaternion quaternionMultiply(const Quaternion& a, const Quaternion& b);
// Throws std::invalid_argument for a zero or non-finite norm.
Quaternion quaternionNormalize(const Quaternion& value);
Quaternion quaternionInverse(const Quaternion& value);

// "<name>_optical_frame" -> "<name>_link". Throws std::invalid_argument for any
// other spelling.
std::string deriveLinkFrame(const std::string& optical_frame);

CameraStaticTransforms composeStaticCameraTransforms(
    const Extrinsic& extrinsic,
    const Quaternion& link_to_optical = kStandardLinkToOpticalQuaternion);

// All loaders throw std::invalid_argument with an operator-readable message.
Intrinsics loadIntrinsics(const std::string& path);
Extrinsic loadExtrinsic(const std::string& path);

struct StillImage {
  // JPEG bytes published as they are. A JPEG input is kept byte for byte; a
  // PNG input is converted once at the requested quality.
  std::vector<std::uint8_t> jpeg;
  std::uint32_t width{0};
  std::uint32_t height{0};
};

// Loads a .png, .jpg or .jpeg scene image and checks it against the intrinsics.
StillImage loadStillImage(
    const std::string& path, const Intrinsics& intrinsics, int jpeg_quality);

}  // namespace xgc2_camera_driver
