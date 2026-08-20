#pragma once

#include <cstdint>
#include <sstream>
#include <stdexcept>

namespace xgc2_camera_driver {

inline void validateCalibrationGeometry(
    const bool calibrated,
    const std::uint32_t calibration_width,
    const std::uint32_t calibration_height,
    const std::uint32_t capture_width,
    const std::uint32_t capture_height)
{
  if (!calibrated) {
    return;
  }
  if (calibration_width == capture_width &&
      calibration_height == capture_height) {
    return;
  }
  std::ostringstream message;
  message << "camera calibration geometry " << calibration_width << "x"
          << calibration_height << " does not match capture geometry "
          << capture_width << "x" << capture_height
          << "; calibrate and load native-resolution intrinsics instead of "
             "resizing or relabeling lower-resolution calibration";
  throw std::invalid_argument(message.str());
}

}  // namespace xgc2_camera_driver
