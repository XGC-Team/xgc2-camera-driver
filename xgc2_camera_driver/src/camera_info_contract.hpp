#pragma once

#include <climits>
#include <cstdint>
#include <cstdlib>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace xgc2_camera_driver {

inline std::string validateStableCameraName(const std::string& value)
{
  static const std::regex pattern("^[A-Za-z][A-Za-z0-9._-]{0,63}$");
  if (!std::regex_match(value, pattern)) {
    throw std::invalid_argument(
        "camera_name must match ^[A-Za-z][A-Za-z0-9._-]{0,63}$");
  }
  return value;
}

inline std::string canonicalPhysicalCameraInfoFile(
    const std::string& value,
    const std::string& camera_name)
{
  validateStableCameraName(camera_name);
  if (value.empty() || value.front() != '/') {
    throw std::invalid_argument(
        "camera_info_file must be an absolute existing file path");
  }
  char resolved[PATH_MAX] = {};
  if (::realpath(value.c_str(), resolved) == nullptr) {
    throw std::invalid_argument(
        "camera_info_file must resolve to an existing file");
  }
  const std::string canonical(resolved);
  const std::size_t file_separator = canonical.find_last_of('/');
  const std::string directory = canonical.substr(0, file_separator);
  const std::string filename = canonical.substr(file_separator + 1U);
  const std::size_t camera_separator = directory.find_last_of('/');
  const std::string camera_partition = directory.substr(camera_separator + 1U);
  const std::string mode_directory = directory.substr(0, camera_separator);
  const std::size_t mode_separator = mode_directory.find_last_of('/');
  const std::string mode_partition = mode_directory.substr(mode_separator + 1U);
  static const std::regex filename_pattern(
      "intrinsics-[0-9]{8}T[0-9]{6}\\.[0-9]{6}Z\\.yaml");
  if (file_separator == std::string::npos ||
      camera_separator == std::string::npos ||
      mode_separator == std::string::npos || camera_partition != camera_name ||
      mode_partition != "phy" || !std::regex_match(filename, filename_pattern)) {
    throw std::invalid_argument(
        "camera_info_file must resolve under <root>/phy/<camera_name>/ and "
        "match the configured camera_name");
  }
  return canonical;
}

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
