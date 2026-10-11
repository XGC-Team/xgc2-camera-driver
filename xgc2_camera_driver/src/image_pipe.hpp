#pragma once

// ROS-independent part of xgc2_ros_image_pipe: converting one sensor_msgs/Image
// payload into the raw frame an encoder pipeline reads from a pipe. Written for
// C++14 so the tool builds on Melodic (gcc 7.5) as well as newer releases.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace xgc2_camera_driver {

enum class PipeFormat { Bgr8, Rgb8, Mono8 };

inline const char* pipeFormatName(const PipeFormat format)
{
  switch (format) {
    case PipeFormat::Bgr8:
      return "bgr8";
    case PipeFormat::Rgb8:
      return "rgb8";
    case PipeFormat::Mono8:
      return "mono8";
  }
  return "bgr8";
}

inline std::size_t pipeFrameBytes(
    const PipeFormat format, const std::uint32_t width, const std::uint32_t height)
{
  return static_cast<std::size_t>(width) * height * (format == PipeFormat::Mono8 ? 1U : 3U);
}

namespace detail {

struct Rgb {
  std::uint8_t r, g, b;
};

inline std::uint8_t luma(const Rgb value)
{
  return static_cast<std::uint8_t>((299U * value.r + 587U * value.g + 114U * value.b) / 1000U);
}

}  // namespace detail

// Appends the width x height frame in `format`, rows without padding, to `out`.
// `encoding` is the sensor_msgs/image_encodings name of `data`; depth images
// (16UC1, mono16) are scaled to 8 bit so that depth_max_mm maps to white.
// Returns an empty string, or why the image cannot be converted.
inline std::string convertImage(
    const std::string& encoding,
    const std::uint32_t width,
    const std::uint32_t height,
    const std::uint32_t step,
    const bool big_endian,
    const std::uint8_t* data,
    const std::size_t size,
    const PipeFormat format,
    const double depth_max_mm,
    std::vector<std::uint8_t>* out)
{
  std::size_t pixel_bytes = 0;
  if (encoding == "bgr8" || encoding == "rgb8") {
    pixel_bytes = 3;
  } else if (encoding == "bgra8" || encoding == "rgba8") {
    pixel_bytes = 4;
  } else if (encoding == "mono8" || encoding == "8UC1") {
    pixel_bytes = 1;
  } else if (encoding == "16UC1" || encoding == "mono16") {
    pixel_bytes = 2;
  } else {
    return "unsupported image encoding '" + encoding + "'";
  }
  if (width == 0U || height == 0U) {
    return "empty image";
  }
  const std::size_t row_bytes = static_cast<std::size_t>(width) * pixel_bytes;
  if (step < row_bytes || size < static_cast<std::size_t>(step) * height) {
    return "image data is shorter than its step and height";
  }
  if (pixel_bytes == 2 && depth_max_mm <= 0.0) {
    return "depth images need a positive depth maximum";
  }
  const std::size_t base = out->size();
  out->resize(base + pipeFrameBytes(format, width, height));
  std::uint8_t* destination = out->data() + base;
  for (std::uint32_t row = 0; row < height; ++row) {
    const std::uint8_t* source = data + static_cast<std::size_t>(row) * step;
    for (std::uint32_t column = 0; column < width; ++column, source += pixel_bytes) {
      detail::Rgb value{};
      if (encoding == "bgr8" || encoding == "bgra8") {
        value = {source[2], source[1], source[0]};
      } else if (encoding == "rgb8" || encoding == "rgba8") {
        value = {source[0], source[1], source[2]};
      } else if (pixel_bytes == 1) {
        value = {source[0], source[0], source[0]};
      } else {
        const unsigned raw = big_endian ? (source[0] << 8U | source[1]) : (source[1] << 8U | source[0]);
        const double scaled = std::min(255.0, raw * 255.0 / depth_max_mm);
        const std::uint8_t level = static_cast<std::uint8_t>(scaled);
        value = {level, level, level};
      }
      switch (format) {
        case PipeFormat::Bgr8:
          *destination++ = value.b;
          *destination++ = value.g;
          *destination++ = value.r;
          break;
        case PipeFormat::Rgb8:
          *destination++ = value.r;
          *destination++ = value.g;
          *destination++ = value.b;
          break;
        case PipeFormat::Mono8:
          *destination++ = detail::luma(value);
          break;
      }
    }
  }
  return "";
}

}  // namespace xgc2_camera_driver
