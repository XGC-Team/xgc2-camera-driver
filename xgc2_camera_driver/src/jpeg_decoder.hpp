#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "picture.hpp"

struct AVCodecContext;
struct AVPacket;

namespace xgc2_camera_driver {

// Decodes complete JPEG images with FFmpeg's MJPEG decoder. The result keeps the
// JPEG's native planar YUV layout, so the H.264 encoders convert it once to
// their own geometry instead of going through an RGB round trip.
class JpegDecoder {
 public:
  JpegDecoder();
  ~JpegDecoder();
  JpegDecoder(const JpegDecoder&) = delete;
  JpegDecoder& operator=(const JpegDecoder&) = delete;

  // Throws std::runtime_error for a corrupt or truncated image.
  Picture decode(const std::uint8_t* data, std::size_t size);

 private:
  AVCodecContext* context_{nullptr};
  AVPacket* packet_{nullptr};
  // FFmpeg may read past the end of a packet; V4L2 buffers promise no padding.
  std::vector<std::uint8_t> padded_;
};

}  // namespace xgc2_camera_driver
