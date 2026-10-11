#pragma once

#include <cstdint>
#include <vector>

#include "picture.hpp"

struct AVCodecContext;
struct AVPacket;

namespace xgc2_camera_driver {

// Decodes whole H.264 access units. The media pipeline uses it to obtain the
// exact reconstruction of an IDR it just produced: feeding an encoder its own
// reconstruction makes every following picture of a still scene a run of skip
// macroblocks instead of a quantisation-noise refinement.
class H264Decoder {
 public:
  H264Decoder();
  ~H264Decoder();
  H264Decoder(const H264Decoder&) = delete;
  H264Decoder& operator=(const H264Decoder&) = delete;

  // Returns the picture the access unit completes. Throws std::runtime_error
  // when it cannot be decoded.
  Picture decode(const std::vector<std::uint8_t>& access_unit);

 private:
  AVCodecContext* context_{nullptr};
  AVPacket* packet_{nullptr};
};

}  // namespace xgc2_camera_driver
