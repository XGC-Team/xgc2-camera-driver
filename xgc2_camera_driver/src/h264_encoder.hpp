#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "picture.hpp"

struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;

namespace xgc2_camera_driver {

struct H264EncoderSettings {
  // Encoded geometry; pictures of any other size are scaled to it.
  std::uint32_t width{0};
  std::uint32_t height{0};
  // Nominal frame rate used by rate control. Presentation times are supplied
  // per picture, so a lower real rate (a still) does not need another setting.
  double fps{30.0};
  // "h264_nvenc" prefers the NVIDIA hardware encoder and falls back to
  // libx264 when it cannot be opened; "libx264" never tries the hardware.
  std::string encoder{"h264_nvenc"};
  // Target bits per second, or the ceiling when constant_quality is set.
  std::uint32_t bitrate{25000000};
  // Constant quality (CRF / CQ) with the bitrate as a VBV ceiling. A static
  // picture then costs bits only when it changes.
  bool constant_quality{false};
  int quality{23};
  // Frames between forced IDRs. A requested keyframe is always an IDR.
  std::uint32_t gop{30};
  // libx264 preset; empty selects "veryfast".
  std::string preset;
};

struct EncodedAccessUnit {
  // Annex-B access unit; SPS and PPS precede every IDR.
  std::vector<std::uint8_t> data;
  std::int64_t pts{0};
  bool keyframe{false};
};

// A low-latency H.264 encoder: no B-frames, no lookahead, one access unit per
// picture. Not thread safe; one thread owns an instance.
class H264Encoder {
 public:
  // Throws std::runtime_error when no encoder can be opened.
  static std::unique_ptr<H264Encoder> open(const H264EncoderSettings& settings);
  ~H264Encoder();
  H264Encoder(const H264Encoder&) = delete;
  H264Encoder& operator=(const H264Encoder&) = delete;

  // The encoder in use: "h264_nvenc" or "libx264".
  const std::string& name() const noexcept { return name_; }
  // Why the hardware encoder was not used; empty when it was, or not wanted.
  const std::string& fallback_reason() const noexcept { return fallback_reason_; }

  // Presentation times are in 90 kHz units and must increase. A forced IDR is
  // emitted for this picture. Appends the access units that become available.
  void encode(
      const Picture& picture,
      std::int64_t pts,
      bool force_idr,
      std::vector<EncodedAccessUnit>* out);

 private:
  H264Encoder() = default;
  bool tryOpen(
      const char* codec_name, const H264EncoderSettings& settings, std::string* error);
  void close() noexcept;

  AVCodecContext* context_{nullptr};
  AVFrame* converted_{nullptr};
  AVPacket* packet_{nullptr};
  SwsContext* scaler_{nullptr};
  // What the scaler was configured for, to reconfigure only when it changes.
  int scaler_width_{0};
  int scaler_height_{0};
  int scaler_format_{-1};
  bool scaler_full_range_{false};
  std::string name_;
  std::string fallback_reason_;
  std::uint32_t width_{0};
  std::uint32_t height_{0};
};

}  // namespace xgc2_camera_driver
