#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <xgc2/camera/camera.hpp>

#include "source_frame.hpp"

namespace xgc2_camera_driver {

// A pull source of captured frames. read() throws xgc2::camera::CameraError;
// ErrorCode::Timeout means no frame arrived in time and the call may be
// repeated, any other code means the source is lost.
class CaptureSource {
 public:
  virtual ~CaptureSource() = default;

  virtual void start() = 0;
  virtual SourceFrame read(std::uint32_t timeout_ms) = 0;
  virtual void stop() noexcept = 0;

  // The geometry and pixel format the source actually delivers.
  virtual std::uint32_t width() const noexcept = 0;
  virtual std::uint32_t height() const noexcept = 0;
  virtual double frame_rate() const noexcept = 0;
  virtual xgc2::camera::PixelFormat pixel_format() const noexcept = 0;
};

// A V4L2 device opened through camera-core.
std::unique_ptr<CaptureSource> makeV4l2Source(
    const xgc2::camera::CaptureConfig& config);

struct SyntheticConfig {
  std::uint32_t width{640};
  std::uint32_t height{480};
  double frame_rate{30.0};
  std::uint32_t seed{1};
  int jpeg_quality{90};
  // Sensor-like noise added to every frame; it makes the JPEG size and the
  // H.264 cost resemble a real camera instead of a flat test card.
  double noise_sigma{3.0};
  // The generator renders this many distinct frames once and cycles them, so a
  // 4K source can run at full rate on a busy machine.
  std::uint32_t ring_frames{30};
};

// Deterministic test camera: moving test pattern delivered as real MJPEG at
// the configured rate with monotonic source timestamps.
std::unique_ptr<CaptureSource> makeSyntheticSource(const SyntheticConfig& config);

}  // namespace xgc2_camera_driver
