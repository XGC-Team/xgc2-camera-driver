#include "capture_source.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace xgc2_camera_driver {
namespace {

using Clock = std::chrono::steady_clock;

class V4l2Source final : public CaptureSource {
 public:
  explicit V4l2Source(const xgc2::camera::CaptureConfig& config)
      : camera_(xgc2::camera::make_camera(config))
  {
  }

  void start() override { camera_->start(); }

  SourceFrame read(const std::uint32_t timeout_ms) override
  {
    return sourceFrameFromCamera(camera_->read(timeout_ms));
  }

  void stop() noexcept override { camera_->stop(); }

  std::uint32_t width() const noexcept override
  {
    return camera_->config().width;
  }
  std::uint32_t height() const noexcept override
  {
    return camera_->config().height;
  }
  double frame_rate() const noexcept override
  {
    return camera_->config().frame_rate;
  }
  xgc2::camera::PixelFormat pixel_format() const noexcept override
  {
    return camera_->config().pixel_format;
  }

 private:
  std::unique_ptr<xgc2::camera::Camera> camera_;
};

std::int64_t steadyNanoseconds(const Clock::time_point point)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             point.time_since_epoch())
      .count();
}

xgc2::camera::Timestamp monotonicTimestamp(const std::int64_t nanoseconds)
{
  xgc2::camera::Timestamp result;
  result.seconds = nanoseconds / 1000000000LL;
  result.nanoseconds = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  result.clock = xgc2::camera::TimestampClock::Monotonic;
  return result;
}

// Renders one test frame: a smooth gradient, a few static discs, a bar that
// moves with the frame index and the index itself as text.
cv::Mat renderPattern(
    const SyntheticConfig& config, const std::uint32_t index, const cv::Mat& base)
{
  cv::Mat image = base.clone();
  const int width = static_cast<int>(config.width);
  const int height = static_cast<int>(config.height);
  const int bar_width = std::max(2, width / 16);
  const int travel = std::max(1, width - bar_width);
  const int x = static_cast<int>(
      (static_cast<std::uint64_t>(index) * static_cast<std::uint64_t>(width) / 24U) %
      static_cast<std::uint64_t>(travel));
  cv::rectangle(
      image,
      cv::Rect(x, height / 4, bar_width, std::max(2, height / 2)),
      cv::Scalar(240, 240, 240),
      cv::FILLED);
  cv::putText(
      image,
      "xgc2 synthetic " + std::to_string(index),
      cv::Point(width / 32, height / 8),
      cv::FONT_HERSHEY_SIMPLEX,
      std::max(0.4, height / 540.0),
      cv::Scalar(20, 20, 220),
      std::max(1, height / 270));
  if (config.noise_sigma > 0.0) {
    cv::Mat wide;
    image.convertTo(wide, CV_16SC3);
    cv::Mat noise(height, width, CV_16SC3);
    cv::RNG rng(config.seed * 2654435761U + index);
    rng.fill(noise, cv::RNG::NORMAL, 0.0, config.noise_sigma);
    wide += noise;
    wide.convertTo(image, CV_8UC3);
  }
  return image;
}

cv::Mat renderBase(const SyntheticConfig& config)
{
  const int width = static_cast<int>(config.width);
  const int height = static_cast<int>(config.height);
  cv::Mat base(height, width, CV_8UC3);
  for (int row = 0; row < height; ++row) {
    cv::Vec3b* pixels = base.ptr<cv::Vec3b>(row);
    const std::uint8_t green = static_cast<std::uint8_t>(row * 255 / std::max(1, height - 1));
    for (int column = 0; column < width; ++column) {
      pixels[column] = cv::Vec3b(
          static_cast<std::uint8_t>(column * 255 / std::max(1, width - 1)),
          green,
          static_cast<std::uint8_t>(96 + (config.seed % 64U)));
    }
  }
  for (int disc = 0; disc < 5; ++disc) {
    cv::circle(
        base,
        cv::Point(width * (disc + 1) / 6, height * (disc % 2 == 0 ? 3 : 2) / 4),
        std::max(2, height / 12),
        cv::Scalar(30 + disc * 40, 200 - disc * 30, 60 + disc * 20),
        cv::FILLED);
  }
  return base;
}

class SyntheticSource final : public CaptureSource {
 public:
  explicit SyntheticSource(const SyntheticConfig& config) : config_(config)
  {
    if (config.width < 16U || config.height < 16U || config.width > 16384U ||
        config.height > 16384U) {
      throw xgc2::camera::CameraError(
          xgc2::camera::ErrorCode::InvalidArgument,
          "synthetic width and height must be in [16, 16384]");
    }
    if (!(config.frame_rate > 0.0) || !std::isfinite(config.frame_rate)) {
      throw xgc2::camera::CameraError(
          xgc2::camera::ErrorCode::InvalidArgument,
          "synthetic frame rate must be positive");
    }
    if (config.jpeg_quality < 1 || config.jpeg_quality > 100 ||
        config.ring_frames == 0U || config.noise_sigma < 0.0) {
      throw xgc2::camera::CameraError(
          xgc2::camera::ErrorCode::InvalidArgument,
          "synthetic JPEG quality, ring size or noise is out of range");
    }
    const cv::Mat base = renderBase(config);
    const std::vector<int> parameters{cv::IMWRITE_JPEG_QUALITY, config.jpeg_quality};
    ring_.reserve(config.ring_frames);
    for (std::uint32_t index = 0; index < config.ring_frames; ++index) {
      auto encoded = std::make_shared<std::vector<std::uint8_t>>();
      if (!cv::imencode(".jpg", renderPattern(config, index, base), *encoded, parameters)) {
        throw xgc2::camera::CameraError(
            xgc2::camera::ErrorCode::SystemError, "synthetic JPEG encoding failed");
      }
      ring_.push_back(std::move(encoded));
    }
    period_ns_ = static_cast<std::int64_t>(std::llround(1e9 / config.frame_rate));
  }

  void start() override
  {
    if (running_) {
      throw xgc2::camera::CameraError(
          xgc2::camera::ErrorCode::InvalidState, "synthetic camera is already running");
    }
    sequence_ = 0U;
    source_start_ = Clock::now();
    next_frame_ = source_start_;
    running_ = true;
  }

  SourceFrame read(const std::uint32_t timeout_ms) override
  {
    if (!running_) {
      throw xgc2::camera::CameraError(
          xgc2::camera::ErrorCode::InvalidState,
          "synthetic camera must be started before read");
    }
    const auto now = Clock::now();
    if (now < next_frame_) {
      const auto deadline = now + std::chrono::milliseconds(timeout_ms);
      if (deadline < next_frame_) {
        std::this_thread::sleep_until(deadline);
        throw xgc2::camera::CameraError(
            xgc2::camera::ErrorCode::Timeout,
            "timed out waiting for a synthetic capture frame");
      }
      std::this_thread::sleep_until(next_frame_);
    }
    const auto& jpeg = ring_[sequence_ % ring_.size()];
    SourceFrame frame;
    frame.data = jpeg->data();
    frame.size = jpeg->size();
    frame.width = config_.width;
    frame.height = config_.height;
    frame.pixel_format = xgc2::camera::PixelFormat::MJPEG;
    frame.sequence = sequence_;
    frame.timestamp = monotonicTimestamp(
        steadyNanoseconds(source_start_) +
        static_cast<std::int64_t>(sequence_) * period_ns_);
    frame.timestamp_reference = xgc2::camera::TimestampReference::StartOfExposure;
    frame.dequeue_timestamp = monotonicTimestamp(steadyNanoseconds(Clock::now()));
    frame.lease = jpeg;
    ++sequence_;
    next_frame_ = std::max(next_frame_ + std::chrono::nanoseconds(period_ns_), Clock::now());
    return frame;
  }

  void stop() noexcept override { running_ = false; }

  std::uint32_t width() const noexcept override { return config_.width; }
  std::uint32_t height() const noexcept override { return config_.height; }
  double frame_rate() const noexcept override { return config_.frame_rate; }
  xgc2::camera::PixelFormat pixel_format() const noexcept override
  {
    return xgc2::camera::PixelFormat::MJPEG;
  }

 private:
  SyntheticConfig config_;
  std::vector<std::shared_ptr<const std::vector<std::uint8_t>>> ring_;
  std::int64_t period_ns_{0};
  bool running_{false};
  std::uint64_t sequence_{0};
  Clock::time_point source_start_;
  Clock::time_point next_frame_;
};

}  // namespace

std::unique_ptr<CaptureSource> makeV4l2Source(
    const xgc2::camera::CaptureConfig& config)
{
  return std::unique_ptr<CaptureSource>(new V4l2Source(config));
}

std::unique_ptr<CaptureSource> makeSyntheticSource(const SyntheticConfig& config)
{
  return std::unique_ptr<CaptureSource>(new SyntheticSource(config));
}

}  // namespace xgc2_camera_driver
