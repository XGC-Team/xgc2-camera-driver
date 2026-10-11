#pragma once

#include <memory>

struct AVFrame;

namespace xgc2_camera_driver {

// A decoded picture shared by every encoder that consumes the same captured
// frame. Copies share the underlying FFmpeg frame, which is immutable once
// published.
class Picture {
 public:
  Picture() = default;
  explicit Picture(std::shared_ptr<const AVFrame> frame) : frame_(std::move(frame)) {}

  explicit operator bool() const noexcept { return static_cast<bool>(frame_); }
  const AVFrame* frame() const noexcept { return frame_.get(); }

 private:
  std::shared_ptr<const AVFrame> frame_;
};

}  // namespace xgc2_camera_driver
