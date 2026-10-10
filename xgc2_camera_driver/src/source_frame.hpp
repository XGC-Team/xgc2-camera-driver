#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <xgc2/camera/camera.hpp>

namespace xgc2_camera_driver {

// One captured picture as the node sees it, whatever produced it: a camera-core
// device, the synthetic generator or a still image. Copies are cheap; the
// payload stays valid for as long as any copy holds the lease.
struct SourceFrame {
  const std::uint8_t* data{nullptr};
  std::size_t size{0};
  // Per-plane layout for multi-plane formats; empty for packed frames.
  std::vector<xgc2::camera::Plane> planes;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t stride{0};
  xgc2::camera::PixelFormat pixel_format{xgc2::camera::PixelFormat::Unknown};
  std::uint64_t sequence{0};
  xgc2::camera::Timestamp timestamp;
  xgc2::camera::TimestampReference timestamp_reference{
      xgc2::camera::TimestampReference::Unknown};
  // Host CLOCK_MONOTONIC observation taken when the frame was dequeued.
  xgc2::camera::Timestamp dequeue_timestamp;
  std::shared_ptr<const void> lease;

  std::int64_t timestamp_ns() const noexcept
  {
    return timestamp.to_nanoseconds();
  }

  std::int64_t dequeue_timestamp_ns() const noexcept
  {
    return dequeue_timestamp.to_nanoseconds();
  }
};

// Shares the camera-core frame; no pixel data is copied.
inline SourceFrame sourceFrameFromCamera(const xgc2::camera::Frame& frame)
{
  SourceFrame result;
  result.data = frame.data();
  result.size = frame.size();
  result.planes = frame.planes();
  result.width = frame.width();
  result.height = frame.height();
  result.stride = frame.stride();
  result.pixel_format = frame.pixel_format();
  result.sequence = frame.sequence();
  result.timestamp = frame.timestamp();
  result.timestamp_reference = frame.timestamp_reference();
  result.dequeue_timestamp = frame.dequeue_timestamp();
  result.lease = std::make_shared<xgc2::camera::Frame>(frame);
  return result;
}

}  // namespace xgc2_camera_driver
