#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace xgc2_camera_driver {

// What the camera-source service tells media-edge about the picture source.
struct SourceInfo {
  std::string source_kind;  // v4l2, still or synthetic
  std::string frame_id;
  std::string ros_master_uri;
  std::string ros_master_run_id;
  std::uint32_t width{0};
  std::uint32_t height{0};
  // Nominal cadence; for a still the rate at which the stream is refreshed.
  double fps{0.0};
  // Source time domain of CaptureFrame::timestamp_ns: simulation,
  // system_realtime, monotonic, device or unknown.
  std::string clock_domain{"system_realtime"};
  // Real calibration only. An uncalibrated camera reports none; the service
  // never invents intrinsics to satisfy a consumer.
  bool calibrated{false};
  std::array<double, 9> camera_matrix{};
  std::vector<double> distortion;
};

// One immutable captured picture: the JPEG as the camera delivered it.
struct CaptureFrame {
  std::shared_ptr<const std::vector<std::uint8_t>> jpeg;
  // Positive; strictly increasing for live sources.
  std::uint64_t sequence{0};
  std::int64_t timestamp_ns{0};
  std::uint32_t width{0};
  std::uint32_t height{0};
};

// Shared between the capture thread, which feeds it, and the service, which
// answers describe, status and capture from it. Thread safe.
class SourceState {
 public:
  explicit SourceState(SourceInfo nominal);

  // The source is open: actual geometry and calibration replace the nominal.
  void sourceOpened(const SourceInfo& info);
  // The source is lost; not ready until it reopens and delivers a frame.
  void sourceClosed(const std::string& reason);
  void framePublished(std::shared_ptr<const CaptureFrame> frame);

  // Called after every change that can alter readiness or the latest frame.
  void setChangeHandler(std::function<void()> handler);

  SourceInfo info() const;
  // Open and at least one frame captured since it opened.
  bool ready() const;
  bool open() const;
  std::string reason() const;
  std::shared_ptr<const CaptureFrame> latest() const;
  // Counts every published frame, across reopenings.
  std::uint64_t frameCount() const;
  // Time since the last published frame; max() if none yet.
  std::chrono::milliseconds frameAge() const;

 private:
  void notify();

  mutable std::mutex mutex_;
  SourceInfo info_;
  bool open_{false};
  bool have_frame_{false};
  std::string reason_{"the source has not been opened"};
  std::shared_ptr<const CaptureFrame> latest_;
  std::uint64_t frame_count_{0};
  std::chrono::steady_clock::time_point last_frame_;
  std::function<void()> on_change_;
};

}  // namespace xgc2_camera_driver
