#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace xgc_camera_driver {
namespace h264 {

struct AccessUnitInfo {
  bool annex_b{false};
  bool has_vcl{false};
  bool keyframe{false};
  bool has_sps{false};
  bool has_pps{false};
};

AccessUnitInfo inspectAccessUnit(
    const std::uint8_t* data,
    std::size_t size) noexcept;

// Maintains the decoder bootstrap state for an Annex-B stream. After startup
// or a discontinuity, P frames are rejected until an IDR is available. Cached
// SPS/PPS NAL units are prepended when a camera omits them from an IDR access
// unit, as required by foxglove_msgs/CompressedVideo.
class AccessUnitGate {
 public:
  bool prepare(
      const std::uint8_t* data,
      std::size_t size,
      std::vector<std::uint8_t>* output,
      AccessUnitInfo* info);

  void discontinuity() noexcept { waiting_for_idr_ = true; }
  bool waitingForIDR() const noexcept { return waiting_for_idr_; }

 private:
  std::vector<std::uint8_t> sps_;
  std::vector<std::uint8_t> pps_;
  bool waiting_for_idr_{true};
};

}  // namespace h264
}  // namespace xgc_camera_driver
