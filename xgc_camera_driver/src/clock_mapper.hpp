#pragma once

#include <cstdint>
#include <string>

#include <xgc2/camera/camera.hpp>

namespace xgc_camera_driver {
namespace timing {

enum class UnknownClockPolicy {
  Reject,
  AssumeRealtime,
  AssumeMonotonic,
};

struct HostClockSample {
  std::int64_t realtime_ns{0};
  std::int64_t monotonic_ns{0};
  std::uint64_t uncertainty_ns{0};
  bool valid{false};
};

struct Mapping {
  bool valid{false};
  std::int64_t source_ns{0};
  std::int64_t ros_time_ns{0};
  std::int64_t source_to_ros_offset_ns{0};
  std::uint64_t uncertainty_ns{0};
  xgc2::camera::TimestampClock effective_clock{
      xgc2::camera::TimestampClock::Unknown};
};

HostClockSample sampleHostClocks() noexcept;

Mapping mapSourceTimestamp(
    const xgc2::camera::Timestamp& source,
    UnknownClockPolicy unknown_policy,
    const HostClockSample& clocks) noexcept;

UnknownClockPolicy unknownClockPolicyFromString(const std::string& value);
const char* toString(UnknownClockPolicy value) noexcept;

}  // namespace timing
}  // namespace xgc_camera_driver
