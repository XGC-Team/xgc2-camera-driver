#include "clock_mapper.hpp"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstring>
#include <stdexcept>
#include <time.h>

namespace xgc2_camera_driver {
namespace timing {
namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr std::int64_t kMaximumRosTimeNs =
    static_cast<std::int64_t>(UINT32_MAX) * kNanosecondsPerSecond +
    (kNanosecondsPerSecond - 1);

bool clockNanoseconds(clockid_t clock, std::int64_t* output) noexcept {
  timespec value;
  std::memset(&value, 0, sizeof(value));
  if (::clock_gettime(clock, &value) != 0 || value.tv_sec < 0 ||
      value.tv_nsec < 0 || value.tv_nsec >= kNanosecondsPerSecond) {
    return false;
  }
  if (value.tv_sec > INT64_MAX / kNanosecondsPerSecond) {
    return false;
  }
  *output = static_cast<std::int64_t>(value.tv_sec) *
                kNanosecondsPerSecond +
            static_cast<std::int64_t>(value.tv_nsec);
  return true;
}

bool addWithoutOverflow(
    std::int64_t left,
    std::int64_t right,
    std::int64_t* output) noexcept {
  if ((right > 0 && left > INT64_MAX - right) ||
      (right < 0 && left < INT64_MIN - right)) {
    return false;
  }
  *output = left + right;
  return true;
}

bool subtractWithoutOverflow(
    std::int64_t left,
    std::int64_t right,
    std::int64_t* output) noexcept {
  if ((right > 0 && left < INT64_MIN + right) ||
      (right < 0 && left > INT64_MAX + right)) {
    return false;
  }
  *output = left - right;
  return true;
}

bool isRepresentableRosTime(const std::int64_t value) noexcept {
  return value >= 0 && value <= kMaximumRosTimeNs;
}

bool sourceNanoseconds(
    const xgc2::camera::Timestamp& source,
    std::int64_t* output) noexcept {
  if (source.seconds < 0 ||
      source.nanoseconds >= static_cast<std::uint32_t>(
                                kNanosecondsPerSecond) ||
      source.seconds >
          (INT64_MAX - static_cast<std::int64_t>(source.nanoseconds)) /
              kNanosecondsPerSecond) {
    return false;
  }
  *output =
      source.seconds * kNanosecondsPerSecond + source.nanoseconds;
  return true;
}

}  // namespace

HostClockSample sampleHostClocks() noexcept {
  // Sandwich CLOCK_MONOTONIC between two realtime reads. The midpoint bounds
  // the cross-clock sampling error without assuming the two syscalls are
  // simultaneous.
  std::int64_t realtime_before = 0;
  std::int64_t monotonic = 0;
  std::int64_t realtime_after = 0;
  HostClockSample result;
  if (!clockNanoseconds(CLOCK_REALTIME, &realtime_before) ||
      !clockNanoseconds(CLOCK_MONOTONIC, &monotonic) ||
      !clockNanoseconds(CLOCK_REALTIME, &realtime_after) ||
      realtime_after < realtime_before) {
    return result;
  }
  const auto span =
      static_cast<std::uint64_t>(realtime_after - realtime_before);
  result.realtime_ns =
      realtime_before + static_cast<std::int64_t>(span / 2U);
  result.monotonic_ns = monotonic;
  result.uncertainty_ns = span / 2U + 1U;
  result.valid = true;
  return result;
}

Mapping mapSourceTimestamp(
    const xgc2::camera::Timestamp& source,
    UnknownClockPolicy unknown_policy,
    const HostClockSample& clocks) noexcept {
  Mapping result;
  result.effective_clock = source.clock;
  if (!sourceNanoseconds(source, &result.source_ns)) {
    return result;
  }
  if (result.effective_clock == xgc2::camera::TimestampClock::Unknown) {
    switch (unknown_policy) {
      case UnknownClockPolicy::Reject:
        return result;
      case UnknownClockPolicy::AssumeRealtime:
        result.effective_clock = xgc2::camera::TimestampClock::Realtime;
        break;
      case UnknownClockPolicy::AssumeMonotonic:
        result.effective_clock = xgc2::camera::TimestampClock::Monotonic;
        break;
    }
  }
  if (result.effective_clock == xgc2::camera::TimestampClock::Realtime) {
    result.ros_time_ns = result.source_ns;
    result.valid = isRepresentableRosTime(result.ros_time_ns);
    return result;
  }
  if (result.effective_clock != xgc2::camera::TimestampClock::Monotonic ||
      !clocks.valid) {
    return result;
  }
  if (!subtractWithoutOverflow(
          clocks.realtime_ns,
          clocks.monotonic_ns,
          &result.source_to_ros_offset_ns) ||
      !addWithoutOverflow(
          result.source_ns,
          result.source_to_ros_offset_ns,
          &result.ros_time_ns) ||
      !isRepresentableRosTime(result.ros_time_ns)) {
    return Mapping{};
  }
  result.uncertainty_ns = clocks.uncertainty_ns;
  result.valid = true;
  return result;
}

UnknownClockPolicy unknownClockPolicyFromString(const std::string& value) {
  if (value == "reject") {
    return UnknownClockPolicy::Reject;
  }
  if (value == "assume_realtime") {
    return UnknownClockPolicy::AssumeRealtime;
  }
  if (value == "assume_monotonic") {
    return UnknownClockPolicy::AssumeMonotonic;
  }
  throw std::invalid_argument(
      "unknown_timestamp_clock must be reject, assume_realtime, or "
      "assume_monotonic");
}

const char* toString(UnknownClockPolicy value) noexcept {
  switch (value) {
    case UnknownClockPolicy::Reject:
      return "reject";
    case UnknownClockPolicy::AssumeRealtime:
      return "assume_realtime";
    case UnknownClockPolicy::AssumeMonotonic:
      return "assume_monotonic";
  }
  return "reject";
}

}  // namespace timing
}  // namespace xgc2_camera_driver
