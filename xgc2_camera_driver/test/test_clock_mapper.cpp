#include "clock_mapper.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace camera = xgc2::camera;
namespace timing = xgc2_camera_driver::timing;

TEST(ClockMapper, MapsMonotonicUsingMeasuredClockPair)
{
  camera::Timestamp source;
  source.seconds = 5;
  source.nanoseconds = 25;
  source.clock = camera::TimestampClock::Monotonic;
  timing::HostClockSample clocks;
  clocks.realtime_ns = 10000000025LL;
  clocks.monotonic_ns = 4000000025LL;
  clocks.uncertainty_ns = 17;
  clocks.valid = true;

  const auto result =
      timing::mapSourceTimestamp(source, timing::UnknownClockPolicy::Reject, clocks);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.source_ns, 5000000025LL);
  EXPECT_EQ(result.source_to_ros_offset_ns, 6000000000LL);
  EXPECT_EQ(result.ros_time_ns, 11000000025LL);
  EXPECT_EQ(result.uncertainty_ns, 17U);
}

TEST(ClockMapper, NeverGuessesAnUnknownClock)
{
  camera::Timestamp source;
  source.seconds = 5;
  source.clock = camera::TimestampClock::Unknown;
  timing::HostClockSample clocks;
  clocks.realtime_ns = 10;
  clocks.monotonic_ns = 5;
  clocks.valid = true;
  EXPECT_FALSE(timing::mapSourceTimestamp(
                   source, timing::UnknownClockPolicy::Reject, clocks)
                   .valid);
  const auto explicit_mapping = timing::mapSourceTimestamp(
      source, timing::UnknownClockPolicy::AssumeMonotonic, clocks);
  EXPECT_TRUE(explicit_mapping.valid);
  EXPECT_EQ(explicit_mapping.source_to_ros_offset_ns, 5);
}

TEST(ClockMapper, MapsRealtimeWithoutAHostClockSample)
{
  camera::Timestamp source;
  source.seconds = 1700000000;
  source.nanoseconds = 123;
  source.clock = camera::TimestampClock::Realtime;
  const auto result = timing::mapSourceTimestamp(
      source, timing::UnknownClockPolicy::Reject, timing::HostClockSample{});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.ros_time_ns, 1700000000000000123LL);
  EXPECT_EQ(result.source_to_ros_offset_ns, 0);
}

TEST(ClockMapper, RejectsTimesOutsideRos1Representation)
{
  camera::Timestamp negative;
  negative.seconds = -1;
  negative.clock = camera::TimestampClock::Realtime;
  EXPECT_FALSE(timing::mapSourceTimestamp(
                   negative,
                   timing::UnknownClockPolicy::Reject,
                   timing::HostClockSample{})
                   .valid);

  camera::Timestamp after_ros1_limit;
  after_ros1_limit.seconds =
      static_cast<std::int64_t>(
          std::numeric_limits<std::uint32_t>::max()) +
      1;
  after_ros1_limit.clock = camera::TimestampClock::Realtime;
  EXPECT_FALSE(timing::mapSourceTimestamp(
                   after_ros1_limit,
                   timing::UnknownClockPolicy::Reject,
                   timing::HostClockSample{})
                   .valid);

  camera::Timestamp overflowing_native;
  overflowing_native.seconds =
      std::numeric_limits<std::int64_t>::max();
  overflowing_native.nanoseconds = 999999999U;
  overflowing_native.clock = camera::TimestampClock::Monotonic;
  EXPECT_FALSE(timing::mapSourceTimestamp(
                   overflowing_native,
                   timing::UnknownClockPolicy::Reject,
                   timing::HostClockSample{})
                   .valid);

  camera::Timestamp maximum;
  maximum.seconds = std::numeric_limits<std::uint32_t>::max();
  maximum.nanoseconds = 999999999U;
  maximum.clock = camera::TimestampClock::Realtime;
  const auto maximum_result = timing::mapSourceTimestamp(
      maximum,
      timing::UnknownClockPolicy::Reject,
      timing::HostClockSample{});
  EXPECT_TRUE(maximum_result.valid);
  EXPECT_EQ(maximum_result.ros_time_ns, 4294967295999999999LL);
}

TEST(ClockMapper, RejectsOverflowingClockOffsets)
{
  camera::Timestamp source;
  source.seconds = 1;
  source.clock = camera::TimestampClock::Monotonic;
  timing::HostClockSample clocks;
  clocks.realtime_ns = std::numeric_limits<std::int64_t>::max();
  clocks.monotonic_ns = -1;
  clocks.valid = true;
  EXPECT_FALSE(timing::mapSourceTimestamp(
                   source, timing::UnknownClockPolicy::Reject, clocks)
                   .valid);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
