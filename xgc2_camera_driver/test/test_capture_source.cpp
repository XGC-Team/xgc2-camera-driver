#include "capture_source.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

namespace camera = xgc2::camera;
namespace driver = xgc2_camera_driver;

namespace {

driver::SyntheticConfig smallConfig()
{
  driver::SyntheticConfig config;
  config.width = 64;
  config.height = 48;
  config.frame_rate = 200.0;
  config.ring_frames = 4;
  return config;
}

std::vector<std::uint8_t> bytes(const driver::SourceFrame& frame)
{
  return std::vector<std::uint8_t>(frame.data, frame.data + frame.size);
}

}  // namespace

TEST(SyntheticSource, DeliversRealMjpegAtTheConfiguredGeometry)
{
  auto source = driver::makeSyntheticSource(smallConfig());
  EXPECT_EQ(source->width(), 64U);
  EXPECT_EQ(source->height(), 48U);
  EXPECT_EQ(source->pixel_format(), camera::PixelFormat::MJPEG);
  source->start();
  for (std::uint64_t expected = 0; expected < 6U; ++expected) {
    const driver::SourceFrame frame = source->read(1000);
    EXPECT_EQ(frame.sequence, expected);
    EXPECT_EQ(frame.pixel_format, camera::PixelFormat::MJPEG);
    EXPECT_EQ(frame.width, 64U);
    EXPECT_EQ(frame.height, 48U);
    EXPECT_EQ(frame.timestamp.clock, camera::TimestampClock::Monotonic);
    ASSERT_GT(frame.size, 4U);
    EXPECT_EQ(frame.data[0], 0xffU);
    EXPECT_EQ(frame.data[1], 0xd8U);
    const cv::Mat decoded = cv::imdecode(bytes(frame), cv::IMREAD_COLOR);
    ASSERT_FALSE(decoded.empty());
    EXPECT_EQ(decoded.cols, 64);
    EXPECT_EQ(decoded.rows, 48);
  }
  source->stop();
}

TEST(SyntheticSource, TimestampsAdvanceByTheFramePeriod)
{
  auto source = driver::makeSyntheticSource(smallConfig());
  source->start();
  const driver::SourceFrame first = source->read(1000);
  const driver::SourceFrame second = source->read(1000);
  const std::int64_t step = second.timestamp_ns() - first.timestamp_ns();
  EXPECT_EQ(step, 5000000);  // 1 / 200 Hz
  EXPECT_GT(second.dequeue_timestamp_ns(), first.dequeue_timestamp_ns());
}

TEST(SyntheticSource, FramesCycleThroughTheRingDeterministically)
{
  auto first = driver::makeSyntheticSource(smallConfig());
  auto second = driver::makeSyntheticSource(smallConfig());
  first->start();
  second->start();
  std::vector<std::vector<std::uint8_t>> frames;
  for (int index = 0; index < 5; ++index) {
    frames.push_back(bytes(first->read(1000)));
    EXPECT_EQ(frames.back(), bytes(second->read(1000))) << "frame " << index;
  }
  EXPECT_NE(frames[0], frames[1]);
  EXPECT_EQ(frames[0], frames[4]);  // the ring holds four frames

  driver::SyntheticConfig other = smallConfig();
  other.seed = 7;
  auto reseeded = driver::makeSyntheticSource(other);
  reseeded->start();
  EXPECT_NE(frames[0], bytes(reseeded->read(1000)));
}

TEST(SyntheticSource, ReadTimesOutWhenTheNextFrameIsNotDue)
{
  driver::SyntheticConfig config = smallConfig();
  config.frame_rate = 0.5;
  auto source = driver::makeSyntheticSource(config);
  source->start();
  (void)source->read(1000);
  try {
    (void)source->read(20);
    FAIL() << "expected a timeout";
  } catch (const camera::CameraError& error) {
    EXPECT_EQ(error.code(), camera::ErrorCode::Timeout);
  }
}

TEST(SyntheticSource, RejectsInvalidConfigurationAndMisuse)
{
  driver::SyntheticConfig config = smallConfig();
  config.width = 4;
  EXPECT_THROW(driver::makeSyntheticSource(config), camera::CameraError);
  config = smallConfig();
  config.frame_rate = 0.0;
  EXPECT_THROW(driver::makeSyntheticSource(config), camera::CameraError);
  config = smallConfig();
  config.jpeg_quality = 101;
  EXPECT_THROW(driver::makeSyntheticSource(config), camera::CameraError);
  auto source = driver::makeSyntheticSource(smallConfig());
  EXPECT_THROW(source->read(10), camera::CameraError);  // not started
  source->start();
  EXPECT_THROW(source->start(), camera::CameraError);   // already running
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
