#include "image_pipe.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace driver = xgc2_camera_driver;

namespace {

std::string convert(
    const std::string& encoding,
    const std::vector<std::uint8_t>& data,
    const std::uint32_t width,
    const std::uint32_t height,
    const std::uint32_t step,
    const driver::PipeFormat format,
    std::vector<std::uint8_t>* out,
    const bool big_endian = false,
    const double depth_max = 4000.0)
{
  return driver::convertImage(
      encoding, width, height, step, big_endian, data.data(), data.size(), format, depth_max, out);
}

}  // namespace

TEST(ImagePipe, PassesMatchingEncodingsThroughAndStripsRowPadding)
{
  // 2x2 bgr8 with 2 bytes of padding per row.
  const std::vector<std::uint8_t> padded = {1, 2, 3, 4, 5, 6, 99, 99, 7, 8, 9, 10, 11, 12, 99, 99};
  std::vector<std::uint8_t> out;
  ASSERT_EQ(convert("bgr8", padded, 2, 2, 8, driver::PipeFormat::Bgr8, &out), "");
  EXPECT_EQ(out, (std::vector<std::uint8_t>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}));
}

TEST(ImagePipe, SwapsChannelOrderBetweenRgbAndBgr)
{
  const std::vector<std::uint8_t> rgb = {10, 20, 30, 40, 50, 60};
  std::vector<std::uint8_t> out;
  ASSERT_EQ(convert("rgb8", rgb, 2, 1, 6, driver::PipeFormat::Bgr8, &out), "");
  EXPECT_EQ(out, (std::vector<std::uint8_t>{30, 20, 10, 60, 50, 40}));
  out.clear();
  ASSERT_EQ(convert("bgr8", rgb, 2, 1, 6, driver::PipeFormat::Rgb8, &out), "");
  EXPECT_EQ(out, (std::vector<std::uint8_t>{30, 20, 10, 60, 50, 40}));
}

TEST(ImagePipe, DropsTheAlphaChannel)
{
  const std::vector<std::uint8_t> rgba = {1, 2, 3, 255, 4, 5, 6, 0};
  std::vector<std::uint8_t> out;
  ASSERT_EQ(convert("rgba8", rgba, 2, 1, 8, driver::PipeFormat::Rgb8, &out), "");
  EXPECT_EQ(out, (std::vector<std::uint8_t>{1, 2, 3, 4, 5, 6}));
  out.clear();
  ASSERT_EQ(convert("bgra8", rgba, 2, 1, 8, driver::PipeFormat::Rgb8, &out), "");
  EXPECT_EQ(out, (std::vector<std::uint8_t>{3, 2, 1, 6, 5, 4}));
}

TEST(ImagePipe, ExpandsAndReducesGray)
{
  const std::vector<std::uint8_t> mono = {0, 128, 255};
  std::vector<std::uint8_t> out;
  ASSERT_EQ(convert("mono8", mono, 3, 1, 3, driver::PipeFormat::Bgr8, &out), "");
  EXPECT_EQ(out, (std::vector<std::uint8_t>{0, 0, 0, 128, 128, 128, 255, 255, 255}));
  out.clear();
  const std::vector<std::uint8_t> colour = {0, 0, 255, 255, 255, 255};  // red, white (bgr8)
  ASSERT_EQ(convert("bgr8", colour, 2, 1, 6, driver::PipeFormat::Mono8, &out), "");
  ASSERT_EQ(out.size(), 2U);
  EXPECT_EQ(out[0], 76);   // 0.299 * 255
  EXPECT_EQ(out[1], 255);
}

TEST(ImagePipe, ScalesDepthToEightBit)
{
  // Little endian 16 bit: 0 mm, 2000 mm, 9000 mm (beyond the 4000 mm range).
  const std::vector<std::uint8_t> depth = {0, 0, 0xd0, 0x07, 0x28, 0x23};
  std::vector<std::uint8_t> out;
  ASSERT_EQ(convert("16UC1", depth, 3, 1, 6, driver::PipeFormat::Mono8, &out), "");
  EXPECT_EQ(out, (std::vector<std::uint8_t>{0, 127, 255}));
  out.clear();
  const std::vector<std::uint8_t> big = {0x07, 0xd0};
  ASSERT_EQ(convert("16UC1", big, 1, 1, 2, driver::PipeFormat::Mono8, &out, true, 4000.0), "");
  EXPECT_EQ(out, (std::vector<std::uint8_t>{127}));
}

TEST(ImagePipe, RejectsWhatItCannotConvertWithoutTouchingTheOutput)
{
  std::vector<std::uint8_t> out = {42};
  const std::vector<std::uint8_t> data(12, 1);
  EXPECT_NE(convert("32FC1", data, 3, 1, 12, driver::PipeFormat::Bgr8, &out), "");
  EXPECT_NE(convert("bgr8", data, 0, 1, 12, driver::PipeFormat::Bgr8, &out), "");
  EXPECT_NE(convert("bgr8", data, 3, 1, 8, driver::PipeFormat::Bgr8, &out), "") << "step too small";
  EXPECT_NE(convert("bgr8", data, 3, 2, 9, driver::PipeFormat::Bgr8, &out), "") << "data too short";
  EXPECT_NE(convert("16UC1", data, 2, 1, 4, driver::PipeFormat::Mono8, &out, false, 0.0), "");
  EXPECT_EQ(out, (std::vector<std::uint8_t>{42}));
}

TEST(ImagePipe, FrameSizeFollowsTheFormat)
{
  EXPECT_EQ(driver::pipeFrameBytes(driver::PipeFormat::Bgr8, 640, 480), 640U * 480U * 3U);
  EXPECT_EQ(driver::pipeFrameBytes(driver::PipeFormat::Mono8, 640, 480), 640U * 480U);
  EXPECT_STREQ(driver::pipeFormatName(driver::PipeFormat::Rgb8), "rgb8");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
