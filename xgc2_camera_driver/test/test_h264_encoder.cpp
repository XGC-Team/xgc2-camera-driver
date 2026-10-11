#include "h264_encoder.hpp"

#include <gtest/gtest.h>

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "h264_decoder.hpp"
#include "h264_test_support.hpp"
#include "jpeg_decoder.hpp"

namespace driver = xgc2_camera_driver;

namespace {

constexpr std::int64_t kFrameTicks = 3000;  // 30 fps at 90 kHz

// A textured test card, round-tripped through JPEG like a camera frame.
cv::Mat testCard(const int width, const int height, const int shift)
{
  cv::Mat card(height, width, CV_8UC3);
  for (int row = 0; row < height; ++row) {
    for (int column = 0; column < width; ++column) {
      card.at<cv::Vec3b>(row, column) = cv::Vec3b(
          static_cast<std::uint8_t>((column + shift) * 255 / width),
          static_cast<std::uint8_t>(row * 255 / height),
          static_cast<std::uint8_t>(((row / 16 + column / 16) % 2) * 160 + 40));
    }
  }
  return card;
}

// What a consumer of the camera frame sees: the decoded picture, plus the same
// JPEG decoded to BGR as the reference for video comparisons.
struct Card {
  driver::Picture picture;
  cv::Mat reference;
};

Card makeCard(const int width, const int height, driver::JpegDecoder* decoder)
{
  std::vector<std::uint8_t> jpeg;
  EXPECT_TRUE(cv::imencode(".jpg", testCard(width, height, 0), jpeg, {cv::IMWRITE_JPEG_QUALITY, 92}));
  return Card{decoder->decode(jpeg.data(), jpeg.size()), cv::imdecode(jpeg, cv::IMREAD_COLOR)};
}

driver::H264EncoderSettings settings(const int width = 320, const int height = 240)
{
  driver::H264EncoderSettings result;
  result.width = static_cast<std::uint32_t>(width);
  result.height = static_cast<std::uint32_t>(height);
  result.fps = 30.0;
  result.encoder = "libx264";
  result.bitrate = 4000000;
  result.gop = 5;
  return result;
}

}  // namespace

TEST(H264Encoder, ProducesADecodableStreamWithAPeriodicGop)
{
  driver::JpegDecoder jpeg;
  const Card card = makeCard(320, 240, &jpeg);
  auto encoder = driver::H264Encoder::open(settings());
  EXPECT_EQ(encoder->name(), "libx264");
  EXPECT_TRUE(encoder->fallback_reason().empty());

  h264_test::Decoder decoder;
  std::vector<driver::EncodedAccessUnit> units;
  for (int index = 0; index < 11; ++index) {
    encoder->encode(card.picture, index * kFrameTicks, false, &units);
  }
  ASSERT_EQ(units.size(), 11U);
  for (std::size_t index = 0; index < units.size(); ++index) {
    EXPECT_EQ(units[index].pts, static_cast<std::int64_t>(index) * kFrameTicks);
    const bool expect_idr = index % 5U == 0U;
    EXPECT_EQ(units[index].keyframe, expect_idr) << "frame " << index;
    EXPECT_EQ(h264_test::isIdr(units[index].data), expect_idr) << "frame " << index;
    const auto pictures = decoder.decode(units[index].data);
    ASSERT_EQ(pictures.size(), 1U) << "frame " << index;
    EXPECT_EQ(pictures[0].cols, 320);
    EXPECT_EQ(pictures[0].rows, 240);
    // The decoded video matches the camera frame within codec and range
    // conversion error.
    EXPECT_LT(h264_test::meanAbsoluteDifference(pictures[0], card.reference), 6.0)
        << "frame " << index;
  }
}

TEST(H264Encoder, ForcedKeyframeIsARealIdrWithParameterSets)
{
  driver::JpegDecoder jpeg;
  const driver::Picture picture = makeCard(320, 240, &jpeg).picture;
  auto encoder = driver::H264Encoder::open([] {
    auto value = settings();
    value.gop = 1000;
    return value;
  }());
  std::vector<driver::EncodedAccessUnit> units;
  for (int index = 0; index < 6; ++index) {
    encoder->encode(picture, index * kFrameTicks, index == 3, &units);
  }
  ASSERT_EQ(units.size(), 6U);
  for (std::size_t index = 0; index < units.size(); ++index) {
    const bool expect_idr = index == 0U || index == 3U;
    EXPECT_EQ(h264_test::isIdr(units[index].data), expect_idr) << "frame " << index;
    EXPECT_EQ(units[index].keyframe, expect_idr) << "frame " << index;
  }
  // The forced IDR is independently decodable: a fresh decoder starts on it.
  h264_test::Decoder late_joiner;
  EXPECT_EQ(late_joiner.decode(units[3].data).size(), 1U);
  EXPECT_EQ(late_joiner.decode(units[4].data).size(), 1U);
}

TEST(H264Encoder, ScalesToTheEncodedGeometry)
{
  driver::JpegDecoder jpeg;
  const driver::Picture picture = makeCard(320, 240, &jpeg).picture;
  auto encoder = driver::H264Encoder::open(settings(160, 120));
  std::vector<driver::EncodedAccessUnit> units;
  encoder->encode(picture, 0, false, &units);
  ASSERT_EQ(units.size(), 1U);
  h264_test::Decoder decoder;
  const auto pictures = decoder.decode(units[0].data);
  ASSERT_EQ(pictures.size(), 1U);
  EXPECT_EQ(pictures[0].cols, 160);
  EXPECT_EQ(pictures[0].rows, 120);
}

TEST(H264Encoder, ConstantQualityMakesAStaticPictureAlmostFree)
{
  driver::JpegDecoder jpeg;
  driver::H264Decoder reconstruction;
  const driver::Picture picture = makeCard(640, 360, &jpeg).picture;
  auto value = settings(640, 360);
  value.fps = 1.0;
  value.constant_quality = true;
  value.quality = 23;
  value.gop = 3600;

  // Feeding the original over and over makes the encoder keep refining the
  // quantisation error of the first picture.
  std::size_t refining = 0;
  {
    auto encoder = driver::H264Encoder::open(value);
    std::vector<driver::EncodedAccessUnit> units;
    for (int index = 0; index < 20; ++index) {
      encoder->encode(picture, index * 90000, false, &units);
    }
    for (std::size_t index = 1; index < units.size(); ++index) {
      refining += units[index].data.size();
    }
  }

  // Feeding the decoded IDR back makes every following picture a run of skips.
  auto encoder = driver::H264Encoder::open(value);
  std::vector<driver::EncodedAccessUnit> units;
  encoder->encode(picture, 0, false, &units);
  ASSERT_EQ(units.size(), 1U);
  EXPECT_TRUE(units[0].keyframe);
  const driver::Picture decoded = reconstruction.decode(units[0].data);
  ASSERT_TRUE(static_cast<bool>(decoded));
  for (int index = 1; index < 20; ++index) {
    encoder->encode(decoded, index * 90000, false, &units);
  }
  ASSERT_EQ(units.size(), 20U);
  std::size_t followers = 0;
  for (std::size_t index = 1; index < units.size(); ++index) {
    EXPECT_FALSE(units[index].keyframe);
    EXPECT_LT(units[index].data.size(), units[0].data.size() / 50U) << "frame " << index;
    followers += units[index].data.size();
  }
  std::cout << "idr=" << units[0].data.size() << " bytes, nineteen skip pictures=" << followers
            << " bytes, nineteen refining pictures=" << refining << " bytes" << std::endl;
  EXPECT_LT(followers * 10U, refining);
}

TEST(H264Encoder, HardwarePreferenceFallsBackToLibx264WhenUnusable)
{
  driver::JpegDecoder jpeg;
  const driver::Picture picture = makeCard(320, 240, &jpeg).picture;
  auto value = settings();
  value.encoder = "h264_nvenc";
  auto encoder = driver::H264Encoder::open(value);
  std::cout << "encoder in use: " << encoder->name()
            << (encoder->fallback_reason().empty()
                    ? std::string()
                    : " (" + encoder->fallback_reason() + ")")
            << std::endl;
  if (encoder->name() == "h264_nvenc") {
    EXPECT_TRUE(encoder->fallback_reason().empty());
  } else {
    // No usable NVENC device on this host: the software encoder serves the
    // session and says why.
    EXPECT_EQ(encoder->name(), "libx264");
    EXPECT_NE(encoder->fallback_reason().find("h264_nvenc"), std::string::npos);
  }
  std::vector<driver::EncodedAccessUnit> units;
  encoder->encode(picture, 0, false, &units);
  ASSERT_EQ(units.size(), 1U);
  EXPECT_TRUE(h264_test::isIdr(units[0].data));
}

TEST(H264Encoder, RejectsInvalidSettings)
{
  auto odd = settings();
  odd.width = 321;
  EXPECT_THROW(driver::H264Encoder::open(odd), std::invalid_argument);
  auto unknown = settings();
  unknown.encoder = "h265";
  EXPECT_THROW(driver::H264Encoder::open(unknown), std::invalid_argument);
  auto idle = settings();
  idle.gop = 0;
  EXPECT_THROW(driver::H264Encoder::open(idle), std::invalid_argument);
  auto slow = settings();
  slow.fps = 0.0;
  EXPECT_THROW(driver::H264Encoder::open(slow), std::invalid_argument);
}

TEST(JpegDecoder, DecodesAndRejectsCorruptInput)
{
  driver::JpegDecoder decoder;
  const cv::Mat card = testCard(64, 48, 0);
  std::vector<std::uint8_t> jpeg;
  ASSERT_TRUE(cv::imencode(".jpg", card, jpeg));
  const driver::Picture picture = decoder.decode(jpeg.data(), jpeg.size());
  ASSERT_TRUE(static_cast<bool>(picture));
  EXPECT_EQ(picture.frame()->width, 64);
  EXPECT_EQ(picture.frame()->height, 48);
  std::vector<std::uint8_t> garbage(100, 0x5a);
  EXPECT_THROW(decoder.decode(garbage.data(), garbage.size()), std::runtime_error);
  EXPECT_THROW(decoder.decode(nullptr, 0), std::runtime_error);
  // The decoder keeps working after a failure.
  EXPECT_TRUE(static_cast<bool>(decoder.decode(jpeg.data(), jpeg.size())));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
