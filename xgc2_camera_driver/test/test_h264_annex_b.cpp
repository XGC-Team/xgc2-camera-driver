#include "h264_annex_b.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace h264 = xgc2_camera_driver::h264;

namespace {

const std::vector<std::uint8_t> kSPS = {0, 0, 0, 1, 0x67, 0x42, 0, 0x1f};
const std::vector<std::uint8_t> kPPS = {0, 0, 1, 0x68, 0xce, 0x06};
const std::vector<std::uint8_t> kIDR = {0, 0, 1, 0x65, 0x88, 0x84};
const std::vector<std::uint8_t> kPFrame = {0, 0, 1, 0x41, 0x9a, 0x22};

std::vector<std::uint8_t> joined(
    std::initializer_list<std::vector<std::uint8_t>> units)
{
  std::vector<std::uint8_t> result;
  for (const auto& unit : units) {
    result.insert(result.end(), unit.begin(), unit.end());
  }
  return result;
}

}  // namespace

TEST(H264AnnexB, InspectsAccessUnitNALTypes)
{
  const auto input = joined({kSPS, kPPS, kIDR});
  const auto info = h264::inspectAccessUnit(input.data(), input.size());
  EXPECT_TRUE(info.annex_b);
  EXPECT_TRUE(info.has_vcl);
  EXPECT_TRUE(info.keyframe);
  EXPECT_TRUE(info.has_sps);
  EXPECT_TRUE(info.has_pps);
}

TEST(H264AnnexB, WaitsForIDRAndRepeatsParameterSets)
{
  h264::AccessUnitGate gate;
  std::vector<std::uint8_t> output;
  h264::AccessUnitInfo info;
  EXPECT_FALSE(gate.prepare(kPFrame.data(), kPFrame.size(), &output, &info));

  const auto bootstrap = joined({kSPS, kPPS, kIDR});
  ASSERT_TRUE(gate.prepare(bootstrap.data(), bootstrap.size(), &output, &info));
  EXPECT_EQ(output, bootstrap);

  ASSERT_TRUE(gate.prepare(kPFrame.data(), kPFrame.size(), &output, &info));
  EXPECT_EQ(output, kPFrame);

  gate.discontinuity();
  EXPECT_FALSE(gate.prepare(kPFrame.data(), kPFrame.size(), &output, &info));
  ASSERT_TRUE(gate.prepare(kIDR.data(), kIDR.size(), &output, &info));
  EXPECT_EQ(output, joined({kSPS, kPPS, kIDR}));
  EXPECT_TRUE(info.has_sps);
  EXPECT_TRUE(info.has_pps);
}

TEST(H264AnnexB, RejectsNonAnnexBInput)
{
  const std::uint8_t avcc[] = {0, 0, 0, 2, 0x65, 0x88};
  h264::AccessUnitGate gate;
  std::vector<std::uint8_t> output;
  h264::AccessUnitInfo info;
  EXPECT_FALSE(gate.prepare(avcc, sizeof(avcc), &output, &info));
  EXPECT_FALSE(info.annex_b);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
