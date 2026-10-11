#include "still_asset.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include <unistd.h>

#include <opencv2/imgcodecs.hpp>

namespace driver = xgc2_camera_driver;

namespace {

// One private directory per test run, removed when the run ends.
std::string testDirectory()
{
  static const std::string directory = [] {
    std::string pattern = testing::TempDir() + "xgc2_still_asset_XXXXXX";
    if (::mkdtemp(&pattern[0]) == nullptr) {
      throw std::runtime_error("cannot create a test directory");
    }
    return pattern;
  }();
  return directory;
}

std::vector<std::string>& createdFiles()
{
  static std::vector<std::string> files;
  return files;
}

std::string temporaryPath(const std::string& name)
{
  const std::string path = testDirectory() + "/" + name;
  createdFiles().push_back(path);
  return path;
}

class RemoveTestDirectory : public ::testing::Environment {
 public:
  void TearDown() override
  {
    for (const std::string& path : createdFiles()) {
      std::remove(path.c_str());
    }
    ::rmdir(testDirectory().c_str());
  }
};

std::string writeFile(const std::string& name, const std::string& content)
{
  const std::string path = temporaryPath(name);
  std::ofstream stream(path, std::ios::binary);
  stream << content;
  return path;
}

const char* const kIntrinsic =
    "schema: xgc2.camera.intrinsic.v1\n"
    "image_width: 64\n"
    "image_height: 48\n"
    "camera_matrix:\n"
    "  rows: 3\n"
    "  cols: 3\n"
    "  data: [50.0, 0.0, 31.5, 0.0, 51.0, 23.5, 0.0, 0.0, 1.0]\n"
    "distortion_model: plumb_bob\n"
    "distortion_coefficients:\n"
    "  rows: 1\n"
    "  cols: 5\n"
    "  data: [0.1, -0.05, 0.001, 0.002, 0.0]\n";

const char* const kExtrinsic =
    "schema: xgc2.camera.extrinsic.v1\n"
    "frame_convention: parent_T_camera_optical\n"
    "parent_frame: world\n"
    "child_frame: usb_cam_optical_frame\n"
    "translation: {x: 1.0, y: 2.0, z: 3.0}\n"
    "quaternion_xyzw: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}\n";

std::string replaceFirst(std::string text, const std::string& from, const std::string& to)
{
  const std::size_t at = text.find(from);
  EXPECT_NE(at, std::string::npos) << from;
  if (at != std::string::npos) {
    text.replace(at, from.size(), to);
  }
  return text;
}

// q and -q are the same rotation.
void expectSameRotation(const driver::Quaternion& actual, const driver::Quaternion& expected)
{
  double dot = 0.0;
  for (std::size_t index = 0; index < 4U; ++index) {
    dot += actual[index] * expected[index];
  }
  const double sign = dot < 0.0 ? -1.0 : 1.0;
  for (std::size_t index = 0; index < 4U; ++index) {
    EXPECT_NEAR(actual[index], sign * expected[index], 1e-12) << "component " << index;
  }
}

}  // namespace

TEST(StillAssetMath, IdentityExtrinsicRecoversTheInverseOpticalRotation)
{
  driver::Extrinsic extrinsic;
  extrinsic.parent_frame = "world";
  extrinsic.child_frame = "usb_cam_optical_frame";
  extrinsic.translation = {{1.0, 2.0, 3.0}};
  extrinsic.quaternion = {{0.0, 0.0, 0.0, 1.0}};
  const driver::CameraStaticTransforms chain = driver::composeStaticCameraTransforms(extrinsic);
  EXPECT_EQ(chain.parent_to_link.parent_frame, "world");
  EXPECT_EQ(chain.parent_to_link.child_frame, "usb_cam_link");
  EXPECT_EQ(chain.parent_to_link.translation, (std::array<double, 3>{{1.0, 2.0, 3.0}}));
  expectSameRotation(chain.parent_to_link.quaternion, {{0.5, -0.5, 0.5, 0.5}});
  EXPECT_EQ(chain.link_to_optical.parent_frame, "usb_cam_link");
  EXPECT_EQ(chain.link_to_optical.child_frame, "usb_cam_optical_frame");
  EXPECT_EQ(chain.link_to_optical.translation, (std::array<double, 3>{{0.0, 0.0, 0.0}}));
  expectSameRotation(chain.link_to_optical.quaternion, driver::kStandardLinkToOpticalQuaternion);
}

TEST(StillAssetMath, ChainRecomposesToTheStoredExtrinsic)
{
  const double half = std::sqrt(0.5);
  driver::Extrinsic extrinsic;
  extrinsic.parent_frame = "world";
  extrinsic.child_frame = "usb_cam_optical_frame";
  extrinsic.translation = {{2.798, 7.207, 1.906}};
  extrinsic.quaternion = {{0.0, 0.0, half, half}};  // 90 degrees about z
  const driver::CameraStaticTransforms chain = driver::composeStaticCameraTransforms(extrinsic);
  EXPECT_EQ(chain.parent_to_link.translation, extrinsic.translation);
  expectSameRotation(
      driver::quaternionMultiply(chain.parent_to_link.quaternion, chain.link_to_optical.quaternion),
      extrinsic.quaternion);
}

TEST(StillAssetMath, CustomLinkRotationIsRespected)
{
  const double half = std::sqrt(0.5);
  driver::Extrinsic extrinsic;
  extrinsic.parent_frame = "world";
  extrinsic.child_frame = "cam_optical_frame";
  extrinsic.translation = {{0.0, 0.0, 0.0}};
  extrinsic.quaternion = {{0.0, 0.0, 0.0, 1.0}};
  const driver::Quaternion about_y{{0.0, half, 0.0, half}};
  const driver::CameraStaticTransforms chain =
      driver::composeStaticCameraTransforms(extrinsic, about_y);
  expectSameRotation(chain.parent_to_link.quaternion, {{0.0, -half, 0.0, half}});
}

TEST(StillAssetMath, LinkFrameDerivation)
{
  EXPECT_EQ(driver::deriveLinkFrame("usb_cam_optical_frame"), "usb_cam_link");
  EXPECT_THROW(driver::deriveLinkFrame("usb_cam"), std::invalid_argument);
  EXPECT_THROW(driver::deriveLinkFrame("_optical_frame"), std::invalid_argument);
  EXPECT_THROW(driver::quaternionNormalize({{0.0, 0.0, 0.0, 0.0}}), std::invalid_argument);
}

TEST(StillAssetIntrinsics, LoadsTheSchema)
{
  const driver::Intrinsics intrinsics = driver::loadIntrinsics(writeFile("intrinsic.yaml", kIntrinsic));
  EXPECT_EQ(intrinsics.width, 64U);
  EXPECT_EQ(intrinsics.height, 48U);
  EXPECT_DOUBLE_EQ(intrinsics.camera_matrix[0], 50.0);
  EXPECT_DOUBLE_EQ(intrinsics.camera_matrix[2], 31.5);
  EXPECT_DOUBLE_EQ(intrinsics.camera_matrix[4], 51.0);
  EXPECT_DOUBLE_EQ(intrinsics.camera_matrix[8], 1.0);
  EXPECT_EQ(intrinsics.distortion_model, "plumb_bob");
  ASSERT_EQ(intrinsics.distortion.size(), 5U);
  EXPECT_DOUBLE_EQ(intrinsics.distortion[1], -0.05);
}

TEST(StillAssetIntrinsics, RejectsMalformedDocuments)
{
  const std::string good = kIntrinsic;
  EXPECT_THROW(driver::loadIntrinsics(temporaryPath("missing.yaml")), std::invalid_argument);
  EXPECT_THROW(
      driver::loadIntrinsics(writeFile("schema.yaml", replaceFirst(good, "intrinsic.v1", "intrinsic.v2"))),
      std::invalid_argument);
  EXPECT_THROW(
      driver::loadIntrinsics(writeFile("width.yaml", replaceFirst(good, "image_width: 64", "image_width: 0"))),
      std::invalid_argument);
  EXPECT_THROW(
      driver::loadIntrinsics(writeFile("float.yaml", replaceFirst(good, "image_width: 64", "image_width: 64.5"))),
      std::invalid_argument);
  EXPECT_THROW(
      driver::loadIntrinsics(writeFile("rows.yaml", replaceFirst(good, "rows: 3", "rows: 2"))),
      std::invalid_argument);
  EXPECT_THROW(
      driver::loadIntrinsics(writeFile("count.yaml", replaceFirst(good, "0.0, 0.0, 1.0]", "0.0, 0.0]"))),
      std::invalid_argument);
  EXPECT_THROW(
      driver::loadIntrinsics(writeFile("nan.yaml", replaceFirst(good, "[50.0,", "[.nan,"))),
      std::invalid_argument);
  EXPECT_THROW(
      driver::loadIntrinsics(writeFile("cols.yaml", replaceFirst(good, "cols: 5", "cols: 4"))),
      std::invalid_argument);
  EXPECT_THROW(
      driver::loadIntrinsics(writeFile("model.yaml", replaceFirst(good, "distortion_model: plumb_bob\n", "distortion_model: ''\n"))),
      std::invalid_argument);
  EXPECT_THROW(driver::loadIntrinsics(writeFile("list.yaml", "- 1\n- 2\n")), std::invalid_argument);
}

TEST(StillAssetExtrinsic, LoadsAndNormalizes)
{
  const std::string scaled = replaceFirst(
      kExtrinsic, "{x: 0.0, y: 0.0, z: 0.0, w: 1.0}", "{x: 0.0, y: 0.0, z: 0.0, w: 4.0}");
  const driver::Extrinsic extrinsic = driver::loadExtrinsic(writeFile("extrinsic.yaml", scaled));
  EXPECT_EQ(extrinsic.parent_frame, "world");
  EXPECT_EQ(extrinsic.child_frame, "usb_cam_optical_frame");
  EXPECT_EQ(extrinsic.translation, (std::array<double, 3>{{1.0, 2.0, 3.0}}));
  EXPECT_DOUBLE_EQ(extrinsic.quaternion[3], 1.0);
}

TEST(StillAssetExtrinsic, RejectsMalformedDocuments)
{
  const std::string good = kExtrinsic;
  EXPECT_THROW(
      driver::loadExtrinsic(writeFile("convention.yaml", replaceFirst(good, "parent_T_camera_optical", "camera_T_parent"))),
      std::invalid_argument);
  EXPECT_THROW(
      driver::loadExtrinsic(writeFile("zero.yaml", replaceFirst(good, "w: 1.0", "w: 0.0"))),
      std::invalid_argument);
  EXPECT_THROW(
      driver::loadExtrinsic(writeFile("axis.yaml", replaceFirst(good, "z: 3.0", "q: 3.0"))),
      std::invalid_argument);
  EXPECT_THROW(
      driver::loadExtrinsic(writeFile("frame.yaml", replaceFirst(good, "parent_frame: world", "parent_frame: ''"))),
      std::invalid_argument);
}

class StillImageTest : public ::testing::Test {
 protected:
  void SetUp() override
  {
    pattern_ = cv::Mat(48, 64, CV_8UC3);
    for (int row = 0; row < pattern_.rows; ++row) {
      for (int column = 0; column < pattern_.cols; ++column) {
        pattern_.at<cv::Vec3b>(row, column) = cv::Vec3b(
            static_cast<std::uint8_t>(column * 4), static_cast<std::uint8_t>(row * 5), 90);
      }
    }
    intrinsics_ = driver::loadIntrinsics(writeFile("intrinsic.yaml", kIntrinsic));
  }

  cv::Mat pattern_;
  driver::Intrinsics intrinsics_;
};

TEST_F(StillImageTest, PngIsConvertedToJpegOnce)
{
  const std::string path = temporaryPath("scene.png");
  ASSERT_TRUE(cv::imwrite(path, pattern_));
  const driver::StillImage image = driver::loadStillImage(path, intrinsics_, 95);
  EXPECT_EQ(image.width, 64U);
  EXPECT_EQ(image.height, 48U);
  ASSERT_GT(image.jpeg.size(), 4U);
  EXPECT_EQ(image.jpeg[0], 0xffU);
  EXPECT_EQ(image.jpeg[1], 0xd8U);
  const cv::Mat decoded = cv::imdecode(image.jpeg, cv::IMREAD_COLOR);
  ASSERT_FALSE(decoded.empty());
  EXPECT_EQ(decoded.size(), pattern_.size());
}

TEST_F(StillImageTest, JpegIsPublishedByteForByte)
{
  const std::string path = temporaryPath("scene.jpg");
  ASSERT_TRUE(cv::imwrite(path, pattern_, {cv::IMWRITE_JPEG_QUALITY, 80}));
  std::ifstream stream(path, std::ios::binary);
  const std::string original((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  const driver::StillImage image = driver::loadStillImage(path, intrinsics_, 95);
  EXPECT_EQ(std::string(image.jpeg.begin(), image.jpeg.end()), original);
}

TEST_F(StillImageTest, RejectsMismatchesAndBadFiles)
{
  const std::string wide = temporaryPath("wide.png");
  ASSERT_TRUE(cv::imwrite(wide, cv::Mat(48, 128, CV_8UC3, cv::Scalar(1, 2, 3))));
  EXPECT_THROW(driver::loadStillImage(wide, intrinsics_, 95), std::invalid_argument);
  EXPECT_THROW(driver::loadStillImage(temporaryPath("absent.png"), intrinsics_, 95), std::invalid_argument);
  EXPECT_THROW(
      driver::loadStillImage(writeFile("scene.bmp", "BM"), intrinsics_, 95), std::invalid_argument);
  EXPECT_THROW(
      driver::loadStillImage(writeFile("garbage.jpg", "not an image at all"), intrinsics_, 95),
      std::invalid_argument);
  const std::string good = temporaryPath("good.png");
  ASSERT_TRUE(cv::imwrite(good, pattern_));
  EXPECT_THROW(driver::loadStillImage(good, intrinsics_, 0), std::invalid_argument);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  testing::AddGlobalTestEnvironment(new RemoveTestDirectory());
  return RUN_ALL_TESTS();
}
