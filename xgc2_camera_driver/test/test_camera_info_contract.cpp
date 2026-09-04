#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>

#include <sys/stat.h>
#include <unistd.h>

#include "camera_info_contract.hpp"

TEST(CameraInfoContract, AcceptsNativeResolutionCalibration)
{
  EXPECT_NO_THROW(xgc2_camera_driver::validateCalibrationGeometry(
      true, 3840U, 2160U, 3840U, 2160U));
}

TEST(CameraInfoContract, AllowsAnExplicitlyUncalibratedStream)
{
  EXPECT_NO_THROW(xgc2_camera_driver::validateCalibrationGeometry(
      false, 0U, 0U, 3840U, 2160U));
}

TEST(CameraInfoContract, DefaultPinholeMatchesWorldCamera110DegAperture)
{
  const auto pinhole =
      xgc2_camera_driver::defaultUncalibratedPinhole(3840U, 2160U);
  const double expected_fx =
      3840.0 / (2.0 * std::tan(55.0 * std::acos(-1.0) / 180.0));
  EXPECT_NEAR(pinhole.fx, expected_fx, 1e-9);
  EXPECT_NEAR(pinhole.fx, 1344.398473, 1e-6);
  EXPECT_NEAR(pinhole.fy, pinhole.fx, 1e-12);
  EXPECT_DOUBLE_EQ(pinhole.cx, 1919.5);
  EXPECT_DOUBLE_EQ(pinhole.cy, 1079.5);
}

TEST(CameraInfoContract, DefaultPinholeRejectsInvalidGeometry)
{
  EXPECT_THROW(
      xgc2_camera_driver::defaultUncalibratedPinhole(0U, 2160U),
      std::invalid_argument);
  EXPECT_THROW(
      xgc2_camera_driver::defaultUncalibratedPinhole(3840U, 2160U, 0.0),
      std::invalid_argument);
  EXPECT_THROW(
      xgc2_camera_driver::defaultUncalibratedPinhole(3840U, 2160U, 180.0),
      std::invalid_argument);
}

TEST(CameraInfoContract, RejectsRelabeled1080pIntrinsicsFor4KCapture)
{
  EXPECT_THROW(
      xgc2_camera_driver::validateCalibrationGeometry(
          true, 1920U, 1080U, 3840U, 2160U),
      std::invalid_argument);
}

TEST(CameraInfoContract, RejectsRelabeled720pIntrinsicsFor4KCapture)
{
  EXPECT_THROW(
      xgc2_camera_driver::validateCalibrationGeometry(
          true, 1280U, 720U, 3840U, 2160U),
      std::invalid_argument);
}

TEST(CameraInfoContract, AcceptsStableCameraNameAndCanonicalPhysicalPartition)
{
  char directory_template[] = "/tmp/xgc2-camera-info-XXXXXX";
  const char* root = ::mkdtemp(directory_template);
  ASSERT_NE(root, nullptr);
  const std::string mode = std::string(root) + "/phy";
  const std::string camera = mode + "/usb_cam";
  ASSERT_EQ(::mkdir(mode.c_str(), 0700), 0);
  ASSERT_EQ(::mkdir(camera.c_str(), 0700), 0);
  const std::string file =
      camera + "/intrinsics-20260830T010203.000000Z.yaml";
  std::ofstream(file).put('\n');

  EXPECT_EQ(
      xgc2_camera_driver::canonicalPhysicalCameraInfoFile(file, "usb_cam"),
      file);
}

TEST(CameraInfoContract, RejectsUnstableCameraNamesBeforeInspectingThePath)
{
  for (const std::string& value :
       {"", "../outside", "usb/cam", "usb cam", ".usb_cam"}) {
    EXPECT_THROW(
        xgc2_camera_driver::canonicalPhysicalCameraInfoFile(
            "/does/not/exist", value),
        std::invalid_argument)
        << value;
  }
}

TEST(CameraInfoContract, RejectsLexicalTraversalIntoAnotherCameraPartition)
{
  char directory_template[] = "/tmp/xgc2-camera-traversal-XXXXXX";
  const char* root = ::mkdtemp(directory_template);
  ASSERT_NE(root, nullptr);
  const std::string mode = std::string(root) + "/phy";
  const std::string selected = mode + "/usb_cam";
  const std::string other = mode + "/other";
  const std::string simulation_mode = std::string(root) + "/sim";
  const std::string simulation_camera = simulation_mode + "/usb_cam";
  ASSERT_EQ(::mkdir(mode.c_str(), 0700), 0);
  ASSERT_EQ(::mkdir(selected.c_str(), 0700), 0);
  ASSERT_EQ(::mkdir(other.c_str(), 0700), 0);
  ASSERT_EQ(::mkdir(simulation_mode.c_str(), 0700), 0);
  ASSERT_EQ(::mkdir(simulation_camera.c_str(), 0700), 0);
  const std::string filename = "intrinsics-20260830T010203.000000Z.yaml";
  std::ofstream(other + "/" + filename).put('\n');
  std::ofstream(simulation_camera + "/" + filename).put('\n');
  const std::string outside_link =
      selected + "/intrinsics-20260830T020304.000000Z.yaml";
  ASSERT_EQ(::symlink((other + "/" + filename).c_str(), outside_link.c_str()), 0);

  EXPECT_THROW(
      xgc2_camera_driver::canonicalPhysicalCameraInfoFile(
      selected + "/../other/" + filename, "usb_cam"),
      std::invalid_argument);
  EXPECT_THROW(
      xgc2_camera_driver::canonicalPhysicalCameraInfoFile(
          simulation_camera + "/" + filename, "usb_cam"),
      std::invalid_argument);
  EXPECT_THROW(
      xgc2_camera_driver::canonicalPhysicalCameraInfoFile(
          outside_link, "usb_cam"),
      std::invalid_argument);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
