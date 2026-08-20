#include <gtest/gtest.h>

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
