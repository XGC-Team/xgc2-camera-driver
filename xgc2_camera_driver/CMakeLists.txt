cmake_minimum_required(VERSION 3.10)
project(xgc2_camera_driver VERSION 0.1.0 LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  tf2_ros
  xgc_camera_msgs
)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(xgc2_camera CONFIG REQUIRED)

# The H.264/RTP media outputs need FFmpeg and the XRPC http.v1 server, which
# exists for Ubuntu 20.04 and newer. Older robots keep the ROS-only camera.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(FFMPEG QUIET IMPORTED_TARGET libavcodec libavutil libswscale)
endif()
find_package(XgcXrpc QUIET COMPONENTS http json_http)
if(FFMPEG_FOUND AND XgcXrpc_FOUND)
  set(XGC2_CAMERA_MEDIA TRUE)
else()
  set(XGC2_CAMERA_MEDIA FALSE)
  message(STATUS "xgc2_camera_driver: media outputs disabled (FFmpeg and XgcXrpc http/json_http are required)")
endif()

catkin_package(
  CATKIN_DEPENDS camera_info_manager cv_bridge diagnostic_msgs diagnostic_updater geometry_msgs image_transport roscpp sensor_msgs tf2_ros xgc_camera_msgs
  DEPENDS OpenCV
)

add_executable(xgc2_camera_driver_node
  src/camera_driver_node.cpp
  src/capture_source.cpp
  src/clock_mapper.cpp
  src/still_asset.cpp
)
add_dependencies(xgc2_camera_driver_node ${catkin_EXPORTED_TARGETS})
target_compile_features(xgc2_camera_driver_node PRIVATE cxx_std_14)
target_compile_options(xgc2_camera_driver_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(xgc2_camera_driver_node
  PRIVATE
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    Threads::Threads
    xgc2::camera
)
target_include_directories(xgc2_camera_driver_node PRIVATE ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})

install(TARGETS xgc2_camera_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS
  scripts/xgc_camera_apply_alignment
  scripts/xgc_camera_alignment
  scripts/xgc_camera_bag_export
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  if(NOT "$ENV{ROS_DISTRO}" STREQUAL "melodic")
    catkin_add_nosetests(test/test_4k_defaults.py)
    catkin_add_nosetests(test/test_camera_tools.py)
  endif()
  catkin_add_gtest(test_clock_mapper
    test/test_clock_mapper.cpp
    src/clock_mapper.cpp
  )
  if(TARGET test_clock_mapper)
    target_compile_features(test_clock_mapper PRIVATE cxx_std_14)
    target_include_directories(test_clock_mapper PRIVATE src)
    target_link_libraries(test_clock_mapper xgc2::camera)
  endif()
  catkin_add_gtest(test_still_asset
    test/test_still_asset.cpp
    src/still_asset.cpp
  )
  if(TARGET test_still_asset)
    target_compile_features(test_still_asset PRIVATE cxx_std_14)
    target_include_directories(test_still_asset PRIVATE src ${OpenCV_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})
    target_link_libraries(test_still_asset ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})
  endif()
  catkin_add_gtest(test_capture_source
    test/test_capture_source.cpp
    src/capture_source.cpp
  )
  if(TARGET test_capture_source)
    target_compile_features(test_capture_source PRIVATE cxx_std_14)
    target_include_directories(test_capture_source PRIVATE src ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(test_capture_source ${OpenCV_LIBRARIES} xgc2::camera)
  endif()
  if(XGC2_CAMERA_MEDIA)
    catkin_add_gtest(test_h264_encoder
      test/test_h264_encoder.cpp
      src/h264_decoder.cpp
      src/h264_encoder.cpp
      src/jpeg_decoder.cpp
    )
    if(TARGET test_h264_encoder)
      target_compile_features(test_h264_encoder PRIVATE cxx_std_14)
      target_include_directories(test_h264_encoder PRIVATE src test ${OpenCV_INCLUDE_DIRS})
      target_link_libraries(test_h264_encoder PkgConfig::FFMPEG ${OpenCV_LIBRARIES})
    endif()
    catkin_add_gtest(test_media_pipeline
      test/test_media_pipeline.cpp
      src/h264_decoder.cpp
      src/h264_encoder.cpp
      src/jpeg_decoder.cpp
      src/media_pipeline.cpp
      src/rtp_h264.cpp
    )
    if(TARGET test_media_pipeline)
      target_compile_features(test_media_pipeline PRIVATE cxx_std_14)
      target_include_directories(test_media_pipeline PRIVATE src test ${OpenCV_INCLUDE_DIRS})
      target_link_libraries(test_media_pipeline PkgConfig::FFMPEG ${OpenCV_LIBRARIES} Threads::Threads)
    endif()
    catkin_add_gtest(test_source_control_service
      test/test_source_control_service.cpp
      src/h264_decoder.cpp
      src/h264_encoder.cpp
      src/jpeg_decoder.cpp
      src/media_pipeline.cpp
      src/rtp_h264.cpp
      src/source_control_service.cpp
      src/source_state.cpp
    )
    if(TARGET test_source_control_service)
      target_compile_features(test_source_control_service PRIVATE cxx_std_17)
      target_include_directories(test_source_control_service PRIVATE src test ${OpenCV_INCLUDE_DIRS})
      target_link_libraries(test_source_control_service
        XgcXrpc::json_http PkgConfig::FFMPEG ${OpenCV_LIBRARIES} Threads::Threads)
    endif()
    catkin_add_gtest(test_rtp_h264
      test/test_rtp_h264.cpp
      src/rtp_h264.cpp
    )
    if(TARGET test_rtp_h264)
      target_compile_features(test_rtp_h264 PRIVATE cxx_std_14)
      target_include_directories(test_rtp_h264 PRIVATE src)
      target_link_libraries(test_rtp_h264 Threads::Threads)
    endif()
  endif()
  catkin_add_gtest(test_camera_info_contract test/test_camera_info_contract.cpp)
  if(TARGET test_camera_info_contract)
    target_compile_features(test_camera_info_contract PRIVATE cxx_std_14)
    target_include_directories(test_camera_info_contract PRIVATE src)
  endif()
  if(NOT "$ENV{ROS_DISTRO}" STREQUAL "melodic")
    add_rostest(test/synthetic_driver.test)
    add_rostest(test/still_driver.test)
  endif()
endif()
