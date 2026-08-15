cmake_minimum_required(VERSION 3.10)
project(xgc2_camera_driver VERSION 0.1.0 LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  diagnostic_msgs
  diagnostic_updater
  foxglove_msgs
  image_transport
  roscpp
  sensor_msgs
  xgc_camera_msgs
)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)
find_package(xgc2_camera CONFIG REQUIRED)

catkin_package(
  CATKIN_DEPENDS camera_info_manager cv_bridge diagnostic_msgs diagnostic_updater foxglove_msgs image_transport roscpp sensor_msgs xgc_camera_msgs
  DEPENDS OpenCV
)

add_executable(xgc2_camera_driver_node
  src/camera_driver_node.cpp
  src/clock_mapper.cpp
  src/h264_annex_b.cpp
)
add_dependencies(xgc2_camera_driver_node ${catkin_EXPORTED_TARGETS})
target_compile_features(xgc2_camera_driver_node PRIVATE cxx_std_14)
target_compile_options(xgc2_camera_driver_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(xgc2_camera_driver_node
  PRIVATE
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    PkgConfig::FFMPEG
    Threads::Threads
    xgc2::camera
)
target_include_directories(xgc2_camera_driver_node PRIVATE ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

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
  scripts/xgc_native_v4l2_rtp
  scripts/xgc_ros_image_rtp
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_camera_tools.py)
  catkin_add_gtest(test_h264_input_buffer test/test_h264_input_buffer.cpp)
  if(TARGET test_h264_input_buffer)
    target_compile_features(test_h264_input_buffer PRIVATE cxx_std_14)
    target_include_directories(test_h264_input_buffer PRIVATE src)
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
  catkin_add_gtest(test_h264_annex_b
    test/test_h264_annex_b.cpp
    src/h264_annex_b.cpp
  )
  if(TARGET test_h264_annex_b)
    target_compile_features(test_h264_annex_b PRIVATE cxx_std_14)
    target_include_directories(test_h264_annex_b PRIVATE src)
  endif()
  add_rostest(test/synthetic_driver.test)
endif()
