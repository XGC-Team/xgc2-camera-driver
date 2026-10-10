cmake_minimum_required(VERSION 3.10)
project(xgc2_camera_driver VERSION 0.1.0 LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  diagnostic_msgs
  diagnostic_updater
  image_transport
  roscpp
  sensor_msgs
  xgc_camera_msgs
)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(Threads REQUIRED)
find_package(xgc2_camera CONFIG REQUIRED)

catkin_package(
  CATKIN_DEPENDS camera_info_manager cv_bridge diagnostic_msgs diagnostic_updater image_transport roscpp sensor_msgs xgc_camera_msgs
  DEPENDS OpenCV
)

add_executable(xgc2_camera_driver_node
  src/camera_driver_node.cpp
  src/clock_mapper.cpp
)
add_dependencies(xgc2_camera_driver_node ${catkin_EXPORTED_TARGETS})
target_compile_features(xgc2_camera_driver_node PRIVATE cxx_std_14)
target_compile_options(xgc2_camera_driver_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(xgc2_camera_driver_node
  PRIVATE
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
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
  catkin_add_gtest(test_camera_info_contract test/test_camera_info_contract.cpp)
  if(TARGET test_camera_info_contract)
    target_compile_features(test_camera_info_contract PRIVATE cxx_std_14)
    target_include_directories(test_camera_info_contract PRIVATE src)
  endif()
  if(NOT "$ENV{ROS_DISTRO}" STREQUAL "melodic")
    add_rostest(test/synthetic_driver.test)
  endif()
endif()
