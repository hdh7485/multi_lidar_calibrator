cmake_minimum_required(VERSION 3.8)
project(multi_lidar_calibrator)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)

add_definitions(${PCL_DEFINITIONS})

add_library(multi_lidar_calibrator_lib SHARED
  src/multi_lidar_calibrator.cpp
)

target_include_directories(multi_lidar_calibrator_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(multi_lidar_calibrator_lib
  rclcpp
  message_filters
  sensor_msgs
  pcl_conversions
)
target_link_libraries(multi_lidar_calibrator_lib Eigen3::Eigen ${PCL_LIBRARIES})

add_executable(multi_lidar_calibrator
  src/multi_lidar_calibrator_node.cpp
)
target_include_directories(multi_lidar_calibrator PRIVATE include)
ament_target_dependencies(multi_lidar_calibrator rclcpp)
target_link_libraries(multi_lidar_calibrator multi_lidar_calibrator_lib)

install(
  TARGETS multi_lidar_calibrator_lib multi_lidar_calibrator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(PROGRAMS test/test_identity_calibration.py DESTINATION share/${PROJECT_NAME}/test)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_identity_calibration.py)
  add_launch_test(test/test_known_transform_calibration.py)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp message_filters sensor_msgs pcl_conversions Eigen3 PCL)
ament_package()
