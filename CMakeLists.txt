cmake_minimum_required(VERSION 2.8.12)
project(multi_lidar_calibrator)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        message_filters
        std_msgs
        sensor_msgs
        geometry_msgs
        pcl_ros
        pcl_conversions
        )

if(CATKIN_ENABLE_TESTING)
        find_package(rostest REQUIRED)
endif()

catkin_package(CATKIN_DEPENDS
        message_filters
        std_msgs
        sensor_msgs
        geometry_msgs
        )

find_package(PCL 1.7 REQUIRED)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_STANDARD 11)

link_directories(${PCL_LIBRARY_DIRS})

# MultiLidar Calibrator
add_library(multi_lidar_calibrator_lib SHARED
        src/multi_lidar_calibrator.cpp
        include/multi_lidar_calibrator.h)

target_include_directories(multi_lidar_calibrator_lib PUBLIC
        ${PCL_INCLUDE_DIRS}
        )

target_link_libraries(multi_lidar_calibrator_lib
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(multi_lidar_calibrator
        src/multi_lidar_calibrator_node.cpp
        )

target_include_directories(multi_lidar_calibrator PRIVATE
        include)

target_link_libraries(multi_lidar_calibrator
        multi_lidar_calibrator_lib)

if(CATKIN_ENABLE_TESTING)
        catkin_install_python(PROGRAMS test/test_identity_calibration.py
                DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        add_rostest(test/multi_lidar_calibrator_identity.test)
endif()
