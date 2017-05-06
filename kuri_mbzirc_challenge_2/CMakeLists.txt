cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_2)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag

    kuri_mbzirc_challenge_2_msgs

    pcl_conversions
    #pcl_ros
)

find_package(PCL 1.7 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})


add_executable(process_velodyne_rosbag src/process_velodyne_rosbag.cpp)
target_link_libraries(process_velodyne_rosbag ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(process_velodyne_rosbag ${catkin_EXPORTED_TARGETS})
