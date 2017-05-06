cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_2_panel_detection)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  #eigen_conversions
  kuri_mbzirc_challenge_2_msgs
  kuri_mbzirc_challenge_2_tools
  pcl_ros
  pcl_conversions
  roscpp
  tf
  tf_conversions
  laser_geometry
  )

#find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS})

#find_package(PCL REQUIRED)
find_package(PCL 1.7 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS
   roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})
#Not the cleanest way, but the only way I could include header files from *_tools package
include_directories(../kuri_mbzirc_challenge_2_tools/include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(circumnavigation_waypoints src/circumnavigation_waypoints.cpp)
target_link_libraries(circumnavigation_waypoints ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(circumnavigation_waypoints ${catkin_EXPORTED_TARGETS})

add_executable(test_velodyne_alignment src/test_velodyne_alignment.cpp)
target_link_libraries(test_velodyne_alignment ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(test_velodyne_alignment ${catkin_EXPORTED_TARGETS})
