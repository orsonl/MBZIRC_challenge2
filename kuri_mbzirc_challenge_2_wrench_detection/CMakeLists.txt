cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_2_wrench_detection)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  kuri_mbzirc_challenge_2_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
    actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(kuri_wrench_detection src/kuri_wrench_detection.cpp)
target_link_libraries(kuri_wrench_detection ${catkin_LIBRARIES})
add_dependencies(kuri_wrench_detection ${catkin_EXPORTED_TARGETS})

add_executable(kuri_wrench_detection_cloudImgPub src/kuri_wrench_detection_cloudImgPub.cpp)
target_link_libraries(kuri_wrench_detection_cloudImgPub ${catkin_LIBRARIES})
add_dependencies(kuri_wrench_detection_cloudImgPub ${catkin_EXPORTED_TARGETS})

add_executable(plane_orientation src/plane_orientation.cpp)
target_link_libraries(plane_orientation ${catkin_LIBRARIES})

#############
## Install ##
#############
install(DIRECTORY launch urdf config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
endif()
