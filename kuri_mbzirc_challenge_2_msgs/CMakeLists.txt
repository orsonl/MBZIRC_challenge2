cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_2_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib_msgs
    geometry_msgs
    message_generation
  )

add_action_files(
    DIRECTORY action
    FILES
    ArmMotion.action
    BoxPosition.action
    Huskynav.action
    PanelPosition.action
    WrenchDetection.action
)

add_message_files(
  FILES
  ObjectPose.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    actionlib_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
    actionlib_msgs
    message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})


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
