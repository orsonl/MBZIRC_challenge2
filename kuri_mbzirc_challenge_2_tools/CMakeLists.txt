cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_2_tools)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    tf_conversions
)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS})

find_package(PCL 1.7 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS
   roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})
