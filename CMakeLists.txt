cmake_minimum_required(VERSION 2.8.3)
project(optoforce)

add_definitions(-std=c++11)
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-pthread")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rospy std_msgs
   DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include/Optoforce6Axis
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/optoDriver.cpp
)

add_executable(optoforce_node src/test.cpp)
target_link_libraries(optoforce_node ${PROJECT_NAME} ${catkin_LIBRARIES})
