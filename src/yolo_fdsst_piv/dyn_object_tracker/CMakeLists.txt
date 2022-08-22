cmake_minimum_required(VERSION 2.8.3)
project(dyn_object_tracker)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread -fconcepts") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")
ADD_COMPILE_OPTIONS(-std=c++14 )
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  nodelet
 target_ros_msgs
 obj_state_msgs
 geometry_msgs
 nav_msgs
 message_filters
)

add_message_files(
    FILES
    Metadata.msg
    )

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    roscpp
    rospy
    std_msgs
    message_runtime
    nodelet
    sensor_msgs
    geometry_msgs
    nav_msgs
    target_ros_msgs
    obj_state_msgs
    )

add_library(dyn_object_tracker src/chen_filter_rgb.cpp src/piv_nodelet.cpp src/Tools.cpp src/kalman.cpp)
target_link_libraries(dyn_object_tracker
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    fftw3
    )

add_dependencies(dyn_object_tracker target_ros_msgs_gencpp obj_state_msgs_gencpp)
#add_executable(chen_filter_rgb src/chen_filter_rgb.cpp)
#target_link_libraries(chen_filter_rgb
#    ${catkin_LIBRARIES})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include
  SYSTEM 
  ${Boost_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)
