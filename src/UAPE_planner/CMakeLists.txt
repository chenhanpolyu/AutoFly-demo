cmake_minimum_required(VERSION 3.10.0)
project(ahpf_planner)

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake/gazebo/")
# set(gazebo_DIR "/usr/lib/x86_64-linux-gnu/cmake/gazebo/")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
#find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  # gazebo REQUIRED
  rospy
  std_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  roscpp
  cv_bridge
  sensor_msgs
  control_msgs
  # gazebo_msgs
  # gazebo_plugins
  geometry_msgs
  nav_msgs
  roslib
  std_srvs
  sensor_msgs
  decomp_ros_utils
  obj_state_msgs
  mavros_msgs
  quadrotor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#catkin_package()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
  catkin_package(
 #   INCLUDE_DIRS include
    INCLUDE_DIRS src
    DEPENDS OpenCV Eigen Boost
    DEPENDS
    roscpp
    # gazebo_ros
    image_transport
    camera_info_manager
    sensor_msgs
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    SYSTEM 
    ${catkin_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
   # ${PROJECT_SOURCE_DIR}/include
   # ${Eigen_INCLUDE_DIRS}
  #  ${Eigen3_INCLUDE_DIRS} 
#  ${catkin_INCLUDE_DIRS}

    # ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
    ${Eigen3_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${DECOMP_UTIL_INCLUDE_DIRS}
    ${CURSES_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/traj_opt
  ${CMAKE_CURRENT_SOURCE_DIR}/src/call_states)

# link_directories(${GAZEBO_LIBRARY_DIRS})

# add_library(realsense_gazebo_plugin
#     src/RealSensePlugin.cpp
#     src/gazebo_ros_realsense.cpp
#     )

# target_link_libraries(realsense_gazebo_plugin
#     ${catkin_LIBRARIES}
#     ${GAZEBO_LIBRARIES})
# add_dependencies(realsense_gazebo_plugin
#     ${catkin_EXPORTED_TARGETS})

set(ENV_OFFB
src/ros_missn.cpp 
src/call_states/ros_communicate.cpp
src/call_states/input.cpp

)

set(RFR 
src/planner_fast.cpp
src/path_searching/src/kinodynamic_astar.cpp
)
# add_executable(vo_republish_rec
#     src/vo_repub_rec.cpp)
# target_link_libraries(vo_republish_rec
#     ${catkin_LIBRARIES})
#add_executable(chen_filter_rgb src/chen_filter_rgb.cpp)
add_executable(
traj_node
 # src/run_plannctrl.cpp 
  src/run_plannctrl_astar.cpp
  src/Tools.cpp
  ${RFR}
  ${ENV_OFFB})
target_link_libraries(traj_node ${catkin_LIBRARIES})
# target_link_libraries(chen_filter_rgb ${catkin_LIBRARIES})
