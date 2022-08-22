#pragma once

#ifndef SE3_PLANNER_h
#define SE3_PLANNER_h
#include <ros/package.h>
#include <cmath>
#include <iostream>
#include <Eigen/Eigen>
#include <memory>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "nav_msgs/Path.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <decomp_basis/data_type.h>
#include <decomp_geometry/polyhedron.h>
#include "decomp_ros_utils/data_ros_utils.h"
#include "decomp_util/ellipsoid_decomp.h"
// #include <quadrotor_msgs/PolynomialTrajectory.h>
// #include <jps_planner/jps_planner/jps_planner.h>
// #include <jps_collision/map_util.h>
#include <std_msgs/Float64.h>
#include "trajectory.hpp"

using namespace std;        
// using namespace JPS;




#endif
