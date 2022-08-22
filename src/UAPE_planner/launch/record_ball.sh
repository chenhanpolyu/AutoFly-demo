#!/bin/bash
cd ~/odom_cmd_bags
rosbag record -o "ball_odom" /vicon_imu_ekf_odom /camera/color/image_raw /camera/depth/image_rect_raw /optimal_traj /optimal_trajectory /objects_states