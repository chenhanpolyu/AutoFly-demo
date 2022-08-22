#!/bin/bash
sleep 3
rostopic pub /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
