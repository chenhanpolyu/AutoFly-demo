#!/bin/bash
sleep 5
echo “boost IMU!”
rosrun mavros mavcmd long 511 31 9500 0 0 0 0 0
