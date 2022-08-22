#!/bin/bash
echo 0000 | sudo -S chmod 777 /dev/ttyACM0
sleep 6
echo “boost IMU!”
rosrun mavros mavcmd long 511 31 6666 0 0 0 0 0
