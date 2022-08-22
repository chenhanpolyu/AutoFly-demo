sudo ./chmod_serial.sh & sleep 1;
roslaunch mavros px4.launch & sleep 3;
#roslaunch vicon_bridge vicon.launch & sleep 1;
roslaunch ekf PX4_vicon.launch & sleep 1;
roslaunch traj_server traj_server.launch & sleep 1;
wait;
