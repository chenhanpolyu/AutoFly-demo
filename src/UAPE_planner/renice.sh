#!/bin/bash
traj_pids=$(pidof traj_node)
echo "traj_pids: $traj_pids"
for pid in $traj_pids
do
	echo 0000 | sudo -S renice -20 -p $pid
done

