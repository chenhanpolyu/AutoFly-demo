mavros_pids=$(pidof mavros_node)
echo "mavros_pids: $mavros_pids"
for pid in $mavros_pids
do
	renice -20 -p $pid
done

ekf_pids=$(pidof ekf)
echo "ekf_pids: $ekf_pids"
for pid in $ekf_pids
do
	renice -20 -p $pid
done

px4ctrl_pids=$(pidof px4ctrl_node)
echo "px4ctrl_pids: $px4ctrl_pids"
for pid in $px4ctrl_pids
do
	renice -20 -p $pid
done

