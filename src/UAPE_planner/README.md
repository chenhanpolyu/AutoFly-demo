# UAPE_planner: Uncertainty-Aware and Perception-Enhanced trajectory planning for dynamic environments

To run in Gazebo simulation, first modify gazebo_sim/launch/sim.sh line 7, l8-20. use your workspace name to replace "han_ws".

Also, in line 4-5 "111" should be replaced with your user password.

It depends on "sfc" and "px4Controller" two packages, please clone them and compile them in your workspace first.

Compile this package and source your workspace.

First, launch the gazebo simulation environment.

`roslaunch gazebo_sim gt_sim_indoor.launch`

Then, launch the px4 controller :

`roslaunch px4ctrl run_ctrl.launch`

After a few seconds, you should see the drone takes off and hovering stably.

To begin the navigation,run:

`roslaunch ahpf_planner traj_node.launch`

The goal is set in `planning_params.yaml`, after the drone reached the goal, you can manually terminate the program and set another goal, save the yaml file, restart the navigation.

You can set "ifMove" to false to check the planned trajectory without the drone move its position, the drone will hover at its original position.

"VelMax" is the upper speed limitation of the planned trajectory. "ThrustAccMax" is for the acceleration, and please set it according to your actual dynamic limit (usually not greater than 20).
