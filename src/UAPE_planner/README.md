# UAPE_planner: Uncertainty-Aware and Perception-Enhanced trajectory planning for dynamic environments


It depends on "sfc" and "px4Controller" two packages, please clone them and compile them in your workspace first.

Compile this package and source your workspace.


To begin the navigation,run:

`roslaunch ahpf_planner traj_node.launch`

The goal is set in `planning_params.yaml`, after the drone reached the goal, you can manually terminate the program and set another goal, save the yaml file, restart the navigation.

Set `ReturnHome` to a number greater than 2 to activate the repeating mode. The number indicates the number of navigations. For example, "4" means fly to goal (1 navigation), return home (2 navigation), fly to goal again (3 navigation), return home again (4 navigation).

Set if_RandomGoal to true to activate the randomly fly mode. The drone will choose the next goal randomly inside the global bounding box (see parameter `GlobalBox_min`, `GlobalBox_size`). It is usaully used in simulation. Please set it to false if you want to use other modes.

You can set "ifMove" to false to check the planned trajectory without the drone move its position, the drone will hover at its original position.

"VelMax" is the upper speed limitation of the planned trajectory. "ThrustAccMax" is for the acceleration, and please set it according to your actual dynamic limit (usually not greater than 20).
