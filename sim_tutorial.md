1.在工作空间内放置Gazebo仿真环境的ROS包。开发时测试所使用的是IRIS飞机模型，px4飞控，以及D435相机，对应的仿真包已经[开源](https://github.com/arclab-hku/gazebo_playground)。
这个包里有修改后的Realsense-Gazebo插件，减轻了对机器的计算资源消耗，轻型笔记本电脑也可以流畅运行。
2.在工作空间的根目录下 catkin_make 编译。
3.source devel/setup.bash。然后在/home/USER_NAME目录下放置PX4-Autopilot（即px4固件源码）并在PX4-Autopilot目录下运行“make px4_sitl_default gazebo”。
具体操作：
''''
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot/
git checkout 71db090
git submodule sync --recursive
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
sudo apt upgrade libignition-math2 #(libignition-math4 for noetic)
make px4_sitl_default gazebo
''''
4.编译成功后，启动仿真环境：“roslaunch gazebo_sim gt_sim_indoor.launch”。里面包含了静态障碍物以及动态行人，以及一架带有深度相机的无人机。

如果配置Gazebo环境中出现问题，请参考[E2ES项目](https://github.com/HKPolyU-UAV/E2ES)的文档，里面对需要安装的依赖有详细的说明。
