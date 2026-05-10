# AutoFly-demo
All the required ROS packages for autonomous flight demo, except for the px4 outer-loop controller.

Just clone this repo at your root path, treat this folder as a workspace folder.

Please enter each folder inside /src and install the packages's dependencies as introduced in the README file.


The **original controller package in this project cannot be open-sourced now**, you can use this [package](https://github.com/chenhanpolyu/px4Controller-linear) and they have the same function for our demo. Just put this controller package inside our /src folder.

Then, run `catkin_make` to build the entire workspace.

启动方法请参考[中文版操作说明](https://github.com/chenhanpolyu/AutoFly-demo/blob/master/%E6%9C%AA%E7%9F%A5%E5%8A%A8%E6%80%81%E7%8E%AF%E5%A2%83%E8%87%AA%E4%B8%BB%E9%A3%9E%E8%A1%8C%E7%B3%BB%E7%BB%9F%E8%AF%B4%E6%98%8E%E4%B9%A6.docx)

如需要在Gazebo仿真环境使用，请参考[仿真飞行操作说明](https://github.com/chenhanpolyu/AutoFly-demo/blob/master/sim_tutorial.md)


## 功能演示
按照仿真流程操作，你会得到如下仿真结果：

<video width="800" controls autoplay loop muted>
  <source src="assets/gazebo.webm" type="video/webm">
  您的浏览器不支持视频播放，请点击<a href="assets/gazebo.webm">这里</a>下载观看。
</video>

<video width="800" controls autoplay loop muted>
  <source src="assets/rviz.webm" type="video/webm">
  您的浏览器不支持视频播放，请点击<a href="assets/rviz.webm">这里</a>下载观看。
</video>


