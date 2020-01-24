# ```Vectornav ROS Wrapper```
---
This package allows you to use the VectorNav INS with ROS. 

## Getting Started
---
Package Dependencies:
* roscpp
* sensor_msgs
* geometry_msgs
* tf2



Open a terminal and build the package:
```shell
cd ~/catkin_ws/src   
git clone https://github.com/stereolabs/zed-ros-wrapper.git
cd ../
catkin_make
source ./devel/setup.bash
```

Run the program:
```shell
rosrun vectornav vectornav_node
```