tiny_rviz_plugins
====

# MultiRobotStateDisplay
This plugin subscribes a topic "multi_robot_state(tiny_rviz_plugins/MultiRobotStateDisplay)".
And then, it shows multiple robots in RViz.
It is useful to take a 3d successive image from data you have.

![MultiRobotStateDisplay example](https://raw.githubusercontent.com/Shuhei-YOSHIDA/ros_tiny_utils/master/tiny_rviz_plugins/images/multirobotstatedisplay.png)

## Install and Try sample
``` bash
$ sudo apt install ros-kinetic-nao-description
$ roslaunch tiny_rviz_plugins sample.launch
```
## Notes
* Currently, topic-name of tiny_rviz_plugins/MultiRobotStateDisplay" should be **multi_robot_state**. 

# multi_prefixed_tf_broadcaster_node
This node subscribe a topic "multi_robot_state(tiny_rviz_plugins/MultiRobotStateDisplay)".
And it reads different urdf and its numbers vir ros-parameter.
The names are "mptb_[num]/robot_description" and "mptb_datasize".
(If not set, "robot_description" is used.)

You need to set multiple "RobotModel" rviz plugins manually.
