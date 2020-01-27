/**
 * @file multi_prefixed_tf_broadcaster_node.cpp
 */

/*
 * 0. Set parameter for URDFs in order of rviz_tiny_plugins/MultiRobotStateDisplay
 * 1. Subscribe rviz_tiny_plugins/MultiRobotStateDisplay
 * 2. Broadcast a set of tf, where prefix is added to each tf for a robot
 * 3. Set "RobotModel" rviz plugins for each robot in RViz
 * 4. You will get multi robot states to be shown
 */

#include "tiny_rviz_plugins/fixed_state_broadcaster.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_prefixed_tf_broadcaster_node");
  
  return 0;
}
