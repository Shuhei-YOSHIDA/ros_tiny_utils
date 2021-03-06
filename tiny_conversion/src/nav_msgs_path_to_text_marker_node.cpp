/**
 * @file nav_msgs_path_to_text_marker_node.cpp
 * @brief Subscribe a path in nav_msgs/Path, and convert it to text marker in visualizing_msgs/MarkerArray
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace visualization_msgs;
using namespace nav_msgs;

ros::Publisher marker_pub;
ros::Subscriber path_sub;

void pathCB(const Path::ConstPtr& msg)
{

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_to_text_marker_node");
  ros::NodeHandle nh;

  marker_pub = nh.advertise<MarkerArray>("path_text_marker", 1);
  path_sub = nh.subscribe("path", 1, pathCB);

  ros::spin();
  return 0;
}
