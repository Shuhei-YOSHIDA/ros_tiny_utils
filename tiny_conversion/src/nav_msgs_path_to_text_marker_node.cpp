/**
 * @file nav_msgs_path_to_text_marker_node.cpp
 * @brief Subscribe a path of nav_msgs/Path, and convert it to text marker of visualizing_msgs/MarkerArray
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace visualization_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;

ros::Publisher marker_pub;
ros::Subscriber path_sub;

string dToStr(double value, int digit=2)
{
  stringstream stream;
  stream << fixed << setprecision(digit) << value;
  return stream.str();
}

Marker textMarker(PoseStamped poses)
{
  Marker m_msg;

  m_msg.header = poses.header;
  m_msg.pose = poses.pose;
  m_msg.scale.z = 1.0;
  m_msg.type = Marker::TEXT_VIEW_FACING;
  m_msg.color.a = 1.0;
  string unit = "m";
  string delimiter = ",";
  m_msg.text = "("
    + dToStr(poses.pose.position.x) + unit + delimiter
    + dToStr(poses.pose.position.y) + unit + delimiter
    + dToStr(poses.pose.position.z) + unit + ")";

  return m_msg;
} 

void pathCB(const Path::ConstPtr& msg)
{
  MarkerArray ma_msg;
  for (int i = 0; i < msg->poses.size(); i++)
  {
    ///@Todo Convert pose by TF?
    auto m_msg = textMarker(msg->poses[i]);
    m_msg.id = i;
    ma_msg.markers.push_back(m_msg);
  }

  marker_pub.publish(ma_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_to_text_marker_node");
  ros::NodeHandle nh;

  marker_pub = nh.advertise<MarkerArray>("path_text_marker", 1, true);
  path_sub = nh.subscribe("path", 1, pathCB);

  ros::spin();
  return 0;
}
