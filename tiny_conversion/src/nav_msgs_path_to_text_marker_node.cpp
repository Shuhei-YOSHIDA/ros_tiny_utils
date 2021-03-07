/**
 * @file nav_msgs_path_to_text_marker_node.cpp
 * @brief Subscribe a path of nav_msgs/Path, and convert it to text marker of visualizing_msgs/MarkerArray
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "color_names/color_names.h"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

using namespace std;
using namespace visualization_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;

ros::Publisher marker_pub;
ros::Subscriber path_sub;

std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr;

string dToStr(double value, int digit=2)
{
  stringstream stream;
  stream << fixed << setprecision(digit) << value;
  return stream.str();
}

// Parameter for text
double font_size = 1.0;
string color_name = "black";
string unit = "m";
string delimiter = ",";
int digit = 2;
string target_frame_id = "";

void paramLoad()
{
  XmlRpc::XmlRpcValue font_data;
  if (ros::param::get("~font_data", font_data))
  {
    font_size = static_cast<double>(font_data["font_size"]);
    color_name = static_cast<string>(font_data["color_name"]);
    unit = static_cast<string>(font_data["unit"]);
    delimiter = static_cast<string>(font_data["delimiter"]);
    digit = static_cast<int>(font_data["digit"]);
    target_frame_id = static_cast<string>(font_data["target_frame_id"]);
  }
}

Marker textMarker(PoseStamped poses)
{
  Marker m_msg;

  m_msg.header = poses.header;
  m_msg.pose = poses.pose;
  m_msg.type = Marker::TEXT_VIEW_FACING;

  m_msg.scale.z = font_size;
  m_msg.color = color_names::makeColorMsg(color_name);

  m_msg.text = "("
    + dToStr(poses.pose.position.x, digit) + unit + delimiter
    + dToStr(poses.pose.position.y, digit) + unit + delimiter
    + dToStr(poses.pose.position.z, digit) + unit + ")";

  return m_msg;
} 

void pathCB(const Path::ConstPtr& msg)
{
  static int previous_size = 0;

  if (previous_size != 0 && previous_size != msg->poses.size())
  {
    Marker delete_msg;
    delete_msg.action = Marker::DELETEALL;
    delete_msg.header.stamp = ros::Time::now();
    MarkerArray ma_msg;
    ma_msg.markers.push_back(delete_msg);
    marker_pub.publish(ma_msg);
    ROS_WARN("Detected that size of path-msg is changed");
    ros::Duration(0.3).sleep();
  }
  previous_size = msg->poses.size();

  MarkerArray ma_msg;
  for (int i = 0; i < msg->poses.size(); i++)
  {
    // Convert each pose with target_frame_id
    Marker m_msg;
    if (target_frame_id == "")
    {
      m_msg = textMarker(msg->poses[i]);
    }
    else
    {
      geometry_msgs::TransformStamped tf;
      try
      {
        tf = tf_buffer_ptr->lookupTransform(target_frame_id, msg->poses[i].header.frame_id, ros::Time(0));
        PoseStamped new_poses;
        tf2::doTransform(msg->poses[i], new_poses, tf);
        m_msg = textMarker(new_poses);
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        continue;
      }
    }

    m_msg.id = i;
    ma_msg.markers.push_back(m_msg);
  }

  marker_pub.publish(ma_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_to_text_marker_node");
  ros::NodeHandle nh;

  tf_buffer_ptr = make_shared<tf2_ros::Buffer>();
  tf_listener_ptr = make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr);

  marker_pub = nh.advertise<MarkerArray>("path_text_marker", 1, true);
  path_sub = nh.subscribe("path", 1, pathCB);

  paramLoad();

  ros::spin();
  return 0;
}
