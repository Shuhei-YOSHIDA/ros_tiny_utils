/**
 * @file color_names_node.cpp
 */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "color_names/color_names.h"
using namespace visualization_msgs;
using namespace color_names;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_names_list");
  ros::NodeHandle nh;

  MarkerArray mrks_msg;
  Marker color_mrk;
  color_mrk.header.frame_id = "/base_link";
  color_mrk.type = Marker::SPHERE_LIST;
  color_mrk.scale.x = color_mrk.scale.y = color_mrk.scale.z = 0.050;
  color_mrk.id = 0;
  int id = 1;
  int count = 0;
  for (auto&& data : COLOR_NAME_DICT)
  {
    geometry_msgs::Point p;
    ///@todo set points in good way, May be need to sort color data by names
    p.x = 0.10*(count%10);
    p.y = 0.10*(count/10);
    count++;

    Marker m_txt;
    m_txt.header.frame_id = "/base_link";
    m_txt.type = Marker::TEXT_VIEW_FACING;
    m_txt.id = id;
    id++;
    m_txt.text = data.first;
    m_txt.color.r = m_txt.color.g = m_txt.color.b = m_txt.color.a = 1.0; // white
    m_txt.scale.z = 0.010;
    m_txt.pose.position = p;
    m_txt.pose.position.z += 0.050;
    m_txt.pose.orientation.w = 1;
    mrks_msg.markers.push_back(m_txt);

    color_mrk.colors.push_back(makeColorMsg(data.first, 1.0));
    color_mrk.points.push_back(p);
  }
  mrks_msg.markers.push_back(color_mrk);

  ros::Publisher mrks_pub = nh.advertise<MarkerArray>("color_sample_marker", 1);
  ROS_INFO("Publish color sample marker: frame_id:/base_link");
  while (ros::ok())
  {
    auto stamp = ros::Time::now();
    for (auto&& m : mrks_msg.markers) m.header.stamp = stamp;
    mrks_pub.publish(mrks_msg);
  }
  return 0;
}
