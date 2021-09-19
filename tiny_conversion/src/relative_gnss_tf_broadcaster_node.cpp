/**
 * @file relative_gnss_tf_broadcaster_node.cpp
 * @brief Subscribe sensor_msgs/NavSatFix, and broadcast TF of UTM pose relative to particular gps data
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

// base_link ->(rotation)-> gps ->(local utm)-> local_gnss
string g_gps_frame_id;
string g_base_link_frame_id;
bool g_is_gps_frame_inversed = true; // from ENU(gps_frame_id) to body(base_link_frame_id)

void gpsCB(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static tf2_ros::TransformBroadcaster br;
  static bool is_first_msg = true;
  static sensor_msgs::NavSatFix first_msg;
  static geodesy::UTMPoint first_utm;
  if (is_first_msg) /// @todo shoule be reset by service
  {
    is_first_msg = false;

    first_msg = *msg;
    geographic_msgs::GeoPoint gp = geodesy::toMsg(*msg);
    geodesy::fromMsg(gp, first_utm);
  }

  geographic_msgs::GeoPoint gp = geodesy::toMsg(*msg);
  geodesy::UTMPoint utm;
  geodesy::fromMsg(gp, utm);

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = g_gps_frame_id;
  tf.header.stamp = ros::Time::now();
  tf.child_frame_id = "local_gnss";
  tf.transform.translation.x = first_utm.easting - utm.easting;
  tf.transform.translation.y = first_utm.northing - utm.northing;
  tf.transform.translation.z = first_utm.altitude - utm.altitude;
  tf.transform.rotation.w = 1.0;

  br.sendTransform(tf);
}

void attitudeCB(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  static tf2_ros::TransformBroadcaster br;

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = g_base_link_frame_id;
  tf.header.stamp = ros::Time::now();
  tf.child_frame_id = g_gps_frame_id;
  if (!g_is_gps_frame_inversed)
  {
    tf.transform.rotation = msg->quaternion;
  }
  else
  {
    tf2::Quaternion q;
    tf2::fromMsg(msg->quaternion, q);
    q = q.inverse();
    tf2::convert(q, tf.transform.rotation);
  }

  br.sendTransform(tf);
}

void usage()
{
  cerr << "Usage:" << endl
       << "rosrun tiny_conversion relative_gnss_tf_broadcaster_node gps_position:=/your/gps_topic attitude:=/usr/attitude_topic _gps_frame_id:=your_gps_id  _base_link_frame_id:=your_body_id" << endl
       << "-------" << endl
       << "Example:" << endl
       << "rosrun tiny_conversion relative_gnss_tf_broadcaster_node gps_position:=/dji_osdk_ros/gps_position attitude:=/dji_osdk_ros/attitude _gps_frame_id:=gps _base_link_frame_id:=body_FLU" << endl
       << "-------" << endl;
}

int main(int argc, char** argv)
{
  usage();
  ros::init(argc, argv, "relative_gnss_tf_broadcaster_node");
  ros::NodeHandle nh;

  ros::param::param<string>("~gps_frame_id", g_gps_frame_id, "gps");
  ros::param::param<string>("~base_link_frame_id", g_base_link_frame_id, "base_link");
  ros::param::param<bool>("~is_gps_frame_inversed", g_is_gps_frame_inversed, "true");
  ROS_INFO("[~gps_frame_id, ~base_link_frame_id]:[%s, %s]",
      g_gps_frame_id.c_str(), g_base_link_frame_id.c_str());
  if (g_is_gps_frame_inversed)
  {
    ROS_INFO("Quaternion of attitude_topic is inversed for rotation from ~base_link_frame_id to ~gps_frame_id(ENU)");
  }

  string gps_topic_name = "gps_position";
  string attitude_topic_name = "attitude";

  ros::Subscriber gps_sub = nh.subscribe(gps_topic_name, 1, gpsCB);
  ros::Subscriber attitude_sub = nh.subscribe(attitude_topic_name, 1, attitudeCB);

  ros::spin();
  return 0;
}
