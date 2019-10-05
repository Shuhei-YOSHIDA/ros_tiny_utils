/**
 * @file sample_multirobotstatedisplay.cpp
 * @brief sample code for RViz plugin "MultiRobotStateDisplay"
 * @detail Try this with a node to publish "joint_states(sensor_msgs::JointState)" and Urdf where root of model is "base_link"
 */

#include <ros/ros.h>
#include <tiny_rviz_plugins/MultiRobotStateDisplay.h>
#include <tf2_ros/static_transform_broadcaster.h>

ros::Publisher mrsd_pub;

void jsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  int states_num = 7;
  double radius = 1.;
  tiny_rviz_plugins::MultiRobotStateDisplay mrsd_msg;
  mrsd_msg.header.stamp = ros::Time::now();
  mrsd_msg.header.frame_id = "world";

  for (int i = 0; i < states_num; i++)
  {
    double theta = (2*M_PI/states_num)*(double)i;
    double x = radius*sin(theta);
    double y = radius*cos(theta);
    double z = 0.;

    geometry_msgs::TransformStamped trans;
    trans.header.stamp = ros::Time::now();
    trans.header.frame_id = "world";
    trans.child_frame_id = std::to_string(i)+"/base_link";
    trans.transform.translation.x = x;
    trans.transform.translation.y = y;
    trans.transform.translation.z = z;
    trans.transform.rotation.w = 1.;

    sensor_msgs::JointState js_msg = *msg;
    for (int j = 0; j < js_msg.position.size(); j++)
    {
      js_msg.position[j] *= (double)i/states_num;
    }

    mrsd_msg.transforms_to_root.push_back(trans);
    mrsd_msg.joint_states.push_back(js_msg);
  }

  mrsd_pub.publish(mrsd_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_multirobotstatedisplay");
  ros::NodeHandle nh;

  // Broadcast static TF for useful
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_trans;

  static_trans.header.stamp = ros::Time::now();
  static_trans.header.frame_id = "world";
  static_trans.child_frame_id = "base_link";
  static_trans.transform.rotation.w = 1;
  static_broadcaster.sendTransform(static_trans);

  mrsd_pub = nh.advertise<tiny_rviz_plugins::MultiRobotStateDisplay>("multirobotstatedisplay", 1);
  auto js_sub = nh.subscribe("joint_states", 1, jsCallback);

  ros::spin();

  return 0;
}
