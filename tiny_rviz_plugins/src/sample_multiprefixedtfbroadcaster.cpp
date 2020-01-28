/**
 * @file sample_multiprefixedtfbroadcaster.cpp
 * @brief sample code for multi_prefixed_tf_broadcaster_node
 * @detail Publish "tiny_rviz_plugins/MultiRobotStateDisplay", where different URDFs are used
 */

#include <ros/ros.h>
#include "tiny_rviz_plugins/MultiRobotStateDisplay.h"
#include <tf2_ros/static_transform_broadcaster.h>

using namespace std;
using namespace tiny_rviz_plugins;


// publish nao and pepper's state
MultiRobotStateDisplay makeMsg()
{
  MultiRobotStateDisplay msg;
  auto stamp = ros::Time::now();

  msg.header.frame_id = "world";
  msg.header.stamp = stamp;

  geometry_msgs::TransformStamped nao_tf, pepper_tf;
  nao_tf.header = msg.header;
  nao_tf.child_frame_id = "mptb_0/base_link";
  nao_tf.transform.translation.x = -1.0;
  nao_tf.transform.rotation.w = 1.0;
  pepper_tf.header = msg.header;
  pepper_tf.child_frame_id = "mptb_1/base_link";
  pepper_tf.transform.translation.x = +1.0;
  pepper_tf.transform.rotation.w = 1.0;

  msg.transforms_to_root.push_back(nao_tf);
  msg.transforms_to_root.push_back(pepper_tf);

  sensor_msgs::JointState nao_js, pepper_js;
  nao_js.header = msg.header;
  pepper_js.header = msg.header;

  nao_js.name = {"HeadYaw", "HeadPitch", "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
  "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll", "LShoulderPitch",
  "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand", "RShoulderPitch", "RShoulderRoll",
  "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand", "RFinger23", "RFinger13", "RFinger12", "LFinger21",
  "LFinger13", "LFinger11", "RFinger22", "LFinger22", "RFinger21", "LFinger12", "RFinger11", "LFinger23",
  "LThumb1", "RThumb1", "RThumb2", "LThumb2"};

  for (int i = 0; i < nao_js.name.size(); ++i)
  {
    nao_js.position.push_back((rand()%360)*M_PI/180.);
  }

  pepper_js.name = {"HeadYaw", "HeadPitch", "HipRoll", "HipPitch", "KneePitch", "LShoulderPitch", "LShoulderRoll",
  "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand", "RShoulderPitch", "RShoulderRoll", "RElbowYaw",
  "RElbowRoll", "RWristYaw", "RHand", "RFinger41", "LFinger42", "RFinger12", "LFinger33", "RFinger31",
  "LFinger21", "RFinger32", "LFinger13", "LFinger32", "LFinger11", "RFinger22", "RFinger13", "LFinger22",
  "RFinger21", "LFinger41", "LFinger12", "RFinger23", "RFinger11", "LFinger23", "LFinger43", "RFinger43",
  "RFinger42", "LFinger31", "RFinger33", "LThumb1", "RThumb2", "RThumb1", "LThumb2", "WheelFL", "WheelB",
  "WheelFR"};

  for (int i = 0; i < pepper_js.name.size(); ++i)
  {
    pepper_js.position.push_back((rand()%360)*M_PI/180.);
  }
  msg.joint_states.push_back(nao_js);
  msg.joint_states.push_back(pepper_js);

  return msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_prefixedtfbroadcaster");
  ros::NodeHandle nh;

  // Broadcast static TF for useful
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_trans;

  static_trans.header.stamp = ros::Time::now();
  static_trans.header.frame_id = "world";
  static_trans.child_frame_id = "base_link";
  static_trans.transform.rotation.w = 1;
  static_broadcaster.sendTransform(static_trans);

  ros::Rate loop(1);
  ros::Publisher mptb_pub = nh.advertise<MultiRobotStateDisplay>("/multi_robot_state", 1);

  ROS_INFO("Start publishing");
  while (ros::ok())
  {
    auto msg = makeMsg();
    mptb_pub.publish(msg);

    loop.sleep();
  }

  return 0;
}
