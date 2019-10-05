/**
 * @file multi_robot_state_process.h
 * @detail This code is based on a part of https://github.com/ros/robot_state_publisher
 */

#ifndef TINY_RVIZ_PLUGINS_MULTI_ROBOT_STATE_PROCESS_H
#define TINY_RVIZ_PLUGINS_MULTI_ROBOT_STATE_PROCESS_H

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tiny_rviz_plugins/MultiRobotStateDisplay.h"

typedef std::map<std::string, urdf::JointMimicSharedPtr> MimicMap;

namespace tiny_rviz_plugins
{
using namespace robot_state_publisher;

class MultiRobotStatePublisher
{
public:
  MultiRobotStatePublisher(const KDL::Tree& tree, const MimicMap& m,
                           const urdf::Model& model=urdf::Model());
  ~MultiRobotStatePublisher();
  void setTfPrefix(std::string tf_prefix);

protected:
  virtual void callbackMRS(
          const MultiRobotStateDisplayConstPtr& state);
  virtual void callbackFixedJoint(const ros::TimerEvent& e);
  void readMRSAndBroadcast(
          const MultiRobotStateDisplayConstPtr& state);

  std::string tf_prefix_;
  ros::Duration publish_interval_;
  robot_state_publisher::RobotStatePublisher state_publisher_;
  ros::Subscriber mrmd_sub_;
  ros::Timer timer_;
  ros::Time last_callback_time_;
  std::vector<std::map<std::string, ros::Time>> last_publish_times_;
  MimicMap mimic_;
  bool use_tf_static_;
  bool ignore_timestamp_;
  int robot_num_;
  tf2_ros::TransformBroadcaster br_;
};

} // namespace tiny_rviz_plugins
#endif
