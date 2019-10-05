/**
 * @file multi_robot_state_process.h
 */
/// @note robot_state_publisher/joint_state_listenerを複数対応にする

#ifndef MULTI_ROBOT_STATE_PUBLISHER_H
#define MULTI_ROBOT_STATE_PUBLISHER_H

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf2_ros/transform_broadcaster.h>
//#include "multi_robot_state_publisher/MultiRobotState.h"
#include "tiny_rviz_plugins/MultiRobotStateDisplay.h"

typedef std::map<std::string, urdf::JointMimicSharedPtr> MimicMap;

//namespace multi_robot_state_publisher
namespace tiny_rviz_plugins
{
using namespace robot_state_publisher;

// readMRSAndBroadcast部分をここにまとめる?
//class MRSBroadcaster
//{
//    
//};

class MultiRobotStatePublisher //MultiRobotModelDisplayListerner?
{
public:
  MultiRobotStatePublisher(const KDL::Tree& tree, const MimicMap& m,
                           const urdf::Model& model=urdf::Model());
  ~MultiRobotStatePublisher();
  void setTfPrefix(std::string tf_prefix); //外部のライブラリで使うように

protected:
  virtual void callbackMRS(
          //const multi_robot_state_publisher::MultiRobotStateDisplayConstPtr& state);
          const MultiRobotStateDisplayConstPtr& state);
  virtual void callbackFixedJoint(const ros::TimerEvent& e);
  void readMRSAndBroadcast(
          //const multi_robot_state_publisher::MultiRobotStateDisplayConstPtr& state);
          const MultiRobotStateDisplayConstPtr& state);

  std::string tf_prefix_; //preprefix?
  ros::Duration publish_interval_;
  robot_state_publisher::RobotStatePublisher state_publisher_;
  ros::Subscriber mrmd_sub_;
  ros::Timer timer_;
  ros::Time last_callback_time_;
  //std::map<std::string, ros::Time> last_publish_time_; // 複数？
  std::vector<std::map<std::string, ros::Time>> last_publish_times_;
  MimicMap mimic_;
  bool use_tf_static_;
  bool ignore_timestamp_;
  int robot_num_; //msg見てロボットの数を記録．tf_prefixにする
  tf2_ros::TransformBroadcaster br_;
};

} // namespace multi_robot_state_publisher
#endif
