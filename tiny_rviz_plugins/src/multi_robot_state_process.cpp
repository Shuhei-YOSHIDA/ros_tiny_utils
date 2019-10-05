/**
 * @file multi_robot_state_process.cpp
 * @detail This code is based on a part of https://github.com/ros/robot_state_publisher
 */

#include "multi_robot_state_process.h"
using namespace tiny_rviz_plugins;
using namespace std;
using namespace ros;
using namespace KDL;

MultiRobotStatePublisher::MultiRobotStatePublisher(const KDL::Tree& tree, const MimicMap& m,
                                                   const urdf::Model& model)
  : state_publisher_(tree, model), mimic_(m), robot_num_(0)
{
  NodeHandle n_tilde("~");
  NodeHandle n;

  // set publish frequency
  double publish_freq;
  n_tilde.param("publish_frequency", publish_freq, 50.0);
  // set whether to use the /tf_static latched static transform broadcaster
  n_tilde.param("use_tf_static", use_tf_static_, false); // false is for rviz plugin
  // ignore_timestamp_ == true, joins_states messages are accepted, no matter their timestamp
  n_tilde.param("ignore_timestamp", ignore_timestamp_, false);
  // get the tf_prefix parameter from the closest namespace
  std::string tf_prefix_key;
  n_tilde.searchParam("tf_prefix", tf_prefix_key);
  n_tilde.param(tf_prefix_key, tf_prefix_, std::string(""));
  publish_interval_ = ros::Duration(1.0/max(publish_freq, 1.0));

  // Setting tcpNoNelay tells the subscriber to ask publishers that connect
  // to set TCP_NODELAY on their side. This prevents some joint_state messages
  // from being bundled together, increasing the latency of one of the messages.
  ros::TransportHints transport_hints;
  transport_hints.tcpNoDelay(true);

  // subscribe to MultiRobotStateDisplay message
  std::string topic_name = "multi_robot_state"; ///@todo This name can only be used at current codes
  mrmd_sub_ = n.subscribe(topic_name, 1,
                          &MultiRobotStatePublisher::callbackMRS, this, transport_hints);

  // trigger to publish fixed joints
  // if using static transform broadcaster, this will be a oneshot trigger and only run once
  timer_ = n_tilde.createTimer(publish_interval_,
                               &MultiRobotStatePublisher::callbackFixedJoint, this,
                               use_tf_static_);
}

MultiRobotStatePublisher::~MultiRobotStatePublisher()
{

}

void MultiRobotStatePublisher::setTfPrefix(string tf_prefix)
{
  tf_prefix_ = tf_prefix;
}


void MultiRobotStatePublisher::callbackFixedJoint(const ros::TimerEvent& e)
{
  (void)e;
  for (int i = 0; i < robot_num_; i++)
  {
    state_publisher_.publishFixedTransforms(tf_prefix_+"/"+to_string(i), use_tf_static_);
  }
}

void MultiRobotStatePublisher::callbackMRS(const MultiRobotStateDisplayConstPtr& state)
{
  readMRSAndBroadcast(state);
}

void MultiRobotStatePublisher::readMRSAndBroadcast(const MultiRobotStateDisplayConstPtr& state)
{
  if (state->joint_states.size() != state->transforms_to_root.size())
  {
    ROS_ERROR("Multi Robot state publisher ignored an invalid MultiRobotModelDisplay message");
  }
  for (auto&& js : state->joint_states)
  {
    if (js.name.size() != js.position.size())
    {
      if (js.position.empty())
      {
        const int throttleSeconds = 300;
        ROS_WARN_THROTTLE(throttleSeconds,
                "Robot state publisher ignored a JointState message about joint(s) "
                "\"%s\"(,...) whose position member was empty. This message will "
                "not reappear for %d seconds.", js.name[0].c_str(),
                throttleSeconds);
      }
      else
      {
        ROS_ERROR("Robot state publisher ignored an invalid JointState message");
      }
      return;
    }
  }

  robot_num_ = state->joint_states.size();

  if (state->joint_states.size() != last_publish_times_.size())
  {
    last_publish_times_.clear();
    last_publish_times_.resize(state->joint_states.size());
  }

  // check if we moved backwards in time (e.g. when playing a bag file)
  ros::Time now = ros::Time::now();
  if (last_callback_time_ > now)
  {
    // force re-publish of joint transforms
    ROS_WARN("Moved backwards in time (probably because ROS clock was reset), re-publishing joint transforms!");
    //last_publish_time_.clear();
    last_publish_times_.clear();
    last_publish_times_.resize(state->joint_states.size());
  }

  for (int robo_num = 0; robo_num < state->joint_states.size(); robo_num++)
  {
    sensor_msgs::JointState js = state->joint_states[robo_num];
    geometry_msgs::TransformStamped tf = state->transforms_to_root[robo_num];

    ros::Duration warning_threshold(30.0);
    if ((state->header.stamp + warning_threshold) < now)
    {
      ROS_WARN_THROTTLE(10, "Received JointState is %f seconds old.",
                        (now - state->header.stamp).toSec());
    }
    last_callback_time_ = now;
  
    // determine least recently published joint
    ros::Time last_published = now;
    for (unsigned int i = 0; i < js.name.size(); i++)
    {
      ros::Time t = last_publish_times_[robo_num][js.name[i]];
      last_published = (t < last_published) ? t : last_published;
    }
    // note: if a joint was seen for the first time,
    //       then last_published is zero.
  
    // check if we need to publish
    if (ignore_timestamp_ || state->header.stamp >= last_published + publish_interval_)
    {
      // get joint positions from state message
      map<string, double> joint_positions;
      for (unsigned int i = 0; i < js.name.size(); i++)
      {
        joint_positions.insert(make_pair(js.name[i],
                                         js.position[i]));
      }
  
      for (MimicMap::iterator i = mimic_.begin(); i != mimic_.end(); i++)
      {
        if(joint_positions.find(i->second->joint_name) != joint_positions.end())
        {
          double pos = joint_positions[i->second->joint_name] * i->second->multiplier
                       + i->second->offset;
          joint_positions.insert(make_pair(i->first, pos));
        }
      }
  
      state_publisher_.publishTransforms(joint_positions, state->header.stamp,
                                         tf_prefix_+"/"+to_string(robo_num));
  
      // store publish time in joint map
      for (unsigned int i = 0; i < js.name.size(); i++)
      {
        last_publish_times_[robo_num][js.name[i]] = state->header.stamp;
      }

      // broadcast each tf of all states
      geometry_msgs::TransformStamped tfs = tf;
      tfs.header.stamp = state->header.stamp;
      br_.sendTransform(tfs);
    }
  }
}
