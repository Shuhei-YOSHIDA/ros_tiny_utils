/**
 * @file fixed_state_broadcaster.h
 * @brief Broadcast only /tf_static
 * @note Inspired by robot_state_publisher (https://github.com/ros/robot_state_publisher)
 */

#ifndef INCLUDE_TINY_RVIZ_PLUGINS_FIXED_STATE_PUBLISHER_H
#define INCLUDE_TINY_RVIZ_PLUGINS_FIXED_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <tf/tf.h>
#include <urdf/model.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>

namespace fixed_state_publisher
{
class SegmentPair
{
    public:
          SegmentPair(const KDL::Segment& p_segment, const std::string& p_root, const std::string& p_tip):
                  segment(p_segment), root(p_root), tip(p_tip){}

            KDL::Segment segment;
              std::string root, tip;
};


class FixedStatePublisher
{
public:
  /** Constructor
   *    * \param tree The kinematic model of a robot, represented by a KDL Tree
   *       */
  FixedStatePublisher(const KDL::Tree& tree, const urdf::Model& model = urdf::Model());

  /// Destructor
  ~FixedStatePublisher(){};

  /** Publish transforms to tf
   * \param joint_positions A map of joint names and joint positions.
   * \param time The time at which the joint positions were recorded
   */
  virtual void publishTransforms(const std::map<std::string, double>& joint_positions, const ros::Time& time, const std::string& tf_prefix);
  virtual void publishFixedTransforms(const std::string& tf_prefix, bool use_tf_static = false);

protected:
  virtual void addChildren(const KDL::SegmentMap::const_iterator segment);

  std::map<std::string, SegmentPair> segments_, segments_fixed_;
  const urdf::Model& model_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

}
#endif /* ifndef INCLUDE_FIXED_STATE_PUBLISHER_H */
