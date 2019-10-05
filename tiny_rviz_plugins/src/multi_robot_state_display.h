/**
 * @file multi_robot_state_display.h
 * @brief RViz Plugin to take successive photo for multi-robot state
 * @detail This code is based on a part of https://github.com/ros/robot_state_publisher
 */
#ifndef TINY_RVIZ_PLUGINS_MULTI_ROBOT_STATE_DISPLAY_H
#define TINY_RVIZ_PLUGINS_MULTI_ROBOT_STATE_DISPLAY_H

#include <rviz/message_filter_display.h>
#include <OgreVector3.h>
#include <map>
#include <tf2_ros/transform_broadcaster.h>
#include "multi_robot_state_process.h"

namespace Ogre
{
class Entity;
class SceneNode;
}

namespace urdf
{
class Model;
}

namespace rviz
{
class Axes;
}

namespace rviz
{

class FloatProperty;
class Property;
class Robot;
class StringProperty;

} // namespace rviz

namespace tiny_rviz_plugins
{
using namespace rviz;
/**
 * @class MultiRobotModelDisplaySubs
 * @brief Uses one robot xml description to display the pieces of multi robots at the transforms broadcast by rosTF
 */
class MultiRobotModelDisplaySubs
    : public MessageFilterDisplay<MultiRobotStateDisplay>
{
Q_OBJECT
public:
  MultiRobotModelDisplaySubs();
  virtual ~MultiRobotModelDisplaySubs();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update(float wall_dt, float ros_dt);
  virtual void fixedFrameChanged();
  virtual void reset();

  void clear();

private Q_SLOTS:
  void updateVisualVisible();
  void updateCollisionVisible();
  void updateTfPrefix();
  void updateAlpha();
  void updateRobotDescription();
  void processMessage(const MultiRobotStateDisplayConstPtr& msg);

protected:
  /**
   * @brief Load a URDF from the ros-param named by our
   * "Robot Description" property, iterates through the links, and
   * loads any necessary models.
   */
  virtual void load();

  // Overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  //Robot* robot_; ///< Handles actually drawing the robot
  std::vector<boost::shared_ptr<Robot>> robots_;

  bool has_new_transforms_; ///< Callback sets this to tell our update function it needs to update the transforms

  float time_since_last_transform_;

  std::string robot_description_;
  boost::shared_ptr<urdf::Model> descr_;
  bool has_new_urdf_;
  int robot_num_; // To generate tf_prefix
  tf2_ros::TransformBroadcaster br_;

  Property* visual_enabled_property_;
  Property* collision_enabled_property_;
  FloatProperty* update_rate_property_;
  StringProperty* robot_description_property_;
  FloatProperty* alpha_property_;
  StringProperty* tf_prefix_property_;

  std::unique_ptr<MultiRobotStatePublisher> mrd_publisher_ptr;
};

} // namespace tiny_rviz_plugins
#endif
