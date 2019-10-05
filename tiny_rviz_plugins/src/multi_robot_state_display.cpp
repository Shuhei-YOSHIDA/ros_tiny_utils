/**
 * @file multi_robot_state_display.cpp
 */
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <tinyxml.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

//#include <tf/transform_listener.h>
//#include <tf2_ros/transform_listener.h>

#include <rviz/display_context.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/tf_link_updater.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>

#include "multi_robot_state_display.h"
#include <std_msgs/String.h>

namespace tiny_rviz_plugins
{
using namespace rviz;
typedef std::map<std::string, urdf::JointMimicSharedPtr> MimicMap;

void linkUpdaterStatusFunction( StatusProperty::Level level,
        const std::string& link_name,
        const std::string& text,
        MultiRobotModelDisplaySubs* display )
{
  display->setStatus( level, QString::fromStdString( link_name ), QString::fromStdString( text ));
}

MultiRobotModelDisplaySubs::MultiRobotModelDisplaySubs()
  : has_new_transforms_( false )
  , has_new_urdf_( false )
  , time_since_last_transform_( 0.0f )
{
  visual_enabled_property_ = new Property( "Visual Enabled", true,
          "Whether to display the visual representation of the robot.",
          this, SLOT( updateVisualVisible() ));

  collision_enabled_property_ = new Property( "Collision Enabled", false,
          "Whether to display the collision representation of the robot.",
          this, SLOT( updateCollisionVisible() ));

  update_rate_property_ = new FloatProperty( "Update Interval", 0,
          "Interval at which to update the links, in seconds. "
          " 0 means to update every update cycle.",
          this );
  update_rate_property_->setMin( 0 );

  alpha_property_ = new FloatProperty( "Alpha", 1,
          "Amount of transparency to apply to the links.",
          this, SLOT( updateAlpha() ));
  alpha_property_->setMin( 0.0 );
  alpha_property_->setMax( 1.0 );

  robot_description_property_ = new StringProperty( "Robot Description", "robot_description",
          "Name of the parameter to search for to load the robot description.",
          this, SLOT( updateRobotDescription() ));

  tf_prefix_property_ = new StringProperty( "TF Prefix", "",
          "Robot Model normally assumes the link name is the same as the tf frame name. "
          " This option allows you to set a prefix.  Mainly useful for multi-robot situations.",
          this, SLOT( updateTfPrefix() ));
}

MultiRobotModelDisplaySubs::~MultiRobotModelDisplaySubs()
{
  if ( initialized() )
  {
    // nothing to do
  }
}

void MultiRobotModelDisplaySubs::onInitialize()
{
  robot_num_ = 0;

  MFDClass::onInitialize();
  updateVisualVisible();
  updateCollisionVisible();
  updateAlpha();

}

void MultiRobotModelDisplaySubs::updateAlpha()
{
  for (auto&& r : robots_) r->setAlpha( alpha_property_->getFloat() );
  context_->queueRender();
}

void MultiRobotModelDisplaySubs::updateRobotDescription()
{
  if( isEnabled() )
  {
      load();
      context_->queueRender();
  }
}

void MultiRobotModelDisplaySubs::updateVisualVisible()
{
  for (auto&& r : robots_)
    r->setVisualVisible( visual_enabled_property_->getValue().toBool() );
  context_->queueRender();
}

void MultiRobotModelDisplaySubs::updateCollisionVisible()
{
  for (auto&& r : robots_)
    r->setCollisionVisible( collision_enabled_property_->getValue().toBool() );
  context_->queueRender();
}

void MultiRobotModelDisplaySubs::updateTfPrefix()
{
  clearStatuses();
  context_->queueRender();
  mrd_publisher_ptr->setTfPrefix(tf_prefix_property_->getStdString());
}

void MultiRobotModelDisplaySubs::processMessage(
        const tiny_rviz_plugins::MultiRobotStateDisplay::ConstPtr& msg)
{
  int js_cnt = msg->joint_states.size();
  int tf_cnt = msg->transforms_to_root.size();
  if (js_cnt != tf_cnt || js_cnt == 0) // bad msg
  {
    setStatus( StatusProperty::Error, "TOPIC",
               "The num of msg.joint_states and msg.transforms_to_root are problem");
    robot_num_ = 0;
    return;
  }

  robot_num_ = js_cnt;
  if (robots_.size() != js_cnt || has_new_urdf_)
  {
    has_new_urdf_ = false;
    robots_.clear();
    for (int i = 0; i < js_cnt; i++)
    {
      auto mp = boost::make_shared<Robot>(
              scene_node_,
              context_,
              "Robot"+std::to_string(i)+": " + getName().toStdString(),
              this);
      mp->load( *descr_ );
      robots_.push_back(mp);
    }
  }

  int count = 0;
  for (auto&& r : robots_)
  {
    r->update( TFLinkUpdater( context_->getFrameManager(),
               boost::bind( linkUpdaterStatusFunction, _1, _2, _3, this ),
               tf_prefix_property_->getStdString()+"/"+std::to_string(count) ));
    count++;
  }
}

void MultiRobotModelDisplaySubs::load()
{
  std::string content;
  if( !update_nh_.getParam( robot_description_property_->getStdString(), content ))
  {
    std::string loc;
    if( update_nh_.searchParam( robot_description_property_->getStdString(), loc ))
    {
      update_nh_.getParam( loc, content );
    }
    else
    {
      clear();
      setStatus( StatusProperty::Error, "URDF",
              "Parameter [" + robot_description_property_->getString()
              + "] does not exist, and was not found by searchParam()" );
      return;
    }
  }

  if( content.empty() )
  {
    clear();
    setStatus( StatusProperty::Error, "URDF", "URDF is empty" );
    return;
  }

  if( content == robot_description_ )
  {
    return;
  }

  robot_description_ = content;

  TiXmlDocument doc;
  doc.Parse( robot_description_.c_str() );
  if( !doc.RootElement() )
  {
    clear();
    setStatus( StatusProperty::Error, "URDF", "URDF failed XML parse" );
    return;
  }

  urdf::Model model;
  if( !model.initXml( doc.RootElement() ))
  {
    clear();
    setStatus( StatusProperty::Error, "URDF", "URDF failed Model parse" );
    return;
  }
  descr_ = boost::make_shared<urdf::Model>(model);
  has_new_urdf_ = true;

  setStatus( StatusProperty::Ok, "URDF", "URDF parsed OK" );
  //robot_->load( descr );
  //robot_->update( TFLinkUpdater( context_->getFrameManager(),
  //            boost::bind( linkUpdaterStatusFunction, _1, _2, _3, this ),
  //            tf_prefix_property_->getStdString() ));
  // Do above code at callback of msg
  robots_.clear();

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    clear();
    //ROS_ERROR("Failed to extract kdl tree from xml robot description");
    setStatus( StatusProperty::Ok, "URDF", "Failed to extract kdl tree from URDF" );
    return;
  }

  MimicMap mimic;

  for(std::map< std::string, urdf::JointSharedPtr >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++)
  {
    if(i->second->mimic)
    {
      mimic.insert(make_pair(i->first, i->second->mimic));
    }
  }
  //mrd_publisher_ptr = std::make_unique<MultiRobotStatePublisher>(tree, mimic, model);
  mrd_publisher_ptr.reset(new MultiRobotStatePublisher(tree, mimic, model));
}

void MultiRobotModelDisplaySubs::onEnable()
{
  load();
  for (auto&& r : robots_) r->setVisible( true );
}

void MultiRobotModelDisplaySubs::onDisable()
{
  for (auto&& r : robots_) r->setVisible( false );
  clear();
}

void MultiRobotModelDisplaySubs::update( float wall_dt, float ros_dt )
{
  time_since_last_transform_ += wall_dt;
  float rate = update_rate_property_->getFloat();
  bool update = rate < 0.0001f || time_since_last_transform_ >= rate;

  if( has_new_transforms_ || update )
  {
    int count = 0;
    for (auto&& r : robots_)
    {
      r->update( TFLinkUpdater( context_->getFrameManager(),
                                boost::bind( linkUpdaterStatusFunction, _1, _2, _3, this ),
                                tf_prefix_property_->getStdString()+"/"+std::to_string(count) ));
      count++;
    }
    context_->queueRender();

    has_new_transforms_ = false;
    time_since_last_transform_ = 0.0f;
  }
}

void MultiRobotModelDisplaySubs::fixedFrameChanged()
{
  has_new_transforms_ = true;
}

void MultiRobotModelDisplaySubs::clear()
{
  for (auto&& r : robots_) r->clear();
  clearStatuses();
  robot_description_.clear();
  mrd_publisher_ptr = NULL;
}

void MultiRobotModelDisplaySubs::reset()
{
  MFDClass::reset();
  has_new_transforms_ = true;
}

} // namespace tiny_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( tiny_rviz_plugins::MultiRobotModelDisplaySubs, rviz::Display )
