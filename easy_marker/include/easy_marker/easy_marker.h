/**
 * @file easy_marker.h
 */

#pragma once
#include <visualization_msgs/Marker.h>
#include "color_names/color_names.h"

namespace easy_marker
{
// For template
visualization_msgs::Marker makeMarkerTemplate(int preset_type);

// Particular markers
visualization_msgs::Marker makeMarkerARROWTemplate(double scale=1.0, std::string color_name="red", std::string frame_id="base_link");
visualization_msgs::Marker makeMarkerCUBETemplate(std::string frame_id="base_link");
visualization_msgs::Marker makeMarkerSPHERETemplate(std::string frame_id="base_link");
visualization_msgs::Marker makeMarkerCYLINDERTemplate(std::string frame_id="base_link");
visualization_msgs::Marker makeMarkerLINE_STRIPTemplate(std::string frame_id="base_link");
visualization_msgs::Marker makeMarkerLINE_LISTTemplate(std::string frame_id="base_link");
visualization_msgs::Marker makeMarkerCUBE_LISTTemplate(std::string frame_id="base_link");
visualization_msgs::Marker makeMarkerSPHERE_LISTTemplate(std::string frame_id="base_link");
visualization_msgs::Marker makeMarkerPOINTSTemplate(std::string frame_id="base_link");
visualization_msgs::Marker makeMarkerTEXT_VIEW_FACINGTemplate(std::string frame_id="base_link");
visualization_msgs::Marker makeMarkerMESH_RESOURCETemplate(std::string frame_id="base_link");
visualization_msgs::Marker makeMarkerTRIANGLE_LISTTemplate(std::string frame_id="base_link");

// Fundamental method
visualization_msgs::Marker makeMarkerTemplate
 (
    std::string frame_id,
    std::string ns,
    int id,
    int type,
    int action,
    geometry_msgs::Pose pose,
    geometry_msgs::Vector3 scale,
    std_msgs::ColorRGBA color,
    ros::Duration lifetime,
    bool frame_locked,
    const std::vector<geometry_msgs::Point>& points,
    const std::vector<std_msgs::ColorRGBA>& colors,
    std::string text,
    std::string mesh_resource,
    bool mesh_use_embedded_materials
  )
{
  visualization_msgs::Marker mrk_msg;
  mrk_msg.header.frame_id = frame_id;
  mrk_msg.ns = ns;
  mrk_msg.id = id;
  mrk_msg.type = type;
  mrk_msg.action = action;
  mrk_msg.pose = pose;
  mrk_msg.scale = scale;
  mrk_msg.color = color;
  mrk_msg.lifetime = lifetime;
  mrk_msg.frame_locked = frame_locked;
  mrk_msg.points = points;
  mrk_msg.colors = colors;
  mrk_msg.text = text;
  mrk_msg.mesh_resource = mesh_resource;
  mrk_msg.mesh_use_embedded_materials = mesh_use_embedded_materials;

  return mrk_msg;
}

visualization_msgs::Marker makeMarkerTemplate(std::string preset_type)
{
  visualization_msgs::Marker mrk_msg;

  return mrk_msg;
}

visualization_msgs::Marker makeMarkerARROWTemplate(double scale, std::string color_name, std::string frame_id)
{
  if (scale <= 0) scale = 1.0;
  std::vector<geometry_msgs::Point> points;
  std::vector<std_msgs::ColorRGBA> colors;
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  geometry_msgs::Vector3 scale3;
  scale3.x = scale3.y = scale3.z = 0.1;
  scale3.x*=scale;
  scale3.y*=scale;
  scale3.z*=scale;
  std_msgs::ColorRGBA color = color_names::makeColorMsg(color_name);

  return makeMarkerTemplate
 (
    frame_id,
    "",
    0,
    visualization_msgs::Marker::ARROW,
    visualization_msgs::Marker::ADD,
    pose,
    scale3,
    color,
    ros::Duration(),
    false,
    points,
    colors,
    "",
    "",
    false
  );
}
//visualization_msgs::Marker makeMarkerCUBETemplate(std::string frame_id);
//visualization_msgs::Marker makeMarkerSPHERETemplate(std::string frame_id);
//visualization_msgs::Marker makeMarkerCYLINDERTemplate(std::string frame_id);
//visualization_msgs::Marker makeMarkerLINE_STRIPTemplate(std::string frame_id);
//visualization_msgs::Marker makeMarkerLINE_LISTTemplate(std::string frame_id);
//visualization_msgs::Marker makeMarkerCUBE_LISTTemplate(std::string frame_id);
//visualization_msgs::Marker makeMarkerSPHERE_LISTTemplate(std::string frame_id);
//visualization_msgs::Marker makeMarkerPOINTSTemplate(std::string frame_id);
//visualization_msgs::Marker makeMarkerTEXT_VIEW_FACINGTemplate(std::string frame_id);
//visualization_msgs::Marker makeMarkerMESH_RESOURCETemplate(std::string frame_id);
//visualization_msgs::Marker makeMarkerTRIANGLE_LISTTemplate(std::string frame_id);



} // namespace easy_marker
