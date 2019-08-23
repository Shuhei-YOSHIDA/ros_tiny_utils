/**
 * @file easy_marker.h
 */

#ifndef INCLUDE_EASY_MARKER_EASY_MARKER_H
#define INCLUDE_EASY_MARKER_EASY_MARKER_H

#include <visualization_msgs/Marker.h>
#include "color_names/color_names.h"

namespace easy_marker
{
/// Make template by visualization_msgs::Marker's enum for markers' type
inline visualization_msgs::Marker makeMarkerTemplate(int preset_type);

/// Make template of visualization_msgs::Marker::ARROW
inline visualization_msgs::Marker makeMarkerARROWTemplate(
    double scale=1.0, std::string color_name="red", std::string frame_id="base_link");
/// Make template of visualization_msgs::Marker::CUBE
inline visualization_msgs::Marker makeMarkerCUBETemplate(
    double scale=1.0, std::string color_name="red", std::string frame_id="base_link");
/// Make template of visualization_msgs::Marker::SPHERE
inline visualization_msgs::Marker makeMarkerSPHERETemplate(
    double scale=1.0, std::string color_name="red", std::string frame_id="base_link");
/// Make template of visualization_msgs::Marker::CYLINDER
inline visualization_msgs::Marker makeMarkerCYLINDERTemplate(
    double scale=1.0, std::string color_name="red", std::string frame_id="base_link");
/// Make template of visualization_msgs::Marker::LINE_STRIP
inline visualization_msgs::Marker makeMarkerLINE_STRIPTemplate(
    std::vector<geometry_msgs::Point> points={},
    std::vector<std::string> color_names={},
    double scale=1.0, std::string frame_id="base_link");
/// Make template of visualization_msgs::Marker::LINE_LIST
inline visualization_msgs::Marker makeMarkerLINE_LISTTemplate(
    std::vector<geometry_msgs::Point> points={},
    std::vector<std::string> color_names={},
    double scale=1.0, std::string frame_id="base_link");
/// Make template of visualization_msgs::Marker::CUBE_LIST
inline visualization_msgs::Marker makeMarkerCUBE_LISTTemplate(
    std::vector<geometry_msgs::Point> points={},
    std::vector<std::string> color_names={},
    double scale=1.0, std::string frame_id="base_link");
/// Make template of visualization_msgs::Marker::SPHERE_LIST
inline visualization_msgs::Marker makeMarkerSPHERE_LISTTemplate(
    std::vector<geometry_msgs::Point> points={},
    std::vector<std::string> color_names={},
    double scale=1.0, std::string frame_id="base_link");
/// Make template of visualization_msgs::Marker::POINTS
inline visualization_msgs::Marker makeMarkerPOINTSTemplate(
    std::vector<geometry_msgs::Point> points={},
    std::vector<std::string> color_names={},
    double scale=1.0, std::string frame_id="base_link");
/// Make template of visualization_msgs::Marker::TEXT_VIEW_FACING
inline visualization_msgs::Marker makeMarkerTEXT_VIEW_FACINGTemplate(
    std::string text="",
    double scale=1.0, std::string color_name="red", std::string frame_id="base_link");
/// Make template of visualization_msgs::Marker::MESH_RESOURCE
inline visualization_msgs::Marker makeMarkerMESH_RESOURCETemplate(
    std::string mesh_resource="package://easy_marker/meshes/xyz_marker.stl", bool mesh_use_embedded_materials=true,
    double scale=1.0, std::string color_name="red", std::string frame_id="base_link");
/// Make template of visualization_msgs::Marker::TRIANGLE_LIST
inline visualization_msgs::Marker makeMarkerTRIANGLE_LISTTemplate(
    std::vector<geometry_msgs::Point> points={},
    std::vector<std::string> color_names={},
    double scale=1.0, std::string frame_id="base_link");

/// Template for sibstutiting
inline visualization_msgs::Marker makeMarkerTemplate
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

visualization_msgs::Marker makeMarkerTemplate(int preset_type)
{
  visualization_msgs::Marker mrk_msg;
  switch (preset_type) {
    case visualization_msgs::Marker::ARROW:
      mrk_msg = makeMarkerARROWTemplate();
    break;
    case visualization_msgs::Marker::CUBE:
      mrk_msg = makeMarkerCUBETemplate();
    break;
    case visualization_msgs::Marker::SPHERE:
      mrk_msg = makeMarkerSPHERETemplate();
    break;
    case visualization_msgs::Marker::CYLINDER:
      mrk_msg = makeMarkerCYLINDERTemplate();
    break;
    case visualization_msgs::Marker::LINE_STRIP:
      mrk_msg = makeMarkerLINE_STRIPTemplate();
    break;
    case visualization_msgs::Marker::LINE_LIST:
      mrk_msg = makeMarkerLINE_LISTTemplate();
    break;
    case visualization_msgs::Marker::CUBE_LIST:
      mrk_msg = makeMarkerCUBE_LISTTemplate();
    break;
    case visualization_msgs::Marker::SPHERE_LIST:
      mrk_msg = makeMarkerSPHERE_LISTTemplate();
    break;
    case visualization_msgs::Marker::POINTS:
      mrk_msg = makeMarkerPOINTSTemplate();
    break;
    case visualization_msgs::Marker::TEXT_VIEW_FACING:
      mrk_msg = makeMarkerTEXT_VIEW_FACINGTemplate();
    break;
    case visualization_msgs::Marker::MESH_RESOURCE:
      mrk_msg = makeMarkerMESH_RESOURCETemplate();
    break;
    case visualization_msgs::Marker::TRIANGLE_LIST:
      mrk_msg = makeMarkerTRIANGLE_LISTTemplate();
    break;
    default:
      mrk_msg.action = visualization_msgs::Marker::DELETEALL;
    break;
  }
  return mrk_msg;
}

visualization_msgs::Marker makeMarkerARROWTemplate(
    double scale, std::string color_name, std::string frame_id)
{
  if (scale <= 0) scale = 1.0;
  std::vector<geometry_msgs::Point> points;
  std::vector<std_msgs::ColorRGBA> colors;
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  geometry_msgs::Vector3 scale3;
  scale3.x = 0.1;
  scale3.y = 0.01;
  scale3.z = 0.01;
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

visualization_msgs::Marker makeMarkerCUBETemplate(
    double scale, std::string color_name, std::string frame_id)
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
    visualization_msgs::Marker::CUBE,
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

visualization_msgs::Marker makeMarkerSPHERETemplate(
    double scale, std::string color_name, std::string frame_id)
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
    visualization_msgs::Marker::SPHERE,
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
visualization_msgs::Marker makeMarkerCYLINDERTemplate(
    double scale, std::string color_name, std::string frame_id)
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
    visualization_msgs::Marker::CYLINDER,
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
visualization_msgs::Marker makeMarkerLINE_STRIPTemplate(
    std::vector<geometry_msgs::Point> points,
    std::vector<std::string> color_names, double scale, std::string frame_id)
{
  std::vector<std_msgs::ColorRGBA> colors;
  for (auto&& c_name : color_names) colors.push_back(color_names::makeColorMsg(c_name));
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  geometry_msgs::Vector3 scale3;
  scale3.x = scale3.y = scale3.z = 0.1;
  scale3.x*=scale;
  scale3.y*=scale;
  scale3.z*=scale;
  std_msgs::ColorRGBA color;

  return makeMarkerTemplate
 (
    frame_id,
    "",
    0,
    visualization_msgs::Marker::LINE_STRIP,
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
visualization_msgs::Marker makeMarkerLINE_LISTTemplate(
    std::vector<geometry_msgs::Point> points,
    std::vector<std::string> color_names, double scale, std::string frame_id)
{
  std::vector<std_msgs::ColorRGBA> colors;
  for (auto&& c_name : color_names) colors.push_back(color_names::makeColorMsg(c_name));
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  geometry_msgs::Vector3 scale3;
  scale3.x = scale3.y = scale3.z = 0.1;
  scale3.x*=scale;
  scale3.y*=scale;
  scale3.z*=scale;
  std_msgs::ColorRGBA color;

  return makeMarkerTemplate
 (
    frame_id,
    "",
    0,
    visualization_msgs::Marker::LINE_LIST,
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
visualization_msgs::Marker makeMarkerCUBE_LISTTemplate(
    std::vector<geometry_msgs::Point> points,
    std::vector<std::string> color_names, double scale, std::string frame_id)
{
  std::vector<std_msgs::ColorRGBA> colors;
  for (auto&& c_name : color_names) colors.push_back(color_names::makeColorMsg(c_name));
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  geometry_msgs::Vector3 scale3;
  scale3.x = scale3.y = scale3.z = 0.1;
  scale3.x*=scale;
  scale3.y*=scale;
  scale3.z*=scale;
  std_msgs::ColorRGBA color;

  return makeMarkerTemplate
 (
    frame_id,
    "",
    0,
    visualization_msgs::Marker::CUBE_LIST,
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
visualization_msgs::Marker makeMarkerSPHERE_LISTTemplate(
    std::vector<geometry_msgs::Point> points,
    std::vector<std::string> color_names, double scale, std::string frame_id)
{
  std::vector<std_msgs::ColorRGBA> colors;
  for (auto&& c_name : color_names) colors.push_back(color_names::makeColorMsg(c_name));
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  geometry_msgs::Vector3 scale3;
  scale3.x = scale3.y = scale3.z = 0.1;
  scale3.x*=scale;
  scale3.y*=scale;
  scale3.z*=scale;
  std_msgs::ColorRGBA color;

  return makeMarkerTemplate
 (
    frame_id,
    "",
    0,
    visualization_msgs::Marker::SPHERE_LIST,
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
visualization_msgs::Marker makeMarkerPOINTSTemplate(
    std::vector<geometry_msgs::Point> points,
    std::vector<std::string> color_names, double scale, std::string frame_id)
{
  std::vector<std_msgs::ColorRGBA> colors;
  for (auto&& c_name : color_names) colors.push_back(color_names::makeColorMsg(c_name));
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  geometry_msgs::Vector3 scale3;
  scale3.x = scale3.y = scale3.z = 0.1;
  scale3.x*=scale;
  scale3.y*=scale;
  scale3.z*=scale;
  std_msgs::ColorRGBA color;

  return makeMarkerTemplate
 (
    frame_id,
    "",
    0,
    visualization_msgs::Marker::POINTS,
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
visualization_msgs::Marker makeMarkerTEXT_VIEW_FACINGTemplate(
    std::string text,
    double scale, std::string color_name, std::string frame_id)
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
    visualization_msgs::Marker::TEXT_VIEW_FACING,
    visualization_msgs::Marker::ADD,
    pose,
    scale3,
    color,
    ros::Duration(),
    false,
    points,
    colors,
    text,
    "",
    false
  );
}
visualization_msgs::Marker makeMarkerMESH_RESOURCETemplate(
    std::string mesh_resource, bool mesh_use_embedded_materials,
    double scale, std::string color_name, std::string frame_id)
{
  if (scale <= 0) scale = 1.0;
  std::vector<geometry_msgs::Point> points;
  std::vector<std_msgs::ColorRGBA> colors;
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  geometry_msgs::Vector3 scale3;
  scale3.x = scale3.y = scale3.z = 1.; // original size of mesh file
  scale3.x*=scale;
  scale3.y*=scale;
  scale3.z*=scale;
  std_msgs::ColorRGBA color = color_names::makeColorMsg(color_name);

  return makeMarkerTemplate
 (
    frame_id,
    "",
    0,
    visualization_msgs::Marker::MESH_RESOURCE,
    visualization_msgs::Marker::ADD,
    pose,
    scale3,
    color,
    ros::Duration(),
    false,
    points,
    colors,
    "",
    mesh_resource,
    mesh_use_embedded_materials
  );
}
visualization_msgs::Marker makeMarkerTRIANGLE_LISTTemplate(
    std::vector<geometry_msgs::Point> points,
    std::vector<std::string> color_names,
    double scale, std::string frame_id)
{
  std::vector<std_msgs::ColorRGBA> colors;
  for (auto&& c_name : color_names) colors.push_back(color_names::makeColorMsg(c_name));
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  geometry_msgs::Vector3 scale3;
  scale3.x = scale3.y = scale3.z = 1; // still used
  scale3.x*=scale;
  scale3.y*=scale;
  scale3.z*=scale;
  std_msgs::ColorRGBA color;

  return makeMarkerTemplate
 (
    frame_id,
    "",
    0,
    visualization_msgs::Marker::TRIANGLE_LIST,
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

} // namespace easy_marker

#endif
