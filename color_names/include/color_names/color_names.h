/**
 * @file color_names.h
 * @brief Preset Color names
 */

#pragma once
#include <std_msgs/ColorRGBA.h>

namespace color_names
{
std_msgs::ColorRGBA makeColorMsg(std::string preset_name, double alpha=1.0);

std::map<std::string, std::array<double, 3>> color_name_dict
{
  //{"COLOR_NAME", {R, G, B}} //template
  {"RED", {1., 0., 0.}},
  {"ERROR", {0, 0, 0}}
};

std_msgs::ColorRGBA makeColorMsg(std::string preset_name, double alpha)
{
  std_msgs::ColorRGBA c_msg;
  c_msg.a = alpha;
  if (c_msg.a < 0.) c_msg.a = 0.;
  if (c_msg.a > 1.) c_msg.a = 1.;

  auto found_itr = color_name_dict.find(preset_name);
  if (found_itr != color_name_dict.end())
  {
    c_msg.r = found_itr->second[0];
    c_msg.g = found_itr->second[1];
    c_msg.b = found_itr->second[2];
  }
  else
  {
    c_msg.r = 0;
    c_msg.g = 0;
    c_msg.b = 0;
  }

  return c_msg;
}

} // namespace color_names
