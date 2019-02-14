/**
 * @file easy_marker.h
 */

#pragma once
#include <visualization_msgs/Marker.h>

namespace easy_marker
{
visualization_msgs::Marker makeMarkerTemplate(std::string preset_type);
} // namespace easy_marker
