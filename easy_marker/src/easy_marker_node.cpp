/**
 * @file easy_marker_node.cpp
 */

#include <ros/ros.h>
#include "easy_marker/easy_marker.h"
#include <visualization_msgs/MarkerArray.h>
using namespace easy_marker;
using namespace visualization_msgs;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "easy_marker_node");
  std::vector<visualization_msgs::Marker> markers;
  markers.push_back(makeMarkerARROWTemplate());
  markers.push_back(makeMarkerCUBETemplate());
  markers.push_back(makeMarkerSPHERETemplate());

  for (int i = 0; i < markers.size(); i++)
  {
    markers[i].pose.position.x = 0.2*i;
    markers[i].id = i;
  }

  //markers[ 3] = makeMarkerCYLINDERTemplate();
  //markers[ 4] = makeMarkerLINE_STRIPTemplate();
  //markers[ 5] = makeMarkerLINE_LISTTemplate();
  //markers[ 6] = makeMarkerCUBE_LISTTemplate();
  //markers[ 7] = makeMarkerSPHERE_LISTTemplate();
  //markers[ 8] = makeMarkerPOINTSTemplate();
  //markers[ 9] = makeMarkerTEXT_VIEW_FACINGTemplate();
  //markers[10] = makeMarkerMESH_RESOURCETemplate();
  //markers[11] = makeMarkerTRIANGLE_LISTTemplate();

  ros::NodeHandle nh;
  ros::Rate loop(1);
  ros::Publisher mrks_pub = nh.advertise<MarkerArray>("marker_templates", 1);
  MarkerArray mrks_msg;
  mrks_msg.markers = markers;
  while (ros::ok())
  {
    for (int i = 0; i < mrks_msg.markers.size(); i++)
      mrks_msg.markers[i].header.stamp = ros::Time::now();

    mrks_pub.publish(mrks_msg);
  }

  return 0;
}
