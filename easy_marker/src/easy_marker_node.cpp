/**
 * @file easy_marker_node.cpp
 */

#include <ros/ros.h>
#include "easy_marker/easy_marker.h"
#include <visualization_msgs/MarkerArray.h>
using namespace std;
using namespace easy_marker;
using namespace visualization_msgs;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "easy_marker_node");
  vector<Marker> markers;
  markers.push_back(makeMarkerARROWTemplate());
  markers.push_back(makeMarkerCUBETemplate());
  markers.push_back(makeMarkerSPHERETemplate());
  vector<geometry_msgs::Point> points;
  for (int i = 0; i < 3; i++)
  {
    geometry_msgs::Point p;
    p.y = i*0.1;
    points.push_back(p);
  }
  vector<string> color_names = {"red", "green", "blue"};
  markers.push_back(makeMarkerPOINTSTemplate(points, color_names));

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
