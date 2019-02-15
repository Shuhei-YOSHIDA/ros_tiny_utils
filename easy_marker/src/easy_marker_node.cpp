/**
 * @file easy_marker_node.cpp
 */

#include "easy_marker/easy_marker.h"
using namespace easy_marker;

int main(int argc, char** argv)
{
  std::vector<visualization_msgs::Marker> markers(12);
  markers[ 0] = makeMarkerARROWTemplate();
  //markers[ 1] = makeMarkerCUBETemplate();
  //markers[ 2] = makeMarkerSPHERETemplate();
  //markers[ 3] = makeMarkerCYLINDERTemplate();
  //markers[ 4] = makeMarkerLINE_STRIPTemplate();
  //markers[ 5] = makeMarkerLINE_LISTTemplate();
  //markers[ 6] = makeMarkerCUBE_LISTTemplate();
  //markers[ 7] = makeMarkerSPHERE_LISTTemplate();
  //markers[ 8] = makeMarkerPOINTSTemplate();
  //markers[ 9] = makeMarkerTEXT_VIEW_FACINGTemplate();
  //markers[10] = makeMarkerMESH_RESOURCETemplate();
  //markers[11] = makeMarkerTRIANGLE_LISTTemplate();

  return 0;
}
