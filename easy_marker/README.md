easy_marker
====
**ROS package**

Header library to prepare visualization_msgs/Marker message easily.

Some functions to output templates are used by this package.

And, Cad file of coordinate marker is provided, which is in `meshes` directory.

## How To Use
**check src/easy_marker_node.cpp**

```c++
#include "easy_marker/easy_marker.h"

double scale = 1.0;
std::string color_name = "red";
std::string frame_id = "base_link";

// All arguments in functions have default value.
visualization_msgs::Marker arrow_marker = easy_marker::makeMarkerARROWTemplate(scale, color_name, frame_id);
// Above line is same to below lines.
// arrow_marker.header.frame_id = frame_id;
// arrow_marker.ns = "";
// arrow_marker.id = 0;
// arrow_marker.type = visualization_msgs::Marker::ARROW;
// arrow_marker.action = visualization_msgs::Marker::ADD;
// arrow_marker.pose.orientation.w = 1.0;
// arrow_marker.scale.x = 0.1*scale;
// arrow_marker.scale.y = 0.01*scale;
// arrow_marker.scale.z = 0.01*scale;
// arrow_marker.color = color_names::makeColorMsg(color_name);
// arrow_marker.lifetime = ros::Duration();
// arrow_marker.frame_locked = false;
// arrow_marker.points = {};
// arrow_marker.colors = {};
// arrow_marker.text = "";
// arrow_marker.mesh_resource = "";
// arrow_marker.mesh_use_embedded_materials = false;

// More functions are provided.
```

## Sample
![easy_marker example](https://raw.githubusercontent.com/Shuhei-YOSHIDA/ros_tiny_utils/master/easy_marker/images/easy_marker_sample.png)
