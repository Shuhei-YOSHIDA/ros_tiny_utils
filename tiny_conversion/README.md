tiny_conversion
====
Conversion from something to something for ROS

## rosbag2text.py
Convert contents of rosbag to text files.
The files is assigned to each topic.

This utility is mainly for small rosbag file, which has simple data.

### How to use
```bash
$ # ./rosbag2text.py path_to_rosbag [output_directory]
$ ./rosbag2text.py test.bag ~/
```

You can get text files which represent messages of each topic.
If output_directory is not set, `/tmp` is used as default.

The output files are written by YAML style.
The root of YAML is set to `messages'.
Hence, for example, you can treat the file by `yq` command.
```bash
$ cat output_text.txt | yq .messages # all messages
$ cat output_text.txt | yq .messages[1] # 2nd message
```

## path_to_text_marker
Convert a topic of `nav_msgs/Path` into another topic of `visualization_msgs::MarkerArray`.
The markers represents positions of the path by using text markers.

It is able to change font size, and so on by using ros parameters.
Please check `tiny_conversion/params/font_data.yaml`.

### How to use
```bash
$ # sample launch file
$ roslaunch tiny_conversion sample_nav_msgs_path_to_text_marker.launch
```

## resolve_path.h
Resolve url as string.

```cpp
#include "tiny_conversion/resolve_path.h"
string url = "package:://tiny_conversion/package.xml";
string resolved_path = tiny_conversion::resolvePathUrlStr(url);
// ex. resolved_path: "/home/your_user/catkin_ws/src/ros_tiny_utils/tiny_conversion/package.xml"
```
