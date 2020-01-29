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
