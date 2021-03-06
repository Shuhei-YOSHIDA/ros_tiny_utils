#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def path_publisher():
    pub = rospy.Publisher('path', Path, queue_size=1, latch=True)
    rospy.init_node('sample_path_publisher')

    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.get_rostime()

    for i in range(6):
        rad = i*60*np.pi/180.
        r = 10.
        p = PoseStamped()
        p.header = path.header
        p.pose.position.x = r*np.cos(rad)
        p.pose.position.y = r*np.sin(rad)
        p.pose.position.z = 0
        p.pose.orientation.w = 1
        path.poses.append(p)

    pub.publish(path)

    rospy.spin()


if __name__ == "__main__":
    try:
        path_publisher()
    except rospy.ROSInterruptException:
        pass
