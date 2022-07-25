#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


if __name__ == "__main__":
    rospy.init_node("load_path")

    file_path = rospy.get_param("~path")
    pub = rospy.Publisher("/path", Path, queue_size=10)

    path = Path()
    path.header.frame_id = "/map"

    with open(file_path, "r") as f:
        lines = f.readlines()

    for line in lines:
        x, y, z = line.split()

        pose = PoseStamped()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1

        path.poses.append(pose)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pub.publish(path)

        rate.sleep()

    rospy.spin()
