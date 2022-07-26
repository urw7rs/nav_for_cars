#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import rospy
import tf

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


if __name__ == "__main__":
    rospy.init_node("load_path")

    file_path = rospy.get_param("~path")
    pub = rospy.Publisher("/path", Path, queue_size=10)
    broadcaster = tf.TransformBroadcaster()

    path = Path()
    path.header.frame_id = "/map"

    with open(file_path, "r") as f:
        lines = f.readlines()

    xys = []
    for line in lines:
        coords = line.split()

        x = float(coords[0])
        y = float(coords[1])

        xys.append([x, y])

        pose = PoseStamped()

        pose.header.frame_id = "map"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1

        path.poses.append(pose)

    xys = np.array(xys)
    mean_xy = xys.mean(axis=0)

    x = mean_xy[0]
    y = mean_xy[1]

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        broadcaster.sendTransform(
            (x, y, 0.0),
            (0, 0, 0, 1),
            rospy.Time.now(),
            "path",
            "map",
        )

        pub.publish(path)

        rate.sleep()
