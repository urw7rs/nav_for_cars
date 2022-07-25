#!/usr/bin/env python

import math
import pyproj

import rospy
import tf

from tf.transformations import quaternion_from_euler

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


class GpsOdom:
    def __init__(self):
        self.prev_ready = False

        rospy.Subscriber("/ublox/fix", NavSatFix, self.publish_odom)
        self.to_utm = pyproj.Proj(
            proj="utm", zone=52, ellps="WGS84", preserve_units=False
        )

        self.pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.broadcaster = tf.TransformBroadcaster()

    def publish_odom(self, msg):
        x, y = self.to_utm(msg.longitude, msg.latitude)

        # skip if previous x, y values are unavailable
        if self.prev_ready is False:
            self.prev_x = x
            self.prev_y = y

            self.prev_ready = True

            return

        dx = x - self.prev_x
        dy = y - self.prev_y
        yaw = math.atan2(dy, dx)

        self.prev_x = x
        self.prev_y = y

        current_time = rospy.Time.now()

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

        self.broadcaster.sendTransform(
            (x, y, 0.0), odom_quat, current_time, "odom", "map"
        )

        msg = Odometry()

        msg.child_frame_id = "odom"
        msg.header.stamp = current_time
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y

        quat = quaternion_from_euler(0.0, 0.0, yaw)

        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        self.pub.publish(msg)

        self.broadcaster.sendTransform(
            (x, y, 0),
            quat,
            rospy.Time.now(),
            "odom",
            "map",
        )


if __name__ == "__main__":
    rospy.init_node("gps_odom")
    node = GpsOdom()
    rospy.spin()
