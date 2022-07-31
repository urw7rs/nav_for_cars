#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import threading

import rospy
import tf

from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDrive

from nav_for_cars.stanley_controller import StanleyController, State


class PathTracker:
    def __init__(
        self,
        target_speed=2.0,
        steer_gain=0.5,
        speed_proportional_gain=1.0,
        wheel_base=1.6,
        max_degrees=28,
    ):
        """
        Args:
            k (float): look forward gain
            Lfc (float): [m] look-ahead distance
            Kp (float): speed proportional gain
            WB (float): [m] wheel base of vehicle
        """

        self.target_speed = target_speed
        self.max_degrees = max_degrees

        self.controller = StanleyController(steer_gain, wheel_base)

        self.state = None

        self.target_path = None
        self.target_idx = 0

        self.lock = threading.Lock()

        rospy.Subscriber("/path", Path, self.load_path)
        rospy.Subscriber("/odom", Odometry, self.load_odom)

        self.pub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size=10)

        self.listener = tf.TransformListener()
        self.listener.waitForTransform(
            "/odom",
            "/map",
            rospy.Time(0),
            rospy.Duration(5),
        )

    def load_path(self, msg):
        """Load local path

        Args:
            msg (Path): nav_msgs/Path message containg local path
        """
        path = []

        for pose in msg.poses:
            pose.header.frame_id = "map"

            pose = self.listener.transformPose("odom", pose)

            x = pose.pose.position.x
            y = pose.pose.position.y

            path.append([x, y])

        path = np.array(path)

        diff = path[:-1] - path[1:]
        dx = diff[:, 0]
        dy = diff[:, 1]

        yaw = np.arctan2(dy, dx).tolist()
        if len(yaw):
            yaw.append(0.0)
        else:
            yaw = [0.0]
        yaw = np.array([yaw]).T

        self.target_path = np.concatenate((path, yaw), axis=1)

        self.follow_path(self.state, self.target_path)

    def load_odom(self, msg):
        """Load odometry data

        Args:
            msg (Odometry): nav_msgs/Odometry message containing current location
        """

        pose = msg.pose.pose

        x = pose.position.x
        y = pose.position.y
        yaw = pose.orientation.z

        twist = msg.twist.twist
        speed = twist.linear.x

        self.state = State(x=x, y=y, yaw=yaw, v=speed)

        self.follow_path(self.state, self.target_path)

    def follow_path(self, state, path_coordinates):
        """Update command basd on vehicle state and refrence path

        Args:
            coordinates (List[float]): x-coordinate
            yaw (float): yaw
            speed (float): speed in m/s
            path_coordinates (List[List[float]]): list of path coordinates (List[float])
        """

        if path_coordinates is None:
            return None

        if state is None:
            return None

        if self.lock.locked():
            return None

        with self.lock:
            path_x = self.target_path[:, 0]
            path_y = self.target_path[:, 1]
            path_yaw = self.target_path[:, 2]

            angle, self.target_idx = self.controller.get_command(
                self.state, path_x, path_y, path_yaw, self.target_idx
            )

            angle = np.clip(angle, -self.max_degrees, self.max_degrees)

            msg = AckermannDrive()
            msg.speed = self.target_speed
            msg.steering_angle = angle

            self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("follow_path")

    target_speed = rospy.get_param("~speed", 5 / 3.6)
    steer_gain = rospy.get_param("~steer_gain", 0.1)
    speed_gain = rospy.get_param("~speed_gain", 1.0)
    wheel_base = rospy.get_param("~wheel_base", 1.6)

    node = PathTracker(
        target_speed=target_speed,
        steer_gain=steer_gain,
        speed_proportional_gain=speed_gain,
        wheel_base=wheel_base,
    )

    rospy.spin()
