#!/usr/bin/env python

from path_tracking.frenet_optimal_planning import frenet_optimal_planning

import rospy

from nav_msgs.msg import Path, PoseStamped


class PathPlanner:
    """Class

    Args:
    """

    def __init__(self):
        rospy.Subscriber("/global_path", Path, self.load_path)

        self.c_speed = 0.0
        self.c_d = 2.0
        self.c_d_d = 0.0
        self.c_c_dd = 0.0
        self.s0 = 0.0

        self.pub = rospy.Publisher("/local_path", Path, queue_size=10)

    def load_path(self, msg):
        """Load global path

        Args:
            msg (Path): nav_msgs/Path message containg global path
        """

        self.plan_path()

    def load_obstacle(self, msg):
        """Load obstacle

        Args:
            msg (): obstacles
        """

        self.plan_path()

    def plan_path(self):
        """

        Args:
            msg (): obstacles
        """
        path = frenet_optimal_planning(
            self.csp, self.s0, self.c_speed, self.c_d, self.c_d_d, self.c_d_dd, self.ob
        )

        self.s0 = path.s[1]
        self.c_d = path.d[1]
        self.c_d_d = path.d_d[1]
        self.c_d_dd = path.d_dd[1]
        self.c_speed = path.s_d[1]

        msg = Path()

        for x, y in zip(path.x[1:], path.y[1:]):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y

            msg.poses.append(pose)

        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("plan_path")

    node = PathPlanner()

    rospy.spin()
