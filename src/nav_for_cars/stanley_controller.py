"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
import numpy as np


class State(object):
    """
    Class representing the state of a vehicle.

    Args:
        x (float): x-coordinate
        y (float): y-coordinate
        yaw (float): yaw angle
        v (float): speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super(State, self).__init__()

        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    Args:
        angle (float): Angle in radians

    Returns:
        (float) angle normalized to [-pi, pi] in radians
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


class StanleyController:
    def __init__(self, control_gain=0.5, wheel_base=1.6):
        self.control_gain = control_gain
        self.wheel_base = wheel_base

    def calc_target_index(self, state, path_x, path_y):
        """
        Compute index in the trajectory list of the target.

        Args:
            state: (State object)
            path_x: [float]
            path_y: [float]

        Returns:
            (int, float)
        """

        # Calc front axle position
        fx = state.x + self.wheel_base * np.cos(state.yaw)
        fy = state.y + self.wheel_base * np.sin(state.yaw)

        # Search nearest point index
        dx = [fx - x for x in path_x]
        dy = [fy - y for y in path_y]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [
            -np.cos(state.yaw + np.pi / 2),
            -np.sin(state.yaw + np.pi / 2),
        ]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle

    def get_command(self, state, path_x, path_y, path_yaw, last_target_idx):
        """Stanley steering control.

        Args:
            state (State object): current state
            path_x ([float]): x coordinates of reference path
            path_y ([float]): y coordinates of reference path
            path_yaw ([float]): list of yaw values of reference path
            last_target_idx (int):

        Returns:
            (float, int)
        """

        # Calc front axle position
        fx = state.x + self.wheel_base * np.cos(state.yaw)
        fy = state.y + self.wheel_base * np.sin(state.yaw)

        # Search nearest point index
        dx = fx - path_x
        dy = fy - path_y
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [
            -np.cos(state.yaw + np.pi / 2),
            -np.sin(state.yaw + np.pi / 2),
        ]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        if last_target_idx >= target_idx:
            current_target_idx = last_target_idx
        else:
            current_target_idx = target_idx

        # theta_e corrects the heading error
        theta_e = normalize_angle(path_yaw[current_target_idx] - state.yaw)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(self.control_gain * error_front_axle, state.v)
        # Steering control
        delta = theta_e + theta_d

        return delta, current_target_idx
