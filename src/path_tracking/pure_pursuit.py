"""
Path tracking simulation with pure pursuit steering and PID speed control.
author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)
"""
import numpy as np
import math

# Parameters
k = 0.1  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
WB = 2.9  # [m] wheel base of vehicle


class Vehicle:
    """Class representing the vehicle of a Vehicle

    Args:
        x (float): x-coordinate
        y (float): y-coordinate
        yaw (float): yaw-angle
        v (float): speed
        WB (float): wheel base in meters
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, WB=1.6):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - (WB / 2) * math.cos(yaw)
        self.rear_y = self.y - (WB / 2) * math.sin(yaw)


def distance_from_vehicle(vehicle, x, y):
    """Calculate distance from a vehicle to a point

    Args:
        vehicle (Vehicle): vehicle
        point_x (float): x-coordinate
        point_y (float): y-coordinate

    Returns:
        float: distance between a car and a point
    """

    dx = vehicle.rear_x - x
    dy = vehicle.rear_y - y

    return np.hypot(dx, dy)


def proportional_control(target, current, Kp=1.0):
    a = Kp * (target - current)

    return a


class ReferencePath:
    """Class representing reference path

    Args:
        path_X (List[float]): list of x-coordinates of the target path
        path_Y (List[float]): list of y-coordinates of the target path
        k (float): look forward gain
        Lfc (float): look-ahed distance in meters
    """

    def __init__(self, cx, cy, k=0.1, Lfc=2.0):
        self.path = np.array([cx, cy]).T
        self.nearest_index = None

        self.k = k
        self.Lfc = Lfc

    def search_target_index(self, vehicle):
        """Search target index

        Args:
            vehicle (Vehicle): vehicle to use
        """

        point = np.array([vehicle.rear_x, vehicle.rear_y])

        distances = np.linalg.norm(point - self.path, ord=2, axis=1)

        i = np.argmin(distances)

        # get the index of the closest point
        # where distance is close to look ahead distance
        # and index is late
        closest_i = np.argsort(np.abs(distances[i:] - self.L))[0]

        target_point = self.path[closest_i, :]

        # To speed up nearest point search, doing it at only first time.
        if self.nearest_index is None:
            # search nearest point index
            dx = vehicle.rear_x - self.cx
            dy = vehicle.rear_y - self.cy
            d = np.hypot(dx, dy)

            self.nearest_index = np.argmin(d)
        else:
            i = self.nearest_index
            x = self.cx[i]
            y = self.cy[i]

            distance = distance_from_vehicle(vehicle, x, y)
            next_distance = np.inf

            x_list = self.cx[i:].tolist()
            y_list = self.cy[i:].tolist()

            for x, y in zip(x_list, y_list):
                pass

            while distance > next_distance and i < len(self.cx):
                distance = next_distance
                i += 1

                x = self.cx[i]
                y = self.cy[i]

                distance = distance_from_vehicle(vehicle, x, y)

                distance = next_distance

            self.nearest_index = i

        Lf = self.k * vehicle.v + self.Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > vehicle.calc_distance(self.cx[i], self.cy[i]):
            if (i + 1) >= len(self.cx):
                break  # not exceed goal
            i += 1

        return i, Lf


def pure_pursuit_steer_control(vehicle, target_path, target_point):
    i, Lf = target_path.search_target_index(vehicle)

    if target_index >= i:
        i = target_index

    if i < len(target_path.cx):
        tx = target_path.cx[i]
        ty = target_path.cy[i]
    else:  # toward goal
        tx = target_path.cx[-1]
        ty = target_path.cy[-1]
        ind = len(target_path.cx) - 1

    alpha = math.atan2(ty - vehicle.rear_y, tx - vehicle.rear_x) - vehicle.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind
