import rospy
import time
import sys
import numpy as np
import copy

from rospy.numpy_msg import numpy_msg

from nav_msgs.msg import *
from std_msgs.msg import *

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point32

from marvelmind_nav.msg import hedge_pos_ang

from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker
from darknet_ros_msgs.msg import BoundingBoxes
from nav_msgs.msg import OccupancyGrid

from marvelmind_nav.msg import hedge_pos_ang
from darknet_ros_msgs.msg import BoundingBoxes
from collections import namedtuple

from update_markers import *
from path_planning import *
from path_tracking import *
from initializer import *


class AGV:
    x1 = x2 = 0
    y1 = 1.9
    y2 = 1.7
    yaw = 0
    x = 0 + 0.38 / 2 * np.cos(np.deg2rad(yaw))
    y = 1.8 + 0.38 / 2 * np.sin(np.deg2rad(yaw))
    yaw_offset = np.zeros((1, 5))
    steering_angle = 0

    def __init__(self, x, y, x1, y1, x2, y2, yaw, yaw_offset, steering_angle):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.x1 = x1  # Reading from left Marvelmind Beacon
        self.y1 = y1
        self.x2 = x2  # Reading from right Marvelmind Beacon
        self.y2 = y2
        self.yaw_offset = yaw_offset
        self.steering_angle = steering_angle


class Person:
    x = 2.2
    y = 1.37
    detected = 0

    def __init__(self, x, y, detected):
        self.x = x
        self.y = y
        self.detected = detected


class Map:
    initial_data = OccupancyGrid
    data = OccupancyGrid()
    goal_x = 0
    goal_y = 0
    res = 0.01
    path = np.zeros([2, 1000])
    p_x = 0
    p_y = 0
    score = 0

    def __init__(self, res, data, initial_data, goal_x, goal_y, path, p_x, p_y, score):
        self.res = res
        self.initial_data = initial_data  # Initial map will be saved
        self.data = data  # Updated map
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.path = path
        self.p_x = p_x
        self.p_y = p_y
        self.score = score


# def update_goal():
#     if Map.score > 24:
#         rospy.signal_shutdown("Score = 24. End of experiment.")
#     if (AGV.x - Map.goal_x)*(AGV.x - Map.goal_x) + (AGV.y - Map.goal_y)*(AGV.y - Map.goal_y) < 0.3:
#         Map.score += 1
#         print("Goal. Score = {}".format(Map.score))
#
#         Map.goal_x = np.random.randint(100, 500)/100
#         Map.goal_y = np.random.randint(100, 300)/100
#         print("New goal = {}, {}".format(Map.goal_x, Map.goal_y))


def update_goal():
    buff = np.transpose(np.array([[1, 0.9], [2.5, 0.5], [4, 0.5], [5, 0.9], [5, 2.7], [4, 3.1], [2.5, 3.1], [1, 2.7]]))
    # buff = np.array([[3.3, 2.3, 2, 3],
    #                  [1.8, 1.5, 1.1, 0.8]])
    # buff = np.array([[2, 3, 5],
    #                  [2.2, 1, 2]])

    if not Map.score:
        Map.goal_x = buff[0, Map.score]
        Map.goal_y = buff[1, Map.score]
    if Map.score > 8:
        rospy.signal_shutdown("Laps complete. End of experiment.")
    if (AGV.x - Map.goal_x) * (AGV.x - Map.goal_x) + (AGV.y - Map.goal_y) * (AGV.y - Map.goal_y) < 0.1:
        Map.goal_x = buff[0, Map.score]
        Map.goal_y = buff[1, Map.score]
        print("New goal = {}, {}".format(Map.goal_x, Map.goal_y))

        Map.score = Map.score + 1


def euler_to_quat(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]
