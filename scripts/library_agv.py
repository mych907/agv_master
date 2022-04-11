import numpy as np
from nav_msgs.msg import OccupancyGrid


class AGV:
    x1 = x2 = 0
    y1 = 1.9
    y2 = 1.7
    yaw = 0
    x = 0 + 0.38/2 * np.cos(np.deg2rad(yaw))
    y = 1.8 + 0.38/2 * np.sin(np.deg2rad(yaw))
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
    goal_x = 4  # 5
    goal_y = 1.8  # 1.8
    res = 0.01
    path = np.zeros([2, 166])
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
