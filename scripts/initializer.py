#!/usr/bin/env python

import rospy
import numpy as np

from library_agv import Map, AGV, Person
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Imu


def initialize_map():
    Map.data = OccupancyGrid()
    Map.data.info.resolution = Map.res
    Map.data.info.width = 600
    Map.data.info.height = 360
    Map.data.info.origin.position.x = 0.0
    Map.data.info.origin.position.y = 0.0
    Map.data.info.origin.position.z = 0.0
    Map.data.info.origin.orientation.x = 0.0
    Map.data.info.origin.orientation.y = 0.0
    Map.data.info.origin.orientation.z = 0.0
    Map.data.info.origin.orientation.w = 0.0
    Map.data.data = []

    np_map = np.zeros([Map.data.info.height, Map.data.info.width])
    np_map = np_map.flatten()
    Map.data.data = np_map

    Map.initial_data = Map.data

    print("Map Generated")


def initialize_yaw():
    rpy = rospy.wait_for_message('/imu', Imu)
    if rpy.orientation.x >= 0:
        AGV.yaw_offset = rpy.orientation.x
        print("Initialized Yaw with Offset")
    else:
        AGV.yaw_offset = rpy.orientation.x
        print("Initialized Yaw with Negative Offset")

    # else:
    #     print("IMU values inconsistent. Check IMU.")
    #     rospy.signal_shutdown(" ")
