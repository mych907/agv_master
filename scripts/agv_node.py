#!/usr/bin/env python

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
from library_agv import *


pub_map = rospy.Publisher('/map', OccupancyGrid, latch=True, queue_size=10)
pub_ego_pose = rospy.Publisher('/ego_pose', Float32MultiArray, latch=True, queue_size=10)
pub_ego_yaw = rospy.Publisher('/ego_yaw', Float32, latch=True, queue_size=10)
pub_person_pose = rospy.Publisher('/person_pose', Float32MultiArray, latch=True, queue_size=10)
pub_path = rospy.Publisher('/path', Float32MultiArray, latch=True, queue_size=10)
pub_steer = rospy.Publisher('/steering_angle', Float32, latch=True, queue_size=10)
pub_path_points = rospy.Publisher('/visual', PointCloud, latch=True, queue_size=10)
pub_marker_goal = rospy.Publisher('/marker_goal', Marker, latch=True, queue_size=10)
pub_marker_car = rospy.Publisher('/marker_car', Marker, latch=True, queue_size=10)
pub_steer_ = rospy.Publisher('/steering_', Float32, latch=True, queue_size=10)
pub_marker_direction = rospy.Publisher('/marker_direction', Marker, latch=True, queue_size=10)
# pub_imu_vel = rospy.Publisher('/ego_vel', Float32MultiArray, latch=True, queue_size=10)
# pub_flag = rospy.Publisher('/flag', Bool, latch=True, queue_size=10)


def listener():
    rospy.Subscriber('/hedge_pos_ang', hedge_pos_ang, callback_update_pos)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback_update_person_pos)
    rospy.Subscriber("/imu", Imu, callback_update_yaw)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        Map.data = Map.initial_data
        update_goal()
        np_map = np.array(Map.data.data)
        np_map = np_map.reshape(Map.data.info.height, Map.data.info.width)
        # Person pos
        if Person.detected == 1:
            np_map = lime_occupancy(np_map, Person.x, Person.y)

        # Path planning
        a_star(np_map)

        # Path tracking
        pure_pursuit()
        # stanley()

        np_map_flat = np_map.flatten()
        Map.data.data = np_map_flat

        talker()
        rate.sleep()


def talker():
    # msg = Float32MultiArray()
    # msg.data = []
    # msg.data = [person_x_abs, person_y_abs]
    # pub_person_pose.publish(msg)

    pub_map.publish(Map.data)

    pose = np.array([AGV.x, AGV.y])
    msg = Float32MultiArray()
    msg.data = pose
    pub_ego_pose.publish(msg)

    msg = Float32()
    msg.data = AGV.yaw
    pub_ego_yaw.publish(msg)

    msg = Float32()
    msg.data = AGV.steering_angle
    pub_steer.publish(msg)

    pub_steer_.publish(np.rad2deg(msg.data))

    msg = Float32MultiArray()
    msg.data = np.array([Map.p_x*Map.res + AGV.x, Map.p_y*Map.res + AGV.y])
    pub_path.publish(msg)

    # msg = Float32MultiArray()
    # msg.data = [imu_vel_x, imu_vel_y]
    # pub_imu_vel.publish(msg)

    msg = PointCloud()
    msg.header.frame_id = "map"
    for i in range(166):
        temp = Point32()
        temp.x = Map.path[0, i] / 100 + AGV.x
        temp.y = Map.path[1, i] / 100 + AGV.y
        msg.points.append(temp)
    pub_path_points.publish(msg)

    marker_goal = update_marker_goal()
    pub_marker_goal.publish(marker_goal)

    marker_car = update_marker_car()
    pub_marker_car.publish(marker_car)

    marker_direction = update_marker_direction()
    pub_marker_direction.publish(marker_direction)


def callback_update_pos(pos):
    if pos.address == 12:
        AGV.x1 = pos.x_m
        AGV.y1 = pos.y_m

    if pos.address == 13:
        AGV.x2 = pos.x_m
        AGV.y2 = pos.y_m

    AGV.x = (AGV.x1 + AGV.x2) / 2 + 0.36/2 * np.cos(np.deg2rad(AGV.yaw))
    AGV.y = (AGV.y1 + AGV.y2) / 2 + 0.36/2 * np.sin(np.deg2rad(AGV.yaw))


def callback_update_yaw(rpy):

    if AGV.yaw_offset > 0:
        if rpy.orientation.x >= 0:
            AGV.yaw = -(rpy.orientation.x - AGV.yaw_offset)
        else:
            AGV.yaw = -(rpy.orientation.x - AGV.yaw_offset + 360)
    else:
        if rpy.orientation.x <= 0:
            AGV.yaw = -(rpy.orientation.x - AGV.yaw_offset)
        else:
            AGV.yaw = -(rpy.orientation.x - AGV.yaw_offset - 360)


def callback_update_person_pos(detected):
    if detected.bounding_boxes:
        for i in range(len(detected.bounding_boxes)):
            if detected.bounding_boxes[i].Class == "person":
                Person.detected = 1
                Person.x = 4.3  # 2.7
                Person.y = 1.7
                continue


if __name__ == '__main__':
    try:
        rospy.init_node('AGV_node', anonymous=True, disable_signals=False)
        initialize_map()
        initialize_yaw()
        listener()

    except rospy.ROSInterruptException:
        pass
