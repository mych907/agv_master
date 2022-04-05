#!/usr/bin/env python

# import necessary libraries

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

from zed_interfaces.msg import ObjectsStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud
from zed_interfaces.msg import Object
from darknet_ros_msgs.msg import BoundingBoxes

pub_map = rospy.Publisher('/map', OccupancyGrid, latch=True, queue_size=10)
pub_flag = rospy.Publisher('/flag', Bool, latch=True, queue_size=10)
pub_ego_pose = rospy.Publisher('/ego_pose', Float32MultiArray, latch=True, queue_size=10)
pub_ego_yaw = rospy.Publisher('/ego_yaw', Float32, latch=True, queue_size=10)
pub_person_pose = rospy.Publisher('/person_pose', Float32MultiArray, latch=True, queue_size=10)
pub_path = rospy.Publisher('/path', Float32MultiArray, latch=True, queue_size=10)
pub_steer = rospy.Publisher('/steering_angle', Float32, latch=True, queue_size=10)
pub_steer_mapped = rospy.Publisher('/steer_mapped', Float32, latch=True, queue_size=10)
pub_desired_path = rospy.Publisher('desired_path', Float32MultiArray, latch=True, queue_size=10)
pub_imu_vel = rospy.Publisher('/ego_vel', Float32MultiArray, latch=True, queue_size=10)
pub_path_points = rospy.Publisher('/visual', PointCloud, latch=True, queue_size=10)


#
res = 0.01
person_x = 2.2
person_y = 1.37
yaw = 0
x = 0.03 + 0.36 * np.cos(np.deg2rad(yaw))
y = 1.8 + 0.36 * np.sin(np.deg2rad(yaw))
x1 = 0.04
y1 = 1.7
x2 = 0.04
y2 = 1.9
zed_x = 0.34
zed_y = 1.8
person_x_abs = 2
person_y_abs = 1.7
person_detected = 0
test_map = OccupancyGrid()
test_map_zero = 0
np_map = np.zeros((360, 600))
pos_offset = np.zeros((2, 2))
path = np.zeros(166*2)
p_x = 0
p_y = 0
yaw = 0
yaw_offset = np.zeros((1,2))
imu_velx = 0
imu_vely = 0

steering_angle = 0
steer_mapped = 0
path_data = np.zeros((2, 166))


def initialize_map():
    global test_map
    global test_map_zero
    global np_map
    test_map.info.resolution = res
    test_map.info.width = 600
    test_map.info.height = 360
    test_map.info.origin.position.x = 0.0
    test_map.info.origin.position.y = 0.0
    test_map.info.origin.position.z = 0.0
    test_map.info.origin.orientation.x = 0.0
    test_map.info.origin.orientation.y = 0.0
    test_map.info.origin.orientation.z = 0.0
    test_map.info.origin.orientation.w = 0.0
    test_map.data = []
    area = test_map.info.width * test_map.info.height
    for i in range(0, area):
        test_map.data.append(0)

    np_map = np.array(test_map.data)
    np_map = np_map.reshape(test_map.info.height, test_map.info.width)
    for i in range(231 - 60, 231):
        for j in range(127, 127 + 19):
            np_map[j, i] = 99
    np_map = np_map.flatten()
    test_map.data = np_map

    test_map_zero = copy.copy(test_map)

    print("Map Generated")


def talker():
    global person_x_abs, person_y_abs, x, y, np_map, test_map, path, p_x, p_y

    msg = Float32MultiArray()
    msg.data = []
    msg.data = [person_x_abs, person_y_abs]
    pub_person_pose.publish(msg)

    pose = np.array([x, y])
    msg = Float32MultiArray()
    msg.data = pose
    pub_ego_pose.publish(msg)

    msg = Float32()
    msg.data = yaw
    pub_ego_yaw.publish(msg)

    pub_map.publish(test_map)

    msg = Float32MultiArray()
    msg.data = [path[21]/100 + x , path[21+166]/100 + y] # Where we stopped last time! ADDED EGO POSE
    pub_path.publish(msg)

    msg = Float32()
    msg.data = steering_angle
    pub_steer.publish(msg)

    msg.data = steer_mapped
    pub_steer_mapped.publish(msg)

    msg = Float32MultiArray()
    msg.data = np.array([p_x/100 + x, p_y/100 + y ])
    pub_desired_path.publish(msg)

    msg = Float32MultiArray()
    msg.data = [imu_velx, imu_vely]
    pub_imu_vel.publish(msg)

    msg = PointCloud()
    for i in range (166):
        temp = Point32()
        temp.x = path[i]/100 + x
        temp.y = path[i+166]/100 + y
        msg.points.append(temp)
    msg.header.frame_id = "map"
    pub_path_points.publish(msg)


def listener():
    rate = rospy.Rate(20)

    global x
    global y
    global x2
    global y2
    global person_x
    global person_y
    global person_detected
    global yaw
    global test_map, np_map

    while not rospy.is_shutdown():
        rospy.Subscriber('/hedge_pos_ang', hedge_pos_ang, callback_update_pos)
        rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, callback_update_pos_zed)
        # rospy.Subscriber("/zed2i/zed_node/obj_det/objects", ObjectsStamped, callback_update_person_pos)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback_update_person_pos)
        rospy.Subscriber("/imu", Imu, callback_update_yaw)

        # person_x_abs = person_x# - 0.34
        # person_y_abs = person_y# - 1.8

        test_map = copy.copy(test_map_zero)
        np_map = np.array(test_map.data)
        np_map = np_map.reshape(test_map.info.height, test_map.info.width)

        # zed pos
        # for i in range(int(x/res)-34, int(x/res)+5):
        #    for j in range(int(y/res)-9, int(y/res)+9):
        #        np_map[j,i]=50

        # marvel pos
        for i in range(int(x / res -38), int(x / res) ):
            for j in range(int(y / res) - 9, int(y / res) + 9):
                np_map[j, i] = 50

        # person pos
        if person_detected == 1:
            np_map = occupancy_function(person_x, person_y)

        a_star(x, y)

        callback_tracker()

        if person_x < 0:
            pub_flag.publish(True)
        else:
            pub_flag.publish(False)

        np_map_flat = np_map.flatten()
        test_map.data = np_map_flat

        talker()
        rate.sleep()


def initialize_pos():
    global pos_offset
    global yaw_offset
    global x, y, yaw
    global x1, y1
    global x2, y2

    rospy.Subscriber('/hedge_pos_ang', hedge_pos_ang, simple_update_pos)
    rospy.Subscriber('/imu', Imu, simple_update_yaw)
    time.sleep(0.5)
    for i in range(2):
        rospy.Subscriber('/hedge_pos_ang', hedge_pos_ang, simple_update_pos)
        rospy.Subscriber('/imu', Imu, simple_update_yaw)
        x = (x1 + x2) / 2
        y = (y1 + y2) / 2
        yaw_offset[0,i] = yaw
        pos_offset[0,i] = x
        pos_offset[1,i] = y
        time.sleep(0.5)

    pos_offset = np.mean(pos_offset, axis=1)
    yaw_offset = np.mean(yaw_offset)

    print("Initialized Position and Yaw with Offset")


def simple_update_pos(pos):
    global x1, y1
    global x2, y2
    if pos.address == 12:
        x1 = pos.x_m
        y1 = pos.y_m

    if pos.address == 13:
        x2 = pos.x_m
        y2 = pos.y_m


def callback_update_pos(pos):
    global x, y
    global x1, y1
    global x2, y2
    global pos_offset
    global yaw

    if pos.address == 12:
        x1 = pos.x_m
        y1 = pos.y_m

    if pos.address == 13:
        x2 = pos.x_m
        y2 = pos.y_m

    x = (x1+x2)/2 + 0.36 * np.cos(np.deg2rad(yaw))# - pos_offset[0] + 0.34
    y = (y1+y2)/2 + 0.36 * np.sin(np.deg2rad(yaw))# - pos_offset[1] + 1.8


def simple_update_yaw(rpy):
    global yaw
    yaw = rpy.orientation.x


def callback_update_yaw(rpy):
    global yaw, yaw_offset, imu_velx, imu_vely
    if rpy.orientation.x >= 0:
        yaw = -(rpy.orientation.x - yaw_offset)
    else:
        yaw = -(rpy.orientation.x - yaw_offset + 360)
    imu_velx = rpy.angular_velocity.x
    imu_vely = rpy.angular_velocity.y


def callback_update_pos_zed(pos):
    global zed_x
    global zed_y
    zed_x = pos.pose.position.x
    zed_y = pos.pose.position.y


def callback_update_person_pos(detected):
    global person_x
    global person_y
    global person_detected
    if detected.bounding_boxes:
        for i in range(len(detected.bounding_boxes)):
            if detected.bounding_boxes[i].Class == "person":
                person_detected = 1
                # obj = detected.bounding_boxes[i]
                # person_x = obj.position[0] + 0.3
                # person_y = obj.position[1] + 1.8
                person_x = 4.3 # 2.7
                person_y = 1.7
                continue
    # else:
    #     person_x = 2.2
    #     person_y = 1.37
    #     person_detected = 0


def occupancy_function(x_, y_):
    global res, np_map
    d_safe = 1.5
    lane_width = 0.9
    res_ = 1 / res
    a = 1
    for i in range(int(x_ * res_ - d_safe * res_), int(x_ * res_ + d_safe * res_)):
        for j in range(int(y_ * res_ - lane_width / 2 * res_ * (np.cos(np.pi * (i - x_ * res_) / d_safe / res_))),
                       int(y_ * res_ + lane_width / 2 * res_ * (np.cos(np.pi * (i - x_ * res_) / d_safe / res_)))):
            np_map[j, i] = 100
    return np_map


def a_star(ego_x_abs, ego_y_abs):
    global res
    global np_map
    global path

    res_ = 1 / res
    goalx = 5 * res_
    goaly = 1.8 * res_
    min_i = 0
    min_j = 0
    ego_x = int(round(ego_x_abs * res_))
    ego_y = int(round(ego_y_abs * res_))
    for n in range(0, 166):
        min_ = 1000000
        for i in range(ego_x - 1, ego_x + 2):
            for j in range(ego_y - 1, ego_y + 2):
                buffer = np.sqrt(np.square(goalx - i) + np.square(goaly - j))
                buffer = buffer + np_map[j, i] * buffer
                if buffer < min_:
                    min_ = copy.copy(buffer)
                    min_i = i
                    min_j = j
        ego_x = min_i
        ego_y = min_j

        path[n] = ego_x - round(ego_x_abs * res_)
        path[166 + n] = ego_y - round(ego_y_abs * res_)


def callback_tracker():
    global steering_angle
    global p_x, p_y
    global path
    global path_data
    global steer_mapped
    res_ = 1/res
    ld = 1.1 * 0.5 * res_  # [m]. Look ahead distance, 0.5s preview time
    L = 0.15 * res_  # wheel base
    yaw_ = np.deg2rad(yaw)

    path_data = np.reshape(path, [2, 166])
    rear_center = [-L / 2 * np.cos(yaw_), -L / 2 * np.sin(yaw_)]
    min = 1000000
    min_n = 0
    for n in range(0, 100):
        buffer = np.sqrt(np.square(rear_center[0] - path_data[0, n]) + np.square(rear_center[1] - path_data[1, n]))
        if abs(buffer - ld) < min:
            min = abs(buffer - ld)
            min_n = n

    p_x = path_data[0, min_n]
    p_y = path_data[1, min_n]

    alpha = np.arctan(p_y / p_x) - yaw_
    steering_angle = np.arctan(2 * L * np.sin(alpha) / ld)
    steer_mapped = (steering_angle*180/np.pi + 2.2)/23


if __name__ == '__main__':
    try:
        rospy.init_node('AGV_node', anonymous=True)
        initialize_map()
        initialize_pos()
        listener()

    except rospy.ROSInterruptException:
        pass
