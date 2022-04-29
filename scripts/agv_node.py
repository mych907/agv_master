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
from collections import namedtuple
from library_agv import AGV, Person, Map

pub_map = rospy.Publisher('/map', OccupancyGrid, latch=True, queue_size=10)
pub_ego_pose = rospy.Publisher('/ego_pose', Float32MultiArray, latch=True, queue_size=10)
pub_ego_yaw = rospy.Publisher('/ego_yaw', Float32, latch=True, queue_size=10)
pub_person_pose = rospy.Publisher('/person_pose', Float32MultiArray, latch=True, queue_size=10)
pub_path = rospy.Publisher('/path', Float32MultiArray, latch=True, queue_size=10)
pub_steer = rospy.Publisher('/steering_angle', Float32, latch=True, queue_size=10)
pub_path_points = rospy.Publisher('/visual', PointCloud, latch=True, queue_size=10)
pub_marker_goal = rospy.Publisher('/marker_goal', Marker, latch=True, queue_size=10)
pub_marker_car = rospy.Publisher('/marker_car', Marker, latch=True, queue_size=10)
pub_marker_direction = rospy.Publisher('/marker_direction', Marker, latch=True, queue_size=10)
# pub_imu_vel = rospy.Publisher('/ego_vel', Float32MultiArray, latch=True, queue_size=10)
# pub_flag = rospy.Publisher('/flag', Bool, latch=True, queue_size=10)

pub_steer_ = rospy.Publisher('/steering_', Float32, latch=True, queue_size=10)


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


def listener():
    rospy.Subscriber('/hedge_pos_ang', hedge_pos_ang, callback_update_pos)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback_update_person_pos)
    rospy.Subscriber("/imu", Imu, callback_update_yaw)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # Initialize map before information is updated
        Map.data = Map.initial_data

        update_goal()

        np_map = np.array(Map.data.data)
        np_map = np_map.reshape(Map.data.info.height, Map.data.info.width)

        # Update Occupancy value from Marvelmind pos 
        # for i in range(int(AGV.x / Map.res - 38), int(AGV.x / Map.res)):
        #     for j in range(int(AGV.y / Map.res) - 9, int(AGV.y / Map.res) + 9):
        #         np_map[j, i] = 50

        # Person pos
        if Person.detected == 1:
            np_map = lime_occupancy(np_map, Person.x, Person.y)

        # Parked Vehicle
        # for i in range(231 - 60, 231):
        #     for j in range(127, 127 + 19):
        #         np_map[j, i] = 99
        a_star(np_map)
        pure_pursuit()
        # stanley()
        np_map_flat = np_map.flatten()
        Map.data.data = np_map_flat

        talker()
        rate.sleep()


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


def lime_occupancy(np_map, x_, y_):
    d_safe = 1.5
    lane_width = 0.9
    res_ = 1 / Map.res
    for i in range(int(x_ * res_ - d_safe * res_), int(x_ * res_ + d_safe * res_)):
        for j in range(int(y_ * res_ - lane_width / 2 * res_ * (np.cos(np.pi * (i - x_ * res_) / d_safe / res_))),
                       int(y_ * res_ + lane_width / 2 * res_ * (np.cos(np.pi * (i - x_ * res_) / d_safe / res_)))):
            np_map[j, i] = 100
    return np_map


def a_star(np_map):
    path = Map.path
    res_ = 1 / Map.res
    goal_x = Map.goal_x * res_
    goal_y = Map.goal_y * res_
    min_i = 0
    min_j = 0
    ego_x = int(round(AGV.x * res_))
    ego_y = int(round(AGV.y * res_))
    for n in range(166):
        min_ = 1000000
        for i in range(ego_x - 1, ego_x + 2):
            for j in range(ego_y - 1, ego_y + 2):
                buffer = np.sqrt(np.square(goal_x - i) + np.square(goal_y - j))
                buffer = buffer + np_map[j, i] * buffer
                if buffer < min_:
                    min_ = copy.copy(buffer)
                    min_i = i
                    min_j = j
        ego_x = min_i
        ego_y = min_j

        path[0, n] = ego_x - round(AGV.x * res_)
        path[1, n] = ego_y - round(AGV.y * res_)

    Map.path = path


def pure_pursuit():
    res_ = 1 / Map.res
    ld = 1.2 * 0.5 * res_  # [m]. Look ahead distance, 0.5s preview time
    wheel_base = 0.15 * res_
    yaw_ = np.deg2rad(AGV.yaw)
    path = Map.path
    rear_center = [-wheel_base / 2 * np.cos(yaw_), -wheel_base / 2 * np.sin(yaw_)]
    min_ = 1000000
    min_n = 0
    for n in range(100):
        buffer = np.sqrt(np.square(rear_center[0] - path[0, n]) + np.square(rear_center[1] - path[1, n]))
        if abs(buffer - ld) < min_:
            min_ = abs(buffer - ld)
            min_n = n

    Map.p_x = path[0, min_n]
    Map.p_y = path[1, min_n]

    Map.p_x += 0.00000001

    if Map.p_x > 0:
        alpha = np.arctan(Map.p_y / Map.p_x) - yaw_
    else:
        if Map.p_y > 0:
            alpha = -np.arctan(Map.p_y / Map.p_x) - yaw_ + np.pi
        else:
            alpha = np.arctan(Map.p_y / Map.p_x) - yaw_ - np.pi

    # print(alpha)
    AGV.steering_angle = np.arctan(2 * wheel_base * np.sin(alpha) / ld)


def stanley():
    res_ = 1 / Map.res
    yaw_ = np.deg2rad(AGV.yaw)
    path = Map.path
    wheel_base = 0.15 * res_
    front_center = [wheel_base / 2 * np.cos(yaw_), wheel_base / 2 * np.sin(yaw_)]
    k_s = 0.000001
    min_ = 1000000
    min_n = 0
    k = 0.5  # Tuning parameter
    v = 0.7  # Approximate velocity in m/s
    for n in range(100):
        buffer = np.sqrt(np.square(front_center[0] - path[0, n]) + np.square(front_center[1] - path[1, n]))
        if buffer < min_:
            min_ = buffer
            min_n = n
    delta1 = np.arctan((path[1, min_n+10] - path[1, min_n-5]) / (path[0, min_n+10] - path[0, min_n-5]))  # heading error
    delta2 = np.arctan(k*min_/(k_s + v))
    AGV.steering_angle = -(delta1 + delta2)


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
    buff = np.transpose(np.array([[1, 0.9],[2.5,0.5],[4,0.5],[5,0.9],[5,2.7],[4,3.1],[2.5,3.1],[1,2.7]]))
    # buff = np.array([[3.3, 2.3, 2, 3],
    #                  [1.8, 1.5, 1.1, 0.8]])
    # buff = np.array([[2, 3, 5],
    #                  [2.2, 1, 2]])

    if not Map.score:
        Map.goal_x = buff[0, Map.score]
        Map.goal_y = buff[1, Map.score]
    if Map.score > 8:
        rospy.signal_shutdown("Laps complete. End of experiment.")
    if (AGV.x - Map.goal_x)*(AGV.x - Map.goal_x) + (AGV.y - Map.goal_y)*(AGV.y - Map.goal_y) < 0.1:
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


def update_marker_goal():
    marker_goal = Marker()
    marker_goal.header.frame_id = "/map"
    marker_goal.type = marker_goal.CUBE
    marker_goal.action = marker_goal.ADD

    marker_goal.pose.position.x = Map.goal_x
    marker_goal.pose.position.y = Map.goal_y
    marker_goal.pose.position.z = 0

    [qx, qy, qz, qw] = euler_to_quat(0, 0, 0)
    marker_goal.pose.orientation.x = qz
    marker_goal.pose.orientation.y = qy
    marker_goal.pose.orientation.z = qz
    marker_goal.pose.orientation.w = qw

    marker_goal.scale.x = 0.2
    marker_goal.scale.y = 0.2
    marker_goal.scale.z = 0.2
    marker_goal.color.a = 0.5
    red_ = 255
    marker_goal.color.r = red_

    return marker_goal


def update_marker_car():
    marker_car = Marker()
    marker_car.header.frame_id = "/map"
    marker_car.type = marker_car.CUBE
    marker_car.action = marker_car.ADD

    marker_car.pose.position.x = AGV.x
    marker_car.pose.position.y = AGV.y
    marker_car.pose.position.z = 0

    [qx, qy, qz, qw] = euler_to_quat(0, 0, np.deg2rad(AGV.yaw))
    marker_car.pose.orientation.x = qx
    marker_car.pose.orientation.y = qy
    marker_car.pose.orientation.z = qz
    marker_car.pose.orientation.w = qw

    marker_car.scale.x = 0.38
    marker_car.scale.y = 0.18
    marker_car.scale.z = 0.2
    marker_car.color.a = 0.5
    green_ = 255
    marker_car.color.g = green_

    return marker_car


def update_marker_direction():
    marker_direction = Marker()
    marker_direction.header.frame_id = "/map"
    marker_direction.type = marker_direction.ARROW
    marker_direction.action = marker_direction.ADD

    marker_direction.pose.position.x = AGV.x
    marker_direction.pose.position.y = AGV.y
    marker_direction.pose.position.z = 0

    [qx, qy, qz, qw] = euler_to_quat(0, 0, AGV.steering_angle + np.deg2rad(AGV.yaw))
    marker_direction.pose.orientation.x = qx
    marker_direction.pose.orientation.y = qy
    marker_direction.pose.orientation.z = qz
    marker_direction.pose.orientation.w = qw

    marker_direction.scale.x = 0.5
    marker_direction.scale.y = 0.03
    marker_direction.scale.z = 0.01
    marker_direction.color.a = 1
    blue_ = 255
    marker_direction.color.b = blue_

    return marker_direction


if __name__ == '__main__':
    try:
        rospy.init_node('AGV_node', anonymous=True, disable_signals=False)
        initialize_map()
        initialize_yaw()
        listener()

    except rospy.ROSInterruptException:
        pass
