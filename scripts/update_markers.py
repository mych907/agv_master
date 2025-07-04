#!/usr/bin/env python

from visualization_msgs.msg import Marker

from library_agv import *


def update_marker_goal():
    marker_goal = Marker()
    marker_goal.header.frame_id = "/map"
    marker_goal.type = marker_goal.CUBE
    marker_goal.action = marker_goal.ADD

    marker_goal.pose.position.x = Map.goal_x
    marker_goal.pose.position.y = Map.goal_y
    marker_goal.pose.position.z = 0

    [qx, qy, qz, qw] = euler_to_quat(0, 0, 0)
    marker_goal.pose.orientation.x = qx
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
