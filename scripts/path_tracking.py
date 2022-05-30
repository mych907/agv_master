#!/usr/bin/env python

import numpy as np

from library_agv import AGV, Person, Map


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