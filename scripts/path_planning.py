#!/usr/bin/env python

import numpy as np
import copy

from library_agv import Map, AGV, Person


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
