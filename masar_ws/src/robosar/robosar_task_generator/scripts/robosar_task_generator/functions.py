#!/usr/bin/env python3

import os
import sys
from collections import deque as queue
from turtle import speed

import numpy as np
from numpy import array, floor, inf
from numpy.linalg import norm
from skimage.segmentation import flood

sys.path.append(os.path.dirname(__file__))


def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data
    index = int(
        (floor((Xp[1] - Xstarty) / resolution) * width)
        + (floor((Xp[0] - Xstartx) / resolution))
    )
    return index


def index_2d_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    return [
        int(floor((Xp[1] - Xstarty) / resolution)),
        int(floor((Xp[0] - Xstartx) / resolution)),
    ]


def point_of_index(mapData, i):
    y = (
        mapData.info.origin.position.y
        + (i / mapData.info.width) * mapData.info.resolution
    )
    x = (
        mapData.info.origin.position.x
        + (i - (i / mapData.info.width) * (mapData.info.width))
        * mapData.info.resolution
    )
    return array([x, y])


def informationGain(mapData, point, r, occ_threshold):
    r_region = int(floor(r / mapData.info.resolution))
    map2 = np.reshape(mapData.data, (mapData.info.height, mapData.info.width))
    x, y = index_2d_of_point(mapData, point)
    x_min = max(x - r_region, 0)
    x_max = min(x + r_region, mapData.info.height)
    y_min = max(y - r_region, 0)
    y_max = min(y + r_region, mapData.info.width)

    area = map2[x_min:x_max, y_min:y_max]
    seed = (min(x, x_max - 1) - x_min, min(y, y_max - 1) - y_min)
    area[seed[0], seed[1]] = 0 if area[seed[0], seed[1]] >= 0 else -1
    mask = flood(area, seed, tolerance=occ_threshold)

    contains_free = np.array(mask == 1) & np.array(
        (area < occ_threshold) & (area >= 0)
    )  # if flood fill contains free space
    if np.any(contains_free):
        info_mask = np.array(mask == 1) & np.array(
            area == -1
        )  # find only unknown spaces
        infoGain = np.sum(info_mask)
    else:

        return 0

    return infoGain / ((x_max - x_min) * (y_max - y_min))


def discount(mapData, assigned_pt, centroids, infoGain, r):
    index = index_of_point(mapData, assigned_pt)
    r_region = int(r / mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start / mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if i >= 0 and i < limit and i < len(mapData.data):
                for j in range(0, len(centroids)):
                    current_pt = centroids[j]
                    if (
                        mapData.data[i] == -1
                        and norm(point_of_index(mapData, i) - current_pt) <= r
                        and norm(point_of_index(mapData, i) - assigned_pt) <= r
                    ):
                        # this should be modified, subtract the area of a cell, not 1
                        infoGain[j] -= 1
    return infoGain


def pathCost(path):
    r_region = 5
    if len(path) > 0:
        i = len(path) / 2
        p1 = (
            array([path[i - 1].pose.position.x, path[i - 1].pose.position.y])(
                2 * r_region
            )
            ** 2
        )
        p2 = array([path[i].pose.position.x, path[i].pose.position.y])
        return norm(p1 - p2) * (len(path) - 1)
    else:
        return inf


def unvalid(mapData, pt):
    index = index_of_point(mapData, pt)
    r_region = 5
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start / mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if i >= 0 and i < limit and i < len(mapData.data):
                if mapData.data[i] == 1:
                    return True
    return False


def Nearest(V, x):
    n = inf
    i = 0
    for i in range(0, V.shape[0]):
        n1 = norm(V[i, :] - x)
        if n1 < n:
            n = n1
            result = i
    return result


def Nearest2(V, x):
    n = inf
    result = 0
    for i in range(0, len(V)):
        n1 = norm(V[i] - x)

        if n1 < n:
            n = n1
    return i

def check_edge_collision(xnear, xnew, mapData):
        rez = float(mapData.info.resolution)
        stepz = int(np.ceil(Norm(xnew[0], xnew[1], xnear[0], xnear[1])) / rez)
        xi = xnear
        obs = 0
        unk = 0

        for _ in range(stepz):
            xi = Steer(xi, xnew, rez)
            if unvalid(mapData, xi):
                obs = 1
                break
            if gridValue(mapData, xi) == 100:
                obs = 1
            if gridValue(mapData, xi) == -1:
                unk = 1
                break
        out = 0
        xnew = xi
        if unk == 1:
            out = -1  # unknown
        if obs == 1:
            out = 0  # occupied
        if obs != 1 and unk != 1:
            out = 1  # free

        return out

def gridValue(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data = mapData.data
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    index = (floor((Xp[1] - Xstarty) / resolution) * width) + (
        floor((Xp[0] - Xstartx) / resolution)
    )

    if int(index) < len(Data):
        return Data[int(index)]
    else:
        return 100


def Norm(x1, y1, x2, y2):
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5


def Steer(x_nearest, x_rand, eta):
    x_new = [x_nearest[0], x_nearest[1]]
    if Norm(x_nearest[0], x_nearest[1], x_rand[0], x_rand[1]) <= eta:
        x_new = x_rand
    else:
        m = (x_rand[1] - x_nearest[1]) / (x_rand[0] - x_nearest[0])

        if x_rand[0] == x_nearest[0]:
            x_new = [x_nearest[0], x_nearest[1] + eta]

        x_new[0] = (np.sign(x_rand[0] - x_nearest[0])) * (
            np.sqrt(eta**2) / ((m**2) + 1)
        ) + x_nearest[0]
        x_new[1] = m * (x_new[0] - x_nearest[0]) + x_nearest[1]
    return x_new
