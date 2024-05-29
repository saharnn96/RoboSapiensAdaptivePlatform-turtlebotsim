#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
import json
import math
import numpy as np
from turtlebot4_sim.turtlebot4.Messages import LidarMsg,composeHeader


def uncertainty_add(distance, angle, sigma):
    mean = np.array([distance, angle])
    covanriance = np.diag(sigma ** 2)
    distance, angle = np.random.multivariate_normal(mean, covanriance)
    distance = max(distance, 0)
    angle = max(angle, 0)
    return [distance, angle]


class LaserSensor:
    def __init__(self, position,Range, gmap, uncertentity,angularResolution=1):
        self.Range = Range
        self.speed = 4  # rounds per seconds
        self.sigma = np.array([uncertentity[0], uncertentity[1]])
        self.position = (position[0], position[1])
        self.map = gmap
        self.sensedObstacles = []
        self.w = 800
        self.h = 1200
        self.angularResolution = int(angularResolution)

        # 2D point cloud data
        self.cloudPoint = None

        # --- Static lidar parameters ---
        self._angle_min = -3.12
        self._angle_max = 3.14
        self._angle_increment = 0.0087
        self._time_increment = 0.000187
        self._range_min = 0.15
        self._range_max = 12.0

        # --- dynamic lidar parameters ---
        self._scan_time = 0.13464
        self._ranges = []

        # --- FAULT INJECTION ---
        self._overlayActive = False
        self._overlay = None            # objects blocking angles of sensor [[angle_min, angle_max],[angle_min, angle_max]]

        # --- exposed battery message ---
        self.lidarMessage = LidarMsg()

    def distance(self, ObstaclePosition):
        px = (ObstaclePosition[0] - self.position[0]) ** 2
        py = (ObstaclePosition[1] - self.position[1]) ** 2
        return math.sqrt(px + py)

    def sense_obstacles(self):
        data = []
        x1, y1 = self.position[0], self.position[1]
        for angle in np.linspace(0, 2 * math.pi, int(360/self.angularResolution), False):
            x2, y2 = (x1 + self.Range * math.cos(angle), y1 - self.Range * math.sin(angle))
            for i in range(0, 101):
                if i == 100:
                    data.append([self._range_max, angle, self.position])
                else:
                    u = i / 100
                    x = int(x2 * u + x1 * (1 - u))
                    y = int(y2 * u + y1 * (1 - u))
                    if 0 < x < self.w and 0 < y < self.h:
                        #check if occupied
                        occupied = self.map.is_occupied((x, y))
                        if occupied:
                            distance = self.distance((x, y))
                            output = uncertainty_add(distance, angle, self.sigma)
                            output.append(self.position)
                            data.append(output)
                            break
            

        if len(data) > 0:
            self.cloudPoint = data
            return data
        else:
            return False

    def activateOverlay(self,overlay):
        self._overlayActive = True
        self._overlay = overlay

    def _performOverlay(self):
        tmp = self.cloudPoint
        # --- apply angle overlay on pointcloud ---
        if self._overlay is not None:
            for obj in self._overlay:
                for point in tmp:
                    if point[1] >=  obj[0] and point[1] <= obj[1]:    # LIDAR IS BLOCKED BY OBJECT
                        point[0] = obj[2]    #BLOCK AT THE GIVEN DISTANCE

        self.cloudPoint = tmp


    def spin_once(self,pos_x,pos_y):

        # --- run virtual sensor ---
        self.position = [pos_x, pos_y]
        self.sense_obstacles()

        # --- perform fault injections if active ---
        if self._overlayActive:
            self._performOverlay()

        # --- compose lidar message ---
        _header = composeHeader(frame_id='')


        self.lidarMessage.header = _header
        # --- static lidar parameters ---
        self.lidarMessage.angle_min = self._angle_min
        self.lidarMessage.angle_max = self._angle_max
        self.lidarMessage.angle_increment = self._angle_increment
        self.lidarMessage.time_increment = self._time_increment
        self.lidarMessage.range_min = self._range_min
        self.lidarMessage.range_max = self._range_max
        # --- dynamic lidar parameters ---
        self.lidarMessage.scan_time = 0.13464

        distances = []
        for point in self.cloudPoint:
            distances.append(point[0])

        self.lidarMessage.ranges = distances
