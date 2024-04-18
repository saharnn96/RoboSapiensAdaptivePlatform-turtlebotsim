#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
import time
import json
import math
import numpy as np
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import *

def uncertainty_add(distance, angle, sigma):
    mean = np.array([distance, angle])
    covanriance = np.diag(sigma ** 2)
    distance, angle = np.random.multivariate_normal(mean, covanriance)
    distance = max(distance, 0)
    angle = max(angle, 0)
    return [distance, angle]
class Camera:
    def __init__(self,position,Range=30.0,angularResolution=1,FOV=120.0,uncertentity=(0.5, 0.01),personList=None):

        self.position = (position[0], position[1])
        self.personList =personList


        # virtual camera configuration
        self.Range = Range
        self.angularResolution = angularResolution
        self.FOV = int(FOV)
        self.sigma = np.array([uncertentity[0], uncertentity[1]])
        self.w = 800
        self.h = 1200

        # virtual detection of the camera
        self._objectList = []


    def distance(self, ObstaclePosition):
        px = (ObstaclePosition[0] - self.position[0]) ** 2
        py = (ObstaclePosition[1] - self.position[1]) ** 2
        return math.sqrt(px + py)

    def personPresent(self,x,y):
        objectList = []
        for person in self.personList:
            dist = math.sqrt((person._x - x) ** 2 + (person._y - y) ** 2)
            if dist <= person._detectionBound:
                #print("x:" + x.__str__() + "y:" + y.__str__() + "- personX:" + person._x.__str__() + " personY:" + person._y.__str__() + " -- distance:" + dist.__str__() + " to person ->" + person._UID)
                objectList.append(person)

        return objectList

    def sense_object(self,theta):
        data = []
        foundID = []
        x1, y1 = self.position[0], self.position[1]
        #for angle in np.linspace(theta-(math.radians(self.FOV/2)), theta+(math.radians(self.FOV/2)), int(self.FOV/self.angularResolution), False):
        for angle in np.linspace(0, 2 * math.pi, int(360 / self.angularResolution), False):         #TODO: MAKE DEPENDENT ON FOV, FOR NOW 360
            x2, y2 = (x1 + self.Range * math.cos(angle), y1 - self.Range * math.sin(angle))
            for i in range(0, 100):
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.w and 0 < y < self.h:
                    #check if person is intersecting the FOV line of camera
                    objectList = self.personPresent(x=x,y=y)
                    if len(objectList) > 0:
                        for obj in objectList:
                            distance = self.distance((obj._x, obj._y))
                            output = uncertainty_add(distance, angle, self.sigma)
                            if obj._isRunning:
                                state = actionState.MOVING
                            else:
                                state = actionState.IDLE
                            if obj._UID not in foundID:
                                foundID.append(obj._UID)
                                person = Object(label="human",label_id=obj._UID,confidence=output,position=[obj._x,obj._y,0.0],velocity=[obj._v,0.0,0.0],trackingState=trackingState.OK,actionState=state)
                                data.append(person)

        if len(data) > 0:
            return data
        else:
            return []

    def spin_once(self,pos_x,pos_y,theta):

        # --- run virtual sensor ---
        self.position = [pos_x, pos_y]
        self._objectList = self.sense_object(theta)

