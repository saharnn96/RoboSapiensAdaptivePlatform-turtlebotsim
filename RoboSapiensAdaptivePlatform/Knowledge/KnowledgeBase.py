#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from RoboSapiensAdaptivePlatform.utils.constants import *
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import *
import queue
import uuid
import hashlib
from lidarocclusion.masks import (LidarMask, ProbLidarMask, BoolLidarMask)


class KnowledgeBase_Property(object):

    def __init__(self,name='tbd',description = 'tbd',value= 0.0,min=0.0,max=0.0,retention=1000):
        """Initialize the Knowledge Base property component.

                Parameters
                ----------
                retention : integer
                    Retention points for all properties

                See Also
                --------
                ..

                Examples
                --------
                >> kb = KnowledgeBase(retention = 5000,verbose=False)

                """
        self._name=name
        self._description=description
        self._value=value
        self._min=min
        self._max=max
        self._history = []
        self.setProperty(value)

    @property
    def name(self):
        """The name property (read-only)."""
        return self._name

    @property
    def description(self):
        """The description property (read-only)."""
        return self._description

    @property
    def value(self):
        """The value property (read-only)."""
        return self._value

    @property
    def minimum(self):
        """The minimum value property (read-only)."""
        return self._min

    @property
    def maximum(self):
        """The minimum value property (read-only)."""
        return self._max


    def history(self,queueSize):
        """The minimum value property (read-only)."""
        try:
            return self._history[-queueSize:]
        except:
            return -1

    def setProperty(self,value):
        self._value = value
        #add to history queue
        self._history.append(value)


class KnowledgeBase_Action(object):

    def __init__(self,ID='tbd',name='tbd',description = 'tbd',propertyList= None,retention=1000):
        """Initialize the Knowledge Base action component.

                Parameters
                ----------
                retention : integer
                    Retention points for all properties

                See Also
                --------
                ..

                Examples
                --------
                >> kb = KnowledgeBase(retention = 5000,verbose=False)

                """
        self._ID=ID
        self._name = name
        self._description=description
        if propertyList is None:
            self._propertyList=[]
        else:
            self._propertyList = propertyList

        self._isValid = False
        str = ID+"/"+description
        self._uuid = hashlib.sha256(str.encode()).__str__()  #TODO:Unique ID for checking of action is executed -> hashlib not serializeble!!

    @property
    def ID(self):
        """The ID (read-only)."""
        return self._ID

    @property
    def name(self):
        """The name (read-only)."""
        return self._name

    @property
    def description(self):
        """The description property (read-only)."""
        return self._description

    @property
    def propertyList(self):
        """The propertyList property (read-only)."""
        return self._propertyList

    @property
    def validity(self):
        """The validity property (read-only)."""
        return self._isValid

    @validity.setter
    def validity(self,validityValue):
        """The validity property (write)."""
        self._isValid = validityValue

    @property
    def uuid(self):
        """The uuid (read-only)."""
        return self._uuid

class KnowledgeBase_ComponentStatus(object):

    def __init__(self,component='tbd',status = 'tbd',accuracy= 1.0,retention=1000):
        """Initialize the Knowledge Base component status component.

                Parameters
                ----------
                retention : integer
                    Retention points for all properties

                See Also
                --------
                ..

                Examples
                --------
                >> kb = KnowledgeBase(retention = 5000,verbose=False)

                """
        self._name=component
        self._status=status
        self._accuracy=accuracy
        self._history = []
        self.setProperty(self._status,self._accuracy)

    @property
    def name(self):
        """The component name property (read-only)."""
        return self._name
    @property
    def status(self):
        """The status property (read-only)."""
        return self._status

    @property
    def accuracy(self):
        """The minimum value property (read-only)."""
        return self._accuracy


    def history(self,queueSize):
        """The minimum value property (read-only)."""
        try:
            return self._history[-queueSize:]
        except:
            return -1

    def setProperty(self,status,accuracy):
        self._status = status
        self._accuracy = accuracy
        #add to history queue
        self._history.append([status,accuracy])


class KnowledgeBase(object):

    def __init__(self,verbose=False):
        """Initialize the Knowledge Base component.

                Parameters
                ----------
                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> kb = KnowledgeBase(verbose=False)

        """

        self._name = "KnowledgeBase"
        self._description = "Simple knowledge base for experimentation purpose."
        self._verbose = verbose

        # List of knowledge properties log
        self._PropertyList = []

        # List of component status
        self._statusList = []

        # Action placeholder
        self._action = None

        # List of detected objects log (for person detection)
        self._objectDetectedList = []

        # List of lidar masks (for lidar occlusion detection)
        self._lidar_masks = []

        # List of lidar masks (for lidar occlusion detection)
        self._prob_lidar_masks = []

        # List of robot odometry log
        self._robotOdometryList = []

        # List of Lidar Range
        self._robotScanList = []

        # List of rotation directions
        self._directions = []

    #------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS---------------------------------------
    # ------------------------------------------------------------------------------------------------
    def write(self, data):
        """Function to write knowledge to the knowlegde base."""
        found = False
        
        match data:
            case Property():
                for p in self._PropertyList:
                    if p.name == data.name:
                        found=True
                        p.setProperty(data.value)

                if not found:
                    kbp = KnowledgeBase_Property(name=data.name,
                        description=data.description, value=data.value, 
                        min=data.min, max=data.max, retention=1000)
                    self._PropertyList.append(kbp)
            case ComponentStatus():
                for s in self._statusList:
                    if s.name == data.component:
                        found = True
                        s.setProperty(status=data.status,accuracy=data.accuracy)

                if not found:
                    kbs = KnowledgeBase_ComponentStatus(component=data.component, status=data.status, accuracy=data.accuracy, retention=1000)
                    self._statusList.append(kbs)
            case Action():
                self._action = None
                self._action = KnowledgeBase_Action(ID=data.ID,name=data.name,description=data.description,propertyList=data.propertyList)
            case ObjectsStamped():
                self._objectDetectedList.append(data)
            case RobotPose():
                self._robotOdometryList.append(data)
            case LidarRange():
                self._robotScanList.append(data)
            case ProbLidarMask():
                self._prob_lidar_masks.append(data)
            case BoolLidarMask():
                self._lidar_masks.append(data)
            case list():
                # A list of directions
                self._directions.append(data)
            case _:
                raise ValueError("Unknown knowledge type")

    def read(self, name, queueSize):
        """Function to read knowledge from the knowlegde base."""
        found = False
        data = None
        historyQueue = -1
        for p in self._PropertyList:
            if p.name == name:
                found=True
                data = Property()
                data.name = p.name
                data.value = p.value
                data.description = p.description
                data.min = p.minimum
                data.max = p.maximum

                historyQueue = p.history(queueSize)

        for s in self._statusList:
            if s.name == name:
                found=True
                data = ComponentStatus()
                data.component = s.name
                data.status = s.status
                data.accuracy = s.accuracy

                historyQueue = s.history(queueSize)

        if self._action != None and name == self._action.ID:
            found = True
            data = Action()
            data.ID = self._action.ID
            data.name = self._action.name
            data.description = self._action.description
            data.propertyList = self._action.propertyList
            data.UUID = self._action.uuid
            historyQueue = -1

        if len(self._objectDetectedList) !=0 and name == self._objectDetectedList[0].name:
            found = True
            data = self._objectDetectedList[-1]  #provide the last detection knowledge
            historyQueue = self._objectDetectedList[-queueSize:]

        if len(self._robotScanList) != 0 and name == self._robotScanList[0].name:
            found = True
            data = self._robotScanList[-1]  #provide the last detection knowledge
            historyQueue = self._robotScanList[-queueSize:]

        if len(self._robotOdometryList) !=0 and name == self._robotOdometryList[0].name:
            found = True
            data = self._robotOdometryList[-1]  #provide the last odometry knowledge
            historyQueue = self._robotOdometryList[-queueSize:]

        if len(self._lidar_masks) != 0 and name == 'LidarMask':
            found = True
            data = self._lidar_masks[-1] 
            historyQueue = self._lidar_masks[-queueSize:]

        if len(self._prob_lidar_masks) != 0 and name == 'ProbLidarMask':
            found = True
            data = self._prob_lidar_masks[-1] 
            historyQueue = self._prob_lidar_masks[-queueSize:]

        if len(self._directions) != 0 and name == 'directions':
            found = True
            data = self._directions[-1] 
            historyQueue = self._directions[-queueSize:]

        if not found:
            return -1,-1   #Property or ComponentStatus not found
        else:
            return data,historyQueue
