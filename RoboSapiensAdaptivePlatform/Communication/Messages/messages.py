from RoboSapiensAdaptivePlatform.utils.constants import *
class Property():
    name="tbd"
    description="tbd"
    value=0.0
    min=0.0
    max=0.0

class ComponentStatus():
    component = "monitor"
    status = "noAnomoly"
    accuracy = 1.0

#class Action():
#    ID= actionType.ADAPTATIONTYPE
#    UUID = "tbd"
#    description="tbd"
#   propertyList=[]
class Action(object):
    def __init__(self,ID=actionType.ADAPTATIONTYPE,description="tbd",propertyList=None):

        self._name = "tbd"
        self._ID= ID
        self._UUID = "tbd"
        self._description=description
        if propertyList is not None:
            self._propertyList=propertyList
        else:
            self._propertyList = []

    @property
    def name(self):
        """The name (read-only)."""
        return self._name

    @name.setter
    def name(self, cmp):
        """The ID (write)."""
        self._name = cmp
    @property
    def ID(self):
        """The ID (read-only)."""
        return self._ID

    @ID.setter
    def ID(self, cmp):
        """The ID (write)."""
        self._ID = cmp

    @property
    def UUID(self):
        """The UUID (read-only)."""
        return self._UUID

    @UUID.setter
    def UUID(self, cmp):
        """The UUID (write)."""
        self._UUID = cmp

    @property
    def description(self):
        """The description (read-only)."""
        return self._description

    @description.setter
    def description(self, cmp):
        """The description (write)."""
        self._description = cmp

    @property
    def propertyList(self):
        """The propertyList (read-only)."""
        return self._propertyList

    @propertyList.setter
    def propertyList(self, cmp):
        """The propertyList (write)."""
        self._propertyList = cmp


class ObjectsStamped(object):
    def __init__(self,name='tbd',ID="tbd",objectList=None):

        self._name= name
        self._ID= ID
        if objectList is not None:
            self._objectList=objectList
        else:
            self._objectList = []

    @property
    def ID(self):
        """The ID (read-only)."""
        return self._ID

    @ID.setter
    def ID(self, cmp):
        """The ID (write)."""
        self._ID = cmp

    @property
    def name(self):
        """The name (read-only)."""
        return self._name

    @name.setter
    def name(self, cmp):
        """The ID (write)."""
        self._name = cmp
    @property
    def objectList(self):
        """The objectList (read-only)."""
        return self._objectList

    @objectList.setter
    def objectList(self, cmp):
        """The objectList (write)."""
        self._objectList = cmp

    def instantiate(self,decodedJSON):          #TODO: this initialize function needs to be generalized to use custom classes
        objList = []
        for p in decodedJSON._objectList:
            obj = Object(label=p["_label"], label_id=p["_label_id"], confidence=p["_confidence"], position=p["_position"],velocity=p["_velocity"], trackingState=p["_trackingState"], actionState=p["_actionState"])
            objList.append(obj)

        self._name = decodedJSON._name
        self._ID = decodedJSON._ID
        self._objectList = objList


class LidarRange(object):
    def __init__(self,name='tbd',ID="tbd",objectList=None):

        self._name= name
        self._ID= ID
        if objectList is not None:
            self._objectList=objectList
        else:
            self._objectList = []

    @property
    def ID(self):
        """The ID (read-only)."""
        return self._ID

    @ID.setter
    def ID(self, cmp):
        """The ID (write)."""
        self._ID = cmp

    @property
    def name(self):
        """The name (read-only)."""
        return self._name

    @name.setter
    def name(self, cmp):
        """The ID (write)."""
        self._name = cmp
    @property
    def objectList(self):
        """The objectList (read-only)."""
        return self._objectList

    @objectList.setter
    def objectList(self, cmp):
        """The objectList (write)."""
        self._objectList = cmp

    def instantiate(self,decodedJSON):          #TODO: this initialize function needs to be generalized to use custom classes
        objList = []
        for p in decodedJSON._objectList:
            obj = Object(label=p["_label"], label_id=p["_label_id"], confidence=p["_confidence"], position=p["_position"],velocity=p["_velocity"], trackingState=p["_trackingState"], actionState=p["_actionState"])
            objList.append(obj)

        self._name = decodedJSON._name
        self._ID = decodedJSON._ID
        self._objectList = objList

        

class Object(object):
    def __init__(self,label='human',label_id='tbd',confidence=99,position=[0.0,0.0,0.0],velocity=[0.0,0.0,0.0],trackingState=trackingState.SEARCHING,actionState=actionState.IDLE):

        self._label = label
        self._label_id = label_id
        self._confidence = confidence
        self._position = position
        self._velocity = velocity
        self._trackingState = trackingState
        self._actionState = actionState

    @property
    def label(self):
        """The label (read-only)."""
        return self._label

    @label.setter
    def label(self, cmp):
        """The label (write)."""
        self._label = cmp

    @property
    def label_id(self):
        """The label_id (read-only)."""
        return self._label_id

    @label_id.setter
    def label_id(self, cmp):
        """The label_id (write)."""
        self._label_id = cmp

    @property
    def confidence(self):
        """The confidence (read-only)."""
        return self._confidence

    @confidence.setter
    def confidence(self, cmp):
        """The confidence (write)."""
        self._confidence = cmp

    @property
    def position(self):
        """The position (read-only)."""
        return self._position

    @position.setter
    def position(self, cmp):
        """The position (write)."""
        self._position = cmp

    @property
    def velocity(self):
        """The velocity (read-only)."""
        return self._velocity

    @velocity.setter
    def velocity(self, cmp):
        """The velocity (write)."""
        self._velocity = cmp

    @property
    def trackingState(self):
        """The trackingState (read-only)."""
        return self._trackingState

    @trackingState.setter
    def trackingState(self, cmp):
        """The trackingState (write)."""
        self._trackingState = cmp

    @property
    def actionState(self):
        """The actionState (read-only)."""
        return self._actionState

    @actionState.setter
    def actionState(self, cmp):
        """The actionState (write)."""
        self._actionState = cmp



class RobotPose(object):
    def __init__(self,name='ROB1',frameID='FID_001',confidence=99,position=[0.0,0.0,0.0],orientation=[0.0,0.0,0.0,0.0],linear=[0.0,0.0,0.0], angular=[0.0,0.0,0.0]):

        self._name= name
        self._frameID = frameID
        self._confidence = confidence
        self._position = position
        self._orientation = orientation
        self._linear = linear
        self._angular = angular


    @property
    def name(self):
        """The name (read-only)."""
        return self._name

    @name.setter
    def name(self, cmp):
        """The name (write)."""
        self._name = cmp

    @property
    def frameID(self):
        """The frameID (read-only)."""
        return self._frameID

    @frameID.setter
    def frameID(self, cmp):
        """The frameID (write)."""
        self._frameID = cmp

    @property
    def confidence(self):
        """The confidence (read-only)."""
        return self._confidence

    @confidence.setter
    def confidence(self, cmp):
        """The confidence (write)."""
        self._confidence = cmp

    @property
    def position(self):
        """The position (read-only)."""
        return self._position

    @position.setter
    def position(self, cmp):
        """The position (write)."""
        self._position = cmp

    @property
    def orientation(self):
        """The orientation (read-only)."""
        return self._orientation

    @orientation.setter
    def orientation(self, cmp):
        """The orientation (write)."""
        self._orientation = cmp

    @property
    def linear(self):
        """The linear (TWIST) (read-only)."""
        return self._linear

    @linear.setter
    def linear(self, cmp):
        """The linear (TWIST) (write)."""
        self._linear = cmp

    @property
    def angular(self):
        """The angular (TWIST) (read-only)."""
        return self._angular

    @angular.setter
    def angular(self, cmp):
        """The angular (TWIST) (write)."""
        self._angular = cmp

    def instantiate(self,decodedJSON):                  #TODO: this initialize function needs to be generalized to use custom classes
        self._name = decodedJSON._name
        self._frameID = decodedJSON._frameID
        self._confidence = decodedJSON._confidence
        self._position = decodedJSON._position
        self._orientation = decodedJSON._orientation
        self._linear = decodedJSON._linear
        self._angular = decodedJSON._angular


class LogMessage(object):
    def __init__(self,name='tbd',message="tbd"):

        self._name = name
        self._message= message

    @property
    def name(self):
        """The name (read-only)."""
        return self._name

    @name.setter
    def name(self, cmp):
        """The ID (write)."""
        self._name = cmp

    @property
    def message(self):
        """The message (read-only)."""
        return self._message

    @message.setter
    def message(self, cmp):
        """The message (write)."""
        self._message = cmp

    def instantiate(self,decodedJSON):                  #TODO: this initialize function needs to be generalized to use custom classes
        self._name = decodedJSON._name
        self._message = decodedJSON._message