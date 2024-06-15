# **********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from RoboSapiensAdaptivePlatform.utils.nodes import TriggeredNode
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import ComponentStatus
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import *
from RoboSapiensAdaptivePlatform.utils.constants import *

import numpy as np
from lidarocclusion.masks import BoolLidarMask
from typing import List, Tuple, Dict
from fractions import Fraction
import traceback

def calculate_lidar_occlusion_rotation_angles(lidar_mask: BoolLidarMask) -> List[Fraction]:
    """
    Calculate the angles of the detected occlusions in the lidar mask.
    :param lidar_mask: The lidar mask.
    :return: A list of angles of the detected occlusions.
    """
    occlusion_angles = []
    mask_angles =  np.concatenate((
        np.arange(0, 1, lidar_mask.base_angle),
        np.arange(-1, 0, lidar_mask.base_angle),
    ))
    mask_values = lidar_mask.map_poly(lambda x: 0 if x else 1)._values
    rotation_angles = (mask_angles * mask_values)

    occlusion_angles = [rotation_angles.min(), rotation_angles.max()]
    
    # Return the two rotations necessary for occlusions on either side
    # of the robot
    match occlusion_angles:
        case [x]:
            return [x, -x]
        case [x, y] if 0 <= x <= y:
            return [y, -y]
        case [x, y] if x <= y <= 0:
            return [x, -x]
        case [x, y] if y - x > 1:
            return [Fraction(2)]
        case [x, y] if abs(x) > abs(y):
            return [x, -x + y, -y]
        case [x, y] if abs(y) > abs(x):
            return [y, -y + x, -x]
        case _:
            assert False

def occlusion_angle_to_rotation(occlusion_angle: Fraction) -> Dict[str, float]:
    signed_angle = float(occlusion_angle)*np.pi
    return {
        'omega': (-1.0)**int(signed_angle < 0),
        'duration': abs(float(signed_angle)),
    }

def occlusion_angles_to_rotations(occlusion_angles: List[Fraction]) -> List[Dict[str, float]]:
    return list(map(occlusion_angle_to_rotation, occlusion_angles))


class Plan(TriggeredNode):

    def __init__(self, logger = None,knowledgeBase = None,verbose=False):
        super().__init__(logger=logger,knowledge = knowledgeBase,verbose=verbose)

        self._name = "plan"
    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _SpinOnceFcn(self, args):

        # TODO: implement planner component
        _status = planStatus.IDLE
        _accuracy = 1.0

        # 1. DETERMINE PLAN
        _status = planStatus.PLANNING
        hack_plan = self.knowledge._action


        # 2. WRITE PLAN TO KNOWLEDGE
        # self.knowledge.write(self._plan)

        # x = self.knowledge._action._propertyList[-1]
        self.logger.log(
            f"[{self._name}] - Plan determined: {hack_plan.__dict__}")

        try:
            lidar_mask = hack_plan._propertyList[-1]['mask']

            self.logger.log(
                f"[{self._name}] - Plan lidar mask determined: {lidar_mask}")

            occlusion_angles = calculate_lidar_occlusion_rotation_angles(lidar_mask)
            directions = occlusion_angles_to_rotations(occlusion_angles)
        except:
            self.logger.log("traceback case")
            occlusion_angles = []
            directions = []
            self.logger.log("traceback: " + traceback.format_exc())

        self.logger.log(
            f"[{self._name}] - Plan rotations determined: {occlusion_angles}")
        # directions = [
        #     {'omega': -1, 'duration': 1.57},
        #     {'omega': 1, 'duration': 1.57},
        # ]
        # Hack to get the lidar mask into the knowledge base
        plan = Action()
        plan.name = 'SpeedAdaptationAction'
        plan.ID = actionType.ADAPTATIONTYPE
        plan.description = "SPEED ADAPTATION"
        plan.propertyList = [{"mask": lidar_mask, "directions": directions}]
        self.knowledge.write(plan)
        self.logger.log(f"[{self._name}] - Plan action written to knowledge")

        # x. SIGNAL PLAN STATE VIA KNOWLEDGE
        _status = planStatus.PLANNED
        self.RaPSignalStatus(component=adaptivityComponents.PLAN,status=_status,accuracy=_accuracy)

        # self.logger.log("["+self._name+"] - "+"Plan determined: planned for "+self._plan.ID+" with variables "+ self._plan.propertyList.__str__())

        # 4. return status of execution (fail = False, success = True)
        return True

    def dummyPlan(self):
        #DUMMY PLAN => SLOW DOWN THE ROBOT IF ANOMALY IS DETECTED
        _plan = None

        poi = Property()
        poi.name = 'target_speed'
        poi.value = 1.0
        poi.description = 'Slow down robot'

        _plan = Action()
        _plan.name = 'SpeedAdaptationAction'
        _plan.ID = actionType.ADAPTATIONTYPE
        _plan.description = "SPEED ADAPTATION"
        _plan.propertyList = [poi]

        return _plan

    def _EnterInitializationModeFcn(self):
        if self._verbose: print("Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        # initial signal after startup
        self.RaPSignalStatus(component=adaptivityComponents.MONITOR,status=monitorStatus.NORMAL,accuracy=1.0)

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("Enter configurationModeFcn not implemented")

