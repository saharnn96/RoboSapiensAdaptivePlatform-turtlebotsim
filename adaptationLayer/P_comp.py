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



class plan(TriggeredNode):

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
        self._plan = self.dummyPlan()

        # 2. WRITE PLAN TO KNOWLEDGE
        self.knowledge.write(self._plan)

        # x. SIGNAL PLAN STATE VIA KNOWLEDGE
        _status = planStatus.PLANNED
        self.RaPSignalStatus(component=adaptivityComponents.PLAN,status=_status,accuracy=_accuracy)

        self.logger.log("["+self._name+"] - "+"Plan determined: planned for "+self._plan.ID+" with variables "+ self._plan.propertyList.__str__())

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

