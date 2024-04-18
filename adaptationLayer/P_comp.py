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
        #DUMMY PLAN => IF LIGHT IS ON, TURN OFF, AND IF OFF, TURN ON
        _plan = None

        light, history1 = self.knowledge.read("light", queueSize=1)

        poi = Proptery()
        poi.name = 'light'
        poi.value = 1 - history1[0]
        poi.description = 'Toggle light'

        _plan = Action()
        _plan.ID = actionType.ADAPTATIONTYPE
        _plan.description = "DUMMY_ADAPTATION_LIGHT"
        _plan.propertyList = [poi]

        return _plan

    def _EnterInitializationModeFcn(self):
        if self._verbose: print("Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        # initial signal after startup
        self.RaPSignalStatus(component=adaptivityComponents.MONITOR,status=monitorStatus.NORMAL,accuracy=1.0)

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("Enter configurationModeFcn not implemented")

