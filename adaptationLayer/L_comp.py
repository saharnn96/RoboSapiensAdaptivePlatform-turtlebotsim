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
from RoboSapiensAdaptivePlatform.utils.constants import *



class legitimize(TriggeredNode):

    def __init__(self, logger = None,knowledgeBase = None,verbose=False):
        super().__init__(logger=logger,knowledge = knowledgeBase,verbose=verbose)

        self._name = 'Legitimize'
    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _SpinOnceFcn(self, args):

        # TODO: implement legitimize component
        _status = legitimizeStatus.IDLE
        _accuracy = 1.0

        # 1. CHECK FOR ACTIVE DIAGNOSIS/ADAPTATION PLAN FROM KB

        _diagnosePlan = self.knowledge.read(actionType.DIAGNOSISTYPE,1)
        _adaptationPlan = self.knowledge.read(actionType.ADAPTATIONTYPE,1)
        # 2. CHECK ACTIVE PLAN
        if _diagnosePlan != -1:
            _plan = _diagnosePlan
        elif _adaptationPlan != -1:
            _plan = _adaptationPlan


        #TODO: IMPLEMENT LEGITIMIZE FUNCTION
        _status = legitimizeStatus.VALID

        # 3. SIGNAL LEGITIMIZE STATE VIA KNOWLEDGE
        self.RaPSignalStatus(component=adaptivityComponents.LEGITIMIZE,status=_status,accuracy=_accuracy)

        self.logger.log("["+self._name+"] - "+"Plan legitimized")

        # 4. return status of execution (fail = False, success = True)
        return True


    def _EnterInitializationModeFcn(self):
        if self._verbose: print("Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        # initial signal after startup
        self.RaPSignalStatus(component=adaptivityComponents.MONITOR,status=monitorStatus.NORMAL,accuracy=1.0)

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("Enter configurationModeFcn not implemented")

