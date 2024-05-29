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



class Execute(TriggeredNode):

    def __init__(self, logger = None,knowledgeBase = None,adaptationManagement = None, verbose=False):
        super().__init__(logger=logger,knowledge = knowledgeBase,verbose=verbose)

        self._name = 'execute'
        self._adaptationManagement = adaptationManagement

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _SpinOnceFcn(self, args):

        # TODO: implement execute component
        _status = executeStatus.IDLE
        _accuracy = 1.0

        # 1. CHECK FOR ACTIVE DIAGNOSIS/ADAPTATION PLAN FROM KB
        _diagnosePlan, historyD = self.knowledge.read(actionType.DIAGNOSISTYPE, 1)
        _adaptationPlan, historyA = self.knowledge.read(actionType.ADAPTATIONTYPE, 1)
        # 2. EXECUTE PLAN
        if _diagnosePlan != -1:
            self.logger.log("[" + self._name + "] - " + "Diagnose action registered to adaptation management")
            self._adaptationManagement.performDiagnosis(_diagnosePlan)
        elif _adaptationPlan != -1:
            self.logger.log("[" + self._name + "] - " + "Adaptation action registered to adaptation management")
            self._adaptationManagement.performAdaptation(_adaptationPlan)


        _status = executeStatus.EXECUTION

        # x. SIGNAL EXECUTE STATE VIA KNOWLEDGE
        self.RaPSignalStatus(component=adaptivityComponents.EXECUTE,status=_status,accuracy=_accuracy)

        # y. return status of execution (fail = False, success = True)
        return True


    def _EnterInitializationModeFcn(self):
        if self._verbose: print("Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        # initial signal after startup
        self.RaPSignalStatus(component=adaptivityComponents.MONITOR,status=monitorStatus.NORMAL,accuracy=1.0)

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("Enter configurationModeFcn not implemented")

