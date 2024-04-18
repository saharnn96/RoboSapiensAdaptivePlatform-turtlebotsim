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



class Effector(TriggeredNode):

    def __init__(self, logger = None,knowledgeBase = None,verbose=False):
        super().__init__(logger=logger,knowledge = knowledgeBase,verbose=verbose)

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _SpinOnceFcn(self, args):
        _status = effectorStatus.IDLE
        # 1. DECODE THE ACTION TO BE PERFORMED
        _rawAction = args   #PASSING ONLY THE ACTION
        _actionType = _rawAction.ID
        _description = _rawAction.description
        _propertyList = _rawAction.propertyList


        if _actionType == actionType.DIAGNOSISTYPE:
            # 2a. PERFORM THE DIAGNOSE ACTION ROUTINE
            self.logger.log("["+self._name+"] - "+'Diagnose action triggered - "'+_description+'"')
            _status = effectorStatus.DIAGNOSIS

        elif _actionType == actionType.ADAPTATIONTYPE:
            # 2b. PERFORM THE ADAPTATION ACTION ROUTINE
            self.logger.log("["+self._name+"] - "+'Adaptation action triggered - "'+_description+'"')
            _status = effectorStatus.ADAPTATION

            #perform adaptation for all properties provided


        else:
            self.logger.log("["+self._name+"] - "+"ERROR - Unknown action type")


        # 3. RETURN STATUS OF THE ACTION
        return _status




    def _EnterInitializationModeFcn(self):
        if self._verbose: print("Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        if self._verbose: print("Exit initializationModeFcn not implemented")

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("Enter configurationModeFcn not implemented")

