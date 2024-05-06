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



class CommunicationEndpoint(TriggeredNode):

    def __init__(self, logger = None,knowledgeBase = None,communicationManagement=None,verbose=False):
        super().__init__(logger=logger,knowledge = knowledgeBase,verbose=verbose)

        self._name = 'CommunicationEndpoint'
        self._com = communicationManagement

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _SpinOnceFcn(self, args):
        _status = communicationEndPointStatus.RUNNING
        self.logger.log("[" + self._name + "] - " + 'effector endpoint triggered - published via communication management"')
        try:
            self._com.publish(args)
            _status = communicationEndPointStatus.IDLE
        except Exception as e:
            print(e)
            _status = communicationEndPointStatus.ERROR

        # 3. RETURN STATUS OF THE ACTION
        return _status




    def _EnterInitializationModeFcn(self):
        if self._verbose: print("Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        if self._verbose: print("Exit initializationModeFcn not implemented")

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("Enter configurationModeFcn not implemented")



