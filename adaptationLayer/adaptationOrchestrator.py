# **********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from RoboSapiensAdaptivePlatform.utils.nodes import OrchestratorNode
from RoboSapiensAdaptivePlatform.Communication.Messages.messages import ComponentStatus
from RoboSapiensAdaptivePlatform.utils.constants import *
from RoboSapiensAdaptivePlatform.utils.timer import perpetualTimer





class adaptationOrchestrator(OrchestratorNode):

    def __init__(self, dt=0.5,M=None, A=None, P=None, L=None, E=None, logger = None,knowledgeBase = None,verbose=False):
        super().__init__(dt=dt,M=M,A=A,P=P,L=L,E=E,logger=logger,knowledge = knowledgeBase,verbose=verbose)


        self._orchestrationStatus = orchestrationStatus.IDLE


    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------


    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    #def _OrchestrationLoop(self, args):
    #    if self._verbose:print("Implement to override the default orchestrator (sequential, periodic monitoring)")




    def _EnterInitializationModeFcn(self):
        if self._verbose: print("Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        # initial signal after startup
        self.RaPSignalStatus(component=adaptivityComponents.MONITOR,status=monitorStatus.NORMAL,accuracy=1.0)

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("Enter configurationModeFcn not implemented")

