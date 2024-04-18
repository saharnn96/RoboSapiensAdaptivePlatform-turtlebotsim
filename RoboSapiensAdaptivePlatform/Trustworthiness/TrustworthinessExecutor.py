#**********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from RoboSapiensAdaptivePlatform.utils.constants import *
from RoboSapiensAdaptivePlatform.utils.nodes import TriggeredNode

class TrustworthinessExecutor(TriggeredNode):

    def __init__(self,logger = None,knowledgeManagement = None,verbose=False):
        super().__init__(logger=logger, knowledge=knowledgeManagement, verbose=verbose)
        """Initialize the generic TrustworthinessExecutor component.

                Parameters
                ----------
                verbose : bool
                    component verbose execution

        See Also
        --------
        ..

        Examples
        --------
        >> cm = TrustworthinessExecutor(verbose=False)

        """

        self._name = "Trustworthiness Executor"
        self._description = "The trusworthiness executor handles the trustworthiness of the execution of the adaptations and/or diagnosis routines on the managed system, triggered from within the trustworthiness checker."
        self._verbose = verbose
        self._state = genericStates.STARTUP

    @property
    def name(self):
        """The name property (read-only)."""
        return self._name

    @property
    def description(self):
        """The description property (read-only)."""
        return self._description


    #------------------------------------------------------------------------------------------------
    # -------------------------------------INTERFACE FUNCTIONS---------------------------------------
    # ------------------------------------------------------------------------------------------------
    def checkTrustworthiness(self, action):
        self.logger.log("["+self._name+"] - "+"Checking trustworthiness of action "+action.description)

        # TODO: IMPLEMENT TRUSTWORTHINESS CHECK

        return True

    # ------------------------------------------------------------------------------------------------
    # -------------------------------------INTERNAL FUNCTIONS----------------------------------------
    # ------------------------------------------------------------------------------------------------
    def _EnterInitializationModeFcn(self):
        if self._verbose: print("Enter initializationModeFcn not implemented")

    def _ExitInitializationModeFcn(self):
        if self._verbose: print("Exit initializationModeFcn not implemented")

    def _EnterConfigurationModeFcn(self):
        if self._verbose: print("Enter configurationModeFcn not implemented")